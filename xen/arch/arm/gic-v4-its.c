/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * xen/arch/arm/gic-v4-its.c
 *
 * ARM Generic Interrupt Controller support v4 version
 * based on xen/arch/arm/gic-v3-its.c and kernel GICv4 driver
 *
 * Copyright (C) 2023 - ARM Ltd
 * Penny Zheng <penny.zheng@arm.com>, ARM Ltd ported to Xen
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <xen/errno.h>
#include <xen/sched.h>
#include <xen/spinlock.h>
#include <asm/gic_v3_defs.h>
#include <asm/gic_v3_its.h>
#include <asm/gic_v4_its.h>
#include <asm/vgic.h>


/*
 * VPE ID is at most 16 bits.
 * Using a bitmap here limits us to 65536 concurrent VPEs.
 */
#define INVALID_VPEID (~0U)

static unsigned long *vpeid_mask;

static spinlock_t vpeid_alloc_lock = SPIN_LOCK_UNLOCKED;

void __init gicv4_its_vpeid_allocator_init(void)
{
    /* Allocate space for vpeid_mask based on MAX_VPEID */
    vpeid_mask = xzalloc_array(unsigned long, BITS_TO_LONGS(MAX_VPEID));

    if ( !vpeid_mask )
        panic("Could not allocate VPEID bitmap space\n");
}

static int its_alloc_vpeid(void)
{
    int id;

    spin_lock(&vpeid_alloc_lock);

    id = find_first_zero_bit(vpeid_mask, MAX_VPEID);

    if ( id == MAX_VPEID )
    {
        id = -EBUSY;
        printk(XENLOG_ERR "VPEID pool exhausted\n");
        goto out;
    }

    set_bit(id, vpeid_mask);

 out:
    spin_unlock(&vpeid_alloc_lock);

    return id;
}

static void its_free_vpeid(uint32_t vpe_id)
{
    spin_lock(&vpeid_alloc_lock);

    clear_bit(vpe_id, vpeid_mask);

    spin_unlock(&vpeid_alloc_lock);
}

static int its_alloc_vpe_entry(uint32_t vpe_id)
{
    struct host_its *hw_its;
    int ret;

    /*
     * Make sure the L2 tables are allocated on *all* v4 ITSs. We
     * could try and only do it on ITSs corresponding to devices
     * that have interrupts targeted at this VPE, but the
     * complexity becomes crazy.
     */
    list_for_each_entry(hw_its, &host_its_list, entry)
    {
        struct its_baser *baser;

        if ( !hw_its->has_vlpis )
            continue;

        baser = its_get_baser(hw_its, GITS_BASER_TYPE_VCPU);
        if ( !baser )
            return -ENODEV;

        ret = its_alloc_table_entry(baser, vpe_id);
        if ( ret )
            return ret;
    }

    return 0;
}

static int its_send_cmd_vsync(struct host_its *its, uint16_t vpeid)
{
    uint64_t cmd[4];

    cmd[0] = GITS_CMD_VSYNC;
    cmd[1] = (uint64_t)vpeid << 32;
    cmd[2] = 0x00;
    cmd[3] = 0x00;

    return its_send_command(its, cmd);
}

static int its_send_cmd_vmapp(struct host_its *its, struct its_vpe *vpe,
                              bool valid)
{
    uint64_t cmd[4];
    uint16_t vpeid = vpe->vpe_id;
    uint64_t vpt_addr;
    int ret;

    cmd[0] = GITS_CMD_VMAPP;
    cmd[1] = (uint64_t)vpeid << 32;
    cmd[2] = valid ? GITS_VALID_BIT : 0;

    /* Unmap command */
    if ( !valid )
        goto out;

    /* Target redistributor */
    cmd[2] |= encode_rdbase(its, vpe->col_idx, 0x0);
    vpt_addr = virt_to_maddr(vpe->vpendtable);
    cmd[3] = (vpt_addr & GENMASK(51, 16)) |
             ((HOST_LPIS_NRBITS - 1) & GENMASK(4, 0));

 out:
    ret = its_send_command(its, cmd);

    return ret;
}

static int its_send_cmd_vinvall(struct host_its *its, struct its_vpe *vpe)
{
    uint64_t cmd[4];
    uint16_t vpeid = vpe->vpe_id;

    cmd[0] = GITS_CMD_VINVALL;
    cmd[1] = (uint64_t)vpeid << 32;
    cmd[2] = 0x00;
    cmd[3] = 0x00;

    return its_send_command(its, cmd);
}

static int its_unmap_vpe(struct host_its *its, struct its_vpe *vpe)
{
    int ret;

    ret = its_send_cmd_vmapp(its, vpe, false);
    if ( ret )
        return ret;

    ret = its_send_cmd_vsync(its, vpe->vpe_id);
    if ( ret )
        return ret;

    return gicv3_its_wait_commands(its);
}

static int its_map_vpe(struct host_its *its, struct its_vpe *vpe)
{
    int ret;

    /*
     * VMAPP command maps the vPE to the target RDbase, including an
     * associated virtual LPI Pending table.
     */
    ret = its_send_cmd_vmapp(its, vpe, true);
    if ( ret )
        return ret;

    ret = its_send_cmd_vinvall(its, vpe);
    if ( ret )
        goto rollback;

    ret = its_send_cmd_vsync(its, vpe->vpe_id);
    if ( ret )
        goto rollback;

    ret = gicv3_its_wait_commands(its);
    if ( ret )
        goto rollback;

    return 0;

 rollback:
    (void)its_unmap_vpe(its, vpe);

    return ret;
}

static void its_free_vpendtable(void *vpendtable)
{
    unsigned int order;

    if ( !vpendtable )
        return;

    order = get_order_from_bytes(max(lpi_max_host_lpis() / 8,
                                     (unsigned long)SZ_64K));

    free_xenheap_pages(vpendtable, order);
}

static int its_vpe_init(struct its_vpe *vpe)
{
    struct host_its *rollback_its;
    struct page_info *vpendpage;
    void *vpendtable = NULL;
    struct host_its *hw_its;
    int rc;

    vpe->vpe_id = INVALID_VPEID;

    /* Allocate vpe id */
    rc = its_alloc_vpeid();
    if ( rc < 0 )
        return rc;

    vpe->vpe_id = rc;

    /* Allocate VPT */
    vpendpage = lpi_allocate_pendtable();
    if ( !vpendpage )
    {
        rc = -ENOMEM;
        goto fail;
    }
    vpendtable = page_to_virt(vpendpage);

    rc = its_alloc_vpe_entry(vpe->vpe_id);
    if ( rc )
        goto fail;

    rwlock_init(&vpe->lock);
    spin_lock_init(&vpe->vpe_lock);
    vpe->vpendtable = vpendtable;
    /*
     * We eagerly inform all the v4 ITS and map vPE to the first
     * possible CPU
     */
    vpe->col_idx = cpumask_first(&cpu_online_map);
    list_for_each_entry(hw_its, &host_its_list, entry)
    {
        if ( !hw_its->has_vlpis )
            continue;

        rc = its_map_vpe(hw_its, vpe);
        if ( rc )
            goto fail_unmap;
    }

    return 0;

 fail_unmap:
    list_for_each_entry(rollback_its, &host_its_list, entry)
    {
        if ( rollback_its == hw_its )
            break;

        if ( rollback_its->has_vlpis )
            (void)its_unmap_vpe(rollback_its, vpe);
    }

    vpe->vpendtable = NULL;

 fail:
    its_free_vpeid(vpe->vpe_id);
    vpe->vpe_id = INVALID_VPEID;
    its_free_vpendtable(vpendtable);

    return rc;
}

static void its_vpe_teardown(struct its_vpe *vpe)
{
    struct host_its *hw_its;

    if ( !vpe )
        return;

    list_for_each_entry(hw_its, &host_its_list, entry)
    {
        if ( hw_its->has_vlpis )
            (void)its_unmap_vpe(hw_its, vpe);
    }

    if ( vpe->vpe_id != INVALID_VPEID )
        its_free_vpeid(vpe->vpe_id);

    its_free_vpendtable(vpe->vpendtable);
    xfree(vpe);
}

int vgic_v4_its_vm_init(struct domain *d)
{
    unsigned int nr_vcpus = d->max_vcpus;
    int ret = -ENOMEM;

    if ( !gicv4_supports_vlpis() )
        return 0;

    d->arch.vgic.its_vm = xzalloc(struct its_vm);
    if ( !d->arch.vgic.its_vm )
        return ret;

    d->arch.vgic.its_vm->vpes = xzalloc_array(struct its_vpe *, nr_vcpus);
    if ( !d->arch.vgic.its_vm->vpes )
        goto fail_vpes;
    d->arch.vgic.its_vm->nr_vpes = nr_vcpus;

    d->arch.vgic.its_vm->vproptable = lpi_allocate_vproptable();
    if ( !d->arch.vgic.its_vm->vproptable )
        goto fail_vprop;

    return 0;

 fail_vprop:
    xfree(d->arch.vgic.its_vm->vpes);
 fail_vpes:
    XFREE(d->arch.vgic.its_vm);

    return ret;
}

void vgic_v4_free_its_vm(struct domain *d)
{
    struct its_vm *its_vm = d->arch.vgic.its_vm;

    if ( !its_vm )
        return;

    xfree(its_vm->vpes);
    xfree(its_vm->db_lpi_bases);
    if ( its_vm->vproptable )
        lpi_free_vproptable(its_vm->vproptable);
    XFREE(d->arch.vgic.its_vm);
}

void vgic_v4_its_vpe_free(struct vcpu *vcpu)
{
    struct its_vm *its_vm = vcpu->domain->arch.vgic.its_vm;
    struct its_vpe *vpe = vcpu->arch.vgic.its_vpe;

    if ( !vpe )
        return;

    if ( its_vm && vcpu->vcpu_id < its_vm->nr_vpes )
        its_vm->vpes[vcpu->vcpu_id] = NULL;

    its_vpe_teardown(vpe);
    vcpu->arch.vgic.its_vpe = NULL;
}

int vgic_v4_its_vpe_init(struct vcpu *vcpu)
{
    int ret;
    struct its_vm *its_vm = vcpu->domain->arch.vgic.its_vm;
    struct its_vpe *vpe;
    unsigned int vcpuid = vcpu->vcpu_id;

    vpe = xzalloc(struct its_vpe);
    if ( !vpe )
        return -ENOMEM;

    vpe->its_vm = its_vm;

    ret = its_vpe_init(vpe);
    if ( ret )
    {
        xfree(vpe);
        return ret;
    }

    vcpu->arch.vgic.its_vpe = vpe;
    its_vm->vpes[vcpuid] = vpe;

    return 0;
}

static int its_send_cmd_vmapti(struct host_its *its, struct its_device *dev,
                               const struct its_vlpi_map *map)
{
    uint64_t cmd[4];
    struct its_vpe *vpe = map->vm->vpes[map->vpe_idx];
    uint32_t deviceid = dev->host_devid;
    uint16_t vpeid = vpe->vpe_id;
    uint32_t vintid = map->vintid;
    uint32_t db_pintid;

    if ( map->db_enabled )
        db_pintid = vpe->vpe_db_lpi;
    else
        db_pintid = INVALID_DBLPI;

    cmd[0] = GITS_CMD_VMAPTI | ((uint64_t)deviceid << 32);
    cmd[1] = map->eventid | ((uint64_t)vpeid << 32);
    cmd[2] = vintid | ((uint64_t)db_pintid << 32);
    cmd[3] = 0x00;

    return its_send_command(its, cmd);
}

static bool pirq_is_forwarded_to_vcpu(struct pending_irq *pirq)
{
    ASSERT(pirq);
    return test_bit(GIC_IRQ_GUEST_FORWARDED, &pirq->status);
}

bool event_is_forwarded_to_vcpu(struct its_device *dev, uint32_t eventid)
{
    struct pending_irq *pirq;

    /* No vlpi maps at all ? */
    if ( !dev->event_map.vlpi_maps || eventid >= dev->event_map.nr_lpis )
        return false;

    pirq = dev->event_map.vlpi_maps[eventid].pirq;
    if ( !pirq )
        return false;

    return pirq_is_forwarded_to_vcpu(pirq);
}

static int its_send_cmd_vmovi(struct host_its *its,
                              const struct its_vlpi_map *map)
{
    uint64_t cmd[4];
    struct its_device *dev = map->dev;
    struct its_vpe *vpe = map->vm->vpes[map->vpe_idx];
    uint32_t eventid = map->eventid;
    uint32_t deviceid = dev->host_devid;
    uint16_t vpeid = vpe->vpe_id;
    uint32_t db_pintid;

    if ( map->db_enabled )
        db_pintid = vpe->vpe_db_lpi;
    else
        db_pintid = INVALID_IRQ;

    cmd[0] = GITS_CMD_VMOVI | ((uint64_t)deviceid << 32);
    cmd[1] = eventid | ((uint64_t)vpeid << 32);
    cmd[2] = (map->db_enabled ? 1UL : 0UL) | ((uint64_t)db_pintid << 32);
    cmd[3] = 0x00;

    return its_send_command(its, cmd);
}

static uint32_t its_event_host_lpi(const struct its_device *dev,
                                   uint32_t eventid)
{
    return dev->host_lpi_blocks[eventid / LPI_BLOCK] +
           (eventid % LPI_BLOCK);
}

static int its_restore_host_mapping(struct its_device *dev, uint32_t eventid)
{
    struct host_its *its = dev->hw_its;
    uint32_t host_lpi = its_event_host_lpi(dev, eventid);
    int ret;

    ret = its_send_cmd_mapti(its, dev->host_devid, eventid, host_lpi, 0);
    if ( ret )
        return ret;

    return its_inv_lpi(its, dev, eventid, 0);
}

static int gicv4_its_vlpi_prepare_locked(struct its_device *dev,
                                         const struct its_vlpi_map *map,
                                         struct its_vlpi_map **vlpi_maps,
                                         bool *new_map_array)
{
    ASSERT(spin_is_locked(&dev->event_map.vlpi_lock));

    if ( !dev->event_map.vm )
    {
        *vlpi_maps = xzalloc_array(struct its_vlpi_map,
                                   dev->event_map.nr_lpis);
        if ( !*vlpi_maps )
            return -ENOMEM;

        *new_map_array = true;
        return 0;
    }

    if ( dev->event_map.vm != map->vm )
        return -EINVAL;

    *vlpi_maps = dev->event_map.vlpi_maps;
    return 0;
}

static int gicv4_its_vlpi_move_locked(struct its_device *dev,
                                      const struct its_vlpi_map *map)
{
    struct its_vlpi_map *old_map;
    struct its_vpe *old_vpe;
    int ret;

    ASSERT(spin_is_locked(&dev->event_map.vlpi_lock));

    if ( !dev->event_map.vlpi_maps )
        return -EINVAL;

    old_map = &dev->event_map.vlpi_maps[map->eventid];

    old_vpe = old_map->vm->vpes[old_map->vpe_idx];

    ret = its_send_cmd_vmovi(dev->hw_its, map);
    if ( ret )
        return ret;

    *old_map = *map;

    /*
     * ARM spec says that if software moves the same interrupt again after a
     * VMOVI, a VSYNC must be issued to the old vPE between the moves.
     */
    return its_send_cmd_vsync(dev->hw_its, old_vpe->vpe_id);
}

static int gicv4_its_vlpi_first_map_locked(struct its_device *dev,
                                           const struct its_vlpi_map *map,
                                           struct its_vlpi_map *vlpi_maps,
                                           bool new_map_array)
{
    int ret;

    ASSERT(spin_is_locked(&dev->event_map.vlpi_lock));

    ret = its_send_cmd_discard(dev->hw_its, dev, map->eventid);
    if ( ret )
        return ret;

    ret = its_send_cmd_vmapti(dev->hw_its, dev, map);
    if ( ret )
    {
        int restore_ret;

        restore_ret = its_restore_host_mapping(dev, map->eventid);
        if ( restore_ret )
        {
            printk(XENLOG_WARNING
                   "ITS: failed to restore host mapping after VMAPTI failure: vmapti=%d restore=%d\n",
                   ret, restore_ret);
            return restore_ret;
        }

        return ret;
    }

    if ( new_map_array )
    {
        dev->event_map.vm = map->vm;
        dev->event_map.vlpi_maps = vlpi_maps;
    }

    dev->event_map.vlpi_maps[map->eventid] = *map;
    dev->event_map.nr_vlpis++;

    return 0;
}

static int gicv4_its_vlpi_map(struct its_vlpi_map *map)
{
    struct its_device *dev;
    struct its_vlpi_map *vlpi_maps = NULL;
    int ret;
    bool forwarded;
    bool new_map_array = false;

    if ( !map )
        return -EINVAL;

    dev = map->dev;

    spin_lock(&dev->event_map.vlpi_lock);

    ret = gicv4_its_vlpi_prepare_locked(dev, map, &vlpi_maps,
                                        &new_map_array);
    if ( ret )
        goto out;

    forwarded = pirq_is_forwarded_to_vcpu(map->pirq);
    ASSERT(!forwarded || dev->event_map.vlpi_maps != NULL);

    if ( forwarded )
        ret = gicv4_its_vlpi_move_locked(dev, map);
    else
        ret = gicv4_its_vlpi_first_map_locked(dev, map, vlpi_maps,
                                              new_map_array);

    if ( ret && new_map_array )
        xfree(vlpi_maps);

out:
    spin_unlock(&dev->event_map.vlpi_lock);
    return ret;
}

int gicv4_its_vlpi_unmap(struct pending_irq *pirq)
{
    struct its_vlpi_map *map = pirq->vlpi_map;
    struct its_device *dev;
    int ret;
    uint32_t host_lpi;

    if ( !map )
        return -EINVAL;

    dev = map->dev;

    spin_lock(&dev->event_map.vlpi_lock);

    if ( !dev->event_map.vm || !pirq_is_tied_to_hw(pirq) )
    {
        ret = -EINVAL;
        goto out;
    }

    /* Drop the virtual mapping */
    ret = its_send_cmd_discard(dev->hw_its, dev, map->eventid);
    if ( ret )
        goto out;

    /* Restore the physical one */
    host_lpi = its_event_host_lpi(dev, map->eventid);
    /* Map every host LPI to host CPU 0 */
    ret = its_send_cmd_mapti(dev->hw_its, dev->host_devid, map->eventid,
                             host_lpi, 0);
    if ( ret )
        goto out;

    ret = lpi_write_config(lpi_host_proptable(), host_lpi, 0,
                           LPI_PROP_ENABLED);
    if ( ret )
        goto out;

    clear_bit(GIC_IRQ_GUEST_FORWARDED, &pirq->status);
    ret = its_inv_lpi(dev->hw_its, dev, map->eventid, 0);
    if ( ret )
        goto out;

    dev->event_map.vlpi_maps[map->eventid] = (struct its_vlpi_map){};
    XFREE(pirq->vlpi_map);
    /*
     * Drop the refcount and make the device available again if
     * this was the last VLPI.
     */
    if ( !--dev->event_map.nr_vlpis )
    {
        dev->event_map.vm = NULL;
        XFREE(dev->event_map.vlpi_maps);
    }

 out:
    spin_unlock(&dev->event_map.vlpi_lock);
    return ret;
}

int gicv4_assign_guest_event(struct domain *d, paddr_t vdoorbell_address,
                             uint32_t vdevid, uint32_t eventid,
                             struct pending_irq *pirq)

{
    int ret = -ENODEV;
    struct its_vm *vm = d->arch.vgic.its_vm;
    struct its_vlpi_map *map;
    struct its_device *dev;

    if ( !is_lpi(pirq->irq) )
        return -EINVAL;

    if ( pirq->irq >= lpi_max_host_lpis() )
        return -ENODEV;

    spin_lock(&d->arch.vgic.its_devices_lock);
    dev = get_its_device(d, vdoorbell_address, vdevid);
    if ( dev && dev->hw_its->has_vlpis && eventid < dev->eventids )
    {
        /* Prepare the vlpi mapping info */
        map = xzalloc(struct its_vlpi_map);
        if ( !map )
        {
            ret = -ENOMEM;
            goto out;
        }
        map->vm = vm;
        map->vintid = pirq->irq;
        map->db_enabled = true;
        map->vpe_idx = pirq->lpi_vcpu_id;
        map->pirq = pirq;
        map->dev = dev;
        map->eventid = eventid;

        ret = gicv4_its_vlpi_map(map);
        if ( ret )
        {
            xfree(map);
            goto out;
        }

        pirq->vlpi_map = map;
    }

 out:
    spin_unlock(&d->arch.vgic.its_devices_lock);
    return ret;
}

int gicv4_its_vlpi_move(struct pending_irq *pirq, struct vcpu *vcpu)
{
    struct its_vlpi_map *map = pirq->vlpi_map;
    struct its_device *dev;

    if ( !map )
        return -EINVAL;

    dev = map->dev;

    if ( !dev->event_map.vm )
        return -EINVAL;

    map->vpe_idx = vcpu->vcpu_id;
    return gicv4_its_vlpi_map(map);
}

/*
 * There is no real VINV command.
 * We do a normal INV, with a VSYNC instead of a SYNC.
 */
int its_send_cmd_vinv(struct host_its *its, struct its_device *dev,
                      uint32_t eventid)
{
    int ret;
    struct its_vlpi_map *map = &dev->event_map.vlpi_maps[eventid];
    struct its_vpe *vpe = map->vm->vpes[map->vpe_idx];
    uint16_t vpeid = vpe->vpe_id;

    ret = its_send_cmd_inv(its, dev->host_devid, eventid);
    if ( ret )
        return ret;

    ret = its_send_cmd_vsync(its, vpeid);
    if ( ret )
        return ret;

    return gicv3_its_wait_commands(its);
}
