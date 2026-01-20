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
static unsigned long *vpeid_mask;

static spinlock_t vpeid_alloc_lock = SPIN_LOCK_UNLOCKED;

void __init gicv4_its_vpeid_allocator_init(void)
{
    /* Allocate space for vpeid_mask based on MAX_VPEID */
    vpeid_mask = xzalloc_array(unsigned long, BITS_TO_LONGS(MAX_VPEID));

    if ( !vpeid_mask )
        panic("Could not allocate VPEID bitmap space\n");
}

static int __init its_alloc_vpeid(struct its_vpe *vpe)
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

static void __init its_free_vpeid(uint32_t vpe_id)
{
    spin_lock(&vpeid_alloc_lock);

    clear_bit(vpe_id, vpeid_mask);

    spin_unlock(&vpeid_alloc_lock);
}

static bool __init its_alloc_vpe_entry(uint32_t vpe_id)
{
    struct host_its *hw_its;

    /*
     * Make sure the L2 tables are allocated on *all* v4 ITSs. We
     * could try and only do it on ITSs corresponding to devices
     * that have interrupts targeted at this VPE, but the
     * complexity becomes crazy.
     */
    list_for_each_entry(hw_its, &host_its_list, entry)
    {
        struct its_baser *baser;

        if ( !hw_its->is_v4 )
            continue;

        baser = its_get_baser(hw_its, GITS_BASER_TYPE_VCPU);
        if ( !baser )
            return false;

        if ( !its_alloc_table_entry(baser, vpe_id) )
            return false;
    }

    return true;
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
        return ret;

    ret = its_send_cmd_vsync(its, vpe->vpe_id);
    if ( ret )
        return ret;

    return 0;
}
static int __init its_vpe_init(struct its_vpe *vpe)
{
    int vpe_id, rc = -ENOMEM;
    struct page_info *vpendtable;
    struct host_its *hw_its;

    /* Allocate vpe id */
    vpe_id = its_alloc_vpeid(vpe);
    if ( vpe_id < 0 )
        return rc;

    /* Allocate VPT */
    vpendtable = lpi_allocate_pendtable();

    if ( !vpendtable )
        goto fail_vpt;

    if ( !its_alloc_vpe_entry(vpe_id) )
        goto fail_entry;

    rwlock_init(&vpe->lock);
    vpe->vpe_id = vpe_id;
    vpe->vpendtable = page_to_virt(vpendtable);
    /*
     * We eagerly inform all the v4 ITS and map vPE to the first
     * possible CPU
     */
    vpe->col_idx = cpumask_first(&cpu_online_map);
    list_for_each_entry(hw_its, &host_its_list, entry)
    {
        if ( !hw_its->is_v4 )
            continue;

        if ( its_map_vpe(hw_its, vpe) )
            goto fail_entry;
    }

    return 0;

 fail_entry:
    xfree(page_to_virt(vpendtable));
 fail_vpt:
    its_free_vpeid(vpe_id);

    return rc;
}

static void __init its_vpe_teardown(struct its_vpe *vpe)
{
    unsigned int order;

    order = get_order_from_bytes(max(lpi_data.max_host_lpi_ids / 8, (unsigned long)SZ_64K));
    its_free_vpeid(vpe->vpe_id);
    free_xenheap_pages(vpe->vpendtable, order);
    xfree(vpe);
}

int vgic_v4_its_vm_init(struct domain *d)
{
    unsigned int nr_vcpus = d->max_vcpus;
    int ret = -ENOMEM;

    if ( !gicv3_its_host_has_its() )
        return 0;

    d->arch.vgic.its_vm = xzalloc(struct its_vm);
    if ( !d->arch.vgic.its_vm )
        return ret;

    d->arch.vgic.its_vm->vpes = xzalloc_array(struct its_vpe *, nr_vcpus);
    if ( !d->arch.vgic.its_vm->vpes )
        goto fail_vpes;
    d->arch.vgic.its_vm->nr_vpes = nr_vcpus;

    d->arch.vgic.its_vm->vproptable = lpi_allocate_proptable();
    if ( !d->arch.vgic.its_vm->vproptable )
        goto fail_vprop;

    return 0;

fail_vprop:
    xfree(d->arch.vgic.its_vm->vpes);
 fail_vpes:
    xfree(d->arch.vgic.its_vm);

    return ret;
}

void vgic_v4_free_its_vm(struct domain *d)
{
    struct its_vm *its_vm = d->arch.vgic.its_vm;
    if ( its_vm->vpes )
        xfree(its_vm->vpes);
    if ( its_vm->vproptable )
        lpi_free_proptable(its_vm);
}

int vgic_v4_its_vpe_init(struct vcpu *vcpu)
{
    int ret;
    struct its_vm *its_vm = vcpu->domain->arch.vgic.its_vm;
    unsigned int vcpuid = vcpu->vcpu_id;

    vcpu->arch.vgic.its_vpe = xzalloc(struct its_vpe);
    if ( !vcpu->arch.vgic.its_vpe )
        return -ENOMEM;

    its_vm->vpes[vcpuid] = vcpu->arch.vgic.its_vpe;
    vcpu->arch.vgic.its_vpe->its_vm = its_vm;

    ret = its_vpe_init(vcpu->arch.vgic.its_vpe);
    if ( ret )
    {
        its_vpe_teardown(vcpu->arch.vgic.its_vpe);
        return ret;
    }
    return 0;
}

static int its_send_cmd_vmapti(struct host_its *its, struct its_device *dev,
                               uint32_t eventid)
{
    uint64_t cmd[4];
    uint32_t deviceid = dev->host_devid;
    struct its_vlpi_map *map = &dev->event_map.vlpi_maps[eventid];
    uint16_t vpeid = map->vm->vpes[map->vpe_idx]->vpe_id;
    uint32_t vintid = map->vintid;
    uint32_t db_pintid;

    if ( map->db_enabled )
        db_pintid = map->vm->vpes[map->vpe_idx]->vpe_db_lpi;
    else
        db_pintid = 1023;

    cmd[0] = GITS_CMD_VMAPTI | ((uint64_t)deviceid << 32);
    cmd[1] = eventid | ((uint64_t)vpeid << 32);
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
    if ( !dev->event_map.vlpi_maps)
        return false;

    pirq = dev->event_map.vlpi_maps[eventid].pirq;
    return pirq_is_forwarded_to_vcpu(pirq);
}

static int its_send_cmd_vmovi(struct host_its *its, struct its_vlpi_map *map)
{
    uint64_t cmd[4];
    struct its_device *dev = map->dev;
    uint32_t eventid = map->eventid;
    uint32_t deviceid = dev->host_devid;
    uint16_t vpeid = map->vm->vpes[map->vpe_idx]->vpe_id;
    uint32_t db_pintid;

    if ( map->db_enabled )
        db_pintid = map->vm->vpes[map->vpe_idx]->vpe_db_lpi;
    else
        db_pintid = 1023;

    cmd[0] = GITS_CMD_VMOVI | ((uint64_t)deviceid << 32);
    cmd[1] = eventid | ((uint64_t)vpeid << 32);
    cmd[2] = (map->db_enabled ? 1UL : 0UL) | ((uint64_t)db_pintid << 32);
    cmd[3] = 0x00;

    return its_send_command(its, cmd);
}

static int gicv4_its_vlpi_map(struct its_vlpi_map *map)
{
    struct its_device *dev;
    struct host_its *its;
    uint32_t eventid;
    int ret;

    if ( !map )
        return -EINVAL;
    dev = map->dev;
    its = map->dev->hw_its;
    eventid = map->eventid;

    spin_lock(&dev->event_map.vlpi_lock);

    if ( !dev->event_map.vm )
    {
        struct its_vlpi_map *maps;

        maps = xzalloc_array(struct its_vlpi_map, dev->event_map.nr_lpis);
        if ( !maps )
        {
            ret = -ENOMEM;
            goto out;
        }

        dev->event_map.vm = map->vm;
        dev->event_map.vlpi_maps = maps;
    }
    else if ( dev->event_map.vm != map->vm )
    {
        ret = -EINVAL;
        goto out;
    }

    /* Get our private copy of the mapping information */
    dev->event_map.vlpi_maps[eventid] = *map;

    if ( pirq_is_forwarded_to_vcpu(map->pirq) )
    {
        struct its_vlpi_map *old = &dev->event_map.vlpi_maps[eventid];
        uint32_t old_vpeid = old->vm->vpes[old->vpe_idx]->vpe_id;

        /* Already mapped, move it around */
        ret = its_send_cmd_vmovi(dev->hw_its, map);
        if ( ret )
            goto out;

        /*
         * ARM spec says that If, after using VMOVI to move an interrupt from
         * vPE A to vPE B, software moves the same interrupt again, a VSYNC
         * command must be issued to vPE A between the moves to ensure correct
         * behavior.
         * So each time we issue VMOVI, we VSYNC the old VPE for good measure.
         */
        ret = its_send_cmd_vsync(dev->hw_its, old_vpeid);
    }
    else
    {
        /* Drop the original physical mapping firstly */
        ret = its_send_cmd_discard(its, dev, eventid);
        if ( ret )
            goto out;

        /* Then install the virtual one */
        ret = its_send_cmd_vmapti(its, dev, eventid);
        if ( ret )
            goto out;

        /* Increment the number of VLPIs */
        dev->event_map.nr_vlpis++;
    }

 out:
    spin_unlock(&dev->event_map.vlpi_lock);
    return ret;
}
int gicv4_its_vlpi_unmap(struct pending_irq *pirq)
{
    struct its_vlpi_map *map = pirq->vlpi_map;
    struct its_device *dev = map->dev;
    int ret;
    uint32_t host_lpi;

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
    clear_bit(GIC_IRQ_GUEST_FORWARDED, &pirq->status);
    host_lpi = dev->host_lpi_blocks[map->eventid / LPI_BLOCK] +
               (map->eventid % LPI_BLOCK);
    /* Map every host LPI to host CPU 0 */
    ret = its_send_cmd_mapti(dev->hw_its, dev->host_devid, map->eventid,
                             host_lpi, 0);
    if ( ret )
        goto out;

    lpi_write_config(lpi_data.lpi_property, host_lpi, 0xff, LPI_PROP_ENABLED);

    ret = its_inv_lpi(dev->hw_its, dev, map->eventid, 0);
    if ( ret )
        goto out;

    xfree(map);
    /*
     * Drop the refcount and make the device available again if
     * this was the last VLPI.
     */
    if ( !--dev->event_map.nr_vlpis )
    {
        dev->event_map.vm = NULL;
        xfree(dev->event_map.vlpi_maps);
    }

out:
    spin_unlock(&dev->event_map.vlpi_lock);
    return ret;
}

int gicv4_assign_guest_event(struct domain *d, paddr_t vdoorbell_address,
                             uint32_t vdevid, uint32_t eventid,
                             struct pending_irq *pirq)

{
    int ret = ENODEV;
    struct its_vm *vm = d->arch.vgic.its_vm;
    struct its_vlpi_map *map;
    struct its_device *dev;

    spin_lock(&d->arch.vgic.its_devices_lock);
    dev = get_its_device(d, vdoorbell_address, vdevid);
    if ( dev && eventid < dev->eventids )
    {
        /* Prepare the vlpi mapping info */
        map = xzalloc(struct its_vlpi_map);
        if ( !map )
            goto out;
        map->vm = vm;
        map->vintid = pirq->irq;
        map->db_enabled = true;
        map->vpe_idx = pirq->lpi_vcpu_id;
        map->properties = pirq->lpi_priority |
                          (test_bit(GIC_IRQ_GUEST_ENABLED, &pirq->status) ?
                          LPI_PROP_ENABLED : 0);
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
        goto out;
    }

 out:
    spin_unlock(&d->arch.vgic.its_devices_lock);
    return ret;
}

int gicv4_its_vlpi_move(struct pending_irq *pirq, struct vcpu *vcpu)
{
    struct its_vlpi_map *map = pirq->vlpi_map;
    struct its_device *dev = map->dev;

    if ( !dev->event_map.vm || !map )
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
    uint16_t vpeid = map->vm->vpes[map->vpe_idx]->vpe_id;

    ret = its_send_cmd_inv(its, dev->host_devid, eventid);
    if ( ret )
        return ret;

    ret = its_send_cmd_vsync(its, vpeid);
    if ( ret )
        return ret;

    return gicv3_its_wait_commands(its);
}
