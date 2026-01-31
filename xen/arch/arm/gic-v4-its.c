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

#include <xen/delay.h>
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

static uint16_t vmovp_seq_num;
static spinlock_t vmovp_lock = SPIN_LOCK_UNLOCKED;

void __init gicv4_its_vpeid_allocator_init(void)
{
    /* Allocate space for vpeid_mask based on MAX_VPEID */
    vpeid_mask = xzalloc_array(unsigned long, BITS_TO_LONGS(MAX_VPEID));

    if ( !vpeid_mask )
        panic("Could not allocate VPEID bitmap space\n");
}

static void __iomem *gic_data_rdist_vlpi_base(unsigned int cpu)
{
    /*
     * Each Redistributor defines two 64KB frames in the physical address map.
     * In GICv4, there are two additional 64KB frames.
     * The frames for each Redistributor must be contiguous and must be
     * ordered as follows:
     * 1. RD_base
     * 2. SGI_base
     * 3. VLPI_base
     * 4. Reserved
     */
    return GICD_RDIST_BASE_CPU(cpu) + SZ_128K;
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

    /* Default doorbell interrupt */
    cmd[1] |= (uint64_t)vpe->vpe_db_lpi;

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

static int its_send_cmd_vmovp(struct its_vpe *vpe)
{
    uint16_t vpeid = vpe->vpe_id;
    int ret;
    struct host_its *hw_its;

    if ( !its_list_map )
    {
        uint64_t cmd[4];

        hw_its = list_first_entry(&host_its_list, struct host_its, entry);
        cmd[0] = GITS_CMD_VMOVP;
        cmd[1] = (uint64_t)vpeid << 32;
        cmd[2] = encode_rdbase(hw_its, vpe->col_idx, 0x0);
        cmd[3] = 0x00;

        return its_send_command(hw_its, cmd);
    }

    /*
     * If using the its_list "feature", we need to make sure that all ITSs
     * receive all VMOVP commands in the same order. The only way
     * to guarantee this is to make vmovp a serialization point.
     */
    spin_lock(&vmovp_lock);

    vmovp_seq_num++;

    /* Emit VMOVPs */
    list_for_each_entry(hw_its, &host_its_list, entry)
    {
        uint64_t cmd[4];

        cmd[0] = GITS_CMD_VMOVP | ((uint64_t)vmovp_seq_num << 32);
        cmd[1] = its_list_map | ((uint64_t)vpeid << 32);
        cmd[2] = encode_rdbase(hw_its, vpe->col_idx, 0x0);
        cmd[3] = 0x00;

        ret = its_send_command(hw_its, cmd);
        if ( ret )
        {
            spin_unlock(&vmovp_lock);
            return ret;
        }
    }

    spin_unlock(&vmovp_lock);

    return 0;
}


static void its_vpe_send_inv_db(struct its_vpe *vpe)
{
    // struct its_device *dev = vpe_proxy.dev;
    // unsigned long flags;

    // spin_lock_irqsave(&vpe_proxy.lock, flags);
    // gicv4_vpe_db_proxy_map_locked(vpe);
    // its_send_cmd_inv(dev->hw_its, dev->host_devid, vpe->vpe_proxy_event);
    // spin_unlock_irqrestore(&vpe_proxy.lock, flags);
}

static void its_vpe_inv_db(struct its_vpe *vpe)
{
    its_vpe_send_inv_db(vpe);
}

void its_vpe_mask_db(struct its_vpe *vpe)
{
    /* Only clear enable bit. */
    lpi_write_config(lpi_data.lpi_property, vpe->vpe_db_lpi, LPI_PROP_ENABLED, 0);
    its_vpe_inv_db(vpe);
}

static void its_vpe_unmask_db(struct its_vpe *vpe)
{
    /* Only set enable bit. */
    lpi_write_config(lpi_data.lpi_property, vpe->vpe_db_lpi, 0, LPI_PROP_ENABLED);
    its_vpe_inv_db(vpe);
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
    unsigned int nr_db_lpis, nr_chunks, i = 0;
    uint32_t *db_lpi_bases;
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
    /* Allocate a doorbell interrupt for each VPE. */
    nr_db_lpis = d->arch.vgic.its_vm->nr_vpes;
    nr_chunks = DIV_ROUND_UP(nr_db_lpis, LPI_BLOCK);
    db_lpi_bases = xzalloc_array(uint32_t, nr_chunks);
    if ( !db_lpi_bases )
        goto fail_db_bases;

    do {
        /* Allocate doorbell interrupts in chunks of LPI_BLOCK (=32). */
        ret = gicv3_allocate_host_lpi_block(d, &db_lpi_bases[i]);
        if ( ret )
            goto fail_db;
    } while ( ++i < nr_chunks );

    d->arch.vgic.its_vm->db_lpi_bases = db_lpi_bases;
    d->arch.vgic.its_vm->nr_db_lpis = nr_db_lpis;

    return 0;

fail_db:
    while ( --i >= 0 )
        gicv3_free_host_lpi_block(d->arch.vgic.its_vm->db_lpi_bases[i]);
    xfree(db_lpi_bases);
fail_db_bases:
    lpi_free_proptable(d->arch.vgic.its_vm->vproptable);
fail_vprop:
    xfree(d->arch.vgic.its_vm->vpes);
 fail_vpes:
    xfree(d->arch.vgic.its_vm);

    return ret;
}

void vgic_v4_free_its_vm(struct domain *d)
{
    struct its_vm *its_vm = d->arch.vgic.its_vm;
    int nr_chunks = DIV_ROUND_UP(its_vm->nr_db_lpis, LPI_BLOCK);
    if ( its_vm->vpes )
        xfree(its_vm->vpes);
    while ( --nr_chunks >= 0 )
        gicv3_free_host_lpi_block(its_vm->db_lpi_bases[nr_chunks]);
    if ( its_vm->db_lpi_bases )
        xfree(its_vm->db_lpi_bases);
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
    vcpu->arch.vgic.its_vpe = vcpu->arch.vgic.its_vpe;
    vcpu->arch.vgic.its_vpe->vpe_db_lpi = its_vm->db_lpi_bases[vcpuid/32] + (vcpuid % 32);
    /*
     * Sometimes vlpi gets firstly mapped before associated vpe
     * becoming resident, so in case missing the interrupt, we intend to
     * enable doorbell at the initialization stage
     */

    vcpu->arch.vgic.its_vpe->its_vm = its_vm;

    gicv3_lpi_update_host_entry(vcpu->arch.vgic.its_vpe->vpe_db_lpi,
                                vcpu->domain->domain_id, INVALID_LPI, true,
                                vcpu->vcpu_id);


    ret = its_vpe_init(vcpu->arch.vgic.its_vpe);
    if ( ret )
    {
        its_vpe_teardown(vcpu->arch.vgic.its_vpe);
        return ret;
    }
    its_vpe_unmask_db(vcpu->arch.vgic.its_vpe);

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

static uint64_t read_vpend_dirty_clean(void __iomem *vlpi_base,
                                       unsigned int count)
{
    uint64_t val;
    bool clean;

    do {
        val = gits_read_vpendbaser(vlpi_base + GICR_VPENDBASER);
        /* Poll GICR_VPENDBASER.Dirty until it reads 0. */
        clean = !(val & GICR_VPENDBASER_Dirty);
        if ( !clean )
        {
            count--;
            cpu_relax();
            udelay(1);
        }
    } while ( !clean && count );

    if ( !clean )
    {
        printk(XENLOG_WARNING "ITS virtual pending table not totally parsed\n");
        val |= GICR_VPENDBASER_PendingLast;
    }

    return val;
}

/*
 * When a vPE is made resident, the GIC starts parsing the virtual pending
 * table to deliver pending interrupts. This takes place asynchronously,
 * and can at times take a long while.
 */
static void its_wait_vpt_parse_complete(void __iomem *vlpi_base)
{
    if ( !gic_support_vptValidDirty() )
        return;

    read_vpend_dirty_clean(vlpi_base, 500);
}

static uint64_t its_clear_vpend_valid(void __iomem *vlpi_base, uint64_t clr,
                                      uint64_t set)
{
    unsigned int count = 1000000;    /* 1s! */
    uint64_t val;

    /*
     * Clearing the Valid bit informs the Redistributor that a context
     * switch is taking place.
     */
    val = gits_read_vpendbaser(vlpi_base + GICR_VPENDBASER);
    val &= ~GICR_VPENDBASER_Valid;
    val &= ~clr;
    val |= set;
    gits_write_vpendbaser(val, vlpi_base + GICR_VPENDBASER);

    return read_vpend_dirty_clean(vlpi_base, count);
}

static void its_make_vpe_resident(struct its_vpe *vpe, unsigned int cpu)
{
    void __iomem *vlpi_base = gic_data_rdist_vlpi_base(cpu);
    uint64_t val;

    /* Switch in this VM's virtual property table. */
    val  = virt_to_maddr(vpe->its_vm->vproptable) & GENMASK(51, 12);
    val |= gicv3_its_get_cacheability() << GICR_VPROPBASER_INNER_CACHEABILITY_SHIFT;
    val |= gicv3_its_get_shareability() << GICR_VPROPBASER_SHAREABILITY_SHIFT;
    val |= GIC_BASER_CACHE_SameAsInner << GICR_VPROPBASER_OUTER_CACHEABILITY_SHIFT;
    val |= (HOST_LPIS_NRBITS - 1) & GICR_VPROPBASER_IDBITS_MASK;
    gits_write_vpropbaser(val, vlpi_base + GICR_VPROPBASER);

    /* Switch in this VCPU's VPT. */
    val  = virt_to_maddr(vpe->vpendtable) & GENMASK(51, 16);
    val |= gicv3_its_get_cacheability() << GICR_VPENDBASER_INNER_CACHEABILITY_SHIFT;
    val |= gicv3_its_get_shareability() << GICR_VPENDBASER_SHAREABILITY_SHIFT;
    val |= GIC_BASER_CACHE_SameAsInner << GICR_VPENDBASER_OUTER_CACHEABILITY_SHIFT;
    /*
     * When the GICR_VPENDBASER.Valid bit is written from 0 to 1,
     * this bit is RES1.
     */
    val |= GICR_VPENDBASER_PendingLast;
    val |= vpe->idai ? GICR_VPENDBASER_IDAI : 0;
    val |= GICR_VPENDBASER_Valid;
    gits_write_vpendbaser(val, vlpi_base + GICR_VPENDBASER);

    its_wait_vpt_parse_complete(vlpi_base);
}

static void its_make_vpe_non_resident(struct its_vpe *vpe, unsigned int cpu)
{
    void __iomem *vlpi_base = gic_data_rdist_vlpi_base(cpu);
    uint64_t val;

    val = its_clear_vpend_valid(vlpi_base, 0, 0);
    vpe->idai = val & GICR_VPENDBASER_IDAI;
    vpe->pending_last = val & GICR_VPENDBASER_PendingLast;
}

static int vpe_to_cpuid_lock(struct its_vpe *vpe, unsigned long *flags)
{
    spin_lock_irqsave(&vpe->vpe_lock, *flags);
    return vpe->col_idx;
}

static void vpe_to_cpuid_unlock(struct its_vpe *vpe, unsigned long *flags)
{
    spin_unlock_irqrestore(&vpe->vpe_lock, *flags);
}

static int gicv4_vpe_set_affinity(struct vcpu *vcpu)
{
    struct its_vpe *vpe = vcpu->arch.vgic.its_vpe;
    unsigned int from, to = vcpu->processor;
    unsigned long flags;
    int ret = 0;

    /*
     * Changing affinity is mega expensive, so let's be as lazy as
     * we can and only do it if we really have to. Also, if mapped
     * into the proxy device, we need to move the doorbell interrupt
     * to its new location.
     *
     * Another thing is that changing the affinity of a vPE affects
     * *other interrupts* such as all the vLPIs that are routed to
     * this vPE. This means that we must ensure nobody samples
     * vpe->col_idx during the update, hence the lock below which
     * must also be taken on any vLPI handling path that evaluates
     * vpe->col_idx, such as reg-based vLPI invalidation.
     */
    from = vpe_to_cpuid_lock(vpe, &flags);
    if ( from == to )
        goto out;

    vpe->col_idx = to;

    ret = its_send_cmd_vmovp(vpe);
    if ( ret )
        goto out;

 out:
    vpe_to_cpuid_unlock(vpe, &flags);
    return ret;
}

void vgic_v4_load(struct vcpu *vcpu)
{
    struct its_vpe *vpe = vcpu->arch.vgic.its_vpe;


    if ( vpe->resident )
        return;

    /*
     * Before making the VPE resident, make sure the redistributor
     * corresponding to our current CPU expects us here
     */
    WARN_ON(gicv4_vpe_set_affinity(vcpu));
    its_vpe_mask_db(vpe);
    its_make_vpe_resident(vpe, vcpu->processor);
    vpe->resident = true;
}

void vgic_v4_put(struct vcpu *vcpu, bool need_db)
{
    struct its_vpe *vpe = vcpu->arch.vgic.its_vpe;

    if ( !vpe->resident )
        return;

    its_make_vpe_non_resident(vpe, vcpu->processor);
    if ( need_db )
        /* Enable the doorbell, as the guest is going to block */
        its_vpe_unmask_db(vpe);
    vpe->resident = false;
}

static int its_vlpi_set_doorbell(struct its_vlpi_map *map, bool enable)
{
    if (map->db_enabled == enable)
        return 0;

    map->db_enabled = enable;

    /*
     * Ideally, we'd issue a VMAPTI to set the doorbell to its LPI
     * value or to 1023, depending on the enable bit. But that
     * would be issuing a mapping for an /existing/ DevID+EventID
     * pair, which is UNPREDICTABLE. Instead, let's issue a VMOVI
     * to the /same/ vPE, using this opportunity to adjust the doorbell.
     */
    return its_send_cmd_vmovi(map->dev->hw_its, map);
}

int its_vlpi_prop_update(struct pending_irq *pirq, uint8_t property,
                         bool needs_inv)
{
    struct its_vlpi_map *map;
    unsigned int cpu;
    int ret;

    if ( !pirq->vlpi_map )
        return -EINVAL;

    map = pirq->vlpi_map;

    /* Cache the updated property and update the vproptable. */
    map->properties = property;
    lpi_write_config(map->vm->vproptable, pirq->irq, 0xff, property);

    if ( needs_inv )
    {
        cpu = map->vm->vpes[map->vpe_idx]->col_idx;
        ret = its_inv_lpi(map->dev->hw_its, map->dev, map->eventid, cpu);
        if ( ret )
            return ret;
    }

    return its_vlpi_set_doorbell(map, property & LPI_PROP_ENABLED);
}

