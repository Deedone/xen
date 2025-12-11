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
#include <xen/init.h>
#include <xen/lib.h>
#include <xen/sched.h>
#include <xen/spinlock.h>

#include <asm/gic_v3_defs.h>
#include <asm/gic_v3_its.h>
#include <asm/gic_v4_its.h>
#include <asm/io.h>
#include <asm/vgic.h>


/*
 * VPE ID is at most 16 bits.
 * Using a bitmap here limits us to 65536 concurrent VPEs.
 */
static unsigned long *vpeid_mask;

static spinlock_t vpeid_alloc_lock = SPIN_LOCK_UNLOCKED;

static uint16_t vmovp_seq_num;
static spinlock_t vmovp_lock = SPIN_LOCK_UNLOCKED;

static struct {
    spinlock_t lock;
    struct its_device *dev;
    struct its_vpe **vpes;
    int next_victim;
} vpe_proxy;

/* per-re-distributor VPE affinity group */
DEFINE_PER_CPU(cpumask_t*, vpe_table_mask);
#define vpe_table_mask_cpu(cpu) (per_cpu(vpe_table_mask, cpu))
#define VPE_TABLE_MASK          (this_cpu(vpe_table_mask))

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
    return per_cpu(rbase, cpu) + SZ_128K;
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

static bool allocate_vpe_l2_table(unsigned int cpu, uint32_t id)
{
    void __iomem *rdbase = per_cpu(rbase, cpu);
    unsigned int psz, esz, idx, npg, gpsz;
    uint64_t val;
    void *buffer;
    __le64 *table;
    paddr_t pa;

    /* Skip non-present CPUs */
    if ( !rdbase )
        return true;

    val = gits_read_vpropbaser(gic_data_rdist_vlpi_base(cpu) +
                               GICR_VPROPBASER);

    gpsz = FIELD_GET(GICR_VPROPBASER_4_1_PAGE_SIZE, val);
    esz = FIELD_GET(GICR_VPROPBASER_4_1_ENTRY_SIZE, val) + 1;
    npg  = FIELD_GET(GICR_VPROPBASER_4_1_SIZE, val) + 1;

    switch ( gpsz ) {
    default:
        WARN_ON(1);
        fallthrough;
    case GIC_PAGE_SIZE_4K:
        psz = SZ_4K;
        break;
    case GIC_PAGE_SIZE_16K:
        psz = SZ_16K;
        break;
    case GIC_PAGE_SIZE_64K:
        psz = SZ_64K;
        break;
    }

    /* Don't allow vpe_id that exceeds single, flat table limit */
    if ( !(val & GICR_VPROPBASER_4_1_INDIRECT) )
        return (id < (npg * psz / (esz * SZ_8)));

    /* Compute 1st level table index and check if that exceeds table limit */
    idx = id >> ilog2(psz / (esz * SZ_8));
    if ( idx >= (npg * psz / GITS_LVL1_ENTRY_SIZE) )
        return false;

    pa = FIELD_GET(GICR_VPROPBASER_4_1_ADDR, val);
    table = (__le64 *)maddr_to_virt(pa << 12);

    /* Allocate memory for 2nd level table */
    if ( !table[idx] )
    {
        buffer = _xzalloc(psz, psz);
        printk(XENLOG_G_INFO "Allocated VPE L2 table for virt %px phys %lx\n",
               buffer, virt_to_maddr(buffer));
        if ( !buffer )
            return -ENOMEM;

        /* Flush Lvl2 table if hw doesn't support coherency */
        if ( !(val & GICR_VPROPBASER_SHAREABILITY_MASK) )
            clean_and_invalidate_dcache_va_range(buffer, psz);

        table[idx] = cpu_to_le64(virt_to_maddr(buffer) | GITS_BASER_VALID);

        /* Flush Lvl1 entry to PoC if hw doesn't support coherency */
        if ( !(val & GICR_VPROPBASER_SHAREABILITY_MASK) )
            clean_and_invalidate_dcache_va_range(table + idx, GITS_LVL1_ENTRY_SIZE);

        /* Ensure updated table contents are visible to RD hardware */
        dsb(sy);
    }

    return true;
}

static bool __init its_alloc_vpe_entry(uint32_t vpe_id)
{
    struct host_its *hw_its;
    unsigned int cpu;

    printk("ALLOC ENTRY VPE\n");
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

    /* Non v4.1? No need to iterate RDs and go back early. */
    if ( !gic_has_v4_1_extension() )
        return true;

    /*
     * Make sure the L2 tables are allocated for all copies of
     * the L1 table on *all* v4.1 RDs.
     */
    for_each_possible_cpu(cpu)
        if ( !allocate_vpe_l2_table(cpu, vpe_id) )
            return false;

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

static void __maybe_unused print_baser(struct host_its *its)
{
    uint64_t val;
    
    val = readq(its->its_base + GITS_BASER0);
    printk("BASER 0: 0x%lx\n", val);
    val = readq(its->its_base + GITS_BASER1);
    printk("BASER 1: 0x%lx\n", val);
    val = readq(its->its_base + GITS_BASER2);
    printk("BASER 2: 0x%lx\n", val);
    
    printk("Valid %lx\n", (val & BIT(63,UL)) >> 63);
    printk("Indirect %lx\n", (val & BIT(62,UL)) >> 62);
    printk("InnerCache %lx\n", (val & GENMASK(61, 59)) >> 59);
    printk("Type %lx\n", (val & GENMASK(58, 56)) >> 56);
    printk("OuterCache %lx\n", (val & GENMASK(55, 53)) >> 53);
    printk("EntrySize %lx\n", (val & GENMASK(52, 48)) >> 48);
    printk("PhysAddr %lx\n", (val & GENMASK(47, 12)));
    printk("ShareAbility %lx\n", (val & GENMASK(11, 10)) >> 10);
    printk("PageSize %lx\n", (val & GENMASK(9, 8)) >> 8);
    printk("Size %lx\n", (val & GENMASK(7, 0)));
    
    
    val = readq(its->its_base + GITS_BASER3);
    printk("BASER 3: 0x%lx\n", val);
    val = readq(its->its_base + GITS_BASER4);
    printk("BASER 4: 0x%lx\n", val);
    val = readq(its->its_base + GITS_BASER5);
    printk("BASER 5: 0x%lx\n", val);
    val = readq(its->its_base + GITS_BASER6);
    printk("BASER 6: 0x%lx\n", val);
    val = readq(its->its_base + GITS_BASER7);
    printk("BASER 7: 0x%lx\n", val);
}

void check_errs(void);
static int its_send_cmd_vmapp(struct host_its *its, struct its_vpe *vpe,
                              bool valid)
{
    uint64_t cmd[4];
    uint16_t vpeid = vpe->vpe_id;
    uint64_t vpt_addr, vprop_addr;
    bool alloc = 0, ptz;
    int ret;

    cmd[0] = GITS_CMD_VMAPP;
    cmd[1] = (uint64_t)vpeid << 32;
    cmd[2] = valid ? GITS_VALID_BIT : 0;

    /* Unmap command */
    if ( !valid )
    {
        if ( its->is_v4_1 )
            alloc = !atomic_dec_return(&vpe->vmapp_count);

        goto out;
    }

    /* Target redistributor */
    cmd[2] |= encode_rdbase(its, vpe->col_idx, 0x0);
    vpt_addr = virt_to_maddr(vpe->vpendtable);
    cmd[3] = (vpt_addr & GENMASK(51, 16)) |
             ((HOST_LPIS_NRBITS - 1) & GENMASK(4, 0));

    if ( !its->is_v4_1 )
        goto out;

    alloc = atomic_inc_return(&vpe->vmapp_count) == 1 ? true : false;
    cmd[0] |= alloc ? GITS_ALLOC_BIT : 0;
    printk("VMAPP vpe %d cpu %d ALLOC %d VPT %lx\n", vpeid, vpe->col_idx, alloc, vpt_addr);
    /* Virtual property table */
    vprop_addr = virt_to_maddr(vpe->its_vm->vproptable);
    cmd[0] |= vprop_addr & GENMASK(51, 16);

    /*
     * GICv4.1 provides a way to get the VLPI state, which needs the vPE
     * to be unmapped first, and in this case, we may remap the vPE
     * back while the VPT is not empty. So we can't assume that the
     * VPT is empty on map. This is why we never advertise PTZ.
     */
    ptz = false;
    cmd[0] |= ptz ? GITS_PTZ_BIT : 0;

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

static int gicv4_vpe_db_proxy_unmap_locked(struct its_vpe *vpe)
{
    int ret;

    /* GICv4.1 doesn't use a proxy, so nothing to do here */
    if ( gic_has_v4_1_extension() )
        return 0;

    /* Already unmapped? */
    if ( vpe->vpe_proxy_event == -1 )
        return 0;

    ret = its_send_cmd_discard(vpe_proxy.dev->hw_its, vpe_proxy.dev,
                               vpe->vpe_proxy_event);
    if ( ret )
        return ret;
    vpe_proxy.vpes[vpe->vpe_proxy_event] = NULL;

    /*
     * We don't track empty slots at all, so let's move the
     * next_victim pointer to quickly reuse the unmapped slot
     */
    if ( vpe_proxy.vpes[vpe_proxy.next_victim] )
        vpe_proxy.next_victim = vpe->vpe_proxy_event;

    vpe->vpe_proxy_event = -1;

    return 0;
}

static void gicv4_vpe_db_proxy_unmap(struct its_vpe *vpe)
{
    /* GICv4.1 doesn't use a proxy, so nothing to do here */
    if ( gic_has_v4_1_extension() )
        return;

    if ( !gic_support_directLPI() )
    {
        unsigned long flags;

        spin_lock_irqsave(&vpe_proxy.lock, flags);
        gicv4_vpe_db_proxy_unmap_locked(vpe);
        spin_unlock_irqrestore(&vpe_proxy.lock, flags);
    }
}

/*
 * If a GICv4.0 doesn't implement Direct LPIs (which is extremely
 * likely), the only way to perform an invalidate is to use a fake
 * device to issue an INV command, implying that the LPI has first
 * been mapped to some event on that device. Since this is not exactly
 * cheap, we try to keep that mapping around as long as possible, and
 * only issue an UNMAP if we're short on available slots.
 *
 * GICv4.1 mandates that we're able to invalidate by writing to a
 * MMIO register. And most of the time, we don't even have to invalidate
 * vPE doorbell, as the redistributor can be told whether to generate a
 * doorbell or not.
 */
static int gicv4_vpe_db_proxy_map_locked(struct its_vpe *vpe)
{
    int ret;

    /* GICv4.1 doesn't use a proxy, so nothing to do here */
    if ( gic_has_v4_1_extension() )
        return 0;

    /* Already mapped? */
    if ( vpe->vpe_proxy_event != -1 )
        return 0;

    /* This slot was already allocated. Kick the other VPE out. */
    if ( vpe_proxy.vpes[vpe_proxy.next_victim] )
    {
        struct its_vpe *old_vpe = vpe_proxy.vpes[vpe_proxy.next_victim];

        ret = gicv4_vpe_db_proxy_unmap_locked(old_vpe);
        if ( ret )
            return ret;
    }

    /* Map the new VPE instead */
    vpe_proxy.vpes[vpe_proxy.next_victim] = vpe;
    vpe->vpe_proxy_event = vpe_proxy.next_victim;
    vpe_proxy.next_victim = (vpe_proxy.next_victim + 1) %
                            vpe_proxy.dev->eventids;

    return its_send_cmd_mapti(vpe_proxy.dev->hw_its, vpe_proxy.dev->host_devid,
                              vpe->vpe_proxy_event, vpe->vpe_db_lpi,
                              vpe->col_idx);
}

int __init gicv4_init_vpe_proxy(void)
{
    struct host_its *hw_its;
    uint32_t devid;

    /* GICv4.1 doesn't use a proxy, so nothing to do here */
    if ( gic_has_v4_1_extension() )
        return 0;

    if ( gic_support_directLPI() )
    {
        printk("ITS: Using DirectLPI for GICv4 VPE invalidation\n");
        return 0;
    }

    /* Any ITS will do, even if not v4 */
    hw_its = list_first_entry(&host_its_list, struct host_its, entry);

    vpe_proxy.vpes = xzalloc_array(struct its_vpe *, nr_cpu_ids);
    if ( !vpe_proxy.vpes )
    {
        printk(XENLOG_ERR "ITS: Can't allocate GICv4 VPE proxy device array\n");
        return -ENOMEM;
    }

    /* Use the last possible DevID */
    devid = BIT(hw_its->devid_bits, UL) - 1;
    vpe_proxy.dev = its_create_device(hw_its, devid, nr_cpu_ids);
    if ( !vpe_proxy.dev )
    {
        printk(XENLOG_ERR "ITS: Can't allocate GICv4 VPE proxy device\n");
        return -ENOMEM;
    }

    spin_lock_init(&vpe_proxy.lock);
    vpe_proxy.next_victim = 0;
    printk(XENLOG_INFO
           "ITS: Allocated DevID %u as GICv4 VPE proxy device\n", devid);

    return 0;
}

static int __init its_vpe_init(struct its_vpe *vpe)
{
    int vpe_id, rc = -ENOMEM;
    struct page_info *vpendtable;
    struct host_its *hw_its;

    printk("VPE INIT\n");
    /* Allocate vpe id */
    vpe_id = its_alloc_vpeid(vpe);
    if ( vpe_id < 0 )
        return rc;

    /* Allocate VPT */
    vpendtable = lpi_allocate_pendtable();

    // vpendtable = 
    if ( !vpendtable )
        goto fail_vpt;

    if ( !its_alloc_vpe_entry(vpe_id) )
        goto fail_entry;

    rwlock_init(&vpe->lock);
    vpe->vpe_id = vpe_id;
    vpe->vpendtable = page_to_virt(vpendtable);
    if ( gic_has_v4_1_extension() )
        atomic_set(&vpe->vmapp_count, 0);
    else
        vpe->vpe_proxy_event = -1;

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
    gicv4_vpe_db_proxy_unmap(vpe);
    its_free_vpeid(vpe->vpe_id);
    xfree(vpe->vpendtable);
}

int vgic_v4_its_vm_init(struct domain *d)
{
    struct its_vm *its_vm = &d->arch.vgic.its_vm;
    unsigned int nr_vcpus = d->max_vcpus;
    unsigned int nr_db_lpis, nr_chunks, i = 0;
    uint32_t *db_lpi_bases;
    struct page_info *vproptable;
    int ret = -ENOMEM;

    printk("%s %d\n", __func__, __LINE__);
    printk("Creating vm fow vgic %p\n", &d->arch.vgic);


    if ( !gicv3_its_host_has_its() )
        return 0;

    d->arch.vgic.its_vm.vpes = xzalloc_array(struct its_vpe *, nr_vcpus);
    printk("VPEs ALLOCATED virt %p phys %lx\n", d->arch.vgic.its_vm.vpes, virt_to_maddr(d->arch.vgic.its_vm.vpes));
    if ( !d->arch.vgic.its_vm.vpes )
        return ret;
    d->arch.vgic.its_vm.nr_vpes = nr_vcpus;

    /* Allocate a doorbell interrupt for each VPE. */
    nr_db_lpis = d->arch.vgic.its_vm.nr_vpes;
    nr_chunks = DIV_ROUND_UP(nr_db_lpis, LPI_BLOCK);
    db_lpi_bases = xzalloc_array(uint32_t, nr_chunks);
    if ( !db_lpi_bases )
        return ret;
    printk("DB LPI BASES ALLOCATED %p\n", db_lpi_bases);

    do {
        /* Allocate doorbell interrupts in chunks of LPI_BLOCK (=32). */
        ret = gicv3_allocate_host_lpi_block(d, &db_lpi_bases[i]);
        if ( ret )
            goto fail_db;
    } while ( ++i < nr_chunks );

    vproptable = lpi_allocate_proptable();
    if ( !vproptable )
        goto fail_db;

    its_vm->db_lpi_bases = db_lpi_bases;
    its_vm->nr_db_lpis = nr_db_lpis;
    its_vm->vproptable = page_to_virt(vproptable);

    return 0;

 fail_db:
    while ( --i >= 0 )
        gicv3_free_host_lpi_block(its_vm->db_lpi_bases[i]);
    xfree(db_lpi_bases);

    return ret;
}

void vgic_v4_free_its_vm(struct domain *d)
{
    struct its_vm *its_vm = &d->arch.vgic.its_vm;
    int nr_chunks = DIV_ROUND_UP(its_vm->nr_db_lpis, LPI_BLOCK);

    if ( its_vm->vpes )
        xfree(its_vm->vpes);
    while ( --nr_chunks >= 0 )
        gicv3_free_host_lpi_block(its_vm->db_lpi_bases[nr_chunks]);
    if ( its_vm->db_lpi_bases )
        xfree(its_vm->db_lpi_bases);
    if ( its_vm->vproptable )
        xfree(its_vm->vproptable);
}

static void its_vpe_unmask_db(struct its_vpe *vpe);
int vgic_v4_its_vpe_init(struct vcpu *vcpu)
{
    struct its_vm *its_vm = &vcpu->domain->arch.vgic.its_vm;
    struct its_vpe *its_vpe = &vcpu->arch.vgic.its_vpe;
    // struct host_its *hw_its;
    unsigned int vcpuid = vcpu->vcpu_id;
    int ret;

    its_vm->vpes[vcpuid] = its_vpe;
    its_vpe->vpe_db_lpi = its_vm->db_lpi_bases[vcpuid/32] + (vcpuid % 32);
    check_errs();
    printk("VPE DB LPI %x\n", its_vpe->vpe_db_lpi);
    /*
     * Sometimes vlpi gets firstly mapped before associated vpe
     * becoming resident, so in case missing the interrupt, we intend to
     * enable doorbell at the initialization stage
     */

     check_errs();
     check_errs();
    its_vpe_unmask_db(its_vpe);
    its_vpe->its_vm = its_vm;

    gicv3_lpi_update_host_entry(its_vpe->vpe_db_lpi, vcpu->domain->domain_id,
                                INVALID_LPI, true, vcpu->vcpu_id);

    ret = its_vpe_init(its_vpe);
    if ( ret )
    {
        its_vpe_teardown(its_vpe);
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
    pirq_clear_forwarded_to_vcpu(pirq);
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
    struct its_vm *vm = &d->arch.vgic.its_vm;
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

bool event_is_forwarded_to_vcpu(struct its_device *dev, uint32_t eventid)
{
    struct pending_irq *pirq;

    /* No vlpi maps at all ? */
    if ( !dev->event_map.vlpi_maps)
        return false;

    pirq = dev->event_map.vlpi_maps[eventid].pirq;
    return pirq_is_forwarded_to_vcpu(pirq);
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

int its_set_vlpi_state(struct pending_irq *pirq, bool state)
{
    struct its_vlpi_map *map;
    int ret;

    map = pirq->vlpi_map;
    if ( !map )
        return -EINVAL;

    if ( state )
        ret = its_send_cmd_int(map->dev->hw_its, map->dev->host_devid,
                               map->eventid);
    else
        ret = its_send_cmd_clear(map->dev->hw_its, map->dev->host_devid,
                                 map->eventid);

    return ret;
}

/* GICR_SYNCR.Busy == 1 until the invalidation completes. */
static void wait_for_syncr(void __iomem *rdbase)
{
    while ( readl_relaxed(rdbase + GICR_SYNCR) & 1 )
        cpu_relax();
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

/*
 * LPI/VLPI invalidation through MMIO registers(GICR_INVLPIR,
 * GICR_INVALLR, and GICR_SYNCR)
 */
void direct_lpi_inv(struct its_device *dev, uint32_t eventid,
                    uint32_t db_lpi, unsigned int cpu)
{
    void __iomem *rdbase;
    uint64_t val;
    struct its_vpe *vpe = NULL;
    unsigned long flags;

    /* Register-based VLPI invalidation */
    if ( dev )
    {
        struct its_vlpi_map *map = &dev->event_map.vlpi_maps[eventid];

        /*
         * The GICR_INVLPIR, GICR_INVALLR, and GICR_SYNCR registers
         * are mandatory in GICv4.1
         */
        WARN_ON(!gic_has_v4_1_extension());

        val  = GICR_INVLPIR_V;
        vpe = map->vm->vpes[map->vpe_idx];
        val |= FIELD_PREP(GICR_INVLPIR_VPEID, vpe->vpe_id);
        val |= FIELD_PREP(GICR_INVLPIR_INTID, map->vintid);
        vpe_to_cpuid_lock(vpe, &flags);
    }
    else
        /* Register-based LPI invalidation for DB on GICv4.0 */
        val = FIELD_PREP(GICR_INVLPIR_INTID, db_lpi);

    rdbase = per_cpu(rbase, cpu);
    writeq_relaxed(val, rdbase + GICR_INVLPIR);
    wait_for_syncr(rdbase);

    if ( vpe )
        vpe_to_cpuid_unlock(vpe, &flags);
}

static void gicv4_vpe_db_proxy_move(struct its_vpe *vpe, unsigned int from,
                                    unsigned int to)
{
    unsigned long flags;

    /* GICv4.1 doesn't use a proxy, so nothing to do here */
    if ( gic_has_v4_1_extension() )
        return;

    if ( gic_support_directLPI() )
    {
        void __iomem *rdbase;

        rdbase = per_cpu(rbase, from);
        /* Clear potential pending state on the old redistributor */
        writeq_relaxed(vpe->vpe_db_lpi, rdbase + GICR_CLRLPIR);
        wait_for_syncr(rdbase);
        return;
    }

    spin_lock_irqsave(&vpe_proxy.lock, flags);

    gicv4_vpe_db_proxy_map_locked(vpe);

    /* MOVI instructs the appropriate Redistributor to move the pending state */
    its_send_cmd_movi(vpe_proxy.dev->hw_its, vpe_proxy.dev->host_devid,
                      vpe->vpe_proxy_event, to);

    /*
     * ARM spec says that If, after using MOVI to move an interrupt from
     * collection A to collection B, software moves the same interrupt again
     * from collection B to collection C, a SYNC command must be used before
     * the second MOVI for the Redistributor associated with collection A to
     * ensure correct behavior.
     * So each time we issue VMOVI, we VSYNC the old VPE for good measure.
     */
    WARN_ON(its_send_cmd_sync(vpe_proxy.dev->hw_its, from));

    spin_unlock_irqrestore(&vpe_proxy.lock, flags);
}

static void its_vpe_send_inv_db(struct its_vpe *vpe)
{
    if ( gic_support_directLPI() )
    {
        unsigned int cpu = vpe->col_idx;

        /* Target the redistributor this VPE is currently known on */
        direct_lpi_inv(NULL, 0, vpe->vpe_db_lpi, cpu);
    }
    else
    {
        struct its_device *dev = vpe_proxy.dev;
        unsigned long flags;

        spin_lock_irqsave(&vpe_proxy.lock, flags);
        gicv4_vpe_db_proxy_map_locked(vpe);
        its_send_cmd_inv(dev->hw_its, dev->host_devid, vpe->vpe_proxy_event);
        spin_unlock_irqrestore(&vpe_proxy.lock, flags);
    }
}

static struct host_its *find_4_1_its(void)
{
    /* Preserve the search result for multiple call */
    static struct host_its *its = NULL;
    static bool once = true;

    /* The search will be only done once */
    if ( !its && once )
    {
        list_for_each_entry(its, &host_its_list, entry)
        {
            if ( its->is_v4_1 )
                return its;
        }

        /* No found */
        its = NULL;
        once = false;
    }

    return its;
}

bool vgic_has_directVSGI(struct domain *d)
{
    return ( find_4_1_its() != NULL ) && (d->arch.vgic.version == GIC_V4_1);
}

void MYTEST(void)
{
    struct host_its *hw_its;
    uint64_t cmd[4];
    uint16_t vpeid = 0;

    hw_its = list_first_entry(&host_its_list, struct host_its, entry);

    printk("MYTEST\n");
    cmd[0] = GITS_CMD_INVDB;
    cmd[1] = (uint64_t)vpeid << 32;
    cmd[2] = 0x00;
    cmd[3] = 0x00;

    its_send_command(hw_its, cmd);

    check_errs();

    /* INVDB is synchronized by a VSYNC command. */
}

static int its_vpe_send_4_1_inv_db(struct host_its *its, struct its_vpe *vpe)
{
    uint64_t cmd[4];
    uint16_t vpeid = vpe->vpe_id;
    int ret;

    cmd[0] = GITS_CMD_INVDB;
    cmd[1] = (uint64_t)vpeid << 32;
    cmd[2] = 0x00;
    cmd[3] = 0x00;

    ret = its_send_command(its, cmd);
    if ( ret )
        return ret;

    /* INVDB is synchronized by a VSYNC command. */
    return its_send_cmd_vsync(its, vpeid);
}

static void its_vpe_inv_db(struct its_vpe *vpe)
{
    struct host_its *its;

    /*
     * GICv4.1 wants doorbells to be invalidated using the
     * INVDB command in order to be broadcast to all RDs. Send
     * it to the first valid ITS, and let the HW do its magic.
     */
    its = find_4_1_its();
    if ( its )
        its_vpe_send_4_1_inv_db(its, vpe);
    else
        its_vpe_send_inv_db(vpe);
}

void its_vpe_mask_db(struct its_vpe *vpe)
{
    /* Only clear enable bit. */
    // printk("MASK DB\n");
    lpi_write_config(lpi_data.lpi_property, vpe->vpe_db_lpi, LPI_PROP_ENABLED, 0);
    its_vpe_inv_db(vpe);
}

static void its_vpe_unmask_db(struct its_vpe *vpe)
{
    /* Only set enable bit. */
    // printk("UNMASK DB\n");
    // printk("db lpi %x\n", vpe->vpe_db_lpi);
    lpi_write_config(lpi_data.lpi_property, vpe->vpe_db_lpi, 0, LPI_PROP_ENABLED);
    its_vpe_inv_db(vpe);
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

static void its_make_vpe_resident(struct its_vpe *vpe, unsigned int cpu)
{
    void __iomem *vlpi_base = gic_data_rdist_vlpi_base(cpu);
    uint64_t val;

    /* Switch in this VM's virtual property table. */
    val  = virt_to_maddr(vpe->its_vm->vproptable) & GENMASK(51, 12);
    val |= GIC_BASER_CACHE_RaWb << GICR_VPROPBASER_INNER_CACHEABILITY_SHIFT;
    val |= GIC_BASER_InnerShareable << GICR_VPROPBASER_SHAREABILITY_SHIFT;
    val |= GIC_BASER_CACHE_SameAsInner << GICR_VPROPBASER_OUTER_CACHEABILITY_SHIFT;
    // val |= GIC_BASER_CACHE_nC << GICR_VPROPBASER_INNER_CACHEABILITY_SHIFT;
    // val |= GIC_BASER_NonShareable << GICR_VPROPBASER_SHAREABILITY_SHIFT;
    // val |= GIC_BASER_CACHE_SameAsInner << GICR_VPROPBASER_OUTER_CACHEABILITY_SHIFT;
    val |= (HOST_LPIS_NRBITS - 1) & GICR_VPROPBASER_IDBITS_MASK;
    gits_write_vpropbaser(val, vlpi_base + GICR_VPROPBASER);

    /* Switch in this VCPU's VPT. */
    val  = virt_to_maddr(vpe->vpendtable) & GENMASK(51, 16);
    val |= GIC_BASER_CACHE_RaWaWb << GICR_VPENDBASER_INNER_CACHEABILITY_SHIFT;
    val |= GIC_BASER_InnerShareable << GICR_VPENDBASER_SHAREABILITY_SHIFT;
    val |= GIC_BASER_CACHE_SameAsInner << GICR_VPENDBASER_OUTER_CACHEABILITY_SHIFT;
    // val |= GIC_BASER_CACHE_nC << GICR_VPROPBASER_INNER_CACHEABILITY_SHIFT;
    // val |= GIC_BASER_NonShareable << GICR_VPROPBASER_SHAREABILITY_SHIFT;
    // val |= GIC_BASER_CACHE_SameAsInner << GICR_VPROPBASER_OUTER_CACHEABILITY_SHIFT;
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

static void encode_vmovp_v4_1(uint64_t *cmd, uint32_t default_db_pintid)
{
    /* Always requiring a Default Doorbell on GICv4.1 */
    cmd[2] |= GITS_DB_BIT;
    cmd[3] |= default_db_pintid;
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

        if ( hw_its->is_v4_1 )
            encode_vmovp_v4_1(cmd, vpe->vpe_db_lpi);

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

        if ( !hw_its->is_v4 )
            continue;

        cmd[0] = GITS_CMD_VMOVP | ((uint64_t)vmovp_seq_num << 32);
        cmd[1] = its_list_map | ((uint64_t)vpeid << 32);
        cmd[2] = encode_rdbase(hw_its, vpe->col_idx, 0x0);
        cmd[3] = 0x00;

        if ( hw_its->is_v4_1 )
            encode_vmovp_v4_1(cmd, vpe->vpe_db_lpi);

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

static void gicv4_vpe_send_clear(struct its_vpe *vpe)
{
    if ( gic_support_directLPI() )
    {
        void __iomem *rdbase;

        rdbase = per_cpu(rbase, vpe->col_idx);
        /* Clear potential pending state */
        writeq_relaxed(vpe->vpe_db_lpi, rdbase + GICR_CLRLPIR);
        wait_for_syncr(rdbase);
    }
    else
    {
        unsigned long flags;

        spin_lock_irqsave(&vpe_proxy.lock, flags);

        gicv4_vpe_db_proxy_map_locked(vpe);

        its_send_cmd_clear(vpe_proxy.dev->hw_its, vpe_proxy.dev->host_devid,
                           vpe->vpe_proxy_event);

        spin_unlock_irqrestore(&vpe_proxy.lock, flags);
    }
}

static uint64_t its_clear_vpend_valid(void __iomem *vlpi_base, uint64_t clr,
                                      uint64_t set);
static void its_make_vpe_4_1_resident(struct its_vpe *vpe, unsigned int cpu)
{
    void __iomem *vlpi_base = gic_data_rdist_vlpi_base(cpu);
    uint64_t val = 0;

    its_clear_vpend_valid(vlpi_base, 0, 0);
    // printk("%d on %d resident\n", vpe->vpe_id, cpu);
    val |= GICR_VPENDBASER_Valid;
    /* All guest LPIs are forwarded as Group 1 */
    val |= GICR_VPENDBASER_4_1_VGRP1EN;
    val |= FIELD_PREP(GICR_VPENDBASER_4_1_VPEID, vpe->vpe_id);

    // printk("MAKE VPE %d resident on %d\n", vpe->vpe_id, cpu);
    gits_write_vpendbaser(val, vlpi_base + GICR_VPENDBASER);
    check_errs();
}

static int gicv4_vpe_set_affinity(struct vcpu *vcpu)
{
    struct its_vpe *vpe = &vcpu->arch.vgic.its_vpe;
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

    /*
     * GICv4.1 allows us to skip VMOVP if moving to a cpu whose RD
     * is sharing its VPE table with the current one.
     */
    if ( gic_has_v4_1_extension() )
        if ( cpumask_empty(vpe_table_mask_cpu(to)) &&
             cpumask_test_cpu(from, vpe_table_mask_cpu(to)) )
            goto out;

    ret = its_send_cmd_vmovp(vpe);
    if ( ret )
        goto out;
    gicv4_vpe_db_proxy_move(vpe, from, to);

 out:
    vpe_to_cpuid_unlock(vpe, &flags);
    return ret;
}

void vgic_v4_load(struct vcpu *vcpu)
{
    struct its_vpe *vpe = &vcpu->arch.vgic.its_vpe;


    if ( vpe->resident )
        return;

    /*
     * Before making the VPE resident, make sure the redistributor
     * corresponding to our current CPU expects us here
     */
    WARN_ON(gicv4_vpe_set_affinity(vcpu));

    if ( gic_has_v4_1_extension() ) {
        /* GICv4.1 can directly deal with doorbells */
        its_make_vpe_4_1_resident(vpe, vcpu->processor);
    } else
    {
        /* Disabled the doorbell, as we're about to enter the guest */
        its_vpe_mask_db(vpe);
        its_make_vpe_resident(vpe, vcpu->processor);
    }

    /*
     * Now that the VPE is resident, let's get rid of a potential
     * doorbell interrupt that would still be pending. This is a
     * GICv4.0 only "feature".
     */
    if ( !gic_has_v4_1_extension() )
        gicv4_vpe_send_clear(vpe);

    vpe->resident = true;
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

static void its_make_vpe_non_resident(struct its_vpe *vpe, unsigned int cpu)
{
    void __iomem *vlpi_base = gic_data_rdist_vlpi_base(cpu);
    uint64_t val;

    val = its_clear_vpend_valid(vlpi_base, 0, 0);
    vpe->idai = val & GICR_VPENDBASER_IDAI;
    vpe->pending_last = val & GICR_VPENDBASER_PendingLast;
    // vpe->pending_last = 0;
}

static void its_make_vpe_4_1_non_resident(struct its_vpe *vpe, unsigned int cpu,
                                          bool req_db)
{
    void __iomem *vlpi_base = gic_data_rdist_vlpi_base(cpu);
    uint64_t val;

    // printk("%d on %d non resident\n", vpe->vpe_id, cpu);
    if ( req_db )
    {
        /*
         * vPE is going to block: make the vPE non-resident with
         * PendingLast clear and DB set. The GIC guarantees that if
         * we read-back PendingLast clear, then a doorbell will be
         * delivered when an interrupt comes.
         */
        val = its_clear_vpend_valid(vlpi_base,
                                    GICR_VPENDBASER_PendingLast,
                                    GICR_VPENDBASER_4_1_DB);
        vpe->pending_last = val & GICR_VPENDBASER_PendingLast;
    }
    else
    {
        /*
         * We're not blocking, so just make the vPE non-resident
         * with PendingLast set, indicating that we'll be back.
         * When GICR_VPENDBASER.Valid is written from 1 to 0,
         * if GICR_VPENDBASER.PendingLast is written as 1 then Doorbell bit
         * is treated as 0.
         */
         val = its_clear_vpend_valid(vlpi_base,
                                     0, GICR_VPENDBASER_PendingLast);
         vpe->pending_last = true;
    }
}

void vgic_v4_put(struct vcpu *vcpu, bool need_db)
{
    struct its_vpe *vpe = &vcpu->arch.vgic.its_vpe;

    if ( !vpe->resident )
        return;

    if ( !gic_has_v4_1_extension() )
    {
        its_make_vpe_non_resident(vpe, vcpu->processor);
        if ( need_db )
            /* Enable the doorbell, as the guest is going to block */
            its_vpe_unmask_db(vpe);
    }
    else
        /* GICv4.1 can directly deal with doorbells */
        its_make_vpe_4_1_non_resident(vpe, vcpu->processor, need_db);

    vpe->resident = false;
}

static int its_vpe_invall(struct its_vpe *vpe)
{
    struct host_its *its;

    list_for_each_entry(its, &host_its_list, entry)
    {
        if ( !its->is_v4 )
            continue;

        /*
         * Sending a VINVALL to a single ITS is enough, as all
         * we need is to reach the redistributors.
         */
        return its_send_cmd_vinvall(its, vpe);
    }

    return 0;
}

static void its_vpe_4_1_invall(struct its_vpe *vpe)
{
    unsigned int cpu;
    void __iomem *rdbase;
    uint64_t val;
    unsigned long flags;

    val  = GICR_INVALLR_V;
    val |= FIELD_PREP(GICR_INVALLR_VPEID, vpe->vpe_id);

    /* Target the redistributor this vPE is currently mapped on */
    cpu = vpe_to_cpuid_lock(vpe, &flags);
    rdbase = GICD_RDIST_BASE_CPU(cpu);
    writeq_relaxed(val, rdbase + GICR_INVALLR);
    wait_for_syncr(rdbase);
    vpe_to_cpuid_unlock(vpe, &flags);
}

int gicv4_its_handle_invall(struct domain *d, struct vcpu *vcpu)
{
    struct its_vpe *vpe = d->arch.vgic.its_vm.vpes[vcpu->vcpu_id];

    if ( !gic_has_v4_1_extension() )
        return its_vpe_invall(vpe);

    its_vpe_4_1_invall(vpe);

    return 0;
}

static uint64_t inherit_vpe_l1_table_from_rd(void)
{
    uint32_t aff;
    uint64_t val;
    unsigned int cpu;
    void __iomem *rbase;

    printk("%s %d\n", __func__, __LINE__);
    val = readl_relaxed(GICD_RDIST_BASE + GICR_TYPER);
    printk("%s %d\n", __func__, __LINE__);
    aff = compute_common_aff(val);
    printk("%s %d\n", __func__, __LINE__);

    for_each_present_cpu ( cpu )
    {
        printk("check cpu %d == %d\n", cpu, smp_processor_id());
        if ( cpu == smp_processor_id() )
            continue;

        if ( !cpu_online(cpu) )
            continue;

        printk("%s %d\n", __func__, __LINE__);
        printk("CPU %d\n", cpu);

        rbase = GICD_RDIST_BASE_CPU(cpu);
        printk("RBASE %lx\n", (unsigned long)rbase);
        printk("RBASE %lx\n", (unsigned long)rbase);
        printk("RBASE %lx\n", (unsigned long)rbase);
        printk("RBASE %lx\n", (unsigned long)rbase);
        printk("RBASE %lx\n", (unsigned long)rbase);
        printk("RBASE %lx\n", (unsigned long)rbase);

        val = readl_relaxed(rbase + GICR_TYPER);
        if ( aff != compute_common_aff(val) )
            continue;
        printk("%s %d\n", __func__, __LINE__);

        /*
         * At this point, we have found a particular CPU, which has
         * already booted, and its Redistributors lives in the same
         * CommonLPIAff group. Then we must inherit its VPROPBASER
         * to ensure they share the same copy of VPE configuration table.
         * Make sure we don't write the Z bit in that case.
         */
        val = gits_read_vpropbaser(gic_data_rdist_vlpi_base(cpu) +
                                   GICR_VPROPBASER);
        val &= ~GICR_VPROPBASER_4_1_Z;
        printk("%s %d\n", __func__, __LINE__);

        /*
         * All redistributors in the same CommonLPIaff group
         * share the same copy.
         */
        VPE_TABLE_MASK = vpe_table_mask_cpu(cpu);

        printk("%s %d\n", __func__, __LINE__);
        return val;
    }

    printk("%s %d\n", __func__, __LINE__);
    return 0;
}

static uint64_t __maybe_unused inherit_vpe_l1_table_from_its(void)
{
    struct host_its *its;
    uint64_t val;
    uint32_t aff;

    printk("%s %d\n", __func__, __LINE__);
    val = readl_relaxed(GICD_RDIST_BASE + GICR_TYPER);
    aff = compute_common_aff(val);

    printk("%s %d\n", __func__, __LINE__);
    list_for_each_entry(its, &host_its_list, entry)
    {
        uint64_t typer, baser;
        paddr_t addr;

        if ( !its->is_v4_1 )
            continue;

        printk("%s %d\n", __func__, __LINE__);
        typer = readq_relaxed(its->its_base + GITS_TYPER);
        if ( !FIELD_GET(GITS_TYPER_SVPET, typer) )
            continue;

        printk("%s %d\n", __func__, __LINE__);
        if ( aff != compute_its_aff(its) )
            continue;

        printk("%s %d\n", __func__, __LINE__);
        /* GICv4.1 guarantees that the vPE table is GITS_BASER2 */
        print_baser(its);
        baser = its->tables[2].val;
        if ( !(baser & GITS_BASER_VALID) )
            continue;

        printk("%s %d\n", __func__, __LINE__);
        /* We have found an ITS, from which we shall inherit vPE table */
        val = GICR_VPROPBASER_4_1_VALID;
        if ( baser & GITS_BASER_INDIRECT )
            val |= GICR_VPROPBASER_4_1_INDIRECT;
        val |= FIELD_PREP(GICR_VPROPBASER_4_1_PAGE_SIZE,
                          FIELD_GET(GITS_BASER_PAGE_SIZE_MASK, baser));
        switch ( FIELD_GET(GITS_BASER_PAGE_SIZE_MASK, baser) )
        {
        case GIC_PAGE_SIZE_64K:
            addr = GITS_BASER_ADDR_48_to_52(baser);
            break;
        default:
            addr = baser & GENMASK(47, 12);
            break;
        }
        printk("%s %d\n", __func__, __LINE__);
        val |= FIELD_PREP(GICR_VPROPBASER_4_1_ADDR, addr >> 12);
        val |= FIELD_PREP(GICR_VPROPBASER_SHAREABILITY_MASK,
                  FIELD_GET(GITS_BASER_SHAREABILITY_MASK, baser));
        val |= FIELD_PREP(GICR_VPROPBASER_INNER_CACHEABILITY_MASK,
                  FIELD_GET(GITS_BASER_INNER_CACHEABILITY_MASK, baser));
        val |= FIELD_PREP(GICR_VPROPBASER_4_1_SIZE,
                          GITS_BASER_NR_PAGES(baser) - 1);

        printk("%s %d\n", __func__, __LINE__);
        printk("INHERIT??? l1 %lx from ITS\n", val);
        return val;
    }

    printk("%s %d\n", __func__, __LINE__);
    return 0;
}

int allocate_vpe_l1_table(void)
{
    void __iomem *vlpi_base = gic_data_rdist_vlpi_base(smp_processor_id());
    void __iomem *buffer;
    uint64_t val, gpsz, npg, pa;
    unsigned int psz = SZ_64K;
    unsigned int epp, esz;

    printk("ALLOC l1 TABLE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    printk("ALLOC l1 TABLE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    printk("ALLOC l1 TABLE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    printk("ALLOC l1 TABLE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    printk("ALLOC l1 TABLE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    
    printk("%s %d\n", __func__, __LINE__);
    if ( !gic_has_v4_1_extension() )
        return 0;

    printk("%s %d\n", __func__, __LINE__);
    /*
     * if VPENDBASER.Valid is set, disable any previously programmed VPE
     * by setting PendingLast while clearing Valid. This has the effect of
     * making sure no doorbell will be generated and we can then safely
     * clear VPROPBASER.Valid.
     */
     printk("VPENDBASE is %lx\n",
            gits_read_vpendbaser(vlpi_base + GICR_VPENDBASER));
    if ( gits_read_vpendbaser(vlpi_base + GICR_VPENDBASER) &
                              GICR_VPENDBASER_Valid )
        gits_write_vpendbaser(GICR_VPENDBASER_PendingLast,
                              vlpi_base + GICR_VPENDBASER);

    printk("%s %d\n", __func__, __LINE__);
    /* Check if we can inherit the configuration from another Redistributor. */
    val = inherit_vpe_l1_table_from_rd();
    if ( val & GICR_VPROPBASER_4_1_VALID )
        goto out;

    VPE_TABLE_MASK = xzalloc(cpumask_t);
    if ( !VPE_TABLE_MASK )
        return -ENOMEM;

    printk("%s %d\n", __func__, __LINE__);
    /* Check if we can inherit the configuration from ITS Baser2. */
    val = inherit_vpe_l1_table_from_its();
    if ( val & GICR_VPROPBASER_4_1_VALID )
        goto out;

    printk("%s %d\n", __func__, __LINE__);
    /* First probe the page size and entry size */
    val = FIELD_PREP(GICR_VPROPBASER_4_1_PAGE_SIZE, GIC_PAGE_SIZE_64K);
    printk("VPROPBASER %lx\n", val);
    gits_write_vpropbaser(val, vlpi_base + GICR_VPROPBASER);
    val = gits_read_vpropbaser(vlpi_base + GICR_VPROPBASER);
    gpsz = FIELD_GET(GICR_VPROPBASER_4_1_PAGE_SIZE, val);
    esz = FIELD_GET(GICR_VPROPBASER_4_1_ENTRY_SIZE, val) + 1;

    printk("%s %d\n", __func__, __LINE__);
    switch ( gpsz )
    {
    default:
        gpsz = GIC_PAGE_SIZE_4K;
    case GIC_PAGE_SIZE_4K:
        psz = SZ_4K;
        break;
    case GIC_PAGE_SIZE_16K:
        psz = SZ_16K;
        break;
    case GIC_PAGE_SIZE_64K:
        psz = SZ_64K;
        break;
    }

    printk("%s %d\n", __func__, __LINE__);
    /* Start populating the register from scratch. */
    val = 0;
    val |= FIELD_PREP(GICR_VPROPBASER_4_1_PAGE_SIZE, gpsz);
    val |= FIELD_PREP(GICR_VPROPBASER_4_1_ENTRY_SIZE, esz - 1);

    printk("%s %d\n", __func__, __LINE__);
    /* How many entries per GIC page? */
    epp = psz / (esz * SZ_8);

    printk("%s %d\n", __func__, __LINE__);
    /*
     * If we need more than just a single L1 page, flag the table
     * as indirect and compute the number of required L1 pages.
     */
    if ( epp < MAX_VPEID )
    {
        int nl2;

        val |= GICR_VPROPBASER_4_1_INDIRECT;

        printk("%s %d\n", __func__, __LINE__);
        /* Number of L2 pages required to cover the VPEID range */
        nl2 = DIV_ROUND_UP(MAX_VPEID, epp);

        printk("%s %d\n", __func__, __LINE__);
        /* Number of L1 pages to point to the L2 pages */
        npg = DIV_ROUND_UP(nl2 * SZ_8, psz);
    }
    else
        npg = 1;

    val |= FIELD_PREP(GICR_VPROPBASER_4_1_SIZE, npg - 1);

    printk("%s %d\n", __func__, __LINE__);
    buffer = _xzalloc(npg * psz, psz);
    printk("ALLOCATED VPE L1 TABLE AT virt %p phys %lx\n", buffer, virt_to_maddr(buffer));
    if ( !buffer )
        return -ENOMEM;
    pa = virt_to_maddr(buffer);

    printk("%s %d\n", __func__, __LINE__);
    val |= FIELD_PREP(GICR_VPROPBASER_4_1_ADDR, pa >> 12);
    // val |= GIC_BASER_CACHE_RaWb;
    // val |= GIC_BASER_InnerShareable;
    val |= GIC_BASER_CACHE_nC << GICR_VPROPBASER_INNER_CACHEABILITY_SHIFT;
    val |= GIC_BASER_NonShareable << GICR_VPROPBASER_SHAREABILITY_SHIFT;
    val |= GICR_VPROPBASER_4_1_Z;
    val |= GICR_VPROPBASER_4_1_VALID;

    printk("%s %d\n", __func__, __LINE__);
 out:
    printk("VPROPBASER %lx\n", val);
    gits_write_vpropbaser(val, vlpi_base + GICR_VPROPBASER);
    cpumask_set_cpu(smp_processor_id(), VPE_TABLE_MASK);

    printk("%s %d\n", __func__, __LINE__);
    printk(XENLOG_DEBUG "CPU%d: VPROPBASER = %lx\n", smp_processor_id(), val);

    printk("%s %d\n", __func__, __LINE__);
    return 0;
}

void print_pend_1k(struct its_vpe *vpe)
{
    unsigned int i;
    uint32_t *table = (uint32_t *)vpe->vpendtable + +0x3F0;
    clean_and_invalidate_dcache_va_range(vpe->vpendtable, 1024);

    printk("VPE %d PEND TABLE DUMP\n", vpe->vpe_id);
    for ( i = 0; i < (128 / 8); i++ )
    {
        if ( (i % 8) == 0 )
            printk("%04x: ", i * 4);
        printk("%08x ", table[i]);
        if ( (i % 8) == 7 )
            printk("\n");
    }
    if ( (i % 8) != 0 )
        printk("\n");
}

static int its_send_cmd_vsgi(struct host_its *its, uint8_t vsgi_irq,
                             struct its_vpe *vpe, bool clear)
{
    uint64_t cmd[4];
    uint16_t vpeid = vpe->vpe_id;
    uint8_t priority = vpe->sgi_config[vsgi_irq].priority;
    int ret;

    if ( vsgi_irq > 15 )
        return -EINVAL;

    printk("ITS SEND CMD VSGI VPE %d IRQ %d EN %d CLEAR %d GROUP %d\n",
           vpeid, vsgi_irq, vpe->sgi_config[vsgi_irq].enabled, clear, vpe->sgi_config[vsgi_irq].group);

    cmd[0] = GITS_CMD_VSGI | ((uint64_t)vsgi_irq << 32) |
             ((uint64_t)priority << 20);
    cmd[0] |= vpe->sgi_config[vsgi_irq].enabled ? GITS_ENABLE_BIT : 0;
    cmd[0] |= vpe->sgi_config[vsgi_irq].group ? GITS_GROUP_BIT : 0;
    cmd[0] |= clear ?  GITS_CLEAR_BIT : 0;
    cmd[1] = (uint64_t)vpeid << 32;
    cmd[2] = 0x00;
    cmd[3] = 0x00;

    // printk("BEFORE:\n");
    // print_pend_1k(vpe);
    ret = its_send_command(its, cmd);
    if ( ret )
        return ret;

    ret = its_send_cmd_vsync(its, vpeid);
    // printk("AFTER:\n");
    // print_pend_1k(vpe);
    
    return ret;
}

int vgic_v4_configure_vcpu_sgi(struct vcpu *v)
{
    unsigned int i;
    struct host_its *hw_its;
    struct its_vpe *vpe = &v->arch.vgic.its_vpe;
    int ret;

    hw_its = find_4_1_its();
    if ( !hw_its )
        return -ENOENT;

    for ( i = 0; i < VGIC_NR_SGIS; i++ )
    {
        vpe->sgi_config[i].enabled = false;
        vpe->sgi_config[i].group = true;
        vpe->sgi_config[i].priority = 0;

         /* Write out the initial VSGI configuration */
        ret = its_send_cmd_vsgi(hw_its, i, vpe, false);
        if ( ret )
            return ret;
    }

    return 0;
}

int its_sgi_mask_irq(struct vcpu *v, unsigned int irq)
{
    int ret;
    struct its_vpe *vpe = &v->arch.vgic.its_vpe;
    struct host_its *hw_its;

    ASSERT(irq < 16);

    hw_its = find_4_1_its();
    if ( !hw_its )
        return -EINVAL;

    vpe->sgi_config[irq].enabled = false;
    ret = its_send_cmd_vsgi(hw_its, irq, vpe, false);
    if ( ret )
        return ret;

    return 0;
}

int its_sgi_unmask_irq(struct vcpu *v, unsigned int irq)
{
    int ret;
    struct its_vpe *vpe = &v->arch.vgic.its_vpe;
    struct host_its *hw_its;

    ASSERT(irq < 16);

    hw_its = find_4_1_its();
    if ( !hw_its )
        return -EINVAL;

    vpe->sgi_config[irq].enabled = true;
    ret = its_send_cmd_vsgi(hw_its, irq, vpe, false);
    if ( ret )
        return ret;

    return 0;
}

int its_sgi_get_pending_state(struct vcpu *v, uint32_t *ipending)
{
    struct its_vpe *vpe = &v->arch.vgic.its_vpe;
    void __iomem *base;
    uint32_t status;
    uint32_t count = 1000000;    /* 1s! */
    unsigned int cpu;
    unsigned long flags;

    /*
     * We can race against the following events:
     *
     * - Concurrent vPE affinity change: we must make sure it cannot
     *   happen, or we'll talk to the wrong redistributor. This is
     *   identical to what happens with vLPIs.
     */
    cpu = vpe_to_cpuid_lock(vpe, &flags);
    base = gic_data_rdist_vlpi_base(cpu);
    writel_relaxed(vpe->vpe_id, base + GICR_VSGIR);
    do {
        status = readl_relaxed(base + GICR_VSGIPENDR);
        /* Wait until BUSY is cleared */
        if ( !(status & GICR_VSGIPENDR_BUSY) )
            goto out;

        count--;
        if ( !count )
        {
            printk(XENLOG_G_ERR "%pv: unable to get SGI pending status\n", v);
            goto out;
        }
        cpu_relax();
        udelay(1);
    } while ( count );

 out:
    vpe_to_cpuid_unlock(vpe, &flags);

    if ( !count )
        return -ENXIO;

    *ipending = status & GICR_VSGIPENDR_PENDING;

    return 0;
}

int its_sgi_set_pending_state(struct vcpu *v, unsigned int vsgi, bool state)
{
    struct its_vpe *vpe = &v->arch.vgic.its_vpe;
    struct host_its *its = find_4_1_its();
    // uint32_t pend;
    uint64_t val;
    int ret = 0;

    if ( state )
    {
        // printk(">>>>>VPEID %d VINTID %d\n", vpe->vpe_id, vsgi);
        val = FIELD_PREP(GITS_SGIR_VPEID, vpe->vpe_id);
        val |= FIELD_PREP(GITS_SGIR_VINTID, vsgi);
        // printk("WRITE SGIR BASE vpe %d intid %d val %lx\n", vpe->vpe_id, vsgi, val);
        writeq_relaxed(val, its->sgir_base + GITS_SGIR);

        // udelay(1);
        // val = FIELD_PREP(GITS_SGIR_VPEID, (vpe->vpe_id + 1) % 2);
        // val |= FIELD_PREP(GITS_SGIR_VINTID, vsgi);
        // printk("WRITE SGIR BASE vpe %d intid %d val %lx\n", (vpe->vpe_id + 1) % 2, vsgi, val);
        // writeq_relaxed(val, its->sgir_base + GITS_SGIR);
        // its_sgi_get_pending_state(v, &pend);
        // printk("SGIR IS %lx\n", readq_relaxed(its->sgir_base + GITS_SGIR));
        // printk("PEND IS %x\n", pend);
    }
    else
    {
         /*
          * Clearing the pending bit by emiting a VSGI command with
          * the "clear" bit set
          */
        // printk(">>>>>CLEAR VPEID %d VINTID %d\n", vpe->vpe_id, vsgi);
        ret = its_send_cmd_vsgi(its, vsgi, vpe, true);
    }

    return ret;
}

int its_sgi_prop_update(struct vcpu *v, unsigned int irq, uint8_t priority)
{
    int ret;
    struct its_vpe *vpe = &v->arch.vgic.its_vpe;
    struct host_its *hw_its;

    ASSERT(irq < 16);

    hw_its = find_4_1_its();
    if ( !hw_its )
        return -EINVAL;

    vpe->sgi_config[irq].priority = priority;
    ret = its_send_cmd_vsgi(hw_its, irq, vpe, false);
    if ( ret )
        return ret;

    return 0;
}

static void vgic_v4_sync_sgi_config(struct its_vpe *vpe,
                                    struct pending_irq *pirq)
{
    bool enabled;

    enabled = test_bit(GIC_IRQ_GUEST_ENABLED, &pirq->status);
    vpe->sgi_config[pirq->irq].enabled = enabled;
    /* Always Group-1 interrupt */
    vpe->sgi_config[pirq->irq].group = true;
    vpe->sgi_config[pirq->irq].priority = pirq->priority;
}

/* Transfer from old, software-emulated SGIs to the new, HW-based ones */
static void vgic_v4_enable_vsgis(struct vcpu *vcpu)
{
    struct its_vpe *vpe = &vcpu->arch.vgic.its_vpe;
    unsigned int i;
    struct host_its *hw_its = find_4_1_its();
    unsigned long flags;

    for ( i = 0; i < VGIC_NR_SGIS; i++ )
    {
        struct pending_irq *p = irq_to_pending(vcpu, i);

        spin_lock_irqsave(&vcpu->arch.vgic.lock, flags);

        if ( p->hw )
            goto unlock;

        /*
         * With GICv4.1, every virtual SGI can be directly injected. So
         * let's pretend that they are HW-based interrupts.
         */
        p->hw = true;

        /* Transfer the full pending_irq state to the vPE */
        vgic_v4_sync_sgi_config(vpe, p);
        WARN_ON(its_send_cmd_vsgi(hw_its, i, vpe, false));

        /* Transfer pending state */
        if ( test_bit(GIC_IRQ_GUEST_ENABLED, &p->status) &&
             !list_empty(&p->inflight) )
        {
            /*
             * If IRQ is lr_pending, we could transfer to use ITS MMIO
             * registers to deliver pending state
             */
            if ( !list_empty(&p->lr_queue) )
            {
                list_del_init(&p->lr_queue);
                WARN_ON(its_sgi_set_pending_state(vcpu, i, true));
                clear_bit(GIC_IRQ_GUEST_QUEUED, &p->status);
                list_del_init(&p->inflight);
            }
            else
                gic_raise_inflight_irq(vcpu, i);
        }

    unlock:
        spin_unlock_irqrestore(&vcpu->arch.vgic.lock, flags);
    }
}

/* Transfer from new, HW-based SGIS to the old, software-emulated ones */
static void vgic_v4_disable_vsgis(struct vcpu *vcpu)
{
    struct its_vpe *vpe = &vcpu->arch.vgic.its_vpe;
    unsigned int i;
    struct host_its *hw_its = find_4_1_its();
    uint32_t ipending;
    unsigned long flags;
    int ret;

    ret = its_sgi_get_pending_state(vcpu, &ipending);
    WARN_ON(ret);

    for ( i = 0; i < VGIC_NR_SGIS; i++ )
    {
        struct pending_irq *p = irq_to_pending(vcpu, i);

        spin_lock_irqsave(&vcpu->arch.vgic.lock, flags);

        if ( !p->hw )
            goto unlock;

        p->hw = false;

        /* Transfer pending state */
        if ( ipending & (1U << i) )
             vgic_inject_irq(vcpu->domain, vcpu, i, true);

        /*
         * Disable HW-based VSGI and clearing the pending bit.
         * For VSGI command:
         * To change the configuration, CLEAR must be set to false,
         * leaving the pending bit unchanged.
         * To clear the pending bit, CLEAR must be set to true, leaving
         * the configuration unchanged.
         * You just can't do both at once, hence the two commands below.
         */
        vpe->sgi_config[i].enabled = false;
        WARN_ON(its_send_cmd_vsgi(hw_its, i, vpe, false));
        WARN_ON(its_send_cmd_vsgi(hw_its, i, vpe, true));

    unlock:
        spin_unlock_irqrestore(&vcpu->arch.vgic.lock, flags);
    }
}

void vgic_v4_configure_vsgis(struct domain *d)
{
    struct vcpu *v;

    WARN_ON(domain_pause_except_self(d));

    for_each_vcpu ( d, v )
    {
        if ( d->arch.vgic.nassgireq )
            vgic_v4_enable_vsgis(v);
        else
            vgic_v4_disable_vsgis(v);
    }

    domain_unpause_except_self(d);
}

bool guest_support_nassgi(struct domain *d)
{
    return d->arch.vgic.nassgireq;
}

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
