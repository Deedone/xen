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

#include <xen/delay.h>
#include <xen/err.h>
#include <xen/errno.h>
#include <xen/init.h>
#include <xen/lib.h>
#include <xen/mm.h>
#include <xen/sched.h>
#include <xen/spinlock.h>
#include <asm/gic_v3_defs.h>
#include <asm/gic_v3_its.h>
#include <asm/gic_v4_its.h>
#include <asm/mm.h>
#include <asm/vgic.h>


/*
 * VPE ID is at most 16 bits.
 * Using a bitmap here limits us to 65536 concurrent VPEs.
 */
#define INVALID_VPEID (~0U)
#define GICR_VPENDBASER_DIRTY_POLL_TIMEOUT_US 100000U
#define GICR_SYNCR_POLL_TIMEOUT_US 100000U

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

bool gicv4_its_doorbell_requires_mask(void)
{
    return !gic_has_v4_1_extension();
}

/* Per-redistributor GICv4.1 VPE table sharing group. */
DEFINE_PER_CPU(cpumask_t *, vpe_table_mask);
#define vpe_table_mask_cpu(cpu) (per_cpu(vpe_table_mask, cpu))
#define VPE_TABLE_MASK          (this_cpu(vpe_table_mask))
static cpumask_t *vpe_table_mask_pool;

static void its_make_vpe_4_1_resident(struct its_vpe *vpe, unsigned int cpu);
static bool its_make_vpe_4_1_non_resident(struct its_vpe *vpe,
                                          unsigned int cpu, bool req_db);
static int vpe_to_cpuid_lock(struct its_vpe *vpe, unsigned long *flags);
static void vpe_to_cpuid_unlock(struct its_vpe *vpe, unsigned long *flags);
static void encode_vmovp_v4_1(uint64_t *cmd, uint32_t default_db_pintid);

void __init gicv4_its_vpeid_allocator_init(void)
{
    /* Allocate space for vpeid_mask based on MAX_VPEID */
    vpeid_mask = xzalloc_array(unsigned long, BITS_TO_LONGS(MAX_VPEID));

    if ( !vpeid_mask )
        panic("Could not allocate VPEID bitmap space\n");

    if ( gic_has_v4_1_extension() )
    {
        vpe_table_mask_pool = xzalloc_array(cpumask_t, nr_cpu_ids);
        if ( !vpe_table_mask_pool )
            panic("Could not allocate VPE table sharing masks\n");
    }
}

static cpumask_t *alloc_vpe_table_mask(void)
{
    cpumask_t *mask;

    /*
     * Secondary CPUs initialise their redistributor before local IRQs are
     * enabled.  Use the preallocated pool there; the boot CPU may reach this
     * path before gicv4_its_vpeid_allocator_init().
     */
    if ( vpe_table_mask_pool )
        mask = &vpe_table_mask_pool[smp_processor_id()];
    else
        mask = xzalloc(cpumask_t);

    if ( mask )
        cpumask_clear(mask);

    return mask;
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

static int allocate_vpe_l2_table(unsigned int cpu, uint32_t id)
{
    void __iomem *vlpi_base = gic_data_rdist_vlpi_base(cpu);
    void __iomem *rdbase = per_cpu(rbase, cpu);
    unsigned int psz, esz, idx, npg, gpsz;
    uint64_t val;
    void *buffer;
    __le64 *table;
    paddr_t pa;

    if ( !rdbase )
        return 0;

    val = gits_read_vpropbaser(vlpi_base + GICR_VPROPBASER);
    if ( !(val & GICR_VPROPBASER_4_1_VALID) )
        return -ENODEV;

    gpsz = MASK_EXTR(val, GICR_VPROPBASER_4_1_PAGE_SIZE);
    esz = MASK_EXTR(val, GICR_VPROPBASER_4_1_ENTRY_SIZE) + 1;
    npg = MASK_EXTR(val, GICR_VPROPBASER_4_1_SIZE) + 1;

    switch ( gpsz )
    {
    case GIC_PAGE_SIZE_4K:
        psz = SZ_4K;
        break;
    case GIC_PAGE_SIZE_16K:
        psz = SZ_16K;
        break;
    case GIC_PAGE_SIZE_64K:
        psz = SZ_64K;
        break;
    default:
        return -EINVAL;
    }

    if ( !(val & GICR_VPROPBASER_4_1_INDIRECT) )
        return (id < (npg * psz / (esz * SZ_8))) ? 0 : -ERANGE;

    idx = id / (psz / (esz * SZ_8));
    if ( idx >= (npg * psz / GITS_LVL1_ENTRY_SIZE) )
        return -ERANGE;

    pa = MASK_EXTR(val, GICR_VPROPBASER_4_1_ADDR) << 12;
    table = (__le64 *)maddr_to_virt(pa);

    if ( table[idx] )
        return 0;

    buffer = alloc_xenheap_pages(get_order_from_bytes(psz),
                                 gicv3_its_get_memflags());
    if ( !buffer )
        return -ENOMEM;

    memset(buffer, 0, psz);

    if ( !(val & GICR_VPROPBASER_SHAREABILITY_MASK) )
        clean_and_invalidate_dcache_va_range(buffer, psz);

    table[idx] = cpu_to_le64(virt_to_maddr(buffer) | GITS_BASER_VALID);

    if ( !(val & GICR_VPROPBASER_SHAREABILITY_MASK) )
        clean_and_invalidate_dcache_va_range(table + idx,
                                             GITS_LVL1_ENTRY_SIZE);

    dsb(sy);

    return 0;
}

static int its_alloc_vpe_entry(uint32_t vpe_id)
{
    struct host_its *hw_its;
    unsigned int cpu;
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

    if ( !gic_has_v4_1_extension() )
        return 0;

    for_each_possible_cpu ( cpu )
    {
        ret = allocate_vpe_l2_table(cpu, vpe_id);
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
    uint64_t vpt_addr, vprop_addr;
    int ret;

    cmd[0] = GITS_CMD_VMAPP;
    cmd[1] = (uint64_t)vpeid << 32;
    cmd[2] = valid ? GITS_VALID_BIT : 0;
    cmd[3] = 0;

    if ( its->is_v4_1 )
    {
        int count = atomic_read(&vpe->vmapp_count);

        if ( !valid && !count )
            return -EINVAL;

        if ( valid ? count == 0 : count == 1 )
            cmd[0] |= GITS_ALLOC_BIT;
    }

    /* Unmap command */
    if ( !valid )
        goto out;

    /* Target redistributor */
    cmd[2] |= encode_rdbase(its, vpe->col_idx, 0x0);
    vpt_addr = virt_to_maddr(vpe->vpendtable);
    cmd[3] = (vpt_addr & GENMASK(51, 16)) |
             ((HOST_LPIS_NRBITS - 1) & GENMASK(4, 0));

    if ( !its->is_v4_1 )
        goto out;

    vprop_addr = virt_to_maddr(vpe->its_vm->vproptable);
    cmd[0] |= vprop_addr & GENMASK(51, 16);

    /* The VPT is not guaranteed to be empty, so never advertise PTZ. */
    cmd[1] |= (uint64_t)vpe->vpe_db_lpi;

 out:
    ret = its_send_command(its, cmd);
    if ( ret )
        return ret;

    if ( its->is_v4_1 )
    {
        if ( valid )
            atomic_inc(&vpe->vmapp_count);
        else
            (void)atomic_dec_return(&vpe->vmapp_count);
    }

    return 0;
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

static int gicv4_vpe_db_proxy_unmap_locked(struct its_vpe *vpe)
{
    int ret;

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
    if ( gic_has_v4_1_extension() )
        return;

    if ( !gic_support_directLPI() )
    {
        unsigned long flags;
        int ret;

        spin_lock_irqsave(&vpe_proxy.lock, flags);
        ret = gicv4_vpe_db_proxy_unmap_locked(vpe);
        spin_unlock_irqrestore(&vpe_proxy.lock, flags);

        if ( ret )
            printk(XENLOG_WARNING
                   "ITS: failed to unmap GICv4 VPE proxy event: %d\n", ret);
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
    unsigned int eventid;
    int ret;

    if ( gic_has_v4_1_extension() )
        return 0;

    /* Already mapped? */
    if ( vpe->vpe_proxy_event != -1 )
        return 0;

    /* This slot was already allocated. Kick the other VPE out. */
    eventid = vpe_proxy.next_victim;

    if ( vpe_proxy.vpes[eventid] )
    {
        struct its_vpe *old_vpe = vpe_proxy.vpes[eventid];

        ret = gicv4_vpe_db_proxy_unmap_locked(old_vpe);
        if ( ret )
            return ret;
    }

    ret = its_send_cmd_mapti(vpe_proxy.dev->hw_its, vpe_proxy.dev->host_devid,
                             eventid, vpe->vpe_db_lpi, vpe->col_idx);
    if ( ret )
        return ret;

    /* Commit the software state only after the ITS accepted the mapping. */
    vpe_proxy.vpes[eventid] = vpe;
    vpe->vpe_proxy_event = eventid;
    vpe_proxy.next_victim = (eventid + 1) % vpe_proxy.dev->eventids;

    return 0;
}

int __init gicv4_init_vpe_proxy(void)
{
    struct host_its *hw_its;
    uint32_t devid;

    if ( gic_support_directLPI() )
    {
        printk("ITS: Using DirectLPI for GICv4 VPE invalidation\n");
        return 0;
    }

    /*
     * The proxy invalidation path uses a fake device with plain ITS
     * MAPTI/INV/MOVI commands, so any host ITS is sufficient here.
     */
    hw_its = list_first_entry_or_null(&host_its_list, struct host_its, entry);
    if ( !hw_its )
    {
        printk(XENLOG_ERR "ITS: No host ITS available for VPE proxy device\n");
        return -ENODEV;
    }

    vpe_proxy.vpes = xzalloc_array(struct its_vpe *, nr_cpu_ids);
    if ( !vpe_proxy.vpes )
    {
        printk(XENLOG_ERR "ITS: Can't allocate GICv4 VPE proxy device array\n");
        return -ENOMEM;
    }

    /* Use the last possible DevID */
    devid = BIT(hw_its->devid_bits, UL) - 1;
    vpe_proxy.dev = its_create_device(hw_its, devid, nr_cpu_ids);
    if ( IS_ERR(vpe_proxy.dev) )
    {
        int ret = PTR_ERR(vpe_proxy.dev);

        printk(XENLOG_ERR "ITS: Can't allocate GICv4 VPE proxy device\n");
        XFREE(vpe_proxy.vpes);
        vpe_proxy.dev = NULL;
        return ret;
    }

    spin_lock_init(&vpe_proxy.lock);
    vpe_proxy.next_victim = 0;
    printk(XENLOG_INFO
           "ITS: Allocated DevID %u as GICv4 VPE proxy device\n", devid);

    return 0;
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

static void encode_vmovp_v4_1(uint64_t *cmd, uint32_t default_db_pintid)
{
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

        if ( !hw_its->has_vlpis )
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

/* GICR_SYNCR.Busy == 1 until the invalidation completes. */
static int wait_for_syncr(void __iomem *rdbase, const char *what)
{
    unsigned int timeout = GICR_SYNCR_POLL_TIMEOUT_US;

    while ( readl_relaxed(rdbase + GICR_SYNCR) & 1 )
    {
        if ( !timeout-- )
        {
            printk(XENLOG_WARNING
                   "GICv4: timeout waiting for GICR_SYNCR.Busy to clear during %s\n",
                   what);
            return -ETIMEDOUT;
        }

        cpu_relax();
        udelay(1);
    }

    return 0;
}

static struct host_its *find_4_1_its(void)
{
    struct host_its *its;

    list_for_each_entry(its, &host_its_list, entry)
    {
        if ( its->is_v4_1 )
            return its;
    }

    return NULL;
}

int direct_lpi_inv(struct its_device *dev, uint32_t eventid,
                   uint32_t db_lpi, unsigned int cpu)
{
    void __iomem *rdbase;
    uint64_t val;
    struct its_vpe *vpe = NULL;
    unsigned long flags = 0;
    int ret;

    if ( dev )
    {
        struct its_vlpi_map *map = &dev->event_map.vlpi_maps[eventid];

        WARN_ON(!gic_has_v4_1_extension());

        vpe = map->vm->vpes[map->vpe_idx];
        cpu = vpe_to_cpuid_lock(vpe, &flags);

        val = GICR_INVLPIR_V;
        val |= MASK_INSR(vpe->vpe_id, GICR_INVLPIR_VPEID);
        val |= MASK_INSR(map->vintid, GICR_INVLPIR_INTID);
    }
    else
    {
        /* Register-based LPI invalidation for DB on GICv4.0. */
        val = MASK_INSR(db_lpi, GICR_INVLPIR_INTID);
    }

    rdbase = per_cpu(rbase, cpu);

    ret = wait_for_syncr(rdbase, "INVLPIR pre-check");
    if ( ret )
    {
        if ( vpe )
            vpe_to_cpuid_unlock(vpe, &flags);
        return ret;
    }

    writeq_relaxed(val, rdbase + GICR_INVLPIR);

    ret = wait_for_syncr(rdbase, "INVLPIR");

    if ( vpe )
        vpe_to_cpuid_unlock(vpe, &flags);

    return ret;
}

static int its_vpe_send_4_1_inv_db(struct host_its *its, struct its_vpe *vpe)
{
    uint64_t cmd[4];
    uint16_t vpeid = vpe->vpe_id;
    int ret;

    cmd[0] = GITS_CMD_INVDB;
    cmd[1] = (uint64_t)vpeid << 32;
    cmd[2] = 0;
    cmd[3] = 0;

    ret = its_send_command(its, cmd);
    if ( ret )
        return ret;

    return its_send_cmd_vsync(its, vpeid);
}

static void its_vpe_inv_db(struct its_vpe *vpe)
{
    struct host_its *its = find_4_1_its();

    if ( its )
    {
        int ret = its_vpe_send_4_1_inv_db(its, vpe);

        if ( ret )
            printk(XENLOG_WARNING
                   "ITS: failed to invalidate GICv4.1 VPE doorbell: %d\n",
                   ret);
    }
    else if ( gic_support_directLPI() )
    {
        unsigned long flags;
        unsigned int cpu;
        int ret;

        /*
         * Serialize register-based invalidation against VPE moves. The
         * architecture makes overlapping INVLPIR and ITS commands that retarget
         * the same doorbell CONSTRAINED UNPREDICTABLE, and vpe->col_idx is also
         * updated under this lock.
         */
        cpu = vpe_to_cpuid_lock(vpe, &flags);

        /* Target the redistributor this VPE is currently known on. */
        ret = direct_lpi_inv(NULL, 0, vpe->vpe_db_lpi, cpu);
        vpe_to_cpuid_unlock(vpe, &flags);

        if ( ret )
            printk(XENLOG_WARNING
                   "ITS: failed to invalidate GICv4 VPE doorbell: %d\n",
                   ret);
    }
    else
    {
        struct its_device *dev = vpe_proxy.dev;
        unsigned long flags;
        int ret;

        spin_lock_irqsave(&vpe_proxy.lock, flags);
        ret = gicv4_vpe_db_proxy_map_locked(vpe);
        if ( !ret )
            ret = its_send_cmd_inv(dev->hw_its, dev->host_devid,
                                   vpe->vpe_proxy_event);
        spin_unlock_irqrestore(&vpe_proxy.lock, flags);

        if ( ret )
            printk(XENLOG_WARNING
                   "ITS: failed to invalidate GICv4 VPE doorbell mapping: %d\n",
                   ret);
    }
}

static void its_vpe_set_db_enabled(struct its_vpe *vpe, bool enable)
{
    lpi_write_config(lpi_host_proptable(), vpe->vpe_db_lpi,
                     enable ? 0 : LPI_PROP_ENABLED,
                     enable ? LPI_PROP_ENABLED : 0);
    its_vpe_inv_db(vpe);
}

void its_vpe_mask_db(struct its_vpe *vpe)
{
    its_vpe_set_db_enabled(vpe, false);
}

static void its_vpe_unmask_db(struct its_vpe *vpe)
{
    its_vpe_set_db_enabled(vpe, true);
}

static void its_vpe_teardown(struct its_vpe *vpe)
{
    struct host_its *hw_its;

    if ( !vpe )
        return;

    gicv4_vpe_db_proxy_unmap(vpe);

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
    unsigned int nr_db_lpis, nr_chunks, i = 0;
    uint32_t *db_lpi_bases;
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

    /* Xen assumes all host ITS instances agree on these attributes. */
    d->arch.vgic.its_vm->vpropbaser =
        virt_to_maddr(d->arch.vgic.its_vm->vproptable) & GENMASK(51, 12);
    d->arch.vgic.its_vm->vpropbaser |=
        gicv3_its_get_cacheability() <<
        GICR_VPROPBASER_INNER_CACHEABILITY_SHIFT;
    d->arch.vgic.its_vm->vpropbaser |=
        gicv3_its_get_shareability() << GICR_VPROPBASER_SHAREABILITY_SHIFT;
    d->arch.vgic.its_vm->vpropbaser |=
        GIC_BASER_CACHE_SameAsInner << GICR_VPROPBASER_OUTER_CACHEABILITY_SHIFT;
    d->arch.vgic.its_vm->vpropbaser |=
        (HOST_LPIS_NRBITS - 1) & GICR_VPROPBASER_IDBITS_MASK;

    return 0;

 fail_db:
    while ( i != 0 )
        gicv3_free_host_lpi_block(db_lpi_bases[--i]);
    xfree(db_lpi_bases);
 fail_db_bases:
    lpi_free_vproptable(d->arch.vgic.its_vm->vproptable);
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

    if ( its_vm->db_lpi_bases )
    {
        int nr_chunks = DIV_ROUND_UP(its_vm->nr_db_lpis, LPI_BLOCK);

        while ( --nr_chunks >= 0 )
            gicv3_free_host_lpi_block(its_vm->db_lpi_bases[nr_chunks]);
        xfree(its_vm->db_lpi_bases);
    }

    xfree(its_vm->vpes);
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
    vpe->vpe_db_lpi = its_vm->db_lpi_bases[vcpuid / LPI_BLOCK] +
                      (vcpuid % LPI_BLOCK);

    ret = its_vpe_init(vpe);
    if ( ret )
    {
        xfree(vpe);
        return ret;
    }

    vcpu->arch.vgic.its_vpe = vpe;
    its_vm->vpes[vcpuid] = vpe;

    /*
     * Sometimes a VLPI gets mapped before the associated VPE becomes
     * resident, so enable the doorbell as soon as initialization succeeds.
     */
    gicv3_lpi_update_host_entry(vpe->vpe_db_lpi, vcpu->domain->domain_id,
                                INVALID_LPI, true, vcpu->vcpu_id);
    its_vpe_unmask_db(vpe);

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
    bool db_enabled = map->db_enabled;

    if ( its->is_v4_1 )
        db_enabled = false;

    if ( db_enabled )
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
    bool db_enabled = map->db_enabled;

    if ( its->is_v4_1 )
        db_enabled = false;

    if ( db_enabled )
        db_pintid = vpe->vpe_db_lpi;
    else
        db_pintid = INVALID_DBLPI;

    cmd[0] = GITS_CMD_VMOVI | ((uint64_t)deviceid << 32);
    cmd[1] = eventid | ((uint64_t)vpeid << 32);
    cmd[2] = (db_enabled ? 1UL : 0UL) | ((uint64_t)db_pintid << 32);
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
    struct its_vlpi_map *live_map;
    struct its_vlpi_map new_map;
    int ret;

    if ( !map )
        return -EINVAL;

    dev = map->dev;

    if ( !dev->event_map.vm )
        return -EINVAL;

    new_map = *map;
    new_map.vpe_idx = vcpu->vcpu_id;

    ret = gicv4_its_vlpi_map(&new_map);

    spin_lock(&dev->event_map.vlpi_lock);
    live_map = dev->event_map.vlpi_maps ?
               &dev->event_map.vlpi_maps[new_map.eventid] : NULL;
    if ( live_map && live_map->pirq == pirq &&
         live_map->eventid == new_map.eventid &&
         live_map->vpe_idx == new_map.vpe_idx )
        *map = new_map;
    spin_unlock(&dev->event_map.vlpi_lock);

    return ret;
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

static int its_vpe_invall(struct its_vpe *vpe)
{
    unsigned long flags;
    struct host_its *its;
    int ret = 0;

    (void)vpe_to_cpuid_lock(vpe, &flags);

    list_for_each_entry(its, &host_its_list, entry)
    {
        if ( !its->has_vlpis )
            continue;

        /*
         * Sending VINVALL to one v4 ITS is enough, as the command targets
         * the redistributors that cache the vPE state.
         */
        ret = its_send_cmd_vinvall(its, vpe);
        if ( ret )
            break;

        ret = its_send_cmd_vsync(its, vpe->vpe_id);
        if ( ret )
            break;

        ret = gicv3_its_wait_commands(its);
        break;
    }

    vpe_to_cpuid_unlock(vpe, &flags);

    return ret;
}

static int its_vpe_4_1_invall(struct its_vpe *vpe)
{
    void __iomem *rdbase;
    unsigned long flags;
    unsigned int cpu;
    uint64_t val;
    int ret;

    val = GICR_INVALLR_V;
    val |= MASK_INSR(vpe->vpe_id, GICR_INVALLR_VPEID);

    cpu = vpe_to_cpuid_lock(vpe, &flags);
    rdbase = per_cpu(rbase, cpu);

    ret = wait_for_syncr(rdbase, "INVALLR pre-check");
    if ( ret )
        goto out;

    writeq_relaxed(val, rdbase + GICR_INVALLR);
    ret = wait_for_syncr(rdbase, "INVALLR");

 out:
    vpe_to_cpuid_unlock(vpe, &flags);
    return ret;
}

int gicv4_its_handle_invall(struct domain *d, struct vcpu *vcpu)
{
    struct its_vm *vm = d->arch.vgic.its_vm;
    struct its_vpe *vpe;

    if ( !vm || vcpu->vcpu_id >= vm->nr_vpes )
        return -ENODEV;

    vpe = vm->vpes[vcpu->vcpu_id];
    if ( !vpe )
        return -ENODEV;

    if ( !gic_has_v4_1_extension() )
        return its_vpe_invall(vpe);

    return its_vpe_4_1_invall(vpe);
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

static bool its_clear_vpend_valid(void __iomem *vlpi_base, uint64_t *val)
{
    unsigned int count = GICR_VPENDBASER_DIRTY_POLL_TIMEOUT_US;

    if ( !gits_clear_vpendbaser_valid(vlpi_base + GICR_VPENDBASER) )
        return false;

    *val = read_vpend_dirty_clean(vlpi_base, count);

    return true;
}

static bool its_clear_vpend_valid_bits(void __iomem *vlpi_base, uint64_t clr,
                                       uint64_t set, uint64_t *val)
{
    void __iomem *vpendbaser = vlpi_base + GICR_VPENDBASER;
    unsigned int count = GICR_VPENDBASER_DIRTY_POLL_TIMEOUT_US;
    uint64_t tmp;

    tmp = gits_read_vpendbaser(vpendbaser);
    if ( tmp & GICR_VPENDBASER_Valid )
    {
        tmp &= ~clr;
        tmp |= set;
        tmp &= ~GICR_VPENDBASER_Valid;
        writeq_relaxed(tmp, vpendbaser);

        do {
            if ( !count-- )
            {
                printk(XENLOG_WARNING
                       "GICv4: timeout clearing GICR_VPENDBASER.Valid\n");
                return false;
            }

            udelay(1);
            tmp = gits_read_vpendbaser(vpendbaser);
        } while ( tmp & GICR_VPENDBASER_Valid );
    }

    *val = read_vpend_dirty_clean(vlpi_base, count);

    return true;
}

static void its_make_vpe_resident(struct its_vpe *vpe, unsigned int cpu)
{
    void __iomem *vlpi_base = gic_data_rdist_vlpi_base(cpu);
    uint64_t val;

    /* Switch in this VM's virtual property table. */
    gits_write_vpropbaser(vpe->its_vm->vpropbaser,
                          vlpi_base + GICR_VPROPBASER);

    /* Switch in this VCPU's VPT. */
    val  = virt_to_maddr(vpe->vpendtable) & GENMASK(51, 16);
    val |= gicv3_its_get_cacheability() <<
           GICR_VPENDBASER_INNER_CACHEABILITY_SHIFT;
    val |= gicv3_its_get_shareability() << GICR_VPENDBASER_SHAREABILITY_SHIFT;
    val |= GIC_BASER_CACHE_SameAsInner <<
           GICR_VPENDBASER_OUTER_CACHEABILITY_SHIFT;
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

static bool its_make_vpe_non_resident(struct its_vpe *vpe, unsigned int cpu)
{
    void __iomem *vlpi_base = gic_data_rdist_vlpi_base(cpu);
    uint64_t val;

    if ( !its_clear_vpend_valid(vlpi_base, &val) )
        return false;

    vpe->idai = val & GICR_VPENDBASER_IDAI;
    write_atomic(&vpe->pending_last, val & GICR_VPENDBASER_PendingLast);

    return true;
}

static void its_make_vpe_4_1_resident(struct its_vpe *vpe, unsigned int cpu)
{
    void __iomem *vlpi_base = gic_data_rdist_vlpi_base(cpu);
    uint64_t old, val = 0;

    (void)its_clear_vpend_valid_bits(vlpi_base, 0, 0, &old);

    val |= GICR_VPENDBASER_Valid;
    /* PendingLast is RES1 when GICR_VPENDBASER.Valid is written 0->1. */
    val |= GICR_VPENDBASER_PendingLast;
    val |= GICR_VPENDBASER_4_1_VGRP1EN;
    val |= MASK_INSR(vpe->vpe_id, GICR_VPENDBASER_4_1_VPEID);

    gits_write_vpendbaser(val, vlpi_base + GICR_VPENDBASER);
}

static bool its_make_vpe_4_1_non_resident(struct its_vpe *vpe,
                                          unsigned int cpu, bool req_db)
{
    void __iomem *vlpi_base = gic_data_rdist_vlpi_base(cpu);
    uint64_t val;
    bool ret;

    if ( req_db )
    {
        ret = its_clear_vpend_valid_bits(vlpi_base,
                                         GICR_VPENDBASER_PendingLast,
                                         GICR_VPENDBASER_4_1_DB, &val);
        if ( ret )
            write_atomic(&vpe->pending_last,
                         !!(val & GICR_VPENDBASER_PendingLast));
    }
    else
    {
        /*
         * We are not arming a default doorbell. Let the implementation
         * report whether pending enabled vLPIs remain instead of forcing
         * PendingLast to 1.
         */
        ret = its_clear_vpend_valid_bits(vlpi_base, GICR_VPENDBASER_4_1_DB,
                                         0, &val);
        if ( ret )
            write_atomic(&vpe->pending_last,
                         !!(val & GICR_VPENDBASER_PendingLast));
    }

    return ret;
}

static uint64_t inherit_vpe_l1_table_from_rd(void)
{
    uint32_t aff;
    uint64_t val;
    unsigned int cpu;

    val = readq_relaxed(GICD_RDIST_BASE + GICR_TYPER);
    aff = compute_common_aff(val);

    for_each_present_cpu ( cpu )
    {
        void __iomem *rbase;

        if ( cpu == smp_processor_id() || !cpu_online(cpu) )
            continue;

        rbase = GICD_RDIST_BASE_CPU(cpu);
        if ( !rbase )
            continue;

        val = readq_relaxed(rbase + GICR_TYPER);
        if ( aff != compute_common_aff(val) )
            continue;

        val = gits_read_vpropbaser(gic_data_rdist_vlpi_base(cpu) +
                                   GICR_VPROPBASER);
        if ( !(val & GICR_VPROPBASER_4_1_VALID) )
            continue;

        VPE_TABLE_MASK = vpe_table_mask_cpu(cpu);
        val &= ~GICR_VPROPBASER_4_1_Z;

        return val;
    }

    return 0;
}

static uint64_t inherit_vpe_l1_table_from_its(void)
{
    struct host_its *its;
    uint64_t val;
    uint32_t aff;

    val = readq_relaxed(GICD_RDIST_BASE + GICR_TYPER);
    aff = compute_common_aff(val);

    list_for_each_entry(its, &host_its_list, entry)
    {
        const struct its_baser *vpe_baser;
        uint64_t typer, baser;
        paddr_t addr;

        if ( !its->is_v4_1 )
            continue;

        typer = readq_relaxed(its->its_base + GITS_TYPER);
        if ( !MASK_EXTR(typer, GITS_TYPER_SVPET) )
            continue;

        if ( aff != compute_its_aff(its) )
            continue;

        vpe_baser = its_get_baser(its, GITS_BASER_TYPE_VCPU);
        if ( !vpe_baser )
            continue;

        baser = vpe_baser->val;
        if ( !(baser & GITS_BASER_VALID) )
            continue;

        val = GICR_VPROPBASER_4_1_VALID;
        if ( baser & GITS_BASER_INDIRECT )
            val |= GICR_VPROPBASER_4_1_INDIRECT;

        val |= MASK_INSR(MASK_EXTR(baser, GITS_BASER_PAGE_SIZE_MASK),
                         GICR_VPROPBASER_4_1_PAGE_SIZE);

        switch ( MASK_EXTR(baser, GITS_BASER_PAGE_SIZE_MASK) )
        {
        case GIC_PAGE_SIZE_64K:
            addr = GITS_BASER_ADDR_48_to_52(baser);
            break;
        default:
            addr = baser & GENMASK(47, 12);
            break;
        }

        val |= MASK_INSR(addr >> 12, GICR_VPROPBASER_4_1_ADDR);
        val |= MASK_INSR(MASK_EXTR(baser, GITS_BASER_SHAREABILITY_MASK),
                         GICR_VPROPBASER_SHAREABILITY_MASK);
        val |= MASK_INSR(
            MASK_EXTR(baser, GITS_BASER_INNER_CACHEABILITY_MASK),
            GICR_VPROPBASER_INNER_CACHEABILITY_MASK);
        val |= MASK_INSR(GITS_BASER_NR_PAGES(baser) - 1,
                         GICR_VPROPBASER_4_1_SIZE);

        return val;
    }

    return 0;
}

int allocate_vpe_l1_table(void)
{
    void __iomem *vlpi_base = gic_data_rdist_vlpi_base(smp_processor_id());
    uint64_t val, gpsz, npg, pa;
    unsigned int psz = SZ_64K;
    unsigned int epp, esz;
    unsigned int order;
    void *buffer;

    if ( !gic_has_v4_1_extension() )
        return 0;

    val = gits_read_vpendbaser(vlpi_base + GICR_VPENDBASER);
    if ( val & GICR_VPENDBASER_Valid )
        writeq_relaxed((val | GICR_VPENDBASER_PendingLast) &
                       ~GICR_VPENDBASER_Valid,
                       vlpi_base + GICR_VPENDBASER);

    val = inherit_vpe_l1_table_from_rd();
    if ( val & GICR_VPROPBASER_4_1_VALID )
        goto out;

    VPE_TABLE_MASK = alloc_vpe_table_mask();
    if ( !VPE_TABLE_MASK )
        return -ENOMEM;

    val = inherit_vpe_l1_table_from_its();
    if ( val & GICR_VPROPBASER_4_1_VALID )
        goto out;

    val = MASK_INSR(GIC_PAGE_SIZE_64K,
                    GICR_VPROPBASER_4_1_PAGE_SIZE);
    gits_write_vpropbaser(val, vlpi_base + GICR_VPROPBASER);
    val = gits_read_vpropbaser(vlpi_base + GICR_VPROPBASER);
    gpsz = MASK_EXTR(val, GICR_VPROPBASER_4_1_PAGE_SIZE);
    esz = MASK_EXTR(val, GICR_VPROPBASER_4_1_ENTRY_SIZE) + 1;

    switch ( gpsz )
    {
    case GIC_PAGE_SIZE_4K:
        psz = SZ_4K;
        break;
    case GIC_PAGE_SIZE_16K:
        psz = SZ_16K;
        break;
    case GIC_PAGE_SIZE_64K:
        psz = SZ_64K;
        break;
    default:
        gpsz = GIC_PAGE_SIZE_4K;
        psz = SZ_4K;
        break;
    }

    val = MASK_INSR(gpsz, GICR_VPROPBASER_4_1_PAGE_SIZE);
    val |= MASK_INSR(esz - 1, GICR_VPROPBASER_4_1_ENTRY_SIZE);

    epp = psz / (esz * SZ_8);
    if ( epp < MAX_VPEID )
    {
        unsigned int nl2;

        val |= GICR_VPROPBASER_4_1_INDIRECT;
        nl2 = DIV_ROUND_UP(MAX_VPEID, epp);
        npg = DIV_ROUND_UP(nl2 * SZ_8, psz);
    }
    else
        npg = 1;

    val |= MASK_INSR(npg - 1, GICR_VPROPBASER_4_1_SIZE);

    order = get_order_from_bytes(npg * psz);
    buffer = alloc_xenheap_pages(order, gicv3_its_get_memflags());
    if ( !buffer )
        return -ENOMEM;

    memset(buffer, 0, PAGE_SIZE << order);
    if ( gicv3_its_get_cacheability() <= GIC_BASER_CACHE_nC )
        clean_and_invalidate_dcache_va_range(buffer, npg * psz);

    pa = virt_to_maddr(buffer);
    val |= MASK_INSR(pa >> 12, GICR_VPROPBASER_4_1_ADDR);
    val |= gicv3_its_get_cacheability() <<
           GICR_VPROPBASER_INNER_CACHEABILITY_SHIFT;
    val |= gicv3_its_get_shareability() <<
           GICR_VPROPBASER_SHAREABILITY_SHIFT;
    val |= GICR_VPROPBASER_4_1_Z;
    val |= GICR_VPROPBASER_4_1_VALID;

 out:
    gits_write_vpropbaser(val, vlpi_base + GICR_VPROPBASER);

    if ( VPE_TABLE_MASK )
        cpumask_set_cpu(smp_processor_id(), VPE_TABLE_MASK);

    return 0;
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

static int gicv4_vpe_db_proxy_move(struct its_vpe *vpe, unsigned int from,
                                   unsigned int to)
{
    unsigned long flags;
    int ret;

    if ( gic_support_directLPI() )
    {
        void __iomem *rdbase;

        rdbase = per_cpu(rbase, from);

        ret = wait_for_syncr(rdbase, "CLRLPIR pre-check");
        if ( ret )
        {
            /*
             * VMOVP already retargeted the VPE. Clearing any stale pending
             * state on the old redistributor is best-effort only, so keep the
             * move successful and rely on wait_for_syncr() for timeout logging.
             */
            return 0;
        }

        /* Clear potential pending state on the old redistributor */
        writeq_relaxed(vpe->vpe_db_lpi, rdbase + GICR_CLRLPIR);

        (void)wait_for_syncr(rdbase, "CLRLPIR");
        return 0;
    }

    spin_lock_irqsave(&vpe_proxy.lock, flags);

    ret = gicv4_vpe_db_proxy_map_locked(vpe);
    if ( ret )
        goto out;

    /* MOVI instructs the appropriate Redistributor to move the pending state */
    ret = its_send_cmd_movi(vpe_proxy.dev->hw_its, vpe_proxy.dev->host_devid,
                            vpe->vpe_proxy_event, to);
    if ( ret )
        goto out;

    /*
     * ARM spec says that If, after using MOVI to move an interrupt from
     * collection A to collection B, software moves the same interrupt again
     * from collection B to collection C, a SYNC command must be used before
     * the second MOVI for the Redistributor associated with collection A to
     * ensure correct behavior.
     * So each time we issue VMOVI, we VSYNC the old VPE for good measure.
     */
    ret = its_send_cmd_sync(vpe_proxy.dev->hw_its, from);

 out:
    spin_unlock_irqrestore(&vpe_proxy.lock, flags);
    return ret;
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
    {
        /*
         * VMOVP did not commit, so keep the software view aligned with
         * the old redistributor target.
         */
        vpe->col_idx = from;
        goto out;
    }
    ret = gicv4_vpe_db_proxy_move(vpe, from, to);

 out:
    vpe_to_cpuid_unlock(vpe, &flags);
    return ret;
}

void vgic_v4_load(struct vcpu *vcpu)
{
    struct its_vpe *vpe = vcpu->arch.vgic.its_vpe;

    if ( !vpe )
        return;

    if ( vpe->resident )
        return;

    /*
     * Before making the VPE resident, make sure the redistributor
     * corresponding to our current CPU expects us here
     */
    WARN_ON(gicv4_vpe_set_affinity(vcpu));
    if ( gic_has_v4_1_extension() )
    {
        its_make_vpe_4_1_resident(vpe, vcpu->processor);
        vpe->resident = true;
        return;
    }

    its_vpe_mask_db(vpe);
    its_make_vpe_resident(vpe, vcpu->processor);
    vpe->resident = true;
}

void vgic_v4_put(struct vcpu *vcpu, bool need_db)
{
    struct its_vpe *vpe = vcpu->arch.vgic.its_vpe;

    if ( !vpe )
        return;

    if ( !vpe->resident )
        return;

    if ( gic_has_v4_1_extension() )
    {
        if ( !its_make_vpe_4_1_non_resident(vpe, vcpu->processor, need_db) )
            return;

        vpe->resident = false;
        return;
    }

    if ( !its_make_vpe_non_resident(vpe, vcpu->processor) )
        return;

    if ( need_db )
        /* Enable the doorbell, as the guest is going to block */
        its_vpe_unmask_db(vpe);
    vpe->resident = false;
}

static int its_vlpi_set_doorbell(struct its_vlpi_map *map, bool enable)
{
    struct its_vlpi_map new_map;
    int ret;

    if ( map->db_enabled == enable )
        return 0;

    new_map = *map;
    new_map.db_enabled = enable;

    /*
     * Ideally, we'd issue a VMAPTI to set the doorbell to its LPI
     * value or to 1023, depending on the enable bit. But that
     * would be issuing a mapping for an /existing/ DevID+EventID
     * pair, which is UNPREDICTABLE. Instead, let's issue a VMOVI
     * to the /same/ vPE, using this opportunity to adjust the doorbell.
     */
    ret = its_send_cmd_vmovi(map->dev->hw_its, &new_map);
    if ( ret )
        return ret;

    map->db_enabled = enable;

    return 0;
}

int its_vlpi_prop_update(struct pending_irq *pirq, uint8_t property,
                         bool needs_inv)
{
    struct its_vlpi_map *map;
    int ret;

    if ( !pirq->vlpi_map )
        return -EINVAL;

    map = pirq->vlpi_map;

    /* Update the vproptable. */
    ret = lpi_write_config(map->vm->vproptable, pirq->irq, 0xff, property);
    if ( ret )
        return ret;

    if ( needs_inv )
    {
        /* The CPU argument is unused for forwarded LPIs. */
        ret = its_inv_lpi(map->dev->hw_its, map->dev, map->eventid, 0);
        if ( ret )
            return ret;
    }

    return its_vlpi_set_doorbell(map, property & LPI_PROP_ENABLED);
}
