/*
 * VGICv3 MMIO handling functions
 * Imported from Linux ("new" KVM VGIC) and heavily adapted to Xen.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <xen/bitops.h>
#include <xen/sched.h>
#include <xen/sizes.h>
#include <asm/new_vgic.h>
#include <asm/gic_v3_defs.h>
#include <asm/vreg.h>

#include "vgic.h"
#include "vgic-mmio.h"

bool vgic_has_its(struct domain *d)
{
    struct vgic_dist *dist = &d->arch.vgic;

    if ( dist->version != GIC_V3 )
        return false;

    return false;
}

static struct vcpu *mpidr_to_vcpu(struct domain *d, unsigned long mpidr)
{
    struct vcpu *vcpu;

    mpidr &= MPIDR_HWID_MASK;
    for_each_vcpu(d, vcpu)
    {
        if ( mpidr == vcpuid_to_vaffinity(vcpu->vcpu_id) )
            return vcpu;
    }
    return NULL;
}

/* extract @num bytes at @offset bytes offset in data */
unsigned long extract_bytes(uint64_t data, unsigned int offset,
                            unsigned int num)
{
    return (data >> (offset * 8)) & GENMASK_ULL(num * 8 - 1, 0);
}

uint64_t update_64bit_reg(u64 reg, unsigned int offset, unsigned int len,
                          unsigned long val)
{
    int lower = (offset & 4) * 8;
    int upper = lower + 8 * len - 1;

    reg &= ~GENMASK_ULL(upper, lower);
    val &= GENMASK_ULL(len * 8 - 1, 0);

    return reg | ((u64)val << lower);
}

static int match_mpidr(u64 sgi_aff, u16 sgi_cpu_mask, struct vcpu *vcpu)
{
    unsigned long affinity;
    int level0;

    /*
     * Split the current VCPU's MPIDR into affinity level 0 and the
     * rest as this is what we have to compare against.
     */
    affinity = vcpuid_to_vaffinity(vcpu->vcpu_id);
    level0   = MPIDR_AFFINITY_LEVEL(affinity, 0);
    affinity &= ~MPIDR_LEVEL_MASK;

    /* bail out if the upper three levels don't match */
    if ( sgi_aff != affinity )
        return -1;

    /* Is this VCPU's bit set in the mask ? */
    if ( !(sgi_cpu_mask & BIT(level0, ULL)) )
        return -1;

    return level0;
}

#define SGI_AFFINITY_LEVEL(reg, level)                                         \
    ((((reg) & (ICH_SGI_AFFx_MASK << ICH_SGI_AFFINITY_LEVEL(level))) >>        \
      ICH_SGI_AFFINITY_LEVEL(level))                                           \
     << MPIDR_LEVEL_SHIFT(level))

static bool vgic_v3_emulate_sgi1r(struct cpu_user_regs *regs, uint64_t *r,
                                  bool read)
{
    struct domain *d  = current->domain;
    struct vcpu *vcpu = current;
    struct vcpu *c_vcpu;
    u16 target_cpus;
    u64 mpidr;
    int sgi;
    int vcpu_id = vcpu->vcpu_id;
    bool broadcast;
    unsigned long flags;

    if ( read )
    {
        gdprintk(XENLOG_WARNING, "Reading SGI1R_EL1 - WO register\n");
        return false;
    }

    sgi         = (*r >> ICH_SGI_IRQ_SHIFT) & ICH_SGI_IRQ_MASK;
    broadcast   = *r & BIT(ICH_SGI_IRQMODE_SHIFT, ULL);
    target_cpus = (*r & ICH_SGI_TARGETLIST_MASK);

    mpidr  = SGI_AFFINITY_LEVEL(*r, 3);
    mpidr |= SGI_AFFINITY_LEVEL(*r, 2);
    mpidr |= SGI_AFFINITY_LEVEL(*r, 1);

    /*
     * We iterate over all VCPUs to find the MPIDRs matching the request.
     * If we have handled one CPU, we clear its bit to detect early
     * if we are already finished. This avoids iterating through all
     * VCPUs when most of the times we just signal a single VCPU.
     */
    for_each_vcpu(d, c_vcpu)
    {
        struct vgic_irq *irq;

        /* Exit early if we have dealt with all requested CPUs */
        if ( !broadcast && target_cpus == 0 )
            break;

        /* Don't signal the calling VCPU */
        if ( broadcast && c_vcpu->vcpu_id == vcpu_id )
            continue;

        if ( !broadcast )
        {
            int level0;

            level0 = match_mpidr(mpidr, target_cpus, c_vcpu);
            if ( level0 == -1 )
                continue;

            /* remove this matching VCPU from the mask */
            target_cpus &= ~BIT(level0, UL);
        }

        irq = vgic_get_irq(vcpu->domain, c_vcpu, sgi);

        spin_lock_irqsave(&irq->irq_lock, flags);

        if ( !irq->hw )
        {
            irq->pending_latch = true;
            vgic_queue_irq_unlock(vcpu->domain, irq, flags);
        }
        else
        {
            printk(XENLOG_ERR "HW SGIs are not implemented\n");
            BUG();
            spin_unlock_irqrestore(&irq->irq_lock, flags);
        }

        vgic_put_irq(vcpu->domain, irq);
    }

    return true;
}

static bool vgic_v3_emulate_sysreg(struct cpu_user_regs *regs, union hsr hsr)
{
    struct hsr_sysreg sysreg = hsr.sysreg;

    ASSERT(hsr.ec == HSR_EC_SYSREG);

    if ( sysreg.read )
        perfc_incr(vgic_sysreg_reads);
    else
        perfc_incr(vgic_sysreg_writes);

    switch ( hsr.bits & HSR_SYSREG_REGS_MASK )
    {
    case HSR_SYSREG_ICC_SGI1R_EL1:
        return vreg_emulate_sysreg(regs, hsr, vgic_v3_emulate_sgi1r);

    default:
        return false;
    }
}

bool vgic_v3_emulate_reg(struct cpu_user_regs *regs, union hsr hsr)
{
    switch ( hsr.ec )
    {
#ifdef CONFIG_ARM_64
    case HSR_EC_SYSREG:
        return vgic_v3_emulate_sysreg(regs, hsr);
#endif
    case HSR_EC_CP15_64:
        printk(XENLOG_ERR
               "vgic_v3_emulate_reg: HSR_EC_CP15_64 not implemented");
        BUG();
        break;
    default:
        return false;
    }
}

/*
 * The Revision field in the IIDR have the following meanings:
 *
 * Revision 2: Interrupt groups are guest-configurable and signaled using
 *            their configured groups.
 */

static unsigned long vgic_mmio_read_v3_misc(struct vcpu *vcpu, paddr_t addr,
                                            unsigned int len)
{
    struct vgic_dist *vgic = &vcpu->domain->arch.vgic;
    uint32_t value         = 0;

    switch ( addr & 0x0c )
    {
    case GICD_CTLR:
        if ( vgic->enabled )
            value |= GICD_CTLR_ENABLE_G1A;
        value |= GICD_CTLR_ARE_NS | GICD_CTLR_DS;
        break;
    case GICD_TYPER:
        value = vgic->nr_spis + VGIC_NR_PRIVATE_IRQS;
        value = (value >> 5) - 1;
        value |= (INTERRUPT_ID_BITS_SPIS - 1) << 19;
        break;
    case GICD_TYPER2:
        break;
    case GICD_IIDR:
        value = (PRODUCT_ID_KVM << 24) | (VARIANT_ID_XEN << 16) |
                (IMPLEMENTER_ARM << 0);
        break;
    default:
        return 0;
    }

    return value;
}

static void vgic_mmio_write_v3_misc(struct vcpu *vcpu, paddr_t addr,
                                    unsigned int len, unsigned long val)
{
    struct vgic_dist *dist = &vcpu->domain->arch.vgic;

    switch ( addr & 0x0c )
    {
    case GICD_CTLR:
    {
        bool was_enabled;

        domain_lock(vcpu->domain);

        was_enabled   = dist->enabled;

        dist->enabled = val & GICD_CTLR_ENABLE_G1A;

        if ( dist->enabled )
            vgic_kick_vcpus(vcpu->domain);

        domain_unlock(vcpu->domain);
        break;
    }
    case GICD_TYPER:
    case GICD_TYPER2:
    case GICD_IIDR:
        /* This is at best for documentation purposes... */
        return;
    }
}

static unsigned long vgic_mmio_read_irouter(struct vcpu *vcpu, paddr_t addr,
                                            unsigned int len)
{
    int intid            = VGIC_ADDR_TO_INTID(addr, 64);
    struct vgic_irq *irq = vgic_get_irq(vcpu->domain, NULL, intid);
    unsigned long ret    = 0;

    if ( !irq )
        return 0;

    /* The upper word is RAZ for us. */
    if ( !(addr & 4) )
        ret = extract_bytes(irq->mpidr, addr & 7, len);

    vgic_put_irq(vcpu->domain, irq);
    return ret;
}

static void vgic_mmio_write_irouter(struct vcpu *vcpu, paddr_t addr,
                                    unsigned int len, unsigned long val)
{
    int intid = VGIC_ADDR_TO_INTID(addr, 64);
    struct vgic_irq *irq;
    unsigned long flags;

    /* The upper word is WI for us since we don't implement Aff3. */
    if ( addr & 4 )
        return;

    irq = vgic_get_irq(vcpu->domain, NULL, intid);

    if ( !irq )
        return;

    spin_lock_irqsave(&irq->irq_lock, flags);

    /* We only care about and preserve Aff0, Aff1 and Aff2. */
    irq->mpidr       = val & GENMASK(23, 0);
    irq->target_vcpu = mpidr_to_vcpu(vcpu->domain, irq->mpidr);

    spin_unlock_irqrestore(&irq->irq_lock, flags);
    vgic_put_irq(vcpu->domain, irq);
}

static bool vgic_mmio_vcpu_rdist_is_last(struct vcpu *vcpu)
{
    struct vgic_dist *vgic    = &vcpu->domain->arch.vgic;
    struct vgic_cpu *vgic_cpu = &vcpu->arch.vgic;
    struct vgic_redist_region *iter, *rdreg = vgic_cpu->rdreg;

    if ( !rdreg )
        return false;

    if ( vgic_cpu->rdreg_index < rdreg->free_index - 1 )
    {
        return false;
    }
    else if ( rdreg->count && vgic_cpu->rdreg_index == (rdreg->count - 1) )
    {
        struct list_head *rd_regions = &vgic->rd_regions;
        paddr_t end = rdreg->base + rdreg->count * VGIC_V3_REDIST_SIZE;

        /*
         * the rdist is the last one of the redist region,
         * check whether there is no other contiguous rdist region
         */
        list_for_each_entry(iter, rd_regions, list)
        {
            if ( iter->base == end && iter->free_index > 0 )
                return false;
        }
    }
    return true;
}

static unsigned long vgic_mmio_read_v3r_typer(struct vcpu *vcpu, paddr_t addr,
                                              unsigned int len)
{
    unsigned long mpidr = vcpuid_to_vaffinity(vcpu->vcpu_id);
    int target_vcpu_id  = vcpu->vcpu_id;
    u64 value;

    value = (u64)(mpidr & GENMASK(23, 0)) << 32;
    value |= ((target_vcpu_id & 0xffff) << 8);

    if ( vgic_mmio_vcpu_rdist_is_last(vcpu) )
        value |= GICR_TYPER_LAST;

    return extract_bytes(value, addr & 7, len);
}

static unsigned long vgic_mmio_read_v3r_iidr(struct vcpu *vcpu, paddr_t addr,
                                             unsigned int len)
{
    return (PRODUCT_ID_KVM << 24) | (VARIANT_ID_XEN << 16) |
           (IMPLEMENTER_ARM << 0);
}

static unsigned long vgic_mmio_read_v3_idregs(struct vcpu *vcpu, paddr_t addr,
                                              unsigned int len)
{
    switch ( addr & 0xfff )
    {
    case GICD_ICPIDR2:
        /* report a GICv3 compliant implementation */
        return 0x3b;
    }

    return 0;
}

static unsigned long vgic_mmio_read_v3r_ctlr(struct vcpu *vcpu, paddr_t addr,
                                             unsigned int len)
{
    struct vgic_cpu *vgic_cpu = &vcpu->arch.vgic;
    unsigned long val;

    val = atomic_read(&vgic_cpu->ctlr);
    val |= GICR_CTLR_IR | GICR_CTLR_CES;

    return val;
}

static void vgic_mmio_write_v3r_ctlr(struct vcpu *vcpu, paddr_t addr,
                                     unsigned int len, unsigned long val)
{
    struct vgic_cpu *vgic_cpu = &vcpu->arch.vgic;
    uint32_t ctlr;

    if ( !vgic_has_its(vcpu->domain) )
        return;

    if ( !(val & GICR_CTLR_ENABLE_LPIS) )
    {
        /*
		 * Don't disable if RWP is set, as there already an
		 * ongoing disable. Funky guest...
		 */
        ctlr = atomic_cmpxchg(&vgic_cpu->ctlr, GICR_CTLR_ENABLE_LPIS,
                              GICR_CTLR_RWP);
        if ( ctlr != GICR_CTLR_ENABLE_LPIS )
            return;
    }
    else
    {
        ctlr = atomic_cmpxchg(&vgic_cpu->ctlr, 0, GICR_CTLR_ENABLE_LPIS);
        if ( ctlr != 0 )
            return;
    }
}

bool vgic_lpis_enabled(struct vcpu *vcpu)
{
    struct vgic_cpu *vgic_cpu = &vcpu->arch.vgic;

    return atomic_read(&vgic_cpu->ctlr) == GICR_CTLR_ENABLE_LPIS;
}

/* We want to avoid outer shareable. */
u64 vgic_sanitise_shareability(u64 field)
{
    switch ( field )
    {
    case GIC_BASER_OuterShareable:
        return GIC_BASER_InnerShareable;
    default:
        return field;
    }
}

/* Avoid any inner non-cacheable mapping. */
u64 vgic_sanitise_inner_cacheability(u64 field)
{
    switch ( field )
    {
    case GIC_BASER_CACHE_nCnB:
    case GIC_BASER_CACHE_nC:
        return GIC_BASER_CACHE_RaWb;
    default:
        return field;
    }
}

/* Non-cacheable or same-as-inner are OK. */
u64 vgic_sanitise_outer_cacheability(u64 field)
{
    switch ( field )
    {
    case GIC_BASER_CACHE_SameAsInner:
    case GIC_BASER_CACHE_nC:
        return field;
    default:
        return GIC_BASER_CACHE_SameAsInner;
    }
}

u64 vgic_sanitise_field(u64 reg, u64 field_mask, int field_shift,
                        u64 (*sanitise_fn)(u64))
{
    u64 field = (reg & field_mask) >> field_shift;

    field     = sanitise_fn(field) << field_shift;
    return (reg & ~field_mask) | field;
}

#define PROPBASER_RES0_MASK                                                    \
    (GENMASK_ULL(63, 59) | GENMASK_ULL(55, 52) | GENMASK_ULL(6, 5))
#define PENDBASER_RES0_MASK                                                    \
    (BIT(63, ULL) | GENMASK_ULL(61, 59) | GENMASK_ULL(55, 52) |                \
     GENMASK_ULL(15, 12) | GENMASK_ULL(6, 0))

static u64 vgic_sanitise_pendbaser(u64 reg)
{
    reg = vgic_sanitise_field(reg, GICR_PENDBASER_SHAREABILITY_MASK,
                              GICR_PENDBASER_SHAREABILITY_SHIFT,
                              vgic_sanitise_shareability);
    reg = vgic_sanitise_field(reg, GICR_PENDBASER_INNER_CACHEABILITY_MASK,
                              GICR_PENDBASER_INNER_CACHEABILITY_SHIFT,
                              vgic_sanitise_inner_cacheability);
    reg = vgic_sanitise_field(reg, GICR_PENDBASER_OUTER_CACHEABILITY_MASK,
                              GICR_PENDBASER_OUTER_CACHEABILITY_SHIFT,
                              vgic_sanitise_outer_cacheability);

    reg &= ~PENDBASER_RES0_MASK;

    return reg;
}

static u64 vgic_sanitise_propbaser(u64 reg)
{
    reg = vgic_sanitise_field(reg, GICR_PROPBASER_SHAREABILITY_MASK,
                              GICR_PROPBASER_SHAREABILITY_SHIFT,
                              vgic_sanitise_shareability);
    reg = vgic_sanitise_field(reg, GICR_PROPBASER_INNER_CACHEABILITY_MASK,
                              GICR_PROPBASER_INNER_CACHEABILITY_SHIFT,
                              vgic_sanitise_inner_cacheability);
    reg = vgic_sanitise_field(reg, GICR_PROPBASER_OUTER_CACHEABILITY_MASK,
                              GICR_PROPBASER_OUTER_CACHEABILITY_SHIFT,
                              vgic_sanitise_outer_cacheability);

    reg &= ~PROPBASER_RES0_MASK;
    return reg;
}

static unsigned long vgic_mmio_read_propbase(struct vcpu *vcpu, paddr_t addr,
                                             unsigned int len)
{
    struct vgic_dist *dist = &vcpu->domain->arch.vgic;

    return extract_bytes(dist->propbaser, addr & 7, len);
}

static void vgic_mmio_write_propbase(struct vcpu *vcpu, paddr_t addr,
                                     unsigned int len, unsigned long val)
{
    struct vgic_dist *dist = &vcpu->domain->arch.vgic;
    u64 old_propbaser, propbaser;

    /* Storing a value with LPIs already enabled is undefined */
    if ( vgic_lpis_enabled(vcpu) )
        return;

    do
    {
        old_propbaser = dist->propbaser;
        propbaser     = old_propbaser;
        propbaser     = update_64bit_reg(propbaser, addr & 4, len, val);
        propbaser     = vgic_sanitise_propbaser(propbaser);
    } while ( cmpxchg64(&dist->propbaser, old_propbaser, propbaser) !=
              old_propbaser );
}

static unsigned long vgic_mmio_read_pendbase(struct vcpu *vcpu, paddr_t addr,
                                             unsigned int len)
{
    struct vgic_cpu *vgic_cpu = &vcpu->arch.vgic;
    u64 value                 = vgic_cpu->pendbaser;

    value &= ~GICR_PENDBASER_PTZ;

    return extract_bytes(value, addr & 7, len);
}

static void vgic_mmio_write_pendbase(struct vcpu *vcpu, paddr_t addr,
                                     unsigned int len, unsigned long val)
{
    struct vgic_cpu *vgic_cpu = &vcpu->arch.vgic;
    u64 old_pendbaser, pendbaser;

    /* Storing a value with LPIs already enabled is undefined */
    if ( vgic_lpis_enabled(vcpu) )
        return;

    do
    {
        old_pendbaser = vgic_cpu->pendbaser;
        pendbaser     = old_pendbaser;
        pendbaser     = update_64bit_reg(pendbaser, addr & 4, len, val);
        pendbaser     = vgic_sanitise_pendbaser(pendbaser);
    } while ( cmpxchg64(&vgic_cpu->pendbaser, old_pendbaser, pendbaser) !=
              old_pendbaser );
}

static const struct vgic_register_region vgic_v3_dist_registers[] = {
    REGISTER_DESC_WITH_LENGTH(GICD_CTLR,
        vgic_mmio_read_v3_misc, vgic_mmio_write_v3_misc,
        16, VGIC_ACCESS_32bit),
    REGISTER_DESC_WITH_LENGTH(GICD_STATUSR,
        vgic_mmio_read_rao, vgic_mmio_write_wi, 4,
        VGIC_ACCESS_32bit),
    REGISTER_DESC_WITH_BITS_PER_IRQ(GICD_IGROUPR,
        vgic_mmio_read_rao, vgic_mmio_write_wi, 1,
        VGIC_ACCESS_32bit),
    REGISTER_DESC_WITH_BITS_PER_IRQ(GICD_ISENABLER,
        vgic_mmio_read_enable, vgic_mmio_write_senable, 1,
        VGIC_ACCESS_32bit),
    REGISTER_DESC_WITH_BITS_PER_IRQ(GICD_ICENABLER,
        vgic_mmio_read_enable, vgic_mmio_write_cenable, 1,
        VGIC_ACCESS_32bit),
    REGISTER_DESC_WITH_BITS_PER_IRQ(GICD_ISPENDR,
        vgic_mmio_read_pending, vgic_mmio_write_spending, 1,
        VGIC_ACCESS_32bit),
    REGISTER_DESC_WITH_BITS_PER_IRQ(GICD_ICPENDR,
        vgic_mmio_read_pending, vgic_mmio_write_cpending, 1,
        VGIC_ACCESS_32bit),
    REGISTER_DESC_WITH_BITS_PER_IRQ(GICD_ISACTIVER,
        vgic_mmio_read_active, vgic_mmio_write_sactive, 1,
        VGIC_ACCESS_32bit),
    REGISTER_DESC_WITH_BITS_PER_IRQ(GICD_ICACTIVER,
        vgic_mmio_read_active, vgic_mmio_write_cactive,
        1, VGIC_ACCESS_32bit),
    REGISTER_DESC_WITH_BITS_PER_IRQ(GICD_IPRIORITYR,
        vgic_mmio_read_priority, vgic_mmio_write_priority,
        8, VGIC_ACCESS_32bit | VGIC_ACCESS_8bit),
    REGISTER_DESC_WITH_BITS_PER_IRQ(GICD_ITARGETSR,
        vgic_mmio_read_raz, vgic_mmio_write_wi, 8,
        VGIC_ACCESS_32bit | VGIC_ACCESS_8bit),
    REGISTER_DESC_WITH_BITS_PER_IRQ(GICD_ICFGR,
        vgic_mmio_read_config, vgic_mmio_write_config, 2,
        VGIC_ACCESS_32bit),
    REGISTER_DESC_WITH_BITS_PER_IRQ(GICD_IGRPMODR,
        vgic_mmio_read_raz, vgic_mmio_write_wi, 1,
        VGIC_ACCESS_32bit),
    REGISTER_DESC_WITH_BITS_PER_IRQ(GICD_IROUTER,
        vgic_mmio_read_irouter, vgic_mmio_write_irouter, 64,
        VGIC_ACCESS_64bit | VGIC_ACCESS_32bit),
    REGISTER_DESC_WITH_LENGTH(GICD_IDREGS,
        vgic_mmio_read_v3_idregs, vgic_mmio_write_wi, 48,
        VGIC_ACCESS_32bit),
};

static const struct vgic_register_region vgic_v3_rd_registers[] = {
    /* RD_base registers */
    REGISTER_DESC_WITH_LENGTH(GICR_CTLR,
        vgic_mmio_read_v3r_ctlr, vgic_mmio_write_v3r_ctlr, 4,
        VGIC_ACCESS_32bit),
    REGISTER_DESC_WITH_LENGTH(GICR_STATUSR,
        vgic_mmio_read_raz, vgic_mmio_write_wi, 4,
        VGIC_ACCESS_32bit),
    REGISTER_DESC_WITH_LENGTH(GICR_IIDR,
        vgic_mmio_read_v3r_iidr, vgic_mmio_write_wi, 4,
        VGIC_ACCESS_32bit),
    REGISTER_DESC_WITH_LENGTH(GICR_TYPER,
        vgic_mmio_read_v3r_typer, vgic_mmio_write_wi, 8,
        VGIC_ACCESS_64bit | VGIC_ACCESS_32bit),
    REGISTER_DESC_WITH_LENGTH(GICR_WAKER,
        vgic_mmio_read_raz, vgic_mmio_write_wi, 4,
        VGIC_ACCESS_32bit),
    REGISTER_DESC_WITH_LENGTH(GICR_PROPBASER,
        vgic_mmio_read_propbase, vgic_mmio_write_propbase, 8,
        VGIC_ACCESS_64bit | VGIC_ACCESS_32bit),
    REGISTER_DESC_WITH_LENGTH(GICR_PENDBASER,
        vgic_mmio_read_pendbase, vgic_mmio_write_pendbase, 8,
        VGIC_ACCESS_64bit | VGIC_ACCESS_32bit),
    REGISTER_DESC_WITH_LENGTH(GICR_INVLPIR,
        vgic_mmio_read_raz, vgic_mmio_write_wi, 8,
        VGIC_ACCESS_64bit | VGIC_ACCESS_32bit),
    REGISTER_DESC_WITH_LENGTH(GICR_INVALLR,
        vgic_mmio_read_raz, vgic_mmio_write_wi, 8,
        VGIC_ACCESS_64bit | VGIC_ACCESS_32bit),
    REGISTER_DESC_WITH_LENGTH(GICR_SYNCR,
        vgic_mmio_read_raz, vgic_mmio_write_wi, 4,
        VGIC_ACCESS_32bit),
    REGISTER_DESC_WITH_LENGTH(GICR_IDREGS,
        vgic_mmio_read_v3_idregs, vgic_mmio_write_wi, 48,
        VGIC_ACCESS_32bit),
    /* SGI_base registers */
    REGISTER_DESC_WITH_LENGTH(SZ_64K + GICR_IGROUPR0,
        vgic_mmio_read_rao, vgic_mmio_write_wi, 4,
        VGIC_ACCESS_32bit),
    REGISTER_DESC_WITH_LENGTH(SZ_64K + GICR_ISENABLER0,
        vgic_mmio_read_enable, vgic_mmio_write_senable, 4,
        VGIC_ACCESS_32bit),
    REGISTER_DESC_WITH_LENGTH(SZ_64K + GICR_ICENABLER0,
        vgic_mmio_read_enable, vgic_mmio_write_cenable, 4,
        VGIC_ACCESS_32bit),
    REGISTER_DESC_WITH_LENGTH(SZ_64K + GICR_ISPENDR0,
        vgic_mmio_read_pending, vgic_mmio_write_spending, 4,
        VGIC_ACCESS_32bit),
    REGISTER_DESC_WITH_LENGTH(SZ_64K + GICR_ICPENDR0,
        vgic_mmio_read_pending, vgic_mmio_write_cpending,4,
        VGIC_ACCESS_32bit),
    REGISTER_DESC_WITH_LENGTH(SZ_64K + GICR_ISACTIVER0,
        vgic_mmio_read_active, vgic_mmio_write_sactive, 4,
        VGIC_ACCESS_32bit),
    REGISTER_DESC_WITH_LENGTH(SZ_64K + GICR_ICACTIVER0,
        vgic_mmio_read_active, vgic_mmio_write_cactive, 4,
        VGIC_ACCESS_32bit),
    REGISTER_DESC_WITH_LENGTH(SZ_64K + GICR_IPRIORITYR0,
        vgic_mmio_read_priority, vgic_mmio_write_priority, 32,
        VGIC_ACCESS_32bit | VGIC_ACCESS_8bit),
    REGISTER_DESC_WITH_LENGTH(SZ_64K + GICR_ICFGR0,
        vgic_mmio_read_config, vgic_mmio_write_config, 8,
        VGIC_ACCESS_32bit),
    REGISTER_DESC_WITH_LENGTH(SZ_64K + GICR_IGRPMODR0,
        vgic_mmio_read_raz, vgic_mmio_write_wi, 4,
        VGIC_ACCESS_32bit),
    REGISTER_DESC_WITH_LENGTH(SZ_64K + GICR_NSACR,
        vgic_mmio_read_raz, vgic_mmio_write_wi, 4,
        VGIC_ACCESS_32bit),
};

unsigned int vgic_v3_init_dist_iodev(struct vgic_io_device *dev)
{
    dev->regions    = vgic_v3_dist_registers;
    dev->nr_regions = ARRAY_SIZE(vgic_v3_dist_registers);

    return SZ_64K;
}

static bool vgic_v3_redist_region_full(struct vgic_redist_region *region)
{
    if ( !region->count )
        return false;

    return (region->free_index >= region->count);
}

/**
 * vgic_v3_rdist_free_slot - Look up registered rdist regions and identify one
 * which has free space to put a new rdist region.
 *
 * @rd_regions: redistributor region list head
 *
 * A redistributor regions maps n redistributors, n = region size / (2 x 64kB).
 * Stride between redistributors is 0 and regions are filled in the index order.
 *
 * Return: the redist region handle, if any, that has space to map a new rdist
 * region.
 */
static struct vgic_redist_region *vgic_v3_rdist_free_slot(struct list_head *rd_regions)
{
    struct vgic_redist_region *rdreg;

    list_for_each_entry(rdreg, rd_regions, list)
    {
        if ( !vgic_v3_redist_region_full(rdreg) )
            return rdreg;
    }
    return NULL;
}


/**
 * vgic_register_redist_iodev - register a single redist iodev
 * @vcpu:    The VCPU to which the redistributor belongs
 *
 * Register a KVM iodev for this VCPU's redistributor using the address
 * provided.
 *
 * Return 0 on success, -ERRNO otherwise.
 */
int vgic_register_redist_iodev(struct vcpu *vcpu)
{
    struct domain *d              = vcpu->domain;
    struct vgic_dist *vgic        = &d->arch.vgic;
    struct vgic_cpu *vgic_cpu     = &vcpu->arch.vgic;
    struct vgic_io_device *rd_dev = &vcpu->arch.vgic.rd_iodev;
    struct vgic_redist_region *rdreg;
    paddr_t rd_base;

    /*
     * We may be creating VCPUs before having set the base address for the
     * redistributor region, in which case we will come back to this
     * function for all VCPUs when the base address is set.  Just return
     * without doing any work for now.
     */
    rdreg = vgic_v3_rdist_free_slot(&vgic->rd_regions);
    if ( !rdreg )
        return 0;

    vgic_cpu->rdreg       = rdreg;
    vgic_cpu->rdreg_index = rdreg->free_index;

    rd_base             = rdreg->base + rdreg->free_index * VGIC_V3_REDIST_SIZE;

    rd_dev->base_fn     = gaddr_to_gfn(rd_base);
    rd_dev->iodev_type  = IODEV_REDIST;
    rd_dev->regions     = vgic_v3_rd_registers;
    rd_dev->nr_regions  = ARRAY_SIZE(vgic_v3_rd_registers);
    rd_dev->redist_vcpu = vcpu;

    register_mmio_handler(d, &vgic_io_ops, rd_base, VGIC_V3_REDIST_SIZE,
                          rd_dev);

    rdreg->free_index++;
    return 0;
}

static int vgic_register_all_redist_iodevs(struct domain *d)
{
    struct vcpu *vcpu;
    int ret = 0;

    for_each_vcpu(d, vcpu)
    {
        ret = vgic_register_redist_iodev(vcpu);
        if ( ret )
            break;
    }

    if ( ret )
    {
        printk(XENLOG_ERR "Failed to register redistributor iodev\n");
    }

    return ret;
}

static inline size_t vgic_v3_rd_region_size(struct domain *d,
                                            struct vgic_redist_region *rdreg)
{
    if ( !rdreg->count )
        return d->max_vcpus * VGIC_V3_REDIST_SIZE;
    else
        return rdreg->count * VGIC_V3_REDIST_SIZE;
}

/**
 * vgic_v3_rdist_overlap - check if a region overlaps with any
 * existing redistributor region
 *
 * @kvm: kvm handle
 * @base: base of the region
 * @size: size of region
 *
 * Return: true if there is an overlap
 */
bool vgic_v3_rdist_overlap(struct domain *domain, paddr_t base, size_t size)
{
    struct vgic_dist *d = &domain->arch.vgic;
    struct vgic_redist_region *rdreg;

    list_for_each_entry(rdreg, &d->rd_regions, list)
    {
        if ( (base + size > rdreg->base) &&
             (base < rdreg->base + vgic_v3_rd_region_size(domain, rdreg)) )
            return true;
    }
    return false;
}

static inline bool vgic_dist_overlap(struct domain *domain, paddr_t base,
                                     size_t size)
{
    struct vgic_dist *d = &domain->arch.vgic;

    return (base + size > d->dbase) && (base < d->dbase + VGIC_V3_DIST_SIZE);
}

struct vgic_redist_region *vgic_v3_rdist_region_from_index(struct domain *d,
                                                           u32 index)
{
    struct list_head *rd_regions = &d->arch.vgic.rd_regions;
    struct vgic_redist_region *rdreg;

    list_for_each_entry(rdreg, rd_regions, list)
    {
        if ( rdreg->index == index )
            return rdreg;
    }
    return NULL;
}

int vgic_check_iorange(paddr_t ioaddr, paddr_t addr, paddr_t alignment,
                       paddr_t size)
{
    if ( !IS_VGIC_ADDR_UNDEF(ioaddr) )
        return -EEXIST;

    if ( !IS_ALIGNED(addr, alignment) || !IS_ALIGNED(size, alignment) )
        return -EINVAL;

    if ( addr + size < addr )
        return -EINVAL;

    return 0;
}

/**
 * vgic_v3_alloc_redist_region - Allocate a new redistributor region
 *
 * Performs various checks before inserting the rdist region in the list.
 * Those tests depend on whether the size of the rdist region is known
 * (ie. count != 0). The list is sorted by rdist region index.
 *
 * @kvm: kvm handle
 * @index: redist region index
 * @base: base of the new rdist region
 * @count: number of redistributors the region is made of (0 in the old style
 * single region, whose size is induced from the number of vcpus)
 *
 * Return 0 on success, < 0 otherwise
 */
static int vgic_v3_alloc_redist_region(struct domain *domain, uint32_t index,
                                       paddr_t base, uint32_t count)
{
    struct vgic_dist *d = &domain->arch.vgic;
    struct vgic_redist_region *rdreg;
    struct list_head *rd_regions = &d->rd_regions;
    int nr_vcpus                 = domain->max_vcpus;
    size_t size                  = count ? count * VGIC_V3_REDIST_SIZE
                                         : nr_vcpus * VGIC_V3_REDIST_SIZE;
    int ret;

    /* cross the end of memory ? */
    if ( base + size < base )
        return -EINVAL;

    if ( list_empty(rd_regions) )
    {
        if ( index != 0 )
            return -EINVAL;
    }
    else
    {
        rdreg = list_last_entry(rd_regions, struct vgic_redist_region, list);

        /* Don't mix single region and discrete redist regions */
        if ( !count && rdreg->count )
            return -EINVAL;

        if ( !count )
            return -EEXIST;

        if ( index != rdreg->index + 1 )
            return -EINVAL;
    }

    /*
     * For legacy single-region redistributor regions (!count),
     * check that the redistributor region does not overlap with the
     * distributor's address space.
     */
    if ( !count && !IS_VGIC_ADDR_UNDEF(d->dbase) &&
         vgic_dist_overlap(domain, base, size) )
        return -EINVAL;

    /* collision with any other rdist region? */
    if ( vgic_v3_rdist_overlap(domain, base, size) )
        return -EINVAL;

    rdreg = xzalloc(struct vgic_redist_region);
    if ( !rdreg )
        return -ENOMEM;

    rdreg->base = VGIC_ADDR_UNDEF;

    ret = vgic_check_iorange(rdreg->base, base, SZ_64K, size);
    if ( ret )
        goto free;

    rdreg->base       = base;
    rdreg->count      = count;
    rdreg->free_index = 0;
    rdreg->index      = index;

    list_add_tail(&rdreg->list, rd_regions);
    return 0;
free:
    xfree(rdreg);
    return ret;
}

void vgic_v3_free_redist_region(struct vgic_redist_region *rdreg)
{
    list_del(&rdreg->list);
    xfree(rdreg);
}

int vgic_v3_set_redist_base(struct domain *d, u32 index, u64 addr, u32 count)
{
    int ret;

    ret = vgic_v3_alloc_redist_region(d, index, addr, count);
    if ( ret )
        return ret;

    /*
     * Register iodevs for each existing VCPU.  Adding more VCPUs
     * afterwards will register the iodevs when needed.
     */
    ret = vgic_register_all_redist_iodevs(d);
    if ( ret )
    {
        struct vgic_redist_region *rdreg;

        rdreg = vgic_v3_rdist_region_from_index(d, index);
        vgic_v3_free_redist_region(rdreg);
        return ret;
    }

    return 0;
}
