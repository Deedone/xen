/* SPDX-License-Identifier: (GPL-2.0-or-later OR BSD-2-Clause) */

/*
 * IOMMU stage-1 emulation for ARM SMMUv3.
 *
 * TODO: Using ARM generic implementer/product/variant to avoid activating
 * errata workarounds in the guest driver due to emulation layer.
 * This needs to be fully handled in the future.
 */

#include <xen/guest_access.h>
#include <xen/param.h>
#include <xen/sched.h>
#include <asm/mmio.h>
#include <asm/vgic-emul.h>
#include <asm/viommu.h>
#include <asm/vreg.h>

#include "smmu-v3.h"

/* Register Definition */
#define ARM_SMMU_IDR2       0x8
#define ARM_SMMU_IDR3       0xc
#define ARM_SMMU_IDR4       0x10
#define IDR0_TERM_MODEL     (1 << 26)
#define IDR3_RIL            (1 << 10)
/* Only CMDQ, EVENTQ and SMMUEN supported */
#define CR0_RESERVED        0xFFFFFFF2
/* TODO: Right now only 16-bit SID size is supported */
#define SMMU_IDR1_SIDSIZE   16
/* The CMDQ/EVTQ sizes are currently set to the architectural maximum. */
#define SMMU_CMDQS          19
#define SMMU_EVTQS          19
#define DWORDS_BYTES        8

/* 
 * SMMUv3 command definitions
 * Some commands are fully handled by the emulation layer, while others are
 * currently treated as architectural no-ops because the required behavior
 * is either implicitly guaranteed by Xen or not yet modeled explicitly.
 *
 * Emulation handled commands:
 *
 * - CMD_CFGI_STE
 * - CMD_TLBI_NH_ASID
 * - CMD_TLBI_NSNH_ALL
 * - CMD_TLBI_NH_VA
 *
 * No-op/Implicitly handled commands:
 *
 * - CMD_SYNC
 * - CMD_PREFETCH_CFG
 * - CMD_CFGI_CD
 * - CMD_CFGI_CD_ALL
 * - CMD_CFGI_ALL
 *
 * TODO: Remaining architecture-defined commands are not supported (error
 * produced), due to lack of support in SMMUv3 driver / emulation layer
 * TODO: Range / per-device TLB invalidation not supported atm
 */
#define CMDQ_OP_PREFETCH_CFG    0x1
#define CMDQ_OP_CFGI_STE        0x3
#define CMDQ_OP_CFGI_ALL        0x4
#define CMDQ_OP_CFGI_CD         0x5
#define CMDQ_OP_CFGI_CD_ALL     0x6
#define CMDQ_OP_TLBI_NH_ASID    0x11
#define CMDQ_OP_TLBI_NH_VA      0x12
#define CMDQ_OP_TLBI_NSNH_ALL   0x30
#define CMDQ_OP_CMD_SYNC        0x46

/* Queue Handling */
#define Q_BASE(q)       ((q)->q_base & Q_BASE_ADDR_MASK)
#define Q_CONS_ENT(q)   (Q_BASE(q) + Q_IDX(q, (q)->cons) * (q)->ent_size)
#define Q_PROD_ENT(q)   (Q_BASE(q) + Q_IDX(q, (q)->prod) * (q)->ent_size)

/* Helper Macros */
#define smmu_cmd_get_command(x)     FIELD_GET(CMDQ_0_OP, x)
#define smmu_cmd_get_sid(x)         FIELD_GET(CMDQ_CFGI_0_SID, x)
#define smmu_get_ste_s1cdmax(x)     FIELD_GET(STRTAB_STE_0_S1CDMAX, x)
#define smmu_get_ste_s1fmt(x)       FIELD_GET(STRTAB_STE_0_S1FMT, x)
#define smmu_get_ste_s1stalld(x)    FIELD_GET(STRTAB_STE_1_S1STALLD, x)
#define smmu_get_ste_s1ctxptr(x)    FIELD_PREP(STRTAB_STE_0_S1CTXPTR_MASK, \
                                    FIELD_GET(STRTAB_STE_0_S1CTXPTR_MASK, x))

/* stage-1 translation configuration */
struct arm_vsmmu_s1_trans_cfg {
    paddr_t s1ctxptr;
    uint8_t s1fmt;
    uint8_t s1cdmax;
    bool    bypassed;             /* translation is bypassed */
    bool    aborted;              /* translation is aborted */
};

/* virtual smmu queue */
struct arm_vsmmu_queue {
    uint64_t    q_base; /* base register */
    uint32_t    prod;
    uint32_t    cons;
    uint8_t     ent_size;
    uint8_t     max_n_shift;
};

struct virt_smmu {
    struct      domain *d;
    struct      list_head viommu_list;
    paddr_t     addr;
    uint32_t    features;
    uint32_t    cr[3];
    uint32_t    cr0ack;
    uint32_t    gerror;
    uint32_t    gerrorn;
    uint32_t    strtab_base_cfg;
    uint64_t    strtab_base;
    uint32_t    irq_ctrl;
    uint64_t    gerror_irq_cfg0;
    uint64_t    evtq_irq_cfg0;
    struct      arm_vsmmu_queue evtq, cmdq;
    spinlock_t  cmd_queue_lock;
    spinlock_t  evt_queue_lock;
    spinlock_t  gerror_lock;
    spinlock_t  cr0_lock;
    spinlock_t  cr1_lock;
    spinlock_t  cr2_lock;
    spinlock_t  strtab_cfg_lock;
    spinlock_t  irq_cfg_lock;
};

/* Helper functions */
static inline bool smmu_get_smmu_enabled(struct virt_smmu *smmu)
{
    bool enabled;

    spin_lock(&smmu->cr0_lock);
    enabled = FIELD_GET(CR0_SMMUEN, smmu->cr[0]);
    spin_unlock(&smmu->cr0_lock);

    return enabled;
}

static inline bool smmu_get_cmdq_enabled(struct virt_smmu *smmu)
{
    bool enabled;

    spin_lock(&smmu->cr0_lock);
    enabled = FIELD_GET(CR0_CMDQEN, smmu->cr[0]);
    spin_unlock(&smmu->cr0_lock);

    return enabled;
}

static inline bool smmu_get_evtq_enabled(struct virt_smmu *smmu)
{
    bool enabled;

    spin_lock(&smmu->cr0_lock);
    enabled = FIELD_GET(CR0_EVTQEN, smmu->cr[0]);
    spin_unlock(&smmu->cr0_lock);

    return enabled;
};

/* Queue manipulation functions */
static bool queue_empty(struct arm_vsmmu_queue *q)
{
    return Q_IDX(q, q->prod) == Q_IDX(q, q->cons) &&
           Q_WRP(q, q->prod) == Q_WRP(q, q->cons);
}

static void queue_inc_cons(struct arm_vsmmu_queue *q)
{
    uint32_t cons = (Q_WRP(q, q->cons) | Q_IDX(q, q->cons)) + 1;
    q->cons = Q_OVF(q->cons) | Q_WRP(q, cons) | Q_IDX(q, cons);
}

static void dump_smmu_command(uint64_t *command)
{
    gprintk(XENLOG_ERR, "cmd 0x%02llx: %016"PRIx64" %016"PRIx64"\n",
             smmu_cmd_get_command(command[0]), command[0], command[1]);
}
static int arm_vsmmu_find_ste(struct virt_smmu *smmu, uint32_t sid,
                              uint64_t *ste)
{
    paddr_t addr, strtab_base;
    struct domain *d = smmu->d;
    uint32_t log2size;
    int strtab_size_shift;
    int ret;

    spin_lock(&smmu->strtab_cfg_lock);
    log2size = FIELD_GET(STRTAB_BASE_CFG_LOG2SIZE, smmu->strtab_base_cfg);
    spin_unlock(&smmu->strtab_cfg_lock);

    if ( sid >= (1 << MIN(log2size, SMMU_IDR1_SIDSIZE)) )
        return -EINVAL;

    spin_lock(&smmu->strtab_cfg_lock);
    if ( FIELD_GET(STRTAB_BASE_CFG_FMT, smmu->strtab_base_cfg) ==
         STRTAB_BASE_CFG_FMT_2LVL )
    {
        int idx, max_l2_ste, span;
        paddr_t l1ptr, l2ptr;
        uint64_t l1std;
        uint8_t sid_split = FIELD_GET(STRTAB_BASE_CFG_SPLIT,
                              smmu->strtab_base_cfg);

        strtab_size_shift = MAX(5, (int)log2size - sid_split - 1 + 3);
        strtab_base = smmu->strtab_base & STRTAB_BASE_ADDR_MASK &
                        ~GENMASK_ULL(strtab_size_shift, 0);
        spin_unlock(&smmu->strtab_cfg_lock);
        idx = (sid >> sid_split) * STRTAB_L1_DESC_DWORDS;
        l1ptr = (paddr_t)(strtab_base + idx * sizeof(l1std));

        ret = access_guest_memory_by_gpa(d, l1ptr, &l1std,
                                         sizeof(l1std), false);
        if ( ret )
        {
            gdprintk(XENLOG_ERR,
                     "Could not read L1PTR at 0X%"PRIx64"\n", l1ptr);
            return ret;
        }

        span = FIELD_GET(STRTAB_L1_DESC_SPAN, l1std);
        if ( !span )
        {
            gdprintk(XENLOG_ERR, "Bad StreamID span\n");
            return -EINVAL;
        }

        max_l2_ste = (1 << span) - 1;
        l2ptr = FIELD_PREP(STRTAB_L1_DESC_L2PTR_MASK,
                    FIELD_GET(STRTAB_L1_DESC_L2PTR_MASK, l1std));
        idx = sid & ((1 << sid_split) - 1);
        if ( idx > max_l2_ste )
        {
            gdprintk(XENLOG_ERR, "idx=%d > max_l2_ste=%d\n",
                     idx, max_l2_ste);
            return -EINVAL;
        }
        addr = l2ptr + idx * sizeof(*ste) * STRTAB_STE_DWORDS;
    }
    else
    {
        strtab_size_shift = log2size + 5;
        strtab_base = smmu->strtab_base & STRTAB_BASE_ADDR_MASK &
                      ~GENMASK_ULL(strtab_size_shift, 0);
        spin_unlock(&smmu->strtab_cfg_lock);
        addr = strtab_base + sid * sizeof(*ste) * STRTAB_STE_DWORDS;
    }
    ret = access_guest_memory_by_gpa(d, addr, ste, sizeof(*ste), false);
    if ( ret )
    {
        gdprintk(XENLOG_ERR,
                "Cannot fetch pte at address=0x%"PRIx64"\n", addr);
        return -EINVAL;
    }

    return 0;
}

static int arm_vsmmu_decode_ste(struct virt_smmu *smmu, uint32_t sid,
                                struct arm_vsmmu_s1_trans_cfg *cfg,
                                uint64_t *ste)
{
    uint64_t val = ste[0];

    if ( !(val & STRTAB_STE_0_V) )
        return -EAGAIN;

    switch ( FIELD_GET(STRTAB_STE_0_CFG, val) )
    {
    case STRTAB_STE_0_CFG_BYPASS:
        cfg->bypassed = true;
        return 0;
    case STRTAB_STE_0_CFG_ABORT:
        cfg->aborted = true;
        return 0;
    case STRTAB_STE_0_CFG_S1_TRANS:
        break;
    case STRTAB_STE_0_CFG_S2_TRANS:
        gdprintk(XENLOG_ERR, "vSMMUv3 does not support stage 2 yet\n");
        goto bad_ste;
    default:
        gdprintk(XENLOG_ERR, "Invalid STE config type\n");
        goto bad_ste;
    }

    cfg->s1ctxptr = smmu_get_ste_s1ctxptr(val);
    cfg->s1fmt = smmu_get_ste_s1fmt(val);
    cfg->s1cdmax = smmu_get_ste_s1cdmax(val);
    if ( cfg->s1cdmax != 0 )
    {
        gdprintk(XENLOG_ERR,
                 "vSMMUv3 does not support multiple context descriptors\n");
        goto bad_ste;
    }

    return 0;

  bad_ste:
    return -EINVAL;
}

static int arm_vsmmu_handle_cfgi_ste(struct virt_smmu *smmu, uint64_t *cmdptr)
{
    int ret;
    uint64_t ste[STRTAB_STE_DWORDS];
    struct domain *d = smmu->d;
    struct domain_iommu *hd = dom_iommu(d);
    struct arm_vsmmu_s1_trans_cfg s1_cfg = {0};
    uint32_t sid = smmu_cmd_get_sid(cmdptr[0]);
    struct iommu_guest_config guest_cfg = {0};

    ret = arm_vsmmu_find_ste(smmu, sid, ste);
    if ( ret )
        return ret;

    ret = arm_vsmmu_decode_ste(smmu, sid, &s1_cfg, ste);
    if ( ret )
        return (ret == -EAGAIN ) ? 0 : ret;

    guest_cfg.s1ctxptr = s1_cfg.s1ctxptr;
    guest_cfg.s1fmt = s1_cfg.s1fmt;
    guest_cfg.s1cdmax = s1_cfg.s1cdmax;

    if ( s1_cfg.bypassed )
        guest_cfg.config = ARM_SMMU_DOMAIN_BYPASS;
    else if ( s1_cfg.aborted )
        guest_cfg.config = ARM_SMMU_DOMAIN_ABORT;
    else
        guest_cfg.config = ARM_SMMU_DOMAIN_NESTED;

    ret = hd->platform_ops->attach_guest_config(d, sid, &guest_cfg);
    if ( ret )
        return ret;

    return 0;
}

static int arm_vsmmu_handle_cmds(struct virt_smmu *smmu)
{
    struct arm_vsmmu_queue *q = &smmu->cmdq;
    struct domain *d = smmu->d;
    uint64_t command[CMDQ_ENT_DWORDS];
    paddr_t addr;
    int ret = 0;

    if ( !smmu_get_cmdq_enabled(smmu) )
        return 0;

    while ( !queue_empty(q) )
    {
        addr = Q_CONS_ENT(q);
        ret = access_guest_memory_by_gpa(d, addr, command,
                                         sizeof(command), false);
        if ( ret ) {
            queue_inc_cons(q);
            return ret;
        }

        switch ( smmu_cmd_get_command(command[0]) )
        {
        case CMDQ_OP_CFGI_STE:
            ret = arm_vsmmu_handle_cfgi_ste(smmu, command);
            break;
        case CMDQ_OP_PREFETCH_CFG:
        case CMDQ_OP_CFGI_CD:
        case CMDQ_OP_CFGI_CD_ALL:
        case CMDQ_OP_CFGI_ALL:
        case CMDQ_OP_CMD_SYNC:
            break;
        case CMDQ_OP_TLBI_NH_ASID:
        case CMDQ_OP_TLBI_NSNH_ALL:
        case CMDQ_OP_TLBI_NH_VA:
            ret = iommu_iotlb_flush_all(smmu->d, 1);
            if ( !ret )
                break;
        default:
            gdprintk(XENLOG_ERR, "vSMMUv3: unhandled command\n");
            dump_smmu_command(command);
            break;
        }

        if ( ret )
        {
            gdprintk(XENLOG_ERR,
                     "vSMMUv3: command error %d while handling command\n",
                     ret);
            dump_smmu_command(command);
        }
        queue_inc_cons(q);
    }

    return ret;
}

static int vsmmuv3_mmio_write(struct vcpu *v, mmio_info_t *info,
                              register_t r, void *priv)
{
    struct virt_smmu *smmu = priv;
    uint64_t reg;
    uint32_t reg32;
    struct hsr_dabt dabt = info->dabt;
    uint64_t offset = info->gpa - smmu->addr;

    if ( offset & 0x10000 ) {
        /* page 1 */
        switch ( info->gpa & 0xffff )
        {
        case VREG32(ARM_SMMU_EVTQ_PROD):
            if ( dabt.size != DABT_WORD ) goto bad_width;
            spin_lock(&smmu->evt_queue_lock);
            reg32 = smmu->evtq.prod;
            vreg_reg32_update(&reg32, r, info);
            smmu->evtq.prod = reg32;
            spin_unlock(&smmu->evt_queue_lock);
            break;
        case VREG32(ARM_SMMU_EVTQ_CONS):
            /*  Must only be written when SMMU_CR0.EVTQEN == 0, otherwise ignore */
            if ( smmu_get_evtq_enabled(smmu) )
                break;
            if ( dabt.size != DABT_WORD ) goto bad_width;
            spin_lock(&smmu->evt_queue_lock);
            reg32 = smmu->evtq.cons;
            vreg_reg32_update(&reg32, r, info);
            smmu->evtq.cons = reg32;
            spin_unlock(&smmu->evt_queue_lock);
            break;
        default:
            printk(XENLOG_G_ERR
                "%pd: vSMMUv3: unhandled read r%d offset %"PRIpaddr"\n",
                v, info->dabt.reg, (unsigned long)info->gpa & 0xffff);
            return IO_HANDLED;
        }
    }
    else {
        /* page 0 */
        switch ( info->gpa & 0xffff )
        {
        case VREG32(ARM_SMMU_CR0):
            if ( dabt.size != DABT_WORD ) goto bad_width;
            spin_lock(&smmu->cr0_lock);
            reg32 = smmu->cr[0];
            vreg_reg32_update(&reg32, r, info);
            smmu->cr[0] = reg32;
            smmu->cr0ack = reg32 & ~CR0_RESERVED;
            spin_unlock(&smmu->cr0_lock);
            break;

        case VREG32(ARM_SMMU_CR1):
            if ( dabt.size != DABT_WORD ) goto bad_width;
            spin_lock(&smmu->cr1_lock);
            reg32 = smmu->cr[1];
            vreg_reg32_update(&reg32, r, info);
            smmu->cr[1] = reg32;
            spin_unlock(&smmu->cr1_lock);
            break;

        case VREG32(ARM_SMMU_CR2):
            if ( dabt.size != DABT_WORD ) goto bad_width;
            spin_lock(&smmu->cr2_lock);
            reg32 = smmu->cr[2];
            vreg_reg32_update(&reg32, r, info);
            smmu->cr[2] = reg32;
            spin_unlock(&smmu->cr2_lock);
            break;

        case VREG64(ARM_SMMU_STRTAB_BASE):
            /*  Must only be written when SMMU_CR0.SMMUEN == 0, otherwise ignore */
            if ( smmu_get_smmu_enabled(smmu) )
                break;
            if ( dabt.size != DABT_DOUBLE_WORD && dabt.size != DABT_WORD )
                goto bad_width;
            spin_lock(&smmu->strtab_cfg_lock);
            reg = smmu->strtab_base;
            vreg_reg64_update(&reg, r, info);
            smmu->strtab_base = reg;
            spin_unlock(&smmu->strtab_cfg_lock);
            break;

        case VREG32(ARM_SMMU_STRTAB_BASE_CFG):
            /*  Must only be written when SMMU_CR0.SMMUEN == 0, otherwise ignore */
            if ( smmu_get_smmu_enabled(smmu) )
                break;
            if ( dabt.size != DABT_WORD ) goto bad_width;
            spin_lock(&smmu->strtab_cfg_lock);
            reg32 = smmu->strtab_base_cfg;
            vreg_reg32_update(&reg32, r, info);
            smmu->strtab_base_cfg = reg32;
            spin_unlock(&smmu->strtab_cfg_lock);
            break;

        case VREG64(ARM_SMMU_CMDQ_BASE):
            /*  Must only be written when SMMU_CR0.CMDQEN == 0, otherwise ignore */
            if ( smmu_get_cmdq_enabled(smmu) )
                break;
            if ( dabt.size != DABT_DOUBLE_WORD && dabt.size != DABT_WORD )
                goto bad_width;
            spin_lock(&smmu->cmd_queue_lock);
            reg = smmu->cmdq.q_base;
            vreg_reg64_update(&reg, r, info);
            smmu->cmdq.q_base = reg;
            smmu->cmdq.max_n_shift = FIELD_GET(Q_BASE_LOG2SIZE, smmu->cmdq.q_base);
            if ( smmu->cmdq.max_n_shift > SMMU_CMDQS )
                smmu->cmdq.max_n_shift = SMMU_CMDQS;
            spin_unlock(&smmu->cmd_queue_lock);
            break;

        case VREG32(ARM_SMMU_CMDQ_PROD):
            if ( dabt.size != DABT_WORD ) goto bad_width;
            spin_lock(&smmu->cmd_queue_lock);
            reg32 = smmu->cmdq.prod;
            vreg_reg32_update(&reg32, r, info);
            smmu->cmdq.prod = reg32;

            if ( arm_vsmmu_handle_cmds(smmu) )
                gdprintk(XENLOG_ERR, "error handling vSMMUv3 commands\n");

            spin_unlock(&smmu->cmd_queue_lock);
            break;

        case VREG32(ARM_SMMU_CMDQ_CONS):
            /*  Must only be written when SMMU_CR0.CMDQEN == 0, otherwise ignore */
            if ( smmu_get_cmdq_enabled(smmu) )
                break;
            if ( dabt.size != DABT_WORD ) goto bad_width;
            spin_lock(&smmu->cmd_queue_lock);
            reg32 = smmu->cmdq.cons;
            vreg_reg32_update(&reg32, r, info);
            smmu->cmdq.cons = reg32;
            spin_unlock(&smmu->cmd_queue_lock);
            break;

        case VREG64(ARM_SMMU_EVTQ_BASE):
            /*  Must only be written when SMMU_CR0.EVTQEN == 0, otherwise ignore */
            if ( smmu_get_evtq_enabled(smmu) )
                break;
            if ( dabt.size != DABT_DOUBLE_WORD && dabt.size != DABT_WORD )
                goto bad_width;
            spin_lock(&smmu->evt_queue_lock);
            reg = smmu->evtq.q_base;
            vreg_reg64_update(&reg, r, info);
            smmu->evtq.q_base = reg;
            smmu->evtq.max_n_shift = FIELD_GET(Q_BASE_LOG2SIZE, smmu->evtq.q_base);
            if ( smmu->evtq.max_n_shift > SMMU_EVTQS )
                smmu->evtq.max_n_shift = SMMU_EVTQS;
            spin_unlock(&smmu->evt_queue_lock);
            break;

        case VREG32(ARM_SMMU_IRQ_CTRL):
            if ( dabt.size != DABT_WORD ) goto bad_width;
            spin_lock(&smmu->irq_cfg_lock);
            reg32 = smmu->irq_ctrl;
            vreg_reg32_update(&reg32, r, info);
            smmu->irq_ctrl = reg32;
            spin_unlock(&smmu->irq_cfg_lock);
            break;

        case VREG64(ARM_SMMU_GERROR_IRQ_CFG0):
            if ( dabt.size != DABT_DOUBLE_WORD && dabt.size != DABT_WORD )
                goto bad_width;
            spin_lock(&smmu->irq_cfg_lock);
            reg = smmu->gerror_irq_cfg0;
            vreg_reg64_update(&reg, r, info);
            smmu->gerror_irq_cfg0 = reg;
            spin_unlock(&smmu->irq_cfg_lock);
            break;

        case VREG64(ARM_SMMU_EVTQ_IRQ_CFG0):
            if ( dabt.size != DABT_DOUBLE_WORD && dabt.size != DABT_WORD )
                goto bad_width;
            spin_lock(&smmu->irq_cfg_lock);
            reg = smmu->evtq_irq_cfg0;
            vreg_reg64_update(&reg, r, info);
            smmu->evtq_irq_cfg0 = reg;
            spin_unlock(&smmu->irq_cfg_lock);
            break;

        case VREG32(ARM_SMMU_GERRORN):
            if ( dabt.size != DABT_WORD ) goto bad_width;
            spin_lock(&smmu->gerror_lock);
            reg32 = smmu->gerrorn;
            vreg_reg32_update(&reg32, r, info);
            smmu->gerrorn = reg32;
            spin_unlock(&smmu->gerror_lock);
            break;

        default:
            printk(XENLOG_G_ERR
                "%pd: vSMMUv3: unhandled write r%d offset %"PRIpaddr"\n",
                v, info->dabt.reg, (unsigned long)info->gpa & 0xffff);
            return IO_HANDLED;
        }
    }

    return IO_HANDLED;

 bad_width:
    gprintk(XENLOG_G_ERR,
            "%pd: vSMMUv3: bad write width %d r%d offset %"PRIpaddr"\n",
            v, dabt.size, dabt.reg, info->gpa & 0xffff);
    return IO_HANDLED;
}

static int vsmmuv3_mmio_read(struct vcpu *v, mmio_info_t *info,
                             register_t *r, void *priv)
{
    struct virt_smmu *smmu = priv;
    uint64_t reg;
    struct hsr_dabt dabt = info->dabt;
    uint64_t offset = info->gpa - smmu->addr;

    if ( offset & 0x10000 ) {
        /* page 1 */
        switch ( info->gpa & 0xffff )
        {
            case VREG32(ARM_SMMU_EVTQ_PROD):
                if ( dabt.size != DABT_WORD ) goto bad_width;
                spin_lock(&smmu->evt_queue_lock);
                *r = vreg_reg32_extract(smmu->evtq.prod, info);
                spin_unlock(&smmu->evt_queue_lock);
                break;

            case VREG32(ARM_SMMU_EVTQ_CONS):
                if ( dabt.size != DABT_WORD ) goto bad_width;
                spin_lock(&smmu->evt_queue_lock);
                *r = vreg_reg32_extract(smmu->evtq.cons, info);
                spin_unlock(&smmu->evt_queue_lock);
                break;

            default:
                printk(XENLOG_G_ERR
                    "%pd: vSMMUv3: unhandled read r%d offset %"PRIpaddr"\n",
                    v, info->dabt.reg, (unsigned long)info->gpa & 0xffff);
                return IO_HANDLED;
        }
    }
    else {
        /* page 0 */
        switch ( info->gpa & 0xffff )
        {
        case VREG32(ARM_SMMU_IDR0):
            if ( dabt.size != DABT_WORD ) goto bad_width;
            /*
            * Emulated SMMU capabilities.
            *
            * Some architectural capabilities are currently not implemented
            * by the stage-1 emulation layer or underlying Xen SMMU support,
            * therefore they are not advertised atm even if supported by the HW.
            */
            reg  = FIELD_PREP(IDR0_S1P, 1) |
                /* 
                * TODO: Coherent access not supported by the emulation atm.
                * Guest must perform explicit cache clean
                */
                FIELD_PREP(IDR0_COHACC, 0) |
                /* Transaction stall on fault not implemented */
                FIELD_PREP(IDR0_STALL_MODEL, 1) |
                /* Terminating a transaction with RAZ/WI behavior not implemented */
                FIELD_PREP(IDR0_TERM_MODEL, 1);

            /* HW reflected capabilities */
            reg |= FIELD_PREP(IDR0_ST_LVL,
                            smmu->features & ARM_SMMU_FEAT_2_LVL_STRTAB ?
                            1 : 0);
            reg |= FIELD_PREP(IDR0_TTF,
                            smmu->features & ARM_SMMU_FEAT_TTF_AARCH32_64 ?
                            IDR0_TTF_AARCH32_64 : IDR0_TTF_AARCH64);

            reg |= FIELD_PREP(IDR0_ASID16,
                            smmu->features & ARM_SMMU_FEAT_ASID_16 ? 1 : 0);
            if ( (smmu->features & ARM_SMMU_FEAT_TT_LE) &&
                (smmu->features & ARM_SMMU_FEAT_TT_BE) )
                reg |= FIELD_PREP(IDR0_TTENDIAN, IDR0_TTENDIAN_MIXED);
            else if ( smmu->features & ARM_SMMU_FEAT_TT_LE )
                reg |= FIELD_PREP(IDR0_TTENDIAN, IDR0_TTENDIAN_LE);
    #ifdef __BIG_ENDIAN
            else if ( smmu->features & ARM_SMMU_FEAT_TT_BE )
                reg |= FIELD_PREP(IDR0_TTENDIAN, IDR0_TTENDIAN_BE);
    #endif
            *r = vreg_reg32_extract(reg, info);
            break;

        case VREG32(ARM_SMMU_IDR1):
            if ( dabt.size != DABT_WORD ) goto bad_width;
            reg  = FIELD_PREP(IDR1_SIDSIZE, SMMU_IDR1_SIDSIZE) |
                FIELD_PREP(IDR1_CMDQS, SMMU_CMDQS) |
                FIELD_PREP(IDR1_EVTQS, SMMU_EVTQS);
            *r = vreg_reg32_extract(reg, info);
            break;

        case VREG32(ARM_SMMU_IDR2):
            goto read_reserved;

        case VREG32(ARM_SMMU_IDR3):
            if ( dabt.size != DABT_WORD ) goto bad_width;
            reg = 0;
            *r = vreg_reg32_extract(reg, info);
            break;

        case VREG32(ARM_SMMU_IDR4):
            goto read_impl_defined;

        case VREG32(ARM_SMMU_IDR5):
            if ( dabt.size != DABT_WORD ) goto bad_width;
            reg = 0;
            if (smmu->features & ARM_SMMU_FEAT_GRAN64K)
                reg |= IDR5_GRAN64K;
            if (smmu->features & ARM_SMMU_FEAT_GRAN16K)
                reg |= IDR5_GRAN16K;
            if (smmu->features & ARM_SMMU_FEAT_GRAN4K)
                reg |= IDR5_GRAN4K;

            if (smmu->features & ARM_SMMU_FEAT_OAS_32_BIT)
                reg |= IDR5_OAS_32_BIT;
            if (smmu->features & ARM_SMMU_FEAT_OAS_36_BIT)
                reg |= IDR5_OAS_36_BIT;
            if (smmu->features & ARM_SMMU_FEAT_OAS_40_BIT)
                reg |= IDR5_OAS_40_BIT;
            if (smmu->features & ARM_SMMU_FEAT_OAS_42_BIT)
                reg |= IDR5_OAS_42_BIT;
            if (smmu->features & ARM_SMMU_FEAT_OAS_44_BIT)
                reg |= IDR5_OAS_44_BIT;
            if (smmu->features & ARM_SMMU_FEAT_OAS_48_BIT)
                reg |= IDR5_OAS_48_BIT;
            if (smmu->features & ARM_SMMU_FEAT_OAS_52_BIT)
                reg |= IDR5_OAS_52_BIT;

            *r = vreg_reg32_extract(reg, info);
            break;

        case VREG32(ARM_SMMU_IIDR):
            if ( dabt.size != DABT_WORD ) goto bad_width;
            /* Use generic ARM IIDR value to avoid activating errata workarounds */
            reg  = FIELD_PREP(IIDR_IMPLEMENTER, IIDR_IMPLEMENTER_ARM) |
                FIELD_PREP(IIDR_REVISION, 0) |
                FIELD_PREP(IIDR_VARIANT, 0) |
                FIELD_PREP(IIDR_PRODUCTID, 0);
            *r = vreg_reg32_extract(reg, info);
            break;

        case VREG32(ARM_SMMU_CR0):
            if ( dabt.size != DABT_WORD ) goto bad_width;
            spin_lock(&smmu->cr0_lock);
            *r = vreg_reg32_extract(smmu->cr[0], info);
            spin_unlock(&smmu->cr0_lock);
            break;

        case VREG32(ARM_SMMU_CR0ACK):
            if ( dabt.size != DABT_WORD ) goto bad_width;
            spin_lock(&smmu->cr0_lock);
            *r = vreg_reg32_extract(smmu->cr0ack, info);
            spin_unlock(&smmu->cr0_lock);
            break;

        case VREG32(ARM_SMMU_CR1):
            if ( dabt.size != DABT_WORD ) goto bad_width;
            spin_lock(&smmu->cr1_lock);
            *r = vreg_reg32_extract(smmu->cr[1], info);
            spin_unlock(&smmu->cr1_lock);
            break;

        case VREG32(ARM_SMMU_CR2):
            if ( dabt.size != DABT_WORD ) goto bad_width;
            spin_lock(&smmu->cr2_lock);
            *r = vreg_reg32_extract(smmu->cr[2], info);
            spin_unlock(&smmu->cr2_lock);
            break;

        case VREG64(ARM_SMMU_STRTAB_BASE):
            if ( dabt.size != DABT_DOUBLE_WORD && dabt.size != DABT_WORD )
                goto bad_width;
            spin_lock(&smmu->strtab_cfg_lock);
            *r = vreg_reg64_extract(smmu->strtab_base, info);
            spin_unlock(&smmu->strtab_cfg_lock);
            break;

        case VREG32(ARM_SMMU_STRTAB_BASE_CFG):
            if ( dabt.size != DABT_WORD ) goto bad_width;
            spin_lock(&smmu->strtab_cfg_lock);
            *r = vreg_reg32_extract(smmu->strtab_base_cfg, info);
            spin_unlock(&smmu->strtab_cfg_lock);
            break;

        case VREG64(ARM_SMMU_CMDQ_BASE):
            if ( dabt.size != DABT_DOUBLE_WORD && dabt.size != DABT_WORD )
                goto bad_width;
            spin_lock(&smmu->cmd_queue_lock);
            *r = vreg_reg64_extract(smmu->cmdq.q_base, info);
            spin_unlock(&smmu->cmd_queue_lock);
            break;

        case VREG32(ARM_SMMU_CMDQ_PROD):
            if ( dabt.size != DABT_WORD ) goto bad_width;
            spin_lock(&smmu->cmd_queue_lock);
            *r = vreg_reg32_extract(smmu->cmdq.prod, info);
            spin_unlock(&smmu->cmd_queue_lock);
            break;

        case VREG32(ARM_SMMU_CMDQ_CONS):
            if ( dabt.size != DABT_WORD ) goto bad_width;
            spin_lock(&smmu->cmd_queue_lock);
            *r = vreg_reg32_extract(smmu->cmdq.cons, info);
            spin_unlock(&smmu->cmd_queue_lock);
            break;

        case VREG64(ARM_SMMU_EVTQ_BASE):
            if ( dabt.size != DABT_DOUBLE_WORD && dabt.size != DABT_WORD )
                goto bad_width;
            spin_lock(&smmu->evt_queue_lock);
            *r = vreg_reg64_extract(smmu->evtq.q_base, info);
            spin_unlock(&smmu->evt_queue_lock);
            break;

        case VREG32(ARM_SMMU_IRQ_CTRL):
        case VREG32(ARM_SMMU_IRQ_CTRLACK):
            if ( dabt.size != DABT_WORD ) goto bad_width;
            spin_lock(&smmu->irq_cfg_lock);
            *r = vreg_reg32_extract(smmu->irq_ctrl, info);
            spin_unlock(&smmu->irq_cfg_lock);
            break;

        case VREG64(ARM_SMMU_GERROR_IRQ_CFG0):
            if ( dabt.size != DABT_DOUBLE_WORD && dabt.size != DABT_WORD )
                goto bad_width;
            spin_lock(&smmu->irq_cfg_lock);
            *r = vreg_reg64_extract(smmu->gerror_irq_cfg0, info);
            spin_unlock(&smmu->irq_cfg_lock);
            break;

        case VREG64(ARM_SMMU_EVTQ_IRQ_CFG0):
            if ( dabt.size != DABT_DOUBLE_WORD && dabt.size != DABT_WORD )
                goto bad_width;
            spin_lock(&smmu->irq_cfg_lock);
            *r = vreg_reg64_extract(smmu->evtq_irq_cfg0, info);
            spin_unlock(&smmu->irq_cfg_lock);
            break;

        case VREG32(ARM_SMMU_GERROR):
            if ( dabt.size != DABT_WORD ) goto bad_width;
            spin_lock(&smmu->gerror_lock);
            *r = vreg_reg32_extract(smmu->gerror, info);
            spin_unlock(&smmu->gerror_lock);
            break;

        case VREG32(ARM_SMMU_GERRORN):
            if ( dabt.size != DABT_WORD ) goto bad_width;
            spin_lock(&smmu->gerror_lock);
            *r = vreg_reg32_extract(smmu->gerrorn, info);
            spin_unlock(&smmu->gerror_lock);
            break;

        default:
            printk(XENLOG_G_ERR
                "%pd: vSMMUv3: unhandled read r%d offset %"PRIpaddr"\n",
                v, info->dabt.reg, (unsigned long)info->gpa & 0xffff);
            return IO_HANDLED;
        }
    }
    return IO_HANDLED;

 read_impl_defined:
    printk(XENLOG_G_DEBUG
           "%pd: vSMMUv3: RAZ on implementation defined register offset %"PRIpaddr"\n",
           v, info->gpa & 0xffff);
    *r = 0;
    return IO_HANDLED;

 read_reserved:
    printk(XENLOG_G_DEBUG
           "%pd: vSMMUv3: RAZ on reserved register offset %"PRIpaddr"\n",
           v, info->gpa & 0xffff);
    *r = 0;
    return IO_HANDLED;
 bad_width:
    gprintk(XENLOG_G_ERR,
            "%pd: vSMMUv3: bad read width %d r%d offset %"PRIpaddr"\n",
            v, dabt.size, dabt.reg, info->gpa & 0xffff);
    return IO_HANDLED;
}

static const struct mmio_handler_ops vsmmuv3_mmio_handler = {
    .read  = vsmmuv3_mmio_read,
    .write = vsmmuv3_mmio_write,
};

static int vsmmuv3_init_single(struct domain *d, paddr_t addr, paddr_t size,
                               uint32_t features)
{
    struct virt_smmu *smmu;

    smmu = xzalloc(struct virt_smmu);
    if ( !smmu )
        return -ENOMEM;

    smmu->d = d;
    smmu->cmdq.ent_size = CMDQ_ENT_DWORDS * DWORDS_BYTES;
    smmu->evtq.ent_size = EVTQ_ENT_DWORDS * DWORDS_BYTES;

    smmu->features = features;

    spin_lock_init(&smmu->cmd_queue_lock);
    spin_lock_init(&smmu->evt_queue_lock);
    spin_lock_init(&smmu->gerror_lock);
    spin_lock_init(&smmu->cr0_lock);
    spin_lock_init(&smmu->cr1_lock);
    spin_lock_init(&smmu->cr2_lock);
    spin_lock_init(&smmu->strtab_cfg_lock);
    spin_lock_init(&smmu->irq_cfg_lock);

    register_mmio_handler(d, &vsmmuv3_mmio_handler, addr, size, smmu);

    /* Register the vIOMMU to be able to clean it up later. */
    list_add_tail(&smmu->viommu_list, &d->arch.viommu_list);

    return 0;
}

int domain_vsmmuv3_init(struct domain *d)
{
    int ret;
    struct host_iommu *hw_iommu;

    INIT_LIST_HEAD(&d->arch.viommu_list);

    if ( is_hardware_domain(d) )
    {
        struct host_iommu *hw_iommu;

        list_for_each_entry(hw_iommu, &host_iommu_list, entry)
        {
            ret = vsmmuv3_init_single(d, hw_iommu->addr, hw_iommu->size,
                                      hw_iommu->features);
            if ( ret )
                return ret;
        }
    }
    else
    {
        hw_iommu = list_first_entry(&host_iommu_list, struct host_iommu, entry);
        ret = vsmmuv3_init_single(d, GUEST_VSMMUV3_BASE, GUEST_VSMMUV3_SIZE,
                                  hw_iommu->features);
        if ( ret )
            return ret;
    }

    return 0;
}

int vsmmuv3_relinquish_resources(struct domain *d)
{
    struct virt_smmu *pos, *temp;

    /* Cope with uninitialized vIOMMU */
    if ( list_head_is_null(&d->arch.viommu_list) )
        return 0;

    list_for_each_entry_safe(pos, temp, &d->arch.viommu_list, viommu_list )
    {
        list_del(&pos->viommu_list);
        xfree(pos);
    }

    return 0;
}

static const struct viommu_ops vsmmuv3_ops = {
    .domain_init = domain_vsmmuv3_init,
    .relinquish_resources = vsmmuv3_relinquish_resources,
};

static const struct viommu_desc vsmmuv3_desc = {
    .ops = &vsmmuv3_ops,
    .viommu_type = XEN_DOMCTL_CONFIG_VIOMMU_SMMUV3,
};

void __init vsmmuv3_set_type(void)
{
    const struct viommu_desc *desc = &vsmmuv3_desc;

    set_cur_viommu(desc);
}

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
