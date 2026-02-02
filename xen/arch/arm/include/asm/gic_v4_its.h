/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * ARM GICv4 ITS support
 *
 * Penny Zheng <penny.zheng@arm.com>
 * Copyright (c) 2023 ARM Ltd.
 */

#ifndef ARM_GIC_V4_ITS_H
#define ARM_GIC_V4_ITS_H

#include <asm/arm64/io.h>

#include <xen/delay.h>
#include <xen/lib.h>
#include <xen/spinlock.h>
#include <xen/types.h>

#define GITS_CMD_VMOVI                   0x21
#define GITS_CMD_VMOVP                   0x22
#define GITS_CMD_VSYNC                   0x25
#define GITS_CMD_VMAPP                   0x29
#define GITS_CMD_VMAPTI                  0x2a
#define GITS_CMD_VINVALL                 0x2d

struct its_device;
struct pending_irq;
struct its_vpe;

struct its_vm {
    struct its_vpe **vpes;
    /* Number of VPE. */
    unsigned int nr_vpes;
    uint32_t *db_lpi_bases;
    unsigned int nr_db_lpis;
    /* Property table per VM. */
    void *vproptable;
};

struct its_vpe {
    rwlock_t lock;
    uint32_t vpe_id;
    /* Pending table per VCPU. */
    void *vpendtable;
    uint32_t vpe_db_lpi;
    struct its_vm *its_vm;
    unsigned int col_idx;
    /* Pending VLPIs on schedule out? */
    bool            pending_last;
    struct {
        /* Implementation Defined Area Invalid */
        bool idai;
        /* VPE proxy mapping */
        int vpe_proxy_event;
    };
    /*
     * Ensure mutual exclusion between affinity setting of the vPE
     * and vLPI operations using vpe->col_idx.
     */
    spinlock_t vpe_lock;
};

/* Describes the mapping of a VLPI */
struct its_vlpi_map {
    struct its_vm       *vm;
    unsigned int        vpe_idx;    /* Index of the VPE */
    uint32_t            vintid;     /* Virtual LPI number */
    bool                db_enabled; /* Is the VPE doorbell to be generated? */
    struct pending_irq  *pirq;
    struct its_device   *dev;
    uint32_t            eventid;
};

struct event_vlpi_map {
    unsigned int            nr_lpis;
    spinlock_t              vlpi_lock;
    struct its_vm           *vm;
    struct its_vlpi_map     *vlpi_maps;
    unsigned int            nr_vlpis;
};

void gicv4_its_vpeid_allocator_init(void);

#define GICR_VPROPBASER                              0x0070
#define GICR_VPENDBASER                              0x0078

#define GICR_VPENDBASER_Dirty                   (1UL << 60)
#define GICR_VPENDBASER_PendingLast             (1UL << 61)
#define GICR_VPENDBASER_IDAI                    (1UL << 62)
#define GICR_VPENDBASER_Valid                   (1UL << 63)

#define GICR_VPENDBASER_OUTER_CACHEABILITY_SHIFT         56
#define GICR_VPENDBASER_SHAREABILITY_SHIFT               10
#define GICR_VPENDBASER_INNER_CACHEABILITY_SHIFT          7
#define GICR_VPENDBASER_POLL_TIMEOUT_US             100000U

#define gits_read_vpropbaser(c)         readq_relaxed(c)
#define gits_write_vpropbaser(v, c)     \
    do { writeq_relaxed(v, c); } while ( 0 )

/*
 * Clearing GICR_VPENDBASER.Valid is an explicit state transition and should
 * only be attempted once the caller expects the VPE to become non-resident.
 */
static inline bool gits_clear_vpendbaser_valid(void __iomem *addr)
{
    uint64_t tmp;
    unsigned int timeout = GICR_VPENDBASER_POLL_TIMEOUT_US;

    tmp = readq_relaxed(addr);
    if ( !(tmp & GICR_VPENDBASER_Valid) )
        return true;

    writeq_relaxed(tmp & ~GICR_VPENDBASER_Valid, addr);

    do {
        if ( !timeout-- )
        {
            printk(XENLOG_WARNING
                   "GICv4: timeout clearing GICR_VPENDBASER.Valid\n");
            return false;
        }

        udelay(1);
        tmp = readq_relaxed(addr);
    } while ( tmp & GICR_VPENDBASER_Valid );

    return true;
}

static inline void gits_write_vpendbaser(uint64_t val, void __iomem *addr)
{
    writeq_relaxed(val, addr);
}
#define gits_read_vpendbaser(c)     readq_relaxed(c)

#endif

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
