/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * ARM GICv4 ITS support
 *
 * Penny Zheng <penny.zheng@arm.com>
 * Copyright (c) 2023 ARM Ltd.
 */

#ifndef ARM_GIC_V4_ITS_H
#define ARM_GIC_V4_ITS_H

#include <xen/lib.h>
#include <xen/spinlock.h>
#include <xen/types.h>

#include <asm/arm64/io.h>
#include <asm/gic.h>

#define GITS_CMD_VMOVI                   0x21
#define GITS_CMD_VMOVP                   0x22
#define GITS_CMD_VSGI                    0x23
#define GITS_CMD_VSYNC                   0x25
#define GITS_CMD_VMAPP                   0x29
#define GITS_CMD_VMAPTI                  0x2a
#define GITS_CMD_VINVALL                 0x2d
#define GITS_CMD_INVDB                   0x2e

/* ITS registers, offsets from the GICv4.1 SGIR frame. */
#define GITS_SGIR                        0x00020
#define GITS_SGIR_VPEID                  GENMASK_ULL(47, 32)
#define GITS_SGIR_VINTID                 GENMASK_ULL(7, 0)

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
    uint64_t vpropbaser;
    /* An ITS still references one or more VM-owned tables. */
    bool teardown_failed;
};

struct its_vpe {
    rwlock_t lock;
    uint32_t vpe_id;
    /* Pending table per VCPU. */
    void *vpendtable;
    uint32_t vpe_db_lpi;
    struct its_vm *its_vm;
    unsigned int col_idx;
    bool resident;
    bool cleanup_failed;
    /* Pending VLPIs hint shared between IRQ and schedule paths. */
    bool            pending_last;
    struct {
        /* Implementation Defined Area Invalid */
        bool idai;
        /* VPE proxy mapping */
        int vpe_proxy_event;
        /* Number of active v4.1 VMAPP mappings for this VPE. */
        unsigned int vmapp_count;
    };
    struct {
        uint8_t priority;
        bool enabled;
        bool group;
    } sgi_config[NR_GIC_SGI];
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
int gicv4_init_vpe_proxy(void);
int allocate_vpe_l1_table(void);
int direct_lpi_inv(struct its_device *dev, uint32_t eventid,
                   uint32_t db_lpi, unsigned int cpu);

#define GICR_VPROPBASER                              0x0070
#define GICR_VPENDBASER                              0x0078

#define GICR_VPROPBASER_OUTER_CACHEABILITY_SHIFT         56
#define GICR_VPROPBASER_SHAREABILITY_SHIFT               10
#define GICR_VPROPBASER_SHAREABILITY_MASK                \
        (3UL << GICR_VPROPBASER_SHAREABILITY_SHIFT)
#define GICR_VPROPBASER_INNER_CACHEABILITY_SHIFT          7
#define GICR_VPROPBASER_INNER_CACHEABILITY_MASK           \
        (7UL << GICR_VPROPBASER_INNER_CACHEABILITY_SHIFT)
#define GICR_VPROPBASER_IDBITS_MASK                    0x1f

#define GICR_VPENDBASER_Dirty                   (1UL << 60)
#define GICR_VPENDBASER_PendingLast             (1UL << 61)
#define GICR_VPENDBASER_IDAI                    (1UL << 62)
#define GICR_VPENDBASER_Valid                   (1UL << 63)

#define GICR_VPENDBASER_OUTER_CACHEABILITY_SHIFT         56
#define GICR_VPENDBASER_SHAREABILITY_SHIFT               10
#define GICR_VPENDBASER_INNER_CACHEABILITY_SHIFT          7
/*
 * GICv4.1 VPENDBASER, used for VPE residency. On top of these fields,
 * also use the above Valid, PendingLast and Dirty bits.
 */
#define GICR_VPENDBASER_4_1_VGRP1EN             (1ULL << 58)
#define GICR_VPENDBASER_4_1_VPEID               GENMASK_ULL(15, 0)
#define GICR_VPENDBASER_4_1_DB                  (1ULL << 62)

/*
 * GICv4.1 VPROPBASER combines old VPROPBASER attributes with ITS_BASER-like
 * table description fields.
 */
#define GICR_VPROPBASER_4_1_VALID               (1ULL << 63)
#define GICR_VPROPBASER_4_1_ENTRY_SIZE          GENMASK_ULL(61, 59)
#define GICR_VPROPBASER_4_1_INDIRECT            (1ULL << 55)
#define GICR_VPROPBASER_4_1_PAGE_SIZE           GENMASK_ULL(54, 53)
#define GICR_VPROPBASER_4_1_Z                   (1ULL << 52)
#define GICR_VPROPBASER_4_1_ADDR                GENMASK_ULL(51, 12)
#define GICR_VPROPBASER_4_1_SIZE                GENMASK_ULL(6, 0)

#define GICR_INVALLR_VPEID                      GENMASK_ULL(47, 32)
#define GICR_INVALLR_V                          (1ULL << 63)

#define gits_read_vpropbaser(c)         readq_relaxed(c)
#define gits_write_vpropbaser(v, c)     \
    do { writeq_relaxed(v, c); } while ( 0 )

static inline void gits_write_vpendbaser(uint64_t val, void __iomem *addr)
{
    writeq_relaxed(val, addr);
}
#define gits_read_vpendbaser(c)     readq_relaxed(c)

#define GICR_INVLPIR_INTID                GENMASK_ULL(31, 0)
#define GICR_INVLPIR_VPEID                GICR_INVALLR_VPEID
#define GICR_INVLPIR_V                    GICR_INVALLR_V

#endif

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
