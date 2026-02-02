/*
 * ARM GICv4 ITS support
 *
 * Penny Zheng <penny.zheng@arm.com>
 * Copyright (c) 2023 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; under version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; If not, see <http://www.gnu.org/licenses/>.
 */

#include <asm/arm64/io.h>

#ifndef __ASM_ARM_GICV4_ITS_H__
#define __ASM_ARM_GICV4_ITS_H__

#define GITS_CMD_VMOVI                   0x21
#define GITS_CMD_VMOVP                   0x22
#define GITS_CMD_VSGI                    0x23
#define GITS_CMD_VSYNC                   0x25
#define GITS_CMD_VMAPP                   0x29
#define GITS_CMD_VMAPTI                  0x2a
#define GITS_CMD_VINVALL                 0x2d
#define GITS_CMD_INVDB                   0x2e

/* ITS registers, offsets from sgir_base */
#define GITS_SGIR                        0x00020

#define GITS_SGIR_VPEID                  GENMASK_ULL(47, 32)
#define GITS_SGIR_VINTID                 GENMASK_ULL(7, 0)

/* Describes the mapping of a VLPI */
struct its_vlpi_map {
    struct its_vm       *vm;
    unsigned int        vpe_idx;    /* Index of the VPE */
    uint32_t            vintid;     /* Virtual LPI number */
    bool                db_enabled; /* Is the VPE doorbell to be generated? */
    uint8_t             properties;
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
 * also use the above Valid, PendingLast and Dirty.
 */
#define GICR_VPENDBASER_4_1_VGRP1EN             (1ULL << 58)
#define GICR_VPENDBASER_4_1_VPEID               GENMASK_ULL(15, 0)
#define GICR_VPENDBASER_4_1_DB                  (1ULL << 62)

/*
 * GICv4.1 VPROPBASER reinvention. A subtle mix between the old
 * VPROPBASER and ITS_BASER.
 */
#define GICR_VPROPBASER_4_1_VALID               (1ULL << 63)
#define GICR_VPROPBASER_4_1_ENTRY_SIZE          GENMASK_ULL(61, 59)
#define GICR_VPROPBASER_4_1_INDIRECT            (1ULL << 55)
#define GICR_VPROPBASER_4_1_PAGE_SIZE           GENMASK_ULL(54, 53)
#define GICR_VPROPBASER_4_1_Z                   (1ULL << 52)
#define GICR_VPROPBASER_4_1_ADDR                GENMASK_ULL(51, 12)
#define GICR_VPROPBASER_4_1_SIZE                GENMASK_ULL(6, 0)

#define GICR_INVALLR_VPEID                      GENMASK_ULL(47, 32)
#define GICR_INVALLR_V                          (1UL << 63)

#define gits_read_vpropbaser(c)         readq_relaxed(c)
#define gits_write_vpropbaser(v, c)     {writeq_relaxed(v, c);}

/*
 * GICR_VPENDBASER - the Valid bit must be cleared before changing
 * anything else.
 */
static inline void gits_write_vpendbaser(uint64_t val, void __iomem *addr)
{
    uint64_t tmp;

    tmp = readq_relaxed(addr);
    while ( tmp & GICR_VPENDBASER_Valid )
    {
        tmp &= ~GICR_VPENDBASER_Valid;
        writeq_relaxed(tmp, addr);
        tmp = readq_relaxed(addr);
    }

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
