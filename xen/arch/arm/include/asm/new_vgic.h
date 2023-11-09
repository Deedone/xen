/*
 * Copyright (C) 2015, 2016 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __ASM_ARM_NEW_VGIC_H
#define __ASM_ARM_NEW_VGIC_H

#include <asm/atomic.h>
#include <asm/mmio.h>
#include <xen/list.h>
#include <xen/mm.h>
#include <xen/spinlock.h>
#include <asm/gic_v3_defs.h>

#define VGIC_V3_MAX_CPUS        255
#define VGIC_V2_MAX_CPUS        8
#define VGIC_NR_SGIS            16
#define VGIC_NR_PPIS            16
#define VGIC_NR_PRIVATE_IRQS    (VGIC_NR_SGIS + VGIC_NR_PPIS)
#define VGIC_MAX_PRIVATE        (VGIC_NR_PRIVATE_IRQS - 1)
#define VGIC_MAX_SPI            1019
#define VGIC_MAX_RESERVED       1023
#define VGIC_MIN_LPI            8192
#define VGIC_V3_DIST_SIZE       SZ_64K
#define VGIC_V3_REDIST_SIZE     (2 * SZ_64K)
#define VGIC_V3_ITS_SIZE        (2 * SZ_64K)

#define irq_is_ppi(irq) ((irq) >= VGIC_NR_SGIS && (irq) < VGIC_NR_PRIVATE_IRQS)
#define irq_is_spi(irq) ((irq) >= VGIC_NR_PRIVATE_IRQS && \
                         (irq) <= VGIC_MAX_SPI)

enum vgic_type {
    VGIC_V2,        /* Good ol' GICv2 */
    VGIC_V3,        /* New fancy GICv3 */
};

#define VGIC_V2_MAX_LRS         (1 << 6)
#define VGIC_V3_MAX_LRS         16
#define VGIC_V3_LR_INDEX(lr)    (VGIC_V3_MAX_LRS - 1 - (lr))

#define VGIC_CONFIG_EDGE        false
#define VGIC_CONFIG_LEVEL       true

struct vgic_irq {
    struct list_head ap_list;

    struct vcpu *vcpu;          /*
                                 * SGIs and PPIs: The VCPU
                                 * SPIs and LPIs: The VCPU whose ap_list
                                 * this is queued on.
                                 */

    struct vcpu *target_vcpu;   /*
                                 * The VCPU that this interrupt should
                                 * be sent to, as a result of the
                                 * targets reg (v2) or the affinity reg (v3).
                                 */

    spinlock_t irq_lock;        /* Protects the content of the struct */
    uint32_t intid;             /* Guest visible INTID */
    atomic_t refcount;          /* Used for LPIs */
    uint32_t hwintid;           /* HW INTID number */
    union
    {
        struct {
            uint8_t targets;    /* GICv2 target VCPUs mask */
            uint8_t source;     /* GICv2 SGIs only */
        };
        uint32_t mpidr;         /* GICv3 target VCPU */
    };
    uint8_t priority;
    bool line_level:1;          /* Level only */
    bool pending_latch:1;       /*
                                 * The pending latch state used to
                                 * calculate the pending state for both
                                 * level and edge triggered IRQs.
                                 */
    bool active:1;              /* not used for LPIs */
    bool enabled:1;
    bool hw:1;                  /* Tied to HW IRQ */
    bool config:1;              /* Level or edge */
    struct list_head lpi_list;  /* Used to link all LPIs together */
};

enum iodev_type {
    IODEV_DIST,
    IODEV_REDIST,
    IODEV_ITS,
};

struct vgic_redist_region {
    uint32_t index;
    paddr_t base;
    uint32_t count; /* number of redistributors or 0 if single region */
    uint32_t free_index; /* index of the next free redistributor */
    struct list_head list;
};

struct vgic_io_device {
    gfn_t base_fn;
    struct vcpu *redist_vcpu;
    const struct vgic_register_region *regions;
    enum iodev_type iodev_type;
    unsigned int nr_regions;
    struct vgic_its *its;
};

struct vgic_its {
    /* The base address of the ITS control register frame */
    paddr_t vgic_its_base;

    bool enabled;
    struct vgic_io_device iodev;
    struct domain *domain;

    /* These registers correspond to GITS_BASER{0,1} */
    u64 baser_device_table;
    u64 baser_coll_table;

    /* Protects the command queue */
    struct spinlock cmd_lock;
    u64 cbaser;
    u32 creadr;
    u32 cwriter;

    /* migration ABI revision in use */
    u32 abi_rev;

    /* Protects the device and collection lists */
    struct spinlock its_lock;
    struct list_head device_list;
    struct list_head collection_list;
    paddr_t doorbell_address;
};

struct vgic_its_device {
    struct list_head dev_list;
    struct domain *d;

    /* the head for the list of ITTEs */
    struct list_head itt_head;
    u32 num_eventid_bits;
    void* itt_addr;
    struct host_its *hw_its;
    paddr_t guest_doorbell;             /* Identifies the virtual ITS */
    uint32_t host_devid;
    uint32_t guest_devid;
    uint32_t eventids;                  /* Number of event IDs (MSIs) */
    uint32_t *host_lpi_blocks;          /* Which LPIs are used on the host */
};

struct vgic_dist {
    bool                ready;
    bool                initialized;

    /* vGIC model the kernel emulates for the guest (GICv2 or GICv3) */
    uint32_t            version;

    /* Do injected MSIs require an additional device ID? */
    bool                msis_require_devid;

    unsigned int        nr_spis;

    /* base addresses in guest physical address space: */
    paddr_t             dbase;     /* distributor */
    union
    {
        /* either a GICv2 CPU interface */
        paddr_t         cbase;
        /* or a number of GICv3 redistributor regions */
        struct list_head rd_regions;
    };
    paddr_t             csize; /* CPU interface size */
    paddr_t             vbase; /* virtual CPU interface base address */

    /* for compatibility with guests DT creation*/
    uint32_t nr_regions;
    struct rdist_region *rdist_regions;

    /* distributor enabled */
    bool                enabled;

    struct vgic_irq     *spis;
    unsigned long       *allocated_irqs; /* bitmap of IRQs allocated */

    struct vgic_io_device   dist_iodev;

    bool                has_its;
    struct vgic_its     *its;

    /*
     * Contains the attributes and gpa of the LPI configuration table.
     * Since we report GICR_TYPER.CommonLPIAff as 0b00, we can share
     * one address across all redistributors.
     * GICv3 spec: 6.1.2 "LPI Configuration tables"
     */
    uint64_t            propbaser;
    spinlock_t          its_devices_lock; /* Protects the its_devices list */

    /* Protects the lpi_list and the count value below. */
    spinlock_t          lpi_list_lock;
    struct list_head    lpi_list_head;
    unsigned int        lpi_list_count;

    /* LPI translation cache */
    struct list_head	lpi_translation_cache;
};

struct vgic_cpu {
    struct vgic_irq private_irqs[VGIC_NR_PRIVATE_IRQS];

    struct list_head ap_list_head;
    spinlock_t ap_list_lock;    /* Protects the ap_list */

    unsigned int used_lrs;

    /*
     * List of IRQs that this VCPU should consider because they are either
     * Active or Pending (hence the name; AP list), or because they recently
     * were one of the two and need to be migrated off this list to another
     * VCPU.
     */

    /*
     * Members below are used with GICv3 emulation only and represent
     * parts of the redistributor.
     */
    struct vgic_io_device   rd_iodev;
    struct vgic_redist_region *rdreg;
    uint32_t rdreg_index;
    atomic_t syncr_busy;
    struct vgic_io_device   sgi_iodev;

    /* Contains the attributes and gpa of the LPI pending tables. */
    uint64_t pendbaser;

    bool lpis_enabled;

    /* Cache guest priority bits */
    uint32_t num_pri_bits;

    /* Cache guest interrupt ID bits */
    uint32_t num_id_bits;

    /* GICR_CTLR.{ENABLE_LPIS,RWP} */
    atomic_t ctlr;
};

static inline paddr_t vgic_cpu_base(const struct vgic_dist *vgic)
{
    return vgic->cbase;
}

static inline paddr_t vgic_dist_base(const struct vgic_dist *vgic)
{
    return vgic->dbase;
}

#ifdef CONFIG_HAS_ITS
void vgic_enable_lpis(struct vcpu *vcpu);
struct vgic_its_device *vgic_its_alloc_device(int nr_events);
void vgic_its_free_device(struct vgic_its_device *its_dev);
int vgic_its_add_device(struct domain *d, struct vgic_its_device *its_dev);
void vgic_its_delete_device(struct domain *d, struct vgic_its_device *its_dev);
struct vgic_its_device* vgic_its_get_device(struct domain *d, paddr_t vdoorbell,
                                         uint32_t vdevid);
#else
static inline void vgic_enable_lpis(struct vcpu *vcpu)
{
}
#endif

#endif /* __ASM_ARM_NEW_VGIC_H */

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
