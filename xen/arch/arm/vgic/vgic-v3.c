
#include <asm/new_vgic.h>
#include <asm/gic_v3_defs.h>
#include <asm/gic.h>
#include <xen/bug.h>
#include <xen/sched.h>
#include <xen/sizes.h>

#include "vgic.h"


static struct {
    bool enabled;
    /* Distributor interface address */
    paddr_t dbase;
    /* Re-distributor regions */
    unsigned int nr_rdist_regions;
    const struct rdist_region *regions;
    unsigned int intid_bits;  /* Number of interrupt ID bits */
} vgic_v3_hw_data;

void vgic_v3_setup_hw(paddr_t dbase,
                      unsigned int nr_rdist_regions,
                      const struct rdist_region *regions,
                      unsigned int intid_bits)
{
    vgic_v3_hw_data.enabled = true;
    vgic_v3_hw_data.dbase = dbase;
    vgic_v3_hw_data.nr_rdist_regions = nr_rdist_regions;
    vgic_v3_hw_data.regions = regions;
    vgic_v3_hw_data.intid_bits = intid_bits;
}

/*
 * transfer the content of the LRs back into the corresponding ap_list:
 * - active bit is transferred as is
 * - pending bit is
 *   - transferred as is in case of edge sensitive IRQs
 *   - set to the line-level (resample time) for level sensitive IRQs
 */
void vgic_v3_fold_lr_state(struct vcpu *vcpu)
{
    struct vgic_cpu *vgic_cpu = &vcpu->arch.vgic;
    unsigned int used_lrs = vcpu->arch.vgic.used_lrs;
    unsigned long flags;
    unsigned int lr;

    if ( !used_lrs )    /* No LRs used, so nothing to sync back here. */
        return;

    gic_hw_ops->update_hcr_status(GICH_HCR_UIE, false);

    for ( lr = 0; lr < used_lrs; lr++ )
    {
        struct gic_lr lr_val;
        uint32_t intid;
        struct vgic_irq *irq;
        struct irq_desc *desc = NULL;

        gic_hw_ops->read_lr(lr, &lr_val);

        /*
         * TODO: Possible optimization to avoid reading LRs:
         * Read the ELRSR to find out which of our LRs have been cleared
         * by the guest. We just need to know the IRQ number for those, which
         * we could save in an array when populating the LRs.
         * This trades one MMIO access (ELRSR) for possibly more than one (LRs),
         * but requires some more code to save the IRQ number and to handle
         * those finished IRQs according to the algorithm below.
         * We need some numbers to justify this: chances are that we don't
         * have many LRs in use most of the time, so we might not save much.
         */
        gic_hw_ops->clear_lr(lr);

        intid = lr_val.virq;
        irq = vgic_get_irq(vcpu->domain, vcpu, intid);

        local_irq_save(flags);

        /*
         * We check this here without taking the lock, because the locking
         * order forces us to do so. irq->hw is a "write-once" member, so
         * whenever we read true, the associated hardware IRQ will not go
         * away anymore.
         * TODO: rework this if possible, either by using the desc pointer
         * directly in struct vgic_irq or by changing the locking order.
         * Especially if we ever drop the assumption above.
         */
        if ( irq->hw )
        {
            desc = irq_to_desc(irq->hwintid);
            spin_lock(&desc->lock);
        }

        spin_lock(&irq->irq_lock);

        /*
         * If a hardware mapped IRQ has been handled for good, we need to
         * clear the _IRQ_INPROGRESS bit to allow handling of new IRQs.
         *
         * TODO: This is probably racy, but is so already in the existing
         * VGIC. A fix does not seem to be trivial.
         */
        if ( irq->hw && !lr_val.active && !lr_val.pending )
            clear_bit(_IRQ_INPROGRESS, &desc->status);

        /* Always preserve the active bit */
        irq->active = lr_val.active;

        /* Edge is the only case where we preserve the pending bit */
        if ( irq->config == VGIC_CONFIG_EDGE && lr_val.pending )
        {
            irq->pending_latch = true;

            if ( vgic_irq_is_sgi(intid) )
                irq->source |= (1U << lr_val.virt.source);
        }

        /* Clear soft pending state when level irqs have been acked. */
        if ( irq->config == VGIC_CONFIG_LEVEL && !lr_val.pending )
            irq->pending_latch = false;

        /*
         * Level-triggered mapped IRQs are special because we only
         * observe rising edges as input to the VGIC.
         *
         * If the guest never acked the interrupt we have to sample
         * the physical line and set the line level, because the
         * device state could have changed or we simply need to
         * process the still pending interrupt later.
         *
         * If this causes us to lower the level, we have to also clear
         * the physical active state, since we will otherwise never be
         * told when the interrupt becomes asserted again.
         */
        if ( vgic_irq_is_mapped_level(irq) && lr_val.pending )
        {
            ASSERT(irq->hwintid >= VGIC_NR_PRIVATE_IRQS);

            irq->line_level = gic_read_pending_state(desc);

            if ( !irq->line_level )
                gic_set_active_state(desc, false);
        }

        spin_unlock(&irq->irq_lock);
        if ( desc )
            spin_unlock(&desc->lock);
        local_irq_restore(flags);

        vgic_put_irq(vcpu->domain, irq);
    }

    gic_hw_ops->update_hcr_status(GICH_HCR_EN, false);
    vgic_cpu->used_lrs = 0;
}


/* Requires the irq to be locked already */
void vgic_v3_populate_lr(struct vcpu *vcpu, struct vgic_irq *irq, int lr)
{    
    struct gic_lr lr_val = {0};

    lr_val.virq = irq->intid;

    if ( irq_is_pending(irq) )
    {
        lr_val.pending = true;

        if ( irq->config == VGIC_CONFIG_EDGE )
            irq->pending_latch = false;

        //TODO FIX
        if ( vgic_irq_is_sgi(irq->intid) && /* MODEL IS V2 BUT NOT FOR NOW*/ 0 )
        {
            uint32_t src = ffs(irq->source);

            BUG_ON(!src);
            lr_val.virt.source = (src - 1);
            irq->source &= ~(1 << (src - 1));
            if ( irq->source )
                irq->pending_latch = true;
        }
    }

    lr_val.active = irq->active;

    if ( irq->hw )
    {
        lr_val.hw_status = true;
        lr_val.hw.pirq = irq->hwintid;
        /*
         * Never set pending+active on a HW interrupt, as the
         * pending state is kept at the physical distributor
         * level.
         */
        if ( irq->active && irq_is_pending(irq) )
            lr_val.pending = false;
    }
    else
    {
        if ( irq->config == VGIC_CONFIG_LEVEL )
            lr_val.virt.eoi = true;
    }

    /*
     * Level-triggered mapped IRQs are special because we only observe
     * rising edges as input to the VGIC.  We therefore lower the line
     * level here, so that we can take new virtual IRQs.  See
     * vgic_v2_fold_lr_state for more info.
     */
    if ( vgic_irq_is_mapped_level(irq) && lr_val.pending )
        irq->line_level = false;

    /* The GICv2 LR only holds five bits of priority. */
    lr_val.priority = irq->priority >> 3;

    gic_hw_ops->write_lr(lr, &lr_val);
}

static bool vgic_v3_redist_region_full(struct vgic_redist_region *region)
{
	if (!region->count)
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
struct vgic_redist_region *vgic_v3_rdist_free_slot(struct list_head *rd_regions)
{
	struct vgic_redist_region *rdreg;

	list_for_each_entry(rdreg, rd_regions, list) {
		if (!vgic_v3_redist_region_full(rdreg))
			return rdreg;
	}
	return NULL;
}

unsigned int vgic_v3_max_rdist_count(const struct domain *d)
{
    /*
     * Normally there is only one GICv3 redistributor region.
     * The GICv3 DT binding provisions for multiple regions, since there are
     * platforms out there which need those (multi-socket systems).
     * For domain using the host memory layout, we have to live with the MMIO
     * layout the hardware provides, so we have to copy the multiple regions
     * - as the first region may not provide enough space to hold all
     * redistributors we need.
     * All the other domains will get a constructed memory map, so we can go
     * with the architected single redistributor region.
     */
    return domain_use_host_layout(d) ? vgic_v3_hw_data.nr_rdist_regions :
                                       GUEST_GICV3_RDIST_REGIONS;
}

int vgic_register_redist_iodev(struct vcpu *vcpu);
void vgic_v3_enable(struct vcpu *vcpu)
{

    /* Get the show on the road... */
    vgic_register_redist_iodev(vcpu);
    gic_hw_ops->update_hcr_status(GICH_HCR_EN, true);
}

int vgic_v3_map_resources(struct domain *d)
{ 
    struct vgic_redist_region *rdist_regions;
    int rdist_count, i, ret;

    /* Allocate memory for Re-distributor regions */
    rdist_count = vgic_v3_max_rdist_count(d);

    // rdist_regions = xzalloc_array(struct vgic_redist_region, rdist_count);
    // if ( !rdist_regions )
    //     return -ENOMEM;

    // d->arch.vgic.nr_regions = rdist_count;
    // d->arch.vgic.rdist_regions = rdist_regions;

    // rwlock_init(&d->arch.vgic.pend_lpi_tree_lock);
    // radix_tree_init(&d->arch.vgic.pend_lpi_tree);

    /*
     * For domain using the host memory layout, it gets the hardware
     * address.
     * Other domains get the virtual platform layout.
     */
    if ( domain_use_host_layout(d) )
    {
        unsigned int first_cpu = 0;

        d->arch.vgic.dbase = vgic_v3_hw_data.dbase;

        for ( i = 0; i < vgic_v3_hw_data.nr_rdist_regions; i++ )
        {
            vgic_v3_set_redist_base(d, i, vgic_v3_hw_data.regions[i].base, 
            vgic_v3_hw_data.regions[i].size / GICV3_GICR_SIZE);

            // paddr_t size = vgic_v3_hw_data.regions[i].size;

            // d->arch.vgic.rdist_regions[i].base = vgic_v3_hw.regions[i].base;
            // d->arch.vgic.rdist_regions[i].size = size;

            // /* Set the first CPU handled by this region */
            // d->arch.vgic.rdist_regions[i].first_cpu = first_cpu;

            // first_cpu += size / GICV3_GICR_SIZE;

            // if ( first_cpu >= d->max_vcpus )
            //     break;
        }

        /*
         * For domain using the host memory layout, it may not use all
         * the re-distributors regions (e.g when the number of vCPUs does
         * not match the number of pCPUs). Update the number of regions to
         * avoid exposing unused region as they will not get emulated.
         */
        // d->arch.vgic.nr_regions = i + 1;

        // d->arch.vgic.intid_bits = vgic_v3_hw_data.intid_bits;
    }
    else
    {
        d->arch.vgic.dbase = GUEST_GICV3_GICD_BASE;

        /* A single Re-distributor region is mapped for the guest. */
        BUILD_BUG_ON(GUEST_GICV3_RDIST_REGIONS != 1);

        /* The first redistributor should contain enough space for all CPUs */
        BUILD_BUG_ON((GUEST_GICV3_GICR0_SIZE / GICV3_GICR_SIZE) < MAX_VIRT_CPUS);
        // d->arch.vgic.rdist_regions[0].base = GUEST_GICV3_GICR0_BASE;
        // d->arch.vgic.rdist_regions[0].size = GUEST_GICV3_GICR0_SIZE;
        // d->arch.vgic.rdist_regions[0].first_cpu = 0;
        vgic_v3_set_redist_base(d, 0, GUEST_GICV3_GICR0_BASE,
                                GUEST_GICV3_GICR0_SIZE / GICV3_GICR_SIZE);

        /*
         * TODO: only SPIs for now, adjust this when guests need LPIs.
         * Please note that this value just describes the bits required
         * in the stream interface, which is of no real concern for our
         * emulation. So we just go with "10" here to cover all eventual
         * SPIs (even if the guest implements less).
         */
        //d->arch.vgic.intid_bits = 10;
    }

    //TODO FIX
    printk(XENLOG_ERR "vgic v3 should init its\n");
    // ret = vgic_v3_its_init_domain(d);
    // if ( ret )
    //     return ret;

    /* Register mmio handle for the Distributor */
    // register_mmio_handler(d, &vgic_distr_mmio_handler, d->arch.vgic.dbase,
    //                       SZ_64K, NULL);
    ret = vgic_register_dist_iodev(d, gaddr_to_gfn(d->arch.vgic.dbase), VGIC_V3);

    /*
     * Register mmio handler per contiguous region occupied by the
     * redistributors. The handler will take care to choose which
     * redistributor is targeted.
     */
    // for ( i = 0; i < d->arch.vgic.nr_regions; i++ )
    // {
    //     struct vgic_rdist_region *region = &d->arch.vgic.rdist_regions[i];

    //     register_mmio_handler(d, &vgic_rdistr_mmio_handler,
    //                           region->base, region->size, region);
    // }

    d->arch.vgic.ready = true;

    return 0;
}
