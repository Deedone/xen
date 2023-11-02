/*
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <asm/new_vgic.h>
#include <asm/gic_v3_defs.h>
#include <asm/gic.h>
#include <xen/sched.h>

#include "vgic.h"

static struct {
    bool enabled;
    /* Distributor interface address */
    paddr_t dbase;
    /* Re-distributor regions */
    unsigned int nr_rdist_regions;
    const struct rdist_region *regions;
    unsigned int intid_bits; /* Number of interrupt ID bits */
} vgic_v3_hw_data;

void vgic_v3_setup_hw(paddr_t dbase, unsigned int nr_rdist_regions,
                      const struct rdist_region *regions,
                      unsigned int intid_bits)
{
    vgic_v3_hw_data.enabled          = true;
    vgic_v3_hw_data.dbase            = dbase;
    vgic_v3_hw_data.nr_rdist_regions = nr_rdist_regions;
    vgic_v3_hw_data.regions          = regions;
    vgic_v3_hw_data.intid_bits       = intid_bits;
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
    unsigned int used_lrs     = vcpu->arch.vgic.used_lrs;
    unsigned long flags;
    unsigned int lr;

    if ( !used_lrs ) /* No LRs used, so nothing to sync back here. */
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
        irq   = vgic_get_irq(vcpu->domain, vcpu, intid);

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
    struct gic_lr lr_val = { 0 };

    lr_val.virq          = irq->intid;

    if ( irq_is_pending(irq) )
    {
        lr_val.pending = true;

        if ( irq->config == VGIC_CONFIG_EDGE )
            irq->pending_latch = false;

        if ( vgic_irq_is_sgi(irq->intid) &&
             vcpu->domain->arch.vgic.version == VGIC_V2 )
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
        lr_val.hw.pirq   = irq->hwintid;
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
    return domain_use_host_layout(d) ? vgic_v3_hw_data.nr_rdist_regions
                                     : GUEST_GICV3_RDIST_REGIONS;
}

void vgic_v3_enable(struct vcpu *vcpu)
{
    /* Get the show on the road... */
    gic_hw_ops->update_hcr_status(GICH_HCR_EN, true);
}

int vgic_v3_map_resources(struct domain *d)
{
    int rdist_count, i, ret;

    /* Allocate memory for Re-distributor regions */
    rdist_count = vgic_v3_max_rdist_count(d);

    /*
     * For domain using the host memory layout, it gets the hardware
     * address.
     * Other domains get the virtual platform layout.
     */
    if ( domain_use_host_layout(d) )
    {
        d->arch.vgic.dbase = vgic_v3_hw_data.dbase;

        for ( i = 0; i < vgic_v3_hw_data.nr_rdist_regions; i++ )
        {
            vgic_v3_set_redist_base(d, i, vgic_v3_hw_data.regions[i].base,
                                    vgic_v3_hw_data.regions[i].size /
                                        GICV3_GICR_SIZE);
        }
    }
    else
    {
        d->arch.vgic.dbase = GUEST_GICV3_GICD_BASE;

        /* A single Re-distributor region is mapped for the guest. */
        BUILD_BUG_ON(GUEST_GICV3_RDIST_REGIONS != 1);

        /* The first redistributor should contain enough space for all CPUs */
        BUILD_BUG_ON((GUEST_GICV3_GICR0_SIZE / GICV3_GICR_SIZE) <
                     MAX_VIRT_CPUS);
        vgic_v3_set_redist_base(d, 0, GUEST_GICV3_GICR0_BASE,
                                GUEST_GICV3_GICR0_SIZE / GICV3_GICR_SIZE);
    }

    /* Register mmio handle for the Distributor */
    ret =
        vgic_register_dist_iodev(d, gaddr_to_gfn(d->arch.vgic.dbase), VGIC_V3);

    d->arch.vgic.ready = true;

    return 0;
}
