/*
 * Copyright (C) 2015, 2016 ARM Ltd.
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
#ifndef __XEN_ARM_VGIC_VGIC_H__
#define __XEN_ARM_VGIC_VGIC_H__

/*
 * We piggy-back on the already used KVM product ID,  but use a different
 * variant (major revision) for Xen.
 */
#define PRODUCT_ID_KVM          0x4b        /* ASCII code K */
#define VARIANT_ID_XEN          0x01
#define IMPLEMENTER_ARM         0x43b

#define VGIC_ADDR_UNDEF     INVALID_PADDR
#define IS_VGIC_ADDR_UNDEF(_x)  ((_x) == VGIC_ADDR_UNDEF)

#define INTERRUPT_ID_BITS_SPIS  10
#define VGIC_PRI_BITS       5

#define vgic_irq_is_sgi(intid) ((intid) < VGIC_NR_SGIS)

static inline bool irq_is_pending(struct vgic_irq *irq)
{
    if ( irq->config == VGIC_CONFIG_EDGE )
        return irq->pending_latch;
    else
        return irq->pending_latch || irq->line_level;
}

static inline bool vgic_irq_is_mapped_level(struct vgic_irq *irq)
{
    return irq->config == VGIC_CONFIG_LEVEL && irq->hw;
}

struct vgic_irq *vgic_get_irq(struct domain *d, struct vcpu *vcpu,
                              uint32_t intid);
void vgic_put_irq(struct domain *d, struct vgic_irq *irq);
void vgic_queue_irq_unlock(struct domain *d, struct vgic_irq *irq,
                           unsigned long flags);
void vgic_kick_vcpus(struct domain *d);

static inline void vgic_get_irq_kref(struct vgic_irq *irq)
{
    if ( irq->intid < VGIC_MIN_LPI )
        return;

    atomic_inc(&irq->refcount);
}

void vgic_sync_hardware_irq(struct domain *d,
                            irq_desc_t *desc, struct vgic_irq *irq);

void vgic_v2_fold_lr_state(struct vcpu *vcpu);
void vgic_v2_populate_lr(struct vcpu *vcpu, struct vgic_irq *irq, int lr);
void vgic_v2_set_underflow(struct vcpu *vcpu);
void vgic_v2_enable(struct vcpu *vcpu);
int vgic_v2_map_resources(struct domain *d);
int vgic_register_dist_iodev(struct domain *d, gfn_t dist_base_fn,
                             enum vgic_type);
#ifdef CONFIG_GICV3
void vgic_v3_fold_lr_state(struct vcpu *vcpu);
void vgic_v3_populate_lr(struct vcpu *vcpu, struct vgic_irq *irq, int lr);
void vgic_v3_enable(struct vcpu *vcpu);
int vgic_v3_map_resources(struct domain *d);
bool vgic_v3_emulate_reg(struct cpu_user_regs *regs, union hsr hsr);
unsigned int vgic_v3_init_dist_iodev(struct vgic_io_device *dev);
int vgic_v3_set_redist_base(struct domain *d, u32 index, u64 addr, u32 count);
int vgic_register_redist_iodev(struct vcpu *vcpu);
#else
static inline void vgic_v3_fold_lr_state(struct vcpu *vcpu)
{
}
static inline void vgic_v3_populate_lr(struct vcpu *vcpu, struct vgic_irq *irq, int lr)
{
}
static inline void vgic_v3_enable(struct vcpu *vcpu)
{
}
static inline int vgic_v3_map_resources(struct domain *d)
{
    return 0;
}
static inline unsigned int vgic_v3_init_dist_iodev(struct vgic_io_device *dev)
{
    return 0;
}
static inline bool vgic_v3_emulate_reg(struct cpu_user_regs *regs, union hsr hsr)
{
    return false;
}
static inline int vgic_v3_set_redist_base(struct domain *d, u32 index, u64 addr, u32 count)
{
    return 0;
}
static inline int vgic_register_redist_iodev(struct vcpu *vcpu)
{
    return 0;
}
#endif /* CONFIG_GICV3 */

#endif /* __XEN_ARM_VGIC_VGIC_H__ */

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
