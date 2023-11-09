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
#ifndef __XEN_ARM_VGIC_VGIC_MMIO_H__
#define __XEN_ARM_VGIC_VGIC_MMIO_H__

struct vgic_register_region {
    unsigned int reg_offset;
    unsigned int len;
    unsigned int bits_per_irq;
    unsigned int access_flags;

    union {
    unsigned long (*read)(struct vcpu *vcpu, paddr_t addr,
                          unsigned int len);
    unsigned long (*its_read)(struct domain *d, struct vgic_its *its,
                    paddr_t addr, unsigned int len);
    };

    union {
    void (*write)(struct vcpu *vcpu, paddr_t addr,
                  unsigned int len, unsigned long val);
    void (*its_write)(struct domain *d, struct vgic_its *its,
                paddr_t addr, unsigned int len,
                unsigned long val);
    };
};

extern struct mmio_handler_ops vgic_io_ops;

#define VGIC_ACCESS_8bit    1
#define VGIC_ACCESS_32bit   2
#define VGIC_ACCESS_64bit   4

/*
 * Generate a mask that covers the number of bytes required to address
 * up to 1024 interrupts, each represented by <bits> bits. This assumes
 * that <bits> is a power of two.
 */
#define VGIC_ADDR_IRQ_MASK(bits) (((bits) * 1024 / 8) - 1)

/*
 * (addr & mask) gives us the _byte_ offset for the INT ID.
 * We multiply this by 8 the get the _bit_ offset, then divide this by
 * the number of bits to learn the actual INT ID.
 * But instead of a division (which requires a "long long div" implementation),
 * we shift by the binary logarithm of <bits>.
 * This assumes that <bits> is a power of two.
 */
#define VGIC_ADDR_TO_INTID(addr, bits)  (((addr) & VGIC_ADDR_IRQ_MASK(bits)) * \
                                         8 >> ilog2(bits))

/*
 * Some VGIC registers store per-IRQ information, with a different number
 * of bits per IRQ. For those registers this macro is used.
 * The _WITH_LENGTH version instantiates registers with a fixed length
 * and is mutually exclusive with the _PER_IRQ version.
 */
#define REGISTER_DESC_WITH_BITS_PER_IRQ(off, rd, wr, bpi, acc)  \
    {                                                           \
        .reg_offset = off,                                      \
        .bits_per_irq = bpi,                                    \
        .len = bpi * 1024 / 8,                                  \
        .access_flags = acc,                                    \
        .read = rd,                                             \
        .write = wr,                                            \
    }

#define REGISTER_DESC_WITH_LENGTH(off, rd, wr, length, acc)     \
    {                                                           \
        .reg_offset = off,                                      \
        .bits_per_irq = 0,                                      \
        .len = length,                                          \
        .access_flags = acc,                                    \
        .read = rd,                                             \
        .write = wr,                                            \
    }

unsigned long vgic_mmio_read_raz(struct vcpu *vcpu,
                                 paddr_t addr, unsigned int len);

unsigned long vgic_mmio_read_rao(struct vcpu *vcpu,
                                 paddr_t addr, unsigned int len);

void vgic_mmio_write_wi(struct vcpu *vcpu, paddr_t addr,
                        unsigned int len, unsigned long val);

unsigned long vgic_mmio_read_enable(struct vcpu *vcpu,
                                    paddr_t addr, unsigned int len);

void vgic_mmio_write_senable(struct vcpu *vcpu,
                             paddr_t addr, unsigned int len,
                             unsigned long val);

void vgic_mmio_write_cenable(struct vcpu *vcpu,
                             paddr_t addr, unsigned int len,
                             unsigned long val);

unsigned long vgic_mmio_read_pending(struct vcpu *vcpu,
                                     paddr_t addr, unsigned int len);

void vgic_mmio_write_spending(struct vcpu *vcpu,
                              paddr_t addr, unsigned int len,
                              unsigned long val);

void vgic_mmio_write_cpending(struct vcpu *vcpu,
                              paddr_t addr, unsigned int len,
                              unsigned long val);

unsigned long vgic_mmio_read_active(struct vcpu *vcpu,
                                    paddr_t addr, unsigned int len);

void vgic_mmio_write_cactive(struct vcpu *vcpu,
                             paddr_t addr, unsigned int len,
                             unsigned long val);

void vgic_mmio_write_sactive(struct vcpu *vcpu,
                             paddr_t addr, unsigned int len,
                             unsigned long val);

unsigned long vgic_mmio_read_priority(struct vcpu *vcpu,
                      paddr_t addr, unsigned int len);

void vgic_mmio_write_priority(struct vcpu *vcpu,
                  paddr_t addr, unsigned int len,
                  unsigned long val);

unsigned long vgic_mmio_read_config(struct vcpu *vcpu,
                    paddr_t addr, unsigned int len);

void vgic_mmio_write_config(struct vcpu *vcpu,
                paddr_t addr, unsigned int len,
                unsigned long val);

unsigned int vgic_v2_init_dist_iodev(struct vgic_io_device *dev);

/* extract @num bytes at @offset bytes offset in data */
unsigned long extract_bytes(uint64_t data, unsigned int offset,
			    unsigned int num);

uint64_t update_64bit_reg(u64 reg, unsigned int offset, unsigned int len,
		     unsigned long val);

#ifdef CONFIG_HAS_ITS
int vgic_its_inv_lpi(struct domain *d, struct vgic_irq *irq);
int vgic_its_invall(struct vcpu *vcpu);
void vgic_its_invalidate_cache(struct domain *d);
#else
static inline int vgic_its_inv_lpi(struct domain *d, struct vgic_irq *irq)
{
    return 0;
}

static inline int vgic_its_invall(struct vcpu *vcpu)
{
    return 0;
}

static inline void vgic_its_invalidate_cache(struct domain *d)
{
}
#endif

#endif
