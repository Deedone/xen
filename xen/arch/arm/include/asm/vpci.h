/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef ARM_VPCI_H
#define ARM_VPCI_H

struct domain;

#ifdef CONFIG_HAS_VPCI
/* Arch-specific MSI data for vPCI. */
struct vpci_arch_msi {
};

/* Arch-specific MSI-X entry data for vPCI. */
struct vpci_arch_msix_entry {
};


int domain_vpci_init(struct domain *d);
unsigned int domain_vpci_get_num_mmio_handlers(struct domain *d);

void platform_pci_fixup_bar(const struct pci_dev *pdev, unsigned int bar_num,
                            paddr_t *addr);

#else
static inline int domain_vpci_init(struct domain *d)
{
    return 0;
}

static inline unsigned int domain_vpci_get_num_mmio_handlers(struct domain *d)
{
    return 0;
}

static inline void platform_pci_fixup_bar(const struct pci_dev *pdev,
                                          unsigned int bar_num,
                                          paddr_t *addr)
{}
#endif /* CONFIG_HAS_VPCI */

#endif /* ARM_VPCI_H */
