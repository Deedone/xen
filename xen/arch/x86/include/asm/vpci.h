/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef X86_VPCI_H
#define X86_VPCI_H

#include <xen/stdbool.h>

/* Arch-specific MSI data for vPCI. */
struct vpci_arch_msi {
    int pirq;
    bool bound;
};

/* Arch-specific MSI-X entry data for vPCI. */
struct vpci_arch_msix_entry {
    int pirq;
};

/* X86 does not require PCI BAR modifications */
static inline void platform_pci_fixup_bar(const struct pci_dev *pdev,
                                          unsigned int bar_num,
                                          paddr_t *addr)
{}

#endif /* X86_VPCI_H */

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
