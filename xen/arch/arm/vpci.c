/*
 * xen/arch/arm/vpci.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <xen/sched.h>
#include <xen/vpci.h>

#include <asm/mmio.h>

static pci_sbdf_t vpci_sbdf_from_gpa(uint16_t segment, uint8_t busn_start,
                                     paddr_t base_addr, paddr_t gpa)
{
    pci_sbdf_t sbdf;

    sbdf.sbdf = VPCI_ECAM_BDF(gpa - base_addr);
    sbdf.seg = segment;
    sbdf.bus += busn_start;
    return sbdf;
}

static int vpci_mmio_read(struct vcpu *v, mmio_info_t *info,
                          register_t *r, bool is_virt, pci_sbdf_t sbdf)
{
    /* data is needed to prevent a pointer cast on 32bit */
    unsigned long data;

#ifdef CONFIG_HAS_VPCI_GUEST_SUPPORT
    /*
     * For the passed through devices we need to map their virtual SBDF
     * to the physical PCI device being passed through.
     */
    if ( is_virt && !vpci_translate_virtual_device(v->domain, &sbdf) )
        return 1;
#endif

    if ( vpci_ecam_read(sbdf, ECAM_REG_OFFSET(info->gpa),
                        1U << info->dabt.size, &data) )
    {
        *r = data;
        return 1;
    }

    *r = ~0ul;

    return 0;
}

static int vpci_mmio_read_root(struct vcpu *v, mmio_info_t *info,
                          register_t *r, void *p)
{
    struct pci_host_bridge *bridge = p;
    pci_sbdf_t sbdf;

    if ( bridge )
        sbdf = vpci_sbdf_from_gpa(bridge->segment,
                                  bridge->cfg->busn_start,
                                  bridge->cfg->phys_addr,
                                  info->gpa);
    else
        sbdf = vpci_sbdf_from_gpa(0, 0, GUEST_VPCI_ECAM_BASE, info->gpa);

    return vpci_mmio_read(v, info, r, !bridge, sbdf);
}

static int vpci_mmio_read_child(struct vcpu *v, mmio_info_t *info,
                          register_t *r, void *p)
{
    struct pci_host_bridge *bridge = p;
    pci_sbdf_t sbdf = vpci_sbdf_from_gpa(bridge->segment,
                                         bridge->child_cfg->busn_start,
                                         bridge->child_cfg->phys_addr,
                                         info->gpa);

    return vpci_mmio_read(v, info, r, !bridge, sbdf);
}


static int vpci_mmio_write(struct vcpu *v, mmio_info_t *info,
                           register_t r, bool is_virt, pci_sbdf_t sbdf)
{
#ifdef CONFIG_HAS_VPCI_GUEST_SUPPORT
    /*
     * For the passed through devices we need to map their virtual SBDF
     * to the physical PCI device being passed through.
     */
    if ( is_virt && !vpci_translate_virtual_device(v->domain, &sbdf) )
        return 1;
#endif

    return vpci_ecam_write(sbdf, ECAM_REG_OFFSET(info->gpa),
                           1U << info->dabt.size, r);
}

static int vpci_mmio_write_root(struct vcpu *v, mmio_info_t *info,
                                register_t r, void *p)
{
    struct pci_host_bridge *bridge = p;
    pci_sbdf_t sbdf;

    if ( bridge )
        sbdf = vpci_sbdf_from_gpa(bridge->segment,
                                  bridge->cfg->busn_start,
                                  bridge->cfg->phys_addr,
                                  info->gpa);
    else
        sbdf = vpci_sbdf_from_gpa(0, 0, GUEST_VPCI_ECAM_BASE, info->gpa);

    return vpci_mmio_write(v, info, r, !bridge, sbdf);
}

static int vpci_mmio_write_child(struct vcpu *v, mmio_info_t *info,
                                register_t r, void *p)
{
    struct pci_host_bridge *bridge = p;
    pci_sbdf_t sbdf = vpci_sbdf_from_gpa(bridge->segment,
                                         bridge->child_cfg->busn_start,
                                         bridge->child_cfg->phys_addr,
                                         info->gpa);

    return vpci_mmio_write(v, info, r, !bridge, sbdf);
}

static const struct mmio_handler_ops vpci_mmio_handler = {
    .read  = vpci_mmio_read_root,
    .write = vpci_mmio_write_root,
};

static const struct mmio_handler_ops vpci_mmio_handler_child = {
    .read  = vpci_mmio_read_child,
    .write = vpci_mmio_write_child,
};

static int vpci_setup_mmio_handler_cb(struct domain *d,
                                      struct pci_host_bridge *bridge)
{
    struct pci_config_window *cfg = bridge->cfg;
    int count = 1;

    register_mmio_handler(d, &vpci_mmio_handler,
                          cfg->phys_addr, cfg->size, bridge);

    if ( bridge->child_ops )
    {
        struct pci_config_window *cfg = bridge->child_cfg;

        register_mmio_handler(d, &vpci_mmio_handler_child,
                              cfg->phys_addr, cfg->size, bridge);
        count++;
    }

    return count;
}

int domain_vpci_init(struct domain *d)
{
    if ( !has_vpci(d) )
        return 0;

    /*
     * The hardware domain gets as many MMIOs as required by the
     * physical host bridge.
     * Guests get the virtual platform layout: one virtual host bridge for now.
     */
    if ( is_hardware_domain(d) )
    {
        int count;

        count = pci_host_iterate_bridges_and_count(d, vpci_setup_mmio_handler_cb);
        if ( count < 0 )
            return count;

        return 0;
    }

    register_mmio_handler(d, &vpci_mmio_handler,
                          GUEST_VPCI_ECAM_BASE, GUEST_VPCI_ECAM_SIZE, NULL);

    return 0;
}

static int vpci_get_num_handlers_cb(struct domain *d,
                                    struct pci_host_bridge *bridge)
{
    int count = 1;

    if ( bridge->child_cfg )
        count++;

    return count;
}

unsigned int domain_vpci_get_num_mmio_handlers(struct domain *d)
{
    unsigned int count;

    if ( !has_vpci(d) )
        return 0;

    if ( is_hardware_domain(d) )
    {
        int ret = pci_host_iterate_bridges_and_count(d, vpci_get_num_handlers_cb);

        return ret < 0 ? 0 : ret;
    }

    /* For a single emulated host bridge's configuration space. */
    count = 1;

#ifdef CONFIG_HAS_PCI_MSI
    /*
     * There's a single MSI-X MMIO handler that deals with both PBA
     * and MSI-X tables per each PCI device being passed through.
     * Maximum number of emulated virtual devices is VPCI_MAX_VIRT_DEV.
     */
    count += VPCI_MAX_VIRT_DEV;
#endif

    return count;
}

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */

