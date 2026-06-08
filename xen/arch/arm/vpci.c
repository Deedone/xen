/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * xen/arch/arm/vpci.c
 */
#include <xen/lib.h>
#include <xen/sched.h>
#include <xen/vpci.h>
#include <xen/domain-layout.h>
#include <xen/iocap.h>

#include <asm/mmio.h>

static pci_sbdf_t vpci_sbdf_from_gpa(struct domain *d,
                                     const struct pci_host_bridge *bridge,
                                     paddr_t gpa, bool use_root)
{
    pci_sbdf_t sbdf;

    if ( !has_vpci_bridge(d) )
    {
        const struct pci_config_window *cfg = use_root ? bridge->cfg
                                                       : bridge->child_cfg;
        sbdf.sbdf = VPCI_ECAM_BDF(gpa - cfg->phys_addr);
        sbdf.seg = bridge->segment;
        sbdf.bus += cfg->busn_start;
    }
    else
    {
        paddr_t start = domain_use_host_layout(d) ? bridge->cfg->phys_addr :
                                                    GUEST_VPCI_ECAM_BASE;
        sbdf.sbdf = VPCI_ECAM_BDF(gpa - start);
    }

    return sbdf;
}

static int vpci_mmio_read(struct vcpu *v, mmio_info_t *info, register_t *r,
                          pci_sbdf_t sbdf)
{
    const unsigned int access_size = (1U << info->dabt.size) * 8;
    const register_t invalid = GENMASK_ULL(access_size - 1, 0);
    /* data is needed to prevent a pointer cast on 32bit */
    unsigned long data;

    if ( vpci_ecam_read(sbdf, ECAM_REG_OFFSET(info->gpa),
                        1U << info->dabt.size, &data) )
    {
        *r = data & invalid;
        return 1;
    }

    *r = invalid;

    return 0;
}

static int vpci_mmio_read_root(struct vcpu *v, mmio_info_t *info, register_t *r,
                               void *p)
{
    struct pci_host_bridge *bridge = p;
    pci_sbdf_t sbdf = vpci_sbdf_from_gpa(v->domain, bridge, info->gpa, true);
    
    return vpci_mmio_read(v, info, r, sbdf);
}

static int vpci_mmio_read_child(struct vcpu *v, mmio_info_t *info,
                                register_t *r, void *p)
{
    struct pci_host_bridge *bridge = p;
    pci_sbdf_t sbdf = vpci_sbdf_from_gpa(v->domain, bridge, info->gpa, false);

    return vpci_mmio_read(v, info, r, sbdf);
}

static int vpci_mmio_write(struct vcpu *v, mmio_info_t *info, register_t r,
                           pci_sbdf_t sbdf)
{
    return vpci_ecam_write(sbdf, ECAM_REG_OFFSET(info->gpa),
                           1U << info->dabt.size, r);
}

static int vpci_mmio_write_root(struct vcpu *v, mmio_info_t *info, register_t r,
                                void *p)
{
    struct pci_host_bridge *bridge = p;
    pci_sbdf_t sbdf = vpci_sbdf_from_gpa(v->domain, bridge, info->gpa, true);

    return vpci_mmio_write(v, info, r, sbdf);
}

static int vpci_mmio_write_child(struct vcpu *v, mmio_info_t *info,
                                 register_t r, void *p)
{
    struct pci_host_bridge *bridge = p;
    pci_sbdf_t sbdf = vpci_sbdf_from_gpa(v->domain, bridge, info->gpa, false);

    return vpci_mmio_write(v, info, r, sbdf);
}

static const struct mmio_handler_ops vpci_mmio_handler = {
    .read = vpci_mmio_read_root,
    .write = vpci_mmio_write_root,
};

static const struct mmio_handler_ops vpci_mmio_handler_child = {
    .read = vpci_mmio_read_child,
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
        struct pci_config_window *child_cfg = bridge->child_cfg;

        register_mmio_handler(d, &vpci_mmio_handler_child, child_cfg->phys_addr,
                              child_cfg->size, bridge);
        count++;
    }

    return count;
}


static int vpci_permit_iomem(const struct dt_device_node *dev,
                                   uint32_t flags, uint64_t pci_addr, uint64_t addr,
                                   uint64_t len, void *data)
{
    struct domain *d = data;

    iomem_permit_access(d, PFN_DOWN(addr), PFN_UP(addr + len));
    return 0;
}

static int vpci_permit_bridge_iomem(struct domain *d,
                                   struct pci_host_bridge *bridge)
{
    dt_for_each_range(bridge->dt_node, vpci_permit_iomem, d);
    return 0;
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
    if ( !has_vpci_bridge(d) )
    {
        int ret;

        ret = pci_host_iterate_bridges_and_count(d, vpci_setup_mmio_handler_cb);
        if ( ret < 0 )
            return ret;

        ret = pci_host_iterate_bridges_and_count(d, vpci_permit_bridge_iomem);
        if ( ret < 0 )
            return ret;
    }
    else
    {
        if ( !IS_ENABLED(CONFIG_HAS_VPCI_GUEST_SUPPORT) )
        {
            gdprintk(XENLOG_ERR, "vPCI requested but guest support not enabled\n");
            return -EINVAL;
        }
        if ( domain_use_host_layout(d) )
        {
            struct pci_host_bridge *bridge;

            /* XXX: assume physical bridge is segment 0 bus 0 */
            bridge = pci_find_host_bridge(0, 0);
            if ( !bridge )
                return 0;

            register_mmio_handler(d, &vpci_mmio_handler,
                                  bridge->cfg->phys_addr, bridge->cfg->size, bridge);
        }
        else
        {
            register_mmio_handler(d, &vpci_mmio_handler,
                                  GUEST_VPCI_ECAM_BASE, GUEST_VPCI_ECAM_SIZE, NULL);
        }
    }

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
    int ret;
    if ( !has_vpci(d) )
        return 0;

    ret = pci_host_iterate_bridges_and_count(d, vpci_get_num_handlers_cb);

    if ( ret < 0 )
    {
        ASSERT_UNREACHABLE();
        return 0;
    }

    if ( ret )
        return ret;

    if ( is_control_domain(d) )
        return 0;
    else
        return 1;
}

void platform_pci_fixup_bar(const struct pci_dev *pdev,
                                          unsigned int bar_num,
                                          paddr_t *addr)
{
    struct pci_host_bridge *bridge = pci_find_host_bridge(pdev->sbdf.seg, pdev->sbdf.bus);

    if ( bridge->ops->fixup_bar )
    {
        bridge->ops->fixup_bar(bridge, bar_num, addr);
    }
}

int vpci_translate_bar_range(const struct pci_dev *pdev, struct vpci_bar *bar)
{
    struct pci_host_bridge *bridge = pci_find_host_bridge(pdev->sbdf.seg, pdev->sbdf.bus);
    struct pci_range_map *map;
    uint64_t start = bar->pci_addr;
    uint64_t end = bar->pci_addr + bar->size - 1;

    if ( !bridge )
        return -EINVAL;

    list_for_each_entry(map, &bridge->range_maps, node)
    {
        if ( start >= map->pci_addr && start <= map->pci_addr + map->len - 1 &&
             end >= map->pci_addr && end <= map->pci_addr + map->len - 1 )
        {
            bar->addr = map->mem_addr + (start - map->pci_addr);
            return 0;
        }
    }

    return -EINVAL;
}

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
