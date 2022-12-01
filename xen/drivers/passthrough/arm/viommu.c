/* SPDX-License-Identifier: (GPL-2.0-or-later OR BSD-2-Clause) */

#include <xen/errno.h>
#include <xen/init.h>
#include <xen/irq.h>
#include <xen/types.h>
#include <xen/sched.h>

#include <asm/viommu.h>

/* List of all host IOMMUs */
LIST_HEAD(host_iommu_list);

/* Struct to hold the vIOMMU ops and vIOMMU type */
static const struct viommu_desc __ro_after_init *cur_viommu;

/* Common function for adding to host_iommu_list */
void add_to_host_iommu_list(paddr_t addr, paddr_t size,
                            const struct dt_device_node *node)
{
    struct host_iommu *iommu_data;

    iommu_data = xzalloc(struct host_iommu);
    if ( !iommu_data )
        panic("vIOMMU: Cannot allocate memory for host IOMMU data\n");

    iommu_data->addr = addr;
    iommu_data->size = size;
    iommu_data->dt_node = node;
    iommu_data->irq = platform_get_irq(node, 0);
    if ( iommu_data->irq < 0 )
    {
        gdprintk(XENLOG_ERR,
                 "vIOMMU: Cannot find a valid IOMMU irq\n");
        xfree(iommu_data);
        return;
    }

    printk("vIOMMU: Found IOMMU @0x%"PRIx64"\n", addr);

    list_add_tail(&iommu_data->entry, &host_iommu_list);
}

int domain_viommu_init(struct domain *d, uint8_t viommu_type)
{
    if ( viommu_type == XEN_DOMCTL_CONFIG_VIOMMU_NONE )
        return 0;

    if ( !cur_viommu )
        return -ENODEV;

    if ( cur_viommu->viommu_type != viommu_type )
        return -EINVAL;

    return cur_viommu->ops->domain_init(d);
}

int viommu_relinquish_resources(struct domain *d)
{
    if ( !cur_viommu )
        return 0;

    return cur_viommu->ops->relinquish_resources(d);
}

uint8_t viommu_get_type(void)
{
    if ( !cur_viommu )
        return XEN_DOMCTL_CONFIG_VIOMMU_NONE;

    return cur_viommu->viommu_type;
}

void set_cur_viommu(const struct viommu_desc *desc)
{
    if ( cur_viommu && (cur_viommu != desc) )
    {
        printk("WARNING: Cannot set vIOMMU, already set to a different value\n");
        return;
    }

    cur_viommu = desc;
}

unsigned int domain_viommu_get_num_mmio_handlers(struct domain *d)
{
    if ( is_hardware_domain(d) )
    {
        struct host_iommu *hw_iommu;
        int count = 0;

        /*
        * For hardware domain, vIOMMU per pIOMMU is exposed. Number of
        * regions required for the configuration space is equal to number of
        * host IOMMUs.
        */
        list_for_each_entry(hw_iommu, &host_iommu_list, entry)
        {
            count++;
        }

        return count;
    }

    /*
     * For unpriviliged domains, single vIOMMU is exposed and requires one
     * region to cover the configuration space.
     */
    return 1;
}

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
