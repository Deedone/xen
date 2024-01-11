/*
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

#include <xen/acpi.h>
#include <xen/device_tree.h>
#include <xen/errno.h>
#include <xen/init.h>
#include <xen/iommu.h>
#include <xen/param.h>
#include <xen/pci.h>

/*
 * PIRQ event channels are not supported on Arm, so nothing to do.
 */
int arch_pci_clean_pirqs(struct domain *d)
{
    return 0;
}

struct pci_dev *dev_to_pci(struct device *dev)
{
    ASSERT(dev->type == DEV_PCI);

    return container_of(dev, struct pci_dev, arch.dev);
}

void arch_pci_init_pdev(struct pci_dev *pdev)
{
    pci_to_dev(pdev)->type = DEV_PCI;
}

static int __init dt_pci_init(void)
{
    struct dt_device_node *np;
    int rc;

    dt_for_each_device_node(dt_host, np)
    {
        rc = device_init(np, DEVICE_PCI_HOSTBRIDGE, NULL);
        /*
         * Ignore the following error codes:
         *   - EBADF: Indicate the current device is not a pci device.
         *   - ENODEV: The pci device is not present or cannot be used by
         *     Xen.
         */
        if( !rc || rc == -EBADF || rc == -ENODEV )
            continue;

        return rc;
    }

    return 0;
}

#ifdef CONFIG_ACPI
static int __init acpi_pci_init(void)
{
    printk(XENLOG_ERR "ACPI pci init not supported \n");
    return -EOPNOTSUPP;
}
#else
static int __init acpi_pci_init(void)
{
    return -EINVAL;
}
#endif

/*
 * Platform-specific PCI host dependencies require dom0 to handle
 * initialization and issue PHYSDEVOP_pci_device_add/remove calls for SMMU
 * device registration. This check is used to enable the minimal PCI
 * subsystem required for dom0 operation when PCI passthrough is disabled.
 */
bool arch_pci_device_physdevop(void)
{
    return iommu_enabled;
}

/* By default pci passthrough is disabled. */
bool __read_mostly pci_passthrough_enabled;
boolean_param("pci-passthrough", pci_passthrough_enabled);

/* By default pci scan is disabled. */
__ro_after_init bool pci_scan_enabled;
boolean_param("pci-scan", pci_scan_enabled);

typedef int (*bar_callback_t)(struct pci_dev *, uint8_t, uint64_t, uint64_t,
                              bool, bool);

static int __init reserve_bar_range(struct pci_dev *pdev, uint8_t reg,
                                    uint64_t addr, uint64_t size, bool is_64bit,
                                    bool prefetch)
{
    if ( pci_check_bar(pdev, maddr_to_mfn(addr),
                       maddr_to_mfn(addr + size - 1)) )
        return pci_reserve_bar_range(pdev, addr, size, prefetch);
    return 0;
}

static int __init setup_bar(struct pci_dev *pdev, uint8_t reg, uint64_t addr,
                            uint64_t size, bool is_64bit, bool prefetch)
{
    if ( !pci_check_bar(pdev, maddr_to_mfn(addr),
                        maddr_to_mfn(addr + size - 1)) )
    {
        uint16_t cmd = pci_conf_read16(pdev->sbdf, PCI_COMMAND);

        addr = pci_get_new_bar_addr(pdev, size, is_64bit, prefetch);
        if ( !addr )
            return -ENOMEM;

        pci_conf_write16(pdev->sbdf, PCI_COMMAND,
                         cmd & ~(PCI_COMMAND_MEMORY | PCI_COMMAND_IO));

        pci_conf_write32(pdev->sbdf, reg,
                         (addr & GENMASK(31, 0)) |
                         (is_64bit ? PCI_BASE_ADDRESS_MEM_TYPE_64 : 0));

        if ( is_64bit )
            pci_conf_write32(pdev->sbdf, reg + 4, addr >> 32);

        pci_conf_write16(pdev->sbdf, PCI_COMMAND, cmd);
    }

    return 0;
}

static int __init bars_iterate(struct pci_dev *pdev, void *arg)
{
    unsigned int i, barsize, ret = 0, num_bars = PCI_HEADER_NORMAL_NR_BARS;
    uint64_t addr, size;
    bar_callback_t cb = arg;

    if ( (pci_conf_read8(pdev->sbdf, PCI_HEADER_TYPE) & 0x7f) ==
         PCI_HEADER_TYPE_NORMAL )
    {
        for ( i = 0; i < num_bars; i += barsize )
        {
            uint8_t reg = PCI_BASE_ADDRESS_0 + i * 4;
            bool prefetch;

            if ( (pci_conf_read32(pdev->sbdf, reg) & PCI_BASE_ADDRESS_SPACE) ==
                 PCI_BASE_ADDRESS_SPACE_IO )
            {
                barsize = 1;
                continue;
            }

            barsize = pci_size_mem_bar(pdev->sbdf, reg, &addr, &size,
                                       (i == num_bars - 1) ? PCI_BAR_LAST : 0);

            if ( !size )
                continue;

            prefetch = pci_conf_read32(pdev->sbdf, reg) &
                       PCI_BASE_ADDRESS_MEM_PREFETCH;

            ret = cb(pdev, reg, addr, size, barsize == 2, prefetch);
            if ( ret )
                return ret;
        }
    }

    return ret;
}

static int __init pci_setup_bars(void)
{
    int ret;
    /* We can't change the signature of bars_iterate to only accept
     * bar_callback_t, so use intermediate variables to ensure callback
     * signatures are always correct
     */
    bar_callback_t cb_reserve = reserve_bar_range;
    bar_callback_t cb_setup = setup_bar;

    pcidevs_lock();
    ret = pci_iterate_devices(bars_iterate, cb_reserve);
    if ( ret )
        goto out;

    ret = pci_iterate_devices(bars_iterate, cb_setup);

out:
    pcidevs_unlock();
    return ret;
}

static int __init pci_init(void)
{
    int ret;

    /*
     * Enable PCI passthrough when has been enabled explicitly
     * (pci-passthrough=on).
     */
    if ( !is_pci_passthrough_enabled() && !arch_pci_device_physdevop() )
        return 0;

    if ( pci_add_segment(0) )
        panic("Could not initialize PCI segment 0\n");

    if ( acpi_disabled )
        ret = dt_pci_init();
    else
        ret = acpi_pci_init();

    if ( ret < 0 )
    {
        printk(XENLOG_ERR "PCI: Failed to initialize PCI host bridges (rc=%d)\n", ret);
        return 0;
    }

    if ( pci_scan_enabled )
    {
        ret = scan_pci_devices();

        if ( ret < 0 )
        {
            printk(XENLOG_ERR "PCI: Failed to scan PCI devices (rc=%d)\n", ret);
            return 0;
        }

        ret = pci_setup_bars();

        if ( ret < 0 )
        {
            printk(XENLOG_ERR "PCI: Failed to configure BARs (rc=%d)\n", ret);
            return 0;
        }
    }

    return 0;
}
__initcall(pci_init);

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
