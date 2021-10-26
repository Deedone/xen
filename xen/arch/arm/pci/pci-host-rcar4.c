/*
 * Based on Linux drivers/pci/controller/pci-host-common.c
 * Based on Linux drivers/pci/controller/pci-host-generic.c
 * Based on xen/arch/arm/pci/pci-host-generic.c
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

#include <xen/init.h>
#include <xen/pci.h>

#include <asm/device.h>
#include <asm/io.h>
#include <asm/pci.h>

#include "pci-designware.h"

#define RCAR4_DWC_VERSION       0x520A

/*
 * PCI host bridges often have different ways to access the root and child
 * bus config spaces:
 *   "dbi"   : the aperture where root port's own configuration registers
 *             are available.
 *   "config": child's configuration space
 *   "atu"   : iATU registers for DWC version 4.80 or later
 */
static int __init rcar4_cfg_reg_index(struct dt_device_node *np)
{
    return dt_property_match_string(np, "reg-names", "dbi");
}

static int __init rcar4_child_cfg_reg_index(struct dt_device_node *np)
{
    return dt_property_match_string(np, "reg-names", "config");
}

/* ECAM ops */
const struct pci_ecam_ops rcar4_pcie_ops = {
    .bus_shift  = 20,
    .cfg_reg_index = rcar4_cfg_reg_index,
    .pci_ops    = {
        .map_bus                = pci_ecam_map_bus,
        .read                   = pci_generic_config_read,
        .write                  = pci_generic_config_write,
        .need_p2m_hwdom_mapping = pci_ecam_need_p2m_hwdom_mapping,
    }
};

const struct pci_ecam_ops rcar4_pcie_child_ops = {
    .bus_shift  = 20,
    .cfg_reg_index = rcar4_child_cfg_reg_index,
    .pci_ops    = {
        .map_bus                = dw_pcie_child_map_bus,
        .read                   = dw_pcie_child_config_read,
        .write                  = dw_pcie_child_config_write,
        .need_p2m_hwdom_mapping = dw_pcie_child_need_p2m_hwdom_mapping,
    }
};

static const struct dt_device_match __initconstrel rcar4_pcie_dt_match[] =
{
    { .compatible = "renesas,r8a779f0-pcie" },
    { .compatible = "renesas,r8a779g0-pcie" },
    { },
};

static int __init pci_host_generic_probe(struct dt_device_node *dev,
                                         const void *data)
{
    struct pci_host_bridge *bridge;

    bridge = dw_pcie_host_probe(dev, data, &rcar4_pcie_ops,
                                &rcar4_pcie_child_ops);

    dw_pcie_set_version(bridge, RCAR4_DWC_VERSION);

    return 0;
}

DT_DEVICE_START(pci_gen, "PCI HOST R-CAR GEN4", DEVICE_PCI_HOSTBRIDGE)
.dt_match = rcar4_pcie_dt_match,
.init = pci_host_generic_probe,
DT_DEVICE_END

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
