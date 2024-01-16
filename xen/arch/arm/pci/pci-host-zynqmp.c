/*
 * Based on Linux drivers/pci/controller/pci-host-common.c
 * Based on Linux drivers/pci/controller/pci-host-generic.c
 * Based on xen/arch/arm/pci/pci-host-generic.c
 * Based on Linux drivers/pci/controller/pcie-xilinx-nwl.c
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
#include <xen/vmap.h>
#include <asm/device.h>
#include <asm/io.h>
#include <asm/pci.h>

/* Bridge core config registers */
#define BRCFG_PCIE_RX0			0x00000000
#define BRCFG_PCIE_RX1			0x00000004
#define BRCFG_INTERRUPT			0x00000010
#define BRCFG_PCIE_RX_MSG_FILTER	0x00000020

/* Egress - Bridge translation registers */
#define E_BREG_CAPABILITIES		0x00000200
#define E_BREG_CONTROL			0x00000208
#define E_BREG_BASE_LO			0x00000210
#define E_BREG_BASE_HI			0x00000214
#define E_ECAM_CAPABILITIES		0x00000220
#define E_ECAM_CONTROL			0x00000228
#define E_ECAM_BASE_LO			0x00000230
#define E_ECAM_BASE_HI			0x00000234

/* Ingress - address translations */
#define I_MSII_CAPABILITIES		0x00000300
#define I_MSII_CONTROL			0x00000308
#define I_MSII_BASE_LO			0x00000310
#define I_MSII_BASE_HI			0x00000314

#define I_ISUB_CONTROL			0x000003E8
#define SET_ISUB_CONTROL		(1 << 0)
/* Rxed msg fifo  - Interrupt status registers */
#define MSGF_MISC_STATUS		0x00000400
#define MSGF_MISC_MASK			0x00000404
#define MSGF_LEG_STATUS			0x00000420
#define MSGF_LEG_MASK			0x00000424
#define MSGF_MSI_STATUS_LO		0x00000440
#define MSGF_MSI_STATUS_HI		0x00000444
#define MSGF_MSI_MASK_LO		0x00000448
#define MSGF_MSI_MASK_HI		0x0000044C

/* Msg filter mask bits */
#define CFG_ENABLE_PM_MSG_FWD		(1 << 1)
#define CFG_ENABLE_INT_MSG_FWD		(1 << 2)
#define CFG_ENABLE_ERR_MSG_FWD		(1 << 3)
#define CFG_ENABLE_MSG_FILTER_MASK	(CFG_ENABLE_PM_MSG_FWD | \
					CFG_ENABLE_INT_MSG_FWD | \
					CFG_ENABLE_ERR_MSG_FWD)

/* Misc interrupt status mask bits */
#define MSGF_MISC_SR_RXMSG_AVAIL	(1 << 0)
#define MSGF_MISC_SR_RXMSG_OVER		(1 << 1)
#define MSGF_MISC_SR_SLAVE_ERR		(1 << 4)
#define MSGF_MISC_SR_MASTER_ERR		(1 << 5)
#define MSGF_MISC_SR_I_ADDR_ERR		(1 << 6)
#define MSGF_MISC_SR_E_ADDR_ERR		(1 << 7)
#define MSGF_MISC_SR_FATAL_AER		(1 << 16)
#define MSGF_MISC_SR_NON_FATAL_AER	(1 << 17)
#define MSGF_MISC_SR_CORR_AER		(1 << 18)
#define MSGF_MISC_SR_UR_DETECT		(1 << 20)
#define MSGF_MISC_SR_NON_FATAL_DEV	(1 << 22)
#define MSGF_MISC_SR_FATAL_DEV		(1 << 23)
#define MSGF_MISC_SR_LINK_DOWN		(1 << 24)
#define MSGF_MSIC_SR_LINK_AUTO_BWIDTH	(1 << 25)
#define MSGF_MSIC_SR_LINK_BWIDTH	(1 << 26)

#define MSGF_MISC_SR_MASKALL		(MSGF_MISC_SR_RXMSG_AVAIL | \
					MSGF_MISC_SR_RXMSG_OVER | \
					MSGF_MISC_SR_SLAVE_ERR | \
					MSGF_MISC_SR_MASTER_ERR | \
					MSGF_MISC_SR_I_ADDR_ERR | \
					MSGF_MISC_SR_E_ADDR_ERR | \
					MSGF_MISC_SR_FATAL_AER | \
					MSGF_MISC_SR_NON_FATAL_AER | \
					MSGF_MISC_SR_CORR_AER | \
					MSGF_MISC_SR_UR_DETECT | \
					MSGF_MISC_SR_NON_FATAL_DEV | \
					MSGF_MISC_SR_FATAL_DEV | \
					MSGF_MISC_SR_LINK_DOWN | \
					MSGF_MSIC_SR_LINK_AUTO_BWIDTH | \
					MSGF_MSIC_SR_LINK_BWIDTH)

/* Legacy interrupt status mask bits */
#define MSGF_LEG_SR_INTA		(1 << 0)
#define MSGF_LEG_SR_INTB		(1 << 1)
#define MSGF_LEG_SR_INTC		(1 << 2)
#define MSGF_LEG_SR_INTD		(1 << 3)
#define MSGF_LEG_SR_MASKALL		(MSGF_LEG_SR_INTA | MSGF_LEG_SR_INTB | \
					MSGF_LEG_SR_INTC | MSGF_LEG_SR_INTD)

/* MSI interrupt status mask bits */
#define MSGF_MSI_SR_LO_MASK		GENMASK(31, 0)
#define MSGF_MSI_SR_HI_MASK		GENMASK(31, 0)

#define MSII_PRESENT			(1 << 0)
#define MSII_ENABLE			(1 << 0)
#define MSII_STATUS_ENABLE		(1 << 15)

/* Bridge config interrupt mask */
#define BRCFG_INTERRUPT_MASK		(1 << 0)
#define BREG_PRESENT			(1 << 0)
#define BREG_ENABLE			(1 << 0)
#define BREG_ENABLE_FORCE		(1 << 1)

/* E_ECAM status mask bits */
#define E_ECAM_PRESENT			(1 << 0)
#define E_ECAM_CR_ENABLE		(1 << 0)
#define E_ECAM_SIZE_LOC			0x1f0000 //GENMASK(20, 16)
#define E_ECAM_SIZE_SHIFT		16
#define NWL_ECAM_VALUE_DEFAULT		12

//#define CFG_DMA_REG_BAR			GENMASK(2, 0)
//#define CFG_PCIE_CACHE			GENMASK(7, 0)

#define INT_PCI_MSI_NR			(2 * 32)

/* Readin the PS_LINKUP */
#define PS_LINKUP_OFFSET		0x00000238
#define PCIE_PHY_LINKUP_BIT		(1 << 0)
#define PHY_RDY_LINKUP_BIT		(1 << 1)

/* Parameters for the waiting for link up routine */
#define LINK_WAIT_MAX_RETRIES          10
#define LINK_WAIT_USLEEP_MIN           90000
#define LINK_WAIT_USLEEP_MAX           100000

static int __init nwl_cfg_reg_index(struct dt_device_node *np)
{
    return dt_property_match_string(np, "reg-names", "cfg");
}

static inline uint32_t nwl_bridge_readl(const void __iomem *breg_base, uint32_t off)
{
    return readl(breg_base + off);
}

static inline void nwl_bridge_writel(void __iomem *breg_base, uint32_t val, uint32_t off)
{
    writel(val, breg_base + off);
}

static bool nwl_pcie_link_up(const void __iomem * pcireg_base)
{
    if (readl(pcireg_base + PS_LINKUP_OFFSET) & PCIE_PHY_LINKUP_BIT)
        return true;
    return false;
}

static bool nwl_phy_link_up(const void __iomem * pcireg_base)
{
    if (readl(pcireg_base + PS_LINKUP_OFFSET) & PHY_RDY_LINKUP_BIT)
        return true;
    return false;
}

static int nwl_wait_for_link(const void __iomem * pcireg_base)
{
    int retries;

    /* check if the link is up or not */
    for (retries = 0; retries < LINK_WAIT_MAX_RETRIES; retries++) {
        if (nwl_phy_link_up(pcireg_base))
            return 0;
#if 0
        usleep_range(LINK_WAIT_USLEEP_MIN, LINK_WAIT_USLEEP_MAX);
#endif
        printk(".");
    }

    printk("PHY link never came up\n");
    return -ETIMEDOUT;
}

static void __iomem *nwl_ecam_map_bus(struct pci_host_bridge *bridge,
                                      pci_sbdf_t sbdf, uint32_t where)
{
    uint32_t ecam_size, ecam_size_max;
    void __iomem *breg_base = ioremap_nocache(0xfd0e0000U, 0x1000); /* AXIPCIE_MAIN module */

    ecam_size = nwl_bridge_readl(breg_base, E_ECAM_CONTROL) >> E_ECAM_SIZE_SHIFT;
    ecam_size_max = nwl_bridge_readl(breg_base, E_ECAM_CAPABILITIES) >> 24;

    iounmap(breg_base);

    if ( ecam_size > ecam_size_max )
    {
        printk("%s:%d:%s ecam_size %u > ecam_size_max %u\n", __FILE__, __LINE__, __func__, ecam_size, ecam_size_max);
        return NULL;
    }

    return pci_ecam_map_bus(bridge, sbdf, where);
}

/* ECAM ops */
const struct pci_ecam_ops nwl_pcie_ops = {
    .bus_shift  = 20,
    .cfg_reg_index = nwl_cfg_reg_index,
    .pci_ops    = {
        .map_bus                = nwl_ecam_map_bus,
        .read                   = pci_generic_config_read,
        .write                  = pci_generic_config_write,
        .need_p2m_hwdom_mapping = pci_ecam_need_p2m_hwdom_mapping,
        .init_bus_range         = pci_generic_init_bus_range,
    }
};

static const struct dt_device_match __initconstrel nwl_pcie_dt_match[] =
{
    { .compatible = "xlnx,nwl-pcie-2.11" },
    { },
};

static int __init pci_host_generic_probe(struct dt_device_node *dev,
                                         const void *data)
{
    int rv;
    struct pci_host_bridge *bridge;
    void __iomem *breg_base;
    void __iomem *pcireg_base;
    uint32_t breg_val, ecam_val, first_busno = 0;
    uint32_t last_busno;

    breg_base = ioremap_nocache(0xfd0e0000U, 0x1000); /* AXIPCIE_MAIN module */
    pcireg_base = ioremap_nocache(0xfd480000U, 0x1000); /* PCIE_ATTRIB module */

    rv = PTR_RET(pci_host_common_probe(dev, &nwl_pcie_ops, NULL));

    bridge = pci_find_host_bridge(0, 0);

    breg_val = nwl_bridge_readl(breg_base, E_BREG_CAPABILITIES) & BREG_PRESENT;
    if (!breg_val) {
        printk("%s:%d:%s BREG is not present\n", __FILE__, __LINE__, __func__);
        return breg_val;
    }

    /* Write bridge_off to breg base */
    nwl_bridge_writel(breg_base, 0xfd0e0000U, E_BREG_BASE_LO);
    nwl_bridge_writel(breg_base, 0x00000000U, E_BREG_BASE_HI);

    /* Enable BREG */
    nwl_bridge_writel(breg_base, ~BREG_ENABLE_FORCE & BREG_ENABLE, E_BREG_CONTROL);

    /* Disable DMA channel registers */
    nwl_bridge_writel(breg_base, 0x00010007, BRCFG_PCIE_RX0);

    /* Enable Ingress subtractive decode translation */
    nwl_bridge_writel(breg_base, SET_ISUB_CONTROL, I_ISUB_CONTROL);

    /* Enable msg filtering details */
    nwl_bridge_writel(breg_base, CFG_ENABLE_MSG_FILTER_MASK, BRCFG_PCIE_RX_MSG_FILTER);

    rv = nwl_wait_for_link(pcireg_base);
    if ( rv )
    {
        printk("%s:%d:%s nwl_wait_for_link failed \n", __FILE__, __LINE__, __func__);
        return rv;
    }

    ecam_val = nwl_bridge_readl(breg_base, E_ECAM_CAPABILITIES) & E_ECAM_PRESENT;
    if (!ecam_val) {
        printk("%s:%d:%s ECAM is not present\n", __FILE__, __LINE__, __func__);
        return ecam_val;
    }

    /* Enable ECAM */
    nwl_bridge_writel(breg_base, nwl_bridge_readl(breg_base, E_ECAM_CONTROL) |
                                 E_ECAM_CR_ENABLE, E_ECAM_CONTROL);
    nwl_bridge_writel(breg_base, (nwl_bridge_readl(breg_base, E_ECAM_CONTROL) & ~E_ECAM_SIZE_LOC) |
                                 (0xc << E_ECAM_SIZE_SHIFT),
                                 E_ECAM_CONTROL);

    nwl_bridge_writel(breg_base, 0x00000000, E_ECAM_BASE_LO);
    nwl_bridge_writel(breg_base, 0x00000080, E_ECAM_BASE_HI);

    /* Get bus range */
    ecam_val = nwl_bridge_readl(breg_base, E_ECAM_CONTROL);
    last_busno = (ecam_val & E_ECAM_SIZE_LOC) >> E_ECAM_SIZE_SHIFT;
    /* Write primary, secondary and subordinate bus numbers */
    ecam_val = first_busno;
    ecam_val |= (first_busno + 1) << 8;
    ecam_val |= (last_busno << E_ECAM_SIZE_SHIFT);

    bridge->cfg->busn_start = first_busno;
    bridge->cfg->busn_end = last_busno;
    rv = pci_init_bridge(PCI_SBDF(0,0,0,0));
    if ( rv )
    {
        printk("%s:%d:%s bridge initialization failed\n", __FILE__, __LINE__, __func__);
    }

    printk("%s:%d:%s 00:00.0 vid: 0x%04x\n", __FILE__, __LINE__, __func__, pci_conf_read16(PCI_SBDF(0,0,0,0), PCI_VENDOR_ID));
    printk("%s:%d:%s 00:00.0 did: 0x%04x\n", __FILE__, __LINE__, __func__, pci_conf_read16(PCI_SBDF(0,0,0,0), PCI_DEVICE_ID));
    printk("%s:%d:%s 00:00.0 cmd: 0x%04x\n", __FILE__, __LINE__, __func__, pci_conf_read16(PCI_SBDF(0,0,0,0), PCI_COMMAND));
    printk("%s:%d:%s 00:00.0 status: 0x%04x\n", __FILE__, __LINE__, __func__, pci_conf_read16(PCI_SBDF(0,0,0,0), PCI_STATUS));

    if (nwl_pcie_link_up(pcireg_base))
    {
        printk("%s:%d:%s Link is UP\n", __FILE__, __LINE__, __func__);
        printk("%s:%d:%s 01:00.0 vid: 0x%04x\n", __FILE__, __LINE__, __func__, pci_conf_read16(PCI_SBDF(0,1,0,0), PCI_VENDOR_ID));
        printk("%s:%d:%s 01:00.0 did: 0x%04x\n", __FILE__, __LINE__, __func__, pci_conf_read16(PCI_SBDF(0,1,0,0), PCI_DEVICE_ID));
        printk("%s:%d:%s 01:00.0 cmd: 0x%04x\n", __FILE__, __LINE__, __func__, pci_conf_read16(PCI_SBDF(0,1,0,0), PCI_COMMAND));
        printk("%s:%d:%s 01:00.0 status: 0x%04x\n", __FILE__, __LINE__, __func__, pci_conf_read16(PCI_SBDF(0,1,0,0), PCI_STATUS));
    }
    else
        printk("%s:%d:%s Link is DOWN\n", __FILE__, __LINE__, __func__);

    /* Disable all INTX interrupts */
    nwl_bridge_writel(breg_base, (uint32_t)~MSGF_LEG_SR_MASKALL, MSGF_LEG_MASK);

    /* Clear pending INTX interrupts */
    nwl_bridge_writel(breg_base, nwl_bridge_readl(breg_base, MSGF_LEG_STATUS) &
              MSGF_LEG_SR_MASKALL, MSGF_LEG_STATUS);

    /* Enable all INTX interrupts */
    nwl_bridge_writel(breg_base, MSGF_LEG_SR_MASKALL, MSGF_LEG_MASK);

    iounmap(breg_base);
    iounmap(pcireg_base);
    return rv;
}

DT_DEVICE_START(pci_gen, "PCI HOST ZYNQMP", DEVICE_PCI_HOSTBRIDGE)
.dt_match = nwl_pcie_dt_match,
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
