/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Handlers for accesses to the SR-IOV capability structure.
 *
 * Copyright (C) 2018 Citrix Systems R&D
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms and conditions of the GNU General Public
 * License, version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this program; If not, see <http://www.gnu.org/licenses/>.
 */

#include <xen/sched.h>
#include <xen/vpci.h>

static int vf_init_bars(const struct pci_dev *vf_pdev)
{
    unsigned int i, sriov_pos;
    int vf_idx;
    const struct pci_dev *pf_pdev = vf_pdev->pf_pdev;
    uint16_t offset, stride;
    struct vpci_bar *bars = vf_pdev->vpci->header.bars;
    struct vpci_bar *physfn_vf_bars = pf_pdev->vpci->sriov->vf_bars;

    sriov_pos = pci_find_ext_capability(pf_pdev->sbdf, PCI_EXT_CAP_ID_SRIOV);
    offset = pci_conf_read16(pf_pdev->sbdf, sriov_pos + PCI_SRIOV_VF_OFFSET);
    stride = pci_conf_read16(pf_pdev->sbdf, sriov_pos + PCI_SRIOV_VF_STRIDE);

    vf_idx = vf_pdev->sbdf.sbdf;
    vf_idx -= pf_pdev->sbdf.sbdf + offset;
    if ( vf_idx < 0 )
        return -EINVAL;
    if ( stride )
    {
        if ( vf_idx % stride )
            return -EINVAL;
        vf_idx /= stride;
    }

    /*
     * Set up BARs for this VF out of PF's VF BARs taking into account
     * the index of the VF.
     */
    for ( i = 0; i < PCI_SRIOV_NUM_BARS; i++)
    {
        bars[i].addr = physfn_vf_bars[i].addr + vf_idx * physfn_vf_bars[i].size;
        if ( pf_pdev->domain == vf_pdev->domain || bars[i].guest_addr == 0 )
            bars[i].guest_addr = bars[i].addr;
        bars[i].size = physfn_vf_bars[i].size;
        bars[i].type = physfn_vf_bars[i].type;
        bars[i].prefetchable = physfn_vf_bars[i].prefetchable;
    }

    return 0;
}

static int map_vf(const struct pci_dev *vf_pdev, uint16_t cmd,
                  enum vpci_map_op map_op)
{
    int rc;

    ASSERT(rw_is_write_locked(&vf_pdev->domain->pci_lock));

    rc = vf_init_bars(vf_pdev);
    if ( rc )
        return rc;

    return vpci_modify_bars(vf_pdev, cmd, map_op, false);
}

static int size_vf_bars(struct pci_dev *pf_pdev, unsigned int sriov_pos)
{
    /*
     * NB: a non-const pci_dev of the PF is needed in order to update
     * vf_rlen.
     */
    struct vpci_bar *bars;
    unsigned int i;
    int rc = 0;

    ASSERT(rw_is_write_locked(&pf_pdev->domain->pci_lock));
    ASSERT(!pf_pdev->info.is_virtfn);

    if ( !pf_pdev->vpci->sriov )
        return -EINVAL;

    /* Read BARs for VFs out of PF's SR-IOV extended capability. */
    bars = pf_pdev->vpci->sriov->vf_bars;
    /* Set the BARs addresses and size. */
    for ( i = 0; i < PCI_SRIOV_NUM_BARS; i += rc )
    {
        unsigned int idx = sriov_pos + PCI_SRIOV_BAR + i * 4;
        uint32_t bar;
        uint64_t addr, size;

        bar = pci_conf_read32(pf_pdev->sbdf, idx);

        rc = pci_size_mem_bar(pf_pdev->sbdf, idx, &addr, &size,
                              PCI_BAR_VF |
                              ((i == PCI_SRIOV_NUM_BARS - 1) ?
                               PCI_BAR_LAST : 0));

        /*
         * Update vf_rlen on the PF. According to the spec the size of
         * the BARs can change if the system page size register is
         * modified, so always update rlen when enabling VFs.
         */
        pf_pdev->physfn.vf_rlen[i] = size;

        if ( !size )
        {
            bars[i].type = VPCI_BAR_EMPTY;
            continue;
        }

        bars[i].addr = addr;
        bars[i].guest_addr = addr;
        bars[i].size = size;
        bars[i].prefetchable = bar & PCI_BASE_ADDRESS_MEM_PREFETCH;

        switch ( rc )
        {
        case 1:
            bars[i].type = VPCI_BAR_MEM32;
            break;

        case 2:
            bars[i].type = VPCI_BAR_MEM64_LO;
            bars[i + 1].type = VPCI_BAR_MEM64_HI;
            break;

        default:
            ASSERT_UNREACHABLE();
        }
    }

    rc = rc > 0 ? 0 : rc;

    return rc;
}

static void cf_check control_write(const struct pci_dev *pdev, unsigned int reg,
                                   uint32_t val, void *data)
{
    unsigned int sriov_pos = reg - PCI_SRIOV_CTRL;
    uint16_t control = pci_conf_read16(pdev->sbdf, reg);
    bool mem_enabled = control & PCI_SRIOV_CTRL_MSE;
    bool new_mem_enabled = val & PCI_SRIOV_CTRL_MSE;

    ASSERT(!pdev->info.is_virtfn);

    if ( new_mem_enabled != mem_enabled )
    {
        if ( new_mem_enabled )
        {
            struct pci_dev *vf_pdev;

            /* FIXME casting away const-ness to modify vf_rlen */
            size_vf_bars((struct pci_dev *)pdev, sriov_pos);

            list_for_each_entry(vf_pdev, &pdev->vf_list, vf_list)
                map_vf(vf_pdev, PCI_COMMAND_MEMORY, VPCI_MAP);
        }
        /* TODO: unmap vf */
    }

    pci_conf_write16(pdev->sbdf, reg, val);
}

#ifdef CONFIG_HAS_VPCI_GUEST_SUPPORT
static void cf_check vf_cmd_write(
    const struct pci_dev *pdev, unsigned int reg, uint32_t cmd, void *data)
{
    struct vpci_header *header = data;

    cmd |= PCI_COMMAND_INTX_DISABLE;

    header->guest_cmd = cmd;

    /*
     * Let Dom0 play with all the bits directly except for the memory
     * decoding one. Bits that are not allowed for DomU are already
     * handled above and by the rsvdp_mask.
     */
    if ( header->bars_mapped != !!(cmd & PCI_COMMAND_MEMORY) )
    {
        /*
         * Ignore the error. No memory has been added or removed from the p2m
         * (because the actual p2m changes are deferred in defer_map) and the
         * memory decoding bit has not been changed, so leave everything as-is,
         * hoping the guest will realize and try again.
         */
        map_vf(pdev, cmd, cmd & PCI_COMMAND_MEMORY ? VPCI_MAP : VPCI_UNMAP);
    }
    else
        pci_conf_write16(pdev->sbdf, reg, cmd);
}

static uint32_t cf_check vf_cmd_read(
    const struct pci_dev *pdev, unsigned int reg, void *data)
{
    const struct vpci_header *header = data;

    return header->guest_cmd;
}
#endif /* CONFIG_HAS_VPCI_GUEST_SUPPORT */

static int vf_init_header(struct pci_dev *vf_pdev)
{
    const struct pci_dev *pf_pdev;
    unsigned int sriov_pos;
    int rc = 0;
    uint16_t ctrl;

    ASSERT(rw_is_write_locked(&vf_pdev->domain->pci_lock));

    if ( !vf_pdev->info.is_virtfn )
        return 0;

    pf_pdev = vf_pdev->pf_pdev;
    ASSERT(pf_pdev);

    sriov_pos = pci_find_ext_capability(pf_pdev->sbdf, PCI_EXT_CAP_ID_SRIOV);
    ctrl = pci_conf_read16(pf_pdev->sbdf, sriov_pos + PCI_SRIOV_CTRL);

#ifdef CONFIG_HAS_VPCI_GUEST_SUPPORT
    if ( pf_pdev->domain != vf_pdev->domain /* TODO: !hw_dom? */)
    {
        uint16_t vid = pci_conf_read16(pf_pdev->sbdf, PCI_VENDOR_ID);
        uint16_t did = pci_conf_read16(pf_pdev->sbdf, sriov_pos + PCI_SRIOV_VF_DID);
        struct vpci_bar *bars = vf_pdev->vpci->header.bars;
        unsigned int i;

        rc = vpci_add_register(vf_pdev->vpci, vpci_read_val, NULL,
                               PCI_VENDOR_ID, 2,
                               (void *)(uintptr_t)vid);
        if ( rc )
            return rc;

        rc = vpci_add_register(vf_pdev->vpci, vpci_read_val, NULL,
                               PCI_DEVICE_ID, 2,
                               (void *)(uintptr_t)did);
        if ( rc )
            return rc;

        /* Hardcode multi-function device bit to 0 */
        rc = vpci_add_register(vf_pdev->vpci, vpci_read_val, NULL,
                               PCI_HEADER_TYPE, 1,
                               (void *)PCI_HEADER_TYPE_NORMAL);
        if ( rc )
            return rc;

        /* TODO: look at the VF ROZ bits in the pci spec */
        rc = vpci_add_register_mask(vf_pdev->vpci,
                                    vf_cmd_read,
                                    vf_cmd_write, PCI_COMMAND, 2,
                                    &vf_pdev->vpci->header, 0, 0,
                                               PCI_COMMAND_RSVDP_MASK |
                                               PCI_COMMAND_IO |
                                               PCI_COMMAND_PARITY |
                                               PCI_COMMAND_WAIT |
                                               PCI_COMMAND_SERR |
                                               PCI_COMMAND_FAST_BACK,
                                    0);
        if ( rc )
            return rc;

        rc = vpci_init_capability_list(vf_pdev);
        if ( rc )
            return rc;

        for ( i = 0; i < PCI_SRIOV_NUM_BARS; i++ )
        {
            switch ( pf_pdev->vpci->sriov->vf_bars[i].type )
            {
            case VPCI_BAR_MEM32:
            case VPCI_BAR_MEM64_LO:
            case VPCI_BAR_MEM64_HI:
                rc = vpci_add_register(vf_pdev->vpci,
                                       vpci_guest_mem_bar_read,
                                       vpci_guest_mem_bar_write,
                                       PCI_BASE_ADDRESS_0 + i * 4, 4, &bars[i]);
                if ( rc )
                    return rc;
                break;
            default:
                rc = vpci_add_register(vf_pdev->vpci, vpci_read_val, NULL,
                                       PCI_BASE_ADDRESS_0 + i * 4, 4,
                                       (void *)0);
                if ( rc )
                    return rc;
                break;
            }
        }

        rc = vf_init_bars(vf_pdev);
        if ( rc )
            return rc;
    }
#endif /* CONFIG_HAS_VPCI_GUEST_SUPPORT */

    if ( (pf_pdev->domain == vf_pdev->domain) && (ctrl & PCI_SRIOV_CTRL_MSE) )
    {
        rc = map_vf(vf_pdev, PCI_COMMAND_MEMORY, VPCI_MAP);
        if ( rc )
            return rc;
    }

    return rc;
}

static int init_sriov(struct pci_dev *pdev)
{
    unsigned int pos;

    if ( pdev->info.is_virtfn )
        return vf_init_header(pdev);

    pos = pci_find_ext_capability(pdev->sbdf, PCI_EXT_CAP_ID_SRIOV);

    if ( !pos )
        return 0;

    if ( !is_hardware_domain(pdev->domain) )
    {
        printk(XENLOG_ERR "%pp: SR-IOV configuration unsupported for unpriv %pd\n",
               &pdev->sbdf, pdev->domain);
        return 0;
    }

    pdev->vpci->sriov = xzalloc(struct vpci_sriov);
    if ( !pdev->vpci->sriov )
        return -ENOMEM;

    return vpci_add_register(pdev->vpci, vpci_hw_read16, control_write,
                             pos + PCI_SRIOV_CTRL, 2, NULL);
}
REGISTER_VPCI_EXTCAP(SRIOV, init_sriov, NULL);

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
