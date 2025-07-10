/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Handlers for accesses to the SR-IOV capability structure.
 *
 * Copyright (C) 2026 Citrix Systems R&D
 */

#include <xen/sched.h>
#include <xen/vpci.h>
#include <xsm/xsm.h>
#include "private.h"

static int vf_init_bars(const struct pci_dev *vf_pdev)
{
    int vf_idx;
    unsigned int i;
    const struct pci_dev *pf_pdev = vf_pdev->pf_pdev;
    struct vpci_bar *bars = vf_pdev->vpci->header.bars;
    struct vpci_bar *physfn_vf_bars = pf_pdev->vpci->sriov->vf_bars;
    unsigned int sriov_pos = pci_find_ext_capability(pf_pdev,
                                                     PCI_EXT_CAP_ID_SRIOV);
    uint16_t offset = pci_conf_read16(pf_pdev->sbdf,
                                      sriov_pos + PCI_SRIOV_VF_OFFSET);
    uint16_t stride = pci_conf_read16(pf_pdev->sbdf,
                                      sriov_pos + PCI_SRIOV_VF_STRIDE);

    vf_idx = vf_pdev->sbdf.sbdf - (pf_pdev->sbdf.sbdf + offset);
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
    for ( i = 0; i < PCI_SRIOV_NUM_BARS; i++ )
    {
        bars[i].addr = physfn_vf_bars[i].addr + vf_idx * physfn_vf_bars[i].size;
        bars[i].guest_addr = bars[i].addr;
        bars[i].size = physfn_vf_bars[i].size;
        bars[i].type = physfn_vf_bars[i].type;
        bars[i].prefetchable = physfn_vf_bars[i].prefetchable;
    }

    return 0;
}

static int map_vfs(const struct pci_dev *pf_pdev, uint16_t cmd)
{
    struct pci_dev *vf_pdev;
    int rc;

    ASSERT(rw_is_write_locked(&pf_pdev->domain->pci_lock));

    list_for_each_entry(vf_pdev, &pf_pdev->vf_list, vf_list)
    {
        rc = vpci_modify_bars(vf_pdev, cmd, false);
        if ( rc )
        {
            gprintk(XENLOG_ERR, "failed to %s VF %pp: %d\n",
                    (cmd & PCI_COMMAND_MEMORY) ? "map" : "unmap",
                    &vf_pdev->sbdf, rc);
            return rc;
        }

        vf_pdev->vpci->header.guest_cmd &= ~PCI_COMMAND_MEMORY;
        vf_pdev->vpci->header.guest_cmd |= (cmd & PCI_COMMAND_MEMORY);
    }

    return 0;
}

static int size_vf_bars(const struct pci_dev *pf_pdev, unsigned int sriov_pos,
                        uint64_t *vf_rlen)
{
    struct vpci_bar *bars;
    unsigned int i;
    int rc = 0;

    ASSERT(rw_is_write_locked(&pf_pdev->domain->pci_lock));
    ASSERT(!pf_pdev->info.is_virtfn);
    ASSERT(pf_pdev->vpci->sriov);

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
                              ((i == PCI_SRIOV_NUM_BARS - 1) ? PCI_BAR_LAST
                                                             : 0));

        /*
         * Update vf_rlen on the PF. According to the spec the size of
         * the BARs can change if the system page size register is
         * modified, so always update rlen when enabling VFs.
         */
        vf_rlen[i] = size;

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
    struct vpci_sriov *sriov = pdev->vpci->sriov;
    uint16_t control = pci_conf_read16(pdev->sbdf, reg);
    bool mem_enabled = control & PCI_SRIOV_CTRL_MSE;
    bool new_mem_enabled = val & PCI_SRIOV_CTRL_MSE;
    bool enabled = control & PCI_SRIOV_CTRL_VFE;
    bool new_enabled = val & PCI_SRIOV_CTRL_VFE;
    int rc;

    ASSERT(!pdev->info.is_virtfn);

    if ( new_enabled == enabled && new_mem_enabled == mem_enabled )
    {
        pci_conf_write16(pdev->sbdf, reg, val);
        return;
    }

    if ( mem_enabled && !new_mem_enabled )
        map_vfs(pdev, 0);


    if ( !enabled && new_enabled )
    {
        size_vf_bars(pdev, sriov_pos, (uint64_t *)data);

        /*
         * Only update the number of active VFs when enabling, when
         * disabling use the cached value in order to always remove the same
         * number of VFs that were active.
         */
        sriov->num_vfs = pci_conf_read16(pdev->sbdf,
                                         sriov_pos + PCI_SRIOV_NUM_VF);
    }

    if ( !mem_enabled && new_mem_enabled )
    {
        rc = map_vfs(pdev, PCI_COMMAND_MEMORY);

        if ( rc )
            map_vfs(pdev, 0);
    }

    pci_conf_write16(pdev->sbdf, reg, val);
}

int vpci_vf_init_header(struct pci_dev *vf_pdev)
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

    rc = vf_init_bars(vf_pdev);
    if ( rc )
        return rc;

    sriov_pos = pci_find_ext_capability(pf_pdev, PCI_EXT_CAP_ID_SRIOV);
    ctrl = pci_conf_read16(pf_pdev->sbdf, sriov_pos + PCI_SRIOV_CTRL);

    if ( IS_ENABLED(CONFIG_HAS_VPCI_GUEST_SUPPORT) && 
         pf_pdev->domain != vf_pdev->domain )
    {
        struct vpci_bar *bars = vf_pdev->vpci->header.bars;
        unsigned int i;

        rc = vpci_add_register(vf_pdev->vpci, vpci_hw_read16, NULL,
                               PCI_VENDOR_ID, 2, NULL)
        if ( rc )
            return rc;

        rc = vpci_add_register(vf_pdev->vpci, vpci_hw_read16, NULL,
                               PCI_DEVICE_ID, 2, NULL);
        if ( rc )
            return rc;

        /* Hardcode multi-function device bit to 0 */
        rc = vpci_add_register(vf_pdev->vpci, vpci_read_val, NULL,
                               PCI_HEADER_TYPE, 1,
                               (void *)PCI_HEADER_TYPE_NORMAL);
        if ( rc )
            return rc;

        rc = vpci_add_register(vf_pdev->vpci, vpci_hw_read32, NULL,
                               PCI_CLASS_REVISION, 4, NULL);
        if ( rc )
            return rc;

        for ( i = 0; i < PCI_SRIOV_NUM_BARS; i++ )
        {
            switch ( pf_pdev->vpci->sriov->vf_bars[i].type )
            {
            case VPCI_BAR_MEM32:
            case VPCI_BAR_MEM64_LO:
            case VPCI_BAR_MEM64_HI:
                rc = vpci_add_register(vf_pdev->vpci, vpci_guest_mem_bar_read,
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
    }

    if ( (pf_pdev->domain == vf_pdev->domain) && (ctrl & PCI_SRIOV_CTRL_MSE) )
    {
        rc = vpci_modify_bars(vf_pdev, PCI_COMMAND_MEMORY, false);
        if ( rc )
            return rc;

        vf_pdev->vpci->header.guest_cmd |= PCI_COMMAND_MEMORY;
    }

    return rc;
}

static int cf_check init_sriov(struct pci_dev *pdev)
{
    unsigned int pos;

    ASSERT(!pdev->info.is_virtfn);

    pos = pci_find_ext_capability(pdev, PCI_EXT_CAP_ID_SRIOV);

    if ( !pos )
        return 0;

    if ( xsm_resource_setup_pci(XSM_PRIV, pdev->sbdf.bdf) )
    {
        printk(XENLOG_ERR
               "%pp: SR-IOV configuration unsupported for unpriv %pd\n",
               &pdev->sbdf, pdev->domain);
        return -EACCES;
    }

    pdev->vpci->sriov = xzalloc(struct vpci_sriov);
    if ( !pdev->vpci->sriov )
        return -ENOMEM;

    /*
     * We need to modify vf_rlen in control_write but we can't do it cleanly
     * from pdev because write callback only accepts const pdev. Moving vf_rlen
     * inside of struct vpci_sriov is also not possible because it is used
     * before vpci init. So pass it here as additional data to not require
     * dropping const in control_write.
     */
    return vpci_add_register(pdev->vpci, vpci_hw_read16, control_write,
                             pos + PCI_SRIOV_CTRL, 2, &pdev->physfn.vf_rlen);
}

static int cf_check cleanup_sriov(const struct pci_dev *pdev, bool hide)
{
    unsigned int pos;
    int rc;

    if ( !pdev->vpci->sriov )
        return 0;

    ASSERT(!pdev->info.is_virtfn);
    ASSERT(list_empty(&pdev->vf_list));


    if ( !hide )
    {
        XFREE(pdev->vpci->sriov);
        return 0;
    }

    pos = pci_find_ext_capability(pdev, PCI_EXT_CAP_ID_SRIOV);
    rc = vpci_remove_registers(pdev->vpci, pos + PCI_SRIOV_CTRL, 2);
    if ( rc )
    {
        printk(XENLOG_ERR "%pd %pp: fail to remove SRIOV handlers rc=%d\n",
                pdev->domain, &pdev->sbdf, rc);
        ASSERT_UNREACHABLE();
        return rc;
    }
    XFREE(pdev->vpci->sriov);

    /*
    * Unprivileged domains have a deny by default register access policy, no
    * need to add any further handlers for them.
    */
   if ( !is_hardware_domain(pdev->domain) )
       return 0;

   rc = vpci_add_register(pdev->vpci, vpci_hw_read16, NULL,
                          pos + PCI_SRIOV_CTRL, 2, NULL);
   if ( rc )
       printk(XENLOG_ERR "%pd %pp: fail to add SRIOV ctrl handler rc=%d\n",
              pdev->domain, &pdev->sbdf, rc);

    return rc;
}

REGISTER_VPCI_EXTCAP(SRIOV, init_sriov, cleanup_sriov);

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
