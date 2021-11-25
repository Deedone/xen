/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Handlers for accesses to the SR-IOV capability structure.
 *
 * Copyright (C) 2026 Citrix Systems R&D
 */

#include <xen/sched.h>
#include <xen/softirq.h>
#include <xen/time.h>
#include <xen/vpci.h>

#include <xsm/xsm.h>

#include "private.h"

static int vf_init_bars(struct pci_dev *vf_pdev)
{
    int vf_idx;
    unsigned int i;
    const struct pci_dev *pf_pdev = vf_pdev->pf_pdev;
    struct vpci_bar *bars = vf_pdev->vpci->header.bars;
    struct vpci_bar *physfn_vf_bars = pf_pdev->vpci->sriov->vf_bars;
    struct vpci_sriov *sriov = pf_pdev->vpci->sriov;
    unsigned int sriov_pos = pci_find_ext_capability(pf_pdev,
                                                     PCI_EXT_CAP_ID_SRIOV);
    uint16_t offset = pci_conf_read16(pf_pdev->sbdf,
                                      sriov_pos + PCI_SRIOV_VF_OFFSET);
    uint16_t stride = pci_conf_read16(pf_pdev->sbdf,
                                      sriov_pos + PCI_SRIOV_VF_STRIDE);

    vf_idx = vf_pdev->sbdf.sbdf - (pf_pdev->sbdf.sbdf + offset);
    if ( vf_idx < 0 || vf_idx >= sriov->num_vfs )
        return -EINVAL;

    if ( sriov->num_vfs > 1 && !stride )
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
        struct vpci_bar *pf_bar = &physfn_vf_bars[i];
        struct vpci_bar *bar = &bars[i];

        if ( pf_bar->type != VPCI_BAR_MEM32 &&
             pf_bar->type != VPCI_BAR_MEM64_LO )
            continue;

        bar->addr         = pf_bar->addr + vf_idx * pf_bar->size;
        bar->guest_addr   = bar->addr;
        bar->size         = pf_bar->size;
        bar->type         = pf_bar->type;
        bar->prefetchable = pf_bar->prefetchable;
    }

    return 0;
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

        rc = vpci_add_register(vf_pdev->vpci, vpci_hw_read16, NULL,
                               PCI_VENDOR_ID, 2, NULL);
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

        for ( unsigned int i = 0; i < PCI_SRIOV_NUM_BARS; i++ )
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
    }

    return rc;
}

static int map_vfs(const struct pci_dev *pf_pdev, uint16_t cmd)
{
    struct pci_dev *vf_pdev;
    int rc;

    ASSERT(rw_is_write_locked(&pf_pdev->domain->pci_lock));

    list_for_each_entry(vf_pdev, &pf_pdev->vf_list, vf_list)
    {
        /* Don't try to change mapping if it is already in the correct state. */
        if ( vf_pdev->vpci->header.bars_mapped == !!cmd )
            continue;

        rc = vpci_modify_bars(vf_pdev, cmd, false);
        if ( rc )
        {
            gprintk(XENLOG_ERR, "failed to %s VF %pp: %d\n",
                    (cmd & PCI_COMMAND_MEMORY) ? "map" : "unmap",
                    &vf_pdev->sbdf, rc);
            return rc;
        }
    }

    return 0;
}

static void size_vf_bars(const struct pci_dev *pf_pdev, unsigned int sriov_pos,
                         uint64_t *vf_rlen)
{
    struct vpci_bar *bars = pf_pdev->vpci->sriov->vf_bars;
    unsigned int i;
    int rc = 0;

    ASSERT(rw_is_write_locked(&pf_pdev->domain->pci_lock));
    ASSERT(!pf_pdev->info.is_virtfn);
    ASSERT(pf_pdev->vpci->sriov);

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
            rc = 1;
        }
    }
}

/*
 * A VF need not respond to config space accesses until 100ms after VF Enable
 * has been set, and adding a device needs locks which control_write() holds.
 * Both are dealt with by marking the BDFs of the VFs: accesses to a marked
 * BDF are terminated by vpci_{read,write}(), and the first one made past the
 * window adds the VF to Xen.  Xen's window starts with the write to hardware,
 * ahead of the guest's, so a guest honouring its own 100ms never has an access
 * terminated.
 */
static pci_sbdf_t vf_sbdf(const struct pci_dev *pf_pdev, unsigned int idx)
{
    const struct vpci_sriov *sriov = pf_pdev->vpci->sriov;
    pci_sbdf_t sbdf = pf_pdev->sbdf;

    sbdf.bdf += sriov->offset + sriov->stride * idx;

    return sbdf;
}

static void mark_vfs(const struct pci_dev *pf_pdev)
{
    struct vpci_sriov *sriov = pf_pdev->vpci->sriov;
    unsigned int i;

    /* VF Offset and VF Stride are only valid while VF Enable is set. */
    sriov->offset = pci_conf_read16(pf_pdev->sbdf,
                                    sriov->pos + PCI_SRIOV_VF_OFFSET);
    sriov->stride = pci_conf_read16(pf_pdev->sbdf,
                                    sriov->pos + PCI_SRIOV_VF_STRIDE);
    sriov->vfs_ready = NOW() + MILLISECS(100);

    for ( i = 0; i < sriov->num_vfs; i++ )
    {
        pci_sbdf_t sbdf = vf_sbdf(pf_pdev, i);
        int rc = pci_vf_setup_begin(sbdf);

        if ( rc )
            gprintk(XENLOG_WARNING, "%pp: cannot set up VF %pp: %d\n",
                    &pf_pdev->sbdf, &sbdf, rc);
    }
}

static void unmark_vfs(const struct pci_dev *pf_pdev)
{
    struct vpci_sriov *sriov = pf_pdev->vpci->sriov;
    unsigned int i;

    if ( !sriov->vfs_ready )
        return;

    for ( i = 0; i < sriov->num_vfs; i++ )
        pci_vf_setup_end(vf_sbdf(pf_pdev, i));

    sriov->vfs_ready = 0;
}

/* Find the PF which has enabled @sbdf, if that VF is ready to be added. */
static const struct pci_dev *ready_pf(const struct domain *d, pci_sbdf_t sbdf)
{
    struct pci_dev *pdev;

    ASSERT(rw_is_locked(&d->pci_lock));

    for_each_pdev ( d, pdev )
    {
        const struct vpci_sriov *sriov = pdev->vpci ? pdev->vpci->sriov : NULL;
        unsigned int i;

        if ( !sriov || !sriov->vfs_ready || NOW() < sriov->vfs_ready )
            continue;

        for ( i = 0; i < sriov->num_vfs; i++ )
            if ( vf_sbdf(pdev, i).sbdf == sbdf.sbdf )
                return pdev;
    }

    return NULL;
}

bool vpci_sriov_add_vf(pci_sbdf_t sbdf)
{
    struct domain *d = current->domain;
    struct pci_dev_info info = { .is_virtfn = true };
    const struct pci_dev *pf;
    pci_sbdf_t pf_sbdf;
    nodeid_t node;
    int rc;

    read_lock(&d->pci_lock);
    pf = ready_pf(d, sbdf);
    if ( !pf )
    {
        read_unlock(&d->pci_lock);
        gprintk(XENLOG_DEBUG, "%pp: config space access during VF setup\n",
                &sbdf);
        return false;
    }

    pf_sbdf = pf->sbdf;
    info.physfn.bus = pf_sbdf.bus;
    info.physfn.devfn = pf_sbdf.devfn;
    node = pf->node;
    read_unlock(&d->pci_lock);

    rc = pci_add_device(d, sbdf.seg, sbdf.bus, sbdf.devfn, &info, node);
    if ( rc )
        gprintk(XENLOG_ERR, "%pp: cannot add VF %pp: %d\n", &pf_sbdf, &sbdf,
                rc);

    /*
     * Stop terminating accesses either way: a VF which Xen failed to add is
     * left for the hardware domain to register.
     */
    pci_vf_setup_end(sbdf);

    return true;
}

void vpci_sriov_drop_vfs(void)
{
    struct vcpu *curr = current;
    pci_sbdf_t pf_sbdf = curr->vpci.drop_vfs_pf;
    struct pci_dev *pf, *vf, *tmp;

    if ( likely(pf_sbdf.sbdf == ~0U) )
        return;

    curr->vpci.drop_vfs_pf.sbdf = ~0U;

    /*
     * The unmapping queued by control_write() still references the VF pdevs,
     * so it has to complete before they are freed.
     */
    while ( vpci_process_pending(curr) )
        process_pending_softirqs();

    pcidevs_lock();
    pf = pci_get_pdev(NULL, pf_sbdf);
    if ( pf )
    {
        list_for_each_entry_safe(vf, tmp, &pf->vf_list, vf_list)
        {
            pci_sbdf_t sbdf = vf->sbdf;
            int rc = pci_remove_device(sbdf.seg, sbdf.bus, sbdf.devfn);

            if ( rc )
                gprintk(XENLOG_ERR, "%pp: cannot remove VF %pp: %d\n",
                        &pf_sbdf, &sbdf, rc);
        }
    }
    pcidevs_unlock();
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

    if ( enabled && !new_enabled )
    {
        unmark_vfs(pdev);

        /* The VFs cease to exist below, so Xen has to remove them. */
        if ( !list_empty(&pdev->vf_list) )
            current->vpci.drop_vfs_pf = pdev->sbdf;
    }

    if ( !enabled && new_enabled )
    {
        size_vf_bars(pdev, sriov_pos, data);

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

    /* The VFs only come into existence with the write above. */
    if ( !enabled && new_enabled )
        mark_vfs(pdev);
}

static int cf_check init_sriov(struct pci_dev *pdev)
{
    unsigned int pos;

    if ( pdev->info.is_virtfn )
        return -EINVAL;

    pos = pci_find_ext_capability(pdev, PCI_EXT_CAP_ID_SRIOV);

    ASSERT(pos);

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

    pdev->vpci->sriov->pos = pos;

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

    unmark_vfs(pdev);

    if ( !list_empty(&pdev->vf_list) )
    {
        struct pci_dev *vf_pdev, *temp;

        printk(XENLOG_WARNING 
               "Attempting to remove SR-IOV PF %pp with VFs still present, VFs will be forcefully removed first\n",
               &pdev->sbdf);

        map_vfs(pdev, 0);
        list_for_each_entry_safe_reverse(vf_pdev, temp, &pdev->vf_list, vf_list)
        {
            pci_remove_device(vf_pdev->sbdf.seg, vf_pdev->sbdf.bus,
                              vf_pdev->sbdf.devfn);
        }
    }

    pos = pdev->vpci->sriov->pos;
    if ( !hide )
    {
        XFREE(pdev->vpci->sriov);
        return 0;
    }

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
