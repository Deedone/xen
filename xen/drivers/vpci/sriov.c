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

#include "xen/compiler.h"
#include "xen/pci_regs.h"
#include "xen/rangeset.h"
#include <xen/sched.h>
#include <xen/vpci.h>

static int vf_init_bars(const struct pci_dev *vf_pdev)
{
    unsigned int i, sriov_pos, rc;
    int vf_idx;
    const struct pci_dev *pf_pdev = vf_pdev->pf_pdev;
    uint16_t offset, stride;
    struct vpci_bar *bars = vf_pdev->vpci->header.bars;
    struct vpci_bar *physfn_vf_bars = pf_pdev->vpci->sriov->vf_bars;
    uint16_t ctrl;

    sriov_pos = pci_find_ext_capability(pf_pdev->sbdf, PCI_EXT_CAP_ID_SRIOV);
    offset = pci_conf_read16(pf_pdev->sbdf, sriov_pos + PCI_SRIOV_VF_OFFSET);
    stride = pci_conf_read16(pf_pdev->sbdf, sriov_pos + PCI_SRIOV_VF_STRIDE);
    ctrl = pci_conf_read16(pf_pdev->sbdf, sriov_pos + PCI_SRIOV_CTRL);

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
        char str[32];
        
        bars[i].addr = physfn_vf_bars[i].addr + vf_idx * physfn_vf_bars[i].size;
        bars[i].guest_addr = bars[i].addr;
        bars[i].size = physfn_vf_bars[i].size;
        bars[i].type = physfn_vf_bars[i].type;
        bars[i].prefetchable = physfn_vf_bars[i].prefetchable;
        //TODO: use vpci funciton
        bars[i].mem = rangeset_new(vf_pdev->domain, str, RANGESETF_no_print);
    }

    /* Also add handlers for the SR-IOV PF/VF BARs. */
    for ( i = 0; i < PCI_SRIOV_NUM_BARS; i++)
    {
        /*
        * FIXME: VFs ROM BAR is read-only and is all zeros. VF may provide
        * access to the PFs ROM via emulation though.
        */
        
        if ( (bars[i].type == VPCI_BAR_IO) ||
            (bars[i].type == VPCI_BAR_EMPTY) ||
            (bars[i].type == VPCI_BAR_ROM) )
            continue;
        
        
        /* This is either VPCI_BAR_MEM32 or VPCI_BAR_MEM64_{LO|HI}. */
        rc = vpci_add_register(vf_pdev->vpci,
                            vpci_guest_mem_bar_read, vpci_guest_mem_bar_write,
                            PCI_BASE_ADDRESS_0 + i * 4, 4, &bars[i]);
        if ( rc ) {
            return rc;
        }
    }
    return 0;
}

static int map_vf(const struct pci_dev *vf_pdev, uint16_t cmd)
{
    int rc;

    ASSERT(rw_is_write_locked(&vf_pdev->domain->pci_lock));

    rc = vf_init_bars(vf_pdev);
    if ( rc )
        return rc;

    return 0;
    // return vpci_modify_bars(vf_pdev, cmd, false);
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
                map_vf(vf_pdev, PCI_COMMAND_MEMORY);
        }
    }

    pci_conf_write16(pdev->sbdf, reg, val);
}

static int vf_init_header(struct pci_dev *vf_pdev)
{
    const struct pci_dev *pf_pdev;
    struct vpci_bar *bars;
    unsigned int i, sriov_pos;
    int rc = 0;
    uint16_t ctrl;

    ASSERT(rw_is_write_locked(&vf_pdev->domain->pci_lock));

    if ( !vf_pdev->info.is_virtfn )
        return 0;

    pf_pdev = vf_pdev->pf_pdev;
    ASSERT(pf_pdev);

    bars = vf_pdev->vpci->header.bars;

    sriov_pos = pci_find_ext_capability(pf_pdev->sbdf, PCI_EXT_CAP_ID_SRIOV);
    ctrl = pci_conf_read16(pf_pdev->sbdf, sriov_pos + PCI_SRIOV_CTRL);

    for ( i = 0; i < PCI_SRIOV_NUM_BARS; i++ )
    {
        rc = vpci_bar_add_rangeset(vf_pdev, &bars[i], i);
        if ( rc )
            return rc;
    }

    if ( (ctrl & PCI_SRIOV_CTRL_MSE) )
    {
        rc = map_vf(vf_pdev, PCI_COMMAND_MEMORY);
        if ( rc )
            return rc;
    }

    return rc;
}

static int init_sriov(struct pci_dev *pdev)
{
    unsigned int pos;
    int ret;

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

    printk("VPCI ADD REGISTER %x\n", pos + PCI_SRIOV_CTRL);
    ret = vpci_add_register(pdev->vpci, vpci_hw_read16, control_write,
                             pos + PCI_SRIOV_CTRL, 2, NULL);
    printk("VPCI ADD REGISTER %x\n", pos + PCI_SRIOV_NUM_VF);
    return vpci_add_register(pdev->vpci, vpci_hw_read16, vpci_hw_write16,
                             pos + PCI_SRIOV_NUM_VF, 2, NULL);
}
REGISTER_VPCI_INIT(init_sriov, VPCI_PRIORITY_LOW);

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
 
static const struct pci_dev *get_physfn_pdev(const struct pci_dev *pdev)
{
    struct domain *d;
    if ( !pdev->info.is_virtfn )
        return NULL;

    d = get_domain_by_id(0);
    printk("searching dom %d bus %x devfn %x\n", d->domain_id, pdev->info.physfn.bus, pdev->info.physfn.devfn);
    return pci_get_pdev(d, PCI_SBDF(0, pdev->info.physfn.bus, pdev->info.physfn.devfn));
}

static uint32_t guest_get_vf_ven_dev_id(const struct pci_dev *pdev)
 {
     const struct pci_dev *physfn_pdev;
     uint32_t dev_ven_id, pos;
 
     /*
      * Find the physical function for this virtual function device
      * and use its VendorID and read our DeviceID.
      */
     physfn_pdev = get_physfn_pdev(pdev);
     if ( !physfn_pdev )
     {
         gprintk(XENLOG_ERR, "%pp cannot find physfn\n",
                 &pdev->sbdf);
         return ~0;
     }
 
     /* Vendor ID is the same as the PF's Venodr ID. */
     dev_ven_id = pci_conf_read16(physfn_pdev->sbdf, PCI_VENDOR_ID);
     /* Device ID comes from the SR-IOV extended capability. */
     pos = pci_find_ext_capability(physfn_pdev->sbdf, PCI_EXT_CAP_ID_SRIOV);
     if ( !pos )
     {
         gprintk(XENLOG_ERR, "%pp cannot find SR-IOV extended capability, PF %pp\n",
                 &pdev->sbdf, &physfn_pdev->sbdf);
         return ~0;
     }
     return dev_ven_id | pci_conf_read16(physfn_pdev->sbdf,
                                         pos + PCI_SRIOV_VF_DID) << 16;
}
 
static uint32_t guest_vendor_id_read(const struct pci_dev *pdev,
                                      unsigned int reg, void *data)
{
    struct vpci_header *header = &pdev->vpci->header;

    if ( !pdev->info.is_virtfn )
        return vpci_hw_read32(pdev, reg, data);
    return header->vf_ven_dev_id;
}
 
static void guest_vendor_id_write(const struct pci_dev *pdev, unsigned int reg,
                                uint32_t val, void *data)
{
}

__maybe_unused static int vf_init_handlers(struct pci_dev *pdev)
{
     struct vpci_header *header = &pdev->vpci->header;
     int rc;
 
     if ( !pdev->info.is_virtfn )
         return 0;
 
     /* Reset the command register for the guest. */
     // vpci_ecam_write(pdev->sbdf, PCI_COMMAND, 0, header);
 
     /*
      * Setup a handler for VENDOR_ID for guests only and allow hwdom reading
      * directly: the handler is used for SR-IOV virtual functions.
      */
     header->vf_ven_dev_id = guest_get_vf_ven_dev_id(pdev);
     if ( header->vf_ven_dev_id == ~0 )
         return -EINVAL;
 
     rc = vpci_add_register(pdev->vpci,
                            guest_vendor_id_read, guest_vendor_id_write,
                            PCI_VENDOR_ID, 4, header);
     if ( rc )
         return rc;
     
     // rc = vpci_add_register_mask(pdev->vpci, vpci_hw_read8, NULL,
     //                             PCI_HEADER_TYPE, 1, NULL, 0x7f, 0, 0, 0x80);
 
     // /* Setup a handler for the command register. */
     // rc = vpci_add_register(pdev->vpci, vpci_hw_read16, vpci_ecam_write,
     //                        PCI_COMMAND, 2, header);
     // if ( rc )
     //     return rc;
 
 
     return 0;
 }
 REGISTER_VPCI_INIT(vf_init_handlers, VPCI_PRIORITY_LOW);
