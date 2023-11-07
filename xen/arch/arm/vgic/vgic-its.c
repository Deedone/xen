/*
 * Imported from Linux ("new" KVM VGIC) and heavily adapted to Xen.
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

#include <xen/sched.h>
#include <xen/guest_access.h>
#include <xen/sizes.h>
#include <xen/err.h>
#include <xen/list_sort.h>
#include <asm/page.h>
#include <asm/new_vgic.h>
#include <asm/gic_v3_defs.h>
#include <asm/gic_v3_its.h>

#include "vgic.h"
#include "vgic-mmio.h"

static unsigned long its_mmio_read_raz(struct domain *d, struct vgic_its *its,
                              paddr_t addr, unsigned int len)
{
    return 0;
}

static void its_mmio_write_wi(struct domain *d, struct vgic_its *its,
                              paddr_t addr, unsigned int len, unsigned long val)
{
    /* Ignore */
}

#define REGISTER_ITS_DESC(off, rd, wr, length, acc)                            \
    {                                                                          \
        .reg_offset = off, .len = length, .access_flags = acc, .its_read = rd, \
        .its_write = wr,                                                       \
    }

static struct vgic_register_region its_registers[] = {
    REGISTER_ITS_DESC(GITS_CTLR,
                        its_mmio_read_raz, its_mmio_write_wi, 4,
                        VGIC_ACCESS_32bit),
    REGISTER_ITS_DESC(GITS_IIDR,
                        its_mmio_read_raz, its_mmio_write_wi, 4,
                        VGIC_ACCESS_32bit),
    REGISTER_ITS_DESC(GITS_TYPER,
                        its_mmio_read_raz, its_mmio_write_wi, 8,
                        VGIC_ACCESS_64bit | VGIC_ACCESS_32bit),
    REGISTER_ITS_DESC(GITS_CBASER,
                        its_mmio_read_raz, its_mmio_write_wi, 8,
                        VGIC_ACCESS_64bit | VGIC_ACCESS_32bit),
    REGISTER_ITS_DESC(GITS_CWRITER, 
                        its_mmio_read_raz, its_mmio_write_wi, 8,
                        VGIC_ACCESS_64bit | VGIC_ACCESS_32bit),
    REGISTER_ITS_DESC(GITS_CREADR,
                        its_mmio_read_raz, its_mmio_write_wi, 8,
                        VGIC_ACCESS_64bit | VGIC_ACCESS_32bit),
    REGISTER_ITS_DESC(GITS_BASER0,
                        its_mmio_read_raz, its_mmio_write_wi, 0x40,
                        VGIC_ACCESS_64bit | VGIC_ACCESS_32bit),
    REGISTER_ITS_DESC(GITS_IDREGS_BASE,
                        its_mmio_read_raz, its_mmio_write_wi, 0x30,
                        VGIC_ACCESS_32bit),
};

static int vgic_register_its_iodev(struct domain *d, struct vgic_its *its,
                                   u64 addr)
{
    struct vgic_io_device *iodev = &its->iodev;
    int ret                      = 0;

    if ( !IS_VGIC_ADDR_UNDEF(its->vgic_its_base) )
    {
        ret = -EBUSY;
        goto out;
    }

    its->vgic_its_base    = addr;
    its->doorbell_address = addr + ITS_DOORBELL_OFFSET;
    iodev->regions        = its_registers;
    iodev->nr_regions     = ARRAY_SIZE(its_registers);

    iodev->base_fn        = gaddr_to_gfn(its->vgic_its_base);
    iodev->iodev_type     = IODEV_ITS;
    iodev->its            = its;
    register_mmio_handler(d, &vgic_io_ops, its->vgic_its_base, VGIC_V3_ITS_SIZE,
                          iodev);
out:
    return ret;
}

static int vgic_its_create(struct domain *d, u64 addr)
{
    struct vgic_its *its;

    its = xzalloc(struct vgic_its);
    if ( !its )
        return -ENOMEM;

    d->arch.vgic.its = its;

    its->vgic_its_base = VGIC_ADDR_UNDEF;

    d->arch.vgic.msis_require_devid = true;
    d->arch.vgic.has_its            = true;
    its->enabled                    = false;

    vgic_register_its_iodev(d, its, addr);

    its->doorbell_address = addr + ITS_DOORBELL_OFFSET;

    return 0;
}

/*
 * For a hardware domain, this will iterate over the host ITSes
 * and map one virtual ITS per host ITS at the same address.
 */
int vgic_v3_its_init_domain(struct domain *d)
{
    int ret;

    if ( is_hardware_domain(d) )
    {
        struct host_its *hw_its;

        list_for_each_entry(hw_its, &host_its_list, entry)
        {
            /*
             * For each host ITS create a virtual ITS using the same
             * base and thus doorbell address.
             * Use the same number of device ID and event ID bits as the host.
             */
            ret = vgic_its_create(d, hw_its->addr);
            if ( ret )
                return ret;
            else
                d->arch.vgic.has_its = true;
        }
    }

    return 0;
}

void vgic_v3_its_free_domain(struct domain *d)
{
    struct vgic_its *its = d->arch.vgic.its;

    xfree(its);
    d->arch.vgic.its = NULL;
}
