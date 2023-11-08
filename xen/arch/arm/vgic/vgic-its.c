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

#define COLLECTION_NOT_MAPPED ((u32)~0)

struct its_collection {
    struct list_head coll_list;

    u32 collection_id;
    u32 target_addr;
};

struct its_ite {
    struct list_head ite_list;

    struct vgic_irq *irq;
    struct its_collection *collection;
    u32 event_id;
};

struct vgic_translation_cache_entry {
    struct list_head entry;
    paddr_t db;
    u32 devid;
    u32 eventid;
    struct vgic_irq *irq;
};

/*
 * Find and returns a device in the device table for an ITS.
 * Must be called with the its_devices_lock mutex held.
 */
static struct vgic_its_device *find_its_device(struct vgic_its *its, u32 device_id)
{
    struct vgic_its_device *device;

    list_for_each_entry(device, &its->device_list, dev_list)
        if ( device_id == device->guest_devid )
            return device;

    return NULL;
}

#define VGIC_ITS_TYPER_IDBITS           16
#define VGIC_ITS_TYPER_DEVBITS          16
#define VGIC_ITS_TYPER_ITE_SIZE         8

#define GIC_LPI_OFFSET              8192

#define LPI_PROP_ENABLE_BIT(p) ((p)&LPI_PROP_ENABLED)
#define LPI_PROP_PRIORITY(p)   ((p)&0xfc)

/*
 * Reads the configuration data for a given LPI from guest memory and
 * updates the fields in struct vgic_irq.
 * If filter_vcpu is not NULL, applies only if the IRQ is targeting this
 * VCPU. Unconditionally applies if filter_vcpu is NULL.
 */
static int update_lpi_config(struct domain *d, struct vgic_irq *irq,
                             struct vcpu *filter_vcpu, bool needs_inv)
{
    u64 propbase = GICR_PROPBASER_ADDRESS(d->arch.vgic.propbaser);
    u8 prop;
    int ret;
    unsigned long flags;

    ret = access_guest_memory_by_gpa(d, propbase + irq->intid - GIC_LPI_OFFSET,
                                     &prop, 1, false);

    if ( ret )
        return ret;

    spin_lock_irqsave(&irq->irq_lock, flags);

    if ( !filter_vcpu || filter_vcpu == irq->target_vcpu )
    {
        irq->priority = LPI_PROP_PRIORITY(prop);
        irq->enabled  = LPI_PROP_ENABLE_BIT(prop);

        if ( !irq->hw )
        {
            vgic_queue_irq_unlock(d, irq, flags);
            return 0;
        }
    }

    spin_unlock_irqrestore(&irq->irq_lock, flags);

    /* GICv4 style VLPIS are not yet supported */
    WARN_ON(irq->hw);

    return 0;
}


/*
 * Create a snapshot of the current LPIs targeting @vcpu, so that we can
 * enumerate those LPIs without holding any lock.
 * Returns their number and puts the kmalloc'ed array into intid_ptr.
 */
int vgic_copy_lpi_list(struct domain *d, struct vcpu *vcpu, u32 **intid_ptr)
{
    struct vgic_dist *dist = &d->arch.vgic;
    struct vgic_irq *irq;
    unsigned long flags;
    u32 *intids;
    int irq_count, i = 0;

    /*
     * There is an obvious race between allocating the array and LPIs
     * being mapped/unmapped. If we ended up here as a result of a
     * command, we're safe (locks are held, preventing another
     * command). If coming from another path (such as enabling LPIs),
     * we must be careful not to overrun the array.
     */
    irq_count = ACCESS_ONCE(dist->lpi_list_count);
    intids    = xmalloc_array(u32, irq_count);
    if ( !intids )
        return -ENOMEM;

    spin_lock_irqsave(&dist->lpi_list_lock, flags);
    list_for_each_entry(irq, &dist->lpi_list_head, lpi_list)
    {
        if ( i == irq_count )
            break;
        /* We don't need to "get" the IRQ, as we hold the list lock. */
        if ( vcpu && irq->target_vcpu != vcpu )
            continue;
        intids[i++] = irq->intid;
    }
    spin_unlock_irqrestore(&dist->lpi_list_lock, flags);

    *intid_ptr = intids;
    return i;
}

void __vgic_put_lpi_locked(struct domain *d, struct vgic_irq *irq)
{
    struct vgic_dist *dist = &d->arch.vgic;

    if ( !atomic_dec_and_test(&irq->refcount) )
    {
        return;
    };

    list_del(&irq->lpi_list);
    dist->lpi_list_count--;

    xfree(irq);
}

static struct vgic_irq *__vgic_its_check_cache(struct vgic_dist *dist,
                                               paddr_t db, u32 devid,
                                               u32 eventid)
{
    struct vgic_translation_cache_entry *cte, *fcte;

    list_for_each_entry(cte, &dist->lpi_translation_cache, entry)
    {
        /*
         * If we hit a NULL entry, there is nothing after this
         * point.
         */
        if ( !cte->irq )
            break;

        if ( cte->db != db || cte->devid != devid || cte->eventid != eventid )
            continue;

        /*
         * Move this entry to the head, as it is the most
         * recently used.
         */
        fcte = list_first_entry(&dist->lpi_translation_cache,
                                struct vgic_translation_cache_entry, entry);

        if ( fcte->irq != cte->irq )
            list_move(&cte->entry, &dist->lpi_translation_cache);

        return cte->irq;
    }

    return NULL;
}

static struct vgic_irq *vgic_its_check_cache(struct domain *d, paddr_t db,
					     u32 devid, u32 eventid)
{
	struct vgic_dist *dist = &d->arch.vgic;
	struct vgic_irq *irq;

	spin_lock(&dist->lpi_list_lock);
	irq = __vgic_its_check_cache(dist, db, devid, eventid);
	spin_unlock(&dist->lpi_list_lock);

	return irq;
}

static void vgic_its_cache_translation(struct domain *d, struct vgic_its *its,
                                       u32 devid, u32 eventid,
                                       struct vgic_irq *irq)
{
    struct vgic_dist *dist = &d->arch.vgic;
    struct vgic_translation_cache_entry *cte;
    unsigned long flags;
    paddr_t db;

    /* Do not cache a directly injected interrupt */
    if ( irq->hw )
        return;

    spin_lock_irqsave(&dist->lpi_list_lock, flags);

    if ( unlikely(list_empty(&dist->lpi_translation_cache)) )
        goto out;

    /*
     * We could have raced with another CPU caching the same
     * translation behind our back, so let's check it is not in
     * already
     */
    db = its->vgic_its_base + GITS_TRANSLATER;
    if ( __vgic_its_check_cache(dist, db, devid, eventid) )
        goto out;

    /* Always reuse the last entry (LRU policy) */
    cte = list_last_entry(&dist->lpi_translation_cache, typeof(*cte), entry);

    /*
     * Caching the translation implies having an extra reference
     * to the interrupt, so drop the potential reference on what
     * was in the cache, and increment it on the new interrupt.
     */
    if ( cte->irq )
        __vgic_put_lpi_locked(d, cte->irq);

    vgic_get_irq_kref(irq);

    cte->db      = db;
    cte->devid   = devid;
    cte->eventid = eventid;
    cte->irq     = irq;

    /* Move the new translation to the head of the list */
    list_move(&cte->entry, &dist->lpi_translation_cache);

out:
    spin_unlock_irqrestore(&dist->lpi_list_lock, flags);
}

void vgic_its_invalidate_cache(struct domain *d)
{
    struct vgic_dist *dist = &d->arch.vgic;
    struct vgic_translation_cache_entry *cte;
    unsigned long flags;

    spin_lock_irqsave(&dist->lpi_list_lock, flags);

    list_for_each_entry(cte, &dist->lpi_translation_cache, entry)
    {
        /*
         * If we hit a NULL entry, there is nothing after this
         * point.
         */
        if ( !cte->irq )
            break;

        __vgic_put_lpi_locked(d, cte->irq);
        cte->irq = NULL;
    }

    spin_unlock_irqrestore(&dist->lpi_list_lock, flags);
}

/* Requires the its_lock to be held. */
static void its_free_ite(struct domain *d, struct its_ite *ite)
{
    list_del(&ite->ite_list);

    /* This put matches the get in vgic_add_lpi. */
    if ( ite->irq )
    {
        /* GICv4 style VLPIS are not yet supported */
        WARN_ON(ite->irq->hw);

        vgic_put_irq(d, ite->irq);
    }

    xfree(ite);
}

/* Requires the its_devices_lock to be held. */
void vgic_its_free_device(struct vgic_its_device *device)
{
    struct its_ite *ite, *temp;
    struct domain *d = device->d;
    
    BUG_ON(!d);
    /*
     * The spec says that unmapping a device with still valid
     * ITTEs associated is UNPREDICTABLE. We remove all ITTEs,
     * since we cannot leave the memory unreferenced.
     */
    list_for_each_entry_safe(ite, temp, &device->itt_head, ite_list)
        its_free_ite(d, ite);

    vgic_its_invalidate_cache(d);

    list_del(&device->dev_list);
    xfree(device);
}

/* its lock must be held */
static void vgic_its_free_device_list(struct domain *d, struct vgic_its *its)
{
    struct vgic_its_device *cur, *temp;

    list_for_each_entry_safe(cur, temp, &its->device_list, dev_list)
        vgic_its_free_device(cur);
}

/* its lock must be held */
static void vgic_its_free_collection_list(struct domain *d,
                                          struct vgic_its *its)
{
    struct its_collection *cur, *temp;

    list_for_each_entry_safe(cur, temp, &its->collection_list, coll_list)
    {
        list_del(&cur->coll_list);
        xfree(cur);
    }
}

/* Must be called with its_devices_lock mutex held */
struct vgic_its_device* vgic_its_get_device(struct domain *d, paddr_t vdoorbell,
                                         uint32_t vdevid)
{
    struct vgic_its *its = d->arch.vgic.its;
    struct vgic_its_device *device;

    if ( !its )
        return NULL;

    device = find_its_device(its, vdevid);
    if ( !device )
        return NULL;

    return device;
}

/* Must be called with its_devices_lock mutex held */
struct vgic_its_device *vgic_its_alloc_device(int nr_events)
{
    struct vgic_its_device *device;

    device = xzalloc(struct vgic_its_device);
    if ( !device )
        goto fail;

    INIT_LIST_HEAD(&device->itt_head);

    device->host_lpi_blocks = xzalloc_array(uint32_t, nr_events);
    if ( !device->host_lpi_blocks )
        goto fail_host_lpi;

    return device;
fail_host_lpi:
    xfree(device);
fail:
    return NULL;
}

/* Must be called with its_devices_lock mutex held */
int vgic_its_add_device(struct domain *d, struct vgic_its_device *its_dev)
{
    struct vgic_its *its = d->arch.vgic.its;
    if ( !its )
        return -EINVAL;

    list_add_tail(&its_dev->dev_list, &its->device_list);

    return 0;
}

/* Must be called with its_devices_lock mutex held */
void vgic_its_delete_device(struct domain *d, struct vgic_its_device *its_dev)
{
    struct vgic_its *its = d->arch.vgic.its;
    if ( !its )
        return;

    list_del(&its_dev->dev_list);
}

/*
 * This function is called with the its_cmd lock held, but the ITS data
 * structure lock dropped.
 */
static int vgic_its_handle_command(struct domain *d, struct vgic_its *its,
                                   u64 *its_cmd)
{

    return -ENODEV;
}

#define ITS_CMD_BUFFER_SIZE(baser) ((((baser)&0xff) + 1) << 12)
#define ITS_CMD_SIZE               32
#define ITS_CMD_OFFSET(reg)        ((reg)&GENMASK(19, 5))

/* Must be called with the cmd_lock held. */
static void vgic_its_process_commands(struct domain *d, struct vgic_its *its)
{
    paddr_t cbaser;
    u64 cmd_buf[4];

    /* Commands are only processed when the ITS is enabled. */
    if ( !its->enabled )
        return;

    cbaser = GITS_CBASER_ADDRESS(its->cbaser);

    while ( its->cwriter != its->creadr )
    {
        int ret = access_guest_memory_by_gpa(d, cbaser + its->creadr, cmd_buf,
                                             ITS_CMD_SIZE, false);
        /*
         * If kvm_read_guest() fails, this could be due to the guest
         * programming a bogus value in CBASER or something else going
         * wrong from which we cannot easily recover.
         * According to section 6.3.2 in the GICv3 spec we can just
         * ignore that command then.
         */
        if ( !ret )
            vgic_its_handle_command(d, its, cmd_buf);

        its->creadr += ITS_CMD_SIZE;
        if ( its->creadr == ITS_CMD_BUFFER_SIZE(its->cbaser) )
            its->creadr = 0;
    }
}

static unsigned long vgic_mmio_read_its_ctlr(struct domain *d,
                                             struct vgic_its *its, paddr_t addr,
                                             unsigned int len)
{
    u32 reg = 0;

    spin_lock(&its->cmd_lock);
    if ( its->creadr == its->cwriter )
        reg |= GITS_CTLR_QUIESCENT;
    if ( its->enabled )
        reg |= GITS_CTLR_ENABLE;
    spin_unlock(&its->cmd_lock);

    return reg;
}

static void vgic_mmio_write_its_ctlr(struct domain *d, struct vgic_its *its,
                                     paddr_t addr, unsigned int len,
                                     unsigned long val)
{
    spin_lock(&its->cmd_lock);

    /*
     * It is UNPREDICTABLE to enable the ITS if any of the CBASER or
     * device/collection BASER are invalid
     */
    if ( !its->enabled && (val & GITS_CTLR_ENABLE) &&
         (!(its->baser_device_table & GITS_VALID_BIT) ||
          !(its->baser_coll_table & GITS_VALID_BIT) ||
          !(its->cbaser & GITS_VALID_BIT)) )
        goto out;

    its->enabled = !!(val & GITS_CTLR_ENABLE);
    if ( !its->enabled )
        vgic_its_invalidate_cache(d);

    /*
     * Try to process any pending commands. This function bails out early
     * if the ITS is disabled or no commands have been queued.
     */
    vgic_its_process_commands(d, its);

out:
    spin_unlock(&its->cmd_lock);
}

static unsigned long vgic_mmio_read_its_iidr(struct domain *d,
                                             struct vgic_its *its, paddr_t addr,
                                             unsigned int len)
{
    u32 val;

    val = (its->abi_rev << GITS_IIDR_REV_SHIFT) & GITS_IIDR_REV_MASK;
    val |= (PRODUCT_ID_KVM << GITS_IIDR_PRODUCTID_SHIFT) | IMPLEMENTER_ARM;
    return val;
}

/*
 * Sync the pending table pending bit of LPIs targeting @vcpu
 * with our own data structures. This relies on the LPI being
 * mapped before.
 */
static int its_sync_lpi_pending_table(struct vcpu *vcpu)
{
    paddr_t pendbase = GICR_PENDBASER_ADDRESS(vcpu->arch.vgic.pendbaser);
    struct vgic_irq *irq;
    int last_byte_offset = -1;
    int ret              = 0;
    u32 *intids;
    int nr_irqs, i;
    unsigned long flags;
    u8 pendmask;

    nr_irqs = vgic_copy_lpi_list(vcpu->domain, vcpu, &intids);
    if ( nr_irqs < 0 )
        return nr_irqs;

    for ( i = 0; i < nr_irqs; i++ )
    {
        int byte_offset, bit_nr;

        byte_offset = intids[i] / BITS_PER_BYTE;
        bit_nr      = intids[i] % BITS_PER_BYTE;

        /*
         * For contiguously allocated LPIs chances are we just read
         * this very same byte in the last iteration. Reuse that.
         */
        if ( byte_offset != last_byte_offset )
        {
            ret = access_guest_memory_by_gpa(vcpu->domain,
                                             pendbase + byte_offset, &pendmask,
                                             1, false);
            if ( ret )
            {
                xfree(intids);
                return ret;
            }
            last_byte_offset = byte_offset;
        }

        irq = vgic_get_irq(vcpu->domain, NULL, intids[i]);
        spin_lock_irqsave(&irq->irq_lock, flags);
        irq->pending_latch = pendmask & (1U << bit_nr);
        vgic_queue_irq_unlock(vcpu->domain, irq, flags);
        vgic_put_irq(vcpu->domain, irq);
    }

    xfree(intids);

    return ret;
}

static unsigned long vgic_mmio_read_its_typer(struct domain *d,
                                              struct vgic_its *its,
                                              paddr_t addr, unsigned int len)
{
    u64 reg                        = GITS_TYPER_PHYSICAL;

    /*
     * We use linear CPU numbers for redistributor addressing,
     * so GITS_TYPER.PTA is 0.
     * Also we force all PROPBASER registers to be the same, so
     * CommonLPIAff is 0 as well.
     * To avoid memory waste in the guest, we keep the number of IDBits and
     * DevBits low - as least for the time being.
     */
    reg |= GIC_ENCODE_SZ(VGIC_ITS_TYPER_DEVBITS, 5) << GITS_TYPER_DEVIDS_SHIFT;
    reg |= GIC_ENCODE_SZ(VGIC_ITS_TYPER_IDBITS, 5) << GITS_TYPER_IDBITS_SHIFT;
    reg |= GIC_ENCODE_SZ(VGIC_ITS_TYPER_ITE_SIZE, 4) << GITS_TYPER_ITT_SIZE_SHIFT;

    return extract_bytes(reg, addr & 7, len);
}

static u64 vgic_sanitise_its_baser(u64 reg)
{
    reg = vgic_sanitise_field(reg, GITS_BASER_SHAREABILITY_MASK,
                              GITS_BASER_SHAREABILITY_SHIFT,
                              vgic_sanitise_shareability);
    reg = vgic_sanitise_field(reg, GITS_BASER_INNER_CACHEABILITY_MASK,
                              GITS_BASER_INNER_CACHEABILITY_SHIFT,
                              vgic_sanitise_inner_cacheability);
    reg = vgic_sanitise_field(reg, GITS_BASER_OUTER_CACHEABILITY_MASK,
                              GITS_BASER_OUTER_CACHEABILITY_SHIFT,
                              vgic_sanitise_outer_cacheability);

    /* We support only one (ITS) page size: 64K */
    reg = (reg & ~GITS_BASER_PAGE_SIZE_MASK) | GITS_BASER_PAGE_SIZE_64K;

    return reg;
}

static u64 vgic_sanitise_its_cbaser(u64 reg)
{
    reg = vgic_sanitise_field(reg, GITS_CBASER_SHAREABILITY_MASK,
                              GITS_CBASER_SHAREABILITY_SHIFT,
                              vgic_sanitise_shareability);
    reg = vgic_sanitise_field(reg, GITS_CBASER_INNER_CACHEABILITY_MASK,
                              GITS_CBASER_INNER_CACHEABILITY_SHIFT,
                              vgic_sanitise_inner_cacheability);
    reg = vgic_sanitise_field(reg, GITS_CBASER_OUTER_CACHEABILITY_MASK,
                              GITS_CBASER_OUTER_CACHEABILITY_SHIFT,
                              vgic_sanitise_outer_cacheability);

    /* Sanitise the physical address to be 64k aligned. */
    reg &= ~GENMASK_ULL(15, 12);

    return reg;
}

static unsigned long vgic_mmio_read_its_cbaser(struct domain *d,
                                               struct vgic_its *its,
                                               paddr_t addr, unsigned int len)
{
    return extract_bytes(its->cbaser, addr & 7, len);
}

static void vgic_mmio_write_its_cbaser(struct domain *d, struct vgic_its *its,
                                       paddr_t addr, unsigned int len,
                                       unsigned long val)
{
    /* When GITS_CTLR.Enable is 1, this register is RO. */
    if ( its->enabled )
        return;

    spin_lock(&its->cmd_lock);
    its->cbaser = update_64bit_reg(its->cbaser, addr & 7, len, val);
    its->cbaser = vgic_sanitise_its_cbaser(its->cbaser);
    its->creadr = 0;
    /*
     * CWRITER is architecturally UNKNOWN on reset, but we need to reset
     * it to CREADR to make sure we start with an empty command buffer.
     */
    its->cwriter = its->creadr;
    spin_unlock(&its->cmd_lock);
}

static unsigned long vgic_mmio_read_its_cwriter(struct domain *d,
                                                struct vgic_its *its,
                                                paddr_t addr, unsigned int len)
{
    return extract_bytes(its->cwriter, addr & 0x7, len);
}

/*
 * By writing to CWRITER the guest announces new commands to be processed.
 * To avoid any races in the first place, we take the its_cmd lock, which
 * protects our ring buffer variables, so that there is only one user
 * per ITS handling commands at a given time.
 */
static void vgic_mmio_write_its_cwriter(struct domain *d, struct vgic_its *its,
                                        paddr_t addr, unsigned int len,
                                        unsigned long val)
{
    u64 reg;

    if ( !its )
        return;

    spin_lock(&its->cmd_lock);

    reg = update_64bit_reg(its->cwriter, addr & 7, len, val);
    reg = ITS_CMD_OFFSET(reg);
    if ( reg >= ITS_CMD_BUFFER_SIZE(its->cbaser) )
    {
        spin_unlock(&its->cmd_lock);
        return;
    }
    its->cwriter = reg;

    vgic_its_process_commands(d, its);

    spin_unlock(&its->cmd_lock);
}

static unsigned long vgic_mmio_read_its_creadr(struct domain *d,
                                               struct vgic_its *its,
                                               paddr_t addr, unsigned int len)
{
    return extract_bytes(its->creadr, addr & 0x7, len);
}

#define BASER_INDEX(addr) (((addr) / sizeof(u64)) & 0x7)

static unsigned long vgic_mmio_read_its_baser(struct domain *d,
                                              struct vgic_its *its,
                                              paddr_t addr, unsigned int len)
{
    uint64_t reg;

    switch ( BASER_INDEX(addr) )
    {
    case 0:
        reg = its->baser_device_table;
        break;
    case 1:
        reg = its->baser_coll_table;
        break;
    default:
        reg = 0;
        break;
    }

    return extract_bytes(reg, addr & 7, len);
}

#define GITS_BASER_RO_MASK (GENMASK_ULL(52, 48) | GENMASK_ULL(58, 56))
#define VGIC_ITS_BASER_DTE_SIZE 8
#define VGIC_ITS_BASER_CTE_SIZE 8

static void vgic_mmio_write_its_baser(struct domain *d, struct vgic_its *its,
                                      paddr_t addr, unsigned int len,
                                      unsigned long val)
{
    u64 entry_size, table_type;
    u64 reg, *regptr, clearbits = 0;

    /* When GITS_CTLR.Enable is 1, we ignore write accesses. */
    if ( its->enabled )
        return;

    switch ( BASER_INDEX(addr) )
    {
    case 0:
        regptr     = &its->baser_device_table;
        entry_size = VGIC_ITS_BASER_DTE_SIZE;
        table_type = GITS_BASER_TYPE_DEVICE;
        break;
    case 1:
        regptr     = &its->baser_coll_table;
        entry_size = VGIC_ITS_BASER_CTE_SIZE;
        table_type = GITS_BASER_TYPE_COLLECTION;
        clearbits  = GITS_BASER_INDIRECT;
        break;
    default:
        return;
    }

    reg = update_64bit_reg(*regptr, addr & 7, len, val);
    reg &= ~GITS_BASER_RO_MASK;
    reg &= ~clearbits;

    reg |= (entry_size - 1) << GITS_BASER_ENTRY_SIZE_SHIFT;
    reg |= table_type << GITS_BASER_TYPE_SHIFT;
    reg     = vgic_sanitise_its_baser(reg);

    *regptr = reg;

    if ( !(reg & GITS_BASER_VALID) )
    {
        /* Take the its_lock to prevent a race with a save/restore */
        spin_lock(&its->its_lock);
        switch ( table_type )
        {
        case GITS_BASER_TYPE_DEVICE:
            vgic_its_free_device_list(d, its);
            break;
        case GITS_BASER_TYPE_COLLECTION:
            vgic_its_free_collection_list(d, its);
            break;
        }
        spin_unlock(&its->its_lock);
    }
}


static unsigned long vgic_mmio_read_its_idregs(struct domain *d,
                                               struct vgic_its *its,
                                               paddr_t addr, unsigned int len)
{
    switch ( addr & 0xffff )
    {
    case GITS_PIDR0:
        return 0x92; /* part number, bits[7:0] */
    case GITS_PIDR1:
        return 0xb4; /* part number, bits[11:8] */
    case GITS_PIDR2:
        return GIC_PIDR2_ARCH_GICv3 | 0x0b;
    case GITS_PIDR4:
        return 0x40; /* This is a 64K software visible page */
    /* The following are the ID registers for (any) GIC. */
    case GITS_CIDR0:
        return 0x0d;
    case GITS_CIDR1:
        return 0xf0;
    case GITS_CIDR2:
        return 0x05;
    case GITS_CIDR3:
        return 0xb1;
    }

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
                        vgic_mmio_read_its_ctlr, vgic_mmio_write_its_ctlr, 4,
                        VGIC_ACCESS_32bit),
    REGISTER_ITS_DESC(GITS_IIDR,
                        vgic_mmio_read_its_iidr, its_mmio_write_wi, 4,
                        VGIC_ACCESS_32bit),
    REGISTER_ITS_DESC(GITS_TYPER,
                        vgic_mmio_read_its_typer, its_mmio_write_wi, 8,
                        VGIC_ACCESS_64bit | VGIC_ACCESS_32bit),
    REGISTER_ITS_DESC(GITS_CBASER,
                        vgic_mmio_read_its_cbaser, vgic_mmio_write_its_cbaser, 8,
                        VGIC_ACCESS_64bit | VGIC_ACCESS_32bit),
    REGISTER_ITS_DESC(GITS_CWRITER, 
                        vgic_mmio_read_its_cwriter, vgic_mmio_write_its_cwriter, 8,
                        VGIC_ACCESS_64bit | VGIC_ACCESS_32bit),
    REGISTER_ITS_DESC(GITS_CREADR,
                        vgic_mmio_read_its_creadr, its_mmio_write_wi, 8,
                        VGIC_ACCESS_64bit | VGIC_ACCESS_32bit),
    REGISTER_ITS_DESC(GITS_BASER0,
                        vgic_mmio_read_its_baser, vgic_mmio_write_its_baser, 0x40,
                        VGIC_ACCESS_64bit | VGIC_ACCESS_32bit),
    REGISTER_ITS_DESC(GITS_IDREGS_BASE,
                        vgic_mmio_read_its_idregs, its_mmio_write_wi, 0x30,
                        VGIC_ACCESS_32bit),
};

/* This is called on setting the LPI enable bit in the redistributor. */
void vgic_enable_lpis(struct vcpu *vcpu)
{
    if ( !(vcpu->arch.vgic.pendbaser & GICR_PENDBASER_PTZ) )
        its_sync_lpi_pending_table(vcpu);
}

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

/* Default is 16 cached LPIs per vcpu */
#define LPI_DEFAULT_PCPU_CACHE_SIZE 16

void vgic_lpi_translation_cache_init(struct domain *d)
{
    struct vgic_dist *dist = &d->arch.vgic;
    unsigned int sz;
    int i;

    if ( !list_empty(&dist->lpi_translation_cache) )
        return;

    sz = d->max_vcpus * LPI_DEFAULT_PCPU_CACHE_SIZE;

    for ( i = 0; i < sz; i++ )
    {
        struct vgic_translation_cache_entry *cte;

        /* An allocation failure is not fatal */
        cte = xzalloc(struct vgic_translation_cache_entry);
        if ( WARN_ON(!cte) )
            break;

        INIT_LIST_HEAD(&cte->entry);
        list_add(&cte->entry, &dist->lpi_translation_cache);
    }
}

void vgic_lpi_translation_cache_destroy(struct domain *d)
{
    struct vgic_dist *dist = &d->arch.vgic;
    struct vgic_translation_cache_entry *cte, *tmp;

    vgic_its_invalidate_cache(d);

    list_for_each_entry_safe(cte, tmp, &dist->lpi_translation_cache, entry)
    {
        list_del(&cte->entry);
        xfree(cte);
    }
}

#define INITIAL_BASER_VALUE                                                    \
    (GIC_BASER_CACHEABILITY(GITS_BASER, INNER, RaWb) |                         \
     GIC_BASER_CACHEABILITY(GITS_BASER, OUTER, SameAsInner) |                  \
     GIC_BASER_SHAREABILITY(GITS_BASER, InnerShareable) |                      \
     GITS_BASER_PAGE_SIZE_64K)

#define INITIAL_PROPBASER_VALUE                                                \
    (GIC_BASER_CACHEABILITY(GICR_PROPBASER, INNER, RaWb) |                     \
     GIC_BASER_CACHEABILITY(GICR_PROPBASER, OUTER, SameAsInner) |              \
     GIC_BASER_SHAREABILITY(GICR_PROPBASER, InnerShareable))

static int vgic_its_create(struct domain *d, u64 addr)
{
    struct vgic_its *its;

    its = xzalloc(struct vgic_its);
    if ( !its )
        return -ENOMEM;

    d->arch.vgic.its = its;

    vgic_lpi_translation_cache_init(d);

    spin_lock_init(&its->its_lock);
    spin_lock_init(&its->cmd_lock);

    its->vgic_its_base = VGIC_ADDR_UNDEF;

    INIT_LIST_HEAD(&its->device_list);
    INIT_LIST_HEAD(&its->collection_list);
    spin_lock_init(&d->arch.vgic.its_devices_lock);

    d->arch.vgic.msis_require_devid = true;
    d->arch.vgic.has_its            = true;
    its->enabled                    = false;
    its->domain                     = d;

    its->baser_device_table = INITIAL_BASER_VALUE | ((u64)GITS_BASER_TYPE_DEVICE
                                                     << GITS_BASER_TYPE_SHIFT);
    its->baser_coll_table = INITIAL_BASER_VALUE |
        ((u64)GITS_BASER_TYPE_COLLECTION << GITS_BASER_TYPE_SHIFT);
    d->arch.vgic.propbaser = INITIAL_PROPBASER_VALUE;

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

    spin_lock(&its->its_lock);
    spin_lock(&d->arch.vgic.its_devices_lock);

    vgic_its_free_device_list(d, its);
    vgic_its_free_collection_list(d, its);
    vgic_lpi_translation_cache_destroy(d);

    spin_unlock(&d->arch.vgic.its_devices_lock);
    spin_unlock(&its->its_lock);
    xfree(its);
    d->arch.vgic.its = NULL;
}
