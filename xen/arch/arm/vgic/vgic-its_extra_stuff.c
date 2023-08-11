#include <xen/sched.h>
#include <xen/guest_access.h>
#include <xen/sizes.h>
#include <xen/err.h>
#include <xen/list_sort.h>
#include <asm/new_vgic.h>
#include <asm/gic_v3_defs.h>
#include <asm/gic_v3_its.h>

#include "vgic.h"
#include "vgic-mmio.h"

static int vgic_its_save_tables_v0(struct vgic_its *its);
static int vgic_its_restore_tables_v0(struct vgic_its *its);
static int vgic_its_commit_v0(struct vgic_its *its);
static int update_lpi_config(struct domain *d, struct vgic_irq *irq,
			     struct vcpu *filter_vcpu, bool needs_inv);

//TODO: remove duplicate functions
/* extract @num bytes at @offset bytes offset in data */
unsigned long extract_bytes(uint64_t data, unsigned int offset,
			    unsigned int num);

uint64_t update_64bit_reg(u64 reg, unsigned int offset, unsigned int len,
		     unsigned long val);

/*
 * Creates a new (reference to a) struct vgic_irq for a given LPI.
 * If this LPI is already mapped on another ITS, we increase its refcount
 * and return a pointer to the existing structure.
 * If this is a "new" LPI, we allocate and initialize a new struct vgic_irq.
 * This function returns a pointer to the _unlocked_ structure.
 */
static struct vgic_irq *vgic_add_lpi(struct domain *d, struct vgic_its *its, u32 intid,
				     u32 devid, u32 eventid, struct vcpu *vcpu)
{
	struct vgic_dist *dist = &d->arch.vgic;
	struct vgic_irq *irq = vgic_get_irq(d, NULL, intid), *oldirq;
	unsigned long flags;
	int ret;

	/* In this case there is no put, since we keep the reference. */
	if (irq)
		return irq;

	// irq = xzalloc(struct vgic_irq);
    irq = gicv3_assign_guest_event(its->domain, its->vgic_its_base + ITS_DOORBELL_OFFSET,
                                   devid, eventid, intid);
	if (!irq)
		return ERR_PTR(-ENOMEM);

	memset(irq, 0, sizeof(*irq));

	INIT_LIST_HEAD(&irq->lpi_list);
	INIT_LIST_HEAD(&irq->ap_list);
	spin_lock_init(&irq->irq_lock);

	irq->config = VGIC_CONFIG_EDGE;
	atomic_set(&irq->refcount, 1);
	irq->intid = intid;
	irq->target_vcpu = vcpu;
	//irq->group = 1;

	spin_lock_irqsave(&dist->lpi_list_lock, flags);

	/*
	 * There could be a race with another vgic_add_lpi(), so we need to
	 * check that we don't add a second list entry with the same LPI.
	 */
	list_for_each_entry(oldirq, &dist->lpi_list_head, lpi_list) {
		if (oldirq->intid != intid)
			continue;

		/* Someone was faster with adding this LPI, lets use that. */
		xfree(irq);
		irq = oldirq;

		/*
		 * This increases the refcount, the caller is expected to
		 * call vgic_put_irq() on the returned pointer once it's
		 * finished with the IRQ.
		 */
		vgic_get_irq_kref(irq);

		goto out_unlock;
	}

	printk(XENLOG_ERR "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!ADD LPI %d\n", intid);
	list_add_tail(&irq->lpi_list, &dist->lpi_list_head);
	dist->lpi_list_count++;

out_unlock:
	spin_unlock_irqrestore(&dist->lpi_list_lock, flags);

	/*
	 * We "cache" the configuration table entries in our struct vgic_irq's.
	 * However we only have those structs for mapped IRQs, so we read in
	 * the respective config data from memory here upon mapping the LPI.
	 *
	 * Should any of these fail, behave as if we couldn't create the LPI
	 * by dropping the refcount and returning the error.
	 */
	ret = update_lpi_config(d, irq, NULL, false);
	if (ret) {
		vgic_put_irq(d, irq);
		return ERR_PTR(ret);
	}

	ret = vgic_v3_lpi_sync_pending_status(d, irq);
	if (ret) {
		vgic_put_irq(d, irq);
		return ERR_PTR(ret);
	}

	return irq;
}

struct vgic_translation_cache_entry {
	struct list_head	entry;
	paddr_t		db;
	u32			devid;
	u32			eventid;
	struct vgic_irq		*irq;
};

struct its_device {
	struct list_head dev_list;

	/* the head for the list of ITTEs */
	struct list_head itt_head;
	u32 num_eventid_bits;
	paddr_t itt_addr;
	u32 device_id;
};

#define COLLECTION_NOT_MAPPED ((u32)~0)

struct its_collection {
	struct list_head coll_list;

	u32 collection_id;
	u32 target_addr;
};

/**
 * struct vgic_its_abi - ITS abi ops and settings
 * @cte_esz: collection table entry size
 * @dte_esz: device table entry size
 * @ite_esz: interrupt translation table entry size
 * @save tables: save the ITS tables into guest RAM
 * @restore_tables: restore the ITS internal structs from tables
 *  stored in guest RAM
 * @commit: initialize the registers which expose the ABI settings,
 *  especially the entry sizes
 */
struct vgic_its_abi {
	int cte_esz;
	int dte_esz;
	int ite_esz;
	int (*save_tables)(struct vgic_its *its);
	int (*restore_tables)(struct vgic_its *its);
	int (*commit)(struct vgic_its *its);
};

#define ABI_0_ESZ	8
#define ESZ_MAX		ABI_0_ESZ

static const struct vgic_its_abi its_table_abi_versions[] = {
	[0] = {
	 .cte_esz = ABI_0_ESZ,
	 .dte_esz = ABI_0_ESZ,
	 .ite_esz = ABI_0_ESZ,
	 .save_tables = vgic_its_save_tables_v0,
	 .restore_tables = vgic_its_restore_tables_v0,
	 .commit = vgic_its_commit_v0,
	},
};

#define NR_ITS_ABIS	ARRAY_SIZE(its_table_abi_versions)

inline static const struct vgic_its_abi *vgic_its_get_abi(struct vgic_its *its)
{
	return &its_table_abi_versions[its->abi_rev];
}

static int vgic_its_set_abi(struct vgic_its *its, u32 rev)
{
	const struct vgic_its_abi *abi;

	its->abi_rev = rev;
	abi = vgic_its_get_abi(its);
	return abi->commit(its);
}
#define its_is_collection_mapped(coll) ((coll) && \
				((coll)->target_addr != COLLECTION_NOT_MAPPED))

struct its_ite {
	struct list_head ite_list;

	struct vgic_irq *irq;
	struct its_collection *collection;
	u32 event_id;
};

#define KVM_MSI_VALID_DEVID	(1U << 0)
struct xen_msi {
	__u32 address_lo;
	__u32 address_hi;
	__u32 data;
	__u32 flags;
	__u32 devid;
	__u8  pad[12];
};

/*
 * Find and returns a device in the device table for an ITS.
 * Must be called with the its_lock mutex held.
 */
static struct its_device *find_its_device(struct vgic_its *its, u32 device_id)
{
	struct its_device *device;

	list_for_each_entry(device, &its->device_list, dev_list)
		if (device_id == device->device_id)
			return device;

	return NULL;
}

/*
 * Find and returns an interrupt translation table entry (ITTE) for a given
 * Device ID/Event ID pair on an ITS.
 * Must be called with the its_lock mutex held.
 */
static struct its_ite *find_ite(struct vgic_its *its, u32 device_id,
				  u32 event_id)
{
	struct its_device *device;
	struct its_ite *ite;

	device = find_its_device(its, device_id);
	if (device == NULL)
		return NULL;

	list_for_each_entry(ite, &device->itt_head, ite_list)
		if (ite->event_id == event_id)
			return ite;

	return NULL;
}

/* To be used as an iterator this macro misses the enclosing parentheses */
#define for_each_lpi_its(dev, ite, its) \
	list_for_each_entry(dev, &(its)->device_list, dev_list) \
		list_for_each_entry(ite, &(dev)->itt_head, ite_list)

#define GIC_LPI_OFFSET 8192

#define VITS_TYPER_IDBITS 16
#define VITS_TYPER_DEVBITS 16
#define VITS_DTE_MAX_DEVID_OFFSET	(BIT(14, UL) - 1)
#define VITS_ITE_MAX_EVENTID_OFFSET	(BIT(16, UL) - 1)

static struct its_collection *find_collection(struct vgic_its *its, int coll_id)
{
	struct its_collection *collection;

	list_for_each_entry(collection, &its->collection_list, coll_list) {
		if (coll_id == collection->collection_id)
			return collection;
	}

	return NULL;
}

#define LPI_PROP_ENABLE_BIT(p)	((p) & LPI_PROP_ENABLED)
#define LPI_PROP_PRIORITY(p)	((p) & 0xfc)

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

	// ret = kvm_read_guest_lock(d, propbase + irq->intid - GIC_LPI_OFFSET,
	// 			  &prop, 1);
	ret = access_guest_memory_by_gpa(d, propbase + irq->intid - GIC_LPI_OFFSET,
									 &prop, 1, false);

	if (ret)
		return ret;

	spin_lock_irqsave(&irq->irq_lock, flags);

	if (!filter_vcpu || filter_vcpu == irq->target_vcpu) {
		irq->priority = LPI_PROP_PRIORITY(prop);
		irq->enabled = LPI_PROP_ENABLE_BIT(prop);

		if (!irq->hw) {
			vgic_queue_irq_unlock(d, irq, flags);
			return 0;
		}
	}

	spin_unlock_irqrestore(&irq->irq_lock, flags);

	// if (irq->hw)
	// 	return its_prop_update_vlpi(irq->host_irq, prop, needs_inv);

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
	irq_count = dist->lpi_list_count;
	intids = xmalloc_array(u32, irq_count);
	if (!intids)
		return -ENOMEM;

	spin_lock_irqsave(&dist->lpi_list_lock, flags);
	list_for_each_entry(irq, &dist->lpi_list_head, lpi_list) {
		if (i == irq_count)
			break;
		/* We don't need to "get" the IRQ, as we hold the list lock. */
		if (vcpu && irq->target_vcpu != vcpu)
			continue;
		intids[i++] = irq->intid;
	}
	spin_unlock_irqrestore(&dist->lpi_list_lock, flags);

	*intid_ptr = intids;
	return i;
}

static int update_affinity(struct vgic_irq *irq, struct vcpu *vcpu)
{
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&irq->irq_lock, flags);
	irq->target_vcpu = vcpu;
	spin_unlock_irqrestore(&irq->irq_lock, flags);

	if (irq->hw) {
		// struct its_vlpi_map map;

		// ret = its_get_vlpi(irq->host_irq, &map);
		// if (ret)
		// 	return ret;

		// if (map.vpe)
		// 	atomic_dec(&map.vpe->vlpi_count);
		// map.vpe = &vcpu->arch.vgic_cpu.vgic_v3.its_vpe;
		// atomic_inc(&map.vpe->vlpi_count);

		// ret = its_map_vlpi(irq->host_irq, &map);
	}

	return ret;
}

/*
 * Promotes the ITS view of affinity of an ITTE (which redistributor this LPI
 * is targeting) to the VGIC's view, which deals with target VCPUs.
 * Needs to be called whenever either the collection for a LPIs has
 * changed or the collection itself got retargeted.
 */
static void update_affinity_ite(struct domain *d, struct its_ite *ite)
{
	struct vcpu *vcpu;

	if (!its_is_collection_mapped(ite->collection))
		return;

	vcpu = d->vcpu[ite->collection->target_addr];
	update_affinity(ite->irq, vcpu);
}

/*
 * Updates the target VCPU for every LPI targeting this collection.
 * Must be called with the its_lock mutex held.
 */
static void update_affinity_collection(struct domain *d, struct vgic_its *its,
				       struct its_collection *coll)
{
	struct its_device *device;
	struct its_ite *ite;

	for_each_lpi_its(device, ite, its) {
		if (!ite->collection || coll != ite->collection)
			continue;

		update_affinity_ite(d, ite);
	}
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

static u32 max_lpis_propbaser(u64 propbaser)
{
	int nr_idbits = (propbaser & 0x1f) + 1;

	return 1U << min(nr_idbits, INTERRUPT_ID_BITS_ITS);
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
	if (irq->hw)
		return;

	spin_lock_irqsave(&dist->lpi_list_lock, flags);

	if (unlikely(list_empty(&dist->lpi_translation_cache)))
		goto out;

	/*
	 * We could have raced with another CPU caching the same
	 * translation behind our back, so let's check it is not in
	 * already
	 */
	//TODO FIX
	db = its->vgic_its_base + GITS_TRANSLATER;
	// if (__vgic_its_check_cache(dist, db, devid, eventid))
	// 	goto out;

	/* Always reuse the last entry (LRU policy) */
	cte = list_last_entry(&dist->lpi_translation_cache,
			      typeof(*cte), entry);

	/*
	 * Caching the translation implies having an extra reference
	 * to the interrupt, so drop the potential reference on what
	 * was in the cache, and increment it on the new interrupt.
	 */
	if (cte->irq)
		__vgic_put_lpi_locked(d, cte->irq);

	vgic_get_irq_kref(irq);

	cte->db		= db;
	cte->devid	= devid;
	cte->eventid	= eventid;
	cte->irq	= irq;

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

	list_for_each_entry(cte, &dist->lpi_translation_cache, entry) {
		/*
		 * If we hit a NULL entry, there is nothing after this
		 * point.
		 */
		if (!cte->irq)
			break;

		__vgic_put_lpi_locked(d, cte->irq);
		cte->irq = NULL;
	}

	spin_unlock_irqrestore(&dist->lpi_list_lock, flags);
}

int vgic_its_resolve_lpi(struct domain *d, struct vgic_its *its,
			 u32 devid, u32 eventid, struct vgic_irq **irq)
{
	struct vcpu *vcpu;
	struct its_ite *ite;

	if (!its->enabled)
		return -EBUSY;

	ite = find_ite(its, devid, eventid);
	if (!ite || !its_is_collection_mapped(ite->collection))
		return E_ITS_INT_UNMAPPED_INTERRUPT;

	vcpu = d->vcpu[ite->collection->target_addr];
	if (!vcpu)
		return E_ITS_INT_UNMAPPED_INTERRUPT;

	if (!vgic_lpis_enabled(vcpu))
		return -EBUSY;

	vgic_its_cache_translation(d, its, devid, eventid, ite->irq);

	*irq = ite->irq;
	return 0;
}

// struct vgic_its *vgic_msi_to_its(struct domain *d, struct xen_msi *msi)
// {
// 	u64 address;
// 	struct kvm_io_device *kvm_io_dev;
// 	struct vgic_io_device *iodev;

// 	if (!vgic_has_its(kvm))
// 		return ERR_PTR(-ENODEV);

// 	if (!(msi->flags & KVM_MSI_VALID_DEVID))
// 		return ERR_PTR(-EINVAL);

// 	address = (u64)msi->address_hi << 32 | msi->address_lo;

// 	kvm_io_dev = kvm_io_bus_get_dev(kvm, KVM_MMIO_BUS, address);
// 	if (!kvm_io_dev)
// 		return ERR_PTR(-EINVAL);

// 	if (kvm_io_dev->ops != &kvm_io_gic_ops)
// 		return ERR_PTR(-EINVAL);

// 	iodev = container_of(kvm_io_dev, struct vgic_io_device, dev);
// 	if (iodev->iodev_type != IODEV_ITS)
// 		return ERR_PTR(-EINVAL);

// 	return iodev->its;
// }

/*
 * Find the target VCPU and the LPI number for a given devid/eventid pair
 * and make this IRQ pending, possibly injecting it.
 * Must be called with the its_lock mutex held.
 * Returns 0 on success, a positive error value for any ITS mapping
 * related errors and negative error values for generic errors.
 */
static int vgic_its_trigger_msi(struct domain *d, struct vgic_its *its,
				u32 devid, u32 eventid)
{
	struct vgic_irq *irq = NULL;
	unsigned long flags;
	int err;

	err = vgic_its_resolve_lpi(d, its, devid, eventid, &irq);
	if (err)
		return err;

	if (irq->hw)
		BUG();
	// 	return irq_set_irqchip_state(irq->host_irq,
	// 				     IRQCHIP_STATE_PENDING, true);

	spin_lock_irqsave(&irq->irq_lock, flags);
	irq->pending_latch = true;
	vgic_queue_irq_unlock(d, irq, flags);

	return 0;
}

static u64 its_cmd_mask_field(u64 *its_cmd, int word, int shift, int size)
{
	return (le64_to_cpu(its_cmd[word]) >> shift) & (BIT(size, ULL) - 1);
}

/* Requires the its_lock to be held. */
static void its_free_ite(struct domain *d, struct its_ite *ite)
{
	list_del(&ite->ite_list);

	/* This put matches the get in vgic_add_lpi. */
	if (ite->irq) {
	// 	if (ite->irq->hw)
	// 		WARN_ON(its_unmap_vlpi(ite->irq->host_irq));

		vgic_put_irq(d, ite->irq);
	}

	xfree(ite);
}

/* Must be called with its_lock mutex held */
static struct its_ite *vgic_its_alloc_ite(struct its_device *device,
					  struct its_collection *collection,
					  u32 event_id)
{
	struct its_ite *ite;

	ite = xzalloc(struct its_ite);
	if (!ite)
		return ERR_PTR(-ENOMEM);

	ite->event_id	= event_id;
	ite->collection = collection;

	list_add_tail(&ite->ite_list, &device->itt_head);
	return ite;
}

#define its_cmd_get_command(cmd)	its_cmd_mask_field(cmd, 0,  0,  8)
#define its_cmd_get_deviceid(cmd)	its_cmd_mask_field(cmd, 0, 32, 32)
#define its_cmd_get_size(cmd)		(its_cmd_mask_field(cmd, 1,  0,  5) + 1)
#define its_cmd_get_id(cmd)		its_cmd_mask_field(cmd, 1,  0, 32)
#define its_cmd_get_physical_id(cmd)	its_cmd_mask_field(cmd, 1, 32, 32)
#define its_cmd_get_collection(cmd)	its_cmd_mask_field(cmd, 2,  0, 16)
#define its_cmd_get_ittaddr(cmd)	(its_cmd_mask_field(cmd, 2,  8, 44) << 8)
#define its_cmd_get_target_addr(cmd)	its_cmd_mask_field(cmd, 2, 16, 32)
#define its_cmd_get_validbit(cmd)	its_cmd_mask_field(cmd, 2, 63,  1)

//TODO: IMPLEMENT
// static bool __is_visible_gfn_locked(struct vgic_its *its, paddr_t gpa)
// {
// 	gfn_t gfn = gpa >> PAGE_SHIFT;
// 	int idx;
// 	bool ret;

// 	idx = srcu_read_lock(&its->dev->kvm->srcu);
// 	ret = kvm_is_visible_gfn(its->dev->kvm, gfn);
// 	srcu_read_unlock(&its->dev->kvm->srcu, idx);
// 	return ret;
// }

/*
 * Check whether an ID can be stored into the corresponding guest table.
 * For a direct table this is pretty easy, but gets a bit nasty for
 * indirect tables. We check whether the resulting guest physical address
 * is actually valid (covered by a memslot and guest accessible).
 * For this we have to read the respective first level entry.
 */
static bool vgic_its_check_id(struct vgic_its *its, u64 baser, u32 id,
			      paddr_t *eaddr)
{
	int l1_tbl_size = GITS_BASER_NR_PAGES(baser) * SZ_64K;
	u64 indirect_ptr, type = GITS_BASER_TYPE(baser);
	paddr_t base = GITS_BASER_ADDR_48_to_52(baser);
	int esz = GITS_BASER_ENTRY_SIZE(baser);
	int index;

	switch (type) {
	case GITS_BASER_TYPE_DEVICE:
		if (id >= BIT(VITS_TYPER_DEVBITS, ULL))
			return false;
		break;
	case GITS_BASER_TYPE_COLLECTION:
		/* as GITS_TYPER.CIL == 0, ITS supports 16-bit collection ID */
		if (id >= BIT(16, ULL))
			return false;
		break;
	default:
		return false;
	}

	if (!(baser & GITS_BASER_INDIRECT)) {
		paddr_t addr;

		if (id >= (l1_tbl_size / esz))
			return false;

		addr = base + id * esz;

		if (eaddr)
			*eaddr = addr;

		return 1;//__is_visible_gfn_locked(its, addr);
	}

	/* calculate and check the index into the 1st level */
	index = id / (SZ_64K / esz);
	if (index >= (l1_tbl_size / sizeof(u64)))
		return false;

	/* Each 1st level entry is represented by a 64-bit value. */
	// if (kvm_read_guest_lock(its->dev->kvm,
	// 		   base + index * sizeof(indirect_ptr),
	// 		   &indirect_ptr, sizeof(indirect_ptr)))
		// return false;
	if (access_guest_memory_by_gpa(its->domain, base + index * sizeof(indirect_ptr),
			   &indirect_ptr, sizeof(indirect_ptr),0))
		return false;

	indirect_ptr = le64_to_cpu(indirect_ptr);

	/* check the valid bit of the first level entry */
	if (!(indirect_ptr & BIT(63, ULL)))
		return false;

	/* Mask the guest physical address and calculate the frame number. */
	indirect_ptr &= GENMASK_ULL(51, 16);

	/* Find the address of the actual entry */
	index = id % (SZ_64K / esz);
	indirect_ptr += index * esz;

	if (eaddr)
		*eaddr = indirect_ptr;

	return 1;//__is_visible_gfn_locked(its, indirect_ptr);
}

/*
 * MAPD maps or unmaps a device ID to Interrupt Translation Tables (ITTs).
 * Must be called with the its_lock mutex held.
 */

/*
 * Add a new collection into the ITS collection table.
 * Returns 0 on success, and a negative error value for generic errors.
 */
static int vgic_its_alloc_collection(struct vgic_its *its,
				     struct its_collection **colp,
				     u32 coll_id)
{
	struct its_collection *collection;

	collection = xzalloc(struct its_collection);
	if (!collection)
		return -ENOMEM;

	collection->collection_id = coll_id;
	collection->target_addr = COLLECTION_NOT_MAPPED;

	list_add_tail(&collection->coll_list, &its->collection_list);
	*colp = collection;

	return 0;
}

static void vgic_its_free_collection(struct vgic_its *its, u32 coll_id)
{
	struct its_collection *collection;
	struct its_device *device;
	struct its_ite *ite;

	/*
	 * Clearing the mapping for that collection ID removes the
	 * entry from the list. If there wasn't any before, we can
	 * go home early.
	 */
	collection = find_collection(its, coll_id);
	if (!collection)
		return;

	for_each_lpi_its(device, ite, its)
		if (ite->collection &&
		    ite->collection->collection_id == coll_id)
			ite->collection = NULL;

	list_del(&collection->coll_list);
	xfree(collection);
}

/* Requires the its_lock to be held. */
static void vgic_its_free_device(struct domain *d, struct its_device *device)
{
	struct its_ite *ite, *temp;

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
	struct its_device *cur, *temp;

	list_for_each_entry_safe(cur, temp, &its->device_list, dev_list)
		vgic_its_free_device(d, cur);
}

/* its lock must be held */
static void vgic_its_free_collection_list(struct domain *d, struct vgic_its *its)
{
	struct its_collection *cur, *temp;

	list_for_each_entry_safe(cur, temp, &its->collection_list, coll_list)
		vgic_its_free_collection(its, cur->collection_id);
}

/* Must be called with its_lock mutex held */
static struct its_device *vgic_its_alloc_device(struct vgic_its *its,
						u32 device_id, paddr_t itt_addr,
						u8 num_eventid_bits)
{
	struct its_device *device;

	device = xzalloc(struct its_device);
	if (!device)
		return ERR_PTR(-ENOMEM);

	device->device_id = device_id;
	device->itt_addr = itt_addr;
	device->num_eventid_bits = num_eventid_bits;
	INIT_LIST_HEAD(&device->itt_head);

	list_add_tail(&device->dev_list, &its->device_list);
	return device;
}

static int vgic_its_cmd_handle_mapd(struct domain *d, struct vgic_its *its,
				    u64 *its_cmd)
{
	u32 device_id = its_cmd_get_deviceid(its_cmd);
	bool valid = its_cmd_get_validbit(its_cmd);
	u8 num_eventid_bits = its_cmd_get_size(its_cmd);
	paddr_t itt_addr = its_cmd_get_ittaddr(its_cmd);
	int ret;
	struct its_device *device;

	if (!vgic_its_check_id(its, its->baser_device_table, device_id, NULL))
	 	return E_ITS_MAPD_DEVICE_OOR;

	if (valid && num_eventid_bits > VITS_TYPER_IDBITS)
		return E_ITS_MAPD_ITTSIZE_OOR;

	device = find_its_device(its, device_id);

	/*
	 * The spec says that calling MAPD on an already mapped device
	 * invalidates all cached data for this device. We implement this
	 * by removing the mapping and re-establishing it.
	 */
	if (device)
		vgic_its_free_device(d, device);

	/*
	 * The spec does not say whether unmapping a not-mapped device
	 * is an error, so we are done in any case.
	 */
	if (!valid)
		return 0;

	device = vgic_its_alloc_device(its, device_id, itt_addr,
				       num_eventid_bits);

    /*
     * There is no easy and clean way for Xen to know the ITS device ID of a
     * particular (PCI) device, so we have to rely on the guest telling
     * us about it. For *now* we are just using the device ID *Dom0* uses,
     * because the driver there has the actual knowledge.
     * Eventually this will be replaced with a dedicated hypercall to
     * announce pass-through of devices.
     */
    if ( is_hardware_domain(its->domain) )
    {

        /*
         * Dom0's ITSes are mapped 1:1, so both addresses are the same.
         * Also the device IDs are equal.
         */
        ret = gicv3_its_map_guest_device(its->domain, its->vgic_its_base + ITS_DOORBELL_OFFSET, device_id,
                                         its->vgic_its_base + ITS_DOORBELL_OFFSET, device_id,
                                         BIT(num_eventid_bits, UL), valid);
        if ( ret && valid )
            return ret;
    }

	return IS_ERR(device) ? PTR_ERR(device) : 0;
}

/*
 * The MAPC command maps collection IDs to redistributors.
 * Must be called with the its_lock mutex held.
 */
static int vgic_its_cmd_handle_mapc(struct domain *d, struct vgic_its *its,
				    u64 *its_cmd)
{
	u16 coll_id;
	u32 target_addr;
	struct its_collection *collection;
	bool valid;

	valid = its_cmd_get_validbit(its_cmd);
	coll_id = its_cmd_get_collection(its_cmd);
	target_addr = its_cmd_get_target_addr(its_cmd);

	//TODO: FIX CHECK
	// if (target_addr >= atomic_read(&d->online_vcpus))
	// 	return E_ITS_MAPC_PROCNUM_OOR;

	if (!valid) {
		vgic_its_free_collection(its, coll_id);
		vgic_its_invalidate_cache(d);
	} else {
		collection = find_collection(its, coll_id);

		if (!collection) {
			int ret;

			if (!vgic_its_check_id(its, its->baser_coll_table,
						coll_id, NULL))
				return E_ITS_MAPC_COLLECTION_OOR;

			ret = vgic_its_alloc_collection(its, &collection,
							coll_id);
			if (ret)
				return ret;
			collection->target_addr = target_addr;
		} else {
			collection->target_addr = target_addr;
			update_affinity_collection(d, its, collection);
		}
	}

	return 0;
}

/*
 * The MAPTI and MAPI commands map LPIs to ITTEs.
 * Must be called with its_lock mutex held.
 */
static int vgic_its_cmd_handle_mapi(struct domain *d, struct vgic_its *its,
				    u64 *its_cmd)
{
	u32 device_id = its_cmd_get_deviceid(its_cmd);
	u32 event_id = its_cmd_get_id(its_cmd);
	u32 coll_id = its_cmd_get_collection(its_cmd);
	struct its_ite *ite;
	struct vcpu *vcpu = NULL;
	struct its_device *device;
	struct its_collection *collection, *new_coll = NULL;
	struct vgic_irq *irq;
	int lpi_nr;

	device = find_its_device(its, device_id);
	if (!device)
		return E_ITS_MAPTI_UNMAPPED_DEVICE;

	//TODO FIX CHECK
	// if (!vgic_its_check_event_id(its, device, event_id))
	// 	return E_ITS_MAPTI_ID_OOR;

	if (its_cmd_get_command(its_cmd) == GITS_CMD_MAPTI)
		lpi_nr = its_cmd_get_physical_id(its_cmd);
	else
		lpi_nr = event_id;
	if (lpi_nr < GIC_LPI_OFFSET ||
	    lpi_nr >= max_lpis_propbaser(d->arch.vgic.propbaser))
		return E_ITS_MAPTI_PHYSICALID_OOR;

	/* If there is an existing mapping, behavior is UNPREDICTABLE. */
	if (find_ite(its, device_id, event_id))
		return 0;

	collection = find_collection(its, coll_id);
	if (!collection) {
		int ret;

		if (!vgic_its_check_id(its, its->baser_coll_table, coll_id, NULL))
			return E_ITS_MAPC_COLLECTION_OOR;

		ret = vgic_its_alloc_collection(its, &collection, coll_id);
		if (ret)
			return ret;
		new_coll = collection;
	}

	ite = vgic_its_alloc_ite(device, collection, event_id);
	if (IS_ERR(ite)) {
		if (new_coll)
			vgic_its_free_collection(its, coll_id);
		return PTR_ERR(ite);
	}

	if (its_is_collection_mapped(collection))
		vcpu = d->vcpu[collection->target_addr];

	irq = vgic_add_lpi(d, its, lpi_nr, device_id, event_id, vcpu);
	if (IS_ERR(irq)) {
		if (new_coll)
			vgic_its_free_collection(its, coll_id);
		its_free_ite(d, ite);
		return PTR_ERR(irq);
	}
	ite->irq = irq;

	return 0;
}

/*
 * The MOVI command moves an ITTE to a different collection.
 * Must be called with the its_lock mutex held.
 */
static int vgic_its_cmd_handle_movi(struct domain *d, struct vgic_its *its,
				    u64 *its_cmd)
{
	u32 device_id = its_cmd_get_deviceid(its_cmd);
	u32 event_id = its_cmd_get_id(its_cmd);
	u32 coll_id = its_cmd_get_collection(its_cmd);
	struct vcpu *vcpu;
	struct its_ite *ite;
	struct its_collection *collection;

	ite = find_ite(its, device_id, event_id);
	if (!ite)
		return E_ITS_MOVI_UNMAPPED_INTERRUPT;

	if (!its_is_collection_mapped(ite->collection))
		return E_ITS_MOVI_UNMAPPED_COLLECTION;

	collection = find_collection(its, coll_id);
	if (!its_is_collection_mapped(collection))
		return E_ITS_MOVI_UNMAPPED_COLLECTION;

	ite->collection = collection;
	vcpu = d->vcpu[collection->target_addr];

	vgic_its_invalidate_cache(d);

	return update_affinity(ite->irq, vcpu);
}

/*
 * The DISCARD command frees an Interrupt Translation Table Entry (ITTE).
 * Must be called with the its_lock mutex held.
 */
static int vgic_its_cmd_handle_discard(struct domain *d, struct vgic_its *its,
				       u64 *its_cmd)
{
	u32 device_id = its_cmd_get_deviceid(its_cmd);
	u32 event_id = its_cmd_get_id(its_cmd);
	struct its_ite *ite;

	ite = find_ite(its, device_id, event_id);
	if (ite && its_is_collection_mapped(ite->collection)) {
		/*
		 * Though the spec talks about removing the pending state, we
		 * don't bother here since we clear the ITTE anyway and the
		 * pending state is a property of the ITTE struct.
		 */
		vgic_its_invalidate_cache(d);

		its_free_ite(d, ite);
		return 0;
	}

	return E_ITS_DISCARD_UNMAPPED_INTERRUPT;
}

/*
 * The CLEAR command removes the pending state for a particular LPI.
 * Must be called with the its_lock mutex held.
 */
static int vgic_its_cmd_handle_clear(struct domain *d, struct vgic_its *its,
				     u64 *its_cmd)
{
	u32 device_id = its_cmd_get_deviceid(its_cmd);
	u32 event_id = its_cmd_get_id(its_cmd);
	struct its_ite *ite;


	ite = find_ite(its, device_id, event_id);
	if (!ite)
		return E_ITS_CLEAR_UNMAPPED_INTERRUPT;

	ite->irq->pending_latch = false;

	// if (ite->irq->hw)
	// 	return irq_set_irqchip_state(ite->irq->host_irq,
	// 				     IRQCHIP_STATE_PENDING, false);

	return 0;
}

/*
 * The MOVALL command moves the pending state of all IRQs targeting one
 * redistributor to another. We don't hold the pending state in the VCPUs,
 * but in the IRQs instead, so there is really not much to do for us here.
 * However the spec says that no IRQ must target the old redistributor
 * afterwards, so we make sure that no LPI is using the associated target_vcpu.
 * This command affects all LPIs in the system that target that redistributor.
 */
static int vgic_its_cmd_handle_movall(struct domain *d, struct vgic_its *its,
				      u64 *its_cmd)
{
	u32 target1_addr = its_cmd_get_target_addr(its_cmd);
	u32 target2_addr = its_cmd_mask_field(its_cmd, 3, 16, 32);
	struct vcpu *vcpu1, *vcpu2;
	struct vgic_irq *irq;
	u32 *intids;
	int irq_count, i;

	//TODO FIX CHECK
	// if (target1_addr >= atomic_read(&kvm->online_vcpus) ||
	//     target2_addr >= atomic_read(&kvm->online_vcpus))
	// 	return E_ITS_MOVALL_PROCNUM_OOR;

	if (target1_addr == target2_addr)
		return 0;

	vcpu1 = d->vcpu[target1_addr];
	vcpu2 = d->vcpu[target2_addr];

	irq_count = vgic_copy_lpi_list(d, vcpu1, &intids);
	if (irq_count < 0)
		return irq_count;

	for (i = 0; i < irq_count; i++) {
		irq = vgic_get_irq(d, NULL, intids[i]);

		update_affinity(irq, vcpu2);

		vgic_put_irq(d, irq);
	}

	vgic_its_invalidate_cache(d);

	xfree(intids);
	return 0;
}

/*
 * The INT command injects the LPI associated with that DevID/EvID pair.
 * Must be called with the its_lock mutex held.
 */
static int vgic_its_cmd_handle_int(struct domain *d, struct vgic_its *its,
				   u64 *its_cmd)
{
	u32 msi_data = its_cmd_get_id(its_cmd);
	u64 msi_devid = its_cmd_get_deviceid(its_cmd);

	return vgic_its_trigger_msi(d, its, msi_devid, msi_data);
}

int vgic_its_inv_lpi(struct domain *d, struct vgic_irq *irq)
{
	return update_lpi_config(d, irq, NULL, true);
}

/*
 * The INV command syncs the configuration bits from the memory table.
 * Must be called with the its_lock mutex held.
 */
static int vgic_its_cmd_handle_inv(struct domain *d, struct vgic_its *its,
				   u64 *its_cmd)
{
	u32 device_id = its_cmd_get_deviceid(its_cmd);
	u32 event_id = its_cmd_get_id(its_cmd);
	struct its_ite *ite;


	ite = find_ite(its, device_id, event_id);
	if (!ite)
		return E_ITS_INV_UNMAPPED_INTERRUPT;

	return vgic_its_inv_lpi(d, ite->irq);
}

/**
 * vgic_its_invall - invalidate all LPIs targetting a given vcpu
 * @vcpu: the vcpu for which the RD is targetted by an invalidation
 *
 * Contrary to the INVALL command, this targets a RD instead of a
 * collection, and we don't need to hold the its_lock, since no ITS is
 * involved here.
 */
int vgic_its_invall(struct vcpu *vcpu)
{
	struct domain *d = vcpu->domain;
	int irq_count, i = 0;
	u32 *intids;

	irq_count = vgic_copy_lpi_list(d, vcpu, &intids);
	if (irq_count < 0)
		return irq_count;

	for (i = 0; i < irq_count; i++) {
		struct vgic_irq *irq = vgic_get_irq(d, NULL, intids[i]);
		if (!irq)
			continue;
		update_lpi_config(d, irq, vcpu, false);
		vgic_put_irq(d, irq);
	}

	xfree(intids);

	// if (vcpu->arch.vgic.vgic_v3.its_vpe.its_vm)
	// 	its_invall_vpe(&vcpu->arch.vgic_cpu.vgic_v3.its_vpe);

	return 0;
}

/*
 * The INVALL command requests flushing of all IRQ data in this collection.
 * Find the VCPU mapped to that collection, then iterate over the VM's list
 * of mapped LPIs and update the configuration for each IRQ which targets
 * the specified vcpu. The configuration will be read from the in-memory
 * configuration table.
 * Must be called with the its_lock mutex held.
 */
static int vgic_its_cmd_handle_invall(struct domain *d, struct vgic_its *its,
				      u64 *its_cmd)
{
	u32 coll_id = its_cmd_get_collection(its_cmd);
	struct its_collection *collection;
	struct vcpu *vcpu;

	collection = find_collection(its, coll_id);
	if (!its_is_collection_mapped(collection))
		return E_ITS_INVALL_UNMAPPED_COLLECTION;

	vcpu = d->vcpu[collection->target_addr];
	vgic_its_invall(vcpu);

	return 0;
}

/*
 * This function is called with the its_cmd lock held, but the ITS data
 * structure lock dropped.
 */
static int vgic_its_handle_command(struct domain *d, struct vgic_its *its,
				   u64 *its_cmd)
{
	int ret = -ENODEV;

	spin_lock(&its->its_lock);
	switch (its_cmd_get_command(its_cmd)) {
	case GITS_CMD_MAPD:
		printk("Handling GITS_CMD_MAPD\n");
		ret = vgic_its_cmd_handle_mapd(d, its, its_cmd);
		break;
	case GITS_CMD_MAPC:
		printk("Handling GITS_CMD_MAPC\n");
		ret = vgic_its_cmd_handle_mapc(d, its, its_cmd);
		break;
	case GITS_CMD_MAPI:
		printk("Handling GITS_CMD_MAPI\n");
		ret = vgic_its_cmd_handle_mapi(d, its, its_cmd);
		break;
	case GITS_CMD_MAPTI:
		printk("Handling GITS_CMD_MAPTI\n");
		ret = vgic_its_cmd_handle_mapi(d, its, its_cmd);
		break;
	case GITS_CMD_MOVI:
		printk("Handling GITS_CMD_MOVI\n");
		ret = vgic_its_cmd_handle_movi(d, its, its_cmd);
		break;
	case GITS_CMD_DISCARD:
		printk("Handling GITS_CMD_DISCARD\n");
		ret = vgic_its_cmd_handle_discard(d, its, its_cmd);
		break;
	case GITS_CMD_CLEAR:
		printk("Handling GITS_CMD_CLEAR\n");
		ret = vgic_its_cmd_handle_clear(d, its, its_cmd);
		break;
	case GITS_CMD_MOVALL:
		printk("Handling GITS_CMD_MOVALL\n");
		ret = vgic_its_cmd_handle_movall(d, its, its_cmd);
		break;
	case GITS_CMD_INT:
		printk("Handling GITS_CMD_INT\n");
		ret = vgic_its_cmd_handle_int(d, its, its_cmd);
		break;
	case GITS_CMD_INV:
		printk("Handling GITS_CMD_INV\n");
		ret = vgic_its_cmd_handle_inv(d, its, its_cmd);
		break;
	case GITS_CMD_INVALL:
		printk("Handling GITS_CMD_INVALL\n");
		ret = vgic_its_cmd_handle_invall(d, its, its_cmd);
		break;
	case GITS_CMD_SYNC:
		/* we ignore this command: we are in sync all of the time */
		printk("Handling GITS_CMD_SYNC\n");
		ret = 0;
		break;
	default:
		printk("Unknown GITS command\n");
		ret = -EINVAL;
		break;
	}
	spin_unlock(&its->its_lock);

	return ret;
}

#define ITS_CMD_BUFFER_SIZE(baser)	((((baser) & 0xff) + 1) << 12)
#define ITS_CMD_SIZE			32
#define ITS_CMD_OFFSET(reg)		((reg) & GENMASK(19, 5))

/* Must be called with the cmd_lock held. */
static void vgic_its_process_commands(struct domain *d, struct vgic_its *its)
{
	paddr_t cbaser;
	u64 cmd_buf[4];

	/* Commands are only processed when the ITS is enabled. */
	if (!its->enabled)
		return;

	cbaser = GITS_CBASER_ADDRESS(its->cbaser);

	while (its->cwriter != its->creadr) {
		//int ret = kvm_read_guest_lock(kvm, cbaser + its->creadr,
					      //cmd_buf, ITS_CMD_SIZE);
        int ret = access_guest_memory_by_gpa(d, cbaser + its->creadr,
                                         cmd_buf, ITS_CMD_SIZE, false);
		/*
		 * If kvm_read_guest() fails, this could be due to the guest
		 * programming a bogus value in CBASER or something else going
		 * wrong from which we cannot easily recover.
		 * According to section 6.3.2 in the GICv3 spec we can just
		 * ignore that command then.
		 */
		if (!ret)
			vgic_its_handle_command(d, its, cmd_buf);

		its->creadr += ITS_CMD_SIZE;
		if (its->creadr == ITS_CMD_BUFFER_SIZE(its->cbaser))
			its->creadr = 0;
	}
}

static unsigned long vgic_mmio_read_its_ctlr(struct domain *d,
					     struct vgic_its *its,
					     paddr_t addr, unsigned int len)
{
	u32 reg = 0;

	spin_lock(&its->cmd_lock);
	if (its->creadr == its->cwriter)
		reg |= GITS_CTLR_QUIESCENT;
	if (its->enabled)
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
	if (!its->enabled && (val & GITS_CTLR_ENABLE) &&
		(!(its->baser_device_table & GITS_VALID_BIT) ||
		 !(its->baser_coll_table & GITS_VALID_BIT) ||
		 !(its->cbaser & GITS_VALID_BIT)))
		goto out;

	its->enabled = !!(val & GITS_CTLR_ENABLE);
	if (!its->enabled)
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
					     struct vgic_its *its,
					     paddr_t addr, unsigned int len)
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
	int ret = 0;
	u32 *intids;
	int nr_irqs, i;
	unsigned long flags;
	u8 pendmask;

	nr_irqs = vgic_copy_lpi_list(vcpu->domain, vcpu, &intids);
	if (nr_irqs < 0)
		return nr_irqs;

	for (i = 0; i < nr_irqs; i++) {
		int byte_offset, bit_nr;

		byte_offset = intids[i] / BITS_PER_BYTE;
		bit_nr = intids[i] % BITS_PER_BYTE;

		/*
		 * For contiguously allocated LPIs chances are we just read
		 * this very same byte in the last iteration. Reuse that.
		 */
		if (byte_offset != last_byte_offset) {
			// ret = kvm_read_guest_lock(vcpu->kvm,
			// 			  pendbase + byte_offset,
			// 			  &pendmask, 1);
			ret = access_guest_memory_by_gpa(vcpu->domain,
			                                 pendbase + byte_offset,
			                                 &pendmask, 1, false);
			if (ret) {
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
	const struct vgic_its_abi *abi = vgic_its_get_abi(its);
	u64 reg = GITS_TYPER_PHYSICAL;

	/*
	 * We use linear CPU numbers for redistributor addressing,
	 * so GITS_TYPER.PTA is 0.
	 * Also we force all PROPBASER registers to be the same, so
	 * CommonLPIAff is 0 as well.
	 * To avoid memory waste in the guest, we keep the number of IDBits and
	 * DevBits low - as least for the time being.
	 */
	reg |= GIC_ENCODE_SZ(VITS_TYPER_DEVBITS, 5) << GITS_TYPER_DEVIDS_SHIFT;
	reg |= GIC_ENCODE_SZ(VITS_TYPER_IDBITS, 5) << GITS_TYPER_IDBITS_SHIFT;
	reg |= GIC_ENCODE_SZ(abi->ite_esz, 4) << GITS_TYPER_ITT_SIZE_SHIFT;

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
	if (its->enabled)
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

	if (!its)
		return;

	spin_lock(&its->cmd_lock);

	reg = update_64bit_reg(its->cwriter, addr & 7, len, val);
	reg = ITS_CMD_OFFSET(reg);
	if (reg >= ITS_CMD_BUFFER_SIZE(its->cbaser)) {
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

	switch (BASER_INDEX(addr)) {
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

#define GITS_BASER_RO_MASK	(GENMASK_ULL(52, 48) | GENMASK_ULL(58, 56))
static void vgic_mmio_write_its_baser(struct domain *d,
				      struct vgic_its *its,
				      paddr_t addr, unsigned int len,
				      unsigned long val)
{
	const struct vgic_its_abi *abi = vgic_its_get_abi(its);
	u64 entry_size, table_type;
	u64 reg, *regptr, clearbits = 0;

	/* When GITS_CTLR.Enable is 1, we ignore write accesses. */
	if (its->enabled)
		return;

	switch (BASER_INDEX(addr)) {
	case 0:
		regptr = &its->baser_device_table;
		entry_size = abi->dte_esz;
		table_type = GITS_BASER_TYPE_DEVICE;
		break;
	case 1:
		regptr = &its->baser_coll_table;
		entry_size = abi->cte_esz;
		table_type = GITS_BASER_TYPE_COLLECTION;
		clearbits = GITS_BASER_INDIRECT;
		break;
	default:
		return;
	}

	reg = update_64bit_reg(*regptr, addr & 7, len, val);
	reg &= ~GITS_BASER_RO_MASK;
	reg &= ~clearbits;

	reg |= (entry_size - 1) << GITS_BASER_ENTRY_SIZE_SHIFT;
	reg |= table_type << GITS_BASER_TYPE_SHIFT;
	reg = vgic_sanitise_its_baser(reg);

	*regptr = reg;

	if (!(reg & GITS_BASER_VALID)) {
		/* Take the its_lock to prevent a race with a save/restore */
		spin_lock(&its->its_lock);
		switch (table_type) {
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
	switch (addr & 0xffff) {
	case GITS_PIDR0:
		return 0x92;	/* part number, bits[7:0] */
	case GITS_PIDR1:
		return 0xb4;	/* part number, bits[11:8] */
	case GITS_PIDR2:
		return GIC_PIDR2_ARCH_GICv3 | 0x0b;
	case GITS_PIDR4:
		return 0x40;	/* This is a 64K software visible page */
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

#define REGISTER_ITS_DESC(off, rd, wr, length, acc)		\
{								\
	.reg_offset = off,					\
	.len = length,						\
	.access_flags = acc,					\
	.its_read = rd,						\
	.its_write = wr,					\
}

static struct vgic_register_region its_registers[] = {
	REGISTER_ITS_DESC(GITS_CTLR,
		vgic_mmio_read_its_ctlr, vgic_mmio_write_its_ctlr, 4,
		VGIC_ACCESS_32bit),
	REGISTER_ITS_DESC(GITS_IIDR,
		vgic_mmio_read_its_iidr, its_mmio_write_wi,
		4, VGIC_ACCESS_32bit),
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
		vgic_mmio_read_its_creadr, its_mmio_write_wi,
		8, VGIC_ACCESS_64bit | VGIC_ACCESS_32bit),
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
	if (!(vcpu->arch.vgic.pendbaser & GICR_PENDBASER_PTZ))
		its_sync_lpi_pending_table(vcpu);
}

static int vgic_register_its_iodev(struct domain *d, struct vgic_its *its,
				   u64 addr)
{
	struct vgic_io_device *iodev = &its->iodev;
	int ret = 0;

	//spin_lock(&d->slots_lock);
	if (!IS_VGIC_ADDR_UNDEF(its->vgic_its_base)) {
		ret = -EBUSY;
		goto out;
	}

	its->vgic_its_base = addr;
	iodev->regions = its_registers;
	iodev->nr_regions = ARRAY_SIZE(its_registers);
	//kvm_iodevice_init(&iodev->dev, &kvm_io_gic_ops);

	iodev->base_fn = gaddr_to_gfn(its->vgic_its_base);
	iodev->iodev_type = IODEV_ITS;
	iodev->its = its;
	// ret = kvm_io_bus_register_dev(kvm, KVM_MMIO_BUS, iodev->base_addr,
	// 			      KVM_VGIC_V3_ITS_SIZE, &iodev->dev);
    register_mmio_handler(d, &vgic_io_ops, its->vgic_its_base, VGIC_V3_ITS_SIZE,
                          iodev);
out:
	//mutex_unlock(&kvm->slots_lock);

	return ret;
}

/* Default is 16 cached LPIs per vcpu */
#define LPI_DEFAULT_PCPU_CACHE_SIZE	16

void vgic_lpi_translation_cache_init(struct domain *d)
{
	struct vgic_dist *dist = &d->arch.vgic;
	unsigned int sz;
	int i;

	if (!list_empty(&dist->lpi_translation_cache))
		return;

	sz = d->max_vcpus * LPI_DEFAULT_PCPU_CACHE_SIZE;

	for (i = 0; i < sz; i++) {
		struct vgic_translation_cache_entry *cte;

		/* An allocation failure is not fatal */
		cte = xzalloc(struct vgic_translation_cache_entry);
		if (WARN_ON(!cte))
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

	list_for_each_entry_safe(cte, tmp,
				 &dist->lpi_translation_cache, entry) {
		list_del(&cte->entry);
		xfree(cte);
	}
}

#define INITIAL_BASER_VALUE						  \
	(GIC_BASER_CACHEABILITY(GITS_BASER, INNER, RaWb)		| \
	 GIC_BASER_CACHEABILITY(GITS_BASER, OUTER, SameAsInner)		| \
	 GIC_BASER_SHAREABILITY(GITS_BASER, InnerShareable)		| \
	 GITS_BASER_PAGE_SIZE_64K)

#define INITIAL_PROPBASER_VALUE						  \
	(GIC_BASER_CACHEABILITY(GICR_PROPBASER, INNER, RaWb)		| \
	 GIC_BASER_CACHEABILITY(GICR_PROPBASER, OUTER, SameAsInner)	| \
	 GIC_BASER_SHAREABILITY(GICR_PROPBASER, InnerShareable))

static int vgic_its_create(struct domain *d, u32 type, u64 addr)
{
	struct vgic_its *its;

	// if (type != DEV_TYPE_ARM_VGIC_ITS)
	// 	return -ENODEV;

	its = xzalloc(struct vgic_its);
	if (!its)
		return -ENOMEM;

	if (1 /*vgic_initialized(dev->kvm)*/) {
		// int ret = vgic_v4_init(dev->kvm);
		// if (ret < 0) {
		// 	kfree(its);
		// 	return ret;
		// }

		vgic_lpi_translation_cache_init(d);
	}

	spin_lock_init(&its->its_lock);
	spin_lock_init(&its->cmd_lock);

	its->vgic_its_base = VGIC_ADDR_UNDEF;

	INIT_LIST_HEAD(&its->device_list);
	INIT_LIST_HEAD(&its->collection_list);
    spin_lock_init(&d->arch.vgic.its_devices_lock);
    d->arch.vgic.its_devices = RB_ROOT;

	d->arch.vgic.msis_require_devid = true;
	d->arch.vgic.has_its = true;
	its->enabled = false;
	its->domain = d;

	its->baser_device_table = INITIAL_BASER_VALUE			|
		((u64)GITS_BASER_TYPE_DEVICE << GITS_BASER_TYPE_SHIFT);
	its->baser_coll_table = INITIAL_BASER_VALUE |
		((u64)GITS_BASER_TYPE_COLLECTION << GITS_BASER_TYPE_SHIFT);
	d->arch.vgic.propbaser = INITIAL_PROPBASER_VALUE;

	//dev->private = its;

	vgic_register_its_iodev(d, its, addr);

	return vgic_its_set_abi(its, NR_ITS_ABIS - 1);
}

// static void vgic_its_destroy(struct kvm_device *kvm_dev)
// {
// 	struct kvm *kvm = kvm_dev->kvm;
// 	struct vgic_its *its = kvm_dev->private;

// 	mutex_lock(&its->its_lock);

// 	vgic_its_free_device_list(kvm, its);
// 	vgic_its_free_collection_list(kvm, its);

// 	mutex_unlock(&its->its_lock);
// 	kfree(its);
// 	kfree(kvm_dev);/* alloc by kvm_ioctl_create_device, free by .destroy */
// }

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
            // ret = vgic_v3_its_init_virtual(d, hw_its->addr,
            //                                hw_its->devid_bits,
            //                                hw_its->evid_bits);
			ret = vgic_its_create(d, 0, hw_its->addr);
            if ( ret )
                return ret;
            else
                d->arch.vgic.has_its = true;
        }
    }

    return 0;
}

static u32 compute_next_devid_offset(struct list_head *h,
				     struct its_device *dev)
{
	struct its_device *next;
	u32 next_offset;

	if (list_is_last(&dev->dev_list, h))
		return 0;
	next = list_next_entry(dev, dev_list);
	next_offset = next->device_id - dev->device_id;

	return min_t(u32, next_offset, VITS_DTE_MAX_DEVID_OFFSET);
}

static u32 compute_next_eventid_offset(struct list_head *h, struct its_ite *ite)
{
	struct its_ite *next;
	u32 next_offset;

	if (list_is_last(&ite->ite_list, h))
		return 0;
	next = list_next_entry(ite, ite_list);
	next_offset = next->event_id - ite->event_id;

	return min_t(u32, next_offset, VITS_ITE_MAX_EVENTID_OFFSET);
}

/**
 * entry_fn_t - Callback called on a table entry restore path
 * @its: its handle
 * @id: id of the entry
 * @entry: pointer to the entry
 * @opaque: pointer to an opaque data
 *
 * Return: < 0 on error, 0 if last element was identified, id offset to next
 * element otherwise
 */
typedef int (*entry_fn_t)(struct vgic_its *its, u32 id, void *entry,
			  void *opaque);

/**
 * scan_its_table - Scan a contiguous table in guest RAM and applies a function
 * to each entry
 *
 * @its: its handle
 * @base: base gpa of the table
 * @size: size of the table in bytes
 * @esz: entry size in bytes
 * @start_id: the ID of the first entry in the table
 * (non zero for 2d level tables)
 * @fn: function to apply on each entry
 *
 * Return: < 0 on error, 0 if last element was identified, 1 otherwise
 * (the last element may not be found on second level tables)
 */
static int scan_its_table(struct vgic_its *its, paddr_t base, int size, u32 esz,
			  int start_id, entry_fn_t fn, void *opaque)
{
	struct domain *d = its->domain;
	unsigned long len = size;
	int id = start_id;
	paddr_t gpa = base;
	char entry[ESZ_MAX];
	int ret;

	memset(entry, 0, esz);

	while (len > 0) {
		int next_offset;
		size_t byte_offset;

		//ret = kvm_read_guest_lock(kvm, gpa, entry, esz);
		ret = access_guest_memory_by_gpa(d, gpa, entry, esz, 0);
		if (ret)
			return ret;

		next_offset = fn(its, id, entry, opaque);
		if (next_offset <= 0)
			return next_offset;

		byte_offset = next_offset * esz;
		id += next_offset;
		gpa += byte_offset;
		len -= byte_offset;
	}
	return 1;
}

/**
 * vgic_its_save_ite - Save an interrupt translation entry at @gpa
 */
static int vgic_its_save_ite(struct vgic_its *its, struct its_device *dev,
			      struct its_ite *ite, paddr_t gpa, int ite_esz)
{
	struct domain *d = its->domain;
	u32 next_offset;
	u64 val;

	next_offset = compute_next_eventid_offset(&dev->itt_head, ite);
	val = ((u64)next_offset << ITS_ITE_NEXT_SHIFT) |
	       ((u64)ite->irq->intid << ITS_ITE_PINTID_SHIFT) |
		ite->collection->collection_id;
	val = cpu_to_le64(val);

	//return kvm_write_guest_lock(kvm, gpa, &val, ite_esz);
	return access_guest_memory_by_gpa(d, gpa, &val, ite_esz, 1);
}

/**
 * vgic_its_restore_ite - restore an interrupt translation entry
 * @event_id: id used for indexing
 * @ptr: pointer to the ITE entry
 * @opaque: pointer to the its_device
 */
static int vgic_its_restore_ite(struct vgic_its *its, u32 event_id,
				void *ptr, void *opaque)
{
	struct its_device *dev = opaque;
	struct its_collection *collection;
	struct domain *d = its->domain;
	struct vcpu *vcpu = NULL;
	u64 val;
	u64 *p = (u64 *)ptr;
	struct vgic_irq *irq;
	u32 coll_id, lpi_id;
	struct its_ite *ite;
	u32 offset;

	val = *p;

	val = le64_to_cpu(val);

	coll_id = val & ITS_ITE_ICID_MASK;
	lpi_id = (val & ITS_ITE_PINTID_MASK) >> ITS_ITE_PINTID_SHIFT;

	if (!lpi_id)
		return 1; /* invalid entry, no choice but to scan next entry */

	if (lpi_id < VGIC_MIN_LPI)
		return -EINVAL;

	offset = val >> ITS_ITE_NEXT_SHIFT;
	if (event_id + offset >= BIT(dev->num_eventid_bits, ULL))
		return -EINVAL;

	collection = find_collection(its, coll_id);
	if (!collection)
		return -EINVAL;

	//TODO FIX
	// if (!vgic_its_check_event_id(its, dev, event_id))
	// 	return -EINVAL;

	ite = vgic_its_alloc_ite(dev, collection, event_id);
	if (IS_ERR(ite))
		return PTR_ERR(ite);

	if (its_is_collection_mapped(collection))
		vcpu = d->vcpu[collection->target_addr];

	irq = 0;//vgic_add_lpi(d, its, lpi_id, vcpu);
	if (IS_ERR(irq)) {
		its_free_ite(d, ite);
		return PTR_ERR(irq);
	}
	ite->irq = irq;

	return offset;
}

static int vgic_its_ite_cmp(void *priv, struct list_head *a,
			    struct list_head *b)
{
	struct its_ite *itea = container_of(a, struct its_ite, ite_list);
	struct its_ite *iteb = container_of(b, struct its_ite, ite_list);

	if (itea->event_id < iteb->event_id)
		return -1;
	else
		return 1;
}

static int vgic_its_save_itt(struct vgic_its *its, struct its_device *device)
{
	const struct vgic_its_abi *abi = vgic_its_get_abi(its);
	paddr_t base = device->itt_addr;
	struct its_ite *ite;
	int ret;
	int ite_esz = abi->ite_esz;

	list_sort(NULL, &device->itt_head, vgic_its_ite_cmp);

	list_for_each_entry(ite, &device->itt_head, ite_list) {
		paddr_t gpa = base + ite->event_id * ite_esz;

		/*
		 * If an LPI carries the HW bit, this means that this
		 * interrupt is controlled by GICv4, and we do not
		 * have direct access to that state without GICv4.1.
		 * Let's simply fail the save operation...
		 */

		// if (ite->irq->hw && !kvm_vgic_global_state.has_gicv4_1)
		// 	return -EACCES;

		ret = vgic_its_save_ite(its, device, ite, gpa, ite_esz);
		if (ret)
			return ret;
	}
	return 0;
}

/**
 * vgic_its_restore_itt - restore the ITT of a device
 *
 * @its: its handle
 * @dev: device handle
 *
 * Return 0 on success, < 0 on error
 */
static int vgic_its_restore_itt(struct vgic_its *its, struct its_device *dev)
{
	const struct vgic_its_abi *abi = vgic_its_get_abi(its);
	paddr_t base = dev->itt_addr;
	int ret;
	int ite_esz = abi->ite_esz;
	size_t max_size = BIT(dev->num_eventid_bits, ULL) * ite_esz;

	ret = scan_its_table(its, base, max_size, ite_esz, 0,
			     vgic_its_restore_ite, dev);

	/* scan_its_table returns +1 if all ITEs are invalid */
	if (ret > 0)
		ret = 0;

	return ret;
}

/**
 * vgic_its_save_dte - Save a device table entry at a given GPA
 *
 * @its: ITS handle
 * @dev: ITS device
 * @ptr: GPA
 */
static int vgic_its_save_dte(struct vgic_its *its, struct its_device *dev,
			     paddr_t ptr, int dte_esz)
{
	struct domain *d = its->domain;
	u64 val, itt_addr_field;
	u32 next_offset;

	itt_addr_field = dev->itt_addr >> 8;
	next_offset = compute_next_devid_offset(&its->device_list, dev);
	val = (1ULL << ITS_DTE_VALID_SHIFT |
	       ((u64)next_offset << ITS_DTE_NEXT_SHIFT) |
	       (itt_addr_field << ITS_DTE_ITTADDR_SHIFT) |
		(dev->num_eventid_bits - 1));
	val = cpu_to_le64(val);
	//return kvm_write_guest_lock(kvm, ptr, &val, dte_esz);
	return access_guest_memory_by_gpa(d, ptr, &val, dte_esz, 1);
}

/**
 * vgic_its_restore_dte - restore a device table entry
 *
 * @its: its handle
 * @id: device id the DTE corresponds to
 * @ptr: kernel VA where the 8 byte DTE is located
 * @opaque: unused
 *
 * Return: < 0 on error, 0 if the dte is the last one, id offset to the
 * next dte otherwise
 */
static int vgic_its_restore_dte(struct vgic_its *its, u32 id,
				void *ptr, void *opaque)
{
	struct its_device *dev;
	u64 baser = its->baser_device_table;
	paddr_t itt_addr;
	u8 num_eventid_bits;
	u64 entry = *(u64 *)ptr;
	bool valid;
	u32 offset;
	int ret;

	entry = le64_to_cpu(entry);

	valid = entry >> ITS_DTE_VALID_SHIFT;
	num_eventid_bits = (entry & ITS_DTE_SIZE_MASK) + 1;
	itt_addr = ((entry & ITS_DTE_ITTADDR_MASK)
			>> ITS_DTE_ITTADDR_SHIFT) << 8;

	if (!valid)
		return 1;

	/* dte entry is valid */
	offset = (entry & ITS_DTE_NEXT_MASK) >> ITS_DTE_NEXT_SHIFT;

	if (!vgic_its_check_id(its, baser, id, NULL))
		return -EINVAL;

	dev = vgic_its_alloc_device(its, id, itt_addr, num_eventid_bits);
	if (IS_ERR(dev))
		return PTR_ERR(dev);

	ret = vgic_its_restore_itt(its, dev);
	if (ret) {
		vgic_its_free_device(its->domain, dev);
		return ret;
	}

	return offset;
}

static int vgic_its_device_cmp(void *priv, struct list_head *a,
			       struct list_head *b)
{
	struct its_device *deva = container_of(a, struct its_device, dev_list);
	struct its_device *devb = container_of(b, struct its_device, dev_list);

	if (deva->device_id < devb->device_id)
		return -1;
	else
		return 1;
}

/**
 * handle_l1_dte - callback used for L1 device table entries (2 stage case)
 *
 * @its: its handle
 * @id: index of the entry in the L1 table
 * @addr: kernel VA
 * @opaque: unused
 *
 * L1 table entries are scanned by steps of 1 entry
 * Return < 0 if error, 0 if last dte was found when scanning the L2
 * table, +1 otherwise (meaning next L1 entry must be scanned)
 */
static int handle_l1_dte(struct vgic_its *its, u32 id, void *addr,
			 void *opaque)
{
	const struct vgic_its_abi *abi = vgic_its_get_abi(its);
	int l2_start_id = id * (SZ_64K / abi->dte_esz);
	u64 entry = *(u64 *)addr;
	int dte_esz = abi->dte_esz;
	paddr_t gpa;
	int ret;

	entry = le64_to_cpu(entry);

	if (!(entry & ITS_L1E_VALID_MASK))
		return 1;

	gpa = entry & ITS_L1E_ADDR_MASK;

	ret = scan_its_table(its, gpa, SZ_64K, dte_esz,
			     l2_start_id, vgic_its_restore_dte, NULL);

	return ret;
}

/**
 * vgic_its_save_device_tables - Save the device table and all ITT
 * into guest RAM
 *
 * L1/L2 handling is hidden by vgic_its_check_id() helper which directly
 * returns the GPA of the device entry
 */
static int vgic_its_save_device_tables(struct vgic_its *its)
{
	const struct vgic_its_abi *abi = vgic_its_get_abi(its);
	u64 baser = its->baser_device_table;
	struct its_device *dev;
	int dte_esz = abi->dte_esz;

	if (!(baser & GITS_BASER_VALID))
		return 0;

	list_sort(NULL, &its->device_list, vgic_its_device_cmp);

	list_for_each_entry(dev, &its->device_list, dev_list) {
		int ret;
		paddr_t eaddr;

		if (!vgic_its_check_id(its, baser,
				       dev->device_id, &eaddr))
			return -EINVAL;

		ret = vgic_its_save_itt(its, dev);
		if (ret)
			return ret;

		ret = vgic_its_save_dte(its, dev, eaddr, dte_esz);
		if (ret)
			return ret;
	}
	return 0;
}

/**
 * vgic_its_restore_device_tables - Restore the device table and all ITT
 * from guest RAM to internal data structs
 */
static int vgic_its_restore_device_tables(struct vgic_its *its)
{
	const struct vgic_its_abi *abi = vgic_its_get_abi(its);
	u64 baser = its->baser_device_table;
	int l1_esz, ret;
	int l1_tbl_size = GITS_BASER_NR_PAGES(baser) * SZ_64K;
	paddr_t l1_gpa;

	if (!(baser & GITS_BASER_VALID))
		return 0;

	l1_gpa = GITS_BASER_ADDR_48_to_52(baser);

	if (baser & GITS_BASER_INDIRECT) {
		l1_esz = GITS_LVL1_ENTRY_SIZE;
		ret = scan_its_table(its, l1_gpa, l1_tbl_size, l1_esz, 0,
				     handle_l1_dte, NULL);
	} else {
		l1_esz = abi->dte_esz;
		ret = scan_its_table(its, l1_gpa, l1_tbl_size, l1_esz, 0,
				     vgic_its_restore_dte, NULL);
	}

	/* scan_its_table returns +1 if all entries are invalid */
	if (ret > 0)
		ret = 0;

	if (ret < 0)
		vgic_its_free_device_list(its->domain, its);

	return ret;
}

static int vgic_its_save_cte(struct vgic_its *its,
			     struct its_collection *collection,
			     paddr_t gpa, int esz)
{
	u64 val;

	val = (1ULL << ITS_CTE_VALID_SHIFT |
	       ((u64)collection->target_addr << ITS_CTE_RDBASE_SHIFT) |
	       collection->collection_id);
	val = cpu_to_le64(val);
	//return kvm_write_guest_lock(its->dev->kvm, gpa, &val, esz);
	return access_guest_memory_by_gpa(its->domain, gpa, &val, esz, 1);
}

/*
 * Restore a collection entry into the ITS collection table.
 * Return +1 on success, 0 if the entry was invalid (which should be
 * interpreted as end-of-table), and a negative error value for generic errors.
 */
static int vgic_its_restore_cte(struct vgic_its *its, paddr_t gpa, int esz)
{
	struct its_collection *collection;
	struct domain *d = its->domain;
	u32 target_addr, coll_id;
	u64 val;
	int ret;

	BUG_ON(esz > sizeof(val));
	//ret = kvm_read_guest_lock(kvm, gpa, &val, esz);
	ret = access_guest_memory_by_gpa(d, gpa, &val, esz, 0);
	if (ret)
		return ret;
	val = le64_to_cpu(val);
	if (!(val & ITS_CTE_VALID_MASK))
		return 0;

	target_addr = (u32)(val >> ITS_CTE_RDBASE_SHIFT);
	coll_id = val & ITS_CTE_ICID_MASK;

	//TODO replace d->vcpu[x] with domain_vcpu(d, x)
	if (target_addr != COLLECTION_NOT_MAPPED &&
	    target_addr >= d->max_vcpus)
		return -EINVAL;

	collection = find_collection(its, coll_id);
	if (collection)
		return -EEXIST;

	if (!vgic_its_check_id(its, its->baser_coll_table, coll_id, NULL))
		return -EINVAL;

	ret = vgic_its_alloc_collection(its, &collection, coll_id);
	if (ret)
		return ret;
	collection->target_addr = target_addr;
	return 1;
}

/**
 * vgic_its_save_collection_table - Save the collection table into
 * guest RAM
 */
static int vgic_its_save_collection_table(struct vgic_its *its)
{
	const struct vgic_its_abi *abi = vgic_its_get_abi(its);
	u64 baser = its->baser_coll_table;
	paddr_t gpa = GITS_BASER_ADDR_48_to_52(baser);
	struct its_collection *collection;
	u64 val;
	size_t max_size, filled = 0;
	int ret, cte_esz = abi->cte_esz;

	if (!(baser & GITS_BASER_VALID))
		return 0;

	max_size = GITS_BASER_NR_PAGES(baser) * SZ_64K;

	list_for_each_entry(collection, &its->collection_list, coll_list) {
		ret = vgic_its_save_cte(its, collection, gpa, cte_esz);
		if (ret)
			return ret;
		gpa += cte_esz;
		filled += cte_esz;
	}

	if (filled == max_size)
		return 0;

	/*
	 * table is not fully filled, add a last dummy element
	 * with valid bit unset
	 */
	val = 0;
	BUG_ON(cte_esz > sizeof(val));
	//ret = kvm_write_guest_lock(its->dev->kvm, gpa, &val, cte_esz);
	ret = access_guest_memory_by_gpa(its->domain, gpa, &val, cte_esz, 1);
	return ret;
}

/**
 * vgic_its_restore_collection_table - reads the collection table
 * in guest memory and restores the ITS internal state. Requires the
 * BASER registers to be restored before.
 */
static int vgic_its_restore_collection_table(struct vgic_its *its)
{
	const struct vgic_its_abi *abi = vgic_its_get_abi(its);
	u64 baser = its->baser_coll_table;
	int cte_esz = abi->cte_esz;
	size_t max_size, read = 0;
	paddr_t gpa;
	int ret;

	if (!(baser & GITS_BASER_VALID))
		return 0;

	gpa = GITS_BASER_ADDR_48_to_52(baser);

	max_size = GITS_BASER_NR_PAGES(baser) * SZ_64K;

	while (read < max_size) {
		ret = vgic_its_restore_cte(its, gpa, cte_esz);
		if (ret <= 0)
			break;
		gpa += cte_esz;
		read += cte_esz;
	}

	if (ret > 0)
		return 0;

	if (ret < 0)
		vgic_its_free_collection_list(its->domain, its);

	return ret;
}


/**
 * vgic_its_save_tables_v0 - Save the ITS tables into guest ARM
 * according to v0 ABI
 */
static int vgic_its_save_tables_v0(struct vgic_its *its)
{
	int ret;

	ret = vgic_its_save_device_tables(its);
	if (ret)
		return ret;

	return vgic_its_save_collection_table(its);
}

/**
 * vgic_its_restore_tables_v0 - Restore the ITS tables from guest RAM
 * to internal data structs according to V0 ABI
 *
 */
static int vgic_its_restore_tables_v0(struct vgic_its *its)
{
	int ret;

	ret = vgic_its_restore_collection_table(its);
	if (ret)
		return ret;

	ret = vgic_its_restore_device_tables(its);
	if (ret)
		vgic_its_free_collection_list(its->domain, its);
	return ret;
}

static int vgic_its_commit_v0(struct vgic_its *its)
{
	const struct vgic_its_abi *abi;

	abi = vgic_its_get_abi(its);
	its->baser_coll_table &= ~GITS_BASER_ENTRY_SIZE_MASK;
	its->baser_device_table &= ~GITS_BASER_ENTRY_SIZE_MASK;

	its->baser_coll_table |= (GIC_ENCODE_SZ(abi->cte_esz, 5)
					<< GITS_BASER_ENTRY_SIZE_SHIFT);

	its->baser_device_table |= (GIC_ENCODE_SZ(abi->dte_esz, 5)
					<< GITS_BASER_ENTRY_SIZE_SHIFT);
	return 0;
}
