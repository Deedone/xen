/*
 * xen/arch/arm/gic-v4-its.c
 *
 * ARM Generic Interrupt Controller support v4 version
 * based on xen/arch/arm/gic-v3-its.c and kernel GICv4 driver
 *
 * Copyright (C) 2023 - ARM Ltd
 * Penny Zheng <penny.zheng@arm.com>, ARM Ltd ported to Xen
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <xen/errno.h>
#include <xen/sched.h>
#include <xen/spinlock.h>
#include <asm/gic_v3_defs.h>
#include <asm/gic_v3_its.h>
#include <asm/gic_v4_its.h>
#include <asm/vgic.h>
static int its_send_cmd_vsync(struct host_its *its, uint16_t vpeid)
{
    uint64_t cmd[4];

    cmd[0] = GITS_CMD_VSYNC;
    cmd[1] = (uint64_t)vpeid << 32;
    cmd[2] = 0x00;
    cmd[3] = 0x00;

    return its_send_command(its, cmd);
}

static int its_send_cmd_vmapti(struct host_its *its, struct its_device *dev,
                               uint32_t eventid)
{
    uint64_t cmd[4];
    uint32_t deviceid = dev->host_devid;
    struct its_vlpi_map *map = &dev->event_map.vlpi_maps[eventid];
    uint16_t vpeid = map->vm->vpes[map->vpe_idx]->vpe_id;
    uint32_t vintid = map->vintid;
    uint32_t db_pintid;

    if ( map->db_enabled )
        db_pintid = map->vm->vpes[map->vpe_idx]->vpe_db_lpi;
    else
        db_pintid = 1023;

    cmd[0] = GITS_CMD_VMAPTI | ((uint64_t)deviceid << 32);
    cmd[1] = eventid | ((uint64_t)vpeid << 32);
    cmd[2] = vintid | ((uint64_t)db_pintid << 32);
    cmd[3] = 0x00;

    return its_send_command(its, cmd);
}

static bool pirq_is_forwarded_to_vcpu(struct pending_irq *pirq)
{
    ASSERT(pirq);
    return test_bit(GIC_IRQ_GUEST_FORWARDED, &pirq->status);
}

bool event_is_forwarded_to_vcpu(struct its_device *dev, uint32_t eventid)
{
    struct pending_irq *pirq;

    /* No vlpi maps at all ? */
    if ( !dev->event_map.vlpi_maps)
        return false;

    pirq = dev->event_map.vlpi_maps[eventid].pirq;
    return pirq_is_forwarded_to_vcpu(pirq);
}

static int its_send_cmd_vmovi(struct host_its *its, struct its_vlpi_map *map)
{
    uint64_t cmd[4];
    struct its_device *dev = map->dev;
    uint32_t eventid = map->eventid;
    uint32_t deviceid = dev->host_devid;
    uint16_t vpeid = map->vm->vpes[map->vpe_idx]->vpe_id;
    uint32_t db_pintid;

    if ( map->db_enabled )
        db_pintid = map->vm->vpes[map->vpe_idx]->vpe_db_lpi;
    else
        db_pintid = 1023;

    cmd[0] = GITS_CMD_VMOVI | ((uint64_t)deviceid << 32);
    cmd[1] = eventid | ((uint64_t)vpeid << 32);
    cmd[2] = (map->db_enabled ? 1UL : 0UL) | ((uint64_t)db_pintid << 32);
    cmd[3] = 0x00;

    return its_send_command(its, cmd);
}

static int gicv4_its_vlpi_map(struct its_vlpi_map *map)
{
    struct its_device *dev;
    struct host_its *its;
    uint32_t eventid;
    int ret;

    if ( !map )
        return -EINVAL;
    dev = map->dev;
    its = map->dev->hw_its;
    eventid = map->eventid;

    spin_lock(&dev->event_map.vlpi_lock);

    if ( !dev->event_map.vm )
    {
        struct its_vlpi_map *maps;

        maps = xzalloc_array(struct its_vlpi_map, dev->event_map.nr_lpis);
        if ( !maps )
        {
            ret = -ENOMEM;
            goto out;
        }

        dev->event_map.vm = map->vm;
        dev->event_map.vlpi_maps = maps;
    }
    else if ( dev->event_map.vm != map->vm )
    {
        ret = -EINVAL;
        goto out;
    }

    /* Get our private copy of the mapping information */
    dev->event_map.vlpi_maps[eventid] = *map;

    if ( pirq_is_forwarded_to_vcpu(map->pirq) )
    {
        struct its_vlpi_map *old = &dev->event_map.vlpi_maps[eventid];
        uint32_t old_vpeid = old->vm->vpes[old->vpe_idx]->vpe_id;

        /* Already mapped, move it around */
        ret = its_send_cmd_vmovi(dev->hw_its, map);
        if ( ret )
            goto out;

        /*
         * ARM spec says that If, after using VMOVI to move an interrupt from
         * vPE A to vPE B, software moves the same interrupt again, a VSYNC
         * command must be issued to vPE A between the moves to ensure correct
         * behavior.
         * So each time we issue VMOVI, we VSYNC the old VPE for good measure.
         */
        ret = its_send_cmd_vsync(dev->hw_its, old_vpeid);
    }
    else
    {
        /* Drop the original physical mapping firstly */
        ret = its_send_cmd_discard(its, dev, eventid);
        if ( ret )
            goto out;

        /* Then install the virtual one */
        ret = its_send_cmd_vmapti(its, dev, eventid);
        if ( ret )
            goto out;

        /* Increment the number of VLPIs */
        dev->event_map.nr_vlpis++;
    }

 out:
    spin_unlock(&dev->event_map.vlpi_lock);
    return ret;
}
int gicv4_its_vlpi_unmap(struct pending_irq *pirq)
{
    struct its_vlpi_map *map = pirq->vlpi_map;
    struct its_device *dev = map->dev;
    int ret;
    uint32_t host_lpi;

    spin_lock(&dev->event_map.vlpi_lock);

    if ( !dev->event_map.vm || !pirq_is_tied_to_hw(pirq) )
    {
        ret = -EINVAL;
        goto out;
    }

    /* Drop the virtual mapping */
    ret = its_send_cmd_discard(dev->hw_its, dev, map->eventid);
    if ( ret )
        goto out;

    /* Restore the physical one */
    clear_bit(GIC_IRQ_GUEST_FORWARDED, &pirq->status);
    host_lpi = dev->host_lpi_blocks[map->eventid / LPI_BLOCK] +
               (map->eventid % LPI_BLOCK);
    /* Map every host LPI to host CPU 0 */
    ret = its_send_cmd_mapti(dev->hw_its, dev->host_devid, map->eventid,
                             host_lpi, 0);
    if ( ret )
        goto out;

    lpi_write_config(lpi_data.lpi_property, host_lpi, 0xff, LPI_PROP_ENABLED);

    ret = its_inv_lpi(dev->hw_its, dev, map->eventid, 0);
    if ( ret )
        goto out;

    xfree(map);
    /*
     * Drop the refcount and make the device available again if
     * this was the last VLPI.
     */
    if ( !--dev->event_map.nr_vlpis )
    {
        dev->event_map.vm = NULL;
        xfree(dev->event_map.vlpi_maps);
    }

out:
    spin_unlock(&dev->event_map.vlpi_lock);
    return ret;
}

int gicv4_assign_guest_event(struct domain *d, paddr_t vdoorbell_address,
                             uint32_t vdevid, uint32_t eventid,
                             struct pending_irq *pirq)

{
    int ret = ENODEV;
    struct its_vm *vm = d->arch.vgic.its_vm;
    struct its_vlpi_map *map;
    struct its_device *dev;

    spin_lock(&d->arch.vgic.its_devices_lock);
    dev = get_its_device(d, vdoorbell_address, vdevid);
    if ( dev && eventid < dev->eventids )
    {
        /* Prepare the vlpi mapping info */
        map = xzalloc(struct its_vlpi_map);
        if ( !map )
            goto out;
        map->vm = vm;
        map->vintid = pirq->irq;
        map->db_enabled = true;
        map->vpe_idx = pirq->lpi_vcpu_id;
        map->properties = pirq->lpi_priority |
                          (test_bit(GIC_IRQ_GUEST_ENABLED, &pirq->status) ?
                          LPI_PROP_ENABLED : 0);
        map->pirq = pirq;
        map->dev = dev;
        map->eventid = eventid;

        ret = gicv4_its_vlpi_map(map);
        if ( ret )
        {
            xfree(map);
            goto out;
        }

        pirq->vlpi_map = map;
        goto out;
    }

 out:
    spin_unlock(&d->arch.vgic.its_devices_lock);
    return ret;
}

int gicv4_its_vlpi_move(struct pending_irq *pirq, struct vcpu *vcpu)
{
    struct its_vlpi_map *map = pirq->vlpi_map;
    struct its_device *dev = map->dev;

    if ( !dev->event_map.vm || !map )
        return -EINVAL;

    map->vpe_idx = vcpu->vcpu_id;
    return gicv4_its_vlpi_map(map);
}

