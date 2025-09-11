/* SPDX-License-Identifier: GPL-2.0-only */
/******************************************************************************
 * Arch-specific sysctl.c
 *
 * System management operations. For use by node control stack.
 *
 * Copyright (c) 2012, Citrix Systems
 */

#include <xen/types.h>
#include <xen/lib.h>
#include <xen/dt-overlay.h>
#include <xen/errno.h>
#include <xen/hypercall.h>
#include <xen/cpu.h>
#include <asm/arm64/sve.h>
#include <public/sysctl.h>

void arch_do_physinfo(struct xen_sysctl_physinfo *pi)
{
    pi->capabilities |= XEN_SYSCTL_PHYSCAP_hvm | XEN_SYSCTL_PHYSCAP_hap;

    pi->arch_capabilities |= MASK_INSR(sve_encode_vl(get_sys_vl_len()),
                                       XEN_SYSCTL_PHYSCAP_ARM_SVE_MASK);
}

static long cpu_up_helper(void *data)
{
    unsigned long cpu = (unsigned long) data;
    return cpu_up(cpu);
}

static long cpu_down_helper(void *data)
{
    unsigned long cpu = (unsigned long) data;
    return cpu_down(cpu);
}

static long smt_up_down_helper(void *data)
{
    bool up = (bool) data;
    unsigned int cpu;
    int ret;

    for_each_present_cpu ( cpu )
    {
        if ( cpu == 0)
            continue;

        if (up)
            ret = cpu_up(cpu);
        else
            ret = cpu_down(cpu);

        if ( ret )
            return ret;
    }

    return 0;
}

static long cpu_hotplug_sysctl(struct xen_sysctl_cpu_hotplug *hotplug)
{
    bool up;

    switch (hotplug->op) {
        case XEN_SYSCTL_CPU_HOTPLUG_ONLINE:
            if ( hotplug->cpu == 0)
                return -EINVAL;
            return continue_hypercall_on_cpu(0, cpu_up_helper, _p(hotplug->cpu));

        case XEN_SYSCTL_CPU_HOTPLUG_OFFLINE:
            if ( hotplug->cpu == 0)
                return -EINVAL;
            return continue_hypercall_on_cpu(0, cpu_down_helper, _p(hotplug->cpu));

        case XEN_SYSCTL_CPU_HOTPLUG_SMT_ENABLE:
        case XEN_SYSCTL_CPU_HOTPLUG_SMT_DISABLE:
            if ( CONFIG_NR_CPUS <= 1 )
                return 0;
            up = hotplug->op == XEN_SYSCTL_CPU_HOTPLUG_SMT_ENABLE;
            return continue_hypercall_on_cpu(0, smt_up_down_helper, _p(up));

        default:
            return -EINVAL;
    }
}

long arch_do_sysctl(struct xen_sysctl *sysctl,
                    XEN_GUEST_HANDLE_PARAM(xen_sysctl_t) u_sysctl)
{
    long ret;

    switch ( sysctl->cmd )
    {
    case XEN_SYSCTL_dt_overlay:
        ret = dt_overlay_sysctl(&sysctl->u.dt_overlay);
        break;

    case XEN_SYSCTL_cpu_hotplug:
        ret = cpu_hotplug_sysctl(&sysctl->u.cpu_hotplug);
        break;

    default:
        ret = -ENOSYS;
        break;
    }

    return ret;
}

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
