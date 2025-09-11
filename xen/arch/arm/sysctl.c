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
#include <xsm/xsm.h>
#include <asm/arm64/sve.h>
#include <public/sysctl.h>

void arch_do_physinfo(struct xen_sysctl_physinfo *pi)
{
    pi->capabilities |= XEN_SYSCTL_PHYSCAP_hvm | XEN_SYSCTL_PHYSCAP_hap;

    pi->arch_capabilities |= MASK_INSR(sve_encode_vl(get_sys_vl_len()),
                                       XEN_SYSCTL_PHYSCAP_ARM_SVE_MASK);
}

static long cpu_hotplug_sysctl(struct xen_sysctl_cpu_hotplug *hotplug)
{
#ifdef CONFIG_RUNTIME_CPU_CONTROL
    int ret;

    switch ( hotplug->op )
    {
    case XEN_SYSCTL_CPU_HOTPLUG_ONLINE:
        ret = xsm_resource_plug_core(XSM_HOOK);
        if ( ret )
            return ret;
        return continue_hypercall_on_cpu(0, cpu_up_helper, _p(hotplug->cpu));

    case XEN_SYSCTL_CPU_HOTPLUG_OFFLINE:
        ret = xsm_resource_unplug_core(XSM_HOOK);
        if ( ret )
            return ret;
        return continue_hypercall_on_cpu(0, cpu_down_helper, _p(hotplug->cpu));

    default:
        return -EOPNOTSUPP;
    }
#else
    return -EOPNOTSUPP;
#endif
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
