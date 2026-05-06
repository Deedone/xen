/* SPDX-License-Identifier: GPL-2.0-only */

#include <asm/psci.h>
#include <asm/suspend.h>

#include <xen/lib.h>
#include <xen/serial.h>

struct resume_cpu_context resume_cpu_context;

/*
 * Non-PSCI infrastructure can make host suspend impossible even when the PSCI
 * SYSTEM_SUSPEND conduit is present, e.g. when a Xen-owned driver has no valid
 * suspend/resume path.
 *
 * This gate is checked only when the last awake control domain attempts to
 * turn a guest SYSTEM_SUSPEND request into a host-suspend request.
 */
static bool __ro_after_init host_system_suspend_runtime_allowed = true;

static bool host_serial_suspend_allowed(void)
{
    if ( serial_suspend_supported() )
        return true;

    printk_once(XENLOG_INFO
                "Host SYSTEM_SUSPEND blocked: serial unsupported\n");

    return false;
}

bool host_system_suspend_allowed(void)
{
    return psci_system_suspend_allowed() &&
           host_serial_suspend_allowed() &&
           host_system_suspend_runtime_allowed;
}

void host_system_suspend_disable(const char *reason)
{
    host_system_suspend_runtime_allowed = false;

    printk(XENLOG_INFO "Host SYSTEM_SUSPEND blocked: %s\n",
           reason ? reason : "unsupported suspend/resume path");
}

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
