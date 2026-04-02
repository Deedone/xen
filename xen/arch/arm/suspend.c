/* SPDX-License-Identifier: GPL-2.0-only */

#include <asm/cpuerrata.h>
#include <asm/cpufeature.h>
#include <asm/gic.h>
#include <asm/psci.h>
#include <asm/suspend.h>

#include <xen/console.h>
#include <xen/cpu.h>
#include <xen/iommu.h>
#include <xen/lib.h>
#include <xen/sched.h>
#include <xen/serial.h>
#include <xen/tasklet.h>

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

/* Xen suspend. data identifies the domain that initiated suspend. */
static void system_suspend(void *data)
{
    int status;
    unsigned long flags;
    struct domain *d = (struct domain *)data;

    BUG_ON(system_state != SYS_STATE_active);

    system_state = SYS_STATE_suspend;

    printk("Xen suspending...\n");

    freeze_domains();
    scheduler_disable();

    /*
     * Non-boot CPUs have to be disabled on suspend and enabled on resume
     * (hotplug-based mechanism). Disabling non-boot CPUs will lead to PSCI
     * CPU_OFF to be called by each non-boot CPU. Depending on the underlying
     * platform capabilities, this may lead to the physical powering down of
     * CPUs.
     */
    status = disable_nonboot_cpus();
    if ( status )
    {
        system_state = SYS_STATE_resume;
        goto resume_nonboot_cpus;
    }

    console_start_sync();
    status = iommu_suspend();
    if ( status )
    {
        system_state = SYS_STATE_resume;
        goto resume_end_sync;
    }

    status = console_suspend();
    if ( status )
    {
        dprintk(XENLOG_ERR, "Failed to suspend the console, err=%d\n", status);
        system_state = SYS_STATE_resume;
        goto resume_iommu;
    }

    local_irq_save(flags);

    time_suspend();

    status = gic_suspend();
    if ( status )
    {
        system_state = SYS_STATE_resume;
        goto resume_time;
    }

    set_init_ttbr(xen_pgtable);

    /*
     * Enable identity mapping before entering suspend to simplify
     * the resume path
     */
    update_boot_mapping(true);

    if ( prepare_resume_ctx() )
    {
        status = call_psci_system_suspend();
        /*
         * If suspend is finalized properly by above system suspend PSCI call,
         * the code below in this 'if' branch will never execute. Execution
         * will continue from hyp_resume which is the hypervisor's resume point.
         * In hyp_resume CPU context will be restored and since link-register is
         * restored as well, it will appear to return from prepare_resume_ctx.
         * The difference in returning from prepare_resume_ctx on system suspend
         * versus resume is in function's return value: on suspend, the return
         * value is a non-zero value, on resume it is zero. That is why the
         * control flow will not re-enter this 'if' branch on resume.
         */
        if ( status )
            dprintk(XENLOG_WARNING, "PSCI system suspend failed, err=%d\n",
                    status);

        system_state = SYS_STATE_resume;
    }
    else
    {
        system_state = SYS_STATE_resume;

        /*
         * CPU0 resumes directly from hyp_resume(), bypassing the CPU hotplug
         * path that re-checks and re-enables errata workarounds for secondary
         * CPUs.
         */
        check_local_cpu_errata();
        check_local_cpu_features();
        BUG_ON(enable_local_cpu_errata_workarounds());
    }

    update_boot_mapping(false);

    gic_resume();

 resume_time:
    time_resume();

    local_irq_restore(flags);

    console_resume();

 resume_iommu:
    iommu_resume();

 resume_end_sync:
    console_end_sync();

 resume_nonboot_cpus:
    /*
     * The rcu_barrier() has to be added to ensure that the per cpu area is
     * freed before a non-boot CPU tries to initialize it (_free_percpu_area()
     * has to be called before the init_percpu_area()). This scenario occurs
     * when non-boot CPUs are hot-unplugged on suspend and hotplugged on resume.
     */
    rcu_barrier();
    enable_nonboot_cpus();

    scheduler_enable();
    thaw_domains();

    system_state = SYS_STATE_active;

    printk("Resume (status %d)\n", status);

    domain_resume(d);
}

static DECLARE_TASKLET(system_suspend_tasklet, system_suspend, NULL);

void host_system_suspend(struct domain *d)
{
    system_suspend_tasklet.data = (void *)d;
    /*
     * The suspend procedure has to be finalized by the pCPU#0 (non-boot pCPUs
     * will be disabled during the suspend).
     */
    tasklet_schedule_on_cpu(&system_suspend_tasklet, 0);
}

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
