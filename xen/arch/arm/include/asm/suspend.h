/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef ARM_SUSPEND_H
#define ARM_SUSPEND_H

#include <xen/types.h>

struct domain;
struct vcpu;
struct vcpu_guest_context;

struct resume_info {
    struct vcpu_guest_context *ctxt;
    struct vcpu *wake_cpu;
};

void arch_domain_resume(struct domain *d);

#ifdef CONFIG_SYSTEM_SUSPEND
#ifdef CONFIG_ARM_64
struct resume_cpu_context {
    register_t callee_regs[12];
    register_t sp;
    register_t vbar_el2;
    register_t vtcr_el2;
    register_t vttbr_el2;
    register_t tpidr_el2;
    register_t mdcr_el2;
    register_t hstr_el2;
    register_t cptr_el2;
    register_t hcr_el2;
    register_t cnthctl_el2;
} __aligned(16);
#else
#error "Define resume_cpu_context structure for arm32"
#endif

extern struct resume_cpu_context resume_cpu_context;

int prepare_resume_ctx(void);
void hyp_resume(void);
bool host_system_suspend_allowed(void);
void host_system_suspend_disable(const char *reason);

#else /* !CONFIG_SYSTEM_SUSPEND */

static inline bool host_system_suspend_allowed(void) { return false; }
static inline void host_system_suspend_disable(const char *reason) {}

#endif

#endif /* ARM_SUSPEND_H */

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
