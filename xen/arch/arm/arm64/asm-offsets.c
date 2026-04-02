/*
 * Generate definitions needed by assembly language modules.
 * This code generates raw asm output which is post-processed
 * to extract and format the required data.
 */
#define COMPILE_OFFSETS

#include <xen/types.h>
#include <xen/sched.h>
#include <xen/bitops.h>
#include <public/xen.h>
#include <asm/current.h>
#include <asm/mm.h>
#include <asm/setup.h>
#include <asm/smccc.h>
#include <asm/suspend.h>

#define DEFINE(_sym, _val)                                                 \
    asm volatile ( "\n.ascii\"==>#define " #_sym " %0 /* " #_val " */<==\""\
                   :: "i" (_val) )
#define BLANK()                                                            \
    asm volatile ( "\n.ascii\"==><==\"" )
#define OFFSET(_sym, _str, _mem)                                           \
    DEFINE(_sym, offsetof(_str, _mem))

void __dummy__(void)
{
   OFFSET(UREGS_X0, struct cpu_user_regs, x0);
   OFFSET(UREGS_X1, struct cpu_user_regs, x1);
   OFFSET(UREGS_LR, struct cpu_user_regs, lr);

   OFFSET(UREGS_SP, struct cpu_user_regs, sp);
   OFFSET(UREGS_PC, struct cpu_user_regs, pc);
   OFFSET(UREGS_CPSR, struct cpu_user_regs, cpsr);
   OFFSET(UREGS_ESR_el2, struct cpu_user_regs, hsr);

   OFFSET(UREGS_SPSR_el1, struct cpu_user_regs, spsr_el1);

   OFFSET(UREGS_SPSR_fiq, struct cpu_user_regs, spsr_fiq);
   OFFSET(UREGS_SPSR_irq, struct cpu_user_regs, spsr_irq);
   OFFSET(UREGS_SPSR_und, struct cpu_user_regs, spsr_und);
   OFFSET(UREGS_SPSR_abt, struct cpu_user_regs, spsr_abt);

   OFFSET(UREGS_SP_el0, struct cpu_user_regs, sp_el0);
   OFFSET(UREGS_SP_el1, struct cpu_user_regs, sp_el1);
   OFFSET(UREGS_ELR_el1, struct cpu_user_regs, elr_el1);

   OFFSET(UREGS_kernel_sizeof, struct cpu_user_regs, spsr_el1);
   BLANK();

   DEFINE(CPUINFO_sizeof, sizeof(struct cpu_info));
   OFFSET(CPUINFO_flags, struct cpu_info, flags);
   BLANK();

   OFFSET(VCPU_arch_saved_context, struct vcpu, arch.saved_context);
   BLANK();

   OFFSET(INITINFO_stack, struct init_info, stack);
   BLANK();

#ifdef CONFIG_SYSTEM_SUSPEND
   OFFSET(RESUME_CTX_X19, struct resume_cpu_context, callee_regs[0]);
   OFFSET(RESUME_CTX_X21, struct resume_cpu_context, callee_regs[2]);
   OFFSET(RESUME_CTX_X23, struct resume_cpu_context, callee_regs[4]);
   OFFSET(RESUME_CTX_X25, struct resume_cpu_context, callee_regs[6]);
   OFFSET(RESUME_CTX_X27, struct resume_cpu_context, callee_regs[8]);
   OFFSET(RESUME_CTX_X29, struct resume_cpu_context, callee_regs[10]);
   OFFSET(RESUME_CTX_SP, struct resume_cpu_context, sp);
   OFFSET(RESUME_CTX_VBAR_EL2, struct resume_cpu_context, vbar_el2);
   OFFSET(RESUME_CTX_VTCR_EL2, struct resume_cpu_context, vtcr_el2);
   OFFSET(RESUME_CTX_VTTBR_EL2, struct resume_cpu_context, vttbr_el2);
   OFFSET(RESUME_CTX_TPIDR_EL2, struct resume_cpu_context, tpidr_el2);
   OFFSET(RESUME_CTX_MDCR_EL2, struct resume_cpu_context, mdcr_el2);
   OFFSET(RESUME_CTX_HSTR_EL2, struct resume_cpu_context, hstr_el2);
   OFFSET(RESUME_CTX_CPTR_EL2, struct resume_cpu_context, cptr_el2);
   OFFSET(RESUME_CTX_HCR_EL2, struct resume_cpu_context, hcr_el2);
   OFFSET(RESUME_CTX_CNTHCTL_EL2, struct resume_cpu_context, cnthctl_el2);
   BLANK();
#endif

   OFFSET(SMCCC_RES_a0, struct arm_smccc_res, a0);
   OFFSET(SMCCC_RES_a2, struct arm_smccc_res, a2);
   OFFSET(ARM_SMCCC_1_2_REGS_X0_OFFS, struct arm_smccc_1_2_regs, a0);
   OFFSET(ARM_SMCCC_1_2_REGS_X2_OFFS, struct arm_smccc_1_2_regs, a2);
   OFFSET(ARM_SMCCC_1_2_REGS_X4_OFFS, struct arm_smccc_1_2_regs, a4);
   OFFSET(ARM_SMCCC_1_2_REGS_X6_OFFS, struct arm_smccc_1_2_regs, a6);
   OFFSET(ARM_SMCCC_1_2_REGS_X8_OFFS, struct arm_smccc_1_2_regs, a8);
   OFFSET(ARM_SMCCC_1_2_REGS_X10_OFFS, struct arm_smccc_1_2_regs, a10);
   OFFSET(ARM_SMCCC_1_2_REGS_X12_OFFS, struct arm_smccc_1_2_regs, a12);
   OFFSET(ARM_SMCCC_1_2_REGS_X14_OFFS, struct arm_smccc_1_2_regs, a14);
   OFFSET(ARM_SMCCC_1_2_REGS_X16_OFFS, struct arm_smccc_1_2_regs, a16);
   BLANK();

#ifdef CONFIG_MPU
   DEFINE(XEN_MPUMAP_MASK_sizeof, sizeof(xen_mpumap_mask));
   DEFINE(XEN_MPUMAP_sizeof, sizeof(xen_mpumap));
   DEFINE(XEN_MPUMAP_ENTRY_SHIFT, ilog2(sizeof(pr_t)));
   DEFINE(XEN_MPUMAP_ENTRY_ZERO_OFFSET, sizeof(prbar_t) + sizeof(prlar_t));
   BLANK();
#endif
}

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
