# Xen scheduler core initialization on boot
import lldb
import sys
import os

from dataclasses import dataclass

@dataclass
class TestResult:
    name: str
    status: bool = False

    def set_passed(self):
        self.status = True
        print(f"  [PASS] {self.name}")

test_state = {
    "softirq": TestResult("Sched SoftIRQ Register"),
    "slave_softirq": TestResult("Sched Slave SoftIRQ Register"),
    "domid_idle": TestResult("IDLE_DOMAIN Register"),
    "cpu0_sched": TestResult("CPU0 Sched Init"),
    "cpu_notifier": TestResult("CPU Notifier Register"),
    "opt_sched": TestResult("Set Scheduler from cmdline"),
    "sched_active": TestResult("Set scheduler_active")
}

bp_handlers = {}

def OpenSoftIrq(debugger, target, process, frame):
    irq_nr = frame.FindRegister("x0").GetValueAsUnsigned()
    handler = frame.FindRegister("x1").GetValueAsUnsigned()

    schedule_addr = target.EvaluateExpression("schedule").GetValueAsUnsigned()
    sched_slave_addr = target.EvaluateExpression("sched_slave").GetValueAsUnsigned()

    # SCHEDULE_SOFTIRQ
    sched_softirq_nr = 3
    # SCHED_SLAVE_SOFTIRQ
    sched_sofrirq_slave_nr = 2

    if irq_nr == sched_softirq_nr and handler == schedule_addr:
        test_state["softirq"].set_passed()
    elif irq_nr == sched_sofrirq_slave_nr and handler == sched_slave_addr:
        test_state["slave_softirq"].set_passed()

    return False


def DomainCreate(debugger, target, process, frame):
    dom_id = frame.FindRegister("x0").GetValueAsUnsigned()

    # DOM_IDLE: 0x7FFF
    if dom_id == 0x7FFF:
        # Verify return valid non-NULL domain on exit
        thread = process.GetSelectedThread()

        thread.StepOut()

        new_frame = thread.GetSelectedFrame()
        ret_val = new_frame.FindRegister("x0").GetValueAsUnsigned()

        max_errno = 4095
        ptr_error_start_range = 0xFFFFFFFFFFFFFFFF - max_errno

        if ret_val != 0 and ret_val < ptr_error_start_range:
            test_state["domid_idle"].set_passed()

    return False


def RegisterCpuNotifier(debugger, target, process, frame):
    nb = frame.FindRegister("x0").GetValueAsUnsigned()
    cpu_schedule_nfb_addr = target.EvaluateExpression("&cpu_schedule_nfb").GetValueAsUnsigned()

    if nb == cpu_schedule_nfb_addr:
        test_state["cpu_notifier"].set_passed()

    return False


def CpuScheduleUp(debugger, target, process, frame):
    cpu = frame.FindRegister("x0").GetValueAsUnsigned()

    if cpu == 0:
        test_state["cpu0_sched"].set_passed()

    return False


def SchedulerInitExit(debugger, target, process, frame):
    sched_active = target.FindFirstGlobalVariable("scheduler_active").GetValueAsUnsigned()

    opt_sched_addr = target.FindFirstGlobalVariable("opt_sched").GetLoadAddress()

    ops_var = target.FindFirstGlobalVariable("operations")
    active_sched_name_addr = ops_var.GetChildMemberWithName("opt_name").GetValueAsUnsigned()

    opt_sched_error = lldb.SBError()
    active_sched_error = lldb.SBError()

    opt_sched_name = process.ReadCStringFromMemory(opt_sched_addr, 256, opt_sched_error)
    active_sched_name_addr = process.ReadCStringFromMemory(active_sched_name_addr, 256, active_sched_error)

    if opt_sched_error.Fail() or active_sched_error.Fail():
        print(f"[-] ERROR: Failed to read memory at requested and active scheduler")
        return True

    if sched_active == True:
        test_state["sched_active"].set_passed()

    if opt_sched_name == active_sched_name_addr:
        test_state["opt_sched"].set_passed()

    return True


def SchedulerInit(debugger, target, process, frame):
    global bp_handlers

    lr_addr = frame.FindRegister("x30").GetValueAsUnsigned()
    exit_bp = target.BreakpointCreateByAddress(lr_addr)
    exit_bp.AddName("SchedulerInitExit")

    bp_handlers[exit_bp.GetID()] = SchedulerInitExit

    return False


def run_test(debugger, target, process):
    global bp_handlers

    bp_open_softirq = target.BreakpointCreateByName("open_softirq")
    bp_domain_create = target.BreakpointCreateByName("domain_create")
    bp_reg_cpu_notifier = target.BreakpointCreateByName("register_cpu_notifier")
    bp_cpu_sched_up = target.BreakpointCreateByName("cpu_schedule_up")
    bp_scheduler_init = target.BreakpointCreateByName("scheduler_init")

    bp_handlers[bp_open_softirq.GetID()] = OpenSoftIrq
    bp_handlers[bp_domain_create.GetID()] = DomainCreate
    bp_handlers[bp_reg_cpu_notifier.GetID()] = RegisterCpuNotifier
    bp_handlers[bp_cpu_sched_up.GetID()] = CpuScheduleUp
    bp_handlers[bp_scheduler_init.GetID()] = SchedulerInit

    while True:
        exit = False
        process.Continue()

        if process.GetState() != lldb.eStateStopped:
            print("\n[-] Target stopped unexpectedly")
            break

        thread = process.GetSelectedThread()

        if thread.GetStopReason() == lldb.eStopReasonBreakpoint:
            hit_bp = thread.GetStopReasonDataAtIndex(0)
            frame = thread.GetSelectedFrame()

            handler_fn = bp_handlers.get(hit_bp)

            if handler_fn is not None:  
                exit = handler_fn(debugger, target, process, frame)

            if exit:
                break


def check_result():
    suite_passed = True

    print("\n  --- Test Report ---")

    for test in test_state.values():
        if not test.status:
            print(f"  [FAIL] {test.name}")
            suite_passed = False

    if suite_passed:
        print(f"  [SUCCESS] All tests passed!")
    else:
        print(f"  [FAIL] Some tests are failed.")

    return suite_passed


def target_connect(elf_path, port):
    debugger = lldb.SBDebugger.Create()
    debugger.SetAsync(False)

    target = debugger.CreateTarget(elf_path)
    if not target.IsValid():
        print(f"[-] ERROR: Could not load elf binary '{elf_path}'")
        sys.exit(1)

    error = lldb.SBError()
    connect_url = f"connect://localhost:{port}"

    process = target.ConnectRemote(debugger.GetListener(), connect_url, "gdb-remote", error)

    if error.Fail() or not process.IsValid():
        print(f"[-] ERROR: Failed to connect to QEMU on port {port}.\n")
        sys.exit(1)

    print("[+] Successfully connected to QEMU")

    return debugger, target, process


elf_path = os.environ.get("XEN_ELF", "xen")
port = int(os.environ.get("XEN_PORT", "1234"))

debugger, target, process = target_connect(elf_path=elf_path, port=port)
try:
    run_test(debugger, target, process)

    success = check_result()
    if success:
        os._exit(0)
    else:
        os._exit(1)

finally:
    print("\n[+] Detaching QEMU target")
    process.Detach()
    lldb.SBDebugger.Destroy(debugger)
