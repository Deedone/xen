# Xen scheduler core initialization on boot

import lldb
import sys
import os

from dataclasses import dataclass

import lldb_automation as dbg

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

def OpenSoftIrq(frame):
    irq_nr = dbg.get_register_value(frame, "x0")
    handler = dbg.get_register_value(frame, "x1")

    schedule_addr = dbg.evaluate_expression_int(frame, "schedule")
    sched_slave_addr = dbg.evaluate_expression_int(frame, "sched_slave")

    # SCHEDULE_SOFTIRQ
    sched_softirq_nr = 3
    # SCHED_SLAVE_SOFTIRQ
    sched_sofrirq_slave_nr = 2

    if irq_nr == sched_softirq_nr and handler == schedule_addr:
        test_state["softirq"].set_passed()
    elif irq_nr == sched_sofrirq_slave_nr and handler == sched_slave_addr:
        test_state["slave_softirq"].set_passed()

def DomainCreateExit(frame):
    ret_val = dbg.get_register_value(frame, "x30")

    max_errno = 4095
    ptr_error_start_range = 0xFFFFFFFFFFFFFFFF - max_errno

    if ret_val != 0 and ret_val < ptr_error_start_range:
        test_state["domid_idle"].set_passed()


def DomainCreate(frame):
    dom_id = dbg.get_register_value(frame, "x0")

    # DOM_IDLE: 0x7FFF
    if dom_id == 0x7FFF:
        # Verify return valid non-NULL domain on exit
        dbg.install_exit_hook(frame, DomainCreateExit)


def RegisterCpuNotifier(frame):
    nb = dbg.get_register_value(frame, "x0")
    cpu_schedule_nfb_addr = dbg.evaluate_expression_int(frame, "&cpu_schedule_nfb")

    if nb == cpu_schedule_nfb_addr:
        test_state["cpu_notifier"].set_passed()


def CpuScheduleUp(frame):
    cpu = dbg.get_register_value(frame, "x0")

    if cpu == 0:
        test_state["cpu0_sched"].set_passed()


def SchedulerInitExit(frame):
    sched_active = dbg.get_global_var_int("scheduler_active")

    opt_sched_name = dbg.evaluate_expression_str(frame, "opt_sched")

    active_sched_name_addr = dbg.evaluate_expression_str(frame, "operations.opt_name")

    if sched_active == True:
        test_state["sched_active"].set_passed()

    if opt_sched_name == active_sched_name_addr:
        test_state["opt_sched"].set_passed()

    check_result()

    sys.stdout.flush()
    os._exit(0)

def SchedulerInit(frame):
    dbg.install_entry_hook("open_softirq", OpenSoftIrq)
    dbg.install_entry_hook("domain_create", DomainCreate)
    dbg.install_entry_hook("register_cpu_notifier", RegisterCpuNotifier)
    dbg.install_entry_hook("cpu_schedule_up", CpuScheduleUp)
    dbg.install_exit_hook(frame, SchedulerInitExit)

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

elf_path = os.environ.get("XEN_ELF", "xen")
port = int(os.environ.get("XEN_PORT", "1234"))

try:
    dbg.connect(elf_path, port)
    dbg.install_entry_hook("scheduler_init", SchedulerInit)
    dbg.resume()

finally:
    dbg.detach()
