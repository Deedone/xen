# Xen scheduler core initialization on boot
import gdb
import os
from dataclasses import dataclass

@dataclass
class TestResult:
    name: str
    status: bool = False

    def set_passed(self):
        self.status = True
        print(f"[PASS] {self.name}")
        print("[PASS]")

test_state = {
    "softirq": TestResult("Sched SoftIRQ Register"),
    "slave_softirq": TestResult("Sched Slave SoftIRQ Register"),
    "domid_idle": TestResult("IDLE_DOMAIN Register"),
    "cpu0_sched": TestResult("CPU0 Sched Init"),
    "cpu_notifier": TestResult("CPU Notifier Register"),
    "opt_sched": TestResult("Set Scheduler from cmdline"),
    "sched_active": TestResult("Set scheduler_active")
}

def setup_environment():
    gdb.execute("set architecture aarch64")
    gdb.execute("set breakpoint pending on")
    gdb.execute("set pagination off")
    gdb.execute(f"file {os.environ['WORKDIR']}/xen-syms")
    gdb.execute("target remote :1234")
    gdb.execute("set confirm off")

class OpenSoftIrq(gdb.Breakpoint):
    def __init__(self):
        super(OpenSoftIrq, self).__init__("open_softirq", internal=True)

    def stop(self):
        try:
            irq_nr = gdb.parse_and_eval("$x0")
            handler = gdb.parse_and_eval("$x1")

            schedule_addr = int(gdb.parse_and_eval("&schedule"))
            sched_slave_addr = int(gdb.parse_and_eval("&sched_slave"))

            sched_softirq_nr = int(gdb.parse_and_eval("SCHEDULE_SOFTIRQ"))
            sched_sofrirq_slave_nr = int(gdb.parse_and_eval("SCHED_SLAVE_SOFTIRQ"))

            if irq_nr == sched_softirq_nr and handler == schedule_addr:
                test_state["softirq"].set_passed()
            elif irq_nr == sched_sofrirq_slave_nr and handler == sched_slave_addr:
                test_state["slave_softirq"].set_passed()
        except gdb.error as e:
            print(f"Error validating softirq: {e}")

        return False

class DomainCreateExit(gdb.FinishBreakpoint):
    def __init__(self, frame):
        super(DomainCreateExit, self).__init__(frame, internal=True)

    def stop(self):
        try:
            ret_val = self.return_value
            if ret_val is None:
                ret_val = gdb.parse_and_eval("$x0")

            max_errno = 4095
            ptr_error_start_range = 0xFFFFFFFFFFFFFFFF - max_errno

            if ret_val != 0 and ret_val < ptr_error_start_range:
                test_state["domid_idle"].set_passed()
        except gdb.error as e:
            print(f"Error validating idle domain ret value: {e}")

        return False

class DomainCreate(gdb.Breakpoint):
    def __init__(self):
        super(DomainCreate, self).__init__("domain_create", internal=True)

    def stop(self):
        try:
            dom_id = gdb.parse_and_eval("$x0")
            # DOM_IDLE: 0x7FFF
            if dom_id == 0x7FFF:
                # Verify return valid non-NULL domain on exit
                DomainCreateExit(gdb.newest_frame())            
        except gdb.error as e:
            print(f"Error validating idle domain: {e}")

        return False

class RegisterCpuNotifier(gdb.Breakpoint):
    def __init__(self):
        super(RegisterCpuNotifier, self).__init__("register_cpu_notifier", internal=True)

    def stop(self):
        try:
            nb = gdb.parse_and_eval("$x0")
            cpu_schedule_nfb_addr = int(gdb.parse_and_eval("&cpu_schedule_nfb"))

            if nb == cpu_schedule_nfb_addr:
                test_state["cpu_notifier"].set_passed()
        except gdb.error as e:
            print(f"Error validating cpu notifier: {e}")  
        return False

class CpuScheduleUp(gdb.Breakpoint):
    def __init__(self):
        super(CpuScheduleUp, self).__init__("cpu_schedule_up", internal=True)

    def stop(self):
        try:
            cpu = gdb.parse_and_eval("$x0")

            if cpu == 0:
                test_state["cpu0_sched"].set_passed()
        except gdb.error as e:
            print(f"Error validating cpu scedule up: {e}")  

        return False

class SchedulerInitExit(gdb.FinishBreakpoint):
    def __init__(self, frame):
        super(SchedulerInitExit, self).__init__(frame, internal=True)

    def sched_active_check(self):
        try:
            sched_active = gdb.parse_and_eval("scheduler_active")

            if sched_active == True:
                test_state["sched_active"].set_passed()
        except gdb.error as e:
            print(f"Error validating scheduler activated: {e}")  

        return False

    def sched_name_check(self):
        try:
            opt_sched_str = gdb.parse_and_eval("opt_sched").string()
            active_sched_str = gdb.parse_and_eval("operations.opt_name").string()
            
            if active_sched_str == opt_sched_str:
                test_state["opt_sched"].set_passed()
        except gdb.error as e:
            print(f"Error validating scheduler assignment: {e}")  

    def check_result(self):
        suite_passed = True

        print("--- Test Report ---")

        for test in test_state.values():
            if not test.status:
                print(f"[FAIL] {test.name}")
                suite_passed = False

        if suite_passed:
            print("Test result: SUCCESS. All tests passed!")
        else:
            print("Test result: FAIL. Some tests are failed.")

        return suite_passed

    def stop(self):
        self.sched_active_check()
        self.sched_name_check()

        exit_code = 0

        if not self.check_result():
            exit_code = 1

        gdb.execute(f"quit {exit_code}")

        return True

class SchedulerInitEntry(gdb.Breakpoint):
    def __init__(self):
        super(SchedulerInitEntry, self).__init__("scheduler_init", internal=True)

    def stop(self):
        try:
            OpenSoftIrq()
            RegisterCpuNotifier()
            CpuScheduleUp()
            DomainCreate()

            # Set breakpoint on exit
            SchedulerInitExit(gdb.newest_frame())
        except ValueError:
            print("Failed to set breakpoints.")

        return False


setup_environment()
SchedulerInitEntry()

gdb.execute("continue")
