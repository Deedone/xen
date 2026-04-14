# Ensure scheduler core initialization forces crash if sched init for cpu '0' fails

import lldb
import sys
import os

import lldb_automation as dbg

def Panic(frame):
    x0_addr = dbg.get_register_value(frame, "x0")

    panic_str = dbg.evaluate_expression_str(frame, f"(char*){x0_addr}")

    expected_str = "Xen BUG"

    if panic_str.startswith(expected_str):
        print(f"[SUCCESS] Received expected panic string.")
        os._exit(0)
    else:
        print(f"[FAIL] Panic string don't match to expected\n"
            f"Expected:{repr(expected_str)}\nGot: {repr(panic_str)}")
        os._exit(1)

def CpuSchedUp(frame):
    dbg.force_return(frame, "1")

def SchedulerInit(frame):
    dbg.install_entry_hook("cpu_schedule_up", CpuSchedUp)

elf_path = os.environ.get("XEN_ELF", "xen")
port = int(os.environ.get("XEN_PORT", "1234"))

try:
    dbg.connect(elf_path, port)
    dbg.install_entry_hook("scheduler_init", SchedulerInit)
    dbg.install_entry_hook("panic", Panic)
    dbg.resume()

finally:
    dbg.detach()