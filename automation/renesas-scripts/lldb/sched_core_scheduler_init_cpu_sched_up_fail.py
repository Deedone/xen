# Ensure scheduler core initialization forces crash if sched init for cpu '0' fails

import lldb
import sys
import os

import lldb_automation as dbg

fault_injected = False

def Panic(frame):
    x0_addr = dbg.get_register_value(frame, "x0")

    panic_str = dbg.evaluate_expression_str(frame, f"(char*){x0_addr}")

    expected_str = "Xen BUG"

    if panic_str.startswith(expected_str):
        print(f"[SUCCESS] Received expected panic string.")
    else:
        print(f"[FAIL] Panic string don't match to expected\n"
            f"Expected:{repr(expected_str)}\nGot: {repr(panic_str)}")

    sys.stdout.flush()
    os._exit(0)

def Xzalloc(frame):
    global fault_injected

    if fault_injected:
        return

    size = dbg.get_register_value(frame, "x0")
    sched_res_size = dbg.evaluate_expression_int(frame, "sizeof(struct sched_resource)")
    if size != sched_res_size:
        return

    caller = frame.GetThread().GetFrameAtIndex(1)
    if caller.GetFunctionName() not in ("sched_alloc_res", "cpu_schedule_up"):
        return

    fault_injected = True
    dbg.force_return(frame, "0")

def SchedulerInit(frame):
    dbg.install_entry_hook("_xzalloc", Xzalloc, pin_thread=True)

elf_path = os.environ.get("XEN_ELF", "xen")
port = int(os.environ.get("XEN_PORT", "1234"))

try:
    dbg.connect(elf_path, port)
    dbg.install_entry_hook("scheduler_init", SchedulerInit)
    dbg.install_entry_hook("panic", Panic)
    dbg.resume()

finally:
    dbg.detach()
