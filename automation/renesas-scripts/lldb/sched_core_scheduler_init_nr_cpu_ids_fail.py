# Ensure scheduler core initialization forces crash if nr_cpu_ids > ARRAY_SIZE(idle_vcpu)

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
    else:
        print(f"[FAIL] Panic string don't match to expected\n"
            f"Expected:{repr(expected_str)}\nGot: {repr(panic_str)}")

    sys.stdout.flush()
    os._exit(0)

def SchedulerInit(frame):
    nr_cpu_ids = dbg.get_global_var_int("nr_cpu_ids")
    print(f"Current nr_cpu_ids: {nr_cpu_ids}")

    idle_vcpu_array_size_expr = "sizeof(idle_vcpu) / sizeof(idle_vcpu[0])"
    max_cpus = dbg.evaluate_expression_int(frame, idle_vcpu_array_size_expr)

    dbg.evaluate_expression(frame, f"nr_cpu_ids = {max_cpus + 1}")

    nr_cpu_ids = dbg.get_global_var_int("nr_cpu_ids")
    print(f"Set nr_cpu_ids: {nr_cpu_ids}")

elf_path = os.environ.get("XEN_ELF", "xen")
port = int(os.environ.get("XEN_PORT", "1234"))

try:
    dbg.connect(elf_path, port)
    dbg.install_entry_hook("scheduler_init", SchedulerInit)
    dbg.install_entry_hook("panic", Panic)
    dbg.resume()

finally:
    dbg.detach()
