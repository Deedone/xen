# Ensure scheduler core initialization clears scheduler plugins without required ops

import lldb
import sys
import os

import lldb_automation as dbg

def SchedulerInitExit(frame):
    expr = f"&((struct scheduler *)__start_schedulers_array)[{0}]"
    sched_ptr = dbg.evaluate_expression_int(frame, expr)

    if sched_ptr == 0:
        print(f"[SUCCESS] Corrupted scheduler is now NULL")
        os._exit(0)
    else:
        print(f"[FAIL] Corrupted scheduler 0x{sched_ptr:x} is still active\n")
        os._exit(1)


def SchedulerInit(frame):
    dbg.install_exit_hook(frame, SchedulerInitExit)

    name_addr_expr = f"((struct scheduler *)__start_schedulers_array)[0]->name"
    name_addr = dbg.evaluate_expression_int(frame, name_addr_expr)
    name = dbg.evaluate_expression_str(frame, f"(char*){name_addr}")

    print(f"[+] Scheduler '{name}' at index {0}!")

    # 4. Calculate the address of do_schedule
    do_sched_expr = f"&(((struct scheduler *)__start_schedulers_array)[0]->do_schedule)"
    do_sched_ofset = dbg.evaluate_expression_int(frame, do_sched_expr)

    dbg.evaluate_expression(frame, f"*(uint64_t *){do_sched_ofset}=0x0")


elf_path = os.environ.get("XEN_ELF", "xen")
port = int(os.environ.get("XEN_PORT", "1234"))

try:
    dbg.connect(elf_path, port)
    dbg.install_entry_hook("scheduler_init", SchedulerInit)
    dbg.resume()

finally:
    dbg.detach()