# Ensure scheduler core initialization aligns sched_ratelimit_us value

import lldb
import sys
import os

import lldb_automation as dbg

def SchedulerInitExit(frame):
    SCHED_DEFAULT_RATELIMIT_US=1000

    sched_ratelimit_us = dbg.get_global_var_int("sched_ratelimit_us")

    if sched_ratelimit_us == SCHED_DEFAULT_RATELIMIT_US:
        print(f"[SUCCESS] sched_ratelimit_us({sched_ratelimit_us}) set as expected")
    else:
        print(f"[FAIL] Expected sched_ratelimit_us "
              f"'{SCHED_DEFAULT_RATELIMIT_US}', got '{sched_ratelimit_us}'")

def SchedulerInit(frame):
    dbg.install_exit_hook(frame, SchedulerInitExit)

    sched_ratelimit_us = dbg.get_global_var_int("sched_ratelimit_us")
    print(f"[+] Cur sched_ratelimit_us: {sched_ratelimit_us}")

    dbg.evaluate_expression(frame, "sched_ratelimit_us = 500001")

    sched_ratelimit_us = dbg.get_global_var_int("sched_ratelimit_us")
    print(f"[+] Set sched_ratelimit_us: {sched_ratelimit_us}")


elf_path = os.environ.get("XEN_ELF", "xen")
port = int(os.environ.get("XEN_PORT", "1234"))

try:
    dbg.connect(elf_path, port)
    dbg.install_entry_hook("scheduler_init", SchedulerInit)
    dbg.resume()

finally:
    dbg.detach()