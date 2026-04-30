# Ensure scheduler core initialization clears scheduler plugins if global_init fails

import lldb
import sys
import os

import lldb_automation as dbg

csched_ind = -1

def Csched2_global_init(frame):
    print(f"Forcing csched_global_init() fail")

    dbg.force_return(frame, "-1")


def Check_csched2_entry(frame):
    if csched_ind == -1:
        print(f"[FAIL] failed to find target scheduler")
        os._exit(1)

    start_addr = dbg.evaluate_expression_int(frame, "&__start_schedulers_array")

    expr = f"((struct scheduler **){start_addr})[{csched_ind}]"
    sched_ptr = dbg.evaluate_expression_int(frame, expr)

    if sched_ptr == 0:
        print(f"[SUCCESS] Corrupted scheduler is now NULL")
        os._exit(0)
    else:
        print(f"[FAIL] Corrupted scheduler 0x{sched_ptr:x} is still active\n")
        os._exit(1)


def Panic(frame):
    # If corrupted scheduler is default scheduler, it is expected to BUG_ON()
    Check_csched2_entry(frame)


def SchedulerInitExit(frame):
    Check_csched2_entry(frame)


def SchedulerInit(frame):
    global csched_ind

    dbg.install_exit_hook(frame, SchedulerInitExit)

    start_addr = dbg.evaluate_expression_int(frame, "&__start_schedulers_array")
    end_addr = dbg.evaluate_expression_int(frame, "&__end_schedulers_array")

    num_schedulers = (end_addr - start_addr) // 8
    print(f"[+] Found {num_schedulers} schedulers")

    for i in range(num_schedulers):
        name_addr_expr = f"((struct scheduler **){start_addr})[{i}]->opt_name"
        name_addr = dbg.evaluate_expression_int(frame, name_addr_expr)

        name = dbg.evaluate_expression_str(frame, f"(char*){name_addr}")

        if name == "credit2":
            print(f"[+] Found 'credit2' scheduler at index '{i}'")
            csched_ind = i
            break


elf_path = os.environ.get("XEN_ELF", "xen")
port = int(os.environ.get("XEN_PORT", "1234"))

try:
    dbg.connect(elf_path, port)
    dbg.install_entry_hook("scheduler_init", SchedulerInit)
    dbg.install_entry_hook("panic", Panic)
    dbg.install_entry_hook("csched2_global_init", Csched2_global_init)
    dbg.resume()

finally:
    dbg.detach()