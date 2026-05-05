# Ensure scheduler core initialization forces crash if no valid scheduler found

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
    start_addr = dbg.evaluate_expression_int(frame, "&__start_schedulers_array")
    end_addr = dbg.evaluate_expression_int(frame, "&__end_schedulers_array")

    num_schedulers = (end_addr - start_addr) // 8
    print(f"[+] Found {num_schedulers} schedulers")

    # Clear opt_sched cmdline parameter
    opt_sched_addr = dbg.evaluate_expression_int(frame, "&opt_sched[0]")

    dbg.evaluate_expression(frame, f"*(char*){opt_sched_addr}=0x0")

    # Ensure sched_get_by_name() returns 'NULL' by corrupting default scheduler
    # Assume CONFIG_SCHED_DEFAULT="credit2"
    for i in range(num_schedulers):
        name_addr_expr = f"((struct scheduler **){start_addr})[{i}]->opt_name"
        name_addr = dbg.evaluate_expression_int(frame, name_addr_expr)

        name = dbg.evaluate_expression_str(frame, f"(char*){name_addr}")

        if name == "credit2":
            print(f"[+] Found 'credit2' scheduler at index '{i}'")

            # Corrupt 'credit2' scheduler name to cause sched_get_by_name() fail
            name_addr += 1
            dbg.evaluate_expression(frame,
                        f"((struct scheduler **){start_addr})[{i}]->opt_name = (char*){name_addr}")

            name_addr = dbg.evaluate_expression_int(frame, name_addr_expr)

            name = dbg.evaluate_expression_str(frame, f"(char*){name_addr}")

            print(f"[+] Updated name to: {name}")

            break


elf_path = os.environ.get("XEN_ELF", "xen")
port = int(os.environ.get("XEN_PORT", "1234"))

try:
    dbg.connect(elf_path, port)
    dbg.install_entry_hook("scheduler_init", SchedulerInit)
    dbg.install_entry_hook("panic", Panic)
    dbg.resume()

finally:
    dbg.detach()
