# Ensure scheduler core initialization forces crash if target scheduler init fails

import lldb
import sys
import os

import lldb_automation as dbg

def Csched2Init(frame):
    dbg.force_return(frame, "-1")

def Panic(frame):
    x0_addr = dbg.get_register_value(frame, "x0")

    panic_str = dbg.evaluate_expression_str(frame, f"(char*){x0_addr}")

    expected_str = "scheduler returned error on init\n"

    if panic_str == expected_str:
        print(f"[SUCCESS] Received expected panic string.")
    else:
        print(f"[FAIL] Panic string don't match to expected\n"
            f"Expected:{repr(expected_str)}\nGot: {repr(panic_str)}")

    sys.stdout.flush()
    os._exit(0)


elf_path = os.environ.get("XEN_ELF", "xen")
port = int(os.environ.get("XEN_PORT", "1234"))

try:
    dbg.connect(elf_path, port)
    dbg.install_entry_hook("csched2_init", Csched2Init)
    dbg.install_entry_hook("panic", Panic)
    dbg.resume()

finally:
    dbg.detach()
