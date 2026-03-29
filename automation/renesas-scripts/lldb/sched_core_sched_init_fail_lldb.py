import lldb
import sys
import os

def target_connect(elf_path, port):
    debugger = lldb.SBDebugger.Create()
    debugger.SetAsync(False)

    target = debugger.CreateTarget(elf_path)
    if not target.IsValid():
        print(f"[-] ERROR: Could not load elf binary '{elf_path}'")
        sys.exit(1)

    error = lldb.SBError()
    connect_url = f"connect://localhost:{port}"

    process = target.ConnectRemote(debugger.GetListener(), connect_url, "gdb-remote", error)

    if error.Fail() or not process.IsValid():
        print(f"[-] ERROR: Failed to connect to QEMU on port {port}.\n")
        sys.exit(1)

    print("[+] Successfully connected to QEMU")

    return debugger, target, process


def run_test(debugger, target, process):
    bp_schedInit = target.BreakpointCreateByName("csched2_init")
    bp_panic = target.BreakpointCreateByName("panic")

    test_passed = False

    while True:
        process.Continue()

        if process.GetState() != lldb.eStateStopped:
            print("\n[-] Target stopped unexpectedly")
            break

        thread = process.GetSelectedThread()

        if thread.GetStopReason() == lldb.eStopReasonBreakpoint:
            hit_bp = thread.GetStopReasonDataAtIndex(0)
            frame = thread.GetSelectedFrame()

            if hit_bp == bp_schedInit.GetID():
                print(f"[+] Hit {frame.GetFunctionName()}. Forcing error return...")

                res = lldb.SBCommandReturnObject()
                debugger.GetCommandInterpreter().HandleCommand("thread return -1", res)

                if not res.Succeeded():
                    print(f"[-] Failed to force return: {res.GetError()}")
                    break
            elif hit_bp == bp_panic.GetID():
                print(f"[+] Hit {frame.GetFunctionName()}")

                x0_addr = frame.FindRegister("x0").GetValueAsUnsigned()

                error = lldb.SBError()
                panic_str = process.ReadCStringFromMemory(x0_addr, 256, error)

                if error.Fail():
                    print(f"[-] ERROR: Failed to read memory at {x0_addr:#x}")
                    break
                
                expected_str = "scheduler returned error on init\n"

                if panic_str == expected_str:
                    test_passed = True
                    print(f"[SUCCESS] Received expected panic string.")
                    break
                else:
                    print(f"[FAIL] Panic string don't match to expected\n"
                        f"Expected:{repr(expected_str)}\nGot: {repr(panic_str)}")
                
    return test_passed

elf_path = os.environ.get("XEN_ELF", "xen")
port = int(os.environ.get("XEN_PORT", "1234"))

debugger, target, process = target_connect(elf_path=elf_path, port=port)
try:
    success = run_test(debugger, target, process)
    if not success:
        sys.exit(1)
finally:
    print("\n[+] Detaching QEMU target")
    process.Detach()
    lldb.SBDebugger.Destroy(debugger)
