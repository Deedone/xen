# Xen panic on sched_init failure

import gdb
import os

def setup_environment():
    gdb.execute("set architecture aarch64")
    gdb.execute(f"file {os.environ['WORKDIR']}/xen-syms")
    gdb.execute("target remote :1234")
    gdb.execute("set confirm off")

class xenPanic(gdb.Breakpoint):
    def __init__(self, expected_str):
        super(xenPanic, self).__init__("panic")
        self.expected_str = expected_str

    def stop(self):
        param = gdb.parse_and_eval("(char *)$x0")
        panic_str = param.string()

        if panic_str == self.expected_str:
            print("Test result: SUCCESS. Received expected panic string")
        else:
            print("Test result: FAIL. Panic string not recognized")

        return False

class schedInit(gdb.Breakpoint):
    def __init__(self):
        super(schedInit, self).__init__("csched2_init")

    def stop(self):
        # Force exit with error
        gdb.execute("return 1")
        
        return False

setup_environment()

schedInit()
xenPanic("scheduler returned error on init\n")

gdb.execute("continue")
