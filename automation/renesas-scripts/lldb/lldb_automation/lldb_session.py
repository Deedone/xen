import lldb

class DebugSession:
    """Singleton class to manage global LLDB state"""
    _instance = None

    def __new__(cls):
        if cls._instance is None:
             cls._instance = super(DebugSession, cls).__new__(cls)
             cls._instance.debugger = None
             cls._instance.target = None
             cls._instance.process = None
        return cls._instance

    def connect(self, elf_path: str, port: int) -> None:
        """
        Create an LLDB debugger instance and connects to a remote GDB stub

        Args:
            elf_path: The file path to the ELF binary with debug symbols.
            port: The localhost port of the GDB stub.

        Raises:
            FileNotFoundError: If the ELF binary cannot be found or invalid.
            ConnectionError: If LLDB fails to connect to the remote GDB stub.
        """

        if self.debugger is not None:
            return

        self.debugger = lldb.SBDebugger.Create()
        self.debugger.SetAsync(False)

        self.target = self.debugger.CreateTarget(elf_path)
        if not self.target.IsValid():
            raise FileNotFoundError(f"Could not load ELF binary at '{elf_path}'")

        error = lldb.SBError()
        connect_url = f"connect://localhost:{port}"

        self.process = self.target.ConnectRemote(self.debugger.GetListener(), connect_url, "gdb-remote", error)

        if error.Fail() or not self.process.IsValid():
            raise ConnectionError(f"Failed to connect to QEMU on port {port}: {error.GetCString()}")

        print("[+] Connected to remote process...")

    def set_bp_handler(self, bp_handler):
        """Registers the master breakpoint handler."""
        self.bp_handler = bp_handler

    def resume(self) -> None:
        """
        Continues the process and manually call breakpoints handlers as they are hit.
        """
        if not self.process or not self.process.IsValid():
            raise RuntimeError("No active process to run.")

        print("[+] Process resumed...")

        while True:
            error = self.process.Continue()
            if error.Fail():
                print(f"[-] Execution aborted: Continue failed: {error.GetCString()}")
                break

            state = self.process.GetState()

            if state == lldb.eStateExited:
                print("[+] Process exited naturally.")
                break

            elif state == lldb.eStateStopped:
                thread = self.process.GetSelectedThread()
                stop_reason = thread.GetStopReason()

                if stop_reason == lldb.eStopReasonBreakpoint:
                    hit_bp_id = thread.GetStopReasonDataAtIndex(0)
                    frame = thread.GetSelectedFrame()

                    should_stop = True

                    if self.bp_handler:
                        should_stop = self.bp_handler(hit_bp_id, frame)
                    else:
                        print(f"[*] Halted at BP {hit_bp_id}, but no handler is registered.")

                    if should_stop:
                        print("[*] Debugger paused")
                        break
                        
                elif stop_reason == lldb.eStopReasonSignal:
                    print(f"[-] Ignoring OS Signal: {thread.GetStopDescription(100)}")
                    continue 

                else:
                    print(f"[*] Halted by unknown reason: {thread.GetStopDescription(100)}")
                    break
  
    def detach(self) -> None:
        """
        Detaches from the currently running process without killing it.

        This hands control back to the target (e.g., QEMU or a remote GDB stub), 
        allowing it to continue executing normally outside the control of LLDB.

        Raises:
            RuntimeError: If there is no active process found in the session.
        """
        if not self.process or not self.process.IsValid():
            raise RuntimeError("Cannot detach: No active process found in the session.")

        error = self.process.Detach()
        
        if error.Fail():
            print(f"[-] Failed to detach from process: {error.GetCString()}")
        else:
            print("[+] Disconnected from process...")

        self.process = None
        self.target = None


session = DebugSession()
