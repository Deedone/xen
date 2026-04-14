from .lldb_session import session
from .lldb_hooks import install_entry_hook, install_exit_hook
from .lldb_util import *

def connect(elf_path: str, port: int) -> None:
    """
    Creates an LLDB debugger instance and connects to a remote GDB stub.

    Args:
        elf_path (str): The file path to the ELF binary with debug symbols.
        port (int): The localhost port of the GDB stub.

    Raises:
        FileNotFoundError: If the ELF binary cannot be found or invalid.
        ConnectionError: If LLDB fails to connect to the remote GDB stub.
    """
    return session.connect(elf_path, port)

def detach() -> None:
    """
    Detaches from the currently running process without killing it.

    This hands control back to the target (e.g., QEMU or a remote GDB stub), 
    allowing it to continue executing normally outside the control of LLDB.

    Raises:
        RuntimeError: If there is no active process to detach from, or if 
                      the detach operation fails.
    """
    return session.detach()

def resume() -> None:
    """
    Resumes the execution of the target process.
    """
    return session.resume()

def get_target() -> lldb.SBTarget:
    """
    Retrieves the active LLDB SBTarget instance.
    
    Returns:
        The current target if connected, otherwise None.
    """
    return session.target

def get_process() -> lldb.SBProcess:
    """
    Retrieves the active LLDB SBProcess instance.
    
    Returns:
        The current process if connected, otherwise None.
    """
    return session.process

def get_debugger() -> lldb.SBDebugger:
    """
    Retrieves the active LLDB SBDebugger instance.
    
    Returns:
        The current debugger if created, otherwise None.
    """
    return session.process


__all__ = [
    "session",
    "connect",
    "detach",
    "resume",
    "get_target",
    "get_process",
    "get_debugger",
    "install_entry_hook",
    "install_exit_hook",
    "get_global_var_int",
    "get_local_var_int",
    "evaluate_expression",
    "evaluate_expression_int",
    "evaluate_expression_str",
    "get_return_value",
    "get_register_value",
    "force_return",
]
