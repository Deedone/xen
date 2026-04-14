import lldb
import sys
from .lldb_session import session

__entry_hooks = {}
__exit_hooks = {}

def install_entry_hook(symbol_name: str, on_entry: callable, pin_thread: bool = False) -> int:
    """
    Installs a persistent breakpoint on the entry of a specified function by name.

    Args:
        symbol_name (str): The C/C++ function name to break on.
        on_entry (callable): A Python callback function to execute when the function 
            is entered. Expected signature: func(frame).
        pin_thread (bool, optional): If True, restricts the breakpoint to the 
            currently active execution thread.

    Returns:
        int: The installed Breakpoint ID.

    Raises:
        RuntimeError: If there is no active debug session, or if same_thread is True 
            but no thread is currently selected.
        ValueError: If the symbol name cannot be resolved to a valid location.
    """

    if not session.target:
        raise RuntimeError("Failed to install hook: no debug session found")

    bp = session.target.BreakpointCreateByName(symbol_name)
    if not bp.IsValid() or bp.GetNumLocations() == 0:
        raise ValueError(f"Could not resolve symbol '{symbol_name}'")

    if pin_thread:
        try:
            bp.SetThreadID(session.thread.GetThreadID())
        except Exception as e:
            print(f"[-] Failed to set pin breakpoint to thread {bp_id}: {e}")

    bp_id = bp.GetID()

    __entry_hooks[bp_id] = on_entry

    return bp_id

def install_exit_hook(frame: lldb.SBFrame, on_exit: callable) -> int:
    """
    Sets a one-shot breakpoint on the return address of the current frame.
    
    This effectively creates a hook that triggers exactly once when the 
    current function finishes executing and pops off the stack.

    Need to be called on function entrance.

    Args:
        frame (lldb.SBFrame): The current execution frame from the debugger.
        on_exit (callable): A Python callback function to execute when the 
            function exits. Expected signature: func(frame).

    Returns:
        int: The installed Breakpoint ID.

    Raises:
        ValueError: If the provided SBFrame is invalid.
        RuntimeError: If the thread cannot be retrieved, no parent frame is 
            found (e.g., stack depth is too shallow), or breakpoint creation fails.
    """
    if not frame.IsValid():
        raise ValueError("Provided SBFrame is invalid.")

    lr_reg = frame.FindRegister("lr")

    if lr_reg.IsValid():
        return_addr = lr_reg.GetValueAsUnsigned()
    else:
        raise RuntimeError("Error: Could not find the 'lr' register.")

    thread = frame.GetThread()
    if not thread.IsValid():
        raise RuntimeError("Could not retrieve thread from frame.")
    
    target = thread.GetProcess().GetTarget()
    
    bp = target.BreakpointCreateByAddress(return_addr)
    
    if not bp.IsValid():
        raise RuntimeError(f"Failed to set breakpoint at {hex(return_addr)}.")

    bp.SetThreadID(thread.GetThreadID())

    bp_id = bp.GetID()

    __exit_hooks[bp_id] = on_exit
    
    return bp_id

def on_breakpoint_hit(bp_id: int, frame: lldb.SBFrame):
    """
    Called by the session's loop whenever ANY breakpoint is hit.
    Returns True to halt the debugger, False to auto-continue.
    """
    
    # Check if it is an Entry Hook
    if bp_id in __entry_hooks:
        try:
            __entry_hooks[bp_id](frame)
            return False
        except Exception as e:
            print(f"[-] CRITICAL: Exception in ENTRY Hook {__entry_hooks[bp_id]}: {e}")
            return True # Stop on crash
            
    # Check if it is an Exit Hook
    elif bp_id in __exit_hooks:
        user_exit_cb = __exit_hooks.pop(bp_id) 
        try:
            user_exit_cb(frame)
            frame.GetThread().GetProcess().GetTarget().BreakpointDelete(bp_id)
            return False
        except Exception as e:
            print(f"[-] CRITICAL: Exception in EXIT Hook {user_exit_cb}: {e}")
            return True # Stop on crash

    # 3. Ghost Breakpoint (Not in either table)
    else:
        print(f"[*] Stopped at BP {bp_id}, but it no handler found")
        return False

session.set_bp_handler(on_breakpoint_hit)