import lldb
from .lldb_session import session

def get_global_var_int(name: str) -> int:
    """
    Fetches a global variable and extracts it as an unsigned integer.

    Args:
        name (str): The name of the global variable to locate.

    Returns:
        int: The raw, unsigned integer value of the global variable.

    Raises:
        RuntimeError: If there is no active debug session or target loaded.
        NameError: If the global variable cannot be found in the target's symbols.
    """
    if not session.target:
        raise RuntimeError("Failed to fetch global variable: no debug session found.")

    var = session.target.FindFirstGlobalVariable(name)
    if not var.IsValid() or var.GetError().Fail():
        raise NameError(f"Global variable '{name}' not found in target symbols.")
    return var.GetValueAsUnsigned()

def get_local_var_int(frame, name) -> int:
    """
    Fetches a local variable from a specific execution frame and
    extracts it as an unsigned integer.

    Args:
        frame (lldb.SBFrame): The current execution frame to search within.
        name (str): The name of the local variable to locate.

    Returns:
        int: The raw, unsigned integer value of the variable.

    Raises:
        NameError: If the variable is not found in the current frame (e.g., if it was 
            optimized out by the compiler).
    """
    var = frame.EvaluateExpression(name)
    if not var.IsValid() or var.GetError().Fail():
        raise NameError(f"Local variable '{name}' not found in current frame (might be optimized out).")
    return var.GetValueAsUnsigned()

def evaluate_expression(frame: lldb.SBFrame, expression: str) -> lldb.SBValue:
    """
    Evaluates a C expression and returns the raw LLDB SBValue object.

    Args:
        frame (lldb.SBFrame): The current execution frame from the debugger.
        expression (str): The C/C++ expression to evaluate.

    Returns:
        lldb.SBValue: The raw LLDB value object representing the result.

    Raises:
        ValueError: If the provided frame is invalid.
        RuntimeError: If the JIT compiler fails to compile or evaluate the expression.
    """
    if not frame.IsValid():
        raise ValueError("Cannot evaluate expression: The provided frame is invalid.")

    val = frame.EvaluateExpression(expression)

    if not val.IsValid() or val.GetError().Fail():
        error_msg = val.GetError().GetCString() if val.GetError().IsValid() else "Unknown evaluation error"
        raise RuntimeError(f"Expression evaluation failed for '{expression}':\n{error_msg}")

    return val

def evaluate_expression_int(frame: lldb.SBFrame, expression: str) -> int:
    """
    Evaluates a C/C++ expression in the given frame and return result as unsigned int.
    
    Args:
        frame (lldb.SBFrame): The current execution frame from the debugger.
        expression (str): The C/C++ expression to evaluate.

    Returns:
        int: The raw, unsigned integer value of the expression result.

    Raises:
        ValueError: If the provided frame is invalid or uninitialized.
        RuntimeError: If the JIT compiler fails to compile or evaluate the expression.
    """
    val = evaluate_expression(frame, expression)

    return val.GetValueAsUnsigned()

def evaluate_expression_str(frame: lldb.SBFrame, expression: str) -> str:
    """
    Evaluates a C/C++ expression expected to be a string and returns a clean Python string.

    This wrapper safely extracts the LLDB string summary and automatically strips 
    the literal double-quotes that LLDB appends to C-strings.

    Args:
        frame (lldb.SBFrame): The current execution frame from the debugger.
        expression (str): The C/C++ expression to evaluate (e.g., "(char*)&opt_sched").

    Returns:
        str: The clean, unquoted string result of the expression.

    Raises:
        RuntimeError: If the expression evaluates successfully but returns an empty 
            or null summary, or if the underlying JIT evaluation fails entirely.
    """
    val = evaluate_expression(frame, expression)
    
    summary = val.GetSummary()
    if not summary:
        raise RuntimeError(f"Expression '{expression}' succeeded, but returned an empty or null string summary.")
        
    # Strip the literal double-quotes LLDB adds to C-strings
    clean_str = summary.strip('"')
    return clean_str.encode('utf-8').decode('unicode_escape')

def get_register_value(frame: lldb.SBFrame, reg_name: str) -> int:
    """
    Reads a hardware register directly from the current CPU context.

    Args:
        frame (lldb.SBFrame): The current execution frame provided by the debugger.
        reg_name (str): The architecture-specific name of the hardware register 
            to read (e.g., "x0", "pc").

    Returns:
        int: The raw, unsigned integer value currently held in the specified register.

    Raises:
        RuntimeError: If the specified register name is invalid, empty, or cannot 
            be found within the current CPU context.   
    """
    reg_value = frame.FindRegister(reg_name)
    
    if not reg_value.IsValid():
        raise RuntimeError(f"Register '{reg_name}' not found in current CPU context.")
        
    return reg_value.GetValueAsUnsigned()

def force_return(frame: lldb.SBFrame, return_expr: str) -> None:
    """
    Forces the debugger to immediately exit the current function, 
    returning the specified value to the caller.

    This gracefully pops the current stack frame and restores CPU 
    registers to simulate a natural function return.

    Args:
        frame (lldb.SBFrame): The current execution frame to abort.
        return_expr (str): A C/C++ expression representing the value 
            to return (e.g., "-1", "0", "true", or a variable name).

    Raises:
        ValueError: If the provided frame is invalid.
        RuntimeError: If the return expression is invalid or if the 
            frame cannot be cleanly popped.
    """
    if not frame or not frame.IsValid():
        raise ValueError("Invalid frame provided to force_return.")

    ret_val = frame.EvaluateExpression(return_expr)
    if not ret_val.IsValid() or ret_val.GetError().Fail():
        raise RuntimeError(f"Failed to evaluate return value '{return_expr}': {ret_val.GetError().GetCString()}")

    thread = frame.GetThread()
    error = thread.ReturnFromFrame(frame, ret_val)
    
    if error.Fail():
        raise RuntimeError(f"Failed to force return from frame: {error.GetCString()}")
