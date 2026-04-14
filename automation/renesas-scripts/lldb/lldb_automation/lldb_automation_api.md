<a id="lldb_automation"></a>

# lldb\_automation

<a id="connect"></a>

#### connect

```python
def connect(elf_path: str, port: int) -> None
```

Creates an LLDB debugger instance and connects to a remote GDB stub.

**Arguments**:

- `elf_path` _str_ - The file path to the ELF binary with debug symbols.
- `port` _int_ - The localhost port of the GDB stub.
  

**Raises**:

- `FileNotFoundError` - If the ELF binary cannot be found or invalid.
- `ConnectionError` - If LLDB fails to connect to the remote GDB stub.

<a id="detach"></a>

#### detach

```python
def detach() -> None
```

Detaches from the currently running process without killing it.

This hands control back to the target (e.g., QEMU or a remote GDB stub),
allowing it to continue executing normally outside the control of LLDB.

**Raises**:

- `RuntimeError` - If there is no active process to detach from, or if
  the detach operation fails.

<a id="resume"></a>

#### resume

```python
def resume() -> None
```

Resumes the execution of the target process.

<a id="get_target"></a>

#### get\_target

```python
def get_target() -> lldb.SBTarget
```

Retrieves the active LLDB SBTarget instance.

**Returns**:

  The current target if connected, otherwise None.

<a id="get_process"></a>

#### get\_process

```python
def get_process() -> lldb.SBProcess
```

Retrieves the active LLDB SBProcess instance.

**Returns**:

  The current process if connected, otherwise None.

<a id="get_debugger"></a>

#### get\_debugger

```python
def get_debugger() -> lldb.SBDebugger
```

Retrieves the active LLDB SBDebugger instance.

**Returns**:

  The current debugger if created, otherwise None.

<a id="get_global_var_int"></a>

#### get\_global\_var\_int

```python
def get_global_var_int(name: str) -> int
```

Fetches a global variable and extracts it as an unsigned integer.

**Arguments**:

- `name` _str_ - The name of the global variable to locate.
  

**Returns**:

- `int` - The raw, unsigned integer value of the global variable.
  

**Raises**:

- `RuntimeError` - If there is no active debug session or target loaded.
- `NameError` - If the global variable cannot be found in the target's symbols.

<a id="get_local_var_int"></a>

#### get\_local\_var\_int

```python
def get_local_var_int(frame, name) -> int
```

Fetches a local variable from a specific execution frame and
extracts it as an unsigned integer.

**Arguments**:

- `frame` _lldb.SBFrame_ - The current execution frame to search within.
- `name` _str_ - The name of the local variable to locate.
  

**Returns**:

- `int` - The raw, unsigned integer value of the variable.
  

**Raises**:

- `NameError` - If the variable is not found in the current frame (e.g., if it was
  optimized out by the compiler).

<a id="evaluate_expression"></a>

#### evaluate\_expression

```python
def evaluate_expression(frame: lldb.SBFrame, expression: str) -> lldb.SBValue
```

Evaluates a C expression and returns the raw LLDB SBValue object.

**Arguments**:

- `frame` _lldb.SBFrame_ - The current execution frame from the debugger.
- `expression` _str_ - The C/C++ expression to evaluate.
  

**Returns**:

- `lldb.SBValue` - The raw LLDB value object representing the result.
  

**Raises**:

- `ValueError` - If the provided frame is invalid.
- `RuntimeError` - If the JIT compiler fails to compile or evaluate the expression.

<a id="evaluate_expression_int"></a>

#### evaluate\_expression\_int

```python
def evaluate_expression_int(frame: lldb.SBFrame, expression: str) -> int
```

Evaluates a C/C++ expression in the given frame and return result as unsigned int.

**Arguments**:

- `frame` _lldb.SBFrame_ - The current execution frame from the debugger.
- `expression` _str_ - The C/C++ expression to evaluate.
  

**Returns**:

- `int` - The raw, unsigned integer value of the expression result.
  

**Raises**:

- `ValueError` - If the provided frame is invalid or uninitialized.
- `RuntimeError` - If the JIT compiler fails to compile or evaluate the expression.

<a id="evaluate_expression_str"></a>

#### evaluate\_expression\_str

```python
def evaluate_expression_str(frame: lldb.SBFrame, expression: str) -> str
```

Evaluates a C/C++ expression expected to be a string and returns a clean Python string.

This wrapper safely extracts the LLDB string summary and automatically strips
the literal double-quotes that LLDB appends to C-strings.

**Arguments**:

- `frame` _lldb.SBFrame_ - The current execution frame from the debugger.
- `expression` _str_ - The C/C++ expression to evaluate (e.g., "(char*)&opt_sched").
  

**Returns**:

- `str` - The clean, unquoted string result of the expression.
  

**Raises**:

- `RuntimeError` - If the expression evaluates successfully but returns an empty
  or null summary, or if the underlying JIT evaluation fails entirely.

<a id="get_register_value"></a>

#### get\_register\_value

```python
def get_register_value(frame: lldb.SBFrame, reg_name: str) -> int
```

Reads a hardware register directly from the current CPU context.

**Arguments**:

- `frame` _lldb.SBFrame_ - The current execution frame provided by the debugger.
- `reg_name` _str_ - The architecture-specific name of the hardware register
  to read (e.g., "x0", "pc").
  

**Returns**:

- `int` - The raw, unsigned integer value currently held in the specified register.
  

**Raises**:

- `RuntimeError` - If the specified register name is invalid, empty, or cannot
  be found within the current CPU context.

<a id="force_return"></a>

#### force\_return

```python
def force_return(frame: lldb.SBFrame, return_expr: str) -> None
```

Forces the debugger to immediately exit the current function,
returning the specified value to the caller.

This gracefully pops the current stack frame and restores CPU
registers to simulate a natural function return.

**Arguments**:

- `frame` _lldb.SBFrame_ - The current execution frame to abort.
- `return_expr` _str_ - A C/C++ expression representing the value
  to return (e.g., "-1", "0", "true", or a variable name).
  

**Raises**:

- `ValueError` - If the provided frame is invalid.
- `RuntimeError` - If the return expression is invalid or if the
  frame cannot be cleanly popped.

<a id="install_entry_hook"></a>

#### install\_entry\_hook

```python
def install_entry_hook(symbol_name: str,
                       on_entry: callable,
                       pin_thread: bool = False) -> int
```

Installs a persistent breakpoint on the entry of a specified function by name.

**Arguments**:

- `symbol_name` _str_ - The C/C++ function name to break on.
- `on_entry` _callable_ - A Python callback function to execute when the function
  is entered. Expected signature: func(frame).
- `pin_thread` _bool, optional_ - If True, restricts the breakpoint to the
  currently active execution thread.
  

**Returns**:

- `int` - The installed Breakpoint ID.
  

**Raises**:

- `RuntimeError` - If there is no active debug session, or if same_thread is True
  but no thread is currently selected.
- `ValueError` - If the symbol name cannot be resolved to a valid location.

<a id="install_exit_hook"></a>

#### install\_exit\_hook

```python
def install_exit_hook(frame: lldb.SBFrame, on_exit: callable) -> int
```

Sets a one-shot breakpoint on the return address of the current frame.

This effectively creates a hook that triggers exactly once when the
current function finishes executing and pops off the stack.

Need to be called on function entrance.

**Arguments**:

- `frame` _lldb.SBFrame_ - The current execution frame from the debugger.
- `on_exit` _callable_ - A Python callback function to execute when the
  function exits. Expected signature: func(frame).
  

**Returns**:

- `int` - The installed Breakpoint ID.
  

**Raises**:

- `ValueError` - If the provided SBFrame is invalid.
- `RuntimeError` - If the thread cannot be retrieved, no parent frame is
  found (e.g., stack depth is too shallow), or breakpoint creation fails.

