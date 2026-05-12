"""Backend-neutral callgraph tooling for Minerva.

`normalized_graph` defines a compiler-independent callgraph
representation (`functions.csv` + `edges.csv` + `metadata.json`).
Adapter scripts under this package convert from
backend-specific inputs into the normalized form:

  gcc_ci_to_normalized.py    GCC `-fcallgraph-info=su` output
  llvm_ir_to_normalized.py   textual LLVM IR (.ll) / bitcode (.bc)

Consumers (indirect_reachability.py, indirect_ci_driver.py) speak
the normalized format and stay backend-neutral.
"""
