# Minerva analysis tooling

Documentation root for the Minerva static- and runtime-analysis
tools that live alongside the Xen tree. These tools are
auxiliary to the Xen source: they read Xen build artifacts and
runtime logs to produce per-run diagnostic reports.

## Contents

| Doc | Subject |
| --- | --- |
| [parser-tools.md](parser-tools.md) | Runtime log parser and comments-file format. |
| [indirect-call-workbench.md](indirect-call-workbench.md) | Indirect-call collector and reachability workbench. |

## Tool inventory

| Tool | Purpose |
| --- | --- |
| `minerva_analysis/log_parser.py` | Parses Xen runtime test logs into call-path summaries, applies a comments database, and emits deterministic output. |
| `minerva_static_analysis/callpath.py` | Static caller/callee path discovery from a GCC `.ci` tree. |
| `scripts/collect.py` | Indirect-call scoping collector--  ops-table inventory, runtime-registration sites, indirect call sites. |
| `scripts/indirect_reachability.py` | Reachability workbench over collector output and a GCC `.ci` callgraph. |

## Outputs are per-run diagnostics

The numbers these tools emit--  edge counts, reachable-function
counts--  are per-run artifact values. They change with Xen
source revisions, configuration, compiler version, callgraph
coverage, and synthetic-edge filtering rules. They are not
committed constants.

## What this tooling does not assert

- No allocation bound. `path_found=yes` is a static-graph
  reachability statement.
- "Not observed at runtime" does not mean impossible. Runtime
  output reflects a specific workload, not all possible
  workloads.
- Runtime-registration rows are a separate analyst worklist;
  the collector inventories them but the reachability tool
  does not expand them into per-field queries.
- Field-name-only candidate rows are kept for analyst review
  but excluded from synthetic-edge counts.
