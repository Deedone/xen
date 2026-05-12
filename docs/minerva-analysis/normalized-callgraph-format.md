# Normalized callgraph format

A compiler-independent callgraph representation. Three sibling
files under one directory; CSV for data, JSON for provenance.

## Files

```
normalized-callgraph/
  functions.csv
  edges.csv
  metadata.json
```

## `functions.csv`

| Column | Required | Meaning |
| --- | --- | --- |
| `function` | yes | function name as seen by the backend |
| `source_file` | yes | absolute or repo-relative source path; may be empty |
| `line` | yes | 1-indexed source line; may be empty |
| `module` | yes | backend-specific module identifier (e.g. `common/page_alloc.ci`, `chain.ll`) |
| `backend` | yes | `gcc-ci` or `llvm-ir` |
| `linkage` | no | LLVM linkage (`internal`, `external`, ...) if known |
| `visibility` | no | LLVM visibility (`default`, `hidden`, ...) if known |
| `notes` | no | adapter-specific note (e.g. `edge-endpoint-only` from the GCC adapter, `intrinsic` from the LLVM adapter) |

Rows are deduplicated by `(function, source_file, line)` and
sorted for stable output.

**`function` alone is not a unique key.** A single function may
legitimately appear on multiple rows, distinguished by
`source_file` / `line`:

- A located declaration from the backend (e.g. a GCC `node:`
  with `-fcallgraph-info=su` filling in file/line) and a
  separately-synthesised edge-endpoint row (file/line empty,
  `notes=edge-endpoint-only`) coexist when the function is
  also referenced by an edge whose endpoint the backend did
  not emit a `node:` for. Both rows are kept; only exact
  triple duplicates collapse.
- A static function with the same name in two translation
  units (a common Xen pattern in arch-specific code) appears
  once per unit: each row carries its own `source_file` and
  `module`. Reachability queries operate on `function` name
  and so collapse these symbols intentionally; the multiple
  rows are kept so the row count, declaration site, and
  `notes` per origin are preserved.
- LLVM and GCC backends running over the same build may both
  emit a row for the same function with different `backend`,
  `module`, or `notes`. The dedup tuple does **not** include
  `backend`, so two adapters writing into the same directory
  produce one row when the `(function, source_file, line)`
  triple matches; otherwise both are kept.

Consumers that need a unique-by-name table (e.g. for display
in a status report) should dedup on `function` after loading,
accepting that they will collapse the cases above. Consumers
that need per-declaration-site data should keep all rows.

## `edges.csv`

| Column | Required | Meaning |
| --- | --- | --- |
| `caller` | yes | function whose body contains the call |
| `callee` | yes | callee function name; empty for `unresolved_indirect` |
| `source_file` | no | call-site source path |
| `line` | no | call-site source line |
| `edge_kind` | yes | one of `direct`, `unresolved_indirect`, `synthetic` |
| `backend` | yes | `gcc-ci` or `llvm-ir` |
| `module` | no | translation unit / file identifier |
| `notes` | no | adapter-specific note (e.g. `alias`, `intrinsic`, `bitcast-wrapper`, `target=%fp`) |

`edge_kind` values:

- **`direct`**--  caller statically calls a named callee.
  Default kind for both backends.
- **`unresolved_indirect`**--  caller dispatches through a
  function pointer / virtual method that the backend could
  not resolve. `callee` is empty. The reachability library
  ignores these edges for default reachability queries; a
  downstream pass that has resolved a function pointer
  (for example, table-aware binding in
  `indirect_reachability.py`) should emit `synthetic` edges
  with the resolved callee.
- **`synthetic`**--  added by an analyst pass or augmentation
  script. Counts toward reachability alongside `direct`.

Rows are deduplicated by
`(caller, callee, edge_kind, source_file, line)`.

## `metadata.json`

```json
{
  "backend": "gcc-ci" | "llvm-ir",
  "toolchain": "...",
  "compiler_version": "...",
  "source_git_sha": "...",
  "config_name": "...",
  "target_arch": "...",
  "inputs": ["..."],
  "warnings": ["..."],
  "generated_at": "2026-..."
}
```

The `backend` field is required; everything else is optional
but recommended for reproducibility. `warnings` records
adapter-side issues (e.g., bitcode files that couldn't be
disassembled).

## Library API

`scripts/callgraph/normalized_graph.py` provides:

```python
load_functions(path)              # list of dicts
load_edges(path)                  # list of dicts
write_functions(out_dir, rows)    # dedup + sort + write CSV
write_edges(out_dir, rows)
write_metadata(out_dir, **fields) # requires backend kwarg

build_reverse_index(edges, kinds=("direct", "synthetic"))
    # -> dict[callee, set[caller]]

functions_reaching_target(edges, target,
                          kinds=("direct", "synthetic"))
    # -> set[function name]; reverse-callgraph BFS, cycle-safe

direct_call_tree_to_target(edges, target, kinds=...,
                           max_depth=64)
    # -> list of paths from leaf-callers to target
```

`functions_reaching_target` is the workhorse for target-tree
membership reachability. It walks the reverse callgraph from
`target` via the cached reverse index, with a visited set for
cycle safety.

## Adapter conventions

Both adapters emit:

- one `direct` edge per call instruction (deduped at write
  time);
- per-function rows from explicit function declarations and
  from edge endpoints that lack an explicit declaration;
- `backend = "gcc-ci"` or `backend = "llvm-ir"` on every row.

### GCC-specific

- Nodes whose `label:` includes the stack-usage suffix
  produced by `-fcallgraph-info=su` get `source_file` and
  `line` populated from the label.
- Nodes whose `label:` lacks the stack-usage suffix (built
  with `-fcallgraph-info` alone) get `source_file=""`,
  `line=""`, and `notes="edge-endpoint-only"`. A reachability
  query through the normalized graph in this case returns a
  superset of what the text-tree adapter
  (`callpath.py to`) reports for the same `.ci` tree, because
  edge-endpoint synthesis preserves callers the text adapter
  drops.

### LLVM-specific

- `@llvm.*` intrinsic edges are tagged
  `module="llvm-intrinsic"`, `notes="intrinsic"`.
- Aliases (`@a = alias ... @b`) emit a `direct` edge with
  `notes="alias"`.
- `bitcast`-wrapper constant expressions
  (`call void bitcast (... @callee ...)(...)`) emit a
  `direct` edge with `notes="bitcast-wrapper"`.
- `bitcast`-wrapper around an SSA value
  (`bitcast (... %fp ...)(...)`) stays
  `unresolved_indirect` with
  `notes="bitcast-wrapper; target=%<ref>"`.
- `, !dbg !N` metadata is followed
  (`!DILocation`  ->  `!DISubprogram`  ->  `!DIFile`) when
  present; missing metadata leaves `source_file` / `line`
  empty.
