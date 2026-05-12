# Callgraph backends

The reachability workbench accepts a callgraph in any of three
forms. The selected backend is recorded in
`status.json::callgraph_backend` for each CI run.

## Supported backends

| Backend | Inputs | Output kind | Status |
| --- | --- | --- | --- |
| `gcc-ci` | GCC `.ci` tree from a build with `-fcallgraph-info=su` | nested-tree (`.paths`) and normalized (`functions.csv` + `edges.csv` + `metadata.json`) | Default. |
| `llvm-ir` | Textual LLVM IR (`.ll`) or bitcode (`.bc`) via `llvm-dis` | normalized only | Supported via IR normalization. |
| `normalized` | Prebuilt normalized graph directory | normalized only | Supported. |

GCC `.ci` is the default; this tooling has been used against
real Xen builds with that path for the longest. The LLVM and
normalized backends are recent additions and share the
normalized graph format for their reachability output. The
GCC text-tree format (`callpath.py to`) is preserved
unchanged when the `gcc-ci` backend is used.

## Selection

`scripts/indirect_ci_driver.py` chooses the backend via
`--callgraph-backend {gcc-ci|llvm-ir|normalized}`. In CI the
selection is driven by `MINERVA_CALLGRAPH_BACKEND`. See
[ci-workflow.md](ci-workflow.md).

| Backend mode | Driver behaviour |
| --- | --- |
| `gcc-ci` | Build (or consume) `.ci` tree  ->  archive `.ci` and emit normalized form alongside  ->  call `indirect_reachability.py --callgraph-backend gcc-ci`  ->  run direct-static via `callpath.py`. |
| `llvm-ir` | Consume `--llvm-ir-dir` (`.ll` / `.bc`)  ->  run `llvm_ir_to_normalized.py`  ->  call `indirect_reachability.py --callgraph-backend normalized`  ->  run direct-static via normalized-graph target-tree membership. |
| `normalized` | Consume `--normalized-callgraph-dir` (pre-existing graph)  ->  skip extraction  ->  call `indirect_reachability.py --callgraph-backend normalized`  ->  run direct-static via normalized-graph target-tree membership. |

## GCC `.ci` backend

GCC emits `.ci` files when compiled with
`-fcallgraph-info=su`. The `=su` modifier adds stack-usage
information; the script reads file/line per node label from
that data. Without `=su` (a plain `-fcallgraph-info`), node
labels omit the stack-usage block; the adapter recovers
function rows from edge endpoints and tags them
`notes=edge-endpoint-only`. Reachability queries through the
normalized graph then return a **superset** of what
`callpath.py`'s printed text reports for the same `.ci` tree,
because edge-endpoint synthesis preserves callers that the
plain-text adapter drops.

The adapter is `scripts/callgraph/gcc_ci_to_normalized.py`.

## LLVM IR backend

`scripts/callgraph/llvm_ir_to_normalized.py` parses textual
LLVM IR (`.ll`) directly. **No LLVM plugin is required and no
DOT output is consumed.** The textual surface is small enough
to walk with regexes:

- `define ... @name(args)` for function definitions (also
  `@"name"` quoted form);
- `call` / `invoke` / `tail call` / `musttail call` /
  `notail call` with an `@callee` operand for direct edges;
- `call void bitcast (... @callee ...)(...)` constant-
  expression wrapper, resolved to a direct edge with
  `notes=bitcast-wrapper`;
- `call ... %ref(...)` for indirect edges, emitted with
  `callee=""` and `edge_kind=unresolved_indirect`;
- bitcast wrappers around SSA values (`bitcast (... %fp
  ...)(...)`) are recognised as indirect with
  `notes=bitcast-wrapper; target=%<ref>`;
- `@llvm.*` intrinsic edges tagged
  `module=llvm-intrinsic`, `notes=intrinsic`;
- `@a = alias ... @b` produces a `direct` edge `a -> b`
  with `notes=alias`;
- `, !dbg !N` metadata, followed
  (`!DILocation`  ->  `!DISubprogram`  ->  `!DIFile`) when
  present.

Bitcode `.bc` inputs are converted to `.ll` via `llvm-dis`
(located on PATH or via `--llvm-dis`) into a temporary file.
Inputs without `llvm-dis` available are skipped with a
warning recorded in `metadata.json::warnings`.

### LLVM-specific limits

- Not a complete IR semantic analyser. The extractor walks
  instructions line by line; it does not model SSA def-use,
  virtual-table layout, or function-pointer flow. Unresolved
  indirect calls stay unresolved.
- Not a coverage-number generator. Reachability over the
  resulting graph is static reachability only.
- Not a replacement for `callpath.py`. `callpath.py` remains
  the GCC-specific consumer for callers who want to keep
  that path; the LLVM extractor is the IR-side bridge into
  the normalized form.
- **Full Xen LLVM build validation is future work.** The
  extractor has been exercised on hand-crafted synthetic IR
  that covers every instruction shape it cares about
  (direct, indirect, bitcast wrappers, intrinsics, aliases,
  invoke, tail-call). Validation against a complete Xen
  build with `clang -S -emit-llvm` on a Linux runner is the
  next step; the extractor itself should not need code
  changes for that.

## Normalized backend

`MINERVA_CALLGRAPH_BACKEND=normalized` lets the operator
supply an already-prepared normalized graph directory and
skip extraction entirely. Useful for:

- replaying analysis over an archived run;
- consuming a graph produced by a non-default backend
  pipeline (e.g. a one-off `clang -S -emit-llvm` build whose
  output is not appropriate to regenerate in CI);
- decoupling the reachability run from the build.

See [normalized-callgraph-format.md](normalized-callgraph-format.md)
for the on-disk schema.

## Backend agreement and disagreement

On the same Xen build, the GCC and LLVM backends are expected
to agree on the direct-call subgraph. Documented sources of
disagreement:

- **GCC `-fcallgraph-info` without `=su`** leaves
  `callpath.py`'s node names empty for most functions; the
  GCC adapter recovers them via edge-endpoint synthesis. A
  reachability query through the normalized graph therefore
  returns a superset of what `callpath.py`'s printed text
  reports.
- **LLVM indirect calls** become `unresolved_indirect`
  edges that the default reachability query ignores. GCC
  doesn't model the same instruction class identically;
  some indirect calls in GCC's graph become `direct` edges
  to a guessed target instead. Comparing direct-only edge
  counts between the two backends is apples-to-oranges;
  consumers should filter to a target set, not equate raw
  edge totals.

## Verification

```
python3 -m py_compile scripts/callgraph/normalized_graph.py
python3 -m py_compile scripts/callgraph/gcc_ci_to_normalized.py
python3 -m py_compile scripts/callgraph/llvm_ir_to_normalized.py
```

Each adapter accepts `--ir-dir` / `--ci-dir`, `--out-dir`, and
the metadata flags `--config-name`, `--compiler-version`,
`--target-arch`, `--source-git-sha`. See each script's
`--help` for the full flag set.
