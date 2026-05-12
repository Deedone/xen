# Runtime / static status labels

Each CI run writes a single status label to
`indirect-reachability/status.json`. The label drives whether
the operator can consume the run as evidence of coverage or
only as diagnostic data.

## Labels

### COMPLETE

All static inputs and the runtime parser output exist and
correspond to the same Xen configuration and source revision:

- expanded `.config`
- callgraph artifacts (`.ci` tree and/or normalized graph)
- collector output (`collect/`)
- reachability workbench output (`reachability/`)
- direct-static baselines (`direct-static/`)
- runtime parser output (`runtime/`)

Operator-visible behaviour:

- `status.json::status` = `"COMPLETE"`.
- `runtime/runtime-summary.json::ok` = `true`.
- The runtime-static-comparison artifact is the per-run
  summary for this configuration; it is not a committed
  coverage number.

**COMPLETE is operator-asserted.** The driver does not yet
automatically prove that the runtime logs came from the same
build, configuration, and source revision as the static
artifacts. Operators must supply aligned runtime logs or
relabel the run as `PROXY`. A future manifest check could
compare runtime log metadata against archived `.config`
symbols and the recorded git SHA; until that exists, the
operator owns the alignment claim.

### STATIC_ONLY

All static inputs exist and correspond to the same Xen build,
but no runtime logs are attached. **Valid success state.**

When the runner gets `STATIC_ONLY`:

- `status.json::status` = `"STATIC_ONLY"`.
- `runtime_ok` = `false`.
- `runtime_reason` is one of:
    - `"not supplied"`--  no `--runtime-log-dir`.
    - `"--no-runtime supplied"`--  operator opted out.
    - `"log_parser.py failed: ..."`--  runtime stage
      attempted but failed; only emitted when
      `MINERVA_CI_ALLOW_RUNTIME_FAILURE` is truthy.

### PROXY

One or more inputs are from a nearby but not identical
configuration. **Not** produced automatically by the driver;
reserved for cases where the operator explicitly supplies
runtime logs from a different configuration and an analyst
downgrades the status by hand.

### PARTIAL

Required static-analysis inputs are missing or a stage
failed:

- expanded `.config` not produced (build failure in build
  mode, or `--config` missing/empty in skip-build mode);
- `.ci` tree is empty;
- collector failed or did not emit `collection-summary.json`;
- reachability workbench failed or did not emit
  `reachability-summary.md`;
- direct-static baseline failed for one or more targets in a
  way that prevented artifact generation.

`PARTIAL` preserves whatever artifacts were produced before
the failing stage. The driver exits non-zero (2). The GitLab
job keeps artifacts via `when: always`.

### UNSUPPORTED_BACKEND

The requested callgraph backend is unavailable for this run.
Examples:

- `MINERVA_CALLGRAPH_BACKEND=llvm-ir` with no `LLVM_IR_DIR`.
- `MINERVA_CALLGRAPH_BACKEND=normalized` with no
  `NORMALIZED_CALLGRAPH_DIR`.

The driver exits non-zero (2). Artifacts produced before the
backend check are preserved.

## Decision rule

```
if static stack OK
  if runtime requested AND runtime OK        -> COMPLETE
  if runtime requested AND runtime failed
     AND MINERVA_CI_ALLOW_RUNTIME_FAILURE    -> STATIC_ONLY
  if runtime requested AND runtime failed
     AND not allow_runtime_failure           -> PARTIAL
  if runtime not requested                   -> STATIC_ONLY
else if backend unavailable                  -> UNSUPPORTED_BACKEND
else                                         -> PARTIAL
```

## Per-run metric policy

The status label, the counters in `status.json`, and the
human-readable summaries are **per-run artifact values**.
They are **not** committed constants. They change with:

- Xen source revisions
- configuration changes (`XEN_DEFCONFIG`, `XEN_EXTRA_CONFIG`)
- compiler version
- callgraph coverage (which artifacts were generated)
- callgraph backend selection (`gcc-ci`, `llvm-ir`,
  `normalized`)
- runtime workload availability
- synthetic-edge filtering rules

The pipeline is what produces these artifacts; no specific
number is committed.

## What this workflow does not assert

- No allocation bound. `path_found=yes` is a static-graph
  reachability statement only.
- `path_found=no` does not mean impossible. The function may
  be reachable via macros, function pointers not modelled by
  the callgraph, or excluded paths.
- "Not observed at runtime" does not mean impossible.
  Runtime parser output reflects a specific workload run,
  not all possible workloads.
- Runtime-registration rows are not expanded into per-field
  reachability queries; they remain an analyst worklist
  surfaced in `candidate-binding-summary.md`.
- Field-name-only candidate rows are kept in
  `indirect-allocation-paths.csv` for diagnostics but
  excluded from `synthetic_edges.candidates.yaml` and from
  the included edge count.
