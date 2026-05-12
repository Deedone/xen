# Artifact contract

Each CI run of `minerva-indirect-reachability` produces a
single archive directory. The layout is fixed; consumers can
rely on it across runs.

## Layout

```
indirect-reachability/
  environment.md
  status.json
  ci-summary.md

  config/
    .config
    config-scope.md

  ci/
    ci-files.list
    ci-files.tar.zst                 (optional; tree or compressed)

  normalized-callgraph/              (when a callgraph backend
                                      produces or consumes one)
    functions.csv
    edges.csv
    metadata.json

  collect/
    config-scope.md
    ops-inventory.csv
    ops-resolution.csv
    indirect-call-sites.csv
    runtime-ops-registration-sites.csv
    runtime-ops-registration-summary.md
    collection-summary.json

  reachability/
    indirect-allocation-paths.csv
    synthetic_edges.candidates.yaml
    synthetic_edges.excluded.yaml
    reachability-summary.md
    candidate-binding-summary.md

  direct-static/
    alloc_domheap_pages.paths        (gcc-ci backend)
    alloc_domheap_pages.functions    (normalized / llvm-ir backend)
    alloc_domheap_pages.stderr
    alloc_xenheap_pages.paths|.functions
    alloc_xenheap_pages.stderr
    _xmalloc.paths|.functions
    _xmalloc.stderr
    direct-static-summary.md
    direct-static-summary.json

  runtime/                            (optional)
    parsed/
    runtime-summary.md
    runtime-summary.json

  runtime-static/
    runtime-static-comparison.md
    runtime-static-comparison.json
```

`.paths` is the GCC `.ci`-style nested-tree format produced by
`callpath.py to`. `.functions` is the flat caller-set produced
by the normalized-graph backend (one function per line). The
backend in effect for a given run is recorded in
`status.json::callgraph_backend`.

## Invariants

For each CI run:

1. The `.config` archived under `config/.config` is the file
   the Xen build actually consumed (in build mode, the
   post-`olddefconfig` `xen/.config`; in skip-build mode, the
   supplied `--config <path>`).
2. The `.ci` files archived under `ci/` come from the **same
   Xen build** as the archived `.config`.
3. `scripts/collect.py` consumes the archived `.config`, not
   the defconfig fragment or env-file the operator supplied.
4. `scripts/indirect_reachability.py` consumes the collector
   output under `collect/` and the same `.ci` tree archived
   under `ci/`.
5. Direct-static summaries are produced from the same `.ci`
   tree.
6. Runtime artifacts under `runtime/` are optional. Absence
   yields `STATIC_ONLY`.

## Status labels

The top-level `status.json` records one of these labels.

- **COMPLETE**--  expanded `.config`, `.ci` tree, collector
  output, reachability output, direct-static output, and
  runtime parsed output all correspond to the same Xen
  configuration and source revision.
- **STATIC_ONLY**--  all static inputs exist for the same Xen
  build, but no runtime logs are attached. Valid success
  state.
- **PROXY**--  one or more inputs are from a nearby but not
  identical configuration. Reserved for cases where someone
  explicitly supplies mismatched runtime logs; the driver
  does not flip COMPLETE to PROXY on its own.
- **PARTIAL**--  required static-analysis inputs are missing
  or a stage failed.
- **UNSUPPORTED_BACKEND**--  the requested callgraph backend
  is unavailable in this run (for example,
  `MINERVA_CALLGRAPH_BACKEND=llvm-ir` with no `LLVM_IR_DIR`).

### Decision table

| Static stack | Runtime logs | Runtime parser | Status |
| --- | --- | --- | --- |
| OK | absent | n/a | **STATIC_ONLY** |
| OK | present + aligned | OK | **COMPLETE** |
| OK | present + mismatch | OK or fail | **PROXY** (operator-set) |
| failure of any static input | any | any | **PARTIAL** |
| backend not available | any | any | **UNSUPPORTED_BACKEND** |

CI archives whatever artifacts exist even when the overall
status is `PARTIAL` or `UNSUPPORTED_BACKEND` so the operator
can diagnose.

## Metric policy

The counters in `status.json` and the human-readable
summaries under each subdirectory are **per-run artifact
values**. Every CI run regenerates them. They are expected to
change with Xen source revisions, configuration changes,
compiler version, callgraph coverage, runtime workload
availability, and synthetic-edge filtering rules.

## What the contract does not assert

- No allocation bound. `path_found=yes` is a static-graph
  reachability statement only.
- No "not runtime-observed means impossible" claim.
- No final runtime-vs-static coverage number.
- Runtime-registration rows remain a separate analyst
  worklist. They are inventoried by `collect.py` but not
  expanded into per-field reachability queries by
  `indirect_reachability.py`.
- Field-name-only candidate rows remain in
  `indirect-allocation-paths.csv` for analyst diagnostics
  but are excluded from `synthetic_edges.candidates.yaml`
  and from `candidate_synthetic_edges_included` counts.
