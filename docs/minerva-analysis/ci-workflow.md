# CI workflow

The Minerva indirect-call analysis runs as a GitLab CI job
named `minerva-indirect-reachability`. The job script invokes
the driver, which orchestrates the full pipeline against a
freshly built Xen `.config` and a freshly generated `.ci`
tree.

## Driver entry point

`scripts/indirect_ci_driver.py` is the single entry point.
For each run it:

1. records host environment, toolchain versions, git SHA, and
   the effective CI variables in `environment.md`;
2. generates the expanded Xen `.config` from a defconfig and
   optional `CONFIG_*` overlays (in skip-build mode, consumes
   an externally supplied `.config`);
3. builds Xen with the configured callgraph flag (default
   `-fcallgraph-info=su`) and archives the resulting `.ci`
   tree (in skip-build mode, archives an externally supplied
   tree);
4. runs `scripts/collect.py` on the archived `.config`;
5. runs `scripts/indirect_reachability.py` on the collector
   output and the archived `.ci` tree;
6. runs `minerva_static_analysis/callpath.py` for each
   allocation target and emits per-target paths;
7. optionally runs `minerva_analysis/log_parser.py` against
   a supplied runtime log directory;
8. writes `status.json` and `ci-summary.md`.

Each run generates a **fresh** `.config` and `.ci` tree; the
CI job does not reuse local build artifacts.

## Variables

### Required

| Variable | Meaning |
| --- | --- |
| `XEN_TARGET_ARCH` | `arm64` or `x86_64`. Passed to `--target-arch`. |
| `XEN_DEFCONFIG` | Make target (`arm64_defconfig`, `arm64_safety_defconfig`, ...) or path to a defconfig file. Passed to `--defconfig`. |
| `CROSS_COMPILE` | Cross-toolchain prefix (e.g. `aarch64-linux-gnu-`). |
| `MINERVA_CONFIG_NAME` | Symbolic name used in collector output and `status.json`. |

### Optional

| Variable | Meaning |
| --- | --- |
| `XEN_EXTRA_CONFIG` | One or more `CONFIG_FOO=y` lines, whitespace-separated, appended before `make olddefconfig`. Each value becomes one `--extra` argument. **Note:** `olddefconfig` may drop symbols that are undefined or have unsatisfied `depends on`; the archived `config/.config` is the source of truth, not the request list. |
| `XEN_CALLGRAPH_FLAG` | GCC callgraph flag; default `-fcallgraph-info=su`. |
| `XEN_BUILD_JOBS` | Parallelism for `make`. Default `$(nproc)` in the GitLab job. |
| `MINERVA_RUNTIME_LOG_DIR` | If set, parsed via `log_parser.py`. Absence yields `STATIC_ONLY`. |
| `MINERVA_INDIRECT_OUT` | Artifact directory; default `indirect-reachability`. |
| `MINERVA_CI_ALLOW_RUNTIME_FAILURE` | If truthy, runtime parser failure downgrades the run to `STATIC_ONLY` instead of `PARTIAL`. |

### Smoke defaults

The defaults baked into `.gitlab-ci.yml` are intended for
smoke validation:

```
XEN_TARGET_ARCH=arm64
XEN_DEFCONFIG=arm64_defconfig
XEN_EXTRA_CONFIG=CONFIG_DEBUG_INFO=y
CROSS_COMPILE=aarch64-linux-gnu-
MINERVA_CONFIG_NAME=${XEN_DEFCONFIG}+debug-info
XEN_CALLGRAPH_FLAG=-fcallgraph-info=su
```

The target Xen configuration in production comes from
whichever variables the pipeline operator sets per pipeline,
schedule, branch, or trigger. The defaults above are not the
only supported configuration.

## Per-pipeline overrides

Operators override variables per pipeline using GitLab's "Run
pipeline" UI, schedules, or `trigger` rules. Example for a
safety defconfig:

```
XEN_DEFCONFIG=arm64_safety_defconfig
XEN_EXTRA_CONFIG=CONFIG_IOREQ_SERVER=y CONFIG_DEBUG_INFO=y
MINERVA_CONFIG_NAME=arm64_safety+ioreq+debuginfo
```

The driver records the effective values in
`indirect-reachability/environment.md`; comparing across
pipelines is the operator's responsibility, not a baked-in
constant.

## Skip-build mode

The driver supports a `--skip-build` mode that consumes an
already-built `.config` and `.ci` tree from externally
supplied paths. This is for standalone validation against
artifacts the operator has prepared by other means; it is
not intended for GitLab CI. The driver tags the artifact
tree as skip-build in `environment.md` and
`status.json::notes`.

## Per-run metric policy

The status label, the counters in `status.json`, and the
human-readable summaries are **per-run artifact values**.
They change with Xen source revisions, configuration changes,
compiler version, callgraph coverage, runtime workload
availability, and synthetic-edge filtering rules. The
pipeline is what produces these artifacts; no specific number
is committed.

See [artifact-contract.md](artifact-contract.md) for the
exact artifact layout and
[runtime-static-status.md](runtime-static-status.md) for the
status-label decision rules.
