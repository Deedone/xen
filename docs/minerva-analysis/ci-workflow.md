# CI workflow

The Minerva indirect-call analysis runs in CI through the
driver described below, which orchestrates the full pipeline
against a freshly built Xen `.config` and a freshly generated
`.ci` tree.

The current pipeline separates the work into independent
producers and a single consumer (see "Corpus producer/consumer
workflow" at the end of this document):

- `minerva-static-analysis` runs the static stages once per
  config/backend/build and emits `static-analysis/`;
- the per-test `minerva-qemu-xtf-*` jobs each run a runtime
  workload and emit `runtime-artifacts/<test>/`, with no static
  analysis;
- `minerva-allocation-assurance-corpus` joins the two by
  manifest identity and emits `corpus-analysis/`.

The earlier combined `minerva-indirect-reachability` job, which
ran the static stages and an optional same-job runtime in one
job, has been retired in favour of these independent producers.
The driver mechanics below still describe what the static
producer runs.

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
| `MINERVA_CALLGRAPH_BACKEND` | `gcc-ci` (default), `llvm-ir`, or `normalized`. See [callgraph-backends.md](callgraph-backends.md). |
| `MINERVA_GENERATE_LLVM_IR` | If truthy *and* the backend is `llvm-ir`, generate IR from this run's build (analysis compile replay) instead of requiring `LLVM_IR_DIR`. Default `false`. |
| `LLVM_CC` | Clang executable used for IR generation. Default `clang`. |
| `LLVM_IR_DIR` | Externally supplied `.ll` / `.bc` tree for the `llvm-ir` backend. Ignored when `MINERVA_GENERATE_LLVM_IR` is truthy. |
| `LLVM_IR_EXTRA_CFLAGS` | Extra flags appended to each IR replay compile (whitespace-separated). |
| `MINERVA_LLVM_IR_CLEAN_BEFORE_CAPTURE` | If truthy, run `make clean` before the verbose capture build so a reused workspace still logs every C compile. |
| `MINERVA_RUNTIME_COMMAND` | Option A: command run after the static stages to produce runtime logs. Empty default keeps the job `STATIC_ONLY`. |
| `MINERVA_RUNTIME_LOG_DIR` | Where the runtime command writes (or where external logs already sit). Default under the artifact dir. |
| `MINERVA_RUNTIME_REQUIRED` | If truthy, a runtime failure makes the run `PARTIAL` instead of a `STATIC_ONLY` downgrade. |
| `MINERVA_RUNTIME_TIMEOUT` | Timeout in seconds for the runtime command. Default 600. |
| `MINERVA_ALLOW_PARTIAL_LLVM_IR` | If truthy, tolerate some-but-not-all IR replay failures instead of `PARTIAL`. |

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

## Enabling LLVM IR generation

To generate LLVM IR in CI instead of building GCC `.ci` files,
set the backend to `llvm-ir` and turn on generation:

```
MINERVA_CALLGRAPH_BACKEND=llvm-ir
MINERVA_GENERATE_LLVM_IR=true
LLVM_CC=clang
```

Each run still generates a fresh expanded `.config` exactly as
in the GCC path; the IR tree is generated from that same
configuration by analysis compile replay (see
[callgraph-backends.md](callgraph-backends.md)). The generated
IR tree is normalized by `llvm_ir_to_normalized.py` and
archived under `indirect-reachability/llvm-ir/`; the normalized
graph is archived under `indirect-reachability/normalized/`.
Both are per-run artifacts and are not committed.

GCC `.ci` remains the default backend. The metric policy below
applies unchanged: nothing about the IR path is a committed
constant.

## Aligned runtime/static comparison (Option A)

To collect runtime allocation logs and compare them against the
static artifacts in the **same job**, set a runtime command:

```
MINERVA_RUNTIME_COMMAND=./automation/run-minerva-runtime-smoke.sh
MINERVA_RUNTIME_LOG_DIR=indirect-reachability/runtime/logs
MINERVA_RUNTIME_REQUIRED=false
MINERVA_RUNTIME_TIMEOUT=600
```

Option A is the preferred alignment mode: the runtime logs come
from the same job, the same expanded `.config`, and the same
git SHA as the static analysis, so no cross-job provenance
guessing is needed. The driver runs the command only after the
static stages succeed, exports the `MINERVA_*` environment
(log dir, config path/name, git SHA, target arch, defconfig),
parses the resulting logs, writes `runtime/runtime-manifest.json`,
and runs `scripts/runtime_static_compare.py`.

The runtime command is optional. With it unset the run stays
`STATIC_ONLY` -- the default success state. With it set and the
comparison succeeding against an aligned manifest, the run is
`COMPLETE`. `COMPLETE` is tool-checked: the driver verifies the
manifest's git SHA and config hash against the job before
applying the label (see
[runtime-static-status.md](runtime-static-status.md)).

Runtime non-observation of a static candidate does not mean the
path is impossible, and static reachability does not mean a path
was exercised at runtime. All comparison metrics are per-run
artifacts.

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

## Corpus producer/consumer workflow

The corpus assurance analysis (see
[corpus-assurance.md](corpus-assurance.md) when present, and
`analyze_reachability_corpus.py`) consumes SEPARATED
static-analysis and runtime artifacts. Three roles make up the
pipeline:

| Role | Job(s) | Emits | needs |
| --- | --- | --- | --- |
| Static producer | `minerva-static-analysis` | `static-analysis/` | none (independent) |
| Runtime producers | `minerva-qemu-xtf-*` (16) | `runtime-artifacts/<test>/` | `xen-atfe-arm64-minerva` (the LLVM-based build) |
| Corpus consumer | `minerva-allocation-assurance-corpus` | `corpus-analysis/` | static + the xtf test jobs (all `optional: true`) |

The static job and the runtime test jobs are fully independent:
neither needs the other, and a runtime test never triggers or
repeats static analysis. Each xtf test job extends the pure-runtime template
(`.minerva-arm64`) that runs the workload and packages the console
log into `runtime-artifacts/<test>/` via `package_artifacts.sh` -- it
runs no static stages.

Only the corpus job joins static and runtime, matching by manifest
identity (`git_sha` / `target_arch` / `config_sha256`). The one
value both sides must compute identically -- `config_sha256` -- is
derived on each side from the expanded `.config` content, so two
independent jobs agree without either depending on the other.

An empty corpus never supports a bounded assurance claim: if there
are no static artifacts, no runtime artifacts, no matched config
groups, or every discovered artifact was quarantined (for example a
runtime manifest missing `config_sha256`), the corpus verdict is
forced to `NOT_SUPPORTED` with an explicit blocker. Absence of
evidence is not evidence of support.

### Booting the instrumented build and the shared config

Each `minerva-qemu-xtf-*` job depends on `xen-atfe-arm64-minerva`
only -- not the plain `xen-atfe-arm64`. Both builds export
`binaries/xen`; depending on both would let the uninstrumented binary
overwrite the instrumented one, and the booted Xen would emit no
`CONFIG_MINERVA_ANALYSIS` allocation traces (the runtime artifact would
parse to an empty corpus).

The join key `config_sha256` has a single source of truth: the expanded
config the instrumented build produces and exports as `xen-config`. The
runtime job derives its hash from that file (it arrives via the build
dependency), and `minerva-static-analysis` consumes the same
`xen-config` (via `MINERVA_CONFIG_INPUT`) instead of expanding its own
config. Because both sides hash the identical file, their
`config_sha256` match by construction -- there is no second config
expansion to keep byte-aligned. The static job still builds its own
callgraph from that config; only the config expansion is shared.

The corpus `needs:` are marked `optional: true`: the corpus depends on
each producer only if it exists and ran in this pipeline. The xtf jobs
are `when: manual`, so an operator can run any subset and still get a
corpus verdict over what ran.

The smoke tests are intentionally not part of the runtime corpus:
`smoke.yaml` is included only under `$RUN_UBSAN` (see `test.yaml`), so
its jobs are absent from normal pipelines -- an unconditional `needs:`
on them fails pipeline creation with "undefined need" -- and
UBSAN-instrumented runtime behaviour is not representative of production
allocation patterns. The unconditionally-included xtf tests are the
runtime corpus.
