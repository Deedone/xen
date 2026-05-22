#!/usr/bin/env bash
# Minerva indirect-reachability CI wrapper.
#
# Translates the CI environment variables documented in
# `docs/minerva-analysis/ci-workflow.md` into an argv array for
# `scripts/indirect_ci_driver.py`. The
# wrapper exists so the GitLab `script:` block has no string
# assembly and no `eval`: every CLI argument crosses the
# wrapper boundary as a separate array element and is forwarded
# to the driver via `"${args[@]}"`. Values containing spaces or
# shell metacharacters survive intact.
#
# All variables are optional. The wrapper falls back to the same
# defaults the GitLab job documents. Operators may also invoke
# the wrapper directly outside CI for parity with the job
# script.

set -euo pipefail

: "${MINERVA_INDIRECT_OUT:=indirect-reachability}"
: "${XEN_TARGET_ARCH:=arm64}"
: "${XEN_DEFCONFIG:=arm64_defconfig}"
: "${CROSS_COMPILE:=aarch64-linux-gnu-}"
: "${MINERVA_CONFIG_NAME:=${XEN_DEFCONFIG}+debug-info}"
: "${XEN_CALLGRAPH_FLAG:=-fcallgraph-info=su}"
: "${MINERVA_CALLGRAPH_BACKEND:=gcc-ci}"
: "${XEN_BUILD_JOBS:=$(nproc 2>/dev/null || echo 2)}"

args=(
  --xen-root .
  --out-dir "$MINERVA_INDIRECT_OUT"
  --config-name "$MINERVA_CONFIG_NAME"
  --target-arch "$XEN_TARGET_ARCH"
  --cross-compile "$CROSS_COMPILE"
  --defconfig "$XEN_DEFCONFIG"
  # `--callgraph-flag` is attached with `=` because its value
  # (default `-fcallgraph-info=su`) starts with `-`, which
  # argparse otherwise treats as another option. Using the
  # equals form keeps the option name and its value in a
  # single argv element so argparse parses it cleanly.
  "--callgraph-flag=$XEN_CALLGRAPH_FLAG"
  --callgraph-backend "$MINERVA_CALLGRAPH_BACKEND"
  --jobs "$XEN_BUILD_JOBS"
)

# Optional `--extra CONFIG_FOO=y` symbols. The variable holds
# whitespace-separated entries; each becomes a separate
# `--extra` argument so Kconfig sees them as discrete requests.
if [[ -n "${XEN_EXTRA_CONFIG:-}" ]]; then
  # shellcheck disable=SC2206
  extras=( $XEN_EXTRA_CONFIG )
  for e in "${extras[@]}"; do
    args+=( --extra "$e" )
  done
fi

# Backend-specific inputs.
if [[ -n "${LLVM_IR_DIR:-}" ]]; then
  args+=( --llvm-ir-dir "$LLVM_IR_DIR" )
fi
if [[ -n "${LLVM_DIS:-}" ]]; then
  args+=( --llvm-dis "$LLVM_DIS" )
fi
if [[ -n "${NORMALIZED_CALLGRAPH_DIR:-}" ]]; then
  args+=( --normalized-callgraph-dir "$NORMALIZED_CALLGRAPH_DIR" )
fi

# Runtime parser hook. Default (no log dir) yields STATIC_ONLY,
# the documented success state for the smoke job. The driver
# reads MINERVA_CI_ALLOW_RUNTIME_FAILURE directly from the
# environment, so no CLI translation is needed.
if [[ -n "${MINERVA_RUNTIME_LOG_DIR:-}" ]]; then
  args+=( --runtime-log-dir "$MINERVA_RUNTIME_LOG_DIR" )
else
  args+=( --no-runtime )
fi

# Forward any extra arguments the caller passed to the wrapper.
# The GitLab job does not pass any (the `script:` line is just
# `bash scripts/run_indirect_ci.sh`), but a developer running
# the wrapper locally can append `--skip-build`, `--status-only`,
# `--targets ...`, etc.; they land after the array, so argparse
# sees the wrapper's defaults first and the caller's overrides
# second.
exec python3 automation/renesas-scripts/minerva/indirect_ci_driver.py "${args[@]}" "$@"
