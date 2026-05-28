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
: "${MINERVA_GENERATE_LLVM_IR:=false}"
: "${LLVM_CC:=clang}"

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

# Optional single-source config. When MINERVA_CONFIG_INPUT points at an
# externally produced expanded .config (e.g. the xen-config the
# instrumented build exports), pass it as --config: the driver consumes
# it verbatim instead of expanding its own, so the static and runtime
# sides derive an identical config_sha256 by construction. The callgraph
# is still built from this config (this is not --skip-build).
if [[ -n "${MINERVA_CONFIG_INPUT:-}" ]]; then
  args+=( --config "$MINERVA_CONFIG_INPUT" )
fi

# Optional `--extra CONFIG_FOO=y` symbols. The variable holds
# whitespace-separated entries; each becomes a separate
# `--extra` argument so Kconfig sees them as discrete requests.
# Ignored when MINERVA_CONFIG_INPUT supplies a complete config.
if [[ -z "${MINERVA_CONFIG_INPUT:-}" && -n "${XEN_EXTRA_CONFIG:-}" ]]; then
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

# LLVM IR generation (opt-in; analysis compile replay). Only acted on
# when the llvm-ir backend is selected and MINERVA_GENERATE_LLVM_IR is
# truthy. The driver itself ignores --generate-llvm-ir for other
# backends, but the wrapper only adds it for llvm-ir to keep the argv
# minimal and the intent obvious.
case "${MINERVA_GENERATE_LLVM_IR:-false}" in
  1 | true | TRUE | yes | YES)
    if [[ "$MINERVA_CALLGRAPH_BACKEND" == "llvm-ir" ]]; then
      args+=( --generate-llvm-ir )
      if [[ -n "${LLVM_CC:-}" ]]; then
        args+=( --llvm-cc "$LLVM_CC" )
      fi
    fi
    ;;
esac
if [[ -n "${LLVM_IR_EXTRA_CFLAGS:-}" ]]; then
  args+=( --llvm-ir-extra-cflags "$LLVM_IR_EXTRA_CFLAGS" )
fi
case "${MINERVA_LLVM_IR_CLEAN_BEFORE_CAPTURE:-false}" in
  1 | true | TRUE | yes | YES)
    args+=( --llvm-ir-clean-before-capture )
    ;;
esac
case "${MINERVA_ALLOW_PARTIAL_LLVM_IR:-false}" in
  1 | true | TRUE | yes | YES)
    args+=( --allow-partial-llvm-ir )
    ;;
esac

# Runtime workload hook (Option A: same-job runtime collection).
# If MINERVA_RUNTIME_COMMAND is set, the driver runs it after the
# static stages, parses the resulting logs, and compares them with
# the static artifacts. If neither a command nor a log dir is set,
# the run stays STATIC_ONLY -- the documented default success state.
if [[ -n "${MINERVA_RUNTIME_COMMAND:-}" ]]; then
  args+=( --runtime-command "$MINERVA_RUNTIME_COMMAND" )
  : "${MINERVA_RUNTIME_LOG_DIR:=${MINERVA_INDIRECT_OUT}/runtime/logs}"
  args+=( --runtime-log-dir "$MINERVA_RUNTIME_LOG_DIR" )
  : "${MINERVA_RUNTIME_TIMEOUT:=600}"
  args+=( --runtime-timeout "$MINERVA_RUNTIME_TIMEOUT" )
  case "${MINERVA_RUNTIME_REQUIRED:-false}" in
    1 | true | TRUE | yes | YES) args+=( --runtime-required ) ;;
  esac
elif [[ -n "${MINERVA_RUNTIME_LOG_DIR:-}" \
        && -d "${MINERVA_RUNTIME_LOG_DIR}" \
        && -n "$(ls -A "${MINERVA_RUNTIME_LOG_DIR}" 2>/dev/null)" ]]; then
  # External logs already present (and non-empty).
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
