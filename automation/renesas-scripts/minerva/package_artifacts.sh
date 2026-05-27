#!/usr/bin/env bash
# Package a Minerva analysis run into the SEPARATED Series 10 artifact
# contract: a static-analysis artifact tree and a runtime artifact
# tree, each with its own manifest. This lets the producer side emit
# what the corpus analyzer consumes without changing the existing
# driver: the driver still writes its combined indirect-reachability/
# output, and this wrapper repackages it.
#
# Two modes (the producer jobs use one each):
#
#   static    -- emit static-analysis/<...>/ (Part 1 contract) from the
#                static stages of an indirect-reachability/ run.
#   runtime   -- emit runtime-artifacts/<test-name>/ (Part 2 contract)
#                from the runtime side of an indirect-reachability/ run.
#
# Usage:
#   package_artifacts.sh static  <indirect-out> <static-analysis-root>
#   package_artifacts.sh runtime <indirect-out> <runtime-artifacts-root> <test-name>
#
# The static-analysis manifest is synthesized from status.json (which
# the driver already writes) so the corpus join key
# (git_sha/target_arch/config_sha256) is present and stable.

set -euo pipefail

mode="${1:?mode: static|runtime}"
src="${2:?source indirect-reachability dir}"
dst="${3:?destination root}"

if [[ ! -d "$src" ]]; then
  echo "package_artifacts: source '$src' not found" >&2
  exit 1
fi

status="$src/status.json"

# Extract a JSON string field from status.json without requiring jq.
_field() {
  python3 - "$status" "$1" <<'PY'
import json, sys
try:
    d = json.load(open(sys.argv[1]))
except Exception:
    d = {}
print(d.get(sys.argv[2], "") or "")
PY
}

# Identity for the manifest join key. A runtime test job is fully
# independent of the static job: it knows its own git_sha / config /
# arch and supplies them via the environment. The static job may
# instead let these come from the driver's status.json. Environment
# values always win when set.
git_sha="${MINERVA_GIT_SHA:-$(_field git_sha)}"
target_arch="${MINERVA_TARGET_ARCH:-$(_field target_arch)}"
config_name="${MINERVA_CONFIG_NAME:-$(_field config_name)}"
config_sha256="${MINERVA_CONFIG_SHA256:-$(_field config_sha256)}"
backend="${MINERVA_CALLGRAPH_BACKEND:-llvm-ir}"

# config_sha256 must match across the static and runtime manifests for
# the corpus join. Both sides derive it from the expanded .config when
# not supplied explicitly: the static job from its config/.config, the
# runtime job from the .config it booted (pointed to by
# MINERVA_CONFIG_FILE). This is the only value the two independent jobs
# must compute identically; deriving both from the same .config content
# guarantees they agree without either job depending on the other.
if [[ -z "$config_sha256" ]]; then
  if [[ -n "${MINERVA_CONFIG_FILE:-}" && -f "${MINERVA_CONFIG_FILE}" ]]; then
    config_sha256="$(sha256sum "${MINERVA_CONFIG_FILE}" | awk '{print $1}')"
  elif [[ -f "$src/config/.config" ]]; then
    config_sha256="$(sha256sum "$src/config/.config" | awk '{print $1}')"
  fi
fi

case "$mode" in
  static)
    root="$dst"
    mkdir -p "$root"
    # Copy the static side of the driver output into the Part 1 layout.
    [[ -f "$src/environment.md" ]] && cp "$src/environment.md" "$root/"
    [[ -f "$status" ]] && cp "$status" "$root/status.json"
    mkdir -p "$root/config"
    [[ -d "$src/config" ]] && cp -r "$src/config/." "$root/config/" 2>/dev/null || true
    # callgraph: the normalized/ tree (llvm-ir / normalized backend) or
    # ci-files for the gcc-ci backend.
    mkdir -p "$root/callgraph"
    if [[ -d "$src/normalized" ]]; then
      mkdir -p "$root/callgraph/normalized"
      cp -r "$src/normalized/." "$root/callgraph/normalized/" 2>/dev/null || true
    fi
    [[ -f "$src/ci-files.list" ]] && cp "$src/ci-files.list" "$root/callgraph/"
    printf '{"backend": "%s"}\n' "$backend" > "$root/callgraph/backend.json"
    for d in collect reachability direct-static; do
      if [[ -d "$src/$d" ]]; then
        mkdir -p "$root/$d"
        cp -r "$src/$d/." "$root/$d/" 2>/dev/null || true
      fi
    done
    # Synthesize the static-analysis manifest (Part 1 schema).
    python3 - "$root" "$git_sha" "$target_arch" "$config_name" \
             "$config_sha256" "$backend" <<'PY'
import hashlib, json, sys, datetime
root, git_sha, arch, cfg_name, cfg_sha, backend = sys.argv[1:7]
sid = "static-" + hashlib.sha256(
    "\x1f".join([git_sha, arch, cfg_sha, backend]).encode()
).hexdigest()[:16]
m = {
    "artifact_type": "static-analysis",
    "git_sha": git_sha,
    "target_arch": arch,
    "config_name": cfg_name,
    "config_sha256": cfg_sha,
    "callgraph_backend": backend,
    "static_artifact_id": sid,
    "generated_at": datetime.datetime.now(
        datetime.timezone.utc).isoformat(),
    "status": "STATIC_READY",
}
json.dump(m, open(root + "/static-analysis-manifest.json", "w"),
          indent=2, sort_keys=True)
print("static-analysis-manifest.json:", sid)
PY
    echo "packaged static-analysis tree at $root"
    ;;

  runtime)
    test_name="${4:?test name required for runtime mode}"
    root="$dst/$test_name"
    mkdir -p "$root/logs" "$root/parsed"
    # Locate raw logs. A pure runtime job points MINERVA_RUNTIME_LOG_DIR
    # at where its workload wrote logs; otherwise look under the source
    # in the driver's conventional locations.
    log_src="${MINERVA_RUNTIME_LOG_DIR:-}"
    if [[ -z "$log_src" ]]; then
      [[ -d "$src/runtime/logs" ]] && log_src="$src/runtime/logs"
      [[ -z "$log_src" && -d "$src/runtime" ]] && log_src="$src/runtime"
      [[ -z "$log_src" && -d "$src/logs" ]] && log_src="$src/logs"
    fi
    if [[ -n "$log_src" && -d "$log_src" ]]; then
      find "$log_src" -maxdepth 1 -name '*.log' -exec cp {} "$root/logs/" \; \
        2>/dev/null || true
    fi
    # Use existing parsed output if a previous step produced it;
    # otherwise parse the raw logs standalone (no static side needed).
    if [[ -d "$src/runtime/parsed" ]]; then
      cp -r "$src/runtime/parsed/." "$root/parsed/" 2>/dev/null || true
    fi
    if [[ -z "$(ls -A "$root/parsed" 2>/dev/null)" \
          && -n "$(ls -A "$root/logs" 2>/dev/null)" ]]; then
      here="$(cd "$(dirname "$0")" && pwd)"
      # The parser treats its positional arg as a directory of log
      # files; point it at logs/ (not the artifact root, which also
      # holds the manifest) and direct parsed output into parsed/.
      python3 "$here/log_parser.py" "$root/logs" \
        --parsed-output-dir "$root/parsed" >/dev/null 2>&1 || \
        echo "package_artifacts: log_parser produced no parsed output" >&2
    fi
    # Carry the driver's runtime-summary.json if present; otherwise
    # synthesize a minimal one so the runtime artifact tree is complete
    # (Part 2 layout) even for a pure runtime job that only has logs.
    if [[ -f "$src/runtime/runtime-summary.json" ]]; then
      cp "$src/runtime/runtime-summary.json" "$root/"
    elif [[ ! -f "$root/runtime-summary.json" ]]; then
      log_count="$(find "$root/logs" -maxdepth 1 -type f 2>/dev/null | wc -l)"
      parsed_count="$(find "$root/parsed" -type f 2>/dev/null | wc -l)"
      python3 - "$root" "$test_name" "$log_count" "$parsed_count" <<'PY'
import json, sys
root, test, logs, parsed = sys.argv[1:5]
json.dump({"test_name": test, "logs": int(logs),
           "parsed_files": int(parsed),
           "note": "synthesized by package_artifacts (runtime producer)"},
          open(root + "/runtime-summary.json", "w"),
          indent=2, sort_keys=True)
PY
    fi
    # Synthesize/normalize the runtime manifest (Part 2 schema),
    # preferring any manifest the driver already wrote.
    python3 - "$root" "$src" "$git_sha" "$target_arch" "$config_name" \
             "$config_sha256" "$test_name" <<'PY'
import hashlib, json, sys, datetime, os
root, src, git_sha, arch, cfg_name, cfg_sha, test = sys.argv[1:8]
existing = {}
for cand in (os.path.join(src, "runtime", "runtime-manifest.json"),
             os.path.join(src, "runtime", "parsed",
                          "runtime-manifest.json")):
    if os.path.exists(cand):
        try:
            existing = json.load(open(cand))
            break
        except Exception:
            pass
git_sha = existing.get("git_sha") or git_sha
arch = existing.get("target_arch") or arch
cfg_name = existing.get("config_name") or cfg_name
cfg_sha = existing.get("config_sha256") or cfg_sha
rid = "runtime-" + hashlib.sha256(
    "\x1f".join([git_sha, arch, cfg_sha, test]).encode()
).hexdigest()[:16]
m = {
    "artifact_type": "runtime",
    "git_sha": git_sha,
    "target_arch": arch,
    "config_name": cfg_name,
    "config_sha256": cfg_sha,
    "test_name": test,
    "job_name": os.environ.get("CI_JOB_NAME", test),
    "pipeline_id": os.environ.get("CI_PIPELINE_ID", ""),
    "runtime_artifact_id": rid,
    "generated_at": datetime.datetime.now(
        datetime.timezone.utc).isoformat(),
    "status": "RUNTIME_READY",
}
json.dump(m, open(root + "/runtime-manifest.json", "w"),
          indent=2, sort_keys=True)
print("runtime-manifest.json:", rid)
PY
    echo "packaged runtime artifact tree at $root"
    ;;

  *)
    echo "package_artifacts: unknown mode '$mode'" >&2
    exit 2
    ;;
esac
