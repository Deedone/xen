#!/usr/bin/env python3
"""Minerva indirect-reachability CI driver.

Orchestrates one CI run of the indirect-call reachability workflow:

  1. produce or consume the expanded Xen `.config`;
  2. produce or consume the callgraph artifacts (GCC `.ci` tree
     and/or normalized graph; optionally LLVM IR);
  3. run scripts/collect.py on the archived `.config`;
  4. run scripts/indirect_reachability.py on the collector output
     and the callgraph artifacts;
  5. run direct-static baselines via
     minerva_static_analysis/callpath.py (gcc-ci backend) or
     normalized-graph target-tree membership (normalized/llvm-ir);
  6. optionally run minerva_analysis/log_parser.py against a
     runtime log directory;
  7. emit a single archive directory matching the per-run artifact
     contract and a top-level status.json.

This script is *configuration-parameterised*. Target arch, defconfig,
extra symbols, cross-compile prefix, and config name are all driver
arguments (typically supplied from CI environment variables). The
driver does not assume one specific Xen configuration.

Status labels: COMPLETE / STATIC_ONLY / PROXY / PARTIAL /
UNSUPPORTED_BACKEND. No final runtime-vs-static coverage number is
produced or claimed; counters in status.json are per-run artifact
values.
"""

from __future__ import annotations

import argparse
import json
import os
import platform
import re
import shutil
import subprocess
import sys
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path


def _ts() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _run(cmd: list[str], cwd: Path | None = None, check: bool = True,
         timeout: int | None = None, env: dict | None = None
         ) -> subprocess.CompletedProcess:
    """Run a subprocess, raising on failure unless check=False.

    Output is captured; the caller decides what to record.
    """
    print(f"+ {' '.join(str(c) for c in cmd)}", file=sys.stderr)
    return subprocess.run(cmd, cwd=str(cwd) if cwd else None,
                          capture_output=True, text=True, check=check,
                          timeout=timeout, env=env)


def _safe(cmd: list[str], cwd: Path | None = None, timeout: int = 60
          ) -> str:
    try:
        return _run(cmd, cwd=cwd, check=False,
                    timeout=timeout).stdout.strip()
    except Exception as e:
        return f"<{e.__class__.__name__}: {e}>"


def write_environment_md(out_dir: Path, args: argparse.Namespace,
                         xen_root: Path):
    p = out_dir / "environment.md"
    lines = [
        "# Indirect-reachability CI run environment",
        "",
        f"timestamp: {_ts()}",
        f"host:      {platform.platform()}",
        f"python:    {sys.version.splitlines()[0]}",
        f"make:      "
        f"{_safe(['make', '--version']).splitlines()[0] if shutil.which('make') else '(no make)'}",
    ]
    cross = args.cross_compile or os.environ.get("CROSS_COMPILE", "")
    if cross:
        cc = f"{cross}gcc"
        lines.append(f"cross gcc: "
                     f"{_safe([cc, '--version']).splitlines()[0] if shutil.which(cc) else '(no ' + cc + ')'}")
    host_cc = "gcc" if shutil.which("gcc") else ""
    if host_cc:
        lines.append(f"host gcc:  "
                     f"{_safe([host_cc, '--version']).splitlines()[0]}")
    git_sha = _safe(["git", "-C", str(xen_root), "rev-parse", "HEAD"])
    git_status = _safe(["git", "-C", str(xen_root), "status", "--short"])
    lines += [
        f"git SHA:   {git_sha}",
        f"git status (short):",
        "```",
        git_status if git_status else "(clean)",
        "```",
        "",
        "## Driver invocation",
        "",
        f"target arch:    `{args.target_arch}`",
        f"defconfig:      `{args.defconfig or '(skip-build; supplied --config)'}`",
        f"cross compile:  `{cross or '(none)'}`",
        f"config name:    `{args.config_name}`",
        f"extras:         {', '.join(args.extra) if args.extra else '(none)'}",
        f"callgraph flag: `{args.callgraph_flag}`",
        f"jobs:           {args.jobs} "
        f"(host cpu_count={os.cpu_count() or 'unknown'})",
        f"mode:           {'skip-build (local validation only)' if args.skip_build else 'build'}",
        f"runtime:        "
        f"{'skipped' if args.no_runtime or not args.runtime_log_dir else str(args.runtime_log_dir)}",
        "",
        "## Command line",
        "",
        "```",
        " ".join(sys.argv),
        "```",
        "",
        "## Relevant CI environment variables (if set)",
        "",
    ]
    for v in ("XEN_TARGET_ARCH", "XEN_DEFCONFIG", "XEN_EXTRA_CONFIG",
              "CROSS_COMPILE", "MINERVA_CONFIG_NAME",
              "XEN_CALLGRAPH_FLAG", "XEN_BUILD_JOBS",
              "MINERVA_RUNTIME_LOG_DIR", "MINERVA_INDIRECT_OUT",
              "MINERVA_CI_ALLOW_RUNTIME_FAILURE",
              "MINERVA_CI_SKIP_RUNTIME",
              "CI_JOB_ID", "CI_COMMIT_SHA", "CI_PIPELINE_ID"):
        val = os.environ.get(v)
        if val is not None:
            lines.append(f"- `{v}` = `{val}`")
    p.write_text("\n".join(lines) + "\n", encoding="utf-8")


def stage_config(out_dir: Path, args: argparse.Namespace,
                 xen_root: Path) -> tuple[Path, dict]:
    """Produce the post-expansion .config. Returns (path, info)."""
    info: dict = {"mode": "skip-build" if args.skip_build else "build"}
    cfg_dir = out_dir / "config"
    cfg_dir.mkdir(parents=True, exist_ok=True)
    target = cfg_dir / ".config"

    if args.skip_build:
        if not args.config or not args.config.exists():
            raise SystemExit(
                "PARTIAL: --skip-build requires --config <expanded-.config>"
            )
        shutil.copy2(args.config, target)
        info["source"] = str(args.config)
        info["copied_from"] = str(args.config)
    else:
        # Build mode: invoke Xen kbuild to expand defconfig.
        if not args.defconfig:
            raise SystemExit("PARTIAL: --defconfig required in build mode")
        # If defconfig is a path, copy to xen/.config. Otherwise treat
        # as a make target.
        defconfig_path = Path(args.defconfig)
        env = os.environ.copy()
        env["XEN_OS"] = "Linux"
        env["XEN_TARGET_ARCH"] = args.target_arch
        if args.cross_compile:
            env["CROSS_COMPILE"] = args.cross_compile
        if defconfig_path.exists():
            shutil.copy2(defconfig_path, xen_root / "xen" / ".config")
        else:
            _run(["make", "-C", "xen", "XEN_OS=Linux",
                  f"XEN_TARGET_ARCH={args.target_arch}",
                  args.defconfig],
                 cwd=xen_root, env=env)
        # Append --extra symbols literally; olddefconfig will fold them.
        extras_path = xen_root / "xen" / ".config"
        if args.extra:
            with extras_path.open("a", encoding="utf-8") as fh:
                for e in args.extra:
                    fh.write(e + "\n")
        _run(["make", "-C", "xen", "XEN_OS=Linux",
              f"XEN_TARGET_ARCH={args.target_arch}",
              "olddefconfig"], cwd=xen_root, env=env)
        shutil.copy2(extras_path, target)
        info["source"] = "xen/.config (post-olddefconfig)"

    info["target_path"] = str(target)
    info["symbol_count"] = sum(
        1 for line in target.read_text(errors="replace").splitlines()
        if line.startswith("CONFIG_")
    )
    # config-scope.md: short orientation file under config/.
    (cfg_dir / "config-scope.md").write_text(
        "# Archived expanded `.config`\n\n"
        f"Source: {info['source']}\n\n"
        f"CONFIG_* symbols: {info['symbol_count']}\n\n"
        "Selected highlights:\n\n```\n"
        + "\n".join(
            line for line in target.read_text(errors="replace").splitlines()
            if line.startswith(("CONFIG_ARM", "CONFIG_HVM",
                                "CONFIG_IOMMU", "CONFIG_SMMU",
                                "CONFIG_DEBUG_INFO", "CONFIG_IOREQ_SERVER"))
        )
        + "\n```\n",
        encoding="utf-8")
    return target, info


def stage_ci(out_dir: Path, args: argparse.Namespace,
             xen_root: Path) -> tuple[Path, dict]:
    """Produce or consume the .ci tree. Returns (ci_dir_path, info)."""
    info: dict = {}
    ci_section = out_dir / "ci"
    ci_section.mkdir(parents=True, exist_ok=True)

    if args.skip_build:
        if not args.ci_dir or not args.ci_dir.exists():
            raise SystemExit(
                "PARTIAL: --skip-build requires --ci-dir <existing-ci-dir>"
            )
        ci_dir = args.ci_dir
        info["source"] = str(ci_dir)
        info["mode"] = "skip-build (existing tree)"
    else:
        # Build Xen with -fcallgraph-info=su (or override).
        env = os.environ.copy()
        env["XEN_OS"] = "Linux"
        env["XEN_TARGET_ARCH"] = args.target_arch
        if args.cross_compile:
            env["CROSS_COMPILE"] = args.cross_compile
        flag = args.callgraph_flag or "-fcallgraph-info=su"
        # Inject the callgraph flag via Xen's EXTRA_CFLAGS_XEN_CORE.
        _run(["make", "-C", "xen",
              "XEN_OS=Linux",
              f"XEN_TARGET_ARCH={args.target_arch}",
              f"EXTRA_CFLAGS_XEN_CORE={flag}",
              f"-j{args.jobs}"],
             cwd=xen_root, env=env)
        ci_dir = xen_root / "xen"
        info["source"] = str(ci_dir)
        info["mode"] = "build (in-tree .ci)"

    ci_files = sorted(p for p in ci_dir.rglob("*.ci"))
    list_path = ci_section / "ci-files.list"
    list_path.write_text("\n".join(str(p) for p in ci_files) + "\n",
                         encoding="utf-8")
    info["count"] = len(ci_files)
    info["ci_dir"] = str(ci_dir)

    if info["count"] == 0:
        raise SystemExit("PARTIAL: zero .ci files found in " + str(ci_dir))
    return ci_dir, info


def stage_collect(xen_root: Path, out_dir: Path, args: argparse.Namespace,
                  config_path: Path) -> dict:
    coll = out_dir / "collect"
    coll.mkdir(parents=True, exist_ok=True)
    _run([sys.executable, str(xen_root / "automation/renesas-scripts/minerva" / "collect.py"),
          "--xen-root", str(xen_root),
          "--config", str(config_path),
          "--config-name", args.config_name,
          "--out-dir", str(coll),
          "--target-arch", args.target_arch])
    summary_path = coll / "collection-summary.json"
    if not summary_path.exists():
        raise SystemExit("PARTIAL: collect.py did not produce "
                         "collection-summary.json")
    return json.loads(summary_path.read_text())


def stage_normalize_gcc(xen_root: Path, out_dir: Path,
                        ci_dir: Path, args) -> Path:
    """Run the GCC `.ci` -> normalized adapter; return out dir."""
    norm = out_dir / "normalized"
    _run([sys.executable,
          str(xen_root / "automation/renesas-scripts/minerva" / "callgraph" /
              "gcc_ci_to_normalized.py"),
          "--ci-dir", str(ci_dir),
          "--out-dir", str(norm),
          "--config-name", args.config_name,
          "--target-arch", args.target_arch])
    return norm


def stage_normalize_llvm(xen_root: Path, out_dir: Path,
                         ir_dir: Path, args) -> Path:
    """Run the LLVM IR -> normalized adapter; return out dir."""
    norm = out_dir / "normalized"
    cmd = [sys.executable,
           str(xen_root / "automation/renesas-scripts/minerva" / "callgraph" /
               "llvm_ir_to_normalized.py"),
           "--ir-dir", str(ir_dir),
           "--out-dir", str(norm),
           "--config-name", args.config_name,
           "--target-arch", args.target_arch]
    if args.llvm_dis:
        cmd += ["--llvm-dis", args.llvm_dis]
    _run(cmd)
    return norm


def stage_generate_llvm_ir(xen_root: Path, out_dir: Path, args
                           ) -> tuple[Path, dict]:
    """Generate a `.ll` tree by analysis compile replay.

    Imports scripts/llvm_ir_gen.py by path (the driver is invoked
    from a checkout that is not necessarily on sys.path) and runs the
    end-to-end generation. Returns (ir_dir, summary).
    """
    import importlib.util
    import shlex
    gen_path = xen_root / "automation/renesas-scripts/minerva" / "llvm_ir_gen.py"
    spec = importlib.util.spec_from_file_location(
        "_llvm_ir_gen", str(gen_path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    ir_dir = (args.llvm_ir_out_dir if args.llvm_ir_out_dir
              else out_dir / "llvm-ir")
    extra = shlex.split(args.llvm_ir_extra_cflags) \
        if args.llvm_ir_extra_cflags else []
    summary = mod.generate_llvm_ir(
        xen_root, ir_out_dir=ir_dir,
        target_arch=args.target_arch,
        cross_compile=args.cross_compile,
        llvm_cc=args.llvm_cc,
        compile_log=args.compile_log,
        jobs=args.jobs, extra_cflags=extra,
        clean_before_capture=args.llvm_ir_clean_before_capture,
        keep_temp=args.keep_ir_temp)
    return ir_dir, summary


def stage_reachability(xen_root: Path, out_dir: Path, ci_dir: Path | None,
                       targets: list[str],
                       backend: str = "gcc-ci",
                       normalized_dir: Path | None = None,
                       target_arch: str = "") -> dict:
    reach = out_dir / "reachability"
    reach.mkdir(parents=True, exist_ok=True)
    cmd = [sys.executable, str(xen_root / "automation/renesas-scripts/minerva" /
                               "indirect_reachability.py"),
           "--collector-run", str(out_dir / "collect"),
           "--targets", *targets,
           "--out-dir", str(reach),
           "--xen-root", str(xen_root),
           "--callgraph-backend", backend]
    if target_arch:
        cmd += ["--target-arch", target_arch]
    if backend == "gcc-ci":
        cmd += ["--ci-dir", str(ci_dir)]
    elif backend == "normalized":
        cmd += ["--normalized-callgraph-dir", str(normalized_dir)]
    _run(cmd)
    rsum = reach / "reachability-summary.md"
    if not rsum.exists():
        raise SystemExit("PARTIAL: indirect_reachability.py did not "
                         "produce reachability-summary.md")
    # Reach-summary metric extraction: read the indirect-allocation-paths
    # CSV and tally the counters that go into status.json.
    import csv
    rows_path = reach / "indirect-allocation-paths.csv"
    info: dict = {
        "path_found_yes": 0, "path_found_no": 0,
        "path_found_UNKNOWN": 0, "path_found_ERROR": 0,
        "candidate_rows_included": 0,
        "candidate_rows_excluded": 0,
        "candidate_rows_field_name_only_excluded": 0,
    }
    impls_examined: set[str] = set()
    impls_reaching: set[str] = set()
    if rows_path.exists():
        with rows_path.open() as fh:
            for r in csv.DictReader(fh):
                info[f"path_found_{r['path_found']}"] = info.get(
                    f"path_found_{r['path_found']}", 0) + 1
                if r["candidate_included"] == "true":
                    info["candidate_rows_included"] += 1
                elif r["candidate_included"] == "false":
                    info["candidate_rows_excluded"] += 1
                if r["candidate_binding"] == "field_name_only":
                    info["candidate_rows_field_name_only_excluded"] += 1
                if r["implementation_function"]:
                    impls_examined.add(r["implementation_function"])
                    if r["path_found"] == "yes":
                        impls_reaching.add(r["implementation_function"])
    info["implementations_examined"] = len(impls_examined)
    info["implementations_reaching_alloc"] = len(impls_reaching)
    # candidate_synthetic_edges_included is the line-count of the
    # candidates YAML minus its config: header and edges: marker.
    yaml_path = reach / "synthetic_edges.candidates.yaml"
    if yaml_path.exists():
        info["candidate_synthetic_edges_included"] = sum(
            1 for ln in yaml_path.read_text().splitlines()
            if ln.startswith("  - call_site_function:")
        )
    yaml_x = reach / "synthetic_edges.excluded.yaml"
    if yaml_x.exists():
        info["candidate_synthetic_edges_excluded"] = sum(
            1 for ln in yaml_x.read_text().splitlines()
            if ln.startswith("  - call_site_function:")
        )
    return info


_FN_NAME_RE = re.compile(r"^\s*([A-Za-z_][A-Za-z0-9_]*)\b")


def stage_direct_static(xen_root: Path, out_dir: Path, ci_dir: Path,
                        targets: list[str]) -> dict:
    ds = out_dir / "direct-static"
    ds.mkdir(parents=True, exist_ok=True)
    info: dict = {}
    md_lines = ["# Direct-static baselines\n",
                f"`.ci` directory: `{ci_dir}`\n",
                "| Target | Path lines | Distinct functions | stderr lines |",
                "| --- | ---: | ---: | ---: |"]
    summary: dict = {"targets": {}}
    cp = xen_root / "minerva_static_analysis" / "callpath.py"
    for t in targets:
        paths = ds / f"{t}.paths"
        errs = ds / f"{t}.stderr"
        proc = subprocess.run(
            [sys.executable, str(cp), "to", str(ci_dir), t, "-i"],
            capture_output=True, text=True, check=False, timeout=600)
        paths.write_text(proc.stdout, encoding="utf-8")
        errs.write_text(proc.stderr, encoding="utf-8")
        names = set()
        for raw in proc.stdout.splitlines():
            m = _FN_NAME_RE.match(raw)
            if m:
                names.add(m.group(1))
        path_lines = sum(1 for _ in proc.stdout.splitlines())
        err_lines = sum(1 for _ in proc.stderr.splitlines())
        ok = (proc.returncode == 0)
        info[f"distinct_functions_{t}"] = len(names)
        summary["targets"][t] = {
            "path_tree_lines": path_lines,
            "distinct_functions": len(names),
            "stderr_lines": err_lines,
            "returncode": proc.returncode,
            "ok": ok,
        }
        md_lines.append(f"| `{t}` | {path_lines} | {len(names)} | "
                        f"{err_lines} |")
    md_lines.append("")
    md_lines.append(
        "Distinct-function count is the unique function-name set across\n"
        "all printed paths; the workbench uses these sets for\n"
        "target-tree-membership reachability decisions.\n"
    )
    (ds / "direct-static-summary.md").write_text(
        "\n".join(md_lines) + "\n", encoding="utf-8")
    (ds / "direct-static-summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8")
    info["all_ok"] = all(t["ok"] for t in summary["targets"].values())
    return info


def stage_runtime_command(xen_root: Path, out_dir: Path, args,
                          config_path: Path, git_sha: str,
                          config_sha256: str) -> tuple[bool, str]:
    """Run the operator's runtime workload command (Option A).

    Runs only after the static stages have succeeded. Sets the
    MINERVA_* environment the command needs, captures stdout/stderr,
    tees stdout into a parseable log, and writes runtime-manifest.json
    recording the config/git/arch this run was produced from so the
    comparison stage can check alignment.
    """
    rt = out_dir / "runtime"
    logs = (args.runtime_log_dir if args.runtime_log_dir
            else rt / "logs")
    logs.mkdir(parents=True, exist_ok=True)
    rt.mkdir(parents=True, exist_ok=True)

    env = dict(os.environ)
    env["MINERVA_RUNTIME_LOG_DIR"] = str(logs)
    env["MINERVA_CONFIG_PATH"] = str(config_path)
    env["MINERVA_CONFIG_NAME"] = args.config_name
    env["MINERVA_GIT_SHA"] = git_sha
    env["MINERVA_TARGET_ARCH"] = args.target_arch
    env["MINERVA_DEFCONFIG"] = args.defconfig or ""

    manifest = {
        "git_sha": git_sha,
        "config_name": args.config_name,
        "target_arch": args.target_arch,
        "defconfig": args.defconfig or "",
        "config_sha256": config_sha256,
        "runtime_command": args.runtime_command,
        "timestamp": _ts(),
        "status": "started",
    }
    (rt / "runtime-manifest.json").write_text(
        json.dumps(manifest, indent=2), encoding="utf-8")

    ok = False
    reason = ""
    try:
        proc = subprocess.run(
            ["bash", "-lc", args.runtime_command],
            cwd=str(xen_root), env=env, capture_output=True,
            text=True, check=False,
            timeout=args.runtime_timeout)
        (rt / "runtime-command.stdout").write_text(
            proc.stdout or "", encoding="utf-8")
        (rt / "runtime-command.stderr").write_text(
            proc.stderr or "", encoding="utf-8")
        # If the command streamed Xen logs to stdout rather than into
        # the log dir, tee a parseable copy so the parser has input.
        if proc.stdout and not any(logs.iterdir()):
            name = (args.runtime_artifact_name or "runtime") + ".log"
            (logs / name).write_text(proc.stdout, encoding="utf-8")
        if proc.returncode == 0:
            ok = True
        else:
            reason = f"runtime command exited {proc.returncode}"
    except subprocess.TimeoutExpired:
        reason = f"runtime command timed out after {args.runtime_timeout}s"
    except OSError as exc:
        reason = f"{exc.__class__.__name__}: {exc}"

    manifest["status"] = "ok" if ok else "failed"
    if reason:
        manifest["failure_reason"] = reason
    (rt / "runtime-manifest.json").write_text(
        json.dumps(manifest, indent=2), encoding="utf-8")
    return ok, reason


def runtime_manifest_aligned(out_dir: Path, git_sha: str,
                             config_sha256: str,
                             same_job: bool) -> tuple[bool, str]:
    """Check the runtime manifest matches the current job metadata.

    Returns (aligned, reason).

    same_job=True (the driver ran --runtime-command): a manifest is
    required. Same-job runs always write one, so a missing manifest is
    treated as not aligned rather than silently COMPLETE.

    same_job=False (external --runtime-log-dir, no command): a missing
    manifest leaves the alignment claim with the operator; we return
    not-aligned so the run is PROXY/PARTIAL unless --allow-proxy is
    given, rather than silently COMPLETE.
    """
    mf = out_dir / "runtime" / "runtime-manifest.json"
    if not mf.exists():
        if same_job:
            return False, ("same-job runtime produced no manifest "
                           "(unexpected)")
        return False, ("external runtime logs have no manifest; "
                       "alignment cannot be verified")
    try:
        m = json.loads(mf.read_text())
    except (ValueError, OSError) as exc:
        return False, f"manifest unreadable: {exc}"
    if git_sha and m.get("git_sha") and m["git_sha"] != git_sha:
        return False, (f"git_sha mismatch: manifest {m['git_sha']} "
                       f"vs job {git_sha}")
    if (config_sha256 and m.get("config_sha256")
            and m["config_sha256"] != config_sha256):
        return False, "config_sha256 mismatch"
    return True, "aligned"


def stage_runtime(xen_root: Path, out_dir: Path,
                  log_dir: Path) -> tuple[bool, str]:
    rt = out_dir / "runtime"
    rt.mkdir(parents=True, exist_ok=True)
    parsed = rt / "parsed"
    parsed.mkdir(parents=True, exist_ok=True)
    try:
        _run([sys.executable,
              str(xen_root / "automation/renesas-scripts/minerva" / "log_parser.py"),
              str(log_dir)],
             timeout=900)
    except subprocess.CalledProcessError as e:
        (rt / "runtime-summary.md").write_text(
            "# Runtime parsing FAILED\n\n```\n"
            + (e.stderr or "") + "\n```\n", encoding="utf-8")
        return False, "log_parser.py failed"
    except (FileNotFoundError, subprocess.TimeoutExpired) as e:
        return False, str(e)
    (rt / "runtime-summary.md").write_text(
        "# Runtime parsing succeeded\n\n"
        f"Source: `{log_dir}`\n\nParsed output under `parsed/`.\n",
        encoding="utf-8")
    (rt / "runtime-summary.json").write_text(
        json.dumps({"source": str(log_dir), "ok": True}, indent=2),
        encoding="utf-8")
    return True, ""


def stage_runtime_static(xen_root: Path, out_dir: Path,
                         config_path: Path, config_name: str,
                         git_sha: str) -> tuple[bool, dict, str]:
    """Run scripts/runtime_static_compare.py over this run's artifacts.

    Returns (ok, metrics, reason).
    """
    rs = out_dir / "runtime-static"
    rs.mkdir(parents=True, exist_ok=True)
    try:
        _run([sys.executable,
              str(xen_root / "automation/renesas-scripts/minerva" / "runtime_static_compare.py"),
              "--runtime-parsed", str(out_dir / "runtime" / "parsed"),
              "--reachability-dir", str(out_dir / "reachability"),
              "--direct-static-dir", str(out_dir / "direct-static"),
              "--collect-dir", str(out_dir / "collect"),
              "--config", str(config_path),
              "--config-name", config_name,
              "--git-sha", git_sha,
              "--out-dir", str(rs)])
    except (subprocess.CalledProcessError, FileNotFoundError) as e:
        reason = f"runtime_static_compare.py failed: {e}"
        (rs / "runtime-static-comparison.md").write_text(
            f"# Runtime/static comparison FAILED\n\n{reason}\n",
            encoding="utf-8")
        return False, {}, reason
    metrics = {}
    mj = rs / "runtime-static-comparison.json"
    if mj.exists():
        metrics = json.loads(mj.read_text())
    return True, metrics, ""


def stage_runtime_static_placeholder(out_dir: Path, status_label: str):
    """Write a static-only runtime-static stub (no runtime logs)."""
    rs = out_dir / "runtime-static"
    rs.mkdir(parents=True, exist_ok=True)
    (rs / "runtime-static-comparison.md").write_text(
        "# Runtime/static comparison\n\n"
        f"Status: **{status_label}**\n\n"
        "No runtime logs were collected for this run, so no\n"
        "comparison was performed. This file is a per-run artifact.\n\n"
        "## Caveats\n\n"
        "- No allocation bound is claimed.\n"
        "- `path_found=no` does not mean impossible.\n"
        "- Static reachability does not mean a path was exercised at\n"
        "  runtime.\n",
        encoding="utf-8")
    (rs / "runtime-static-comparison.json").write_text(
        json.dumps({"status": status_label}, indent=2),
        encoding="utf-8")


def write_status(out_dir: Path, status_label: str, *,
                 config_name: str, git_sha: str, target_arch: str,
                 defconfig: str | None, extras: list[str],
                 ci_count: int, collector_ok: bool, reachability_ok: bool,
                 direct_static_ok: bool, runtime_ok: bool,
                 runtime_reason: str, counters: dict, notes: list[str],
                 callgraph_backend: str = "gcc-ci",
                 callgraph_artifact_kind: str = "n/a",
                 llvm_ir_files: int = 0,
                 llvm_bc_files: int = 0,
                 llvm_extractor_ok: bool = False,
                 normalized_graph_ok: bool = False,
                 llvm_ir_generation_requested: bool = False,
                 llvm_ir_generation_ok: bool = False,
                 llvm_ir_generation_mode: str = "n/a",
                 llvm_ir_compiler: str = "n/a",
                 llvm_ir_compiler_version: str = "n/a",
                 llvm_ir_failures: int = 0,
                 jobs: int = 0,
                 host_cpu_count: int | None = None):
    status = {
        "status": status_label,
        "config_name": config_name,
        "git_sha": git_sha,
        "target_arch": target_arch,
        "defconfig": defconfig,
        "extra_config": extras,
        "ci_files": ci_count,
        "collector_ok": collector_ok,
        "reachability_ok": reachability_ok,
        "direct_static_ok": direct_static_ok,
        "runtime_ok": runtime_ok,
        "runtime_reason": runtime_reason,
        "runtime_command_ok": counters.get("runtime_command_ok", False),
        "runtime_parser_ok": counters.get("runtime_parser_ok", False),
        "runtime_manifest_ok": counters.get("runtime_manifest_ok", False),
        "runtime_static_compare_ok":
            counters.get("runtime_static_compare_ok", False),
        "runtime_paths_total": counters.get("runtime_paths_total", 0),
        "runtime_paths_direct_static_explained":
            counters.get("runtime_paths_direct_static_explained", 0),
        "runtime_paths_indirect_explained":
            counters.get("runtime_paths_indirect_explained", 0),
        "runtime_paths_unexplained":
            counters.get("runtime_paths_unexplained", 0),
        "indirect_candidates_observed":
            counters.get("indirect_candidates_observed", 0),
        "indirect_candidates_not_observed":
            counters.get("indirect_candidates_not_observed", 0),
        "callgraph_backend": callgraph_backend,
        "callgraph_artifact_kind": callgraph_artifact_kind,
        "callgraph_functions": counters.get("callgraph_functions", 0),
        "callgraph_edges": counters.get("callgraph_edges", 0),
        "llvm_ir_files": llvm_ir_files,
        "llvm_bc_files": llvm_bc_files,
        "llvm_extractor_ok": llvm_extractor_ok,
        "llvm_ir_generation_requested": llvm_ir_generation_requested,
        "llvm_ir_generation_ok": llvm_ir_generation_ok,
        "llvm_ir_generation_mode": llvm_ir_generation_mode,
        "llvm_ir_compiler": llvm_ir_compiler,
        "llvm_ir_compiler_version": llvm_ir_compiler_version,
        "llvm_ir_failures": llvm_ir_failures,
        "normalized_graph_ok": normalized_graph_ok,
        "normalized_graph_functions": counters.get(
            "callgraph_functions", 0),
        "normalized_graph_edges": counters.get("callgraph_edges", 0),
        "jobs": jobs,
        "host_cpu_count": host_cpu_count,
        "counters": counters,
        "notes": notes,
        "generated_at": _ts(),
    }
    (out_dir / "status.json").write_text(
        json.dumps(status, indent=2), encoding="utf-8")
    # Mirror a one-screen summary as ci-summary.md.
    lines = ["# CI summary", "",
             f"**Status:** {status_label}", "",
             f"- config_name: `{config_name}`",
             f"- target_arch: `{target_arch}`",
             f"- defconfig: `{defconfig}`",
             f"- extras: {', '.join(extras) if extras else '(none)'}",
             f"- git_sha: `{git_sha}`",
             f"- .ci files: {ci_count}",
             f"- collector_ok: {collector_ok}",
             f"- reachability_ok: {reachability_ok}",
             f"- direct_static_ok: {direct_static_ok}",
             f"- runtime_ok: {runtime_ok} ({runtime_reason})",
             "", "## Counters\n",
             "```", json.dumps(counters, indent=2), "```", "",
             "## Notes\n"]
    for n in notes:
        lines.append(f"- {n}")
    (out_dir / "ci-summary.md").write_text(
        "\n".join(lines) + "\n", encoding="utf-8")


def main():
    p = argparse.ArgumentParser(
        description="Minerva indirect-reachability CI driver.")
    p.add_argument("--xen-root", type=Path, default=Path("."))
    p.add_argument("--out-dir", type=Path, required=True)
    p.add_argument("--config-name", required=True)
    p.add_argument("--target-arch", required=True,
                   choices=("arm64", "x86_64"))
    p.add_argument("--cross-compile", default="")
    p.add_argument("--defconfig", default="")
    p.add_argument("--extra", action="append", default=[])
    p.add_argument("--runtime-log-dir", type=Path, default=None)
    p.add_argument("--runtime-command", default="",
                   help="Command to execute after the static stages "
                        "to produce runtime logs (Option A, same "
                        "job). Run with MINERVA_* env set; stdout is "
                        "teed into the runtime log dir if the command "
                        "does not write logs there itself.")
    p.add_argument("--runtime-artifact-name", default="",
                   help="Prefix for the teed runtime log file.")
    p.add_argument("--runtime-timeout", type=int, default=600,
                   help="Timeout in seconds for --runtime-command.")
    p.add_argument("--runtime-required", action="store_true",
                   help="If set, a runtime failure makes the run "
                        "PARTIAL. If not set, a runtime failure may "
                        "downgrade to STATIC_ONLY only when "
                        "--allow-runtime-failure (or "
                        "MINERVA_CI_ALLOW_RUNTIME_FAILURE) is set.")
    p.add_argument("--allow-runtime-failure", action="store_true",
                   help="A failed runtime stage records the failure "
                        "but preserves static artifacts and "
                        "downgrades to STATIC_ONLY.")
    p.add_argument("--allow-proxy", action="store_true",
                   help="If the runtime manifest does not align with "
                        "this job's git_sha / config hash, label the "
                        "run PROXY instead of PARTIAL. For Option A "
                        "(same job) this should not be needed.")
    p.add_argument("--skip-build", action="store_true")
    p.add_argument("--config", type=Path, default=None,
                   help="Required with --skip-build; expanded `.config` "
                        "from an external build.")
    p.add_argument("--ci-dir", type=Path, default=None,
                   help="Required with --skip-build; .ci tree from the "
                        "same external build.")
    p.add_argument("--no-runtime", action="store_true",
                   help="Force STATIC_ONLY: skip runtime parsing even "
                        "if --runtime-log-dir is supplied.")
    # Default jobs: scale with the runner (`os.cpu_count()`).
    # Fall back to 2 on hosts where `cpu_count()` returns None.
    # The wrapper script (`scripts/run_indirect_ci.sh`) supplies
    # `$XEN_BUILD_JOBS` (default `nproc`) in CI; this default
    # applies when the driver is invoked directly. The effective
    # value is recorded in `environment.md` and `status.json` so
    # the runner host's CPU count is auditable per run.
    p.add_argument("--jobs", type=int, default=(os.cpu_count() or 2))
    p.add_argument("--callgraph-flag", default="-fcallgraph-info=su")
    p.add_argument("--callgraph-backend",
                   choices=("gcc-ci", "llvm-ir", "normalized"),
                   default="gcc-ci",
                   help="Callgraph backend. `gcc-ci` (default): build "
                        "with GCC callgraph flag and feed .ci tree to "
                        "callpath.py. `llvm-ir`: consume textual LLVM "
                        "IR (.ll/.bc) under --llvm-ir-dir, extract a "
                        "normalized graph, and run reachability "
                        "against it. `normalized`: skip extraction; "
                        "consume an already-prepared normalized graph "
                        "directory via --normalized-callgraph-dir.")
    p.add_argument("--llvm-ir-dir", type=Path, default=None,
                   help="Directory of textual LLVM IR (.ll) and/or "
                        "bitcode (.bc) files when "
                        "--callgraph-backend=llvm-ir.")
    p.add_argument("--llvm-dis", default="",
                   help="Path to `llvm-dis` for .bc -> .ll conversion. "
                        "Defaults to the binary on PATH.")
    # LLVM IR generation (analysis compile replay). Opt-in; only
    # consulted when --callgraph-backend=llvm-ir. When set, the
    # driver generates a .ll tree from the same expanded .config and
    # feeds it to the normalizer, instead of requiring an external
    # --llvm-ir-dir.
    p.add_argument("--generate-llvm-ir", action="store_true",
                   help="Generate LLVM IR artifacts from the "
                        "configured Xen build (analysis compile "
                        "replay) and consume them, instead of "
                        "requiring --llvm-ir-dir. Only meaningful "
                        "with --callgraph-backend=llvm-ir.")
    p.add_argument("--llvm-cc", default="clang",
                   help="Clang executable used for IR generation. "
                        "Default: clang.")
    p.add_argument("--llvm-ir-out-dir", type=Path, default=None,
                   help="Output directory for generated IR. Default: "
                        "<out-dir>/llvm-ir.")
    p.add_argument("--compile-log", type=Path, default=None,
                   help="Pre-captured verbose build log to parse for "
                        "compile commands. If absent, the driver runs "
                        "a verbose build to capture them.")
    p.add_argument("--ir-replay-from-build-log", action="store_true",
                   help="Explicit selector for build-log replay mode. "
                        "This is the default when --generate-llvm-ir "
                        "is set without --compile-log; the flag exists "
                        "for symmetry and self-documentation.")
    p.add_argument("--keep-ir-temp", action="store_true",
                   help="Keep temporary replay state (failed-command "
                        "argv) in the IR generation summary for "
                        "debugging.")
    p.add_argument("--llvm-ir-extra-cflags", default="",
                   help="Extra flags appended to each IR replay "
                        "command (whitespace-separated).")
    p.add_argument("--llvm-ir-clean-before-capture", action="store_true",
                   help="Run `make -C xen clean` before the verbose "
                        "capture build (build-log-replay mode) so "
                        "every C compile is logged. Use in reused "
                        "workspaces.")
    p.add_argument("--allow-partial-llvm-ir", action="store_true",
                   help="Continue when some (but not all) IR replays "
                        "fail. Default is strict: any replay failure "
                        "makes the run PARTIAL.")
    p.add_argument("--normalized-callgraph-dir", type=Path, default=None,
                   help="Prebuilt normalized graph directory. Used "
                        "when --callgraph-backend=normalized; bypasses "
                        "extraction.")
    p.add_argument("--status-only", action="store_true",
                   help="Write status.json + ci-summary.md from "
                        "whatever artifacts already exist; do not "
                        "re-run any stage.")
    p.add_argument("--targets", nargs="+",
                   default=["alloc_domheap_pages", "alloc_xenheap_pages",
                            "_xmalloc"])
    args = p.parse_args()

    xen_root = args.xen_root.resolve()
    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    notes: list[str] = []
    counters: dict = {}
    status_label = "STATIC_ONLY"
    collector_ok = False
    reachability_ok = False
    direct_static_ok = False
    runtime_ok = False
    runtime_reason = "not attempted"
    ci_count = 0

    write_environment_md(out_dir, args, xen_root)
    if args.skip_build:
        notes.append(
            "Skip-build mode: .config and .ci were supplied externally; "
            "this is a local-validation artifact, not a fresh CI build."
        )

    callgraph_backend = args.callgraph_backend
    callgraph_artifact_kind = "n/a"
    llvm_ir_files = 0
    llvm_bc_files = 0
    llvm_extractor_ok = False
    normalized_graph_ok = False
    llvm_ir_generation_requested = bool(
        args.generate_llvm_ir
        and callgraph_backend == "llvm-ir")
    llvm_ir_generation_ok = False
    llvm_ir_generation_mode = "n/a"
    llvm_ir_compiler = args.llvm_cc
    llvm_ir_compiler_version = "n/a"
    llvm_ir_failures = 0
    ci_dir: Path | None = None

    try:
        config_path, _cfg_info = stage_config(out_dir, args, xen_root)
        # The gcc-ci backend needs a .ci tree from the build (or
        # skip-build). The llvm-ir backend needs a .ll tree. The
        # normalized backend needs nothing but the prebuilt
        # functions.csv/edges.csv.
        if callgraph_backend == "gcc-ci":
            ci_dir, ci_info = stage_ci(out_dir, args, xen_root)
            ci_count = ci_info["count"]
            callgraph_artifact_kind = "gcc-ci"
        elif callgraph_backend == "llvm-ir":
            # Two ways to obtain IR: generate it from this run's
            # expanded .config (analysis compile replay), or consume
            # an externally supplied tree via --llvm-ir-dir.
            if args.generate_llvm_ir:
                if args.llvm_ir_dir:
                    notes.append(
                        "--generate-llvm-ir overrides --llvm-ir-dir; "
                        "generating IR from this run's build.")
                ir_dir, gen_summary = stage_generate_llvm_ir(
                    xen_root, out_dir, args)
                llvm_ir_generation_mode = gen_summary.get("mode", "n/a")
                llvm_ir_compiler_version = gen_summary.get(
                    "compiler_version", "n/a")
                llvm_ir_failures = gen_summary.get("replay_failed", 0)
                llvm_ir_files = gen_summary.get("ir_files_generated", 0)
                if llvm_ir_files == 0:
                    raise SystemExit(
                        "PARTIAL: LLVM IR generation produced zero "
                        ".ll files; see llvm-ir/"
                        "ir-generation-summary.json")
                if llvm_ir_failures and not args.allow_partial_llvm_ir:
                    raise SystemExit(
                        f"PARTIAL: {llvm_ir_failures} LLVM IR replay "
                        f"command(s) failed; pass --allow-partial-"
                        f"llvm-ir to continue on the rest")
                if llvm_ir_failures and args.allow_partial_llvm_ir:
                    notes.append(
                        f"{llvm_ir_failures} IR replay failure(s) "
                        f"tolerated (--allow-partial-llvm-ir); "
                        f"continuing on {llvm_ir_files} generated "
                        f"file(s).")
                llvm_ir_generation_ok = True
                # Point the consumer path at the generated tree.
                args.llvm_ir_dir = ir_dir
            if not args.llvm_ir_dir or not args.llvm_ir_dir.exists():
                raise SystemExit(
                    "UNSUPPORTED_BACKEND: --callgraph-backend=llvm-ir "
                    "requires --llvm-ir-dir <existing-path> or "
                    "--generate-llvm-ir")
            llvm_ir_files = sum(
                1 for _ in args.llvm_ir_dir.rglob("*.ll"))
            llvm_bc_files = sum(
                1 for _ in args.llvm_ir_dir.rglob("*.bc"))
            if llvm_ir_files + llvm_bc_files == 0:
                raise SystemExit(
                    f"UNSUPPORTED_BACKEND: no .ll or .bc files under "
                    f"{args.llvm_ir_dir}")
            callgraph_artifact_kind = "llvm-ir"
        elif callgraph_backend == "normalized":
            if (not args.normalized_callgraph_dir
                    or not args.normalized_callgraph_dir.exists()):
                raise SystemExit(
                    "UNSUPPORTED_BACKEND: "
                    "--callgraph-backend=normalized requires "
                    "--normalized-callgraph-dir <existing-path>")
            callgraph_artifact_kind = "normalized-prebuilt"

        cs = stage_collect(xen_root, out_dir, args, config_path)
        counters.update({
            k: cs.get(k) for k in (
                "ops_resolution_rows", "indirect_call_sites",
                "indirect_call_sites_with_function",
                "runtime_registration_sites",
            ) if k in cs
        })
        collector_ok = True

        # Produce or reuse a normalized graph, then run reachability
        # against whichever backend the caller picked.
        normalized_dir: Path | None = None
        if callgraph_backend == "gcc-ci":
            # Archive a normalized view of the gcc-ci graph alongside
            # the legacy reachability output. This keeps the CI
            # artifact set backend-neutral even when the build was
            # GCC-based.
            try:
                normalized_dir = stage_normalize_gcc(
                    xen_root, out_dir, ci_dir, args)
                normalized_graph_ok = True
            except subprocess.CalledProcessError as e:
                notes.append(
                    f"gcc-ci -> normalized adapter failed: "
                    f"{(e.stderr or '')[:200]}")
            rinfo = stage_reachability(
                xen_root, out_dir, ci_dir, args.targets,
                backend="gcc-ci", target_arch=args.target_arch)
        elif callgraph_backend == "llvm-ir":
            normalized_dir = stage_normalize_llvm(
                xen_root, out_dir, args.llvm_ir_dir, args)
            llvm_extractor_ok = True
            normalized_graph_ok = True
            rinfo = stage_reachability(
                xen_root, out_dir, None, args.targets,
                backend="normalized",
                normalized_dir=normalized_dir,
                target_arch=args.target_arch)
        else:  # normalized
            normalized_dir = args.normalized_callgraph_dir
            normalized_graph_ok = True
            rinfo = stage_reachability(
                xen_root, out_dir, None, args.targets,
                backend="normalized",
                normalized_dir=normalized_dir,
                target_arch=args.target_arch)
        counters.update(rinfo)
        reachability_ok = True

        # Surface callgraph_functions / callgraph_edges from the
        # archived normalized graph directly so status.json has them
        # even when the gcc-ci legacy reachability path was taken
        # (its counters scrape doesn't include those fields).
        if normalized_dir is not None:
            try:
                with (normalized_dir / "functions.csv").open() as fh:
                    counters["callgraph_functions"] = max(
                        0, sum(1 for _ in fh) - 1)
                with (normalized_dir / "edges.csv").open() as fh:
                    counters["callgraph_edges"] = max(
                        0, sum(1 for _ in fh) - 1)
            except OSError:
                pass

        if ci_dir is not None:
            dinfo = stage_direct_static(
                xen_root, out_dir, ci_dir, args.targets)
            direct_static_ok = bool(dinfo.get("all_ok"))
            counters.update({k: v for k, v in dinfo.items()
                             if k.startswith("distinct_functions_")})
        elif normalized_dir is not None:
            # Direct-static summary from the normalized graph: how
            # many distinct functions reach each target.
            ds = out_dir / "direct-static"
            ds.mkdir(parents=True, exist_ok=True)
            ng_mod_path = (xen_root / "automation/renesas-scripts/minerva" / "callgraph"
                           / "normalized_graph.py")
            import importlib.util
            spec = importlib.util.spec_from_file_location(
                "_ng_runtime", str(ng_mod_path))
            ng_mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(ng_mod)
            edges = ng_mod.load_edges(normalized_dir)
            ds_summary: dict = {"format": "normalized-target-tree",
                                "targets": {}}
            md_lines = ["# Direct-static baselines (normalized backend)\n",
                        f"Source: `{normalized_dir}`\n",
                        "| Target | Distinct functions reaching target |",
                        "| --- | ---: |"]
            for t in args.targets:
                reach_set = ng_mod.functions_reaching_target(edges, t)
                (ds / f"{t}.functions").write_text(
                    "\n".join(sorted(reach_set)) + "\n",
                    encoding="utf-8")
                (ds / f"{t}.stderr").write_text("", encoding="utf-8")
                ds_summary["targets"][t] = {
                    "distinct_functions": len(reach_set),
                    "format": "normalized-target-tree",
                    "ok": True,
                }
                counters[f"distinct_functions_{t}"] = len(reach_set)
                md_lines.append(f"| `{t}` | {len(reach_set)} |")
            md_lines.append("")
            (ds / "direct-static-summary.md").write_text(
                "\n".join(md_lines) + "\n", encoding="utf-8")
            (ds / "direct-static-summary.json").write_text(
                json.dumps(ds_summary, indent=2), encoding="utf-8")
            direct_static_ok = True
    except SystemExit as e:
        notes.append(str(e))
        msg = str(e)
        if "UNSUPPORTED_BACKEND" in msg:
            status_label = "UNSUPPORTED_BACKEND"
        else:
            status_label = "PARTIAL"

    # Runtime + comparison (Option A: same-job runtime collection).
    git_sha = _safe(["git", "-C", str(xen_root), "rev-parse", "HEAD"])
    config_sha256 = ""
    if config_path and Path(config_path).exists():
        import hashlib
        config_sha256 = hashlib.sha256(
            Path(config_path).read_bytes()).hexdigest()

    runtime_command_ok = False
    runtime_parser_ok = False
    runtime_manifest_ok = False
    runtime_static_compare_ok = False
    compare_metrics: dict = {}

    runtime_requested = bool(args.runtime_command or args.runtime_log_dir)
    allow_rt_fail = (args.allow_runtime_failure
                     or os.environ.get("MINERVA_CI_ALLOW_RUNTIME_FAILURE",
                                       "").lower() in ("1", "true", "yes"))

    if status_label == "PARTIAL":
        pass  # static stages already failed; leave PARTIAL.
    elif args.no_runtime or not runtime_requested:
        runtime_reason = ("--no-runtime supplied" if args.no_runtime
                          else "no runtime command or log dir supplied")
        status_label = "STATIC_ONLY"
        stage_runtime_static_placeholder(out_dir, status_label)
    else:
        # 1. Optional runtime command (produces logs).
        if args.runtime_command:
            runtime_command_ok, cmd_reason = stage_runtime_command(
                xen_root, out_dir, args, config_path, git_sha,
                config_sha256)
            if not runtime_command_ok:
                runtime_reason = cmd_reason
        else:
            runtime_command_ok = True  # external logs path

        # 2. Runtime parser (if command ok, or external logs supplied).
        log_dir = (args.runtime_log_dir if args.runtime_log_dir
                   else out_dir / "runtime" / "logs")
        if runtime_command_ok:
            runtime_ok, rt_reason = stage_runtime(
                xen_root, out_dir, log_dir)
            runtime_parser_ok = runtime_ok
            if not runtime_ok:
                runtime_reason = rt_reason

        # 3. Manifest alignment + comparison.
        if runtime_parser_ok:
            same_job = bool(args.runtime_command)
            aligned, align_reason = runtime_manifest_aligned(
                out_dir, git_sha, config_sha256, same_job)
            runtime_manifest_ok = aligned
            notes.append(f"Runtime manifest alignment: {align_reason}.")
            ok, compare_metrics, cmp_reason = stage_runtime_static(
                xen_root, out_dir, config_path, args.config_name,
                git_sha)
            runtime_static_compare_ok = ok
            if not ok:
                runtime_reason = cmp_reason
                status_label = "PARTIAL"
            elif not aligned:
                if args.allow_proxy:
                    status_label = "PROXY"
                    notes.append(f"Runtime manifest not aligned "
                                 f"({align_reason}); labelled PROXY "
                                 f"per --allow-proxy.")
                else:
                    status_label = "PARTIAL"
                    notes.append(f"Runtime manifest not aligned "
                                 f"({align_reason}); PARTIAL. Pass "
                                 f"--allow-proxy to accept unaligned "
                                 f"logs as PROXY.")
            else:
                status_label = "COMPLETE"
        else:
            # Runtime (command or parser) failed.
            if args.runtime_required:
                status_label = "PARTIAL"
                notes.append("Runtime stage failed and "
                             "--runtime-required is set; PARTIAL.")
            elif allow_rt_fail:
                status_label = "STATIC_ONLY"
                notes.append(f"Runtime stage failed ({runtime_reason}) "
                             f"but failure is allowed; downgraded to "
                             f"STATIC_ONLY.")
            else:
                status_label = "PARTIAL"
                notes.append(f"Runtime stage failed ({runtime_reason}); "
                             f"PARTIAL. Pass --allow-runtime-failure to "
                             f"downgrade to STATIC_ONLY.")
            stage_runtime_static_placeholder(out_dir, status_label)

    # Surface comparison metrics into status counters.
    for k in ("runtime_paths_total",
              "runtime_paths_direct_static_explained",
              "runtime_paths_indirect_explained",
              "runtime_paths_unexplained",
              "indirect_candidates_observed",
              "indirect_candidates_not_observed"):
        if k in compare_metrics:
            counters[k] = compare_metrics[k]
    counters["runtime_command_ok"] = runtime_command_ok
    counters["runtime_parser_ok"] = runtime_parser_ok
    counters["runtime_manifest_ok"] = runtime_manifest_ok
    counters["runtime_static_compare_ok"] = runtime_static_compare_ok

    write_status(out_dir, status_label,
                 config_name=args.config_name,
                 git_sha=git_sha,
                 target_arch=args.target_arch,
                 defconfig=args.defconfig or None,
                 extras=args.extra,
                 ci_count=ci_count,
                 collector_ok=collector_ok,
                 reachability_ok=reachability_ok,
                 direct_static_ok=direct_static_ok,
                 runtime_ok=runtime_ok,
                 runtime_reason=runtime_reason,
                 counters=counters,
                 notes=notes,
                 callgraph_backend=callgraph_backend,
                 callgraph_artifact_kind=callgraph_artifact_kind,
                 llvm_ir_files=llvm_ir_files,
                 llvm_bc_files=llvm_bc_files,
                 llvm_extractor_ok=llvm_extractor_ok,
                 normalized_graph_ok=normalized_graph_ok,
                 llvm_ir_generation_requested=llvm_ir_generation_requested,
                 llvm_ir_generation_ok=llvm_ir_generation_ok,
                 llvm_ir_generation_mode=llvm_ir_generation_mode,
                 llvm_ir_compiler=llvm_ir_compiler,
                 llvm_ir_compiler_version=llvm_ir_compiler_version,
                 llvm_ir_failures=llvm_ir_failures,
                 jobs=args.jobs,
                 host_cpu_count=os.cpu_count())

    print(f"\nFINAL STATUS: {status_label}", file=sys.stderr)
    if status_label == "PARTIAL":
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
