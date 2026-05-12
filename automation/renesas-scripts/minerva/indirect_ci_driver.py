#!/usr/bin/env python3
"""Minerva indirect-reachability CI driver.

Orchestrates one CI run of the indirect-call reachability workflow:

  1. produce or consume the expanded Xen `.config`;
  2. produce or consume the GCC callgraph `.ci` tree;
  3. run scripts/collect.py on the archived `.config`;
  4. run scripts/indirect_reachability.py on the collector output
     and the `.ci` tree;
  5. run direct-static baselines via
     minerva_static_analysis/callpath.py;
  6. optionally run minerva_analysis/log_parser.py against a
     runtime log directory;
  7. emit a single archive directory matching the per-run artifact
     contract and a top-level status.json.

This script is *configuration-parameterised*. Target arch, defconfig,
extra symbols, cross-compile prefix, and config name are all driver
arguments (typically supplied from CI environment variables). The
driver does not assume one specific Xen configuration.

Status labels: COMPLETE / STATIC_ONLY /
PROXY / PARTIAL. No final runtime-vs-static coverage number is
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
        f"jobs:           {args.jobs}",
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
          "--out-dir", str(coll)])
    summary_path = coll / "collection-summary.json"
    if not summary_path.exists():
        raise SystemExit("PARTIAL: collect.py did not produce "
                         "collection-summary.json")
    return json.loads(summary_path.read_text())


def stage_reachability(xen_root: Path, out_dir: Path, ci_dir: Path,
                       targets: list[str]) -> dict:
    reach = out_dir / "reachability"
    reach.mkdir(parents=True, exist_ok=True)
    _run([sys.executable, str(xen_root / "automation/renesas-scripts/minerva" /
                              "indirect_reachability.py"),
          "--collector-run", str(out_dir / "collect"),
          "--ci-dir", str(ci_dir),
          "--targets", *targets,
          "--out-dir", str(reach),
          "--xen-root", str(xen_root)])
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


def stage_runtime_static(out_dir: Path, status_label: str):
    rs = out_dir / "runtime-static"
    rs.mkdir(parents=True, exist_ok=True)
    (rs / "runtime-static-comparison.md").write_text(
        "# Runtime/static comparison\n\n"
        f"Status: **{status_label}**\n\n"
        "This file is a per-run artifact. No final\n"
        "runtime-vs-static coverage number is committed. Numbers in\n"
        "the sibling status.json reflect this CI run only.\n\n"
        "## Caveats\n\n"
        "- No allocation bound is claimed.\n"
        "- `path_found=no` does not mean impossible.\n"
        "- 'Not observed at runtime' does not mean impossible.\n"
        "- Field-name-only candidates are excluded from synthetic-edge\n"
        "  counts by the table-aware binding pass.\n"
        "- Runtime-registration rows remain an analyst worklist; they\n"
        "  are not expanded into per-field reachability queries.\n",
        encoding="utf-8")
    (rs / "runtime-static-comparison.json").write_text(
        json.dumps({"status": status_label}, indent=2),
        encoding="utf-8")


def write_status(out_dir: Path, status_label: str, *,
                 config_name: str, git_sha: str, target_arch: str,
                 defconfig: str | None, extras: list[str],
                 ci_count: int, collector_ok: bool, reachability_ok: bool,
                 direct_static_ok: bool, runtime_ok: bool,
                 runtime_reason: str, counters: dict, notes: list[str]):
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
        description="Minerva indirect-reachability CI driver "
                    ".")
    p.add_argument("--xen-root", type=Path, default=Path("."))
    p.add_argument("--out-dir", type=Path, required=True)
    p.add_argument("--config-name", required=True)
    p.add_argument("--target-arch", required=True,
                   choices=("arm64", "x86_64"))
    p.add_argument("--cross-compile", default="")
    p.add_argument("--defconfig", default="")
    p.add_argument("--extra", action="append", default=[])
    p.add_argument("--runtime-log-dir", type=Path, default=None)
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
    p.add_argument("--jobs", type=int, default=(os.cpu_count() or 2))
    p.add_argument("--callgraph-flag", default="-fcallgraph-info=su")
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

    try:
        config_path, _cfg_info = stage_config(out_dir, args, xen_root)
        ci_dir, ci_info = stage_ci(out_dir, args, xen_root)
        ci_count = ci_info["count"]

        cs = stage_collect(xen_root, out_dir, args, config_path)
        counters.update({
            k: cs.get(k) for k in (
                "ops_resolution_rows", "indirect_call_sites",
                "indirect_call_sites_with_function",
                "runtime_registration_sites",
            ) if k in cs
        })
        collector_ok = True

        rinfo = stage_reachability(xen_root, out_dir, ci_dir, args.targets)
        counters.update(rinfo)
        reachability_ok = True

        dinfo = stage_direct_static(xen_root, out_dir, ci_dir, args.targets)
        direct_static_ok = bool(dinfo.get("all_ok"))
        counters.update({k: v for k, v in dinfo.items()
                         if k.startswith("distinct_functions_")})
    except SystemExit as e:
        notes.append(str(e))
        status_label = "PARTIAL"

    if status_label != "PARTIAL":
        if args.no_runtime or not args.runtime_log_dir:
            runtime_reason = ("--no-runtime supplied" if args.no_runtime
                              else "not supplied")
            status_label = "STATIC_ONLY"
        else:
            runtime_ok, runtime_reason = stage_runtime(
                xen_root, out_dir, args.runtime_log_dir)
            if runtime_ok:
                # Operator is responsible for asserting alignment via
                # the runtime log directory's own provenance. The
                # driver records COMPLETE pending an explicit mismatch
                # signal in --notes (future work).
                status_label = "COMPLETE"
            else:
                if os.environ.get("MINERVA_CI_ALLOW_RUNTIME_FAILURE",
                                  "").lower() in ("1", "true", "yes"):
                    status_label = "STATIC_ONLY"
                    notes.append("Runtime parsing failed but "
                                 "MINERVA_CI_ALLOW_RUNTIME_FAILURE is "
                                 "set; downgraded to STATIC_ONLY.")
                else:
                    status_label = "PARTIAL"

    stage_runtime_static(out_dir, status_label)

    git_sha = _safe(["git", "-C", str(xen_root), "rev-parse", "HEAD"])
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
                 notes=notes)

    print(f"\nFINAL STATUS: {status_label}", file=sys.stderr)
    if status_label == "PARTIAL":
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
