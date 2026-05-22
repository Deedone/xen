#!/usr/bin/env python3
"""Generate LLVM IR callgraph inputs by analysis compile replay.

This module produces textual LLVM IR (`.ll`) for the Xen hypervisor C
sources so the existing `scripts/callgraph/llvm_ir_to_normalized.py`
extractor has something to consume in CI. It does *not* replace the
production Xen object build, and it does not require an LLVM plugin or
any DOT output.

Strategy: analysis compile replay.

  1. Capture a verbose build log (or read one supplied by the caller)
     that contains the per-file C compile command lines.
  2. Extract the Clang C-compile commands from that log.
  3. Rewrite each command into an `-S -emit-llvm` analysis compile that
     writes a `.ll` next to a mirrored object-relative path under the
     IR output tree, dropping object-only and `.d` dependency flags.
  4. Replay the rewritten commands.
  5. Summarise: `ir-files.list`, `ir-generation-summary.json`,
     `ir-generation-summary.md`.

The generated `.ll` tree is a static-analysis artifact. The caller
(`scripts/indirect_ci_driver.py`) then runs the normalizer over the
tree and the normalized reachability backend over the result. The
generated IR is never committed.

Failure policy: failures are recorded, never silently dropped. Zero
generated files is a hard failure. With some-but-not-all replays
failing, the default is strict (the caller treats it as PARTIAL);
`allow_partial=True` lets the caller continue on the files that did
generate.
"""

from __future__ import annotations

import json
import os
import shlex
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path


def _ts() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


# Compiler-driver argv[0] basenames that count as a C compile we can
# replay as `-S -emit-llvm`. The configured compiler basename is added
# to this set at call time.
_CLANG_BASENAMES = {"clang", "clang++", "cc", "c++"}

# Source extensions we replay. Assembly is excluded on purpose.
_C_SOURCE_SUFFIXES = (".c",)

# Tool invocations that are never C compiles.
_NON_COMPILE_TOOLS = {
    "ld", "ld.lld", "ld.bfd", "ld.gold", "ar", "llvm-ar",
    "objcopy", "llvm-objcopy", "ranlib", "nm", "strip", "as",
    "llvm-as", "llvm-dis", "objdump", "readelf", "size",
}

# Dependency-output flags. `-MF`, `-MT`, `-MQ`, `-MJ` take a following
# argument; the rest are standalone. They are stripped from the replay
# because the `.d`/`.json` targets they name belong to the object
# build, not the IR side-build. Clang/GCC also accept the joined forms
# (`-MFfile`, `-MTtarget`, `-MQtarget`, `-MJfile`); those are matched
# by prefix so the joined form does not slip through and write an
# object-build dependency file.
_DEP_FLAGS_WITH_ARG = {"-MF", "-MT", "-MQ", "-MJ"}
_DEP_FLAGS_JOINED_PREFIXES = ("-MF", "-MT", "-MQ", "-MJ")
_DEP_FLAGS_STANDALONE = {"-MMD", "-MD", "-MP", "-MG", "-MM", "-M"}


def _argv0_basename(argv: list[str]) -> str:
    if not argv:
        return ""
    return os.path.basename(argv[0])


def _is_compiler(basename: str, compiler_basename: str) -> bool:
    if basename == compiler_basename:
        return True
    if basename in _CLANG_BASENAMES:
        return True
    # Cross/wrapped names like `aarch64-linux-gnu-clang` or
    # `clang-18`.
    return ("clang" in basename
            and basename not in _NON_COMPILE_TOOLS)


def _is_joined_dep_flag(tok: str) -> bool:
    """True for joined dependency flags like -MFfile / -MTtarget.

    Matches a `-MF`/`-MT`/`-MQ`/`-MJ` prefix immediately followed by
    a value (no space). The bare `-MF` etc. forms (which take a
    separate argument) are handled separately and are not matched
    here.
    """
    for pre in _DEP_FLAGS_JOINED_PREFIXES:
        if tok.startswith(pre) and len(tok) > len(pre):
            return True
    return False


def _source_of(argv: list[str]) -> str | None:
    """Return the first C source operand in argv, else None."""
    skip_next = False
    for i, tok in enumerate(argv[1:], start=1):
        if skip_next:
            skip_next = False
            continue
        if tok in _DEP_FLAGS_WITH_ARG or tok == "-o":
            skip_next = True
            continue
        if tok.startswith("-"):
            continue
        if tok.endswith(_C_SOURCE_SUFFIXES):
            return tok
    return None


def _object_of(argv: list[str]) -> str | None:
    for i, tok in enumerate(argv):
        if tok == "-o" and i + 1 < len(argv):
            return argv[i + 1]
    return None


def extract_clang_compile_commands(build_log_text: str,
                                   compiler_basename: str = "clang"
                                   ) -> list[dict]:
    """Extract Clang C-compile commands from a verbose build log.

    Returns a list of dicts: {"argv", "source", "object", "raw"}.
    A line qualifies when its argv[0] is a Clang-family compiler, it
    has `-c`, it names a `.c` source, and it names a `-o` object.
    Assembly, link, archive, and host-tool lines are ignored.
    Commands whose source is under a `xen/` path sort first so the
    hypervisor tree is preferred when callers truncate.
    """
    commands: list[dict] = []
    seen: set[tuple] = set()
    for raw in build_log_text.splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        # Verbose make often prefixes recipe echoes; only attempt a
        # shlex parse on lines that mention the compiler and a source.
        if ".c" not in line:
            continue
        try:
            argv = shlex.split(line)
        except ValueError:
            # Unbalanced quotes: not a command we can faithfully
            # replay. Skip rather than guess.
            continue
        if not argv:
            continue
        base = _argv0_basename(argv)
        if base in _NON_COMPILE_TOOLS:
            continue
        if not _is_compiler(base, compiler_basename):
            continue
        if "-c" not in argv:
            continue
        if "-S" in argv or "-emit-llvm" in argv:
            continue
        src = _source_of(argv)
        obj = _object_of(argv)
        if not src or not obj:
            continue
        if not obj.endswith(".o"):
            continue
        key = (src, obj)
        if key in seen:
            continue
        seen.add(key)
        commands.append({"argv": argv, "source": src,
                         "object": obj, "raw": line})

    def _xen_first(cmd: dict) -> tuple:
        s = cmd["source"].replace("\\", "/")
        return (0 if ("xen/" in s or s.startswith("xen/")) else 1,
                cmd["source"])

    commands.sort(key=_xen_first)
    return commands


def _ir_output_path(object_path: str, source_root: Path,
                    ir_root: Path, used: set[str]) -> Path:
    """Map an object path to a mirrored `.ll` path under ir_root.

    `xen/common/foo.o` -> `<ir_root>/xen/common/foo.ll`. Collisions
    (two commands mapping to the same `.ll`) get a stable numeric
    suffix so no output silently overwrites another.
    """
    obj = Path(object_path)
    rel: Path
    try:
        rel = obj.resolve().relative_to(source_root.resolve())
    except (ValueError, OSError):
        # Not under source_root (or unresolvable): keep the path
        # tail, stripping any leading separators/drive.
        parts = [p for p in obj.parts if p not in ("/", "\\")
                 and not p.endswith(":")]
        rel = Path(*parts) if parts else Path(obj.name)
    ll = rel.with_suffix(".ll")
    candidate = ir_root / ll
    key = str(candidate)
    if key in used:
        stem = ll.stem
        parent = ll.parent
        n = 1
        while key in used:
            candidate = ir_root / parent / f"{stem}.{n}.ll"
            key = str(candidate)
            n += 1
    used.add(key)
    return candidate


def rewrite_compile_command_to_llvm_ir(argv: list[str],
                                       source_root: Path,
                                       ir_root: Path,
                                       used_outputs: set[str],
                                       llvm_cc: str | None = None,
                                       extra_cflags: list[str] | None = None
                                       ) -> tuple[list[str], Path]:
    """Rewrite one object-producing C compile into an IR compile.

    Removes `-c`, the `-o <object>` pair, and `.d` dependency flags;
    adds `-S -emit-llvm -o <ir>.ll`; preserves include paths, defines,
    target flags, and warning flags. Optionally overrides argv[0] with
    `llvm_cc` and appends `extra_cflags`.
    Returns (new_argv, ir_output_path).
    """
    obj = _object_of(argv)
    if obj is None:
        raise ValueError("compile command has no -o object output")
    ir_path = _ir_output_path(obj, source_root, ir_root, used_outputs)

    out: list[str] = []
    skip_next = False
    for i, tok in enumerate(argv):
        if skip_next:
            skip_next = False
            continue
        if i == 0:
            out.append(llvm_cc if llvm_cc else tok)
            continue
        if tok == "-c":
            continue
        if tok == "-o":
            skip_next = True   # drop the object operand too
            continue
        if tok in _DEP_FLAGS_WITH_ARG:
            skip_next = True
            continue
        if tok in _DEP_FLAGS_STANDALONE:
            continue
        if _is_joined_dep_flag(tok):
            continue
        out.append(tok)
    out += ["-S", "-emit-llvm"]
    if extra_cflags:
        out += list(extra_cflags)
    out += ["-o", str(ir_path)]
    return out, ir_path


def capture_build_log(xen_root: Path, *, target_arch: str,
                      cross_compile: str, llvm_cc: str,
                      jobs: int, clean_before_capture: bool = False,
                      extra_make_args: list[str] | None = None,
                      env: dict | None = None) -> tuple[str, int, str]:
    """Run a verbose Xen build to capture C compile command lines.

    The build is run with `V=1` and `CC=<llvm_cc>` and its combined
    output is captured. `check=False`: even a build that fails partway
    yields the compile lines emitted before the failure, which is all
    the replay step needs. Returns (log_text, returncode, command).

    In a reused workspace `make` may find objects up to date and emit
    few or zero compile lines. `clean_before_capture=True` runs
    `make -C xen clean` first so every C file is recompiled and its
    command line appears in the log.
    """
    e = dict(os.environ if env is None else env)
    e["XEN_OS"] = "Linux"
    e["XEN_TARGET_ARCH"] = target_arch
    if cross_compile:
        e["CROSS_COMPILE"] = cross_compile
    if clean_before_capture:
        clean_cmd = ["make", "-C", "xen", "XEN_OS=Linux",
                     f"XEN_TARGET_ARCH={target_arch}", "clean"]
        print(f"+ {' '.join(clean_cmd)}", file=sys.stderr)
        subprocess.run(clean_cmd, cwd=str(xen_root), env=e,
                       capture_output=True, text=True, check=False)
    cmd = ["make", "-C", "xen", "V=1",
           "XEN_OS=Linux",
           f"XEN_TARGET_ARCH={target_arch}",
           f"CC={llvm_cc}",
           f"-j{jobs}"]
    if extra_make_args:
        cmd += list(extra_make_args)
    print(f"+ {' '.join(cmd)}", file=sys.stderr)
    proc = subprocess.run(cmd, cwd=str(xen_root), env=e,
                          capture_output=True, text=True, check=False)
    return (proc.stdout + "\n" + proc.stderr, proc.returncode,
            " ".join(cmd))


def run_llvm_ir_replay(commands: list[dict], *, source_root: Path,
                       ir_root: Path, llvm_cc: str | None,
                       extra_cflags: list[str] | None,
                       keep_temp: bool = False) -> dict:
    """Replay rewritten IR compile commands. Records every result.

    Returns a summary dict with per-command outcomes. Failures are
    retained in the summary; nothing is silently dropped.
    """
    ir_root.mkdir(parents=True, exist_ok=True)
    used: set[str] = set()
    results: list[dict] = []
    ok_count = 0
    fail_count = 0
    for cmd in commands:
        argv = cmd["argv"]
        try:
            new_argv, ir_path = rewrite_compile_command_to_llvm_ir(
                argv, source_root, ir_root, used,
                llvm_cc=llvm_cc, extra_cflags=extra_cflags)
        except ValueError as exc:
            fail_count += 1
            results.append({"source": cmd.get("source"),
                            "object": cmd.get("object"),
                            "ok": False, "stage": "rewrite",
                            "error": str(exc)})
            continue
        ir_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            proc = subprocess.run(new_argv, capture_output=True,
                                  text=True, check=False, cwd=source_root/"xen")
        except OSError as exc:
            # Missing/invalid compiler, exec failure, etc. Record it
            # as a replay failure; never let it crash the driver.
            fail_count += 1
            rec = {"source": cmd.get("source"),
                   "object": cmd.get("object"),
                   "ir": str(ir_path),
                   "ok": False,
                   "stage": "compile",
                   "error": f"{exc.__class__.__name__}: {exc}"}
            if keep_temp:
                rec["argv"] = new_argv
            results.append(rec)
            continue
        ok = (proc.returncode == 0 and ir_path.exists())
        if ok:
            ok_count += 1
        else:
            fail_count += 1
        rec = {"source": cmd.get("source"),
               "object": cmd.get("object"),
               "ir": str(ir_path),
               "ok": ok,
               "stage": "compile",
               "returncode": proc.returncode}
        if not ok:
            rec["stderr_tail"] = "\n".join(
                (proc.stderr or "").splitlines()[-8:])
            if keep_temp:
                rec["argv"] = new_argv
        results.append(rec)
    return {"ok_count": ok_count, "fail_count": fail_count,
            "results": results}


def summarize_ir_generation(ir_root: Path, *, mode: str, compiler: str,
                            compiler_version: str, capture_command: str,
                            capture_returncode: int | None,
                            extracted: int, replay: dict,
                            requested: bool) -> dict:
    """Write ir-files.list, ir-generation-summary.{json,md}.

    Returns the JSON summary dict (also written to disk).
    """
    ir_files = sorted(str(p.relative_to(ir_root))
                      for p in ir_root.rglob("*.ll"))
    (ir_root / "ir-files.list").write_text(
        "\n".join(ir_files) + ("\n" if ir_files else ""),
        encoding="utf-8")

    failures = [r for r in replay.get("results", []) if not r["ok"]]
    summary = {
        "generated_at": _ts(),
        "mode": mode,
        "requested": requested,
        "compiler": compiler,
        "compiler_version": compiler_version,
        "capture_command": capture_command,
        "capture_returncode": capture_returncode,
        "commands_extracted": extracted,
        "ir_files_generated": len(ir_files),
        "replay_ok": replay.get("ok_count", 0),
        "replay_failed": replay.get("fail_count", 0),
        "failures": failures,
    }
    (ir_root / "ir-generation-summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8")

    md = [
        "# LLVM IR generation summary",
        "",
        "Generated `.ll` files are a **static-analysis artifact**, not a",
        "production object build. No LLVM plugin and no DOT output are",
        "used. These files are not committed.",
        "",
        f"- mode: `{mode}`",
        f"- compiler: `{compiler}`",
        f"- compiler version: `{compiler_version}`",
        f"- commands extracted: {extracted}",
        f"- IR files generated: {len(ir_files)}",
        f"- replay ok: {replay.get('ok_count', 0)}",
        f"- replay failed: {replay.get('fail_count', 0)}",
        f"- capture returncode: {capture_returncode}",
        "",
    ]
    if failures:
        md.append("## Replay failures")
        md.append("")
        md.append("| Source | Object | Stage | rc |")
        md.append("| --- | --- | --- | ---: |")
        for r in failures[:50]:
            md.append(f"| `{r.get('source')}` | `{r.get('object')}` | "
                      f"{r.get('stage')} | {r.get('returncode', '')} |")
        if len(failures) > 50:
            md.append(f"")
            md.append(f"... and {len(failures) - 50} more (see JSON).")
        md.append("")
    if extracted == 0:
        md.append("## No compile commands extracted")
        md.append("")
        md.append("Zero extracted compile commands usually means the "
                  "build was up to date")
        md.append("(a reused workspace), or the verbose log did not "
                  "contain Clang C")
        md.append("compile lines. In build-log-replay mode, pass "
                  "--llvm-ir-clean-before-capture")
        md.append("(or set MINERVA_LLVM_IR_CLEAN_BEFORE_CAPTURE) to "
                  "force a full rebuild so")
        md.append("every C compile is logged.")
        md.append("")
    md.append("Consumed by `scripts/callgraph/llvm_ir_to_normalized.py`.")
    md.append("")
    (ir_root / "ir-generation-summary.md").write_text(
        "\n".join(md) + "\n", encoding="utf-8")
    return summary


def _compiler_version(llvm_cc: str) -> str:
    try:
        proc = subprocess.run([llvm_cc, "--version"],
                              capture_output=True, text=True,
                              check=False, timeout=30)
        return (proc.stdout or proc.stderr).splitlines()[0] \
            if (proc.stdout or proc.stderr) else "(unknown)"
    except Exception as exc:    # noqa: BLE001
        return f"<{exc.__class__.__name__}>"


def generate_llvm_ir(xen_root: Path, *, ir_out_dir: Path,
                     target_arch: str, cross_compile: str,
                     llvm_cc: str = "clang",
                     compile_log: Path | None = None,
                     jobs: int = 2,
                     extra_cflags: list[str] | None = None,
                     clean_before_capture: bool = False,
                     keep_temp: bool = False) -> dict:
    """End-to-end IR generation. Returns a summary dict.

    If `compile_log` is given it is parsed directly (mode
    `compile-log`). Otherwise a verbose build is run to capture the
    commands (mode `build-log-replay`).
    """
    ir_out_dir.mkdir(parents=True, exist_ok=True)
    compiler_basename = os.path.basename(llvm_cc)
    version = _compiler_version(llvm_cc)

    if compile_log is not None:
        mode = "compile-log"
        log_text = Path(compile_log).read_text(errors="replace")
        capture_cmd = f"(supplied) {compile_log}"
        capture_rc: int | None = None
    else:
        mode = "build-log-replay"
        log_text, capture_rc, capture_cmd = capture_build_log(
            xen_root, target_arch=target_arch,
            cross_compile=cross_compile, llvm_cc=llvm_cc, jobs=jobs,
            clean_before_capture=clean_before_capture)
        (ir_out_dir / "capture-build.log").write_text(
            log_text, encoding="utf-8")

    commands = extract_clang_compile_commands(log_text, compiler_basename)
    replay = run_llvm_ir_replay(
        commands, source_root=xen_root, ir_root=ir_out_dir,
        llvm_cc=llvm_cc, extra_cflags=extra_cflags, keep_temp=keep_temp)
    summary = summarize_ir_generation(
        ir_out_dir, mode=mode, compiler=llvm_cc,
        compiler_version=version, capture_command=capture_cmd,
        capture_returncode=capture_rc, extracted=len(commands),
        replay=replay, requested=True)
    return summary


# --------------------------------------------------------------------
# Self-test: command-rewrite unit checks. Runnable in CI without a
# compiler. `python3 scripts/llvm_ir_gen.py --self-test`.
# --------------------------------------------------------------------

def _self_test() -> int:
    import tempfile
    failures: list[str] = []

    def check(name: str, cond: bool):
        if not cond:
            failures.append(name)
            print(f"FAIL {name}", file=sys.stderr)
        else:
            print(f"ok   {name}", file=sys.stderr)

    root = Path(tempfile.mkdtemp())
    ir = root / "llvm-ir"

    # 1. Basic rewrite.
    used: set[str] = set()
    argv = shlex.split(
        "clang -Iinc -DTEST=1 -c xen/common/foo.c -o xen/common/foo.o")
    new_argv, ir_path = rewrite_compile_command_to_llvm_ir(
        argv, root, ir, used)
    check("1.no_-c", "-c" not in new_argv)
    check("1.has_-S", "-S" in new_argv)
    check("1.has_-emit-llvm", "-emit-llvm" in new_argv)
    check("1.keeps_-Iinc", "-Iinc" in new_argv)
    check("1.keeps_-DTEST=1", "-DTEST=1" in new_argv)
    check("1.no_object_o", "xen/common/foo.o" not in new_argv)
    check("1.ir_suffix_ll", str(ir_path).endswith(".ll"))
    check("1.ir_mirror_path",
          str(ir_path).replace("\\", "/").endswith(
              "llvm-ir/xen/common/foo.ll"))
    oi = new_argv.index("-o")
    check("1.o_targets_ll", new_argv[oi + 1].endswith("foo.ll"))

    # 2. Dependency flags dropped.
    used = set()
    argv = shlex.split(
        "clang -MMD -MF xen/common/.foo.o.d -MT xen/common/foo.o "
        "-MP -c xen/common/foo.c -o xen/common/foo.o")
    new_argv, _ = rewrite_compile_command_to_llvm_ir(argv, root, ir, used)
    check("2.no_-MMD", "-MMD" not in new_argv)
    check("2.no_-MF", "-MF" not in new_argv)
    check("2.no_dotd_target",
          not any(a.endswith(".o.d") for a in new_argv))
    check("2.no_-MT", "-MT" not in new_argv)
    check("2.no_-MP", "-MP" not in new_argv)

    # 2b. Joined dependency flags dropped (-MFfile / -MTtarget).
    used = set()
    argv = shlex.split(
        "clang -MD -MFxen/common/.foo.o.d -MTxen/common/foo.o "
        "-c xen/common/foo.c -o xen/common/foo.o")
    new_argv, _ = rewrite_compile_command_to_llvm_ir(argv, root, ir, used)
    check("2b.no_joined_-MF",
          not any(a.startswith("-MF") for a in new_argv))
    check("2b.no_joined_-MT",
          not any(a.startswith("-MT") for a in new_argv))
    check("2b.no_dotd_anywhere",
          not any(a.endswith(".o.d") for a in new_argv))
    check("2b.kept_-S", "-S" in new_argv)

    # 3. Quoted args / spaces preserved through shlex.
    used = set()
    line = ('clang -DMSG="hello world" -I"my inc" -c '
            'xen/common/foo.c -o xen/common/foo.o')
    cmds = extract_clang_compile_commands(line, "clang")
    check("3.one_command", len(cmds) == 1)
    if cmds:
        new_argv, _ = rewrite_compile_command_to_llvm_ir(
            cmds[0]["argv"], root, ir, set())
        check("3.define_with_space",
              "-DMSG=hello world" in new_argv)
        check("3.include_with_space", "-Imy inc" in new_argv)

    # 4. Zero commands extracted from an empty / irrelevant log.
    cmds = extract_clang_compile_commands(
        "ld -o vmlinux a.o b.o\nmake[1]: nothing to do\n", "clang")
    check("4.zero_commands", len(cmds) == 0)

    # 4b. Assembly and archive lines ignored.
    log = ("clang -c xen/common/foo.S -o xen/common/foo.o\n"
           "llvm-ar rcs lib.a a.o b.o\n"
           "aarch64-linux-gnu-clang-18 -Iinc -c xen/arch/arm/p2m.c "
           "-o xen/arch/arm/p2m.o\n")
    cmds = extract_clang_compile_commands(log, "clang")
    check("4b.only_c_compile", len(cmds) == 1)
    check("4b.picked_p2m",
          bool(cmds) and cmds[0]["source"].endswith("p2m.c"))

    # 5. Collision disambiguation.
    used = set()
    a_argv = shlex.split("clang -c x/foo.c -o x/foo.o")
    b_argv = shlex.split("clang -c x/foo.c -o x/foo.o")
    _, p1 = rewrite_compile_command_to_llvm_ir(a_argv, root, ir, used)
    _, p2 = rewrite_compile_command_to_llvm_ir(b_argv, root, ir, used)
    check("5.collision_distinct", str(p1) != str(p2))

    # 6. Missing/invalid compiler is recorded, not raised.
    miss_ir = root / "miss-ir"
    cmd = {"argv": shlex.split("clang -c x/foo.c -o x/foo.o"),
           "source": "x/foo.c", "object": "x/foo.o"}
    crashed = False
    try:
        rep = run_llvm_ir_replay(
            [cmd], source_root=root, ir_root=miss_ir,
            llvm_cc="/nonexistent/clang-does-not-exist",
            extra_cflags=None)
    except Exception as exc:    # noqa: BLE001
        crashed = True
        print(f"     unexpected raise: {exc}", file=sys.stderr)
    check("6.no_traceback", not crashed)
    check("6.recorded_failure",
          (not crashed) and rep["fail_count"] == 1
          and rep["ok_count"] == 0)
    check("6.failure_has_error",
          (not crashed) and bool(rep["results"][0].get("error")))

    if failures:
        print(f"\nSELF-TEST FAILED: {len(failures)} check(s): "
              f"{', '.join(failures)}", file=sys.stderr)
        return 1
    print("\nSELF-TEST PASSED", file=sys.stderr)
    return 0


def main() -> int:
    import argparse
    p = argparse.ArgumentParser(
        description="Generate LLVM IR callgraph inputs by analysis "
                    "compile replay, or run the command-rewrite "
                    "self-test.")
    p.add_argument("--self-test", action="store_true",
                   help="Run command-rewrite unit checks and exit. "
                        "No compiler required.")
    p.add_argument("--xen-root", type=Path, default=Path("."))
    p.add_argument("--ir-out-dir", type=Path, default=None)
    p.add_argument("--target-arch", default="arm64")
    p.add_argument("--cross-compile", default="")
    p.add_argument("--llvm-cc", default="clang")
    p.add_argument("--compile-log", type=Path, default=None)
    p.add_argument("--jobs", type=int, default=(os.cpu_count() or 2))
    p.add_argument("--llvm-ir-extra-cflags", default="")
    p.add_argument("--llvm-ir-clean-before-capture", action="store_true",
                   help="Run `make -C xen clean` before the verbose "
                        "capture build so every C compile is logged. "
                        "Use in reused workspaces where make would "
                        "otherwise find objects up to date.")
    p.add_argument("--keep-ir-temp", action="store_true")
    args = p.parse_args()

    if args.self_test:
        return _self_test()

    if args.ir_out_dir is None:
        p.error("--ir-out-dir is required unless --self-test")
    extra = shlex.split(args.llvm_ir_extra_cflags) \
        if args.llvm_ir_extra_cflags else []
    summary = generate_llvm_ir(
        args.xen_root.resolve(), ir_out_dir=args.ir_out_dir.resolve(),
        target_arch=args.target_arch, cross_compile=args.cross_compile,
        llvm_cc=args.llvm_cc, compile_log=args.compile_log,
        jobs=args.jobs, extra_cflags=extra,
        clean_before_capture=args.llvm_ir_clean_before_capture,
        keep_temp=args.keep_ir_temp)
    print(json.dumps(summary, indent=2))
    if summary["ir_files_generated"] == 0:
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
