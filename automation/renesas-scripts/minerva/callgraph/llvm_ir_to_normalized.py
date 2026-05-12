#!/usr/bin/env python3
"""Convert textual LLVM IR (`.ll`) into the normalized graph format.

This is the LLVM/Clang counterpart of `gcc_ci_to_normalized.py`.
The design constraint for this extractor is no LLVM plugin and
no DOT output, so the implementation is a small textual-IR
parser. Inputs:

  *.ll                  textual LLVM IR (preferred)
  *.bc                  bitcode; converted via `llvm-dis` when the
                        binary is available

Output: functions.csv, edges.csv, metadata.json under --out-dir.

Coverage

  - function definitions:    `define ... @function_name(...)` and
                             `define ... @"function_name"(...)`
  - direct calls / invokes:  `call ... @callee(...)`,
                             `invoke ... @callee(...) to ...`,
                             `tail call ...`,
                             `musttail call ...`,
                             `call ... bitcast (... @callee to ...)(...)`
  - indirect calls:          `call ... %funcptr(...)` (any %-ref in
                             the callee slot). Emitted with
                             `callee=""` and
                             `edge_kind=unresolved_indirect`.
  - LLVM intrinsics:         `@llvm.*` direct edges are tagged
                             `module="llvm-intrinsic"` for easy
                             filtering by consumers; the workbench
                             ignores them by default.
  - aliases:                 best-effort: `@alias = alias ... @target`
                             yields an edge `alias -> target` with
                             `edge_kind=direct, notes="alias"`.
  - locations:               if `!dbg !N` follows the call and the
                             `.ll` file contains `!N = !DILocation(
                             line: L, scope: ..., file: <mdref>)`,
                             extract (file, line). Optional; absence
                             leaves source_file/line blank. Debug
                             metadata is not mandatory for
                             reachability queries.
"""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

_SELF = Path(__file__).resolve()
sys.path.insert(0, str(_SELF.parent.parent))
from callgraph import normalized_graph as ng  # noqa: E402


# ---- IR-level regexes ----------------------------------------------------

# `define [linkage] [visibility] ret_type @name(args)` or
# `define [linkage] [visibility] ret_type @"name"(args)`.
# Allow leading attribute tokens by matching anything up to ` @ ` /
# ` @"`.
_DEFINE_RE = re.compile(
    r'^\s*define\b[^@]*@(?:"(?P<q_name>[^"]+)"|(?P<name>[A-Za-z_$.][\w$.]*))\s*\('
)

# Direct call. Anchored on the keyword sequence; tolerant to the
# many qualifiers `tail`, `musttail`, `notail`, `fast`, `nuw`, etc.
# may interleave between `call`/`invoke` and the operand list.
_DIRECT_CALL_RE = re.compile(
    r'\b(?:call|invoke|musttail call|tail call|notail call)\b[^@%]*'
    r'@(?:"(?P<q_callee>[^"]+)"|(?P<callee>[A-Za-z_$.][\w$.]*))\s*\('
)

# Direct call through a `bitcast` constant-expression wrapper:
#   call void bitcast (void (...)* @callee to void ()*)()
# The inner @-symbol identifies the real callee. Distinct from the
# plain direct regex because the `(` after the callee name belongs
# to the bitcast's type list (`to ...`), not to the call's
# argument list. Resolved to a `direct` edge with notes
# "bitcast-wrapper".
_BITCAST_DIRECT_CALL_RE = re.compile(
    r'\b(?:call|invoke|musttail call|tail call|notail call)\b'
    r'[^@%]*\bbitcast\s*\([^@%]*'
    r'@(?:"(?P<q_callee>[^"]+)"|(?P<callee>[A-Za-z_$.][\w$.]*))'
)

# Same shape but wrapping an SSA value:
#   call void bitcast (i32 (i32)* %fp to void ()*)()
# Remains unresolved_indirect (the target is still a runtime
# pointer); we just recognise the shape so we do not drop the
# line silently.
_BITCAST_INDIRECT_CALL_RE = re.compile(
    r'\b(?:call|invoke|musttail call|tail call|notail call)\b'
    r'[^@%]*\bbitcast\s*\([^@%]*'
    r'%(?P<callee_ref>[A-Za-z0-9_."]+)'
)

# Indirect call: `%name` or `%N` in the callee slot. We over-match
# slightly and let the direct match take precedence on each line.
_INDIRECT_CALL_RE = re.compile(
    r'\b(?:call|invoke|musttail call|tail call|notail call)\b[^@%]*'
    r'%(?P<callee_ref>[A-Za-z0-9_."]+)\s*\('
)

# `@a = alias ... @b` (alias to a function). Aliases are recorded
# as direct edges from the alias to the target.
_ALIAS_RE = re.compile(
    r'^\s*@(?:"(?P<q_alias>[^"]+)"|(?P<alias>[A-Za-z_$.][\w$.]*))\s*='
    r'.*\balias\b[^@]*@'
    r'(?:"(?P<q_target>[^"]+)"|(?P<target>[A-Za-z_$.][\w$.]*))'
)

# Debug-location markers: `!N = !DILocation(line: L, ..., scope: ...)`.
_DILOCATION_RE = re.compile(
    r'^!(?P<id>\d+)\s*=\s*!DILocation\(line:\s*(?P<line>\d+)'
    r'(?:,\s*column:\s*\d+)?'
    r'(?:,\s*scope:\s*!(?P<scope>\d+))?'
)

# `!N = distinct !DISubprogram(name: "...", ..., file: !M, ...)`.
_DISUBPROGRAM_FILE_RE = re.compile(
    r'^!(?P<id>\d+)\s*=\s*(?:distinct\s*)?!DISubprogram\([^)]*'
    r'\bfile:\s*!(?P<file>\d+)'
)

# `!N = !DIFile(filename: "X", directory: "Y")`.
_DIFILE_RE = re.compile(
    r'^!(?P<id>\d+)\s*=\s*!DIFile\(filename:\s*"(?P<filename>[^"]*)"'
    r'(?:,\s*directory:\s*"(?P<directory>[^"]*)")?'
)

# `, !dbg !N` attached to a call instruction.
_DBG_TAG_RE = re.compile(r',\s*!dbg\s+!(?P<id>\d+)\b')


def _resolve_dbg(metadata_index: dict[str, dict], dbg_id: str
                 ) -> tuple[str, str]:
    """Return (file, line) for `!dbg !N` if metadata is sufficient.

    The IR contains a small graph of metadata nodes. We follow:
      DILocation(line, scope=!M) -> DISubprogram(file=!K) -> DIFile.
    Best effort; missing pieces yield ("", "").
    """
    loc = metadata_index.get(("loc", dbg_id))
    if not loc:
        return ("", "")
    line = loc.get("line", "")
    scope_id = loc.get("scope", "")
    if not scope_id:
        return ("", str(line) if line else "")
    sub = metadata_index.get(("sub", scope_id))
    if not sub:
        return ("", str(line) if line else "")
    file_id = sub.get("file", "")
    if not file_id:
        return ("", str(line) if line else "")
    df = metadata_index.get(("file", file_id))
    if not df:
        return ("", str(line) if line else "")
    name = df.get("filename", "")
    return (name, str(line) if line else "")


def parse_ll(path: Path, source_root: Path | None = None
             ) -> tuple[list[dict], list[dict], list[str]]:
    """Parse one textual-IR file. Returns (functions, edges, warnings)."""
    warnings: list[str] = []
    try:
        text = path.read_text(errors="replace")
    except OSError as e:
        return [], [], [f"could not read {path}: {e}"]

    module = str(path.name)
    if source_root:
        try:
            module = str(path.relative_to(source_root.resolve())).replace("\\", "/")
        except ValueError:
            pass

    # 1. metadata index pass.
    metadata_index: dict[tuple[str, str], dict] = {}
    for line in text.splitlines():
        m = _DILOCATION_RE.match(line)
        if m:
            metadata_index[("loc", m.group("id"))] = {
                "line": m.group("line"),
                "scope": m.group("scope") or "",
            }
            continue
        m = _DISUBPROGRAM_FILE_RE.match(line)
        if m:
            metadata_index[("sub", m.group("id"))] = {
                "file": m.group("file"),
            }
            continue
        m = _DIFILE_RE.match(line)
        if m:
            metadata_index[("file", m.group("id"))] = {
                "filename": m.group("filename") or "",
                "directory": m.group("directory") or "",
            }
            continue

    # 2. function / call pass with a tiny state machine.
    functions: list[dict] = []
    edges: list[dict] = []
    cur_fn = ""
    cur_fn_line = 0
    in_body = False
    brace_depth = 0
    for idx, raw in enumerate(text.splitlines(), start=1):
        # Track function-definition spans by brace depth so call
        # instructions are correctly attributed to the enclosing
        # function.
        m = _DEFINE_RE.match(raw)
        if m and not in_body:
            cur_fn = m.group("q_name") or m.group("name") or ""
            cur_fn_line = idx
            # Find the opening `{` (often on the same line).
            if "{" in raw:
                in_body = True
                brace_depth = raw.count("{") - raw.count("}")
            # Register the function row.
            functions.append({
                "function": cur_fn,
                "source_file": "",
                "line": "",
                "module": module,
                "backend": "llvm-ir",
                "linkage": "",
                "visibility": "",
                "notes": "",
            })
            continue
        if not in_body:
            # Top-level: aliases.
            a = _ALIAS_RE.match(raw)
            if a:
                alias = a.group("q_alias") or a.group("alias") or ""
                target = a.group("q_target") or a.group("target") or ""
                if alias and target:
                    edges.append({
                        "caller": alias, "callee": target,
                        "source_file": "", "line": "",
                        "edge_kind": "direct",
                        "backend": "llvm-ir", "module": module,
                        "notes": "alias",
                    })
            continue
        # Inside a function body.
        brace_depth += raw.count("{") - raw.count("}")
        if brace_depth <= 0:
            in_body = False
            cur_fn = ""
            cur_fn_line = 0
            continue
        # Look for a direct call first (it's the dominant form).
        d = _DIRECT_CALL_RE.search(raw)
        if d:
            callee = d.group("q_callee") or d.group("callee") or ""
            dbg = _DBG_TAG_RE.search(raw)
            src, line_no = ("", "")
            if dbg:
                src, line_no = _resolve_dbg(metadata_index, dbg.group("id"))
            is_intr = callee.startswith("llvm.")
            edges.append({
                "caller": cur_fn,
                "callee": callee,
                "source_file": src,
                "line": line_no,
                "edge_kind": "direct",
                "backend": "llvm-ir",
                "module": "llvm-intrinsic" if is_intr else module,
                "notes": "intrinsic" if is_intr else "",
            })
            continue
        # `bitcast`-wrapper direct call:
        #   call void bitcast (void (...)* @callee to void ()*)()
        # The plain direct regex rejects this because the `(` after
        # @callee belongs to the bitcast type, not the argument list.
        # Detect and resolve to a direct edge.
        bd = _BITCAST_DIRECT_CALL_RE.search(raw)
        if bd:
            callee = bd.group("q_callee") or bd.group("callee") or ""
            dbg = _DBG_TAG_RE.search(raw)
            src, line_no = ("", "")
            if dbg:
                src, line_no = _resolve_dbg(metadata_index, dbg.group("id"))
            is_intr = callee.startswith("llvm.")
            note = "bitcast-wrapper"
            if is_intr:
                note = "bitcast-wrapper; intrinsic"
            edges.append({
                "caller": cur_fn,
                "callee": callee,
                "source_file": src,
                "line": line_no,
                "edge_kind": "direct",
                "backend": "llvm-ir",
                "module": "llvm-intrinsic" if is_intr else module,
                "notes": note,
            })
            continue
        # `bitcast`-wrapper around an SSA value: still indirect, but
        # the shape differs from the plain `%fp(...)` form, so call
        # it out explicitly.
        bi = _BITCAST_INDIRECT_CALL_RE.search(raw)
        if bi:
            dbg = _DBG_TAG_RE.search(raw)
            src, line_no = ("", "")
            if dbg:
                src, line_no = _resolve_dbg(metadata_index, dbg.group("id"))
            edges.append({
                "caller": cur_fn,
                "callee": "",
                "source_file": src,
                "line": line_no,
                "edge_kind": "unresolved_indirect",
                "backend": "llvm-ir",
                "module": module,
                "notes": f"bitcast-wrapper; target=%{bi.group('callee_ref')}",
            })
            continue
        # Then an indirect call.
        i = _INDIRECT_CALL_RE.search(raw)
        if i:
            dbg = _DBG_TAG_RE.search(raw)
            src, line_no = ("", "")
            if dbg:
                src, line_no = _resolve_dbg(metadata_index, dbg.group("id"))
            edges.append({
                "caller": cur_fn,
                "callee": "",
                "source_file": src,
                "line": line_no,
                "edge_kind": "unresolved_indirect",
                "backend": "llvm-ir",
                "module": module,
                "notes": f"target=%{i.group('callee_ref')}",
            })
    if in_body:
        warnings.append(f"{path}: unmatched function body at EOF "
                        f"(brace_depth={brace_depth})")
    return functions, edges, warnings


def disassemble_bc(bc: Path, llvm_dis: Path, tmpdir: Path) -> Path:
    """Run `llvm-dis` to produce a sibling `.ll` under tmpdir.

    Returns the path to the produced `.ll`. Raises on failure.
    """
    out = tmpdir / (bc.stem + ".ll")
    subprocess.run([str(llvm_dis), "-o", str(out), str(bc)],
                   check=True, capture_output=True)
    return out


def main():
    p = argparse.ArgumentParser(
        description="Convert textual LLVM IR (`.ll` / `.bc`) into "
                    "the normalized graph format.")
    p.add_argument("--ir-dir", type=Path, required=True,
                   help="Directory containing `.ll` / `.bc` files.")
    p.add_argument("--out-dir", type=Path, required=True,
                   help="Output directory for the normalized graph.")
    p.add_argument("--llvm-dis", default="",
                   help="Path to `llvm-dis`. If empty and `.bc` "
                        "files are present, looked up on PATH.")
    p.add_argument("--source-root", type=Path, default=None,
                   help="Source-tree root used to relativize "
                        "per-file `module` names.")
    p.add_argument("--config-name", default="")
    p.add_argument("--compiler-version", default="")
    p.add_argument("--target-arch", default="")
    p.add_argument("--source-git-sha", default="")
    args = p.parse_args()

    ir_dir = args.ir_dir.resolve()
    if not ir_dir.exists() or not ir_dir.is_dir():
        print(f"ERROR: --ir-dir {ir_dir} is not a directory",
              file=sys.stderr)
        return 2

    ll_files = sorted(ir_dir.rglob("*.ll"))
    bc_files = sorted(ir_dir.rglob("*.bc"))

    warnings: list[str] = []
    llvm_dis = args.llvm_dis or shutil.which("llvm-dis") or ""
    bc_temp = tempfile.mkdtemp(prefix="g15h-bc-")
    bc_temp_path = Path(bc_temp)
    try:
        for bc in bc_files:
            if not llvm_dis:
                warnings.append(
                    f"{bc}: no `llvm-dis` available, skipping bitcode")
                continue
            try:
                ll = disassemble_bc(bc, Path(llvm_dis), bc_temp_path)
                ll_files.append(ll)
            except subprocess.CalledProcessError as e:
                warnings.append(
                    f"{bc}: llvm-dis failed ({e.stderr.decode(errors='replace')[:120]})")

        all_functions: list[dict] = []
        all_edges: list[dict] = []
        for ll in ll_files:
            f, e, w = parse_ll(ll, args.source_root)
            all_functions.extend(f)
            all_edges.extend(e)
            warnings.extend(w)
    finally:
        shutil.rmtree(bc_temp, ignore_errors=True)

    ng.write_functions(args.out_dir, all_functions)
    ng.write_edges(args.out_dir, all_edges)
    ng.write_metadata(
        args.out_dir,
        backend="llvm-ir",
        toolchain="clang",
        compiler_version=args.compiler_version,
        source_git_sha=args.source_git_sha,
        config_name=args.config_name,
        target_arch=args.target_arch,
        inputs=[str(f) for f in ll_files + bc_files],
        warnings=warnings,
    )

    direct = sum(1 for e in all_edges if e["edge_kind"] == "direct")
    indirect = sum(1 for e in all_edges
                   if e["edge_kind"] == "unresolved_indirect")
    intrinsics = sum(1 for e in all_edges if e["notes"] == "intrinsic")
    print(f"llvm-ir -> normalized: "
          f"{len(ll_files)} .ll (+ {len(bc_files)} .bc), "
          f"{len(all_functions)} function rows (before dedup), "
          f"{direct} direct edges, "
          f"{indirect} unresolved_indirect edges "
          f"({intrinsics} intrinsics), "
          f"{len(warnings)} warnings",
          file=sys.stderr)
    print(f"  out_dir: {args.out_dir}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
