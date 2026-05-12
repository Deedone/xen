#!/usr/bin/env python3
"""Convert GCC `.ci` callgraph files to the normalized graph format.

GCC's `-fcallgraph-info=su` emits one `.ci` file per translation
unit. Each file contains VCG-style `node:`/`edge:` declarations.
`minerva_static_analysis/callpath.py` parses these directly; this
adapter applies the same regex parsing but writes the result to
the backend-neutral normalized form so consumers can stay
unaware of which compiler produced the graph.

Output: functions.csv, edges.csv, metadata.json under --out-dir,
per `scripts/callgraph/normalized_graph.py`.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

# Allow this script to be run directly without installing the package.
_SELF = Path(__file__).resolve()
sys.path.insert(0, str(_SELF.parent.parent))
from callgraph import normalized_graph as ng  # noqa: E402


# GCC `.ci` parsing. The same flag (-fcallgraph-info=...) emits
# different label shapes depending on whether `su` is appended:
#
#   -fcallgraph-info       label: "<name>\n<file:line:col>"
#   -fcallgraph-info=su    label: "<name>\n<file:line:col>\n<N> bytes (...)"
#
# Optional ` shape : ellipse` suffix appears on `node:` lines for
# nodes that have no source-tree definition (built-ins, external
# decls). Match the common prefix and treat both the stack-usage
# and the shape suffix as optional.
NODE_RE = re.compile(
    r"node: \{ title: \".+?\" label: \"(?P<name>\w+)\\n"
    r"(?P<location>[^\"]+?)"
    r"(?:\\n(?P<stack_usage>\d+) bytes \([^\)]*\))?"
    r"\"(?: shape : \w+)? \}"
)
EDGE_RE = re.compile(
    r"edge: \{ sourcename: \"(.+?:)?(?P<caller>.+?)\" "
    r"targetname: \"(.+?:)?(?P<called>.+?)\" "
    r"label: \"(?P<location>.+?)\" \}"
)


def _split_location(loc: str) -> tuple[str, str]:
    """`xen/common/page_alloc.c:2460:19` -> (`xen/common/page_alloc.c`, `2460`).

    Some labels carry a trailing column; some don't. Best effort.
    """
    if not loc:
        return "", ""
    parts = loc.rsplit(":", 2)
    if len(parts) >= 2 and parts[-2].isdigit():
        return parts[0], parts[-2]
    if len(parts) >= 1 and parts[-1].isdigit():
        return ":".join(parts[:-1]), parts[-1]
    return loc, ""


def parse_ci_file(path: Path, ci_root: Path) -> tuple[list[dict], list[dict]]:
    """Return (functions, edges) for one .ci file.

    `ci_root` is the top-level directory used as a module prefix
    when relativizing per-file module names.
    """
    text = path.read_text(errors="replace")
    try:
        module = str(path.relative_to(ci_root)).replace("\\", "/")
    except ValueError:
        module = path.name

    functions: list[dict] = []
    edges: list[dict] = []
    seen_fn_keys: set[tuple[str, str, str]] = set()

    def _add_fn(name: str, src: str, line: str, note: str = ""):
        if not name:
            return
        key = (name, src, line)
        if key in seen_fn_keys:
            return
        seen_fn_keys.add(key)
        functions.append({
            "function": name, "source_file": src, "line": line,
            "module": module, "backend": "gcc-ci",
            "linkage": "", "visibility": "", "notes": note,
        })

    for m in NODE_RE.finditer(text):
        src, line = _split_location(m.group("location"))
        # `<built-in>` and similar synthetic locations yield empty
        # source/line under _split_location.
        if src.startswith("<") and src.endswith(">"):
            note = f"node-location-marker={src}"
            src, line = "", ""
        else:
            note = ""
        _add_fn(m.group("name"), src, line, note)
    for m in EDGE_RE.finditer(text):
        src, line = _split_location(m.group("location") or "")
        caller = m.group("caller")
        callee = m.group("called")
        edges.append({
            "caller": caller, "callee": callee,
            "source_file": src, "line": line,
            "edge_kind": "direct", "backend": "gcc-ci",
            "module": module, "notes": "",
        })
        # Functions referenced only via edges (typical for nodes the
        # NODE_RE pattern doesn't cover, e.g. external decls without
        # a `label:` carrying source/line) still need an entry in
        # functions.csv so a consumer can iterate the function set.
        _add_fn(caller, "", "", note="edge-endpoint-only")
        _add_fn(callee, "", "", note="edge-endpoint-only")
    return functions, edges


def main():
    p = argparse.ArgumentParser(
        description="Convert GCC `.ci` callgraph output to the "
                    "normalized graph format.")
    p.add_argument("--ci-dir", type=Path, required=True,
                   help="Directory containing GCC `.ci` files (rglob).")
    p.add_argument("--out-dir", type=Path, required=True,
                   help="Output directory for the normalized graph.")
    p.add_argument("--config-name", default="",
                   help="Symbolic configuration name to record in "
                        "metadata.")
    p.add_argument("--compiler-version", default="",
                   help="Compiler version string to record in "
                        "metadata.")
    p.add_argument("--source-git-sha", default="",
                   help="Git SHA of the analysed source tree.")
    p.add_argument("--target-arch", default="",
                   help="Target architecture string.")
    args = p.parse_args()

    ci_dir = args.ci_dir.resolve()
    if not ci_dir.exists() or not ci_dir.is_dir():
        print(f"ERROR: --ci-dir {ci_dir} is not a directory",
              file=sys.stderr)
        return 2
    ci_files = sorted(ci_dir.rglob("*.ci"))
    if not ci_files:
        print(f"ERROR: no .ci files under {ci_dir}", file=sys.stderr)
        return 2

    all_functions: list[dict] = []
    all_edges: list[dict] = []
    warnings: list[str] = []
    for f in ci_files:
        try:
            funcs, edges = parse_ci_file(f, ci_dir)
        except OSError as e:
            warnings.append(f"could not read {f}: {e}")
            continue
        all_functions.extend(funcs)
        all_edges.extend(edges)

    ng.write_functions(args.out_dir, all_functions)
    ng.write_edges(args.out_dir, all_edges)
    ng.write_metadata(
        args.out_dir,
        backend="gcc-ci",
        toolchain="gcc",
        compiler_version=args.compiler_version,
        source_git_sha=args.source_git_sha,
        config_name=args.config_name,
        target_arch=args.target_arch,
        inputs=[str(f) for f in ci_files],
        warnings=warnings,
    )

    # Stable counts on stderr so a caller can capture them without
    # re-parsing the CSVs.
    print(f"gcc-ci -> normalized: {len(ci_files)} .ci files, "
          f"{len(all_functions)} function rows (before dedup), "
          f"{len(all_edges)} edge rows (before dedup)",
          file=sys.stderr)
    print(f"  out_dir: {args.out_dir}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
