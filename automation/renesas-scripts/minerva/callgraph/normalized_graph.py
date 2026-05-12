#!/usr/bin/env python3
"""Backend-neutral normalized callgraph format.

The normalized form is three sibling files under a directory:

  functions.csv  -- every function declaration the backend emitted
                    a node for
  edges.csv      -- every caller  ->  callee edge the backend emitted
  metadata.json  -- provenance + warnings

This module is the read/write/query library for that format. It is
deliberately simple and dependency-free; both the GCC `.ci`
adapter and the LLVM IR extractor reuse the same writers, and the
reachability workbench reuses the same readers and the
target-tree query.

Schema
------

`functions.csv`:

  function, source_file, line, module, backend,
  linkage, visibility, notes

Minimum required columns: function, source_file, line, module,
backend. The trailing three are optional and may be empty.

`edges.csv`:

  caller, callee, source_file, line, edge_kind, backend, module, notes

`edge_kind` is one of:

  direct               -- caller statically calls a named callee
  unresolved_indirect  -- caller calls through a function pointer
                          / virtual dispatch; callee is empty
  synthetic            -- added by an analyst pass or augmentation
                          script (not produced by a backend adapter
                          directly)

`metadata.json`:

  {
    "backend": "gcc-ci" | "llvm-ir",
    "toolchain": "...",
    "compiler_version": "...",
    "source_git_sha": "...",
    "config_name": "...",
    "target_arch": "...",
    "inputs": [...],
    "generated_at": "...",
    "warnings": [...]
  }

No claim is made that the graph is exhaustive. `path_found=no`
from a reachability query against this graph does not mean
impossible.
"""

from __future__ import annotations

import csv
import json
from collections import defaultdict, deque
from datetime import datetime, timezone
from pathlib import Path


FUNCTION_COLUMNS = (
    "function", "source_file", "line", "module", "backend",
    "linkage", "visibility", "notes",
)
EDGE_COLUMNS = (
    "caller", "callee", "source_file", "line", "edge_kind",
    "backend", "module", "notes",
)
EDGE_KINDS = ("direct", "unresolved_indirect", "synthetic")


def _now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


# ----- I/O ---------------------------------------------------------------


def load_functions(path: Path) -> list[dict]:
    """Read functions.csv. Missing columns are filled with ''."""
    p = Path(path)
    if p.is_dir():
        p = p / "functions.csv"
    if not p.exists():
        return []
    with p.open(newline="", encoding="utf-8") as fh:
        return [
            {col: (row.get(col) or "") for col in FUNCTION_COLUMNS}
            for row in csv.DictReader(fh)
        ]


def load_edges(path: Path) -> list[dict]:
    """Read edges.csv. Missing columns are filled with ''."""
    p = Path(path)
    if p.is_dir():
        p = p / "edges.csv"
    if not p.exists():
        return []
    with p.open(newline="", encoding="utf-8") as fh:
        return [
            {col: (row.get(col) or "") for col in EDGE_COLUMNS}
            for row in csv.DictReader(fh)
        ]


def write_functions(out_dir: Path, functions: list[dict]) -> Path:
    """Write functions.csv. Rows are sorted by (function, source_file,
    line) for deterministic output and deduplicated on that key."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    p = out_dir / "functions.csv"
    seen: set[tuple[str, str, str]] = set()
    deduped: list[dict] = []
    for r in functions:
        key = (r.get("function", ""), r.get("source_file", ""),
               str(r.get("line", "")))
        if key in seen:
            continue
        seen.add(key)
        deduped.append(r)
    deduped.sort(key=lambda r: (r.get("function", ""),
                                r.get("source_file", ""),
                                str(r.get("line", ""))))
    with p.open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=FUNCTION_COLUMNS)
        w.writeheader()
        for r in deduped:
            w.writerow({col: r.get(col, "") for col in FUNCTION_COLUMNS})
    return p


def write_edges(out_dir: Path, edges: list[dict]) -> Path:
    """Write edges.csv. Rows are deduplicated on
    (caller, callee, edge_kind, source_file, line) and sorted."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    p = out_dir / "edges.csv"
    seen: set[tuple[str, str, str, str, str]] = set()
    deduped: list[dict] = []
    for r in edges:
        key = (r.get("caller", ""), r.get("callee", ""),
               r.get("edge_kind", ""), r.get("source_file", ""),
               str(r.get("line", "")))
        if key in seen:
            continue
        seen.add(key)
        deduped.append(r)
    deduped.sort(key=lambda r: (r.get("caller", ""),
                                r.get("callee", ""),
                                r.get("edge_kind", ""),
                                r.get("source_file", ""),
                                str(r.get("line", ""))))
    with p.open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=EDGE_COLUMNS)
        w.writeheader()
        for r in deduped:
            w.writerow({col: r.get(col, "") for col in EDGE_COLUMNS})
    return p


def write_metadata(out_dir: Path, **fields) -> Path:
    """Write metadata.json. `backend` is required; all other fields
    are optional and stored verbatim."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    p = out_dir / "metadata.json"
    if "backend" not in fields:
        raise ValueError("metadata.backend is required")
    fields.setdefault("generated_at", _now())
    p.write_text(json.dumps(fields, indent=2, sort_keys=True),
                 encoding="utf-8")
    return p


# ----- Queries -----------------------------------------------------------


def build_reverse_index(edges: list[dict],
                        kinds: tuple[str, ...] = ("direct", "synthetic")
                        ) -> dict[str, set[str]]:
    """Map callee  ->  set of direct callers (filtered by edge_kind).

    `unresolved_indirect` edges are dropped by default because they
    have no resolved callee. A caller pass that has resolved a
    function pointer should emit `synthetic` edges with the resolved
    callee.
    """
    idx: dict[str, set[str]] = defaultdict(set)
    for e in edges:
        if e.get("edge_kind") not in kinds:
            continue
        callee = e.get("callee", "")
        caller = e.get("caller", "")
        if not callee or not caller:
            continue
        idx[callee].add(caller)
    return idx


def functions_reaching_target(edges: list[dict], target: str,
                              kinds: tuple[str, ...] = ("direct", "synthetic")
                              ) -> set[str]:
    """Return the set of functions that statically reach `target`.

    BFS over the reverse callgraph. Cycles and duplicate edges are
    handled by the visited set. Returns a set including `target`
    itself when an edge reaches it.
    """
    reverse = build_reverse_index(edges, kinds=kinds)
    reachable: set[str] = set()
    if target in reverse or any(e.get("callee") == target for e in edges
                                if e.get("edge_kind") in kinds):
        reachable.add(target)
    frontier: deque[str] = deque([target])
    while frontier:
        cur = frontier.popleft()
        for caller in reverse.get(cur, ()):
            if caller not in reachable:
                reachable.add(caller)
                frontier.append(caller)
    return reachable


def direct_call_tree_to_target(edges: list[dict], target: str,
                               kinds: tuple[str, ...] = ("direct", "synthetic"),
                               max_depth: int = 64
                               ) -> list[list[str]]:
    """Return a list of (caller chain) paths ending at `target`.

    Each path is a list of function names from a leaf-caller (a
    function with no callers in the filtered graph) down to
    `target`. Depth is bounded to avoid pathological output on dense
    graphs. Cycles are broken by refusing to revisit a node in the
    same path.
    """
    reverse = build_reverse_index(edges, kinds=kinds)
    paths: list[list[str]] = []

    def walk(cur: str, suffix: list[str]):
        if len(suffix) > max_depth:
            return
        callers = reverse.get(cur, set())
        if not callers:
            paths.append([cur, *suffix])
            return
        for c in sorted(callers):
            if c in suffix or c == cur:
                # cycle break
                continue
            walk(c, [cur, *suffix])

    walk(target, [])
    return paths
