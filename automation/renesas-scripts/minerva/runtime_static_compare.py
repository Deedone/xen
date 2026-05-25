#!/usr/bin/env python3
"""Compare runtime-observed allocation paths against static artifacts.

This tool closes the runtime/static loop: it takes the parsed runtime
allocation paths produced by the runtime log parser and checks each
one against the static artifacts generated from the *same* expanded
Xen config and source revision -- the direct-static target trees, the
indirect reachability candidates, and the table-aware synthetic-edge
candidates.

It makes no claim about allocation bounds, about `path_found=no`
meaning impossible, or about runtime non-observation meaning
impossible. A runtime path that no static artifact explains is
reported as such; a static candidate not seen at runtime is reported
as `not_observed_in_this_workload`, never as dead.

Matching is layered, and every classification records the coarsest
layer it relied on via a `comparison_confidence` field so nothing is
overclaimed:

    exact_stack     full ordered frame sequence matched
    function_set    the set of functions matched, order ignored
    head_function   only the innermost (allocating) frame matched
    target_only     only the allocation target matched

Runtime-path classifications:
    direct_static_explained          reaches an alloc target through
                                     a direct-static call tree
    indirect_candidate_explained     explained only via an indirect
                                     reachability / synthetic-edge
                                     candidate
    runtime_only_unexplained         no static artifact explains it
    boundary_or_parser_artifact      empty / degenerate path, or a
                                     known parser boundary case
    unresolved_normalization_mismatch
                                     frames present but no normalized
                                     form lined up (diagnostic bucket)

Static-candidate classifications:
    observed_at_runtime
    not_observed_in_this_workload    (NOT "impossible")
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import sys
from pathlib import Path


# --------------------------------------------------------------------
# Input loading. The runtime parser's parsed output is read
# defensively: we accept either a JSON file of allocation paths or a
# directory of per-path JSON / CSV. Each runtime path is normalized to
# a dict: {"frames": [fn, ...], "target": fn_or_None, "domain": str,
# "raw": ...}. frames are ordered outermost -> innermost (the
# allocation call is the last frame).
# --------------------------------------------------------------------

DEFAULT_TARGETS = ["alloc_domheap_pages", "alloc_xenheap_pages", "_xmalloc"]


def _coerce_path_record(obj: dict) -> dict | None:
    """Normalize one parsed runtime allocation record."""
    frames = obj.get("frames") or obj.get("stack") or obj.get("path")
    if frames is None and "call_path" in obj:
        frames = obj["call_path"]
    if isinstance(frames, str):
        # "a;b;c" or "a -> b -> c"
        sep = ";" if ";" in frames else "->"
        frames = [f.strip() for f in frames.split(sep) if f.strip()]
    if not isinstance(frames, list):
        return None
    frames = [str(f).strip() for f in frames if str(f).strip()]
    target = obj.get("target") or obj.get("allocation_target")
    if not target and frames:
        # Innermost frame that looks like an allocation entry point.
        for f in reversed(frames):
            if "alloc" in f or f == "_xmalloc":
                target = f
                break
    domain = str(obj.get("domain") or obj.get("dom") or "").strip()
    return {"frames": frames, "target": target or "",
            "domain": domain, "raw": obj}


def load_runtime_paths(parsed_dir: Path,
                       known_targets: set | None = None) -> list[dict]:
    """Load parsed runtime allocation paths from a directory.

    Accepts, in order of preference:
      - parsed_dir/allocation-paths.json  (list of records)
      - parsed_dir/*.json                 (each a record or list)
      - parsed_dir/allocation-paths.csv   (frames in a `frames`/`path`
                                           column, ; or -> separated)

    known_targets, when provided, filters the text-report fallback so
    only records whose allocating function is a known allocation
    target become runtime paths -- a free-path, comment, or diagnostic
    section shaped like a header is not mistaken for an allocation.
    """
    paths: list[dict] = []
    if not parsed_dir.exists():
        return paths

    agg = parsed_dir / "allocation-paths.json"
    if agg.exists():
        try:
            data = json.loads(agg.read_text())
            records = data if isinstance(data, list) else data.get(
                "paths", [])
            for obj in records:
                rec = _coerce_path_record(obj)
                if rec:
                    paths.append(rec)
            return paths
        except (ValueError, OSError):
            pass

    csv_path = parsed_dir / "allocation-paths.csv"
    if csv_path.exists():
        with csv_path.open() as fh:
            for row in csv.DictReader(fh):
                rec = _coerce_path_record(row)
                if rec:
                    paths.append(rec)
        return paths

    for jf in sorted(parsed_dir.glob("*.json")):
        try:
            data = json.loads(jf.read_text())
        except (ValueError, OSError):
            continue
        records = data if isinstance(data, list) else [data]
        for obj in records:
            if isinstance(obj, dict):
                rec = _coerce_path_record(obj)
                if rec:
                    paths.append(rec)
    if paths:
        return paths

    # Fallback: the runtime log parser's text output. The parser emits
    # a `comments` report (and per-domain text reports) where each
    # allocation record is a header line naming the allocating
    # function followed by an indented call-path. We read those rather
    # than require a structured export, so the comparator consumes the
    # real parser artifact. A structured allocation-paths.json (when
    # the parser or driver provides one) is always preferred above.
    paths.extend(_load_parser_text_reports(parsed_dir, known_targets))
    return paths


# Parser text-report shapes. A record header names the allocating
# function and optionally a domain; following indented lines are the
# call-path frames (outermost first), each an identifier optionally
# followed by `@ file:line`.
_TEXT_HEADER_RE = re.compile(
    r"^(?P<fn>[A-Za-z_][A-Za-z0-9_]*)\(\)"
    r"(?:.*\bdom(?:ain)?[ =:]+(?P<dom>\S+))?")
_TEXT_FRAME_RE = re.compile(
    r"^\s+(?P<fn>[A-Za-z_][A-Za-z0-9_]*)\b")


def _load_parser_text_reports(parsed_dir: Path,
                              known_targets: set | None = None
                              ) -> list[dict]:
    """Read the parser's text `comments`/report output into records.

    Only records whose header (allocating) function is a known
    allocation target are emitted. A header that is not an allocation
    target -- a free-path entry like `free_domheap_pages()`, a comment,
    or a diagnostic section -- is skipped, so non-allocation report
    sections do not become phantom runtime paths with unknown targets.
    """
    known = set(known_targets) if known_targets else set(DEFAULT_TARGETS)
    out: list[dict] = []
    candidates: list[Path] = []
    for name in ("comments", "comments.txt"):
        p = parsed_dir / name
        if p.exists():
            candidates.append(p)
    candidates += sorted(parsed_dir.glob("*.txt"))
    candidates += sorted(parsed_dir.glob("*.report"))
    for rep in candidates:
        try:
            text = rep.read_text(errors="replace")
        except OSError:
            continue
        cur: dict | None = None
        for raw in text.splitlines():
            if not raw.strip():
                continue
            mh = _TEXT_HEADER_RE.match(raw)
            mf = _TEXT_FRAME_RE.match(raw)
            # A header line starts at column 0; frame lines are
            # indented. Prefer the header interpretation at column 0.
            if mh and not raw[:1].isspace():
                if cur and cur["frames"]:
                    out.append(cur)
                fn = mh.group("fn")
                if fn not in known:
                    # Not an allocation target -- skip this record and
                    # its frames entirely.
                    cur = None
                    continue
                dom = mh.group("dom") or ""
                cur = {"frames": [], "target": fn, "domain": dom,
                       "raw": raw, "_head": fn}
            elif mf and cur is not None:
                cur["frames"].append(mf.group("fn"))
        if cur and cur["frames"]:
            out.append(cur)
    # Ensure the allocating function is the last (innermost) frame.
    for rec in out:
        head = rec.pop("_head", "")
        if head and (not rec["frames"] or rec["frames"][-1] != head):
            rec["frames"].append(head)
    return out


# --------------------------------------------------------------------
# Static artifact loading.
# --------------------------------------------------------------------

def load_direct_static(direct_static_dir: Path) -> dict[str, set]:
    """Return {target -> set(functions reaching it)} from direct-static.

    Reads `<target>.functions` (flat, one function per line, the
    normalized/llvm-ir backend) and/or `<target>.paths` (the gcc-ci
    nested-tree format) for each target.
    """
    out: dict[str, set] = {}
    if not direct_static_dir.exists():
        return out
    for fn_file in sorted(direct_static_dir.glob("*.functions")):
        target = fn_file.stem
        fns = {ln.strip() for ln in fn_file.read_text().splitlines()
               if ln.strip()}
        out.setdefault(target, set()).update(fns)
    for pf in sorted(direct_static_dir.glob("*.paths")):
        target = pf.stem
        fns = out.setdefault(target, set())
        for ln in pf.read_text().splitlines():
            tok = ln.strip().split("@")[0].strip()
            # leading identifier of the (possibly indented) tree line
            ident = tok.split()[0] if tok else ""
            if ident and all(c.isalnum() or c == "_" for c in ident):
                fns.add(ident)
    return out


def load_indirect_candidates(reachability_dir: Path
                             ) -> tuple[set, dict]:
    """Return (candidate_impls, per_impl_reaches) from reachability.

    Reads synthetic_edges.candidates.yaml (the included candidates).
    candidate_impls is the set of implementation functions; reaches
    maps impl -> set(targets) where recorded.
    """
    impls: set = set()
    reaches: dict[str, set] = {}
    yaml_path = reachability_dir / "synthetic_edges.candidates.yaml"
    if not yaml_path.exists():
        return impls, reaches
    cur_impl = None
    in_reaches = False
    for raw in yaml_path.read_text().splitlines():
        line = raw.rstrip()
        s = line.strip()
        if s.startswith("implementation:"):
            cur_impl = s.split(":", 1)[1].strip()
            if cur_impl:
                impls.add(cur_impl)
                reaches.setdefault(cur_impl, set())
            in_reaches = False
        elif s == "reaches:":
            in_reaches = True
        elif in_reaches and s.startswith("- "):
            tgt = s[2:].strip()
            if cur_impl and tgt:
                reaches.setdefault(cur_impl, set()).add(tgt)
        elif s.startswith("- call_site_function:"):
            in_reaches = False
            cur_impl = None
    return impls, reaches


def load_allocation_targets(direct_static: dict[str, set],
                            explicit: list[str]) -> list[str]:
    targets = list(direct_static.keys())
    for t in explicit:
        if t not in targets:
            targets.append(t)
    return targets or list(DEFAULT_TARGETS)


# --------------------------------------------------------------------
# Matching.
# --------------------------------------------------------------------

def matching_indirect_impls_for_target(frames, target, indirect_impls,
                                       indirect_reaches) -> set:
    """Indirect candidate impls in `frames` that reach `target`.

    Empty when the target is unknown -- an unconfirmed-reach frame
    match is not counted as a matched (observed) candidate.
    """
    frame_impls = set(frames) & indirect_impls
    if not target:
        return set()
    return {impl for impl in frame_impls
            if target in indirect_reaches.get(impl, set())}


def classify_runtime_path(rec: dict, direct_static: dict[str, set],
                          indirect_impls: set,
                          indirect_reaches: dict,
                          targets: list[str]
                          ) -> tuple[str, str, str, set]:
    """Classify one runtime path.

    Returns (classification, confidence, basis, matched_impls) where
    matched_impls is the set of indirect candidate impls that actually
    explained the path (reach its target). matched_impls is empty for
    every non-indirect classification, so observed-candidate
    accounting cannot over-count.
    """
    frames = rec["frames"]
    target = rec["target"]
    if not frames:
        return ("boundary_or_parser_artifact", "target_only",
                "empty frame list", set())

    fnset = set(frames)
    head = frames[-1]
    # Frames other than the allocation target itself. The target frame
    # appearing in a path is not evidence of a *direct* path -- every
    # allocation path ends at the target -- so direct-static matching
    # must rely on a non-target frame being in the reaching set.
    non_target_frames = {f for f in frames if f != target}

    # 1. Direct-static explanation: a non-target frame of the path is
    #    in the direct-static reaching set of the path's target.
    ds_target = target if target in direct_static else None
    if ds_target is None:
        for t in targets:
            if t in direct_static and (non_target_frames
                                       & direct_static[t]):
                ds_target = t
                break
    indirect_inter = fnset & indirect_impls
    if ds_target is not None and (non_target_frames
                                  & direct_static[ds_target]):
        reaching = direct_static[ds_target]
        # Prefer indirect attribution when the only matching frame is
        # itself a known indirect candidate impl (the dispatch went
        # through a function pointer, not a direct call).
        ds_frames = non_target_frames & reaching
        if ds_frames and ds_frames <= indirect_impls and indirect_inter:
            pass  # fall through to indirect classification
        elif all(f in reaching for f in non_target_frames):
            return ("direct_static_explained", "function_set",
                    f"all non-target frames in direct-static reaching "
                    f"set of {ds_target}", set())
        else:
            return ("direct_static_explained", "function_set",
                    f"frame(s) in direct-static reaching set of "
                    f"{ds_target}", set())

    # 2. Indirect candidate explanation. A candidate impl explains the
    #    path only if it actually reaches the path's target (per the
    #    recorded synthetic-edge reaches). A frame match without a
    #    confirmed target reach is NOT an explanation: an unknown
    #    target, or a candidate that reaches only some other target,
    #    leaves the path unexplained / unresolved rather than
    #    optimistically "explained".
    frame_impls = fnset & indirect_impls
    if frame_impls:
        target_matching = matching_indirect_impls_for_target(
            frames, target, indirect_impls, indirect_reaches)
        if target_matching:
            if head in target_matching:
                return ("indirect_candidate_explained", "head_function",
                        f"head {head} is an indirect candidate "
                        f"reaching {target}", target_matching)
            return ("indirect_candidate_explained", "function_set",
                    f"frame(s) {sorted(target_matching)[:3]} are "
                    f"indirect candidates reaching {target}",
                    target_matching)
        if not target:
            # Target unknown: a frame is a candidate impl, but we
            # cannot confirm it reaches this path's target. This is a
            # normalization gap, not an explanation, and is not
            # counted as an observed candidate.
            return ("unresolved_normalization_mismatch", "target_only",
                    f"frame(s) {sorted(frame_impls)[:3]} are indirect "
                    f"candidates but the path target is unknown; reach "
                    f"unconfirmed", set())
        # Frames are candidates but none reaches this target.
        return ("runtime_only_unexplained", "function_set",
                f"frame(s) {sorted(frame_impls)[:3]} are indirect "
                f"candidates but none reaches {target}", set())

    # 3. Target known but no reaching explanation lined up.
    if target and target in targets:
        return ("unresolved_normalization_mismatch", "target_only",
                f"target {target} known but no frame matched a static "
                f"reaching set", set())

    # 4. Nothing explains it.
    return ("runtime_only_unexplained", "function_set",
            "no direct-static or indirect candidate matched", set())


# --------------------------------------------------------------------
# Comparison driver.
# --------------------------------------------------------------------

RUNTIME_CLASSES = [
    "direct_static_explained",
    "indirect_candidate_explained",
    "runtime_only_unexplained",
    "boundary_or_parser_artifact",
    "unresolved_normalization_mismatch",
]


def _config_sha256(config_path: Path | None) -> str:
    if not config_path or not config_path.exists():
        return ""
    import hashlib
    return hashlib.sha256(config_path.read_bytes()).hexdigest()


def compare(runtime_parsed: Path, reachability_dir: Path,
            direct_static_dir: Path, collect_dir: Path,
            config_path: Path | None, out_dir: Path,
            config_name: str = "", git_sha: str = "",
            targets: list[str] | None = None) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    direct_static = load_direct_static(direct_static_dir)
    indirect_impls, indirect_reaches = load_indirect_candidates(
        reachability_dir)
    target_list = load_allocation_targets(direct_static, targets or [])
    # Load runtime paths with the known allocation targets so the
    # text-report fallback only emits genuine allocation records.
    runtime_paths = load_runtime_paths(runtime_parsed,
                                       known_targets=set(target_list))

    # Cross-check against the collector run's recorded metadata, when
    # available. A config_name disagreement is recorded as a note so a
    # reader can see the comparison was run over a collector output
    # from a different configuration than the caller claimed.
    collect_notes: list[str] = []
    collect_summary = collect_dir / "collection-summary.json"
    if collect_summary.exists():
        try:
            cs = json.loads(collect_summary.read_text())
            cs_name = str(cs.get("config_name", ""))
            if config_name and cs_name and cs_name != config_name:
                collect_notes.append(
                    f"collector config_name {cs_name!r} differs from "
                    f"comparison config_name {config_name!r}")
        except (ValueError, OSError):
            collect_notes.append("collector summary unreadable")

    # Classify runtime paths.
    runtime_rows: list[dict] = []
    coarsest = "exact_stack"
    confidence_rank = {"exact_stack": 0, "function_set": 1,
                       "head_function": 2, "target_only": 3}
    observed_impls: set = set()
    for rec in runtime_paths:
        cls, conf, basis, matched = classify_runtime_path(
            rec, direct_static, indirect_impls, indirect_reaches,
            target_list)
        if confidence_rank[conf] > confidence_rank[coarsest]:
            coarsest = conf
        # Only candidates that actually explained this path (reach its
        # target) count as observed -- never every indirect frame.
        if cls == "indirect_candidate_explained":
            observed_impls |= matched
        runtime_rows.append({
            "classification": cls,
            "confidence": conf,
            "target": rec["target"],
            "domain": rec["domain"],
            "head_function": rec["frames"][-1] if rec["frames"] else "",
            "frame_count": len(rec["frames"]),
            "basis": basis,
        })

    counts = {c: sum(1 for r in runtime_rows
                     if r["classification"] == c)
              for c in RUNTIME_CLASSES}

    # Static-candidate observation.
    candidates_observed = sorted(observed_impls)
    candidates_not_observed = sorted(indirect_impls - observed_impls)

    # Write per-path audit CSVs.
    unmatched = [r for r in runtime_rows
                 if r["classification"] in
                 ("runtime_only_unexplained",
                  "unresolved_normalization_mismatch")]
    _write_csv(out_dir / "runtime-unmatched-paths.csv",
               ["classification", "confidence", "target", "domain",
                "head_function", "frame_count", "basis"], unmatched)

    static_only_rows = [{"implementation": i,
                         "status": "not_observed_in_this_workload"}
                        for i in candidates_not_observed]
    _write_csv(out_dir / "static-only-indirect-paths.csv",
               ["implementation", "status"], static_only_rows)

    summary_rows = [{"metric": k, "value": v} for k, v in counts.items()]
    summary_rows += [
        {"metric": "runtime_paths_total", "value": len(runtime_rows)},
        {"metric": "indirect_candidates_total",
         "value": len(indirect_impls)},
        {"metric": "indirect_candidates_observed",
         "value": len(candidates_observed)},
        {"metric": "indirect_candidates_not_observed",
         "value": len(candidates_not_observed)},
        {"metric": "direct_static_targets_present",
         "value": len([t for t in direct_static if direct_static[t]])},
    ]
    _write_csv(out_dir / "comparison-summary.csv",
               ["metric", "value"], summary_rows)

    # The comparison tool reports only that a comparison was performed
    # (comparison_status=COMPARED). It does NOT infer the run-level
    # STATIC_ONLY / COMPLETE / PROXY / PARTIAL label -- that is the
    # driver's job, decided from runtime-stage success and manifest
    # alignment. A comparison over zero runtime paths is still a valid
    # comparison result (runtime_paths_total=0), not STATIC_ONLY.
    comparison_status = "COMPARED"

    metrics = {
        "comparison_status": comparison_status,
        "config_name": config_name,
        "git_sha": git_sha,
        "config_sha256": _config_sha256(config_path),
        "runtime_paths_total": len(runtime_rows),
        "runtime_paths_direct_static_explained":
            counts["direct_static_explained"],
        "runtime_paths_indirect_explained":
            counts["indirect_candidate_explained"],
        "runtime_paths_unexplained":
            counts["runtime_only_unexplained"],
        "runtime_paths_boundary_or_parser_artifact":
            counts["boundary_or_parser_artifact"],
        "runtime_paths_unresolved_normalization_mismatch":
            counts["unresolved_normalization_mismatch"],
        "indirect_candidates_total": len(indirect_impls),
        "indirect_candidates_observed": len(candidates_observed),
        "indirect_candidates_not_observed":
            len(candidates_not_observed),
        "direct_static_targets_present":
            len([t for t in direct_static if direct_static[t]]),
        "comparison_confidence": coarsest if runtime_paths else "n/a",
        "notes": [
            "No allocation bound is claimed.",
            "path_found=no does not mean impossible.",
            "not_observed_in_this_workload does not mean impossible.",
            "Static reachability does not mean runtime exercised.",
            "Metrics are per-run artifacts, not committed constants.",
        ] + collect_notes,
    }
    (out_dir / "runtime-static-comparison.json").write_text(
        json.dumps(metrics, indent=2), encoding="utf-8")
    _write_markdown(out_dir / "runtime-static-comparison.md", metrics,
                    counts)
    return metrics


def _write_csv(path: Path, columns: list[str], rows: list[dict]):
    with path.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=columns)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in columns})


def _write_markdown(path: Path, metrics: dict, counts: dict):
    lines = [
        "# Runtime/static comparison",
        "",
        f"Comparison status: **{metrics['comparison_status']}**  ",
        f"Config: `{metrics['config_name']}`  ",
        f"git SHA: `{metrics['git_sha']}`  ",
        f"Comparison confidence (coarsest layer relied on): "
        f"`{metrics['comparison_confidence']}`",
        "",
        "This file is a per-run artifact. The numbers below describe "
        "one CI run and one workload; they are not committed "
        "constants and not a coverage figure.",
        "",
        "## Runtime path classification",
        "",
        "| Class | Count |",
        "| --- | ---: |",
    ]
    for c in RUNTIME_CLASSES:
        lines.append(f"| `{c}` | {counts[c]} |")
    lines += [
        f"| **total** | {metrics['runtime_paths_total']} |",
        "",
        "## Indirect static candidates",
        "",
        f"- total: {metrics['indirect_candidates_total']}",
        f"- observed at runtime: "
        f"{metrics['indirect_candidates_observed']}",
        f"- not observed in this workload: "
        f"{metrics['indirect_candidates_not_observed']}",
        "",
        "## Caveats",
        "",
        "- No allocation bound is claimed.",
        "- `path_found=no` does not mean impossible.",
        "- `not_observed_in_this_workload` does not mean impossible; "
        "it means this workload did not exercise it.",
        "- Static reachability does not mean the path was exercised "
        "at runtime.",
        "- Coarse matching (function-set / head-function / "
        "target-only) is reported in `comparison_confidence`; an "
        "explanation at a coarse layer is not an exact-stack match.",
        "",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    p = argparse.ArgumentParser(
        description="Compare runtime-observed allocation paths against "
                    "static reachability artifacts.")
    p.add_argument("--runtime-parsed", type=Path, required=True)
    p.add_argument("--reachability-dir", type=Path, required=True)
    p.add_argument("--direct-static-dir", type=Path, required=True)
    p.add_argument("--collect-dir", type=Path, default=None)
    p.add_argument("--config", type=Path, default=None)
    p.add_argument("--out-dir", type=Path, required=True)
    p.add_argument("--config-name", default="")
    p.add_argument("--git-sha", default="")
    p.add_argument("--targets", nargs="+", default=None)
    args = p.parse_args()

    metrics = compare(
        args.runtime_parsed, args.reachability_dir,
        args.direct_static_dir,
        args.collect_dir or Path("."), args.config, args.out_dir,
        config_name=args.config_name, git_sha=args.git_sha,
        targets=args.targets)
    print(json.dumps(metrics, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
