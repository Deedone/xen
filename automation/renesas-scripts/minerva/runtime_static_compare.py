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

# Warnings accumulated while reading parser text reports (e.g. a
# malformed report that was skipped rather than allowed to crash the
# comparator). Reset at the start of each compare() and surfaced in the
# comparison notes.
_PARSE_REPORT_WARNINGS: list[str] = []

# Allocator-layer aliases: runtime label observed -> canonical static
# target the analysis trees are framed around.
#
# The CONFIG_MINERVA_ANALYSIS instrumentation in the Xen allocator
# emits a debug line whose label is taken from the inner helper being
# invoked, while the static target trees are keyed on the enclosing
# allocation entry point. In particular the WARN inside _xmalloc()
# (xen/common/xmalloc_tlsf.c) prints "xmem_pool_alloc - size ..."
# because that is the helper _xmalloc is about to call, so the runtime
# head frame is "xmem_pool_alloc" while the static target is
# "_xmalloc". They are the same allocation site, named at different
# layers.
#
# This map is intentionally explicit and small: it is NOT fuzzy
# matching. Each entry corresponds to a known instrumentation label
# whose enclosing function differs from the printed name. Frames or
# targets not listed here are left unchanged. When the analysis adds a
# new instrumented allocator whose label differs from its target,
# add the pair here with a source reference.
ALLOCATOR_ALIASES = {
    # runtime label        : canonical static target
    "xmem_pool_alloc": "_xmalloc",   # WARN inside _xmalloc(), tlsf.c
}

# Allocator-layer plumbing that is not a caller. A free running between
# two allocations (e.g. _xmalloc -> xfree -> _xmalloc) is interleaved
# deallocation, not a call frame on the allocation's path, so it must
# not be treated as a non-target frame to match against a reaching set
# (it never appears in any caller reaching set, and its presence
# otherwise forces a spurious unresolved_normalization_mismatch). These
# tokens are dropped during frame canonicalization. Kept explicit and
# small, like ALLOCATOR_ALIASES -- only deallocation helpers that are
# known allocator plumbing belong here.
ALLOCATOR_PLUMBING = {
    "xfree",
    "free_xenheap_pages",
    "free_domheap_pages",
}


def _canonical_alloc(fn: str) -> str:
    """Map an allocator-layer label to its canonical static target."""
    return ALLOCATOR_ALIASES.get(fn, fn)


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
    frames = [_canonical_alloc(str(f).strip())
              for f in frames if str(f).strip()]
    # Drop allocator plumbing (interleaved frees): a free between two
    # allocations is not a call frame on the allocation's path.
    frames = [f for f in frames if f not in ALLOCATOR_PLUMBING]
    target = obj.get("target") or obj.get("allocation_target")
    if target:
        target = _canonical_alloc(str(target).strip())
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

    # Recurse for structured per-path JSON. Skip manifest files and
    # anything that is not valid JSON (a text report mislabelled .json
    # raises and is ignored, never treated as a record).
    for jf in sorted(parsed_dir.rglob("*.json")):
        if jf.name in ("runtime-manifest.json", "runtime-summary.json",
                       "runtime-static-comparison.json"):
            continue
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

    # Fallback: the runtime log parser's text output, read recursively
    # (the parser shards reports by arch under parsed/<arch>/). We read
    # the real parser artifact rather than require a structured export;
    # a structured allocation-paths.json, when present, is preferred
    # above.
    paths.extend(_load_parser_text_reports(parsed_dir, known_targets))
    return paths


# Parser text-report shapes. Two shapes are recognized:
#
# 1. A flat `comments`-style record: a header line at column 0 naming
#    the allocating function (`fn()` optionally with a domain), then
#    indented call-path frames.
#
# 2. The per-arch parser report (e.g. parsed/<arch>/<job>.log), where
#    each non-freed-allocation record is introduced by a
#    `[dN] <fn>:` line and the call path appears under a
#    `max size path:` sub-block as indented frames.
#
# Frames may be `file.c#func`; only the function part is kept so a
# frame like `strtoull.c#_xmalloc` matches the known target `_xmalloc`.
_TEXT_HEADER_RE = re.compile(
    r"^(?P<fn>[A-Za-z_][A-Za-z0-9_]*)\(\)"
    r"(?:.*\bdom(?:ain)?[ =:]+(?P<dom>\S+))?")
_TEXT_FRAME_RE = re.compile(
    r"^\s+(?P<fn>[A-Za-z_][A-Za-z0-9_.#]*)\b")
_REPORT_REC_RE = re.compile(
    r"^\s*\[(?P<dom>[^\]]+)\]\s+(?P<fn>[A-Za-z_][A-Za-z0-9_]*)\s*:")
_REPORT_PATH_HDR_RE = re.compile(r"^\s*max size path:\s*$")
_REPORT_PATH_FRAME_RE = re.compile(
    r"^\s+(?P<fn>[A-Za-z_][A-Za-z0-9_.#]*)\s*(?:\(|\[|$)")


def _frame_func(token: str) -> str:
    """Normalize a frame token to a bare, canonical function name.

    Strips a `file.c#` qualifier (`strtoull.c#_xmalloc` -> `_xmalloc`)
    and maps an allocator-layer label to its canonical static target
    (`xmem_pool_alloc` -> `_xmalloc`; see ALLOCATOR_ALIASES).
    """
    t = token.strip()
    if "#" in t:
        t = t.split("#", 1)[1]
    return _canonical_alloc(t)


def _iter_report_files(parsed_dir: Path):
    """Yield candidate parser text files anywhere under parsed_dir.

    Recurses, because the parser shards reports by arch
    (parsed/<arch>/<job>.log) and may nest a verbose/ variant. Skips
    .json files (handled elsewhere) so a mislabelled or text-bearing
    .json is never parsed as a report here.
    """
    seen: set = set()
    for name in ("comments", "comments.txt"):
        p = parsed_dir / name
        if p.exists():
            seen.add(p.resolve())
            yield p
    for pat in ("*.txt", "*.report", "*.log"):
        for p in sorted(parsed_dir.rglob(pat)):
            rp = p.resolve()
            if rp not in seen:
                seen.add(rp)
                yield p


def _resolve_target(header_fn: str, frames: list[str], known: set) -> str:
    """Pick a record's allocation target from its header or frames.

    Header and frames are canonicalized through the allocator-alias
    map first, so an allocator-layer label (e.g. xmem_pool_alloc)
    resolves to its static target (_xmalloc).
    """
    h = _canonical_alloc(header_fn)
    if h in known:
        return h
    for f in frames:
        cf = _canonical_alloc(f)
        if cf in known:
            return cf
    return ""


def _parse_report_text(text: str, known: set) -> list[dict]:
    out: list[dict] = []

    def flush(header_fn, dom, frames):
        if not frames:
            return
        norm = [_frame_func(f) for f in frames]
        # Drop allocator plumbing (interleaved frees) here as well as in
        # _coerce_path_record, so the drop applies regardless of which
        # loader produced the frames. A free between two allocations is
        # not a call frame on the allocation's path.
        norm = [f for f in norm if f not in ALLOCATOR_PLUMBING]
        if not norm:
            # Every frame was allocator plumbing (e.g. a path of only
            # frees): this is not an allocation path, so there is
            # nothing to attribute. Skip rather than indexing an empty
            # list below.
            return
        # Collapse consecutive duplicate frames (the parser may print
        # the head frame twice at increasing indent).
        collapsed: list[str] = []
        for f in norm:
            if not collapsed or collapsed[-1] != f:
                collapsed.append(f)
        norm = collapsed
        target = _resolve_target(_frame_func(header_fn), norm, known)
        if not target:
            return  # resolves to no known allocation target -> skip
        if norm[-1] != target:
            norm = norm + [target]
        out.append({"frames": norm, "target": target,
                    "domain": dom or "", "raw": header_fn})

    lines = text.splitlines()
    i, n = 0, len(lines)
    while i < n:
        raw = lines[i]
        rec = _REPORT_REC_RE.match(raw)
        flat = _TEXT_HEADER_RE.match(raw)
        if rec:
            header_fn = rec.group("fn")
            dom = rec.group("dom")
            frames: list[str] = []
            j = i + 1
            in_path = False
            while j < n:
                lj = lines[j]
                if _REPORT_REC_RE.match(lj):
                    break
                if _REPORT_PATH_HDR_RE.match(lj):
                    if frames:
                        # Already collected the first path block; a
                        # second "max size path:" belongs to a trailing
                        # summary block for the same record -- stop so
                        # its frames are not appended twice.
                        break
                    in_path = True
                    j += 1
                    continue
                if in_path:
                    mf = _REPORT_PATH_FRAME_RE.match(lj)
                    if mf:
                        frames.append(mf.group("fn"))
                    elif lj.strip():
                        # Any non-frame, non-blank line ends the path
                        # block (e.g. "[d0] total non-freed ...").
                        in_path = False
                j += 1
            flush(header_fn, dom, frames)
            i = j
            continue
        if flat and not raw[:1].isspace():
            header_fn = flat.group("fn")
            dom = flat.group("dom") or ""
            frames = []
            j = i + 1
            while j < n:
                lj = lines[j]
                if (_TEXT_HEADER_RE.match(lj) and not lj[:1].isspace()) \
                        or _REPORT_REC_RE.match(lj):
                    break
                mf = _TEXT_FRAME_RE.match(lj)
                if mf:
                    frames.append(mf.group("fn"))
                j += 1
            flush(header_fn, dom, frames)
            i = j
            continue
        i += 1
    return out


def _load_parser_text_reports(parsed_dir: Path,
                              known_targets: set | None = None
                              ) -> list[dict]:
    """Read the parser's text reports (recursively) into records.

    Only records that resolve to a known allocation target are
    emitted -- either the header function is a known target, or one of
    the path frames is (covering the parser report shape where the
    header is e.g. `xmem_pool_alloc` but the path reaches `_xmalloc`).
    A free-path entry, comment, or diagnostic section that resolves to
    no known target is skipped, so non-allocation sections do not
    become phantom runtime paths.
    """
    known = set(known_targets) if known_targets else set(DEFAULT_TARGETS)
    out: list[dict] = []
    for rep in _iter_report_files(parsed_dir):
        try:
            text = rep.read_text(errors="replace")
        except OSError:
            continue
        try:
            out.extend(_parse_report_text(text, known))
        except Exception as e:  # noqa: BLE001
            # A single malformed report must not crash the comparator and
            # block the whole corpus. Skip it and carry on; the absence
            # of its paths is visible in the metrics, and the failure is
            # noted rather than fatal. This mirrors the JSON/CSV loaders,
            # which already swallow malformed input.
            _PARSE_REPORT_WARNINGS.append(
                f"{rep}: unreadable parser report skipped ({type(e).__name__})")
            continue
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

    # 0. Target observed with no caller context. After allocator-alias
    #    canonicalization a trace can collapse to just the target
    #    itself (e.g. the _xmalloc WARN logs only "xmem_pool_alloc",
    #    which aliases to _xmalloc, with no caller frame). The target
    #    was genuinely observed at runtime, but there is no caller frame
    #    to tie to a direct-static or indirect path, so neither an
    #    explanation nor an unexplained verdict is warranted. Reported
    #    distinctly, and counted as observing the target.
    if target and not non_target_frames:
        return ("target_observed_no_caller_context", "target_only",
                f"target {target} observed at runtime with no caller "
                f"frame (allocator entry point only)", set())

    # 0b. Only allocator chain-links above the target. If every
    #     non-target frame is itself an allocation target (the path
    #     traversed only the allocator chain, e.g.
    #     alloc_domheap_pages -> alloc_xenheap_pages ->
    #     alloc_domheap_pages, or _xmalloc -> _xmalloc after plumbing
    #     was dropped), there is no external caller captured. This is the
    #     no-caller case, not a normalization mismatch.
    target_set = set(targets)
    if target and non_target_frames and non_target_frames <= target_set:
        return ("target_observed_no_caller_context", "target_only",
                f"target {target} observed via allocator chain "
                f"({'->'.join(f for f in frames if f != target) or 'self'}) "
                f"with no external caller frame", set())

    # 1. Direct-static explanation: a non-target frame of the path is
    #    in the direct-static reaching set of an allocation target the
    #    path actually traversed.
    #
    #    The path is attributed to its innermost allocator (`target`),
    #    but an allocation can cross an allocator chain -- e.g.
    #    `avc_audit -> _xmalloc -> alloc_xenheap_pages ->
    #    alloc_domheap_pages`, where the real caller (avc_audit) reaches
    #    _xmalloc, not the innermost alloc_domheap_pages. So consider, in
    #    order: the path's own target first, then any OTHER allocation
    #    target that appears as a frame in this path (i.e. an allocator
    #    the path genuinely went through). This is strictly scoped to
    #    allocator targets present in the path -- it does not try
    #    unrelated targets -- so it only widens matching for real
    #    allocator-chain traversals. The first candidate whose reaching
    #    set contains a non-target frame explains the path.
    chain_targets = [t for t in targets
                     if t != target and t in fnset and t in direct_static]
    candidate_targets = []
    if target in direct_static:
        candidate_targets.append(target)
    candidate_targets.extend(chain_targets)
    # Fall back to the original "any target with a matching frame" scan
    # when the path's own target is unknown to direct_static and no
    # in-path allocator matched, preserving prior behaviour.
    ds_target = None
    for t in candidate_targets:
        if non_target_frames & direct_static[t]:
            ds_target = t
            break
    if ds_target is None and target not in direct_static:
        for t in targets:
            if t in direct_static and (non_target_frames
                                       & direct_static[t]):
                ds_target = t
                break
    indirect_inter = fnset & indirect_impls
    if ds_target is not None and (non_target_frames
                                  & direct_static[ds_target]):
        reaching = direct_static[ds_target]
        # Non-target frames excluding allocator targets the path merely
        # passed through on its way to `target`: those are chain links,
        # not unexplained callers, so they should not defeat the
        # "all non-target frames explained" check below.
        chain_links = {t for t in targets if t != ds_target}
        accounted = {f for f in non_target_frames if f not in chain_links}
        # Prefer indirect attribution when the only matching frame is
        # itself a known indirect candidate impl (the dispatch went
        # through a function pointer, not a direct call).
        ds_frames = non_target_frames & reaching
        if ds_frames and ds_frames <= indirect_impls and indirect_inter:
            pass  # fall through to indirect classification
        elif all(f in reaching for f in accounted):
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
    "target_observed_no_caller_context",
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
    _PARSE_REPORT_WARNINGS.clear()
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
        # Diagnostic detail for unmatched-path triage. Recomputed here
        # (not in the classifier, whose return contract is unchanged)
        # from the same inputs the classifier used. This makes a
        # normalization residual diagnosable straight from the artifact:
        # the full canonicalised frame list, the non-target frames the
        # matcher actually had to work with, and the static reaching set
        # of the path's target that it was checked against. The gap
        # between non_target_frames and reaching_set_checked is exactly
        # what a fix would need to close.
        frames = rec["frames"]
        target = rec["target"]
        non_target = [f for f in frames if f != target]
        reaching = sorted(direct_static.get(target, set())) if target else []
        # Frames that DID land in the reaching set (empty for a
        # mismatch -- that emptiness is the finding) vs those that did
        # not, to point straight at the offending tokens.
        reaching_set = set(reaching)
        nt_in_reaching = [f for f in non_target if f in reaching_set]
        nt_not_in_reaching = [f for f in non_target if f not in reaching_set]
        runtime_rows.append({
            "classification": cls,
            "confidence": conf,
            "target": target,
            "domain": rec["domain"],
            "head_function": frames[-1] if frames else "",
            "frame_count": len(frames),
            "basis": basis,
            # Diagnostic-only fields (do not affect classification).
            "frames": " -> ".join(frames),
            "non_target_frames": " -> ".join(non_target),
            "non_target_in_reaching": " -> ".join(nt_in_reaching),
            "non_target_not_in_reaching": " -> ".join(nt_not_in_reaching),
            "reaching_set_checked": " ".join(reaching),
            "reaching_set_size": len(reaching),
        })

    counts = {c: sum(1 for r in runtime_rows
                     if r["classification"] == c)
              for c in RUNTIME_CLASSES}

    # Static-candidate observation.
    candidates_observed = sorted(observed_impls)
    candidates_not_observed = sorted(indirect_impls - observed_impls)

    # Write per-path audit CSVs. Include target_observed_no_caller_context
    # alongside the unexplained/mismatch classes: all three are the
    # "did not resolve to a static path" residue we triage. The extra
    # columns expose the full canonicalised frames, the non-target frames
    # split by whether they landed in the reaching set, and the reaching
    # set itself -- enough to classify a residual as an alias-map gap, a
    # symbol-naming mismatch, or a missing static edge without the raw log.
    unmatched_classes = (
        "runtime_only_unexplained",
        "unresolved_normalization_mismatch",
        "target_observed_no_caller_context",
    )
    unmatched = [r for r in runtime_rows
                 if r["classification"] in unmatched_classes]
    unmatched_columns = [
        "classification", "confidence", "target", "domain",
        "head_function", "frame_count", "basis",
        "frames", "non_target_frames",
        "non_target_in_reaching", "non_target_not_in_reaching",
        "reaching_set_size", "reaching_set_checked",
    ]
    _write_csv(out_dir / "runtime-unmatched-paths.csv",
               unmatched_columns, unmatched)
    # JSON sidecar: same rows, but frame lists as arrays rather than
    # arrow-joined strings, which is far easier to read and diff for the
    # longer stacks.
    unmatched_json = []
    for r in unmatched:
        unmatched_json.append({
            "classification": r["classification"],
            "target": r["target"],
            "domain": r["domain"],
            "frame_count": r["frame_count"],
            "frames": r["frames"].split(" -> ") if r["frames"] else [],
            "non_target_frames": (r["non_target_frames"].split(" -> ")
                                  if r["non_target_frames"] else []),
            "non_target_not_in_reaching": (
                r["non_target_not_in_reaching"].split(" -> ")
                if r["non_target_not_in_reaching"] else []),
            "reaching_set_checked": (r["reaching_set_checked"].split(" ")
                                     if r["reaching_set_checked"] else []),
            "basis": r["basis"],
        })
    (out_dir / "runtime-unmatched-paths.json").write_text(
        json.dumps(unmatched_json, indent=2) + "\n")

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
        "runtime_paths_target_observed_no_caller_context":
            counts["target_observed_no_caller_context"],
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
        ] + collect_notes + list(_PARSE_REPORT_WARNINGS),
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
