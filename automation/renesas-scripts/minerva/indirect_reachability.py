#!/usr/bin/env python3
"""Indirect-call reachability workbench.

Reads the artifacts produced by `scripts/collect.py` for a single Xen
configuration and asks, for each indirect-dispatch call site:

  "Does any concrete implementation of the dispatched field
   transitively reach an allocation target
   (alloc_domheap_pages / alloc_xenheap_pages / _xmalloc)?"

The workbench is a thin glue layer between the collector and
`minerva_static_analysis/callpath.py`. It does not compute the final
runtime-vs-static coverage number, which still requires aligned
runtime parser output and a real Xen build's .ci files for the same
configuration.

Inputs
------
  --ci-dir <path-to-.ci-dir>
      Directory containing `.ci` files (gcc -fcallgraph-info output)
      for a Xen build at the chosen configuration. If omitted or
      empty, the workbench runs in dry-run mode: it identifies the
      candidate (impl, target) queries and emits the exact callpath.py
      commands an operator would run, marking path_found=UNKNOWN.

  --collector-run <path-to-runs/<config>>
      Directory of collector artifacts for the chosen configuration.
      Must contain ops-resolution.csv, indirect-call-sites.csv, and
      runtime-ops-registration-sites.csv.

  --targets <name> [<name> ...]
      Allocation target function names. Default:
      alloc_domheap_pages alloc_xenheap_pages _xmalloc.

  --out-dir <path>
      Output directory for the workbench artifacts.

Outputs
-------
  indirect-allocation-paths.csv
  synthetic_edges.candidates.yaml
  reachability-summary.md

Caveats
-------
  - The workbench does not claim an allocation bound. `path_found=yes`
    means callpath.py found at least one path; it does not mean the
    path is actually exercised at runtime or that allocation is
    bounded.
  - `path_found=no` means the static graph at the given .ci closure
    does not enumerate a path. It does not mean the path is
    impossible; it may exist via macros, function pointers not
    modelled by .ci, or via paths excluded by `-e`.
  - `path_found=UNKNOWN` means the workbench could not query
    callpath.py--  either because --ci-dir was not supplied, the
    directory contained no `.ci` files, or the query errored out.
  - PASS mode (`--ci-dir` supplied + callpath.py found) is batch
    analysis, not an instant report. Each candidate implementation
    is queried once per target by spawning callpath.py in a
    subprocess; on a full Xen `.ci` tree a query takes seconds and
    a single workbench run may run for several minutes. Repeated
    (impl, target) queries are cached in-process so duplicate
    candidates only fire once.
  - Runtime-registration rows are inventoried by collect.py but
    are *not* threaded into the field-dispatch query loop. They
    bind whole tables / callback objects rather than a single
    dispatched field, and require a separate analyst step to map
    each registered object to the fields it dispatches in the
    selected configuration. The reachability-summary surfaces
    them as a worklist.
"""

import argparse
import csv
import json
import re
import shlex
import subprocess
import sys
from collections import Counter, defaultdict
from pathlib import Path


DEFAULT_TARGETS = ["alloc_domheap_pages", "alloc_xenheap_pages", "_xmalloc"]

INDIRECT_ALLOCATION_COLUMNS = [
    "config_name",
    "call_site_function",
    "source_file",
    "line",
    "field_name",
    "receiver_expression",
    "implementation_function",
    "allocation_target",
    "path_found",
    "path_signature",
    "confidence",
    "basis",
    "notes",
    # Table-aware binding fields.
    "candidate_binding",
    "candidate_included",
    "exclusion_reason",
    "binding_basis",
]


# Table-aware binding helpers.
#
# An indirect call site can syntactically dispatch a field of the same
# name as one declared on many unrelated ops tables (`.init`, `.read`,
# `.write`, `.map_page`, ...). A naive field-name-only join across
# ops-resolution.csv and indirect-call-sites.csv treats every such
# pair as a candidate synthetic edge and inflates the candidate count
# numerator with false correlations.
#
# Each (call-site, candidate-impl) pair is classified into each (call-site, candidate-impl) pair into one of
# these `candidate_binding` buckets, and surfaces only the first three
# in `synthetic_edges.candidates.yaml`:
#
#   table_compatible          The call site dispatches through a
#                             receiver whose name appears in the
#                             candidate ops-table instance. Strongest
#                             signal.
#
#   receiver_family_compatible
#                             The call site uses a family-implying
#                             macro (`iommu_call`, `iommu_vcall`) and
#                             the candidate ops-table belongs to that
#                             family (table_instance / table_type /
#                             source_file contains a family-keyword).
#
#   curated_compatible        Reserved for an explicit family map,
#                             not produced by default.
#
#   field_name_only           Match is purely field-name; no table or
#                             receiver-family alignment. Diagnostic
#                             only; not emitted to
#                             synthetic_edges.candidates.yaml.
#
#   unresolved                The call site or ops row lacks enough
#                             data to establish any compatibility.

_IOMMU_MACRO_RE = re.compile(r"\b(iommu_call|iommu_vcall)\s*\(")
_IOMMU_FAMILY_HINTS = ("iommu", "smmu", "ipmmu")

# Receiver names so generic that they appear as substrings of nearly
# every `*_ops` table instance. We refuse to call those a
# table_compatible signal.
_GENERIC_RECEIVER_TOKENS = {
    "ops", "op", "p", "ptr", "x", "obj", "self", "impl", "handler",
    "cb", "data", "info", "ctx", "ctxt", "hd", "v", "d",
}


_IOMMU_PATH_HINTS = ("iommu", "smmu", "ipmmu", "passthrough")


def detect_call_site_family(xen_root: Path, src: str, line_no: str,
                            call_fn: str,
                            file_cache: dict[str, list[str]]
                            ) -> tuple[str, str]:
    """Return (family, basis). family in {'IOMMU', ''}.

    Three signals are checked, in order:
      1. The call-site source line--  IOMMU macros (`iommu_call`,
         `iommu_vcall`).
      2. The call-site source file path--  IOMMU-family directory or
         file naming (`iommu`, `smmu`, `ipmmu`, `passthrough`).
      3. The containing function name--  IOMMU-family naming.
    The first signal that matches wins. File contents are cached
    per source path for the duration of the workbench run.
    """
    if not src and not call_fn:
        return ("", "no source location and no containing function")
    src_lower = src.lower()
    # 1. Macro at the exact line.
    if src and line_no:
        try:
            ln = int(line_no)
        except (TypeError, ValueError):
            ln = 0
        if ln > 0:
            if src not in file_cache:
                try:
                    file_cache[src] = (xen_root / src).read_text(
                        errors="replace").splitlines()
                except OSError:
                    file_cache[src] = []
            lines = file_cache[src]
            if 0 < ln <= len(lines) and _IOMMU_MACRO_RE.search(lines[ln - 1]):
                return ("IOMMU", "iommu_call / iommu_vcall macro at call site")
    # 2. Source-file path hint.
    if any(h in src_lower for h in _IOMMU_PATH_HINTS):
        matched = next(h for h in _IOMMU_PATH_HINTS if h in src_lower)
        return ("IOMMU",
                f"call site in IOMMU subtree (path contains '{matched}')")
    # 3. Containing-function name hint.
    if call_fn:
        fn_lower = call_fn.lower()
        if any(h in fn_lower for h in ("iommu", "smmu", "ipmmu")):
            matched = next(h for h in ("iommu", "smmu", "ipmmu")
                           if h in fn_lower)
            return ("IOMMU",
                    f"containing function name contains '{matched}'")
    return ("", "")


def classify_candidate_binding(site_family: str,
                               cand: dict,
                               receiver: str
                               ) -> tuple[str, str, str]:
    """Return (candidate_binding, exclusion_reason_or_empty, binding_basis).

    cand carries `table_instance`, `basis`, and (via the workbench's
    `build_implementations`) was originally pulled from
    ops-resolution.csv with an associated source_file. classify_*
    treats the ops-resolution row as one fixed candidate; it does not
    second-guess the field-name match the caller already made.
    """
    table_lower = (cand.get("table_instance") or "").lower()
    src_lower = (cand.get("source_file") or "").lower()
    # 1. Macro-implied family wins.
    if site_family == "IOMMU":
        if any(h in table_lower for h in _IOMMU_FAMILY_HINTS):
            return ("receiver_family_compatible", "",
                    "iommu_call macro at site; "
                    f"candidate table_instance '{cand.get('table_instance')}' "
                    "matches IOMMU family")
        if any(h in src_lower for h in _IOMMU_FAMILY_HINTS):
            return ("receiver_family_compatible", "",
                    "iommu_call macro at site; "
                    f"candidate source_file '{cand.get('source_file')}' "
                    "matches IOMMU family")
        return ("field_name_only",
                "field_name_only_without_table_or_receiver_match",
                "iommu_call macro at site but candidate table "
                "and source_file do not match IOMMU family")
    # 2. Receiver-name -> table-instance substring (table_compatible).
    # Skip generic receiver names that would otherwise match every
    # *_ops table by substring.
    if receiver:
        recv_lower = receiver.lower()
        # Strip common pointer-deref noise. Receivers captured in
        # indirect-call-sites.csv may include chained accesses like
        # `d->iommu` or `hd->arch.iommu`; take the last identifier as
        # the most informative token.
        last_tok = re.split(r"[\.\->]+", recv_lower)[-1]
        if (last_tok
                and last_tok not in _GENERIC_RECEIVER_TOKENS
                and len(last_tok) >= 4
                and last_tok in table_lower):
            return ("table_compatible", "",
                    f"receiver token '{last_tok}' appears in "
                    f"table_instance '{cand.get('table_instance')}'")
    # 3. Default: field-name-only, excluded.
    return ("field_name_only",
            "field_name_only_without_table_or_receiver_match",
            "field-name match only; no table or receiver-family signal")


def read_csv_rows(path: Path) -> list[dict]:
    if not path.exists():
        return []
    with path.open() as fh:
        return list(csv.DictReader(fh))


def find_callpath_py(repo_root: Path) -> Path | None:
    """Locate minerva_static_analysis/callpath.py.

    Resolves relative to the workbench script if --collector-run is
    inside a Xen-minerva checkout, otherwise None.
    """
    candidates = [
        repo_root / "minerva_static_analysis" / "callpath.py",
        Path(__file__).resolve().parent.parent
            / "minerva_static_analysis" / "callpath.py",
    ]
    for c in candidates:
        if c.exists():
            return c
    return None


def ci_dir_has_files(ci_dir: Path | None) -> bool:
    if ci_dir is None:
        return False
    if not ci_dir.exists() or not ci_dir.is_dir():
        return False
    for _ in ci_dir.rglob("*.ci"):
        return True
    return False


# Function-name extraction at the head of an indented line in
# `callpath.py to <target>` output. The tree shape is one function
# per line, optionally indented, optionally followed by `@
# file:line:col`.
_FN_NAME_RE = re.compile(r"^\s*([A-Za-z_][A-Za-z0-9_]*)\b")


def query_target_tree(callpath_py: Path, ci_dir: Path, target: str,
                      timeout_s: int) -> tuple[set[str], dict[str, str], str]:
    """Run `callpath.py to <ci-dir> <target> -i` once and return:

      (set of function names that appear anywhere in the paths-to-target
       tree, first-line-by-function map, error string or "")

    The target-tree-membership reachability strategy uses this once per
    allocation target and then answers every (implementation, target)
    reachability question by set membership. That replaces the
    per-implementation `callpath.py from <impl>` subprocess loop, which
    scales as O(unique implementations) and on Windows hosts is
    dominated by per-spawn overhead.
    """
    cmd = [sys.executable, str(callpath_py), "to", str(ci_dir), target, "-i"]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True,
                              timeout=timeout_s, check=False)
    except subprocess.TimeoutExpired:
        return set(), {}, f"timeout after {timeout_s}s"
    if proc.returncode != 0:
        return set(), {}, proc.stderr.strip()[:240]
    names: set[str] = set()
    first: dict[str, str] = {}
    for raw in proc.stdout.splitlines():
        m = _FN_NAME_RE.match(raw)
        if m:
            n = m.group(1)
            names.add(n)
            if n not in first:
                first[n] = raw.strip()[:300]
    return names, first, ""


def query_reachability(callpath_py: Path, ci_dir: Path,
                       impl: str, target: str,
                       timeout_s: int) -> tuple[str, str, str]:
    """Return (path_found, path_signature, error).

    path_found in {"yes", "no", "ERROR"}. path_signature is the first
    line of the printed path that mentions the target, truncated.
    """
    cmd = [sys.executable, str(callpath_py), "from",
           str(ci_dir), impl, "-i"]
    try:
        proc = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout_s,
            check=False,
        )
    except subprocess.TimeoutExpired:
        return ("ERROR", "", f"timeout after {timeout_s}s")
    if proc.returncode != 0:
        return ("ERROR", "", proc.stderr.strip()[:240])
    for line in proc.stdout.splitlines():
        if target in line:
            return ("yes", line.strip()[:300], "")
    return ("no", "", "")


def callpath_command(callpath_py: Path | None, ci_dir: Path | None,
                     impl: str) -> str:
    cp = str(callpath_py) if callpath_py else "<callpath.py>"
    ci = str(ci_dir) if ci_dir else "<ci-dir>"
    return f"python3 {shlex.quote(cp)} from {shlex.quote(ci)} {shlex.quote(impl)} -i"


def build_implementations(ops_resolution: list[dict],
                          ) -> dict[str, list[dict]]:
    """Return field_name -> list of candidate implementation records
    drawn from ops-resolution.csv.

    Each record carries the implementation function name, where it
    came from, and the static-vs-config-guarded basis.

    Runtime registrations are deliberately *not* threaded into this
    map. A registration site binds a whole table or callback object
    rather than one dispatched field, so the field-dispatch query
    loop has no way to consume it; see `build_runtime_registration_
    worklist` for the separate worklist.
    """
    by_field: dict[str, list[dict]] = defaultdict(list)
    for r in ops_resolution:
        impl = (r.get("implementation_function") or "").strip()
        field = (r.get("field_name") or "").strip()
        scope = (r.get("config_scope") or "").strip()
        if not impl or not field:
            continue
        # Strip leading & from address-of expressions and trailing comments.
        impl = impl.lstrip("&").strip().rstrip(",;")
        if not impl:
            continue
        # Only literal identifiers can be queried; expressions with
        # casts, struct fields, or arithmetic are skipped.
        if not impl.replace("_", "").replace(".", "").isalnum():
            continue
        if "." in impl or "->" in impl:
            continue
        by_field[field].append({
            "impl": impl,
            "table_instance": r.get("table_instance", ""),
            "table_type": r.get("table_type", ""),
            "source_file": r.get("source_file", ""),
            "source": "ops_resolution",
            "scope": scope,
            "basis": (r.get("basis") or "").strip(),
        })
    # Deduplicate by (impl) inside each field; first record wins.
    out: dict[str, list[dict]] = {}
    for field, recs in by_field.items():
        seen = set()
        deduped = []
        for rec in recs:
            if rec["impl"] in seen:
                continue
            seen.add(rec["impl"])
            deduped.append(rec)
        out[field] = deduped
    return out


def build_runtime_registration_worklist(runtime_reg: list[dict]
                                        ) -> list[dict]:
    """Return the analyst-worklist view of runtime-registration rows.

    A runtime registration binds a whole table / callback object /
    notifier--  not a single dispatched field--  so these rows are
    *not* expanded into the field-dispatch reachability query loop.
    Surface them as a separate worklist that an analyst can map to
    the fields actually dispatched in the selected configuration,
    once the binding semantics are understood.

    Only rows whose semantic classification places them at a static
    binding (``static_impl_obvious`` /
    ``static_impl_config_guarded``) are included; ``dynamic_argument``
    rows are intentionally excluded because there is no static
    candidate to surface yet.
    """
    out: list[dict] = []
    for r in runtime_reg:
        cls = (r.get("classification") or "").strip()
        cand = (r.get("candidate_impl") or "").strip()
        if cls not in ("static_impl_obvious", "static_impl_config_guarded"):
            continue
        if not cand:
            continue
        out.append({
            "impl": cand,
            "table_or_field": r.get("table_or_field_if_obvious", ""),
            "scope": ("in_scope" if cls == "static_impl_obvious"
                      else "out_of_scope_config"),
            "source_file": r.get("source_file", ""),
            "line": r.get("line", ""),
            "containing_function": r.get("containing_function", ""),
            "pattern": r.get("pattern", ""),
            "classification": cls,
        })
    return out


def derive_confidence(call_site_fn: str, impl_record: dict) -> str:
    """high / medium / low confidence for a (call site, impl) edge.

    high   ops_resolution static initializer in-scope, with
           call_site_function resolved.
    medium ops_resolution initializer with config uncertainty or
           call_site_function unresolved.
    low    runtime-registration-derived candidate.
    """
    if impl_record["source"] == "runtime_registration":
        return "low"
    if impl_record["scope"] == "in_scope" \
            and call_site_fn and call_site_fn != "(unresolved)":
        return "high"
    return "medium"


def write_reachability_summary(out_dir: Path, mode: str,
                               config_name: str,
                               targets: list[str],
                               counts: dict,
                               unresolved_examples: list[str],
                               commands: list[str],
                               runtime_registration_worklist: list[dict],
                               strategy: str = "target-tree",
                               target_set_sizes: dict[str, int] | None = None):
    f = out_dir / "reachability-summary.md"
    lines: list[str] = []
    lines.append("# Indirect-reachability workbench summary\n")
    lines.append(f"Config: `{config_name}`")
    lines.append(f"Targets: {', '.join('`' + t + '`' for t in targets)}")
    lines.append(f"Mode: **{mode}**")
    if mode == "PASS":
        lines.append(f"Strategy: **{strategy}**")
    lines.append("")
    if mode == "PASS" and strategy == "target-tree" and target_set_sizes:
        lines.append("## Paths-to-target tree sizes\n")
        lines.append("| Target | Distinct functions reaching target |")
        lines.append("| --- | ---: |")
        for t in targets:
            lines.append(f"| `{t}` | {target_set_sizes.get(t, 0)} |")
        lines.append("")
    if mode == "PARTIAL":
        lines.append(
            "The workbench ran without a usable `--ci-dir`. "
            "`path_found` values in `indirect-allocation-paths.csv` "
            "are `UNKNOWN`; `synthetic_edges.candidates.yaml` lists "
            "candidate edges with no reachability decision. The "
            "final yes/no decision requires a real Xen build's `.ci` "
            "files for the same configuration and a re-run of this "
            "workbench against them."
        )
        lines.append("")
    lines.append("## Counts\n")
    lines.append("| Counter | Value |")
    lines.append("| --- | --- |")
    for k in ("implementations_examined",
              "implementations_reaching_alloc",
              "indirect_call_sites_examined",
              "indirect_call_sites_reaching_alloc",
              "candidate_synthetic_edges",
              "candidate_synthetic_edges_included",
              "candidate_synthetic_edges_excluded",
              "candidate_rows_total",
              "candidate_rows_included",
              "candidate_rows_excluded",
              "candidate_rows_field_name_only_excluded",
              "path_found_yes",
              "path_found_no",
              "path_found_UNKNOWN",
              "path_found_ERROR"):
        if k in counts:
            lines.append(f"| `{k}` | {counts.get(k, 0)} |")
    lines.append("")
    if unresolved_examples:
        lines.append("## Implementations with no static-graph path "
                     "(samples)\n")
        for ex in unresolved_examples[:10]:
            lines.append(f"- {ex}")
        lines.append("")
    if mode == "PARTIAL" and commands:
        lines.append("## Commands to run when `.ci` becomes available\n")
        lines.append("Run each command from the repository root. The "
                     "workbench can then be re-invoked with the same "
                     "`--ci-dir` to populate yes/no values. The list "
                     "below is truncated to the first 50 entries; the "
                     "full count is shown in the parenthetical.\n")
        lines.append("```")
        for c in commands[:50]:
            lines.append(c)
        if len(commands) > 50:
            lines.append(f"# ... {len(commands) - 50} more")
        lines.append("```")
        lines.append("")
    lines.append("## Runtime-registration rows\n")
    lines.append(
        "Runtime-registration rows from "
        "`runtime-ops-registration-sites.csv` are **not** expanded "
        "into reachability queries in this workbench run. They "
        "represent whole-table or callback-object installation "
        "sites, not a single dispatched field. They remain an "
        "analyst worklist and must be expanded separately by "
        "mapping the registered table / object to the fields "
        "actually dispatched in this configuration."
    )
    lines.append("")
    lines.append(
        "Candidate synthetic edges in this report are derived from "
        "field-resolved indirect-call candidates "
        "(`ops-resolution.csv` joined with `indirect-call-sites.csv`), "
        "**not** from runtime-registration rows."
    )
    lines.append("")
    lines.append(
        f"Runtime-registration worklist size (static_impl rows "
        f"surfaced for analyst review): "
        f"**{len(runtime_registration_worklist)}**."
    )
    if runtime_registration_worklist:
        lines.append("")
        lines.append("First few worklist entries:")
        lines.append("")
        lines.append("| impl | table_or_field | scope | source |")
        lines.append("| --- | --- | --- | --- |")
        for r in runtime_registration_worklist[:10]:
            lines.append(
                f"| `{r['impl']}` | `{r['table_or_field']}` | "
                f"{r['scope']} | "
                f"{r['source_file']}:{r['line']} |"
            )
        if len(runtime_registration_worklist) > 10:
            lines.append(
                f"\n(showing 10 of "
                f"{len(runtime_registration_worklist)}.)"
            )
        lines.append("")
    lines.append("## Counter arithmetic\n")
    lines.append(
        "Per-target `indirect-allocation-paths.csv` row count is "
        "(in-scope (site, impl) candidate pairs) + (sites with no "
        "candidate impl, which surface a single fallback row per "
        "target so analyst review can still see them). "
        "`candidate_synthetic_edges` counts deduplicated (site, "
        "impl) candidate pairs only and so excludes the "
        "no-candidate fallback rows."
    )
    lines.append("")
    lines.append("## Caveats\n")
    lines.append("- No allocation bound is claimed. `path_found=yes` "
                 "is a static-graph reachability statement only.")
    lines.append("- `path_found=no` does not mean impossible.")
    lines.append("- A runtime-vs-static comparison requires aligned "
                 "runtime parser output, callpath.py output, and "
                 "this workbench's yes/no decisions for the same "
                 "expanded `.config`.")
    f.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_synthetic_edges_yaml(out_dir: Path, config_name: str,
                               edges: list[dict],
                               filename: str = "synthetic_edges.candidates.yaml"):
    f = out_dir / filename
    lines: list[str] = []
    lines.append(f"config: {config_name}")
    lines.append("edges:")
    for e in edges:
        lines.append(f"  - call_site_function: {e['call_site_function']}")
        lines.append(f"    source: \"{e['source']}\"")
        lines.append(f"    dispatch: \"{e['dispatch']}\"")
        lines.append(f"    implementation: {e['implementation']}")
        lines.append(f"    basis: \"{e['basis']}\"")
        lines.append(f"    confidence: {e['confidence']}")
        if "candidate_binding" in e:
            lines.append(f"    candidate_binding: {e['candidate_binding']}")
        if "binding_basis" in e:
            lines.append(f"    binding_basis: \"{e['binding_basis']}\"")
        if "exclusion_reason" in e:
            lines.append(f"    exclusion_reason: {e['exclusion_reason']}")
        if e.get("reaches"):
            lines.append("    reaches:")
            for t in e["reaches"]:
                lines.append(f"      - {t}")
        else:
            lines.append("    reaches: []  # not yet decided")
    f.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--ci-dir", type=Path, default=None)
    p.add_argument("--collector-run", type=Path, required=True)
    p.add_argument("--targets", nargs="+", default=DEFAULT_TARGETS)
    p.add_argument("--out-dir", type=Path, required=True)
    p.add_argument("--timeout", type=int, default=30,
                   help="Per-callpath.py-invocation timeout in seconds. "
                        "target-tree mode runs one query per allocation "
                        "target; per-impl mode runs one per (impl, target) "
                        "pair, so this matters more there.")
    p.add_argument("--reachability-strategy",
                   choices=("target-tree", "per-impl"),
                   default="target-tree",
                   help="PASS-mode reachability strategy. target-tree "
                        "(default): run `callpath.py to <target>` once "
                        "per allocation target and answer each (impl, "
                        "target) question by set membership. per-impl: "
                        "run `callpath.py from <impl>` once per unique "
                        "implementation and grep for the target. Both "
                        "produce the same yes/no semantic; target-tree "
                        "is dramatically faster on hosts where Python "
                        "subprocess spawn is expensive (Windows / "
                        "anti-virus / launcher shims).")
    p.add_argument("--xen-root", type=Path, default=Path("."),
                   help="Used only to locate callpath.py if --ci-dir "
                        "is supplied.")
    args = p.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    collector = args.collector_run
    if not collector.exists():
        print(f"ERROR: --collector-run {collector} does not exist",
              file=sys.stderr)
        return 2

    # Load collector inputs.
    summary_path = collector / "collection-summary.json"
    summary = json.loads(summary_path.read_text()) if summary_path.exists() \
        else {"config_name": collector.name}
    config_name = summary.get("config_name", collector.name)
    ops_resolution = read_csv_rows(collector / "ops-resolution.csv")
    indirect_sites = read_csv_rows(collector / "indirect-call-sites.csv")
    runtime_reg = read_csv_rows(
        collector / "runtime-ops-registration-sites.csv"
    )

    impls_by_field = build_implementations(ops_resolution)
    runtime_registration_worklist = build_runtime_registration_worklist(
        runtime_reg
    )

    # Decide mode.
    #   PASS                live reachability via callpath.py.
    #   PARTIAL             --ci-dir absent or unusable; emit UNKNOWN
    #                       and command stubs.
    have_ci = ci_dir_has_files(args.ci_dir)
    callpath_py = find_callpath_py(args.xen_root.resolve()) if have_ci else None
    mode = "PASS" if (have_ci and callpath_py) else "PARTIAL"
    if have_ci and not callpath_py:
        print(
            "WARNING: --ci-dir has .ci files but callpath.py could not "
            "be located; falling back to dry-run.",
            file=sys.stderr,
        )

    # PASS-mode reachability strategy. target-tree (default): one
    # callpath.py call per allocation target; answer each (impl, target)
    # query by set membership. per-impl: original loop, one call per
    # unique implementation per target.
    target_sets: dict[str, set[str]] = {}
    target_first_line: dict[str, dict[str, str]] = {}
    target_errors: dict[str, str] = {}
    if mode == "PASS" and args.reachability_strategy == "target-tree":
        for target in args.targets:
            names, first, err = query_target_tree(
                callpath_py, args.ci_dir, target, args.timeout,
            )
            target_sets[target] = names
            target_first_line[target] = first
            target_errors[target] = err
            if err:
                print(
                    f"WARNING: callpath.py to <ci-dir> {target} failed: "
                    f"{err}",
                    file=sys.stderr,
                )
            else:
                print(
                    f"  target {target}: {len(names)} distinct "
                    f"functions in paths-to-target tree",
                    file=sys.stderr,
                )

    # Cache (impl, target) -> result so duplicate impls only fire once.
    cache: dict[tuple[str, str], tuple[str, str, str]] = {}
    commands: list[str] = []

    # File-content cache for the call-site family detector. One read
    # per source file across all sites in that file.
    site_source_cache: dict[str, list[str]] = {}

    impls_examined: set[str] = set()
    impls_reaching: set[str] = set()
    sites_examined = 0
    sites_reaching = 0
    candidate_edges: list[dict] = []
    no_path_examples: list[str] = []

    rows: list[dict] = []
    excluded_edges: list[dict] = []
    for site in indirect_sites:
        field = (site.get("field_name") or "").strip()
        call_fn = (site.get("call_site_function") or "").strip()
        src = (site.get("source_file") or "").strip()
        line_no = (site.get("line_or_context") or "").strip()
        recv = (site.get("receiver_expression") or "").strip()
        candidates = impls_by_field.get(field, [])
        sites_examined += 1
        site_any_reach = False
        site_family, _site_family_basis = detect_call_site_family(
            args.xen_root, src, line_no, call_fn, site_source_cache)
        for cand in candidates:
            impl = cand["impl"]
            impls_examined.add(impl)
            for target in args.targets:
                key = (impl, target)
                if key in cache:
                    path_found, sig, err = cache[key]
                else:
                    if mode == "PASS" and args.reachability_strategy == "target-tree":
                        terr = target_errors.get(target, "")
                        if terr:
                            path_found, sig, err = "UNKNOWN", "", terr
                        else:
                            tree = target_sets.get(target, set())
                            if impl in tree:
                                path_found = "yes"
                                sig = target_first_line[target].get(impl, "")
                                err = ""
                            else:
                                path_found, sig, err = "no", "", ""
                    elif mode == "PASS":
                        path_found, sig, err = query_reachability(
                            callpath_py, args.ci_dir, impl, target,
                            args.timeout,
                        )
                    else:
                        path_found, sig, err = "UNKNOWN", "", ""
                    cache[key] = (path_found, sig, err)
                    cmd = callpath_command(callpath_py, args.ci_dir, impl)
                    if cmd not in commands:
                        commands.append(cmd)
                confidence = derive_confidence(call_fn, cand)
                binding, excl_reason, binding_basis = classify_candidate_binding(
                    site_family, cand, recv)
                included = binding in ("table_compatible",
                                       "receiver_family_compatible",
                                       "curated_compatible")
                row = {
                    "config_name": config_name,
                    "call_site_function": call_fn,
                    "source_file": src,
                    "line": line_no,
                    "field_name": field,
                    "receiver_expression": recv,
                    "implementation_function": impl,
                    "allocation_target": target,
                    "path_found": path_found,
                    "path_signature": sig,
                    "confidence": confidence,
                    "basis": cand["basis"],
                    "notes": (f"src={cand['source']};scope={cand['scope']}"
                              + (f";err={err}" if err else "")),
                    "candidate_binding": binding,
                    "candidate_included": "true" if included else "false",
                    "exclusion_reason": excl_reason,
                    "binding_basis": binding_basis,
                }
                rows.append(row)
                if path_found == "yes":
                    impls_reaching.add(impl)
                    site_any_reach = True
                    edge = {
                        "call_site_function": call_fn or "(unresolved)",
                        "source": f"{src}:{line_no}",
                        "dispatch": f"{recv}.{field}",
                        "implementation": impl,
                        "basis": cand["basis"][:200],
                        "confidence": confidence,
                        "candidate_binding": binding,
                        "binding_basis": binding_basis,
                        "reaches": [target],
                    }
                    if included:
                        candidate_edges.append(edge)
                    else:
                        edge["exclusion_reason"] = excl_reason
                        excluded_edges.append(edge)
                elif path_found == "no" and len(no_path_examples) < 20:
                    no_path_examples.append(
                        f"{impl} ? {target} (from "
                        f"{call_fn or '(unresolved)'} @ {src}:{line_no})"
                    )
        if site_any_reach:
            sites_reaching += 1
        if not candidates:
            # No candidate found for this field; still surface the site
            # so the analyst can see it.
            for target in args.targets:
                rows.append({
                    "config_name": config_name,
                    "call_site_function": call_fn,
                    "source_file": src,
                    "line": line_no,
                    "field_name": field,
                    "receiver_expression": recv,
                    "implementation_function": "",
                    "allocation_target": target,
                    "path_found": "UNKNOWN",
                    "path_signature": "",
                    "confidence": "low",
                    "basis": "no candidate implementation found in "
                             "ops-resolution.csv for this field",
                    "notes": "site has no resolved candidate",
                    "candidate_binding": "unresolved",
                    "candidate_included": "false",
                    "exclusion_reason": "no_candidate_implementation",
                    "binding_basis":
                        "no ops-resolution row for the dispatched field",
                })

    # When in PARTIAL mode, emit synthetic-edge candidates for every
    # in-scope (site, impl) pair so the analyst pass still has a
    # worklist. reaches=[] in that case; table-aware binding still
    # decides included vs excluded.
    if mode != "PASS":
        seen_edge_keys: set[tuple[str, str]] = set()
        candidate_edges = []
        excluded_edges = []
        for site in indirect_sites:
            field = (site.get("field_name") or "").strip()
            call_fn = (site.get("call_site_function") or "").strip()
            src = (site.get("source_file") or "").strip()
            line_no = (site.get("line_or_context") or "").strip()
            recv = (site.get("receiver_expression") or "").strip()
            site_family, _ = detect_call_site_family(
                args.xen_root, src, line_no, call_fn, site_source_cache)
            candidates = impls_by_field.get(field, [])
            for cand in candidates:
                if cand["scope"] == "out_of_scope_config":
                    continue
                key = (cand["impl"],
                       f"{call_fn}@{src}:{line_no}.{field}")
                if key in seen_edge_keys:
                    continue
                seen_edge_keys.add(key)
                binding, excl_reason, binding_basis = classify_candidate_binding(
                    site_family, cand, recv)
                included = binding in ("table_compatible",
                                       "receiver_family_compatible",
                                       "curated_compatible")
                edge = {
                    "call_site_function": call_fn or "(unresolved)",
                    "source": f"{src}:{line_no}",
                    "dispatch": f"{recv}.{field}",
                    "implementation": cand["impl"],
                    "basis": cand["basis"][:200] or
                             f"static initializer "
                             f"({cand['table_instance']})",
                    "confidence": derive_confidence(call_fn, cand),
                    "candidate_binding": binding,
                    "binding_basis": binding_basis,
                    "reaches": [],
                }
                if included:
                    candidate_edges.append(edge)
                else:
                    edge["exclusion_reason"] = excl_reason
                    excluded_edges.append(edge)

    # Deduplicate candidate edges by (call_site_function, source,
    # dispatch, implementation) and merge their `reaches` lists. The
    # per-target loop above appends one entry per allocation target;
    # the YAML schema expects `reaches` to be a list, so one edge per
    # (site, impl) with multiple targets is the canonical form.
    def _dedup_edges(edges: list[dict]) -> list[dict]:
        idx: dict[tuple[str, str, str, str], dict] = {}
        for e in edges:
            k = (e["call_site_function"], e["source"], e["dispatch"],
                 e["implementation"])
            if k in idx:
                for t in e["reaches"]:
                    if t not in idx[k]["reaches"]:
                        idx[k]["reaches"].append(t)
            else:
                idx[k] = dict(e, reaches=list(e["reaches"]))
        return list(idx.values())

    unique_edges = _dedup_edges(candidate_edges)
    unique_excluded_edges = _dedup_edges(excluded_edges)

    # Write artifacts.
    out_csv = args.out_dir / "indirect-allocation-paths.csv"
    with out_csv.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=INDIRECT_ALLOCATION_COLUMNS)
        w.writeheader()
        for r in rows:
            w.writerow(r)

    write_synthetic_edges_yaml(args.out_dir, config_name, unique_edges)
    write_synthetic_edges_yaml(args.out_dir, config_name,
                               unique_excluded_edges,
                               filename="synthetic_edges.excluded.yaml")

    binding_counter = Counter(r["candidate_binding"] for r in rows)
    excl_reason_counter = Counter(
        r["exclusion_reason"] for r in rows if r["exclusion_reason"]
    )
    included_rows = sum(1 for r in rows if r["candidate_included"] == "true")
    excluded_rows = sum(1 for r in rows if r["candidate_included"] == "false")

    counts = {
        "implementations_examined": len(impls_examined),
        "implementations_reaching_alloc": len(impls_reaching),
        "indirect_call_sites_examined": sites_examined,
        "indirect_call_sites_reaching_alloc": sites_reaching,
        "candidate_synthetic_edges": len(unique_edges),
        "candidate_synthetic_edges_included": len(unique_edges),
        "candidate_synthetic_edges_excluded": len(unique_excluded_edges),
        "candidate_rows_total": len(rows),
        "candidate_rows_included": included_rows,
        "candidate_rows_excluded": excluded_rows,
        "candidate_rows_field_name_only_excluded":
            sum(1 for r in rows if r["candidate_binding"] == "field_name_only"),
        "path_found_yes": sum(1 for r in rows if r["path_found"] == "yes"),
        "path_found_no": sum(1 for r in rows if r["path_found"] == "no"),
        "path_found_UNKNOWN":
            sum(1 for r in rows if r["path_found"] == "UNKNOWN"),
        "path_found_ERROR":
            sum(1 for r in rows if r["path_found"] == "ERROR"),
    }

    # Per-binding and per-exclusion-reason breakdowns surface in
    # candidate-binding-summary.md.
    binding_summary_lines: list[str] = []
    binding_summary_lines.append("# Candidate binding summary\n")
    binding_summary_lines.append(f"Config: `{config_name}`\n")
    binding_summary_lines.append("## Counts by `candidate_binding`\n")
    binding_summary_lines.append("| Binding | Rows |")
    binding_summary_lines.append("| --- | ---: |")
    for k in ("table_compatible", "receiver_family_compatible",
              "curated_compatible", "field_name_only",
              "runtime_registration_worklist", "unresolved"):
        binding_summary_lines.append(f"| `{k}` | {binding_counter.get(k, 0)} |")
    binding_summary_lines.append("")
    binding_summary_lines.append("## Counts by `exclusion_reason`\n")
    binding_summary_lines.append("| Reason | Rows |")
    binding_summary_lines.append("| --- | ---: |")
    for k, v in sorted(excl_reason_counter.items(), key=lambda kv: -kv[1]):
        binding_summary_lines.append(f"| `{k}` | {v} |")
    if not excl_reason_counter:
        binding_summary_lines.append("| _(none)_ | 0 |")
    binding_summary_lines.append("")
    # Per-implementation included vs excluded positives.
    impl_yes_inc: dict[str, set[str]] = defaultdict(set)
    impl_yes_exc: dict[str, set[str]] = defaultdict(set)
    for r in rows:
        if r["path_found"] != "yes":
            continue
        impl = r["implementation_function"]
        if r["candidate_included"] == "true":
            impl_yes_inc[impl].add(r["allocation_target"])
        else:
            impl_yes_exc[impl].add(r["allocation_target"])
    binding_summary_lines.append("## Included positives by implementation\n")
    if impl_yes_inc:
        binding_summary_lines.append("| Implementation | Targets reached |")
        binding_summary_lines.append("| --- | --- |")
        for impl, ts in sorted(impl_yes_inc.items()):
            binding_summary_lines.append(
                f"| `{impl}` | {', '.join(sorted(ts))} |"
            )
    else:
        binding_summary_lines.append("_(none)_")
    binding_summary_lines.append("")
    binding_summary_lines.append("## Excluded positives by implementation\n")
    if impl_yes_exc:
        binding_summary_lines.append(
            "These would have been counted as positives by a pure "
            "field-name join, but are excluded under table-aware "
            "binding because the (site, impl) pair lacks a "
            "table/receiver-family signal."
        )
        binding_summary_lines.append("")
        binding_summary_lines.append("| Implementation | Targets reached |")
        binding_summary_lines.append("| --- | --- |")
        for impl, ts in sorted(impl_yes_exc.items()):
            binding_summary_lines.append(
                f"| `{impl}` | {', '.join(sorted(ts))} |"
            )
    else:
        binding_summary_lines.append("_(none)_")
    binding_summary_lines.append("")
    # Generic field names that drove exclusions.
    fn_excl = Counter(
        r["field_name"] for r in rows
        if r["candidate_binding"] == "field_name_only"
    )
    binding_summary_lines.append(
        "## Field names that drove field_name_only exclusions\n"
    )
    if fn_excl:
        binding_summary_lines.append("| Field | Excluded rows |")
        binding_summary_lines.append("| --- | ---: |")
        for fn, n in fn_excl.most_common(20):
            binding_summary_lines.append(f"| `{fn}` | {n} |")
    else:
        binding_summary_lines.append("_(none)_")
    binding_summary_lines.append("")
    binding_summary_lines.append(
        "## Runtime-registration worklist\n"
    )
    binding_summary_lines.append(
        f"Runtime-registration rows are not expanded into per-field "
        f"reachability queries; they remain an analyst worklist. "
        f"Worklist size on this run: "
        f"**{len(runtime_registration_worklist)}**."
    )
    (args.out_dir / "candidate-binding-summary.md").write_text(
        "\n".join(binding_summary_lines) + "\n", encoding="utf-8")

    target_set_sizes = ({t: len(target_sets.get(t, set()))
                         for t in args.targets}
                        if mode == "PASS"
                        and args.reachability_strategy == "target-tree"
                        else None)
    write_reachability_summary(
        args.out_dir, mode, config_name, args.targets,
        counts, no_path_examples, commands,
        runtime_registration_worklist,
        strategy=args.reachability_strategy,
        target_set_sizes=target_set_sizes,
    )

    # Console summary.
    print(f"mode: {mode}", file=sys.stderr)
    for k, v in counts.items():
        print(f"  {k}: {v}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
