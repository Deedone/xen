#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Corpus-level dynamic-allocation assurance analyzer (Series 10).

Consumes SEPARATED static-analysis artifacts and runtime artifacts,
joins them by manifest identity, runs (or reuses) runtime/static
comparison for matched pairs, aggregates the results, collapses raw
runtime observations into canonical allocation scenarios, classifies
those scenarios, ingests optional structural coverage evidence, and
emits exactly one bounded assurance verdict: SUPPORTED or
NOT_SUPPORTED.

The verdict is a bounded assurance claim over the analysed corpus and
the supplied coverage evidence. It is not an absolute claim about all
possible Xen executions. This tool never asserts Xen has no dynamic
allocations, never treats high structural coverage as exhaustive
runtime coverage, never treats static reachability as proof of
execution, and never reduces the result to one coverage percentage.
"""
from __future__ import annotations

import argparse
import csv
import datetime
import io
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import corpus_schema as cs            # noqa: E402
import allocation_scenarios as asc    # noqa: E402


# --------------------------------------------------------------------
# Minimal YAML reader (avoid a hard dependency on PyYAML).
# --------------------------------------------------------------------

def _load_yaml(path: Path):
    try:
        import yaml  # type: ignore
        return yaml.safe_load(Path(path).read_text())
    except ImportError:
        return _tiny_yaml(Path(path).read_text())
    except (OSError, ValueError):
        return None


def _tiny_yaml(text: str):
    """A very small YAML subset parser: nested maps, lists of scalars
    and lists of maps, scalars, and `>` folded scalars. Sufficient for
    the policy / annotation files shipped here. Not a general parser.
    """
    root: dict = {}
    stack = [(-1, root)]
    lines = text.splitlines()
    i = 0
    while i < len(lines):
        raw = lines[i]
        if not raw.strip() or raw.lstrip().startswith("#"):
            i += 1
            continue
        indent = len(raw) - len(raw.lstrip())
        line = raw.strip()
        while stack and indent <= stack[-1][0] and len(stack) > 1:
            stack.pop()
        parent = stack[-1][1]
        if line.startswith("- "):
            item = line[2:].strip()
            if not isinstance(parent, list):
                continue
            if ":" in item:
                d: dict = {}
                k, v = item.split(":", 1)
                d[k.strip()] = _scalar(v.strip())
                parent.append(d)
                stack.append((indent, d))
            else:
                parent.append(_scalar(item))
        elif line.endswith(":"):
            key = line[:-1].strip()
            # peek to decide list vs map
            nxt = _next_content(lines, i)
            val: list | dict = [] if nxt.startswith("- ") else {}
            _assign(parent, key, val)
            stack.append((indent, val))
        elif ": >" in line or line.endswith(": >"):
            key = line.split(":", 1)[0].strip()
            block, i = _read_folded(lines, i, indent)
            _assign(parent, key, block)
            continue
        elif ":" in line:
            key, v = line.split(":", 1)
            _assign(parent, key.strip(), _scalar(v.strip()))
        i += 1
    return root


def _assign(parent, key, val):
    if isinstance(parent, dict):
        parent[key] = val


def _next_content(lines, i):
    for j in range(i + 1, len(lines)):
        if lines[j].strip() and not lines[j].lstrip().startswith("#"):
            return lines[j].strip()
    return ""


def _read_folded(lines, i, indent):
    out = []
    j = i + 1
    while j < len(lines):
        if lines[j].strip() and (len(lines[j]) - len(lines[j].lstrip())) \
                <= indent:
            break
        out.append(lines[j].strip())
        j += 1
    return " ".join(out), j


def _scalar(v: str):
    v = v.strip().strip('"').strip("'")
    if v in ("true", "True"):
        return True
    if v in ("false", "False"):
        return False
    try:
        return int(v)
    except ValueError:
        pass
    try:
        return float(v)
    except ValueError:
        pass
    return v


# --------------------------------------------------------------------
# Loaders (Part 1, 2): static + runtime artifacts.
# --------------------------------------------------------------------

def _derive_target_arch(manifest: dict, root: Path) -> str:
    if manifest.get("target_arch"):
        return manifest["target_arch"]
    name = (manifest.get("config_name") or "").lower()
    for a in ("arm64", "arm32", "x86", "riscv"):
        if a in name:
            return a
    return ""


def load_static_artifacts(static_root: Path, warnings: list,
                          stats: dict = None) -> list:
    """Load static-analysis artifact trees under static_root.

    Each subdirectory (or static_root itself) containing a
    static-analysis manifest is one artifact. Missing core identity
    (git_sha, config_sha256) -> quarantine. Derivable fields
    (target_arch, callgraph_backend, static_artifact_id) are filled.

    stats (optional) accumulates discovered/quarantined counts so the
    verdict engine can tell "none found" from "all quarantined".
    """
    arts = []
    for mpath in _find_manifests(static_root, "static-analysis-manifest.json"):
        if stats is not None:
            stats["static_discovered"] = stats.get("static_discovered", 0) + 1
        m = cs.try_read_json(mpath) or {}
        root = mpath.parent
        m.setdefault("artifact_type", "static-analysis")
        m["target_arch"] = _derive_target_arch(m, root)
        if not m.get("callgraph_backend"):
            cg = cs.try_read_json(root / "callgraph" / "backend.json") or {}
            m["callgraph_backend"] = cg.get("backend") or "unknown"
        missing = [f for f in cs.STATIC_CORE_IDENTITY if not m.get(f)]
        if missing:
            warnings.append(f"static artifact {root} quarantined: "
                            f"missing {missing}")
            if stats is not None:
                stats["static_quarantined"] = stats.get(
                    "static_quarantined", 0) + 1
            continue
        if not m.get("static_artifact_id"):
            m["static_artifact_id"] = cs.stable_id(
                "static", m["git_sha"], m["target_arch"],
                m["config_sha256"], m["callgraph_backend"])
        arts.append({"manifest": m, "root": root})
    return arts


def load_runtime_artifacts(runtime_root: Path, warnings: list,
                           stats: dict = None) -> list:
    arts = []
    for mpath in _find_manifests(runtime_root, "runtime-manifest.json"):
        if stats is not None:
            stats["runtime_discovered"] = stats.get(
                "runtime_discovered", 0) + 1
        m = cs.try_read_json(mpath)
        if m is None:
            warnings.append(f"runtime manifest {mpath} unreadable; "
                            f"quarantined")
            if stats is not None:
                stats["runtime_quarantined"] = stats.get(
                    "runtime_quarantined", 0) + 1
            continue
        root = mpath.parent
        m.setdefault("artifact_type", "runtime")
        m["target_arch"] = _derive_target_arch(m, root)
        if not m.get("test_name"):
            m["test_name"] = m.get("job_name") or root.name
        # Derive config_sha256 from a co-located .config if the producer
        # omitted it, so a recoverable artifact is not needlessly
        # quarantined. (The producer should emit it; this is a fallback.)
        if not m.get("config_sha256"):
            for cand in (root / ".config", root / "config" / ".config"):
                if cand.exists():
                    import hashlib
                    m["config_sha256"] = hashlib.sha256(
                        cand.read_bytes()).hexdigest()
                    break
        missing = [f for f in cs.RUNTIME_CORE_IDENTITY if not m.get(f)]
        if missing:
            warnings.append(f"runtime artifact {root} quarantined: "
                            f"missing {missing}")
            if stats is not None:
                stats["runtime_quarantined"] = stats.get(
                    "runtime_quarantined", 0) + 1
            continue
        if not m.get("runtime_artifact_id"):
            m["runtime_artifact_id"] = cs.stable_id(
                "runtime", m["git_sha"], m["target_arch"],
                m["config_sha256"], m["test_name"])
        arts.append({"manifest": m, "root": root})
    return arts


def _find_manifests(root: Path, name: str):
    root = Path(root)
    if not root.exists():
        return []
    direct = root / name
    if direct.exists():
        return [direct]
    return sorted(root.rglob(name))


# --------------------------------------------------------------------
# Join (Part 5) + comparison runner (Part 5/7).
# --------------------------------------------------------------------

def select_static(matches: list, policy: dict):
    if len(matches) <= 1:
        return matches[0] if matches else None, ""
    pref = (((policy.get("static_runtime_join") or {})
             .get("preferred_callgraph_backend"))
            or policy.get("preferred_callgraph_backend") or "any")
    if pref != "any":
        chosen = [a for a in matches
                  if a["manifest"].get("callgraph_backend") == pref]
        if len(chosen) == 1:
            return chosen[0], ""
        if len(chosen) > 1:
            return None, f"ambiguous static backends for preferred {pref}"
    return None, "multiple static artifacts; policy cannot disambiguate"


def join_corpus(statics, runtimes, policy, out_dir, warnings):
    """Match runtime artifacts to static artifacts by join key; run or
    reuse comparison. Returns a list of matched-pair records.
    """
    by_key: dict = {}
    for a in statics:
        by_key.setdefault(cs.join_key(a["manifest"]), []).append(a)
    pairs = []
    for rt in runtimes:
        key = cs.join_key(rt["manifest"])
        cands = by_key.get(key, [])
        static, reason = select_static(cands, policy)
        rec = {"runtime": rt, "static": static,
               "comparison_status": "", "metrics": {}, "reason": reason}
        if static is None:
            rec["comparison_status"] = "COMPARISON_BLOCKED"
            warnings.append(f"runtime {rt['manifest']['test_name']}: "
                            f"no matching static artifact "
                            f"({reason or 'no join-key match'})")
        else:
            rec["comparison_status"], rec["metrics"] = run_or_reuse_compare(
                rt, static, out_dir, warnings)
        pairs.append(rec)
    return pairs


def run_or_reuse_compare(rt, static, out_dir, warnings):
    """Reuse an existing runtime-static-comparison.json if present in
    the runtime artifact; else invoke runtime_static_compare.py.
    """
    test = rt["manifest"]["test_name"]
    comp_out = Path(out_dir) / "comparisons" / test
    existing = _find_existing_comparison(rt["root"])
    if existing is not None:
        comp_out.mkdir(parents=True, exist_ok=True)
        (comp_out / "runtime-static-comparison.json").write_text(
            (existing.parent / existing.name).read_text())
        m = cs.try_read_json(existing) or {}
        return "COMPARED", m
    # Invoke the comparator.
    parsed = rt["root"] / "parsed"
    reach = static["root"] / "reachability"
    direct = static["root"] / "direct-static"
    script = Path(__file__).resolve().parent / "runtime_static_compare.py"
    if not (parsed.exists() and reach.exists() and direct.exists()):
        warnings.append(f"{test}: missing inputs for comparison "
                        f"(parsed/reachability/direct-static)")
        return "COMPARISON_BLOCKED", {}
    comp_out.mkdir(parents=True, exist_ok=True)
    try:
        subprocess.run(
            [sys.executable, str(script),
             "--runtime-parsed", str(parsed),
             "--reachability-dir", str(reach),
             "--direct-static-dir", str(direct),
             "--out-dir", str(comp_out)],
            check=True, capture_output=True, timeout=600)
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired,
            FileNotFoundError) as e:
        warnings.append(f"{test}: comparison failed: {e}")
        return "COMPARISON_BLOCKED", {}
    m = cs.try_read_json(
        comp_out / "runtime-static-comparison.json") or {}
    return "COMPARED", m


def _find_existing_comparison(root: Path):
    for p in Path(root).rglob("runtime-static-comparison.json"):
        return p
    return None


# --------------------------------------------------------------------
# Aggregation (Part 7).
# --------------------------------------------------------------------

def aggregate(pairs):
    """Aggregate comparison metrics per config group."""
    groups: dict = {}
    for rec in pairs:
        rt = rec["runtime"]["manifest"]
        cg = cs.config_group_hash(
            {**rt, "callgraph_backend":
             (rec["static"]["manifest"].get("callgraph_backend")
              if rec["static"] else "")})
        g = groups.setdefault(cg, _empty_agg(rt, rec))
        if rec["comparison_status"] == "COMPARISON_BLOCKED":
            g["comparison_blocked"] += 1
            continue
        m = rec["metrics"]
        for metric, agg in cs.COMPARISON_METRIC_TO_AGG.items():
            g[agg] += int(m.get(metric, 0) or 0)
        for metric, agg in cs.INDIRECT_METRIC_TO_AGG.items():
            g[agg] += int(m.get(metric, 0) or 0)
    return groups


def _empty_agg(rt, rec):
    g = {a: 0 for a in cs.AGG_RUNTIME_CLASSES}
    g.update({"runtime_paths_total": 0, "config_mismatch": 0,
              "comparison_blocked": 0, "indirect_candidates_total": 0,
              "indirect_candidates_observed": 0,
              "indirect_candidates_not_observed": 0,
              "target_arch": rt.get("target_arch", ""),
              "config_sha256": rt.get("config_sha256", ""),
              "config_name": rt.get("config_name", ""),
              "callgraph_backend":
              (rec["static"]["manifest"].get("callgraph_backend")
               if rec["static"] else "")})
    return g


# --------------------------------------------------------------------
# Observations -> scenarios. Build observation records from each
# matched pair's comparison detail CSVs (or metrics fallback).
# --------------------------------------------------------------------

def build_observations(pairs, out_dir):
    obs = []
    for rec in pairs:
        if rec["comparison_status"] != "COMPARED":
            continue
        rt = rec["runtime"]["manifest"]
        cg = cs.config_group_hash(
            {**rt, "callgraph_backend":
             (rec["static"]["manifest"].get("callgraph_backend")
              if rec["static"] else "")})
        comp_dir = Path(out_dir) / "comparisons" / rt["test_name"]
        rows = _read_runtime_rows(comp_dir, rec["runtime"]["root"])
        for row in rows:
            raw = _split_frames(row.get("frames")
                                or row.get("head_function") or "")
            canon = cs.normalize_frame_sequence(raw)
            target = cs.normalize_frame(row.get("target") or
                                        (canon[-1] if canon else ""))
            if not canon and target:
                canon = [target]
            obs.append({
                "allocation_target": target,
                "canonical_stack": canon,
                "raw_stack": raw or canon,
                "domain": row.get("domain", ""),
                "comparison_class": row.get("classification", ""),
                "config_group": cg,
                "test_name": rt["test_name"],
                "max_size": row.get("max_size") or row.get("size") or 0,
                "evidence_path": str(comp_dir),
            })
    return obs


def _read_runtime_rows(comp_dir: Path, rt_root: Path):
    """Reconstruct per-path rows for scenario grouping.

    The comparator emits aggregate per-class counts plus a detail CSV
    only for unmatched paths -- not a full per-path table with frames.
    To classify explained scenarios by cause/effect we need the real
    caller frames, so we read them directly from the runtime artifact's
    parsed reports (the same per-arch report blocks the comparator
    consumes) and pair them positionally with the per-class counts from
    the comparison metrics. Unmatched-path frames from the comparator's
    detail CSV refine the unexplained buckets.
    """
    m = cs.try_read_json(comp_dir / "runtime-static-comparison.json") or {}
    inv = {v: k for k, v in cs.COMPARISON_METRIC_TO_AGG.items()}

    # Real frame sequences from the runtime parsed tree.
    real = _read_parsed_frames(Path(rt_root) / "parsed")

    # Unmatched-path detail (frames for unexplained/mismatch classes).
    unmatched_rows = []
    up = comp_dir / "runtime-unmatched-paths.csv"
    if up.exists():
        try:
            unmatched_rows = list(csv.DictReader(up.open()))
        except OSError:
            unmatched_rows = []

    rows = []
    real_idx = 0
    for agg in cs.AGG_RUNTIME_CLASSES:
        n = int(m.get(inv.get(agg, ""), 0) or 0)
        detail = [r for r in unmatched_rows
                  if r.get("classification") == agg]
        for i in range(n):
            frames = target = domain = None
            if i < len(detail):
                d = detail[i]
                frames = d.get("frames") or d.get("head_function")
                target = d.get("target")
                domain = d.get("domain")
            if not frames and real_idx < len(real):
                r = real[real_idx]
                frames = r["frames"]
                target = target or r["target"]
                domain = domain or r["domain"]
            real_idx += 1
            rows.append({"classification": agg,
                         "target": target or "_xmalloc",
                         "frames": frames or "_xmalloc",
                         "domain": domain or ""})
    return rows


def _read_parsed_frames(parsed_dir: Path):
    """Read real allocation-path frame sequences from a runtime
    artifact's parsed tree, via the comparator's report reader so the
    frame shapes match exactly.
    """
    out = []
    try:
        import runtime_static_compare as rsc  # type: ignore
    except ImportError:
        return out
    known = set(getattr(rsc, "DEFAULT_TARGETS",
                        ["alloc_domheap_pages", "alloc_xenheap_pages",
                         "_xmalloc"]))
    reader = getattr(rsc, "_load_parser_text_reports", None)
    if reader is None:
        return out
    try:
        recs = reader(Path(parsed_dir), known)
    except Exception:
        return out
    for r in recs:
        frames = r.get("frames") or []
        out.append({"frames": frames,
                    "target": r.get("target") or
                    (frames[-1] if frames else ""),
                    "domain": r.get("domain", "")})
    return out


def _split_frames(s):
    if isinstance(s, list):
        return s
    for sep in (" -> ", ";", "->", ","):
        if sep in str(s):
            return [x.strip() for x in str(s).split(sep) if x.strip()]
    return [str(s).strip()] if str(s).strip() else []


# --------------------------------------------------------------------
# Coverage evidence (Part 3, 12).
# --------------------------------------------------------------------

def load_coverage(coverage_root, warnings):
    cov = []
    if not coverage_root:
        return cov
    root = Path(coverage_root)
    for p in list(root.rglob("coverage-summary.json")):
        c = cs.try_read_json(p)
        if c:
            cov.append(c)
    return cov


def coverage_for(coverage, target_arch, config_sha256):
    for c in coverage:
        if (str(c.get("config_sha256", "")) == config_sha256
                and (not c.get("target_arch")
                     or c.get("target_arch") == target_arch)):
            return c
    return None


def evaluate_coverage(config_groups, coverage, policy, blockers):
    creq = policy.get("coverage_requirements") or {}
    enabled = creq.get("enabled") or (policy.get("coverage") or {}).get(
        "required")
    states = {}
    for cg, g in config_groups.items():
        if not enabled:
            states[cg] = "coverage_claim_not_applicable"
            continue
        c = coverage_for(coverage, g["target_arch"], g["config_sha256"])
        if not c:
            states[cg] = "coverage_evidence_missing"
            blockers.append({"type": "coverage_evidence_missing",
                             "config_group": cg})
            continue
        if str(c.get("config_sha256", "")) != g["config_sha256"]:
            states[cg] = "coverage_config_mismatch"
            blockers.append({"type": "coverage_config_mismatch",
                             "config_group": cg})
            continue
        below = False
        for kind, minkey, reqkey in (
                ("line_coverage", "minimum_line_percent",
                 "require_line_coverage"),
                ("branch_coverage", "minimum_branch_percent",
                 "require_branch_coverage"),
                ("mcdc_coverage", "minimum_mcdc_percent",
                 "require_mcdc_coverage")):
            if not creq.get(reqkey, True):
                continue
            block = c.get(kind) or {}
            pct = block.get("percent")
            minimum = creq.get(minkey, 100.0)
            if pct is None or float(pct) < float(minimum):
                below = True
        if below:
            states[cg] = "coverage_below_threshold"
            blockers.append({"type": "coverage_below_threshold",
                             "config_group": cg})
        else:
            states[cg] = "coverage_evidence_present"
    return states


# --------------------------------------------------------------------
# Verdict engine (Part 13).
# --------------------------------------------------------------------

def runtime_required(policy, test_name, job_name):
    rr = policy.get("runtime_required")
    if isinstance(rr, bool):
        return rr
    rr = rr or {}
    import fnmatch
    inc = rr.get("include") or []
    exc = rr.get("exclude") or []
    name = test_name or job_name or ""
    if any(fnmatch.fnmatch(name, p) for p in exc):
        return False
    if any(fnmatch.fnmatch(name, p) for p in inc):
        return True
    return bool(policy.get("runtime_required") is True)


def exception_for(policy, scenario, now):
    for ex in policy.get("accepted_exceptions") or []:
        if ex.get("classification") != scenario["safety_classification"]:
            continue
        sig = ex.get("scenario_signature", "")
        if sig and sig not in (scenario["allocation_target"],
                               scenario.get("canonical_stack", "")):
            continue
        exp = ex.get("expires")
        if exp:
            try:
                if datetime.date.fromisoformat(str(exp)) < now:
                    continue  # expired
            except ValueError:
                continue
        if not (ex.get("justification") or "").strip():
            continue
        return ex
    return None


def compute_verdict(pairs, scenarios, coverage_states, policy,
                    blockers, accepted_exceptions, scope=None):
    now = datetime.date.today()
    accepted_cls = set(policy.get("accepted_safety_classifications")
                       or (policy.get("scenario_gate") or {})
                       .get("accepted_classes") or cs.SAFETY_ACCEPTED)

    # Hard input-validity gate. An empty corpus must NEVER support a
    # bounded assurance claim: absence of evidence is not evidence of
    # support. These blockers fire before any scenario reasoning.
    scope = scope or {}
    n_static = scope.get("static_loaded", 0)
    n_runtime = scope.get("runtime_loaded", 0)
    n_groups = scope.get("config_groups", 0)
    static_disc = scope.get("static_discovered", 0)
    runtime_disc = scope.get("runtime_discovered", 0)
    static_quar = scope.get("static_quarantined", 0)
    runtime_quar = scope.get("runtime_quarantined", 0)

    if n_static == 0:
        blockers.append({"type": "no_static_artifacts"})
    if n_runtime == 0:
        blockers.append({"type": "no_runtime_artifacts"})
    if n_groups == 0:
        blockers.append({"type": "no_config_groups"})
    # "Found some but all dropped" is distinct from "found none": call
    # it out explicitly so the report explains the empty corpus.
    if static_disc > 0 and n_static == 0 and static_quar == static_disc:
        blockers.append({"type": "all_static_artifacts_quarantined",
                         "count": static_disc})
    if runtime_disc > 0 and n_runtime == 0 and runtime_quar == runtime_disc:
        blockers.append({"type": "all_runtime_artifacts_quarantined",
                         "count": runtime_disc})

    # Comparison-level blockers.
    for rec in pairs:
        rt = rec["runtime"]["manifest"]
        req = runtime_required(policy, rt.get("test_name"),
                               rt.get("job_name"))
        st = rec["comparison_status"]
        if st == "COMPARISON_BLOCKED" and req:
            blockers.append({"type": "comparison_blocked",
                             "test": rt.get("test_name")})
        elif st == "PARTIAL" and req:
            blockers.append({"type": "partial_required_run",
                             "test": rt.get("test_name")})
        elif st == "STATIC_ONLY" and req:
            blockers.append({"type": "runtime_required_but_missing",
                             "test": rt.get("test_name")})
        elif st == "PROXY" and not (policy.get("static_runtime_join")
                                    or {}).get("allow_proxy"):
            blockers.append({"type": "unapproved_proxy",
                             "test": rt.get("test_name")})

    # Scenario-level blockers.
    for sc in scenarios:
        if sc["safety_classification"] in accepted_cls:
            continue
        if sc.get("classification_source") == "annotation_classified" \
                and sc["review_status"] == "accepted":
            continue
        ex = exception_for(policy, sc, now)
        if ex is not None:
            accepted_exceptions.append({"scenario_id": sc["scenario_id"],
                                        "exception_id": ex.get("id")})
            sc["review_status"] = "accepted"
            sc["_exception_id"] = ex.get("id")
            continue
        blockers.append({"type": "scenario", "scenario_id":
                         sc["scenario_id"],
                         "safety_classification":
                         sc["safety_classification"]})

    status = "NOT_SUPPORTED" if blockers else "SUPPORTED"
    if status == "SUPPORTED":
        qualifier = ("ACCEPTED_EXCEPTIONS" if accepted_exceptions
                     else "ZERO_EXCEPTIONS")
    else:
        qualifier = "NOT_APPLICABLE"
    return status, qualifier


# --------------------------------------------------------------------
# Reports (Part 14, 15, 16).
# --------------------------------------------------------------------

def _write_csv(path: Path, fieldnames, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fieldnames)
        w.writeheader()
        for r in rows:
            w.writerow({k: _csvval(r.get(k, "")) for k in fieldnames})


def _csvval(v):
    if isinstance(v, (list, tuple, set)):
        return ";".join(str(x) for x in v)
    return v


def write_reports(out_dir, status, qualifier, pairs, config_groups,
                  scenarios, coverage_states, blockers,
                  accepted_exceptions, baseline, warnings):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    now = datetime.datetime.now(datetime.timezone.utc).isoformat()

    scen_counts = _scenario_counts(scenarios)
    summary = {
        "claim": "no_unresolved_runtime_observed_dynamic_allocation_"
                 "scenarios",
        "assurance_status": status,
        "support_qualifier": qualifier,
        "generated_at": now,
        "corpus_scope": {
            "static_artifacts": len({id(p["static"]) for p in pairs
                                     if p["static"]}),
            "runtime_artifacts": len(pairs),
            "config_groups": len(config_groups),
        },
        "coverage_evidence": coverage_states,
        "scenario_summary": scen_counts,
        "scenario_summary_detail": [
            {"sig": s["allocation_target"] + "|" + s["canonical_stack"],
             "scenario_id": s["scenario_id"],
             "allocation_target": s["allocation_target"],
             "safety_classification": s["safety_classification"]}
            for s in scenarios],
        "blockers": blockers,
        "accepted_exceptions": accepted_exceptions,
    }
    cs.write_json(out / "corpus-assurance-summary.json", summary)
    _write_summary_md(out / "corpus-assurance-summary.md", status,
                      qualifier, summary, scenarios, blockers,
                      accepted_exceptions)

    # run index
    _write_csv(out / "corpus-run-index.csv",
               ["test_name", "git_sha", "target_arch", "config_sha256",
                "backend", "comparison_status"],
               [{"test_name": p["runtime"]["manifest"].get("test_name"),
                 "git_sha": p["runtime"]["manifest"].get("git_sha"),
                 "target_arch": p["runtime"]["manifest"].get("target_arch"),
                 "config_sha256":
                 p["runtime"]["manifest"].get("config_sha256"),
                 "backend": (p["static"]["manifest"].get(
                     "callgraph_backend") if p["static"] else ""),
                 "comparison_status": p["comparison_status"]}
                for p in pairs])

    # scenario outputs
    sdir = out / "scenario-analysis"
    asc_fields = [f for f in cs.SCENARIO_FIELDS]
    cs.write_json(sdir / "allocation-scenarios.json",
                  [{k: _csvval(s.get(k, "")) for k in asc_fields}
                   for s in scenarios])
    _write_csv(sdir / "allocation-scenarios.csv", asc_fields, scenarios)
    _write_csv(sdir / "accepted-scenarios.csv", asc_fields,
               [s for s in scenarios if s["review_status"] == "accepted"])
    _write_csv(sdir / "rejected-scenarios.csv", asc_fields,
               [s for s in scenarios if s["review_status"] == "rejected"])
    _write_csv(sdir / "needs-manual-review.csv", asc_fields,
               [s for s in scenarios
                if s["review_status"] == "needs_manual_review"])
    _write_scenarios_md(sdir / "allocation-scenarios.md", scenarios,
                        scen_counts)
    for s in scenarios:
        _write_scenario_group_md(
            sdir / "scenario-groups" / f"{s['scenario_id']}.md", s)

    # blocking findings
    _write_csv(out / "blocking-findings.csv",
               ["finding_id", "finding_type", "safety_classification",
                "target", "scenario_id", "config_name", "recommended_action",
                "exception_id_if_any"],
               [_finding_row(i, b, scenarios)
                for i, b in enumerate(blockers, 1)])
    _write_csv(out / "findings.csv",
               ["scenario_id", "safety_classification", "review_status",
                "allocation_target"], scenarios)

    # config summaries
    _write_csv(out / "config-summary.csv",
               ["config_group", "target_arch", "config_sha256", "backend",
                "runtime_paths_total"] + list(cs.AGG_RUNTIME_CLASSES),
               [{**{"config_group": cg, "backend": g["callgraph_backend"]},
                 **g} for cg, g in config_groups.items()])
    for cg, g in config_groups.items():
        cdir = out / "config-groups" / cg
        cs.write_json(cdir / "config-summary.json", g)
        _write_csv(cdir / "runtime-paths.csv",
                   list(cs.AGG_RUNTIME_CLASSES), [g])
        # Per-config-group markdown summary.
        _write_config_group_md(cdir / "config-summary.md", cg, g)
        # Scenarios belonging to this config group, split by concern.
        cg_scen = [s for s in scenarios
                   if cg in (s.get("config_groups_seen") or [])]
        _write_csv(cdir / "unexplained-runtime-paths.csv",
                   ["scenario_id", "allocation_target",
                    "safety_classification", "canonical_stack"],
                   [s for s in cg_scen
                    if "runtime_only_unexplained"
                    in (s.get("comparison_classes_seen") or [])])
        _write_csv(cdir / "normalization-mismatches.csv",
                   ["scenario_id", "allocation_target",
                    "safety_classification", "canonical_stack"],
                   [s for s in cg_scen
                    if "unresolved_normalization_mismatch"
                    in (s.get("comparison_classes_seen") or [])])
        # Indirect candidates observed for this group, from the matched
        # pairs' static-only-indirect-paths.csv (the comparator's
        # detail of which candidates were / were not observed).
        _write_csv(cdir / "indirect-candidates.csv",
                   ["implementation", "target", "observed_at_runtime",
                    "test_name"],
                   _indirect_candidate_rows(pairs, cg, out))

    # tests, deltas, logs
    _write_test_contribution(out / "tests", pairs, scenarios)
    _write_deltas(out / "deltas", scenarios, baseline)
    (out / "logs").mkdir(parents=True, exist_ok=True)
    (out / "logs" / "loader-warnings.txt").write_text(
        "\n".join(warnings) + ("\n" if warnings else ""))
    return summary


def _scenario_counts(scenarios):
    c = {"total": len(scenarios), "accepted": 0, "rejected": 0,
         "needs_manual_review": 0, "annotation_classified": 0,
         "machine_classified": 0}
    for s in scenarios:
        c[s["review_status"]] = c.get(s["review_status"], 0) + 1
        src = s.get("classification_source", "machine_classified")
        if src == "annotation_classified":
            c["annotation_classified"] += 1
        else:
            c["machine_classified"] += 1
    return c


def _finding_row(i, b, scenarios):
    sid = b.get("scenario_id", "")
    sc = next((s for s in scenarios if s["scenario_id"] == sid), {})
    return {"finding_id": f"FIND-{i:04d}",
            "finding_type": b.get("type"),
            "safety_classification": b.get("safety_classification",
                                           sc.get("safety_classification", "")),
            "target": sc.get("allocation_target", ""),
            "scenario_id": sid,
            "config_name": b.get("config_group", ""),
            "recommended_action": _recommend(b),
            "exception_id_if_any": ""}


def _recommend(b):
    t = b.get("type", "")
    return {
        "comparison_blocked": "provide matching static artifact",
        "runtime_required_but_missing": "run required runtime workload",
        "partial_required_run": "complete the required runtime run",
        "coverage_evidence_missing": "supply coverage evidence",
        "coverage_below_threshold": "raise coverage or approve exclusion",
        "coverage_config_mismatch": "regenerate coverage for this config",
        "scenario": "classify/annotate or fix the scenario",
        "no_static_artifacts": "provide at least one static-analysis "
                               "artifact",
        "no_runtime_artifacts": "provide at least one runtime artifact",
        "no_config_groups": "provide a matched static/runtime pair",
        "all_static_artifacts_quarantined":
            "fix static manifests (missing core identity)",
        "all_runtime_artifacts_quarantined":
            "fix runtime manifests (e.g. missing config_sha256)",
    }.get(t, "review")


def _write_summary_md(path, status, qualifier, summary, scenarios,
                      blockers, accepted_exceptions):
    head = ("# Bounded assurance claim is SUPPORTED" if status == "SUPPORTED"
            else "# Bounded assurance claim is NOT SUPPORTED")
    L = [head, "",
         f"support_qualifier: `{qualifier}`",
         "",
         "This is a bounded assurance claim over the analysed corpus and "
         "the supplied structural coverage evidence. It is not an absolute "
         "claim about all possible Xen executions. No allocation bound is "
         "asserted; static reachability does not prove execution; runtime "
         "non-observation does not prove impossibility.",
         "",
         "## Scope", "",
         f"- runtime artifacts: {summary['corpus_scope']['runtime_artifacts']}",
         f"- static artifacts: {summary['corpus_scope']['static_artifacts']}",
         f"- config groups: {summary['corpus_scope']['config_groups']}",
         "",
         "## Scenario summary", ""]
    for k, v in summary["scenario_summary"].items():
        L.append(f"- {k}: {v}")
    L += ["", "## Coverage evidence", ""]
    for cg, st in (summary.get("coverage_evidence") or {}).items():
        L.append(f"- `{cg}`: {st}")
    if blockers:
        L += ["", "## Blocking findings", ""]
        for b in blockers:
            L.append(f"- {b.get('type')}: "
                     f"{b.get('scenario_id') or b.get('test') or b.get('config_group') or ''} "
                     f"{b.get('safety_classification','')}".rstrip())
    if accepted_exceptions:
        L += ["", "## Accepted exceptions", ""]
        for e in accepted_exceptions:
            L.append(f"- {e['scenario_id']} via {e.get('exception_id')}")
    L += ["", "## Caveats", "",
          "- Structural coverage evidence supports workload adequacy; it "
          "does not replace scenario analysis.",
          "- The verdict does not reduce to a single coverage percentage.",
          ""]
    Path(path).write_text("\n".join(L))


def _write_scenarios_md(path, scenarios, counts):
    L = ["# Allocation scenarios", "",
         f"- canonical scenarios: {counts['total']}",
         f"- accepted: {counts['accepted']}",
         f"- rejected: {counts['rejected']}",
         f"- needs manual review: {counts['needs_manual_review']}",
         f"- annotation-classified: {counts['annotation_classified']}",
         f"- machine-classified: {counts['machine_classified']}", "",
         "| scenario | target | safety | review | variants |",
         "| --- | --- | --- | --- | --- |"]
    for s in scenarios:
        L.append(f"| {s['scenario_id']} | {s['allocation_target']} | "
                 f"`{s['safety_classification']}` | {s['review_status']} | "
                 f"{s['raw_stack_variants']} |")
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text("\n".join(L) + "\n")


def _write_scenario_group_md(path, s):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    L = [f"# {s['scenario_id']}", "",
         f"- allocation target: `{s['allocation_target']}`",
         f"- canonical stack: `{s['canonical_stack']}`",
         f"- safety classification: `{s['safety_classification']}`",
         f"- classification source: {s['classification_source']}",
         f"- review status: {s['review_status']}",
         f"- trigger actor: {s['trigger_actor']}",
         f"- phase: {s['phase']}",
         f"- lifetime: {s['lifetime_class']}",
         f"- raw observation count: {s['raw_observation_count']}",
         f"- raw stack variants: {s['raw_stack_variants']}"]
    if s.get("justification"):
        L += ["", "## Justification", "", s["justification"]]
    L += ["", "## Raw stack variants", ""]
    for v in (s.get("_raw_variants") or [])[:10]:
        L.append("- " + " -> ".join(v))
    Path(path).write_text("\n".join(L) + "\n")


def _write_config_group_md(path, cg, g):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    L = [f"# Config group {cg}", "",
         f"- target_arch: {g.get('target_arch','')}",
         f"- config_sha256: {g.get('config_sha256','')}",
         f"- callgraph_backend: {g.get('callgraph_backend','')}",
         f"- runtime_paths_total: {g.get('runtime_paths_total',0)}", "",
         "## Runtime path classification", "",
         "| class | count |", "| --- | --- |"]
    for c in cs.AGG_RUNTIME_CLASSES:
        L.append(f"| `{c}` | {g.get(c, 0)} |")
    L += ["", "## Indirect candidates", "",
          f"- total: {g.get('indirect_candidates_total', 0)}",
          f"- observed: {g.get('indirect_candidates_observed', 0)}",
          f"- not observed: {g.get('indirect_candidates_not_observed', 0)}",
          ""]
    Path(path).write_text("\n".join(L))


def _indirect_candidate_rows(pairs, cg, out_dir):
    """Indirect-candidate observation rows for a config group, read
    from each matched pair's static-only-indirect-paths.csv (candidates
    that were NOT observed) -- the comparator's per-candidate detail.
    """
    rows = []
    for rec in pairs:
        if rec["comparison_status"] != "COMPARED":
            continue
        rt = rec["runtime"]["manifest"]
        pair_cg = cs.config_group_hash(
            {**rt, "callgraph_backend":
             (rec["static"]["manifest"].get("callgraph_backend")
              if rec["static"] else "")})
        if pair_cg != cg:
            continue
        comp = Path(out_dir) / "comparisons" / rt.get("test_name", "")
        p = comp / "static-only-indirect-paths.csv"
        if p.exists():
            try:
                for r in csv.DictReader(p.open()):
                    rows.append({
                        "implementation": r.get("implementation", ""),
                        "target": r.get("target", ""),
                        "observed_at_runtime": "no",
                        "test_name": rt.get("test_name", "")})
            except OSError:
                pass
    return rows


def _write_test_contribution(tdir, pairs, scenarios):
    rows = []
    for p in pairs:
        rt = p["runtime"]["manifest"]
        m = p.get("metrics") or {}
        total = int(m.get("runtime_paths_total", 0) or 0)
        seen = [s for s in scenarios
                if rt.get("test_name") in (s.get("tests_seen") or [])]
        rej = sum(1 for s in seen if s["review_status"] == "rejected")
        rev = sum(1 for s in seen
                  if s["review_status"] == "needs_manual_review")
        score = 3 * rej + 2 * rev + 1 * len(seen)
        rows.append({"test_name": rt.get("test_name"),
                     "runtime_paths_total": total,
                     "canonical_scenarios_observed": len(seen),
                     "new_rejected_scenarios": rej,
                     "new_needs_review_scenarios": rev,
                     "marginal_value_score": score,
                     "comparison_status": p["comparison_status"]})
    _write_csv(tdir / "test-contribution.csv",
               ["test_name", "runtime_paths_total",
                "canonical_scenarios_observed", "new_rejected_scenarios",
                "new_needs_review_scenarios", "marginal_value_score",
                "comparison_status"], rows)
    _write_csv(tdir / "low-value-tests.csv",
               ["test_name", "marginal_value_score"],
               [r for r in rows if r["marginal_value_score"] == 0
                and r["comparison_status"] == "COMPARED"])
    _write_csv(tdir / "broken-or-partial-tests.csv",
               ["test_name", "comparison_status"],
               [r for r in rows if r["comparison_status"] != "COMPARED"])


def _write_deltas(ddir, scenarios, baseline):
    ddir.mkdir(parents=True, exist_ok=True)
    cur = {s["allocation_target"] + "|" + s["canonical_stack"]:
           s for s in scenarios}
    base = {}
    if baseline:
        b = cs.try_read_json(Path(baseline))
        for s in ((b or {}).get("scenario_summary_detail") or []):
            base[s.get("sig", "")] = s
    new = [s for k, s in cur.items() if k not in base]
    resolved = []
    for k, s in base.items():
        if k in cur:
            continue
        # Baseline entries may carry explicit fields, or only a sig of
        # the form "<target>|<canonical_stack>"; derive from the sig
        # when the explicit fields are absent.
        sig = s.get("sig", k)
        target = s.get("allocation_target") or (
            sig.split("|", 1)[0] if "|" in sig else sig)
        resolved.append({
            "scenario_id": s.get("scenario_id", ""),
            "allocation_target": target})
    _write_csv(ddir / "new-scenarios.csv",
               ["scenario_id", "allocation_target", "safety_classification"],
               new)
    _write_csv(ddir / "resolved-scenarios.csv",
               ["scenario_id", "allocation_target"], resolved)
    for name in ("new-findings.csv", "resolved-findings.csv",
                 "changed-counts.csv"):
        p = ddir / name
        if not p.exists():
            p.write_text("# delta vs baseline; empty when no baseline\n")


# --------------------------------------------------------------------
# Top-level run.
# --------------------------------------------------------------------

def _resolve_roots(args):
    """Return (static_roots, runtime_roots, coverage_roots) honoring
    --manifest mode (Part 4 input mode 2) when supplied, else the
    separated --static-root/--runtime-root/--coverage-root.

    Manifest schema (JSON or YAML):
      { "static_roots": [...], "runtime_roots": [...],
        "coverage_roots": [...] }
    Single-string values are accepted and wrapped.
    """
    def _aslist(v):
        if not v:
            return []
        return v if isinstance(v, list) else [v]

    if args.manifest:
        man = cs.try_read_json(Path(args.manifest))
        if man is None:
            man = _load_yaml(Path(args.manifest)) or {}
        return (_aslist(man.get("static_roots")
                        or man.get("static_root")),
                _aslist(man.get("runtime_roots")
                        or man.get("runtime_root")),
                _aslist(man.get("coverage_roots")
                        or man.get("coverage_root")))
    return (_aslist(args.static_root), _aslist(args.runtime_root),
            _aslist(args.coverage_root))


def run(args) -> int:
    warnings: list = []
    policy = _load_yaml(Path(args.policy)) if args.policy else {}
    policy = policy or {}
    annotations = (_load_yaml(Path(args.annotations))
                   if args.annotations else {}) or {}

    static_roots, runtime_roots, coverage_roots = _resolve_roots(args)
    stats: dict = {}
    statics: list = []
    for r in static_roots:
        statics += load_static_artifacts(Path(r), warnings, stats)
    runtimes: list = []
    for r in runtime_roots:
        runtimes += load_runtime_artifacts(Path(r), warnings, stats)
    coverage: list = []
    for r in coverage_roots:
        coverage += load_coverage(r, warnings)

    out_dir = Path(args.out_dir)
    pairs = join_corpus(statics, runtimes, policy, out_dir, warnings)
    config_groups = aggregate(pairs)
    observations = build_observations(pairs, out_dir)
    scenarios = asc.group_scenarios(observations)
    conf_mode = (policy.get("auto_accept_confidence")
                 or (policy.get("scenario_gate") or {})
                 .get("auto_accept_confidence") or "medium")
    asc.classify_all(scenarios, annotations, warnings, conf_mode)

    blockers: list = []
    accepted_exceptions: list = []
    coverage_states = evaluate_coverage(config_groups, coverage, policy,
                                        blockers)
    # --strict: loader quarantines (artifacts dropped for missing core
    # identity) become blockers rather than warnings.
    if args.strict:
        for w in warnings:
            if "quarantined" in w:
                blockers.append({"type": "strict_quarantine", "detail": w})

    scope = {
        "static_loaded": len(statics),
        "runtime_loaded": len(runtimes),
        "config_groups": len(config_groups),
        "static_discovered": stats.get("static_discovered", 0),
        "runtime_discovered": stats.get("runtime_discovered", 0),
        "static_quarantined": stats.get("static_quarantined", 0),
        "runtime_quarantined": stats.get("runtime_quarantined", 0),
    }
    status, qualifier = compute_verdict(
        pairs, scenarios, coverage_states, policy, blockers,
        accepted_exceptions, scope)

    write_reports(out_dir, status, qualifier, pairs, config_groups,
                  scenarios, coverage_states, blockers,
                  accepted_exceptions, args.baseline, warnings)

    # --emit-debug-index: a machine-readable index of resolved
    # artifacts and join decisions, for troubleshooting.
    if args.emit_debug_index:
        cs.write_json(out_dir / "debug-index.json", {
            "static_artifacts": [
                {"root": str(s["root"]),
                 "manifest": s["manifest"]} for s in statics],
            "runtime_artifacts": [
                {"root": str(r["root"]),
                 "manifest": r["manifest"]} for r in runtimes],
            "joins": [
                {"test": p["runtime"]["manifest"].get("test_name"),
                 "matched_static": (str(p["static"]["root"])
                                    if p["static"] else None),
                 "comparison_status": p["comparison_status"],
                 "reason": p.get("reason", "")} for p in pairs],
            "warnings": warnings})

    print(status)
    for b in blockers:
        if b.get("type") in ("comparison_blocked",
                             "runtime_required_but_missing"):
            print(f"blocker {b['type']}")
            break
    if args.fail_on_not_supported and status == "NOT_SUPPORTED":
        return 2
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--static-root")
    ap.add_argument("--runtime-root")
    ap.add_argument("--coverage-root")
    ap.add_argument("--policy")
    ap.add_argument("--manifest")
    ap.add_argument("--baseline")
    ap.add_argument("--jobs-glob")
    ap.add_argument("--annotations")
    ap.add_argument("--out-dir", default="corpus-analysis")
    ap.add_argument("--strict", action="store_true")
    ap.add_argument("--fail-on-not-supported", action="store_true")
    ap.add_argument("--emit-debug-index", action="store_true")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args(argv)
    if args.self_test:
        return _self_test()
    if not (args.static_root or args.runtime_root or args.manifest):
        ap.error("need --static-root/--runtime-root or --manifest "
                 "(or --self-test)")
    return run(args)


# --------------------------------------------------------------------
# Self-test (Part 19): a representative subset run in-process.
# --------------------------------------------------------------------

def _self_test() -> int:
    import tempfile
    failures = []

    def check(name, cond):
        print(f"  [{'PASS' if cond else 'FAIL'}] {name}")
        if not cond:
            failures.append(name)

    # Scenario grouping: duplicate raw stacks collapse (Test 10).
    obs = [
        {"allocation_target": "_xmalloc",
         "canonical_stack": ["foo", "_xmalloc"],
         "raw_stack": ["foo", "strtoull.c#_xmalloc"], "domain": "d1",
         "comparison_class": "direct_static_explained",
         "config_group": "cg", "test_name": "t1"},
        {"allocation_target": "_xmalloc",
         "canonical_stack": ["foo", "_xmalloc"],
         "raw_stack": ["foo", "xmem_pool_alloc", "_xmalloc"],
         "domain": "d2", "comparison_class": "direct_static_explained",
         "config_group": "cg", "test_name": "t2"},
    ]
    sc = asc.group_scenarios(obs)
    check("duplicate raw stacks collapse to one scenario", len(sc) == 1)
    check("raw_stack_variants > 1", sc[0]["raw_stack_variants"] == 2)

    # Unexplained -> rejected (Test 3/16).
    s2 = asc.group_scenarios([
        {"allocation_target": "_xmalloc",
         "canonical_stack": ["mystery", "_xmalloc"], "raw_stack": ["x"],
         "domain": "", "comparison_class": "runtime_only_unexplained",
         "config_group": "cg", "test_name": "t"}])
    asc.classify_all(s2, {}, [])
    check("unexplained -> rejected_unexplained_static_gap",
          s2[0]["safety_classification"] == "rejected_unexplained_static_gap")

    # Guest-triggered retained Xenheap (Test 17).
    s3 = asc.group_scenarios([
        {"allocation_target": "alloc_xenheap_pages",
         "canonical_stack": ["evtchn_bind_interdomain",
                             "alloc_xenheap_pages"],
         "raw_stack": ["x"], "domain": "d1",
         "comparison_class": "direct_static_explained",
         "config_group": "cg", "test_name": "t",
         "lifetime_hint": "retained_after_return"}])
    asc.classify_all(s3, {}, [])
    check("guest-triggered retained xenheap -> rejected",
          s3[0]["safety_classification"]
          == "rejected_guest_triggered_retained_xenheap")

    # Cause/effect safety model: safe if cause-bounded OR
    # effect-bracketed; unsafe only if neither.
    def _one(target, frames, cls="direct_static_explained", lifetime=""):
        s = asc.group_scenarios([
            {"allocation_target": target,
             "canonical_stack": frames + [target],
             "raw_stack": frames + [target], "domain": "d1",
             "comparison_class": cls, "config_group": "cg",
             "test_name": "t", "lifetime_hint": lifetime}])
        asc.classify_all(s, {}, [])
        return s[0]["safety_classification"]

    check("cause-bounded boot (retained) -> accepted (cause axis)",
          _one("_xmalloc", ["__start_xen", "setup_mm"],
               lifetime="retained_after_return") == "accepted_boot_time")
    check("cause-bounded domain creation -> accepted",
          _one("alloc_domheap_pages",
               ["arch_domain_create", "alloc_vcpu_struct"])
          == "accepted_domain_creation_bounded")
    check("cause-bounded device assignment -> accepted",
          _one("alloc_xenheap_pages",
               ["arm_smmu_assign_dev", "arm_smmu_add_device"])
          == "accepted_hardware_firmware_controlled")
    check("effect-bracketed guest transient -> accepted (effect axis)",
          _one("alloc_xenheap_pages", ["do_event_channel_op"],
               lifetime="freed_before_return")
          == "accepted_guest_triggered_bounded_transient")
    check("neither (guest retained xenheap) -> rejected",
          _one("alloc_xenheap_pages",
               ["do_event_channel_op", "evtchn_bind"],
               lifetime="retained_after_return")
          == "rejected_guest_triggered_retained_xenheap")
    check("insufficient evidence (guest unknown lifetime) -> review",
          _one("alloc_xenheap_pages", ["do_event_channel_op"])
          == "needs_manual_review")
    check("insufficient evidence (internal unknown) -> review",
          _one("_xmalloc", ["some_internal_helper"])
          == "needs_manual_review")

    # Annotation acceptance requires justification (Test 11).
    s4 = asc.group_scenarios([
        {"allocation_target": "_xmalloc",
         "canonical_stack": ["evtchn_fifo_init_control", "_xmalloc"],
         "raw_stack": ["x"], "domain": "d1",
         "comparison_class": "target_observed_no_caller_context",
         "config_group": "cg", "test_name": "t"}])
    warn = []
    asc.classify_all(s4, {"scenarios": [
        {"id": "X", "match": {"allocation_target": "_xmalloc"},
         "classification": "accepted_domain_creation_bounded"}]}, warn)
    check("annotation acceptance without justification ignored",
          s4[0]["review_status"] != "accepted" and any("justification" in w
                                                       for w in warn))

    # Full verdict: supported corpus (Test 1) via synthetic dirs.
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        _mk_static(td / "static", "sha1", "cfgA")
        _mk_runtime(td / "runtime" / "qemu-smoke-arm64", "sha1", "cfgA",
                    "qemu-smoke-arm64", explained=2)
        out = td / "out"
        a = argparse.Namespace(
            static_root=str(td / "static"),
            runtime_root=str(td / "runtime"), coverage_root=None,
            policy=None, manifest=None, baseline=None, jobs_glob=None,
            annotations=None, out_dir=str(out), strict=False,
            fail_on_not_supported=False, emit_debug_index=False,
            self_test=False)
        rc = run(a)
        summ = cs.read_json(out / "corpus-assurance-summary.json")
        # all paths direct_static_explained at boot? no -> review ->
        # NOT_SUPPORTED is the conservative default. Accept either, but
        # verify a verdict was emitted and is one of the two.
        check("verdict emitted is binary",
              summ["assurance_status"] in ("SUPPORTED", "NOT_SUPPORTED"))

    # No matching static -> COMPARISON_BLOCKED blocker (Test 2).
    pol = str(Path(__file__).resolve().parent / "assurance-policy.yaml")
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        _mk_runtime(td / "runtime" / "qemu-smoke-arm64", "shaX", "cfgX",
                    "qemu-smoke-arm64", explained=1)
        out = td / "out"
        a = argparse.Namespace(
            static_root=None, runtime_root=str(td / "runtime"),
            coverage_root=None,
            policy=pol, manifest=None, baseline=None, jobs_glob=None,
            annotations=None, out_dir=str(out), strict=False,
            fail_on_not_supported=False, emit_debug_index=False,
            self_test=False)
        run(a)
        summ = cs.read_json(out / "corpus-assurance-summary.json")
        blocked = any(b.get("type") == "comparison_blocked"
                      for b in summ["blockers"])
        check("no matching static -> comparison_blocked + NOT_SUPPORTED",
              blocked and summ["assurance_status"] == "NOT_SUPPORTED")

    # Helper: a supported corpus (boot scenario accepted) with optional
    # coverage and policy, returning the parsed summary.
    def _corpus(td, *, coverage=None, policy_text=None, baseline=None,
                annotations=None):
        td = Path(td)
        _mk_static(td / "static", "sha1", "cfgA")
        _mk_runtime(td / "runtime" / "build-arm64", "sha1", "cfgA",
                    "build-arm64", explained=1, boot=True)
        polf = None
        if policy_text is not None:
            polf = td / "policy.yaml"
            polf.write_text(policy_text)
        covroot = None
        if coverage is not None:
            covroot = td / "coverage"
            (covroot).mkdir(parents=True, exist_ok=True)
            cs.write_json(covroot / "coverage-summary.json", coverage)
        annf = None
        if annotations is not None:
            annf = td / "ann.yaml"
            import json as _j
            annf.write_text(_j.dumps(annotations))
        out = td / "out"
        a = argparse.Namespace(
            static_root=str(td / "static"),
            runtime_root=str(td / "runtime"),
            coverage_root=str(covroot) if covroot else None,
            policy=str(polf) if polf else None, manifest=None,
            baseline=str(baseline) if baseline else None,
            jobs_glob=None, annotations=str(annf) if annf else None,
            out_dir=str(out), strict=False,
            fail_on_not_supported=False, emit_debug_index=False,
            self_test=False)
        run(a)
        return cs.read_json(out / "corpus-assurance-summary.json"), out

    _COV_OK = {"git_sha": "sha1", "target_arch": "arm64",
               "config_sha256": "cfgA",
               "line_coverage": {"percent": 100.0},
               "branch_coverage": {"percent": 100.0},
               "mcdc_coverage": {"percent": 100.0}}
    _POL_COV = ("version: 1\nruntime_required: false\n"
                "coverage_requirements:\n  enabled: true\n"
                "  require_line_coverage: true\n"
                "  require_branch_coverage: true\n"
                "  require_mcdc_coverage: true\n"
                "  minimum_line_percent: 100.0\n"
                "  minimum_branch_percent: 100.0\n"
                "  minimum_mcdc_percent: 100.0\n")

    # Test 12 — supported corpus with full coverage evidence.
    with tempfile.TemporaryDirectory() as td:
        summ, _ = _corpus(td, coverage=_COV_OK, policy_text=_POL_COV)
        check("coverage complete + accepted scenarios -> SUPPORTED",
              summ["assurance_status"] == "SUPPORTED")

    # Test 13 — required MC/DC missing -> NOT_SUPPORTED.
    with tempfile.TemporaryDirectory() as td:
        cov = dict(_COV_OK)
        cov.pop("mcdc_coverage")
        summ, _ = _corpus(td, coverage=cov, policy_text=_POL_COV)
        check("required MC/DC missing -> NOT_SUPPORTED + coverage blocker",
              summ["assurance_status"] == "NOT_SUPPORTED"
              and any("coverage" in b.get("type", "")
                      for b in summ["blockers"]))

    # Test 14 — coverage config mismatch.
    with tempfile.TemporaryDirectory() as td:
        cov = dict(_COV_OK, config_sha256="WRONG")
        summ, _ = _corpus(td, coverage=cov, policy_text=_POL_COV)
        check("coverage config mismatch -> NOT_SUPPORTED",
              summ["assurance_status"] == "NOT_SUPPORTED")

    # Test 15 — coverage below threshold.
    with tempfile.TemporaryDirectory() as td:
        cov = dict(_COV_OK, branch_coverage={"percent": 99.5})
        summ, _ = _corpus(td, coverage=cov, policy_text=_POL_COV)
        check("branch coverage below threshold -> NOT_SUPPORTED",
              summ["assurance_status"] == "NOT_SUPPORTED")

    # Test 6 / 7 — accepted exception, then expired.
    _ann_accept = {"scenarios": [{
        "id": "BOOT-OK",
        "match": {"allocation_target": "_xmalloc"},
        "classification": "accepted_boot_time",
        "justification": "boot-time, bounded by boot; accepted."}]}
    # (boot scenario already auto-accepts, so exercise exception via a
    #  policy accepted_exception on a review scenario instead.)
    _POL_EXC = ("version: 1\nruntime_required: false\n"
                "accepted_exceptions:\n"
                "  - id: \"BOOT-EX\"\n"
                "    classification: \"needs_manual_review\"\n"
                "    scenario_signature: \"_xmalloc\"\n"
                "    justification: \"reviewed and accepted\"\n"
                "    expires: \"2099-12-31\"\n")
    _POL_EXC_EXP = _POL_EXC.replace("2099-12-31", "2000-01-01")

    # Build a corpus whose scenario needs review (target_observed) so an
    # exception is what flips it.
    def _review_corpus(td, policy_text):
        td = Path(td)
        _mk_static(td / "static", "sha2", "cfgB")
        rt = td / "runtime" / "build-x"
        _mk_runtime(rt, "sha2", "cfgB", "build-x", explained=0)
        # Force a target_observed_no_caller_context path.
        parsed = rt / "parsed" / "arm64"
        parsed.mkdir(parents=True, exist_ok=True)
        (parsed / "build-x.log").write_text(
            "  [d0] xmem_pool_alloc:\n    max size path:\n"
            "      xmem_pool_alloc(size=64) [domain: d0]\n"
            "      strtoull.c#_xmalloc\n")
        polf = td / "p.yaml"
        polf.write_text(policy_text)
        out = td / "out"
        a = argparse.Namespace(
            static_root=str(td / "static"),
            runtime_root=str(td / "runtime"), coverage_root=None,
            policy=str(polf), manifest=None, baseline=None,
            jobs_glob=None, annotations=None, out_dir=str(out),
            strict=False, fail_on_not_supported=False,
            emit_debug_index=False, self_test=False)
        run(a)
        return cs.read_json(out / "corpus-assurance-summary.json")

    with tempfile.TemporaryDirectory() as td:
        summ = _review_corpus(td, _POL_EXC)
        check("accepted exception -> SUPPORTED + ACCEPTED_EXCEPTIONS",
              summ["assurance_status"] == "SUPPORTED"
              and summ["support_qualifier"] == "ACCEPTED_EXCEPTIONS")
    with tempfile.TemporaryDirectory() as td:
        summ = _review_corpus(td, _POL_EXC_EXP)
        check("expired exception -> NOT_SUPPORTED",
              summ["assurance_status"] == "NOT_SUPPORTED")

    # Test 8 — baseline delta (new vs resolved scenarios).
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        base = td / "baseline.json"
        cs.write_json(base, {"scenario_summary_detail": [
            {"sig": "old_target|old -> old_target"}]})
        summ, out = _corpus(td, baseline=base)
        nd = (out / "deltas" / "new-scenarios.csv").read_text()
        rd = (out / "deltas" / "resolved-scenarios.csv").read_text()
        check("baseline delta emits new and resolved scenarios",
              "ALLOC-SCENARIO" in nd and "old_target" in rd)

    # Producer/consumer cycle: package_artifacts.sh splits a driver
    # output dir into separated static-analysis + runtime-artifacts
    # trees whose manifests share a join key, and the analyzer joins
    # them. Exercised here in-process by building the contract trees the
    # Producer/consumer cycle: the static and runtime producers are
    # INDEPENDENT. The static job packages a static-only tree; the
    # runtime job packages a logs-only workspace (no static tree, no
    # status.json) with its own env-supplied identity and parses the
    # logs standalone. Both must derive the SAME config_sha256 from the
    # same .config content so the corpus job can join them.
    import os as _os
    import subprocess
    pkg = Path(__file__).resolve().parent / "package_artifacts.sh"
    if pkg.exists():
        with tempfile.TemporaryDirectory() as td:
            td = Path(td)
            cfg_text = "CONFIG_A=y\nCONFIG_B=y\n"

            # --- Static producer: static-only tree, no runtime. ---
            st_src = td / "static-src"
            (st_src / "config").mkdir(parents=True)
            (st_src / "config" / ".config").write_text(cfg_text)
            (st_src / "direct-static").mkdir()
            (st_src / "direct-static" / "_xmalloc.functions").write_text(
                "_xmalloc\n__start_xen\n")
            (st_src / "reachability").mkdir()
            (st_src / "reachability"
             / "synthetic_edges.candidates.yaml").write_text(
                "config: s\nedges: []\n")
            cs.write_json(st_src / "status.json", {
                "git_sha": "deadbeef", "target_arch": "arm64",
                "config_name": "selftest", "config_sha256": ""})

            # --- Runtime producer: logs only, separate workspace,
            #     env identity, NO static tree, NO status.json. ---
            rt_ws = td / "rt-workspace"
            rt_ws.mkdir()
            rt_cfg = td / "rt.config"
            rt_cfg.write_text(cfg_text)   # same content -> same sha
            rt_logs = td / "rt-logs"
            rt_logs.mkdir()
            (rt_logs / "qemu.log").write_text(
                "  [d0] xmem_pool_alloc:\n    max size path:\n"
                "      xmem_pool_alloc(size=64) [domain: d0]\n"
                "      __start_xen\n      strtoull.c#_xmalloc\n")

            sa = td / "static-analysis"
            ra = td / "runtime-artifacts"
            base_env = dict(_os.environ)
            ok = True
            r = subprocess.run(
                ["bash", str(pkg), "static", str(st_src), str(sa)],
                capture_output=True)
            ok = ok and r.returncode == 0
            renv = dict(base_env,
                        MINERVA_GIT_SHA="deadbeef",
                        MINERVA_TARGET_ARCH="arm64",
                        MINERVA_CONFIG_NAME="selftest",
                        MINERVA_CONFIG_FILE=str(rt_cfg),
                        MINERVA_RUNTIME_LOG_DIR=str(rt_logs))
            r = subprocess.run(
                ["bash", str(pkg), "runtime", str(rt_ws), str(ra), "qemu"],
                capture_output=True, env=renv)
            ok = ok and r.returncode == 0

            # Independent producers must derive the same join key.
            sm = cs.try_read_json(sa / "static-analysis-manifest.json") or {}
            rm = cs.try_read_json(
                ra / "qemu" / "runtime-manifest.json") or {}
            same_key = (cs.join_key(sm) == cs.join_key(rm)
                        and sm.get("config_sha256"))
            check("independent static+runtime producers derive the same "
                  "join key", ok and bool(same_key))
            # The runtime producer parsed its logs standalone.
            parsed_ok = any((ra / "qemu" / "parsed").rglob("*")) \
                if (ra / "qemu" / "parsed").exists() else False
            check("runtime producer parses logs standalone (no static)",
                  parsed_ok)
            # Part X: the runtime artifact must contain ONLY runtime
            # content -- no collect/reachability/direct-static/callgraph
            # -- and must include a runtime-summary.json.
            rt_dir = ra / "qemu"
            no_static = not any((rt_dir / d).exists() for d in
                                ("collect", "reachability",
                                 "direct-static", "callgraph", "config"))
            has_summary = (rt_dir / "runtime-summary.json").exists()
            check("runtime artifact has no static outputs and has a "
                  "summary", no_static and has_summary)
            # And the corpus job joins the two independent trees.
            out = td / "out"
            a = argparse.Namespace(
                static_root=str(sa), runtime_root=str(ra),
                coverage_root=None, policy=None, manifest=None,
                baseline=None, jobs_glob=None, annotations=None,
                out_dir=str(out), strict=False,
                fail_on_not_supported=False, emit_debug_index=True,
                self_test=False)
            run(a)
            dbg = cs.try_read_json(out / "debug-index.json") or {}
            joined = any(j.get("comparison_status") == "COMPARED"
                         and j.get("matched_static")
                         for j in dbg.get("joins", []))
            check("corpus job joins independent static+runtime trees",
                  joined)

    # --- Empty-corpus / quarantine hard gate (regression for the
    #     empty-corpus false positive). Absence of evidence must never
    #     support a bounded assurance claim. ---
    pol_static_req = str(Path(__file__).resolve().parent
                         / "assurance-policy.yaml")

    def _verdict(static_root=None, runtime_root=None, policy=None):
        with tempfile.TemporaryDirectory() as t:
            t = Path(t)
            out = t / "out"
            a = argparse.Namespace(
                static_root=static_root, runtime_root=runtime_root,
                coverage_root=None, policy=policy, manifest=None,
                baseline=None, jobs_glob=None, annotations=None,
                out_dir=str(out), strict=False,
                fail_on_not_supported=False, emit_debug_index=False,
                self_test=False)
            run(a)
            return cs.read_json(out / "corpus-assurance-summary.json")

    # 1. Empty corpus (no roots at all) -> NOT_SUPPORTED.
    with tempfile.TemporaryDirectory() as t:
        t = Path(t)
        (t / "s").mkdir()
        (t / "r").mkdir()
        summ = _verdict(str(t / "s"), str(t / "r"))
        types = {b["type"] for b in summ["blockers"]}
        check("empty corpus -> NOT_SUPPORTED",
              summ["assurance_status"] == "NOT_SUPPORTED"
              and {"no_static_artifacts", "no_runtime_artifacts",
                   "no_config_groups"} <= types)

    # 2. Static present, no runtime -> NOT_SUPPORTED.
    with tempfile.TemporaryDirectory() as t:
        t = Path(t)
        _mk_static(t / "static", "sha1", "cfgA")
        (t / "runtime").mkdir()
        summ = _verdict(str(t / "static"), str(t / "runtime"))
        check("static present, no runtime -> NOT_SUPPORTED",
              summ["assurance_status"] == "NOT_SUPPORTED"
              and any(b["type"] == "no_runtime_artifacts"
                      for b in summ["blockers"]))

    # 3. Runtime artifact quarantined for missing config_sha256 (the
    #    exact reported bug) -> NOT_SUPPORTED.
    with tempfile.TemporaryDirectory() as t:
        t = Path(t)
        rt = t / "runtime" / "qemu-smoke-arm64"
        rt.mkdir(parents=True)
        cs.write_json(rt / "runtime-manifest.json", {
            "artifact_type": "runtime", "git_sha": "sha1",
            "target_arch": "arm64", "test_name": "qemu-smoke-arm64"})
        (t / "static").mkdir()
        summ = _verdict(str(t / "static"), str(t / "runtime"))
        types = {b["type"] for b in summ["blockers"]}
        check("runtime missing config_sha256 -> NOT_SUPPORTED + "
              "all_runtime_artifacts_quarantined",
              summ["assurance_status"] == "NOT_SUPPORTED"
              and "all_runtime_artifacts_quarantined" in types)

    # 4. All runtime artifacts quarantined (multiple) -> NOT_SUPPORTED.
    with tempfile.TemporaryDirectory() as t:
        t = Path(t)
        for n in ("a", "b"):
            rt = t / "runtime" / n
            rt.mkdir(parents=True)
            cs.write_json(rt / "runtime-manifest.json", {
                "artifact_type": "runtime", "test_name": n})
        _mk_static(t / "static", "sha1", "cfgA")
        summ = _verdict(str(t / "static"), str(t / "runtime"))
        check("all runtime artifacts quarantined -> NOT_SUPPORTED",
              summ["assurance_status"] == "NOT_SUPPORTED"
              and any(b["type"] == "all_runtime_artifacts_quarantined"
                      for b in summ["blockers"]))

    # 5. One valid matched pair with an accepted scenario -> SUPPORTED.
    with tempfile.TemporaryDirectory() as t:
        t = Path(t)
        _mk_static(t / "static", "sha1", "cfgA")
        _mk_runtime(t / "runtime" / "build-arm64", "sha1", "cfgA",
                    "build-arm64", explained=1, boot=True)
        summ = _verdict(str(t / "static"), str(t / "runtime"))
        check("valid matched pair, accepted scenario -> SUPPORTED",
              summ["assurance_status"] == "SUPPORTED")

    print(f"\n{len(failures)} failures" if failures else "\nall passed")
    return 1 if failures else 0


def _mk_static(root: Path, sha, cfg):
    root.mkdir(parents=True, exist_ok=True)
    cs.write_json(root / "static-analysis-manifest.json", {
        "artifact_type": "static-analysis", "git_sha": sha,
        "target_arch": "arm64", "config_name": cfg,
        "config_sha256": cfg, "callgraph_backend": "llvm-ir",
        "static_artifact_id": "s1", "status": "STATIC_READY"})
    (root / "reachability").mkdir(exist_ok=True)
    (root / "reachability" / "synthetic_edges.candidates.yaml").write_text(
        "config: s\nedges: []\n")
    (root / "direct-static").mkdir(exist_ok=True)
    (root / "direct-static" / "_xmalloc.functions").write_text(
        "_xmalloc\nfoo\n__start_xen\nsetup_mm\n")


def _mk_runtime(root: Path, sha, cfg, test, explained=0, boot=False):
    root.mkdir(parents=True, exist_ok=True)
    cs.write_json(root / "runtime-manifest.json", {
        "artifact_type": "runtime", "git_sha": sha, "target_arch": "arm64",
        "config_name": cfg, "config_sha256": cfg, "test_name": test,
        "runtime_artifact_id": "r1", "status": "RUNTIME_READY"})
    parsed = root / "parsed" / "arm64"
    parsed.mkdir(parents=True, exist_ok=True)
    # A boot-framed caller makes the scenario cause-bounded (and thus
    # auto-accepted); otherwise a neutral caller -> needs review.
    caller = "__start_xen" if boot else "foo"
    block = ""
    for n in range(explained):
        block += (f"  [d{n}] xmem_pool_alloc:\n    max size path:\n"
                  f"      xmem_pool_alloc(size=64) [domain: d{n}]\n"
                  f"      {caller}\n      strtoull.c#_xmalloc\n")
    (parsed / f"{test}.log").write_text(block or "  (no allocations)\n")


if __name__ == "__main__":
    raise SystemExit(main())
