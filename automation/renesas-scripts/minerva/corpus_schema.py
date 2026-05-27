#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Shared schemas and identity helpers for corpus assurance analysis.

This module is the single source of truth for the field names and
identity computations used across the corpus analyzer's layers
(loaders, join, aggregation, scenarios, coverage, policy, reports).
Keeping these here prevents the layers from inventing incompatible
JSON/CSV shapes.

Nothing here decides assurance support; it only defines vocabulary
and deterministic identities.
"""
from __future__ import annotations

import hashlib
import json
import re
from pathlib import Path


# --------------------------------------------------------------------
# Artifact identity.
# --------------------------------------------------------------------

# The join key that ties a runtime artifact to the static artifact it
# must be compared against. Comparing across a differing join key is
# only allowed under an explicit PROXY policy, never by default.
JOIN_KEY_FIELDS = ("git_sha", "target_arch", "config_sha256")

STATIC_REQUIRED_IDENTITY = (
    "git_sha", "target_arch", "config_sha256",
    "callgraph_backend", "static_artifact_id")
RUNTIME_REQUIRED_IDENTITY = (
    "git_sha", "target_arch", "config_sha256",
    "test_name", "runtime_artifact_id")

# The minimal subset we can never derive or synthesize. If any of
# these is absent the artifact is quarantined; the rest of the
# "required" identity above may be derived (see loaders).
STATIC_CORE_IDENTITY = ("git_sha", "config_sha256")
RUNTIME_CORE_IDENTITY = ("git_sha", "config_sha256")


def stable_id(prefix: str, *parts: str) -> str:
    """Deterministic short id from identity parts.

    Used to synthesize *_artifact_id when a producer did not emit one,
    so the same artifact always gets the same id across runs.
    """
    h = hashlib.sha256("\x1f".join(p or "" for p in parts).encode()).hexdigest()
    return f"{prefix}-{h[:16]}"


def join_key(manifest: dict) -> tuple:
    return tuple(str(manifest.get(f, "") or "") for f in JOIN_KEY_FIELDS)


def config_group_key(manifest: dict) -> tuple:
    """Config group is (target_arch, config_sha256, callgraph_backend)."""
    return (str(manifest.get("target_arch", "") or ""),
            str(manifest.get("config_sha256", "") or ""),
            str(manifest.get("callgraph_backend", "") or ""))


def config_group_hash(manifest: dict) -> str:
    return stable_id("cfg", *config_group_key(manifest))


# --------------------------------------------------------------------
# Frame normalization (shared with runtime/static comparison).
# --------------------------------------------------------------------

# The declared allocator-alias map. Kept identical in intent to the
# runtime/static comparator's ALLOCATOR_ALIASES: explicit, declared,
# source-grounded, NOT fuzzy. See runtime_static_compare.py and
# docs/minerva-analysis/runtime-static-comparison.md.
ALLOCATOR_ALIASES = {
    "xmem_pool_alloc": "_xmalloc",   # WARN inside _xmalloc(), tlsf.c
}

# Unstable stack-dump frames that carry no allocation-path meaning and
# would otherwise split otherwise-identical scenarios. Declared, not
# pattern-guessed.
DUMP_STACK_FRAMES = frozenset({
    "dump_stack", "show_stack", "_show_registers", "show_registers",
    "do_bug_frame", "vcpu_show_execution_state", "show_execution_state",
})

_LINE_SUFFIX_RE = re.compile(r"[:#]\d+$")
_BUILD_PREFIX_RE = re.compile(r"^(?:\.\./)+|^/[^ ]*?/xen/")


def normalize_frame(token: str) -> str:
    """Normalize one frame token to a bare, canonical function name.

    - strip a build-directory prefix from a path-qualified token;
    - convert `file.c#func` (or `file.c:func`) to `func`;
    - strip a trailing line number;
    - apply the declared allocator-alias map.

    Returns "" for a token that is only a location with no function.
    Never fuzzy-matches.
    """
    t = (token or "").strip()
    if not t:
        return ""
    t = _LINE_SUFFIX_RE.sub("", t)
    if "#" in t:
        t = t.split("#", 1)[1]
    elif t.endswith(".c") or t.endswith(".h"):
        return ""  # bare file with no function
    elif "/" in t and t.count(".c") == 0:
        t = t.rsplit("/", 1)[1]
    return ALLOCATOR_ALIASES.get(t, t)


def normalize_frame_sequence(frames) -> list[str]:
    """Normalize a frame list: per-frame normalize, drop unstable
    dump-stack frames and empties, collapse consecutive duplicates.
    The raw sequence must be preserved separately by the caller as
    evidence; this returns the canonical sequence only.
    """
    out: list[str] = []
    for f in frames or []:
        nf = normalize_frame(str(f))
        if not nf or nf in DUMP_STACK_FRAMES:
            continue
        if not out or out[-1] != nf:
            out.append(nf)
    return out


# --------------------------------------------------------------------
# Comparison-class vocabulary.
# --------------------------------------------------------------------

# Runtime-path classes as emitted by runtime_static_compare.py, mapped
# to the corpus aggregation field names (Part 7). The comparator emits
# metric keys runtime_paths_<x>; the corpus layer aggregates under
# these canonical names.
COMPARISON_METRIC_TO_AGG = {
    "runtime_paths_direct_static_explained": "direct_static_explained",
    "runtime_paths_indirect_explained": "indirect_candidate_explained",
    "runtime_paths_target_observed_no_caller_context":
        "target_observed_no_caller_context",
    "runtime_paths_unexplained": "runtime_only_unexplained",
    "runtime_paths_unresolved_normalization_mismatch":
        "unresolved_normalization_mismatch",
    "runtime_paths_boundary_or_parser_artifact": "parser_artifact",
    "runtime_paths_total": "runtime_paths_total",
}

INDIRECT_METRIC_TO_AGG = {
    "indirect_candidates_total": "indirect_candidates_total",
    "indirect_candidates_observed": "indirect_candidates_observed",
    "indirect_candidates_not_observed": "indirect_candidates_not_observed",
}

# Corpus-level comparison outcomes that the comparator itself never
# emits (they are decided by the join layer).
CORPUS_COMPARISON_STATES = (
    "COMPARED", "COMPARISON_BLOCKED", "CONFIG_MISMATCH", "PROXY")

AGG_RUNTIME_CLASSES = (
    "direct_static_explained",
    "indirect_candidate_explained",
    "target_observed_no_caller_context",
    "runtime_only_unexplained",
    "unresolved_normalization_mismatch",
    "parser_artifact",
)

# Classes that, if a scenario's observations fall into them, block a
# SUPPORTED verdict unless an accepted exception covers the scenario.
EXPLAINED_CLASSES = frozenset({
    "direct_static_explained",
    "indirect_candidate_explained",
    "target_observed_no_caller_context",
})
UNRESOLVED_CLASSES = frozenset({
    "runtime_only_unexplained",
    "unresolved_normalization_mismatch",
    "parser_artifact",
})


# --------------------------------------------------------------------
# Scenario vocabulary (Part 9). Declared enums so the classifier and
# the policy engine agree on spellings.
# --------------------------------------------------------------------

TRIGGER_ACTORS = (
    "xen_boot", "dom0_or_hwdom", "toolstack", "ordinary_guest",
    "hardware", "firmware", "unknown")
PHASES = (
    "boot", "domain_creation", "domain_destroy", "hypercall",
    "device_assignment", "interrupt_or_timer", "firmware_call",
    "runtime_background", "unknown")
PRIVILEGE = (
    "privileged", "unprivileged_guest", "hardware_firmware",
    "internal", "unknown")
SIZE_BOUND_KINDS = (
    "constant_size", "config_bounded", "domain_limit_bounded",
    "hardware_table_bounded", "observed_only", "unknown")
FREQUENCY_CLASSES = (
    "one_time_boot", "per_domain_creation", "per_device", "per_vcpu",
    "per_event_channel", "runtime_repeated", "unknown")
LIFETIME_CLASSES = (
    "freed_before_return", "retained_after_return",
    "transferred_to_domain_lifetime", "transferred_to_global_state",
    "boot_lifetime", "unknown")

SAFETY_ACCEPTED = (
    "accepted_boot_time",
    "accepted_dom0_or_hwdom_controlled",
    "accepted_domain_creation_bounded",
    "accepted_hardware_firmware_controlled",
    "accepted_runtime_bounded",
    "accepted_guest_triggered_bounded_transient",
    "accepted_guest_triggered_bounded_domain_lifetime",
)
SAFETY_REVIEW = ("needs_manual_review",)
SAFETY_REJECTED = (
    "rejected_unbounded_guest_controlled",
    "rejected_guest_triggered_retained_xenheap",
    "rejected_unexplained_static_gap",
    "rejected_normalization_gap",
    "rejected_config_mismatch",
    "rejected_parser_artifact",
)
SAFETY_ALL = SAFETY_ACCEPTED + SAFETY_REVIEW + SAFETY_REJECTED

REVIEW_STATUS = ("accepted", "rejected", "needs_manual_review")

CLASSIFICATION_SOURCE = (
    "machine_classified", "annotation_classified", "needs_manual_review")

# Full ordered scenario record field list (Parts 8-9), the single
# column order used by allocation-scenarios.csv and the JSON records.
SCENARIO_FIELDS = (
    "scenario_id", "allocation_target", "canonical_stack",
    "raw_stack_variants", "raw_observation_count", "config_groups_seen",
    "tests_seen", "domains_seen", "max_size_observed",
    "allocation_size_kind", "allocation_size_formula",
    "allocation_size_bound_source", "frequency_class", "phase",
    "trigger_actor", "privilege", "controllability", "initiating_call",
    "returns_before_free", "lifetime_class", "cleanup_function",
    "cleanup_phase", "retention_bound", "accumulation_possible",
    "static_explanation", "comparison_classes_seen",
    "safety_classification", "classification_source", "review_status",
    "justification", "evidence_paths",
)


# --------------------------------------------------------------------
# Small IO helpers.
# --------------------------------------------------------------------

def read_json(path: Path):
    return json.loads(Path(path).read_text())


def try_read_json(path: Path):
    try:
        return read_json(path)
    except (OSError, ValueError):
        return None


def write_json(path: Path, obj) -> None:
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(obj, indent=2, sort_keys=True) + "\n")
