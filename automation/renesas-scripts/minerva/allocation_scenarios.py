#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Allocation scenario grouping and semantic classification.

Consumes normalized runtime allocation paths (with their comparison
classification) and collapses them into canonical *scenarios* -- the
unit of assurance. Each scenario is then classified by actor, phase,
size/frequency bound, lifetime, privilege and a final safety
classification.

Static explanation answers "where"; scenario classification answers
"is it bounded/acceptable?". A scenario is never accepted solely
because it is statically explained.
"""
from __future__ import annotations

import re

try:
    from . import corpus_schema as cs
except ImportError:  # run as a script
    import corpus_schema as cs


# --------------------------------------------------------------------
# Observation -> scenario grouping.
# --------------------------------------------------------------------

# Frames that anchor a scenario's identity: the allocation target plus
# the nearest few non-target "important" frames. We do not group by
# the full raw stack (too granular -> duplicate scenarios) nor by the
# target alone (too coarse -> distinct scenarios merged).
_IMPORTANT_PREFIX = 3


def _subsystem_of(frames: list[str]) -> str:
    """Best-effort subsystem from a frame name. Heuristic, low weight."""
    joined = " ".join(frames).lower()
    for key, sub in (("evtchn", "event_channel"), ("event_channel",
                     "event_channel"), ("domain", "domain"),
                     ("grant", "grant_table"), ("vcpu", "vcpu"),
                     ("sched", "scheduler"), ("iommu", "iommu"),
                     ("smmu", "iommu"), ("gic", "interrupt"),
                     ("vgic", "interrupt"), ("p2m", "p2m"),
                     ("xsm", "xsm"), ("dom0", "dom0")):
        if key in joined:
            return sub
    return "unknown"


def _grouping_key(obs: dict) -> tuple:
    """Scenario grouping key.

    Includes allocation target, the canonical important frames, the
    subsystem, and (when known) phase / trigger actor / size formula /
    lifetime. Built from already-normalized frames.
    """
    frames = obs["canonical_stack"]
    target = obs["allocation_target"]
    important = tuple(f for f in frames if f != target)[:_IMPORTANT_PREFIX]
    return (
        target,
        important,
        _subsystem_of(frames),
        obs.get("phase_hint", ""),
        obs.get("actor_hint", ""),
        obs.get("size_formula_hint", ""),
        obs.get("lifetime_hint", ""),
    )


def group_scenarios(observations: list[dict]) -> list[dict]:
    """Collapse normalized observations into canonical scenarios.

    Each observation is a dict with at least:
      allocation_target, canonical_stack (list), raw_stack (list),
      domain, comparison_class, config_group, test_name,
      max_size (optional), evidence_path (optional)
    """
    groups: dict[tuple, dict] = {}
    order: list[tuple] = []
    for obs in observations:
        key = _grouping_key(obs)
        g = groups.get(key)
        if g is None:
            g = {
                "allocation_target": obs["allocation_target"],
                "canonical_stack": list(obs["canonical_stack"]),
                "raw_stack_variants": [],
                "raw_observation_count": 0,
                "config_groups_seen": set(),
                "tests_seen": set(),
                "domains_seen": set(),
                "max_size_observed": 0,
                "comparison_classes_seen": set(),
                "evidence_paths": [],
                "_hints": {
                    "phase": obs.get("phase_hint", ""),
                    "actor": obs.get("actor_hint", ""),
                    "size_formula": obs.get("size_formula_hint", ""),
                    "lifetime": obs.get("lifetime_hint", ""),
                },
            }
            groups[key] = g
            order.append(key)
        raw = tuple(obs.get("raw_stack") or obs["canonical_stack"])
        if list(raw) not in g["raw_stack_variants"]:
            g["raw_stack_variants"].append(list(raw))
        g["raw_observation_count"] += 1
        if obs.get("config_group"):
            g["config_groups_seen"].add(obs["config_group"])
        if obs.get("test_name"):
            g["tests_seen"].add(obs["test_name"])
        if obs.get("domain"):
            g["domains_seen"].add(obs["domain"])
        if obs.get("comparison_class"):
            g["comparison_classes_seen"].add(obs["comparison_class"])
        try:
            g["max_size_observed"] = max(
                g["max_size_observed"], int(obs.get("max_size") or 0))
        except (TypeError, ValueError):
            pass
        if obs.get("evidence_path"):
            g["evidence_paths"].append(obs["evidence_path"])

    scenarios = []
    for i, key in enumerate(order, start=1):
        g = groups[key]
        sid = f"ALLOC-SCENARIO-{i:04d}"
        scenarios.append(_finalize_scenario(sid, g))
    return scenarios


def _finalize_scenario(sid: str, g: dict) -> dict:
    sc = {f: "" for f in cs.SCENARIO_FIELDS}
    sc["scenario_id"] = sid
    sc["allocation_target"] = g["allocation_target"]
    sc["canonical_stack"] = " -> ".join(g["canonical_stack"])
    sc["raw_stack_variants"] = len(g["raw_stack_variants"])
    sc["raw_observation_count"] = g["raw_observation_count"]
    sc["config_groups_seen"] = sorted(g["config_groups_seen"])
    sc["tests_seen"] = sorted(g["tests_seen"])
    sc["domains_seen"] = sorted(g["domains_seen"])
    sc["max_size_observed"] = g["max_size_observed"]
    sc["comparison_classes_seen"] = sorted(g["comparison_classes_seen"])
    sc["_raw_variants"] = g["raw_stack_variants"]  # internal, for md
    sc["_canonical_frames"] = list(g["canonical_stack"])
    sc["evidence_paths"] = sorted(set(g["evidence_paths"]))
    sc["allocation_size_kind"] = "observed_only" if \
        g["max_size_observed"] else "unknown"
    sc["allocation_size_bound_source"] = ""
    sc["frequency_class"] = "unknown"
    sc["returns_before_free"] = "unknown"
    sc["lifetime_class"] = g["_hints"].get("lifetime") or "unknown"
    sc["accumulation_possible"] = "unknown"
    sc["static_explanation"] = ", ".join(
        sorted(g["comparison_classes_seen"]))
    return sc


# --------------------------------------------------------------------
# Semantic classification (Part 9).
# --------------------------------------------------------------------

# Frame-name hints, grounded in the real direct-static reaching sets
# of the allocation targets (arch_domain_create, alloc_vcpu_struct,
# arm_smmu_assign_dev, cpu_schedule_*, arm_smmu_dt_init, ...). These
# are heuristics: a match assigns a phase/actor at a stated
# confidence, never a guaranteed truth.
_BOOT_HINTS = ("start_xen", "__start_xen", "init_done", "setup_",
               "boot_", "__init", "dt_init", "acpi_", "discard_initial",
               "scrub_", "end_boot", "init_done", "early_")
_DOMAIN_CREATE_HINTS = ("domain_create", "arch_domain_create",
                        "construct_dom", "create_domain",
                        "domain_build", "alloc_domain",
                        "alloc_domain_struct", "alloc_vcpu_struct",
                        "arch_vcpu_create", "vcpu_create",
                        "sched_init_domain", "evtchn_init")
_DOMAIN_DESTROY_HINTS = ("domain_destroy", "domain_kill",
                         "complete_domain_destroy", "domain_teardown",
                         "arch_domain_destroy", "free_domheap")
_DEVICE_ASSIGN_HINTS = ("smmu", "iommu", "assign_dev", "deassign_dev",
                        "add_device", "device_cfg", "pci_", "msi_")
_HW_FW_HINTS = ("acpi_", "dt_", "firmware", "psci", "smccc",
                "platform_", "efi_")
_INTERRUPT_HINTS = ("gic", "vgic", "irq", "interrupt", "timer",
                    "do_IRQ", "vtimer")
_SCHED_HINTS = ("schedule", "sched_", "cpu_schedule", "credit",
                "rt_alloc", "csched")
_HYPERCALL_HINTS = ("do_", "hypercall", "compat_")
_EVTCHN_HINTS = ("evtchn", "event_channel")
_XENHEAP_TARGETS = ("alloc_xenheap_pages", "_xmalloc")
# Privileged event-channel / domain-setup frames: an evtchn frame that
# is part of domain construction (not a guest-issued hypercall) is
# privileged, bounded per-domain.
_PRIV_EVTCHN_HINTS = ("alloc_unbound_xen_event_channel",
                      "alloc_static_evtchn", "alloc_domain_evtchn",
                      "evtchn_init", "evtchn_fifo_init_control")


def _frames_match(frames, needles) -> bool:
    return any(any(n in f for n in needles) for f in frames)


def classify_scenario(sc: dict, confidence_mode: str = "medium") -> dict:
    """Classify a scenario by the cause/effect safety model.

    Order of reasoning:
      1. comparison-class rejects (static gap / mismatch / parser
         artifact) -- an analysis failure, rejected regardless;
      2. no-caller-frame / non-explained -> manual review (cause and
         effect cannot be assessed);
      3. gather evidence: actor / phase / privilege / frequency / size
         and lifetime from the frames;
      4. assess two independent axes:
         - cause: does the trigger have a quantifiable upper boundary?
           (BOUNDED / UNBOUNDED / UNKNOWN)
         - effect: is the allocation immediately, sequentially freed
           with no allocation permitted in between, so it cannot
           accumulate? (BRACKETED / NOT_BRACKETED / UNKNOWN)
      5. decide:
         - SAFE (accepted) if cause is BOUNDED *or* effect is
           BRACKETED;
         - UNSAFE (rejected) if cause is UNBOUNDED *and* effect is
           NOT_BRACKETED;
         - needs_manual_review otherwise (neither safety nor danger
           positively established).

    Acceptance requires positive proof on at least one axis -- never
    the mere absence of disproof. Static explanation answers "where";
    this answers "is it bounded by cause or by effect?".

    confidence_mode is accepted for API stability but no longer
    selects an aggressiveness level: safety is determined by the
    cause/effect evidence, not by a global dial.
    """
    classes = set(sc.get("comparison_classes_seen") or [])
    frames = sc.get("_canonical_frames") or []
    target = sc.get("allocation_target", "")

    # 1. Comparison-class-driven rejects.
    if "runtime_only_unexplained" in classes:
        return _set(sc, "rejected_unexplained_static_gap",
                    "machine_classified", "unknown", "unknown")
    if "unresolved_normalization_mismatch" in classes:
        return _set(sc, "rejected_normalization_gap",
                    "machine_classified", "unknown", "unknown")
    if "parser_artifact" in classes:
        return _set(sc, "rejected_parser_artifact",
                    "machine_classified", "unknown", "unknown")

    # 2. Evidence-grounded actor / phase / privilege / bound. Order is
    #    deliberate: device-assignment and hardware/firmware paths are
    #    recognized before the generic evtchn/hypercall fallbacks, and
    #    privileged event-channel setup before guest event-channel use,
    #    so a privileged domain-setup path is not mistaken for a guest
    #    hypercall.
    phase = actor = priv = "unknown"
    conf = "low"
    size_kind = "observed_only" if sc.get("max_size_observed") else "unknown"
    freq = "unknown"
    if _frames_match(frames, _BOOT_HINTS):
        phase, actor, priv, conf = "boot", "xen_boot", "internal", "high"
        freq, size_kind = "one_time_boot", "config_bounded"
    elif _frames_match(frames, _DEVICE_ASSIGN_HINTS) or \
            _frames_match(frames, _HW_FW_HINTS):
        phase, actor, priv, conf = ("device_assignment",
                                    "hardware", "hardware_firmware",
                                    "medium")
        freq, size_kind = "per_device", "hardware_table_bounded"
    elif _frames_match(frames, _INTERRUPT_HINTS):
        phase, actor, priv, conf = ("interrupt_or_timer", "hardware",
                                    "hardware_firmware", "medium")
        freq = "runtime_repeated"
    elif _frames_match(frames, _DOMAIN_CREATE_HINTS) or \
            _frames_match(frames, _PRIV_EVTCHN_HINTS):
        phase, actor, priv, conf = ("domain_creation", "dom0_or_hwdom",
                                    "privileged", "medium")
        freq, size_kind = "per_domain_creation", "domain_limit_bounded"
    elif _frames_match(frames, _DOMAIN_DESTROY_HINTS):
        phase, actor, priv, conf = ("domain_destroy", "dom0_or_hwdom",
                                    "privileged", "medium")
        freq = "per_domain_creation"
    elif _frames_match(frames, _SCHED_HINTS):
        phase, actor, priv, conf = ("runtime_background", "dom0_or_hwdom",
                                    "internal", "low")
        freq = "per_vcpu"
    elif _frames_match(frames, _EVTCHN_HINTS):
        # Generic event-channel frame not matched as privileged setup:
        # treat as a guest-reachable hypercall path.
        phase, actor, priv, conf = ("hypercall", "ordinary_guest",
                                    "unprivileged_guest", "low")
    elif _frames_match(frames, _HYPERCALL_HINTS):
        phase, actor, priv, conf = "hypercall", "unknown", "unknown", "low"
    sc["phase"] = phase
    sc["trigger_actor"] = actor
    sc["privilege"] = priv
    sc["controllability"] = priv
    if sc.get("frequency_class", "unknown") in ("", "unknown"):
        sc["frequency_class"] = freq
    if sc.get("allocation_size_kind", "unknown") in ("", "unknown"):
        sc["allocation_size_kind"] = size_kind
    sc["_confidence"] = conf
    xenheap = target in _XENHEAP_TARGETS
    lifetime = sc.get("lifetime_class", "unknown")

    # target_observed_no_caller_context: there is no caller frame, so
    # neither cause nor effect can be assessed. Always review.
    if "target_observed_no_caller_context" in classes:
        return _set(sc, "needs_manual_review", "machine_classified",
                    phase, actor)
    if not (classes & cs.EXPLAINED_CLASSES):
        return _set(sc, "needs_manual_review", "machine_classified",
                    phase, actor)

    # --- Cause/effect safety model ---------------------------------
    #
    # Cause-safety:  the cause (how often / how much this allocation is
    #   triggered) has a quantifiable upper boundary -> BOUNDED.
    #   A guest-controllable cause with no quota -> UNBOUNDED.
    #   Otherwise -> UNKNOWN.
    # Effect-safety: the allocation is immediately, temporally and
    #   sequentially followed by its deallocation, with no further
    #   allocation permitted in between -> BRACKETED (cannot
    #   accumulate). Retained / transferred / unknown -> NOT_BRACKETED
    #   or UNKNOWN.
    #
    # Rule: an allocation is SAFE if it is cause-bounded OR
    # effect-bracketed. It is UNSAFE only if it is neither -- and
    # affirmatively unsafe (rejected) when the cause is positively
    # UNBOUNDED and the effect is positively NOT_BRACKETED. When
    # neither safety can be positively established but no positive
    # danger is shown either, the scenario is needs_manual_review:
    # acceptance requires positive proof of one axis, not the mere
    # absence of disproof.
    cause = _assess_cause(sc, actor, phase, conf, xenheap)
    effect = _assess_effect(sc, lifetime)
    sc["_cause_bound"] = cause
    sc["_effect_bracket"] = effect

    if cause == "BOUNDED" or effect == "BRACKETED":
        return _set(sc, _safe_class(cause, effect, actor, phase),
                    "machine_classified", phase, actor)
    if cause == "UNBOUNDED" and effect == "NOT_BRACKETED":
        # Positively unsafe by both axes. A guest-controllable Xenheap
        # retention is the canonical instance.
        if actor == "ordinary_guest" and xenheap:
            return _set(sc, "rejected_guest_triggered_retained_xenheap",
                        "machine_classified", phase, actor)
        return _set(sc, "rejected_unbounded_guest_controlled",
                    "machine_classified", phase, actor)
    # Neither axis positively safe, but not positively unsafe either.
    return _set(sc, "needs_manual_review", "machine_classified",
                phase, actor)


def _assess_cause(sc, actor, phase, conf, xenheap) -> str:
    """Quantifiable upper boundary on the allocation's cause?

    BOUNDED   - boot (one-time), per-domain (domain-limit bounded),
                per-device / hardware-table bounded, or a frequency
                class with an inherent upper bound, at sufficient
                confidence.
    UNBOUNDED - guest-controllable trigger with no quota evidence.
    UNKNOWN   - cannot tell from the available evidence.
    """
    freq = sc.get("frequency_class", "unknown")
    size_kind = sc.get("allocation_size_kind", "unknown")
    if actor == "ordinary_guest":
        # Guest can drive the cause; bounded only with explicit quota
        # evidence (supplied via annotation, not inferable here).
        return "UNBOUNDED"
    bounded_freqs = ("one_time_boot", "per_domain_creation", "per_device",
                     "per_vcpu", "per_event_channel")
    bounded_sizes = ("constant_size", "config_bounded",
                     "domain_limit_bounded", "hardware_table_bounded")
    if conf in ("medium", "high") and (freq in bounded_freqs
                                       or size_kind in bounded_sizes):
        return "BOUNDED"
    if actor == "xen_boot" and phase == "boot":
        return "BOUNDED"
    return "UNKNOWN"


def _assess_effect(sc, lifetime) -> str:
    """Is the allocation immediately bracketed by its deallocation?

    BRACKETED     - freed before the initiating operation returns, with
                    no intervening allocation permitted (cannot
                    accumulate).
    NOT_BRACKETED - retained after return, or transferred to a longer
                    lifetime / global state.
    UNKNOWN       - lifetime not established.

    The strict "no allocation in between" requirement is represented by
    the freed_before_return lifetime class together with a
    returns_before_free marker; absent positive evidence the effect is
    UNKNOWN, never assumed BRACKETED.
    """
    rbf = sc.get("returns_before_free", "unknown")
    if lifetime == "freed_before_return" or rbf == "true":
        # Only bracketed if nothing may allocate between alloc and free
        # in the same sequence. We treat freed_before_return as the
        # bracketed signal; a weaker "freed somewhere later" is not.
        if sc.get("accumulation_possible", "unknown") in ("false", "no"):
            return "BRACKETED"
        if lifetime == "freed_before_return":
            return "BRACKETED"
    if lifetime in ("retained_after_return", "transferred_to_global_state"):
        return "NOT_BRACKETED"
    if lifetime in ("transferred_to_domain_lifetime", "boot_lifetime"):
        # Transferred to a bounded longer lifetime: not promptly freed,
        # so not effect-bracketed (its safety rests on the cause bound).
        return "NOT_BRACKETED"
    return "UNKNOWN"


def _safe_class(cause, effect, actor, phase):
    """Pick the accepted_* class for a scenario proven safe by cause
    OR effect, preferring the most specific applicable class.
    """
    if effect == "BRACKETED" and actor == "ordinary_guest":
        return "accepted_guest_triggered_bounded_transient"
    if effect == "BRACKETED":
        return "accepted_runtime_bounded"
    # cause == BOUNDED below.
    if actor == "xen_boot" and phase == "boot":
        return "accepted_boot_time"
    if actor == "hardware":
        return "accepted_hardware_firmware_controlled"
    if phase in ("domain_creation", "domain_destroy"):
        return "accepted_domain_creation_bounded"
    if actor in ("dom0_or_hwdom", "toolstack"):
        return "accepted_dom0_or_hwdom_controlled"
    return "accepted_runtime_bounded"


def _set(sc, safety, source, phase, actor):
    sc["safety_classification"] = safety
    sc["classification_source"] = source
    if phase and not sc.get("phase"):
        sc["phase"] = phase
    if actor and not sc.get("trigger_actor"):
        sc["trigger_actor"] = actor
    sc["review_status"] = (
        "accepted" if safety in cs.SAFETY_ACCEPTED
        else "rejected" if safety in cs.SAFETY_REJECTED
        else "needs_manual_review")
    return sc


# --------------------------------------------------------------------
# Annotation application (Part 10).
# --------------------------------------------------------------------

def apply_annotations(scenarios: list[dict], annotations: dict,
                      warnings: list[str]) -> None:
    """Apply curated annotations. An annotation may refine fields and
    may accept a scenario only if it supplies a justification.
    Malformed annotations are reported and ignored.
    """
    rules = (annotations or {}).get("scenarios") or []
    for rule in rules:
        rid = rule.get("id", "<no-id>")
        match = rule.get("match") or {}
        want_target = match.get("allocation_target")
        frames_any = match.get("frames_any") or []
        classification = rule.get("classification")
        justification = (rule.get("justification") or "").strip()
        for sc in scenarios:
            if want_target and sc["allocation_target"] != want_target:
                continue
            if frames_any and not _frames_match(
                    sc.get("_canonical_frames") or [], frames_any):
                continue
            # Refinements.
            for fld_src, fld in (("actor", "trigger_actor"),
                                 ("phase", "phase"),
                                 ("privilege", "privilege"),
                                 ("controllability", "controllability")):
                if rule.get(fld_src):
                    sc[fld] = rule[fld_src]
            lt = rule.get("lifetime") or {}
            if lt.get("class"):
                sc["lifetime_class"] = lt["class"]
            if lt.get("cleanup"):
                sc["cleanup_function"] = lt["cleanup"]
            sb = rule.get("size_bound") or {}
            if sb.get("kind"):
                sc["allocation_size_kind"] = sb["kind"]
                sc["allocation_size_bound_source"] = sb.get(
                    "explanation", "")
            fr = rule.get("frequency") or {}
            if fr.get("class"):
                sc["frequency_class"] = fr["class"]
            # Acceptance requires justification.
            if classification:
                if classification in cs.SAFETY_ACCEPTED and \
                        not justification:
                    warnings.append(
                        f"annotation {rid}: acceptance without "
                        f"justification ignored for {sc['scenario_id']}")
                    continue
                sc["safety_classification"] = classification
                sc["classification_source"] = "annotation_classified"
                sc["justification"] = justification
                sc["review_status"] = (
                    "accepted" if classification in cs.SAFETY_ACCEPTED
                    else "rejected" if classification in cs.SAFETY_REJECTED
                    else "needs_manual_review")
                sc["_annotation_id"] = rid


def classify_all(scenarios: list[dict], annotations: dict,
                 warnings: list[str],
                 confidence_mode: str = "medium") -> list[dict]:
    for sc in scenarios:
        classify_scenario(sc, confidence_mode)
    apply_annotations(scenarios, annotations, warnings)
    return scenarios
