#!/usr/bin/env python3
"""
Indirect-call scoping collector.

Given a Xen source tree and a config file, emit the scoping inputs for
the indirect-call coverage analysis:

  ops-inventory.csv
  ops-resolution.csv
  indirect-call-sites.csv
  config-scope.md (config summary section)

The script is config-driven. The chosen config can be:
  - a generated .config file (lines like "CONFIG_FOO=y" or
    "# CONFIG_FOO is not set");
  - a defconfig fragment (same syntax, subset);
  - the path to a CI-job env file that sets EXTRA_XEN_CONFIG.

The script does not hard-code any config or arch. To analyze a
different configuration, run the script with a different --config.

Validation: run with --config xen/arch/arm/configs/arm64_defconfig
against this repository. The output CSVs and config summary are
regenerated from the source tree and the supplied config; no manual
edits are required.

Limitations (documented in the report, not papered over by the
script):
  - Implementation resolution is best-effort. Some ops fields resolve
    to multiple implementations at runtime (e.g. PCI capability
    handlers); the script marks these "small_set" or
    "unresolved_runtime".
  - Indirect call sites are discovered by regex against the source.
    Function-pointer arrays embedded in non-`_ops` structs are not
    always caught. The output csv flags these as "unresolved".
  - Reachability to allocation entry points is not computed by this
    script. Coupling to callpath.py output is left to the report
    stage.
"""

import argparse
import csv
import json
import re
import sys
from dataclasses import dataclass, asdict
from pathlib import Path
from collections import defaultdict

# Shared architecture-scope classifier (single source of truth, also
# used by the reachability workbench). Imported by path so the
# collector runs from outside an installed package.
import importlib.util as _ilu
_arch_scope_spec = _ilu.spec_from_file_location(
    "arch_scope", str(Path(__file__).resolve().parent / "arch_scope.py"))
arch_scope = _ilu.module_from_spec(_arch_scope_spec)
_arch_scope_spec.loader.exec_module(arch_scope)


CONFIG_LINE_SET = re.compile(r"^CONFIG_(?P<name>\w+)=(?P<val>.+)$")
CONFIG_LINE_UNSET = re.compile(r"^# CONFIG_(?P<name>\w+) is not set$")
CONFIG_TOKEN_RE = re.compile(r"^CONFIG_(?P<name>\w+)=(?P<val>.+)$")
EXTRA_LINE_RE = re.compile(
    r"^\s*EXTRA_XEN_CONFIG\s*\+?=\s*(?P<rhs>.*?)\s*$"
)


def parse_config_token(token: str) -> tuple[str, str] | None:
    """Parse a single CONFIG_FOO=value token.

    Returns (name, value) or None if the token is not in that form.
    Tokens starting with '$' (shell variable references) are
    rejected; the caller is expected to filter those before
    calling this helper.
    """
    if not token or token.startswith("$"):
        return None
    m = CONFIG_TOKEN_RE.match(token)
    if not m:
        return None
    return (m.group("name"), m.group("val"))


def parse_extra_xen_config_value(value: str) -> dict[str, str]:
    """Parse the RHS of an EXTRA_XEN_CONFIG[+]= shell assignment.

    Strips matched surrounding single or double quotes, then
    whitespace-splits the remainder. Each token is parsed as a
    CONFIG_FOO=value pair; tokens starting with '$' are ignored
    (shell variable references; no shell evaluation is performed).

    Returns a mapping CONFIG name -> value for the literal tokens
    that parsed successfully. Tokens that do not parse are
    silently dropped.
    """
    value = value.strip()
    # Strip exactly one matching pair of surrounding quotes.
    if len(value) >= 2 and (
        (value[0] == '"' and value[-1] == '"')
        or (value[0] == "'" and value[-1] == "'")
    ):
        value = value[1:-1]
    out: dict[str, str] = {}
    for token in value.split():
        kv = parse_config_token(token)
        if kv:
            out[kv[0]] = kv[1]
    return out


def read_config(path: Path) -> dict[str, str]:
    """Parse a Kconfig-style file or a CI env file into a dict.

    Three line shapes are recognised:

      CONFIG_FOO=value          -> {"FOO": "value"}
      # CONFIG_FOO is not set   -> {"FOO": "n"}
      EXTRA_XEN_CONFIG="..."    -> literal CONFIG_FOO=value tokens
                                   inside the RHS are parsed.

    Unrelated env-file lines are ignored. No shell evaluation is
    performed; '$VAR' tokens inside EXTRA_XEN_CONFIG are dropped.
    """
    out: dict[str, str] = {}
    for line in path.read_text(errors="replace").splitlines():
        line = line.strip()
        if not line:
            continue
        m = CONFIG_LINE_SET.match(line)
        if m:
            out[m.group("name")] = m.group("val")
            continue
        m = CONFIG_LINE_UNSET.match(line)
        if m:
            out[m.group("name")] = "n"
            continue
        em = EXTRA_LINE_RE.match(line)
        if em:
            out.update(parse_extra_xen_config_value(em.group("rhs")))
            continue
    return out


@dataclass
class OpsTable:
    table_type: str
    table_instance: str
    source_file: str
    line: int
    fields: dict[str, str]  # field_name -> implementation_function
    subsystem: str
    config_guards: list[str]
    arch_scope: str = arch_scope.IN_SCOPE


OPS_DEF_PATTERN = re.compile(
    r"^(?:static\s+)?(?:const\s+)?struct\s+(?P<type>\w+)\s+"
    r"(?P<name>\w+)(?:\s*__\w+(?:\([^)]*\))?)*\s*=\s*\{",
    re.MULTILINE,
)
FIELD_ASSIGN_PATTERN = re.compile(
    r"(?<![A-Za-z0-9_])\.(?P<field>\w+)\s*=\s*"
    r"(?P<impl>[^,\}\n]+?)\s*[,\}]",
    re.MULTILINE,
)


def discover_ops_tables(xen_root: Path, paths: list[str],
                        target_arch: str = "arm64") -> list[OpsTable]:
    """Scan source directories for ops-table definitions.

    A definition has the shape:
       (static)? (const)? struct foo_ops name [annotations] = {
            .field = impl,
            ...
       };

    Annotations like __initconstrel are tolerated. Field assignments
    are extracted on a best-effort basis. Each table is tagged with
    its architecture scope against target_arch; tables under a
    non-target arch tree are retained (never dropped) and marked
    out_of_scope_arch.
    """
    tables: list[OpsTable] = []
    for rel in paths:
        root = xen_root / rel
        if not root.exists():
            continue
        for p in root.rglob("*.c"):
            text = p.read_text(errors="replace")
            lines = text.splitlines()
            rel_p = str(p.relative_to(xen_root)).replace("\\", "/")
            ascope = arch_scope.classify_arch_scope(
                rel_p, target_arch)[0]
            for m in OPS_DEF_PATTERN.finditer(text):
                ty = m.group("type")
                name = m.group("name")
                if not ty.endswith("_ops") and not ty.endswith("_operations"):
                    continue
                start = m.start()
                line_num = text.count("\n", 0, start) + 1
                # Walk forward to find closing }; collect field assignments
                depth = 0
                rest = text[m.end() - 1 :]
                end = 0
                for i, ch in enumerate(rest):
                    if ch == "{":
                        depth += 1
                    elif ch == "}":
                        depth -= 1
                        if depth == 0:
                            end = i + 1
                            break
                body = rest[:end]
                fields: dict[str, str] = {}
                for fm in FIELD_ASSIGN_PATTERN.finditer(body):
                    fields[fm.group("field")] = fm.group("impl").strip()
                subsystem = classify_subsystem(p.relative_to(xen_root))
                guards = config_guards_near(lines, line_num)
                tables.append(
                    OpsTable(
                        table_type=f"struct {ty}",
                        table_instance=name,
                        source_file=rel_p,
                        line=line_num,
                        fields=fields,
                        subsystem=subsystem,
                        config_guards=guards,
                        arch_scope=ascope,
                    )
                )
    return tables


def classify_subsystem(p: Path) -> str:
    parts = p.parts
    if "passthrough" in parts:
        if "arm" in parts:
            return "iommu/arm"
        if "amd" in parts:
            return "iommu/x86/amd"
        if "vtd" in parts:
            return "iommu/x86/intel"
        return "iommu"
    if "hvm" in parts:
        return "hvm"
    if "sched" in parts:
        return "sched"
    if "vgic" in parts:
        return "interrupts/arm/vgic"
    if "vpci" in parts:
        return "pci/vpci"
    if "event_" in str(p) or "event_channel" in str(p):
        return "event_channel"
    if "tee" in parts or "ffa" in str(p) or "optee" in str(p):
        return "tee"
    if "livepatch" in str(p):
        return "livepatch"
    if "rcu" in str(p):
        return "rcu"
    if "timer" in str(p):
        return "timers"
    if "tasklet" in str(p):
        return "tasklets"
    if "xsm" in str(p):
        return "xsm"
    if "coverage" in parts:
        return "coverage"
    if "microcode" in parts:
        return "microcode"
    if "vpmu" in str(p):
        return "vpmu"
    if "arch/arm" in str(p):
        return "arch/arm"
    if "arch/x86" in str(p):
        return "arch/x86"
    return "common"


GUARD_PATTERN = re.compile(r"^\s*#if(?:def)?\s+(?:!)?\s*(?:defined\s*\(\s*)?CONFIG_(\w+)")


def config_guards_near(lines: list[str], line_num: int) -> list[str]:
    """Return the list of CONFIG_* guards in effect at the given line.

    Walks backwards from the line, tracking #if/#endif depth at
    coarse granularity. Best-effort: nested guards may not be fully
    captured.
    """
    guards: list[str] = []
    depth = 0
    for i in range(line_num - 1, -1, -1):
        s = lines[i].strip()
        if s.startswith("#endif"):
            depth += 1
        elif s.startswith("#if"):
            if depth > 0:
                depth -= 1
            else:
                m = GUARD_PATTERN.match(s)
                if m:
                    guards.append(m.group(1))
    return guards


SUBSYSTEM_SCOPE_HINTS: dict[str, list[str]] = {
    "iommu/arm": ["ARM_SMMU_V3", "ARM_SMMU", "IPMMU_VMSA"],
    "iommu/x86/amd": ["AMD_IOMMU"],
    "iommu/x86/intel": ["INTEL_IOMMU"],
    "hvm": ["HVM"],
    "sched": ["SCHED_CREDIT2", "SCHED_CREDIT", "SCHED_RTDS",
              "SCHED_ARINC653", "SCHED_NULL"],
    "interrupts/arm/vgic": ["HAS_ITS", "HAS_GICV3", "HAS_GICV2"],
    "pci/vpci": ["HAS_VPCI", "PCI_PASSTHROUGH"],
    "tee": ["TEE", "FFA", "OPTEE"],
    "livepatch": ["LIVEPATCH"],
    "xsm": ["XSM", "XSM_FLASK", "XSM_SILO", "XSM_FLASK_POLICY"],
    "vpmu": ["VPMU"],
    "microcode": ["MICROCODE"],
}


def resolve_scope(table: OpsTable, config: dict[str, str]) -> tuple[str, str]:
    """Decide an in/out-of-scope verdict for one ops table under the config.

    Returns (verdict, basis).

    Architecture scope takes precedence: a table under a non-target
    arch tree is out_of_scope_arch regardless of its config guards,
    because it is not built for the target architecture at all.
    """
    if table.arch_scope == arch_scope.OUT_OF_SCOPE_ARCH:
        return ("out_of_scope_arch",
                "source under a non-target architecture tree")
    # Direct file-local guards take precedence.
    for g in table.config_guards:
        v = config.get(g, None)
        if v == "n":
            return ("out_of_scope_config", f"file guard CONFIG_{g}=n")
        if v == "y":
            return ("in_scope", f"file guard CONFIG_{g}=y")
    # Subsystem-class hints.
    hints = SUBSYSTEM_SCOPE_HINTS.get(table.subsystem, [])
    if hints:
        on = [h for h in hints if config.get(h, "n") == "y"]
        off = [h for h in hints if config.get(h, "n") == "n"]
        # If at least one expected enabler is on, consider in-scope.
        if on:
            return ("in_scope", f"subsystem hint CONFIG_{on[0]}=y")
        if off and not on:
            return ("out_of_scope_config",
                    "no relevant CONFIG_ enabler set "
                    f"(checked {','.join('CONFIG_' + h for h in hints)})")
    # No actionable hint: leave unresolved.
    return ("unresolved_needs_manual_review",
            "no config guard found; classify manually")


# Indirect-call site discovery.
# Matches `iommu_call(...)`, `iommu_vcall(...)`, `something->field(`
# inside .c/.h files in scope.
DISPATCH_PATTERNS = [
    re.compile(r"\biommu_call\s*\(\s*(?P<obj>[^,]+),\s*(?P<field>\w+)\s*,"),
    re.compile(r"\biommu_vcall\s*\(\s*(?P<obj>[^,]+),\s*(?P<field>\w+)\s*,"),
    # Generic ops->field(  pattern. Captures the receiver
    # expression upstream of "->" and the dispatched field.
    re.compile(r"(?P<obj>\w+(?:\.\w+|->\w+)*)\s*->\s*(?P<field>\w+)\s*\("),
]


# Reserved words that may legitimately appear immediately before `(` but
# are not function names, so the containing-function scanner must skip
# them when extracting the identifier from a candidate signature.
_NON_FUNCTION_WORDS = {
    "if", "while", "for", "switch", "return", "sizeof", "typeof",
    "__typeof__", "_Generic", "alignof", "_Alignof",
}


def find_top_level_functions(text: str) -> list[tuple[str, int, int]]:
    """Return [(function_name, start_line, end_line)] for top-level
    function definitions in the given C source text.

    Detection is best-effort and based on Xen-style formatting where the
    function body's opening `{` sits alone at column 0 on its own line.
    Struct initializers (`= {`), function prototypes (ending in `;`),
    macro bodies, and typedefs are rejected because they do not match
    that exact column-0 brace shape or fail the trailing-`)` check.

    Line numbers are 1-indexed and inclusive on both ends.
    """
    lines = text.splitlines()
    funcs: list[tuple[str, int, int]] = []
    for i, raw in enumerate(lines):
        if raw.rstrip() != "{":
            continue
        # Walk backward to assemble the function signature.
        sig_parts: list[str] = []
        stop = False
        for j in range(i - 1, -1, -1):
            sj = lines[j].rstrip()
            stripped = sj.strip()
            if not stripped:
                if sig_parts:
                    break
                continue
            # `#define` / `#undef` may legitimately appear inside a
            # function's signature (e.g. x86emul_fpu in Xen), so step
            # past them rather than terminating the walk-back. Check
            # this before the trailing-comment heuristic, because a
            # `#define foo /* note */` line both starts with `#define`
            # and ends with `*/`.
            if stripped.startswith("#"):
                if (stripped.startswith("#define ")
                        or stripped.startswith("#undef ")):
                    continue
                break
            # End of a previous declaration / statement.
            if (stripped.endswith(";") or stripped.endswith("}")
                    or stripped.endswith("*/")):
                break
            sig_parts.insert(0, stripped)
        if not sig_parts:
            continue
        sig = " ".join(sig_parts)
        # Trim a trailing __attribute__((...)) if present (already
        # collected as part of sig_parts).
        sig = re.sub(r"\s*__attribute__\s*\(\([^)]*\)\)\s*$", "", sig)
        if not sig.endswith(")"):
            continue
        first_paren = sig.find("(")
        if first_paren < 0:
            continue
        before = sig[:first_paren].rstrip()
        m = re.search(r"(\w+)\s*\**\s*$", before)
        if not m:
            continue
        fname = m.group(1)
        if fname in _NON_FUNCTION_WORDS:
            continue
        # Walk forward from the `{` line, brace-counting, to find the
        # matching close.
        depth = 0
        end_line = -1
        for k in range(i, len(lines)):
            for ch in lines[k]:
                if ch == "{":
                    depth += 1
                elif ch == "}":
                    depth -= 1
                    if depth == 0:
                        end_line = k + 1
                        break
            if end_line > 0:
                break
        if end_line < 0:
            continue
        funcs.append((fname, i + 1, end_line))
    return funcs


def containing_function(funcs: list[tuple[str, int, int]],
                        line: int) -> str | None:
    """Return the function name whose span contains the given 1-indexed
    line, or None if no function span contains it.

    If multiple spans nest (a rare but legal C edge case), the innermost
    containing span wins.
    """
    best: tuple[int, str] | None = None
    for name, start, end in funcs:
        if start <= line <= end:
            span = end - start
            if best is None or span < best[0]:
                best = (span, name)
    return best[1] if best else None


# Runtime ops-registration patterns. Each entry is (compiled regex,
# classification label). These are deliberately broad: the output is a
# seed inventory for the analyst pass, not a semantic resolution. False
# positives are expected and acceptable; the columns include the matched
# expression so they can be filtered downstream.
REGISTRATION_PATTERNS: list[tuple[re.Pattern, str]] = [
    # register_FOO(expr, ...)
    (re.compile(
        r"\bregister_(?P<rest>\w+)\s*\(\s*(?P<expr>&?[\w\.\->]+)"
    ), "register_call"),
    # set_FOO_ops(expr, ...)
    (re.compile(
        r"\bset_(?P<rest>\w+_ops)\s*\(\s*(?P<expr>&?[\w\.\->]+)"
    ), "set_ops_call"),
    # Bare assignment to a *_ops variable:  foo_ops = &impl;
    # Restricted to `[;,]` trailers so it does not match the head of a
    # `struct foo_ops bar_ops = { ... };` initializer.
    (re.compile(
        r"\b(?P<lhs>\w+_ops)\s*=\s*(?P<expr>&?\w+)\s*[;,]"
    ), "ops_assignment"),
    # Designated initializer / runtime member assignment: `.ops = &impl`
    # or `->ops = &impl`.
    (re.compile(
        r"(?P<lhs>(?:\.|->)\s*ops)\s*=\s*(?P<expr>&?\w+)\s*[;,}]"
    ), "ops_member_assignment"),
]


# Generic parameter-like names that obscure the actual implementation
# when used as the candidate argument. A registration call whose argument
# is one of these is dynamic, not statically resolvable.
_DYNAMIC_ARGUMENT_NAMES = {
    "ops", "impl", "handler", "callback", "cb", "func", "fn",
    "ptr", "p", "x", "y", "obj", "self", "data",
    "new_ops", "old_ops", "new", "old",
}


def _classify_candidate(candidate: str,
                        table_index: dict[str, "OpsTable"],
                        config: dict[str, str],
                        near_guards: list[str]) -> tuple[str, str, str]:
    """Decide (candidate_impl, table_or_field_if_obvious, classification)
    for a runtime registration match.

    candidate is the raw expression captured for the implementation
    argument; the leading `&` has already been stripped.

    table_index maps ops-table instance names to their OpsTable; that
    lets the classifier confirm a name is a real static initializer in
    the current scan and inspect the table's config scope.

    near_guards are the file-local `CONFIG_*` guards in effect at the
    registration site's line, walked back via config_guards_near().
    """
    # Strip noise. If anything weird remains (operators, dots, arrows),
    # treat as dynamic/unresolved.
    impl = candidate.strip()
    if not impl:
        return ("", "", "unresolved")
    if not re.fullmatch(r"[A-Za-z_]\w*", impl):
        return (impl, "", "dynamic_argument")
    # Near-line CONFIG_X=n turns the whole site off regardless of impl.
    for g in near_guards:
        if config.get(g, "n") == "n":
            table_hint = impl if re.fullmatch(r"\w+_(ops|operations)", impl) else ""
            return (impl, table_hint, "out_of_scope_config")
    # Generic parameter name (e.g. the handler of a register_FOO(impl)
    # helper). Without a literal table name we can't resolve it
    # statically.
    if impl in _DYNAMIC_ARGUMENT_NAMES:
        return (impl, "", "dynamic_argument")
    # Known static ops table?
    t = table_index.get(impl)
    if t is not None:
        scope_verdict, _ = resolve_scope(t, config)
        if scope_verdict == "out_of_scope_config":
            return (impl, impl, "static_impl_config_guarded")
        return (impl, impl, "static_impl_obvious")
    # Name follows the *_ops / *_operations shape but is not in the
    # collected inventory (header-declared table, runtime container,
    # etc.). Still a literal name; record it as obvious-looking.
    if re.fullmatch(r"\w+_(ops|operations)", impl):
        return (impl, impl, "static_impl_obvious")
    # Otherwise a plain identifier whose referent is not known. Could be
    # a function literal (e.g. register_keyhandler(my_handler)), a
    # struct instance, or an opaque global. Mark as dynamic_argument so
    # the analyst pass treats it as needing resolution.
    return (impl, "", "dynamic_argument")


def discover_runtime_registration(xen_root: Path,
                                  paths: list[str],
                                  tables: list["OpsTable"],
                                  config: dict[str, str],
                                  target_arch: str = "arm64") -> list[dict]:
    """Discover runtime ops-registration sites under the given subdirs.

    The pass scans for four syntactic shapes:

      1. register_FOO(expr, ...)
      2. set_FOO_ops(expr, ...)
      3. foo_ops = &impl;            (bare assignment to a *_ops variable)
      4. .ops = &impl / ->ops = &impl

    For each match the captured implementation argument is normalised
    (a leading `&` stripped), looked up against the static ops-table
    inventory, and classified semantically. Near-line `CONFIG_*` guards
    are recorded as `config_guard_summary` and turn the row into
    `out_of_scope_config` when a guard is `=n` under the current
    config.

    Classification labels emitted:

      static_impl_obvious        candidate is a literal name and (a)
                                 matches a discovered static ops-table
                                 instance, or (b) follows the *_ops /
                                 *_operations naming convention.
      static_impl_config_guarded same as static_impl_obvious, but the
                                 referenced static ops table resolves
                                 to out_of_scope_config under the
                                 current config.
      dynamic_argument           candidate is a parameter / variable
                                 / non-trivial expression; the
                                 implementation is not knowable from
                                 the registration site alone.
      out_of_scope_config        a near-line CONFIG_X=n guard takes
                                 the entire site out of scope.
      unresolved                 candidate captured but empty / not
                                 a parseable identifier.
    """
    sites: list[dict] = []
    seen: set[tuple[str, int, str]] = set()
    table_index = {t.table_instance: t for t in tables}
    for rel in paths:
        root = xen_root / rel
        if not root.exists():
            continue
        for p in root.rglob("*.c"):
            text = p.read_text(errors="replace")
            lines = text.splitlines()
            funcs = find_top_level_functions(text)
            rel_p = str(p.relative_to(xen_root)).replace("\\", "/")
            for rx, pattern_label in REGISTRATION_PATTERNS:
                for m in rx.finditer(text):
                    line_num = text.count("\n", 0, m.start()) + 1
                    key = (rel_p, line_num, pattern_label)
                    if key in seen:
                        continue
                    seen.add(key)
                    fname = containing_function(funcs, line_num) or "(unresolved)"
                    expression = re.sub(r"\s+", " ", m.group(0).strip())
                    if len(expression) > 200:
                        expression = expression[:197] + "..."
                    # Normalise the implementation argument.
                    raw_expr = m.group("expr").strip()
                    candidate_raw = raw_expr.lstrip("&").strip()
                    near_guards = config_guards_near(lines, line_num)
                    config_guard_summary = "|".join(near_guards) if near_guards else ""
                    candidate_impl, table_hint, classification = _classify_candidate(
                        candidate_raw, table_index, config, near_guards,
                    )
                    if pattern_label == "register_call":
                        notes = f"register_{m.group('rest')}"
                    elif pattern_label == "set_ops_call":
                        notes = f"set_{m.group('rest')}"
                    elif pattern_label == "ops_assignment":
                        notes = f"lhs={m.group('lhs')}"
                    else:
                        notes = f"lhs={m.group('lhs').strip()}"
                    sites.append({
                        "source_file": rel_p,
                        "line": line_num,
                        "containing_function": fname,
                        "pattern": pattern_label,
                        "expression": expression,
                        "candidate_impl": candidate_impl,
                        "table_or_field_if_obvious": table_hint,
                        "config_guard_summary": config_guard_summary,
                        "classification": classification,
                        "arch_scope": arch_scope.classify_arch_scope(
                            rel_p, target_arch)[0],
                        "notes": notes,
                    })
    return sites


def discover_call_sites(xen_root: Path, paths: list[str],
                        target_arch: str = "arm64") -> list[dict]:
    """Discover indirect-dispatch call sites under the given subdirs.

    Each emitted site carries a best-effort containing-function name.
    Sites whose enclosing function cannot be identified by the scanner
    are emitted with call_site_function="(unresolved)" rather than
    being dropped. Each site is tagged with its architecture scope
    against target_arch; non-target arch sites are retained and marked
    out_of_scope_arch.
    """
    sites: list[dict] = []
    field_filter = {
        # Fields named here are typical alloc-relevant dispatches.
        "map_page", "unmap_page", "assign_device", "iotlb_flush",
        "alloc_page_table", "free_page_table",
        "read", "write", "intercept",
        "alloc_pdata", "alloc_vdata", "alloc_domdata", "init", "deinit",
        "do_domctl", "do_sysctl",
        "handle_call",
        "register", "deregister",
    }
    for rel in paths:
        root = xen_root / rel
        if not root.exists():
            continue
        for p in root.rglob("*.c"):
            text = p.read_text(errors="replace")
            funcs = find_top_level_functions(text)
            rel_p = str(p.relative_to(xen_root)).replace("\\", "/")
            ascope = arch_scope.classify_arch_scope(
                rel_p, target_arch)[0]
            for pattern in DISPATCH_PATTERNS:
                for m in pattern.finditer(text):
                    field = m.group("field")
                    if field not in field_filter:
                        continue
                    line_num = text.count("\n", 0, m.start()) + 1
                    fname = containing_function(funcs, line_num) or "(unresolved)"
                    sites.append({
                        "call_site_function": fname,
                        "source_file": rel_p,
                        "line_or_context": line_num,
                        "receiver_expression": m.group("obj").strip(),
                        "field_name": field,
                        "arch_scope": ascope,
                    })
    return sites


def write_config_scope(out: Path, config_path: Path, config: dict[str, str],
                       extras: dict[str, str]):
    out.mkdir(parents=True, exist_ok=True)
    f = out / "config-scope.md"
    lines = []
    lines.append("# Configuration scope (auto-generated)\n")
    lines.append(f"Generated from `{config_path}`.")
    if extras:
        rendered = ", ".join(f"`CONFIG_{k}={v}`" for k, v in sorted(extras.items()))
        lines.append(f"Additional --extra settings: {rendered}.")
    lines.append("")
    lines.append("This file is regenerated by scripts/collect.py. Do not edit.\n")
    lines.append("## Effective Kconfig values\n")
    lines.append("```")
    for name, val in sorted(config.items()):
        if val == "n":
            lines.append(f"# CONFIG_{name} is not set")
        else:
            lines.append(f"CONFIG_{name}={val}")
    lines.append("```\n")
    f.write_text("\n".join(lines))


def write_ops_inventory(out: Path, tables: list[OpsTable], config: dict[str, str]):
    f = out / "ops-inventory.csv"
    with f.open("w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow([
            "table_type", "table_instance", "source_file", "line",
            "subsystem", "config_guards", "config_scope", "arch_scope",
            "basis", "fields_summary"
        ])
        for t in tables:
            verdict, basis = resolve_scope(t, config)
            fields_summary = "; ".join(
                f"{k}={v}" for k, v in sorted(t.fields.items())
            )[:300]
            w.writerow([
                t.table_type, t.table_instance, t.source_file, t.line,
                t.subsystem,
                "|".join(t.config_guards) if t.config_guards else "",
                verdict, t.arch_scope, basis, fields_summary,
            ])


def write_ops_resolution(out: Path, tables: list[OpsTable], config: dict[str, str],
                         config_name: str):
    f = out / "ops-resolution.csv"
    with f.open("w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow([
            "config_name", "table_type", "table_instance", "field_name",
            "implementation_function", "config_scope", "arch_scope",
            "basis", "source_file", "line"
        ])
        for t in tables:
            verdict, basis = resolve_scope(t, config)
            # Out-of-scope rows (config or arch) are not emitted to the
            # resolution matrix; the inventory retains them for audit.
            if verdict in ("out_of_scope_config", "out_of_scope_arch"):
                continue
            for field, impl in sorted(t.fields.items()):
                w.writerow([
                    config_name, t.table_type, t.table_instance, field,
                    impl, verdict, t.arch_scope, basis, t.source_file, t.line,
                ])


RUNTIME_REGISTRATION_COLUMNS = [
    "source_file", "line", "containing_function",
    "pattern", "expression",
    "candidate_impl", "table_or_field_if_obvious", "config_guard_summary",
    "classification", "arch_scope", "notes",
]


def write_runtime_registration(out: Path, sites: list[dict]):
    f = out / "runtime-ops-registration-sites.csv"
    with f.open("w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(RUNTIME_REGISTRATION_COLUMNS)
        for s in sites:
            w.writerow([s[k] for k in RUNTIME_REGISTRATION_COLUMNS])


def write_runtime_registration_summary(out: Path, sites: list[dict],
                                       config_name: str):
    f = out / "runtime-ops-registration-summary.md"
    from collections import Counter
    by_class = Counter(s["classification"] for s in sites)
    by_pattern = Counter(s["pattern"] for s in sites)
    top_impls = Counter(
        s["candidate_impl"] for s in sites if s["candidate_impl"]
    ).most_common(10)
    lines: list[str] = []
    lines.append("# Runtime ops-registration inventory summary\n")
    lines.append(f"Config: `{config_name}`\n")
    lines.append(f"Total sites: **{len(sites)}**\n")
    lines.append("## Classification breakdown\n")
    lines.append("| Classification | Count |")
    lines.append("| --- | --- |")
    for label in ("static_impl_obvious", "static_impl_config_guarded",
                  "small_set_selector", "dynamic_argument",
                  "out_of_scope_config", "unresolved"):
        lines.append(f"| `{label}` | {by_class.get(label, 0)} |")
    lines.append("")
    lines.append("## Pattern breakdown\n")
    lines.append("| Pattern | Count |")
    lines.append("| --- | --- |")
    for label in ("register_call", "set_ops_call", "ops_assignment",
                  "ops_member_assignment"):
        lines.append(f"| `{label}` | {by_pattern.get(label, 0)} |")
    lines.append("")
    if top_impls:
        lines.append("## Top candidate_impl values (by frequency)\n")
        lines.append("| candidate_impl | Count |")
        lines.append("| --- | --- |")
        for name, n in top_impls:
            lines.append(f"| `{name}` | {n} |")
        lines.append("")
    lines.append(
        "This inventory is a seed for the analyst pass; classification "
        "is per-row syntactic + ops-table-index lookup, not full "
        "semantic resolution. `dynamic_argument` and "
        "`static_impl_config_guarded` rows still need analyst review.\n"
    )
    f.write_text("\n".join(lines))


def write_call_sites(out: Path, sites: list[dict]):
    f = out / "indirect-call-sites.csv"
    with f.open("w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow([
            "source_file", "line_or_context", "receiver_expression",
            "field_name", "call_site_function", "arch_scope",
        ])
        for s in sites:
            w.writerow([
                s["source_file"], s["line_or_context"],
                s["receiver_expression"], s["field_name"],
                s["call_site_function"], s.get("arch_scope",
                                               arch_scope.IN_SCOPE),
            ])


def write_summary(out: Path, config_name: str, config_path: Path,
                  tables: list[OpsTable], sites: list[dict],
                  reg_sites: list[dict], config: dict[str, str],
                  target_arch: str = "arm64"):
    f = out / "collection-summary.json"
    total_field_assignments = sum(len(t.fields) for t in tables)
    tables_with_fields = sum(1 for t in tables if t.fields)
    ops_resolution_rows = sum(
        len(t.fields) for t in tables
        if resolve_scope(t, config)[0]
        not in ("out_of_scope_config", "out_of_scope_arch")
    )
    sites_with_function = sum(
        1 for s in sites if s["call_site_function"] != "(unresolved)"
    )
    reg_with_function = sum(
        1 for s in reg_sites if s["containing_function"] != "(unresolved)"
    )
    OOSA = arch_scope.OUT_OF_SCOPE_ARCH
    ops_tables_out_of_scope_arch = sum(
        1 for t in tables if t.arch_scope == OOSA)
    call_sites_out_of_scope_arch = sum(
        1 for s in sites if s.get("arch_scope") == OOSA)
    reg_sites_out_of_scope_arch = sum(
        1 for s in reg_sites if s.get("arch_scope") == OOSA)
    from collections import Counter
    reg_by_class = Counter(s["classification"] for s in reg_sites)
    summary = {
        "config_name": config_name,
        "config_path": str(config_path),
        "target_arch": target_arch,
        "config_symbol_count": len(config),
        "ops_tables_discovered": len(tables),
        "ops_tables_with_fields": tables_with_fields,
        "total_field_assignments": total_field_assignments,
        "ops_resolution_rows": ops_resolution_rows,
        "ops_tables_out_of_scope_arch": ops_tables_out_of_scope_arch,
        "indirect_call_sites": len(sites),
        "indirect_call_sites_with_function": sites_with_function,
        "indirect_call_sites_without_function": len(sites) - sites_with_function,
        "indirect_call_sites_out_of_scope_arch":
            call_sites_out_of_scope_arch,
        "runtime_registration_sites": len(reg_sites),
        "runtime_registration_sites_found": len(reg_sites),
        "runtime_registration_sites_with_function": reg_with_function,
        "runtime_registration_out_of_scope_arch":
            reg_sites_out_of_scope_arch,
        "runtime_registration_static_impl_obvious":
            reg_by_class.get("static_impl_obvious", 0),
        "runtime_registration_static_impl_config_guarded":
            reg_by_class.get("static_impl_config_guarded", 0),
        "runtime_registration_dynamic_argument":
            reg_by_class.get("dynamic_argument", 0),
        "runtime_registration_out_of_scope_config":
            reg_by_class.get("out_of_scope_config", 0),
        "runtime_registration_unresolved":
            reg_by_class.get("unresolved", 0),
    }
    f.write_text(json.dumps(summary, indent=2))
    return summary


def parse_extras(extras_args: list[str]) -> dict[str, str]:
    """Parse --extra CONFIG_FOO=y entries into a dict."""
    out: dict[str, str] = {}
    for e in extras_args or []:
        if "=" not in e:
            continue
        k, v = e.split("=", 1)
        k = k.strip()
        if k.startswith("CONFIG_"):
            k = k[len("CONFIG_"):]
        out[k] = v.strip()
    return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--xen-root", required=True, type=Path,
                   help="Path to Xen source tree (e.g. ./)")
    p.add_argument("--config", type=Path, default=None,
                   help="Path to a Kconfig-style .config / defconfig file, or "
                        "a shell-style CI env-file fragment containing literal "
                        "EXTRA_XEN_CONFIG CONFIG_* tokens. If omitted, defaults "
                        "to <xen-root>/xen/arch/arm/configs/arm64_safety for "
                        "local smoke testing. CI should pass --config "
                        "explicitly.")
    p.add_argument("--config-name", default=None,
                   help="Symbolic name for this config (used in CSV headers)")
    p.add_argument("--extra", action="append", default=[],
                   help="Additional CONFIG_FOO=y settings (repeatable)")
    p.add_argument("--out-dir", required=True, type=Path,
                   help="Output directory for the generated artifacts")
    p.add_argument("--scan-paths", default="xen/arch,xen/common,xen/drivers",
                   help="Comma-separated tree-relative paths to scan")
    p.add_argument("--target-arch", default="arm64",
                   help="Target architecture for the analysis run. "
                        "Sources under a non-target xen/arch/<a>/ tree "
                        "are tagged out_of_scope_arch (retained for "
                        "audit, excluded from the resolution and "
                        "candidate matrices). Default: arm64.")
    args = p.parse_args()

    # Resolve the default config relative to --xen-root, not the current
    # working directory. This lets the script run from outside the repo
    # tree with the default config still pointing at the right defconfig.
    if args.config is None:
        config_path = args.xen_root / "xen/arch/arm/configs/arm64_safety"
    else:
        config_path = args.config

    config = read_config(config_path)
    extras = parse_extras(args.extra)
    config.update(extras)
    config_name = args.config_name or config_path.name

    paths = args.scan_paths.split(",")
    tables = discover_ops_tables(args.xen_root, paths, args.target_arch)
    sites = discover_call_sites(args.xen_root, paths, args.target_arch)
    reg_sites = discover_runtime_registration(args.xen_root, paths,
                                              tables, config,
                                              args.target_arch)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    write_config_scope(args.out_dir, config_path, config, extras)
    write_ops_inventory(args.out_dir, tables, config)
    write_ops_resolution(args.out_dir, tables, config, config_name)
    write_call_sites(args.out_dir, sites)
    write_runtime_registration(args.out_dir, reg_sites)
    write_runtime_registration_summary(args.out_dir, reg_sites, config_name)
    summary = write_summary(args.out_dir, config_name, config_path,
                            tables, sites, reg_sites, config,
                            args.target_arch)

    print(f"OK: wrote artifacts to {args.out_dir}", file=sys.stderr)
    print(f"  config symbols           : {summary['config_symbol_count']}",
          file=sys.stderr)
    print(f"  ops tables discovered    : {summary['ops_tables_discovered']}",
          file=sys.stderr)
    print(f"  ops tables with fields   : {summary['ops_tables_with_fields']}",
          file=sys.stderr)
    print(f"  total field assignments  : {summary['total_field_assignments']}",
          file=sys.stderr)
    print(f"  ops resolution rows      : {summary['ops_resolution_rows']}",
          file=sys.stderr)
    print(f"  indirect call sites      : {summary['indirect_call_sites']}",
          file=sys.stderr)
    print(f"    with function          : {summary['indirect_call_sites_with_function']}",
          file=sys.stderr)
    print(f"    without function       : {summary['indirect_call_sites_without_function']}",
          file=sys.stderr)
    print(f"  runtime reg sites        : {summary['runtime_registration_sites']}",
          file=sys.stderr)
    print(f"    static_impl_obvious     : {summary['runtime_registration_static_impl_obvious']}",
          file=sys.stderr)
    print(f"    static_impl_config_guarded: {summary['runtime_registration_static_impl_config_guarded']}",
          file=sys.stderr)
    print(f"    dynamic_argument        : {summary['runtime_registration_dynamic_argument']}",
          file=sys.stderr)
    print(f"    out_of_scope_config     : {summary['runtime_registration_out_of_scope_config']}",
          file=sys.stderr)
    print(f"    unresolved              : {summary['runtime_registration_unresolved']}",
          file=sys.stderr)

    # Hard-fail when discovery clearly worked but field extraction did
    # not: an empty ops-resolution.csv must not pass silently.
    if summary["ops_tables_discovered"] > 0 and \
       summary["total_field_assignments"] == 0:
        print(
            "ERROR: ops_tables_discovered > 0 but "
            "total_field_assignments == 0; FIELD_ASSIGN_PATTERN likely "
            "failed (check re.MULTILINE / initializer-body extraction).",
            file=sys.stderr,
        )
        raise SystemExit(2)


if __name__ == "__main__":
    main()
