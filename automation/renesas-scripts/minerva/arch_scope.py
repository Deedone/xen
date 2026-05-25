#!/usr/bin/env python3
"""Architecture-scope classification for Minerva analysis.

A single Xen source tree contains every architecture's `xen/arch/<a>/`
subtree. An analysis run targets one architecture (`--target-arch`),
so sources under a non-target arch tree are not reachable in the
target build and must not pollute the target's reachability matrix.

This module is the single source of truth for that decision, shared
by the collector (ops-table discovery, ops-resolution, indirect
call-site discovery, runtime-registration inventory) and by the
workbench (reachability candidate generation). It classifies a
tree-relative source path against a target arch and returns a verdict
plus a human-readable basis.

Policy (rows are never silently dropped):

  - `xen/arch/<target>/`                  -> in_scope
  - `xen/common/`, `xen/drivers/`,
    `xen/include/`, `xen/lib/`, `xen/test/`,
    and anything outside `xen/arch/`        -> in_scope (shared)
  - `xen/arch/<other-arch>/`              -> out_of_scope_arch

Consumers record `out_of_scope_arch` rows with
`candidate_included = false` and `exclusion_reason =
out_of_scope_arch` so reports can audit what was excluded.
"""

from __future__ import annotations

# Canonical Xen arch tree names. `--target-arch` values like `arm64`
# and `x86_64` map onto the arch *tree* names `arm` and `x86`.
_ARCH_TREE_DIRS = ("arm", "x86", "riscv", "ppc")

# Map a --target-arch value onto its xen/arch/<dir> tree. Unknown or
# already-canonical values pass through unchanged so a future arch
# tree needs only a one-line addition here (or works as-is when the
# target value already equals the tree name).
_TARGET_ARCH_TO_TREE = {
    "arm64": "arm",
    "arm32": "arm",
    "aarch64": "arm",
    "arm": "arm",
    "x86_64": "x86",
    "x86-64": "x86",
    "amd64": "x86",
    "x86": "x86",
    "riscv64": "riscv",
    "riscv": "riscv",
    "ppc64": "ppc",
    "ppc": "ppc",
    "powerpc": "ppc",
}

IN_SCOPE = "in_scope"
OUT_OF_SCOPE_ARCH = "out_of_scope_arch"
EXCLUSION_REASON = "out_of_scope_arch"


def target_arch_tree(target_arch: str) -> str:
    """Return the xen/arch/<dir> tree name for a --target-arch value.

    Falls back to the lowercased input when unmapped, so an unknown
    target still classifies its own `xen/arch/<input>/` as in-scope
    rather than excluding everything.
    """
    t = (target_arch or "").strip().lower()
    return _TARGET_ARCH_TO_TREE.get(t, t)


def _normalize(rel_path: str) -> str:
    return (rel_path or "").replace("\\", "/").lstrip("./")


def arch_of_path(rel_path: str) -> str | None:
    """Return the arch tree name a path belongs to, or None if shared.

    `xen/arch/arm/foo.c` -> "arm"; `xen/common/bar.c` -> None.
    Tolerates a leading `xen/` being present or absent.
    """
    p = _normalize(rel_path)
    parts = p.split("/")
    # Find the "arch" segment and take the following segment.
    for i, seg in enumerate(parts[:-1]):
        if seg == "arch":
            return parts[i + 1] if i + 1 < len(parts) else None
    return None


def classify_arch_scope(rel_path: str, target_arch: str
                        ) -> tuple[str, str]:
    """Classify a tree-relative source path against a target arch.

    Returns (verdict, basis):
      - (IN_SCOPE, ...)          shared tree, or the target arch tree;
      - (OUT_OF_SCOPE_ARCH, ...) a different arch's tree.
    """
    tree = target_arch_tree(target_arch)
    arch = arch_of_path(rel_path)
    if arch is None:
        return (IN_SCOPE, "shared tree (not under xen/arch/)")
    if arch == tree:
        return (IN_SCOPE, f"target arch tree xen/arch/{arch}/")
    return (OUT_OF_SCOPE_ARCH,
            f"non-target arch tree xen/arch/{arch}/ "
            f"(target {target_arch} -> xen/arch/{tree}/)")


def is_in_scope(rel_path: str, target_arch: str) -> bool:
    """Convenience predicate: True unless the path is a non-target
    arch tree."""
    return classify_arch_scope(rel_path, target_arch)[0] == IN_SCOPE


# --------------------------------------------------------------------
# Self-test. `python3 scripts/arch_scope.py --self-test`.
# --------------------------------------------------------------------

def _self_test() -> int:
    import sys
    fails: list[str] = []

    def check(name: str, cond: bool):
        if cond:
            print(f"ok   {name}", file=sys.stderr)
        else:
            fails.append(name)
            print(f"FAIL {name}", file=sys.stderr)

    # arm64 target.
    check("arm64.arm_in",
          classify_arch_scope("xen/arch/arm/p2m.c", "arm64")[0]
          == IN_SCOPE)
    check("arm64.x86_out",
          classify_arch_scope("xen/arch/x86/mm.c", "arm64")[0]
          == OUT_OF_SCOPE_ARCH)
    check("arm64.riscv_out",
          classify_arch_scope("xen/arch/riscv/setup.c", "arm64")[0]
          == OUT_OF_SCOPE_ARCH)
    check("arm64.ppc_out",
          classify_arch_scope("xen/arch/ppc/mm.c", "arm64")[0]
          == OUT_OF_SCOPE_ARCH)
    check("arm64.common_in",
          classify_arch_scope("xen/common/memory.c", "arm64")[0]
          == IN_SCOPE)
    check("arm64.drivers_in",
          classify_arch_scope("xen/drivers/passthrough/iommu.c",
                              "arm64")[0] == IN_SCOPE)
    check("arm64.include_in",
          classify_arch_scope("xen/include/xen/sched.h", "arm64")[0]
          == IN_SCOPE)

    # x86_64 target inverts.
    check("x86.x86_in",
          classify_arch_scope("xen/arch/x86/mm.c", "x86_64")[0]
          == IN_SCOPE)
    check("x86.arm_out",
          classify_arch_scope("xen/arch/arm/p2m.c", "x86_64")[0]
          == OUT_OF_SCOPE_ARCH)
    check("x86.common_in",
          classify_arch_scope("xen/common/memory.c", "x86_64")[0]
          == IN_SCOPE)

    # Path normalization: leading ./ and backslashes, missing xen/.
    check("norm.backslash",
          classify_arch_scope("xen\\arch\\x86\\mm.c", "arm64")[0]
          == OUT_OF_SCOPE_ARCH)
    check("norm.no_xen_prefix",
          classify_arch_scope("arch/x86/mm.c", "arm64")[0]
          == OUT_OF_SCOPE_ARCH)
    check("norm.dot_slash",
          classify_arch_scope("./xen/arch/arm/p2m.c", "arm64")[0]
          == IN_SCOPE)

    # Target-arch alias mapping.
    check("alias.aarch64",
          target_arch_tree("aarch64") == "arm")
    check("alias.amd64",
          target_arch_tree("amd64") == "x86")
    check("alias.unknown_passthrough",
          target_arch_tree("mips") == "mips")
    # An unknown target still treats its own arch tree as in-scope.
    check("alias.unknown_self_in",
          classify_arch_scope("xen/arch/mips/foo.c", "mips")[0]
          == IN_SCOPE)

    # arch_of_path helper.
    check("arch_of.arm", arch_of_path("xen/arch/arm/p2m.c") == "arm")
    check("arch_of.shared",
          arch_of_path("xen/common/memory.c") is None)

    if fails:
        print(f"\nSELF-TEST FAILED: {len(fails)} check(s): "
              f"{', '.join(fails)}", file=sys.stderr)
        return 1
    print("\nSELF-TEST PASSED", file=sys.stderr)
    return 0


def main() -> int:
    import argparse
    p = argparse.ArgumentParser(
        description="Classify a tree-relative source path against a "
                    "target architecture, or run the self-test.")
    p.add_argument("--self-test", action="store_true")
    p.add_argument("--target-arch", default="arm64")
    p.add_argument("path", nargs="?")
    args = p.parse_args()
    if args.self_test:
        return _self_test()
    if not args.path:
        p.error("a path argument is required unless --self-test")
    verdict, basis = classify_arch_scope(args.path, args.target_arch)
    print(f"{verdict}\t{basis}")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
