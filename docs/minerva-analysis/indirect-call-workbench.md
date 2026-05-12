# Indirect-call analysis workbench

The workbench answers: starting from a Xen configuration, which
indirect-dispatch call sites can plausibly reach an allocation
target, and through which `*_ops` table fields?

It is composed of two scripts:

- **`scripts/collect.py`**--  the scoping collector. Reads a
  Xen `.config` (or a defconfig + `--extra` overlay) and
  produces an inventory of in-scope ops tables, the
  indirect-dispatch call sites under the scanned subtrees,
  and runtime-registration sites.
- **`scripts/indirect_reachability.py`**--  the reachability
  workbench. Consumes collector output and a GCC `.ci`
  callgraph (the same artifact format
  `minerva_static_analysis/callpath.py` accepts) and produces
  per-target reachability summaries.

## `collect.py`

```
python3 scripts/collect.py \
    --xen-root <path-to-xen-source> \
    [--config <path-to-config>] \
    [--extra CONFIG_FOO=y]... \
    --config-name <symbolic-name> \
    --out-dir <out>
```

`--config` accepts three input shapes:

1. **Kconfig-style file**--  `CONFIG_FOO=value` and
   `# CONFIG_FOO is not set` lines.
2. **defconfig fragment**--  same syntax, smaller subset.
3. **CI env-file fragment**--  shell-style assignments such as
   `EXTRA_XEN_CONFIG="CONFIG_IOREQ_SERVER=y CONFIG_DEBUG_INFO=y"`.
   Parsed literally: quotes stripped, whitespace-split, each
   `CONFIG_FOO=value` token recorded. Tokens starting with
   `$` are ignored; no shell is executed; YAML colon syntax
   is not supported.

`--extra CONFIG_FOO=y` (repeatable) overlays additional symbols
on top of the file.

### Use a fully expanded `.config` for final measurements

`collect.py` reads the supplied config literally. It does not
parse Kconfig dependency clauses or apply Kconfig `default`
rules. A symbol that is `y` in a built kernel only by virtue
of a `default y` elsewhere will not be `y` in the collector's
view of a defconfig fragment.

For quick scoping passes--  comparing two CI fragments,
sanity-checking a defconfig--  running the collector against a
defconfig is fine. For measurements feeding the reachability
workbench, supply the fully expanded `.config` the Xen build
itself uses (the post-`olddefconfig` file).

### Outputs

```
<out>/
  config-scope.md                     # effective Kconfig surface
  ops-inventory.csv                   # ops-table inventory
  ops-resolution.csv                  # in-scope field resolution
  indirect-call-sites.csv             # dispatch call sites
  runtime-ops-registration-sites.csv  # runtime registration sites
  runtime-ops-registration-summary.md
  collection-summary.json             # counts + provenance
```

Per-file summary:

- `config-scope.md`--  the effective Kconfig surface, rendered
  in canonical defconfig order with an auto-generated header.
- `ops-inventory.csv`--  every `struct *_ops` /
  `*_operations` definition under the scanned subdirectories,
  with a per-table verdict (`in_scope`,
  `out_of_scope_config`, `unresolved_needs_manual_review`)
  derived from `CONFIG_*` guards near the definition and a
  subsystem-class hint table.
- `ops-resolution.csv`--  for the in-scope subset, every
  (field  ->  implementation) entry with the resolution basis.
- `indirect-call-sites.csv`--  every indirect-dispatch call
  site found, filtered to alloc-relevant fields. Each row
  carries the containing function in `call_site_function`
  (or `(unresolved)` if the function-span scanner could not
  identify one).
- `runtime-ops-registration-sites.csv`--  syntactic
  inventory of runtime ops bindings: `register_FOO(...)`,
  `set_FOO_ops(...)`, `foo_ops = &impl;`, and
  `.ops =` / `->ops =` designated initializers or runtime
  member assignments.
- `collection-summary.json`--  provenance metadata and
  counters.

The collector exits non-zero if `ops_tables_discovered > 0`
and `total_field_assignments == 0`--  a guard against
header-only `ops-resolution.csv` produced by a broken
field-extraction regex.

### Runtime-registration rows are a separate worklist

`runtime-ops-registration-sites.csv` is a **syntactic seed
inventory**, not a semantic resolution. The four recognised
shapes catch the common registration patterns; they do not
pair each site with its resolved implementation, and they do
not follow callback registration helpers
(`set_*_callback`, etc.) beyond what those four patterns
cover. The analyst pass filters and resolves these rows by
hand; they are not expanded into per-field reachability
queries by the workbench.

## `indirect_reachability.py`

```
python3 scripts/indirect_reachability.py \
    --collector-run <out-from-collect.py> \
    --targets alloc_domheap_pages alloc_xenheap_pages _xmalloc \
    --out-dir <reachability-out> \
    --xen-root <xen-source-root> \
    --ci-dir <path-to-.ci-tree>
```

The workbench takes collector output and a GCC `.ci`
callgraph and produces, per allocation target:

- the set of functions that statically reach the target via
  the callgraph (reverse BFS over direct + synthetic edges);
- candidate `*_ops` field bindings whose implementations
  appear in that set (table-aware binding);
- a synthetic-edge candidates manifest in YAML;
- an excluded-edges manifest recording the field-name-only
  rows kept for diagnostics but not promoted to synthetic
  edges.

### Target-tree reachability

The reachability strategy is per-target: for each allocation
target, compute the set of functions that can reach it via
the callgraph, then check candidate-implementation
membership against that set. This keeps the cost linear in
the callgraph for a fixed target list.

### Table-aware candidate binding

Candidate bindings come from two sources:

1. **Field-name + table-class match**--  a `*_ops` table
   whose subsystem class matches the dispatch site's
   declared interface, with the field name present in the
   table definition.
2. **Field-name-only match**--  the field name appears in
   the table but the table class does not match the
   declared interface. These rows are kept in
   `indirect-allocation-paths.csv` for analyst review but
   are **excluded** from `synthetic_edges.candidates.yaml`
   and from the `candidate_synthetic_edges_included`
   counter. They appear in `synthetic_edges.excluded.yaml`
   so the operator can audit what was dropped.

### Outputs

```
<reachability-out>/
  indirect-allocation-paths.csv
  synthetic_edges.candidates.yaml
  synthetic_edges.excluded.yaml
  reachability-summary.md
  candidate-binding-summary.md
```

## What this workbench does not assert

- No allocation bound. `path_found=yes` is a static-graph
  reachability statement only.
- `path_found=no` does not mean impossible. The function may
  be reachable via macros, function pointers not modelled by
  the callgraph, or excluded paths.
- Runtime-registration rows are not expanded into per-field
  reachability queries; they remain an analyst worklist.
- Field-name-only candidate rows are kept in
  `indirect-allocation-paths.csv` for diagnostics but
  excluded from `synthetic_edges.candidates.yaml`.

## Verification

```
python3 -m py_compile scripts/collect.py
python3 -m py_compile scripts/indirect_reachability.py
python3 scripts/collect.py -h
python3 scripts/indirect_reachability.py -h
```
