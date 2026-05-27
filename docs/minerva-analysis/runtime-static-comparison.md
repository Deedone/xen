# Runtime/static comparison

`runtime_static_compare.py` compares runtime-observed allocation
paths against the static artifacts generated from the same expanded
config and source revision: the direct-static target trees and the
indirect reachability candidates. It is run by the CI driver
(`indirect_ci_driver.py`) after the runtime log parser, and emits a
per-run comparison; it never claims an allocation bound, and never
treats a runtime non-observation as impossible.

## Inputs

```
runtime/parsed/        parser output (see "Parser output" below)
reachability/          synthetic_edges.candidates.yaml (indirect)
direct-static/         <target>.functions / <target>.paths
```

The comparator reads runtime paths in this order of preference:

1. `runtime/parsed/allocation-paths.json` (or `.csv`) — structured
   records, if the parser or driver provides them.
2. otherwise the parser's text reports, read recursively from
   `runtime/parsed/**`.

## Parser output

`log_parser.py` writes per-architecture reports, sharding by arch:

```
runtime/parsed/
  comments
  <arch>/<job>.log
  <arch>/verbose/<job>.log
```

The driver passes `--parsed-output-dir runtime/parsed` so the parser
writes where the comparator reads. The comparator recurses into
`<arch>/` and reads the per-arch report block (`[dN] <fn>:`
introducing a record, with the call path under `max size path:`) in
addition to the flat `comments` records. `.json` files are not read
as text reports, and a record that resolves to no known allocation
target is ignored, so a free-path entry, comment, or diagnostic
section never becomes a phantom runtime path.

## Runtime path classification

Each runtime path is classified into exactly one class:

| Class | Meaning |
| --- | --- |
| `direct_static_explained` | a non-target frame is in the direct-static reaching set of the path's target |
| `indirect_candidate_explained` | a frame is an indirect candidate that reaches the path's target |
| `target_observed_no_caller_context` | the target was observed, but the trace has no caller frame above it to match against a static path |
| `runtime_only_unexplained` | frames present, but none matches a static reaching set |
| `boundary_or_parser_artifact` | no frames (empty record) |
| `unresolved_normalization_mismatch` | target known, but no frame lined up and the target could not be resolved to a caller path |

`target_observed_no_caller_context` is for a trace that, after frame
normalization, is just the allocation target itself with no caller
(see allocator aliases below). The target was genuinely observed at
runtime, so it is not `runtime_only_unexplained`; but there is no
caller frame to attribute to a direct or indirect static path, so it
is not `*_explained` either. A record carrying a target with no
frames at all is a parser artifact, not this class.

## Allocator-layer aliases

A runtime trace may name a lower allocator helper while the static
target trees are framed around the enclosing entry point. The
`CONFIG_MINERVA_ANALYSIS` WARN inside `_xmalloc()`
(`xen/common/xmalloc_tlsf.c`) prints a line labelled
`xmem_pool_alloc` — the helper `_xmalloc` is about to call — so the
runtime head frame is `xmem_pool_alloc` while the static target is
`_xmalloc`. They are the same allocation site, named at different
layers.

`ALLOCATOR_ALIASES` maps such a runtime label to its canonical static
target:

```python
ALLOCATOR_ALIASES = {
    "xmem_pool_alloc": "_xmalloc",   # WARN inside _xmalloc(), tlsf.c
}
```

The map is applied at every runtime-path ingress point so structured
and text inputs behave identically: `_frame_func()` (text frames),
`_resolve_target()` (text report target), and `_coerce_path_record()`
(structured JSON/CSV).

### Adding a new allocator WARN site

This map is a **declared, source-grounded** list, not a fuzzy
"close-enough allocator name" mechanism. Keep it that way.

Add an entry only when a `CONFIG_MINERVA_ANALYSIS` WARN prints a label
that differs from the function that encloses it (the static target).
To check a site, compare the label in its `printk` string against its
enclosing function. As of this writing only `_xmalloc` aliases; the
other instrumented sites print their own enclosing name:

| WARN label | enclosing function | aliased |
| --- | --- | --- |
| `xmem_pool_alloc` | `_xmalloc` | yes |
| `xmalloc_whole_pages` | `xmalloc_whole_pages` | no |
| `alloc_domheap_pages` | `alloc_domheap_pages` | no |
| `free_xenheap_pages` | `free_xenheap_pages` | no (free) |
| `free_domheap_pages` | `free_domheap_pages` | no (free) |

When you add a pair, cite the WARN site in a trailing comment, as
above. Do not add prefix/substring rules (e.g. "anything containing
`xmalloc`"): an over-broad alias silently misattributes unrelated
traces and defeats the point of an explicit map.

## Outputs

```
runtime-static/
  runtime-static-comparison.json   metrics + comparison_status
  runtime-static-comparison.md     human-readable summary
  comparison-summary.csv           one row per metric
  runtime-unmatched-paths.csv      unexplained / unresolved paths
  static-only-indirect-paths.csv   candidates not observed this run
```

The tool reports only `comparison_status=COMPARED`; it does not infer
the run-level label. The CI driver owns `COMPLETE` / `STATIC_ONLY` /
`PROXY` / `PARTIAL`. A comparison over zero runtime paths is still
`COMPARED` with `runtime_paths_total=0`; the driver maps that to
`STATIC_ONLY` ("nothing to compare"), and a trivial runtime command
(`true` / `:` / `/bin/true`) to `STATIC_ONLY` ("runtime not
attempted"). `COMPLETE` requires a real workload that produced
parseable paths and a manifest aligned to this run.

All metrics are per-run artifacts, not committed constants.
