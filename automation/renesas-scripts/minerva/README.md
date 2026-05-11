# Runtime Anonymous Memory Allocation Tracking and Evaluation Tool

The tool described in this document automate the process of evaluating
"anonymous" memory allocations performed at runtime from the shared Xen heap.

- Runtime:
  here means after the system has transitioned to the `SYS_STATE_active` state

- Anonymous:
  allocations that are insisting on the common Xen heap pool

The system is composed by a set of patches added to `xen/common/page_alloc.c`
and `xen/common/xmalloc_tlsf.c` that will result in a warning stack being
produced at runtime when anonymous allocations are detected.
In such cases, the warning will try to detect additional information like:
the domain currently running and the pointer to the allocated memory area.
Such information is later matched (if possible) against corresponding frees.
Specifically, if possible, anonymous memory allocated via `alloc_domheap_pages`
is matched against `free_domheap_pages`, while memory allocated via `xmalloc` is
matched against `xfree`.

Depending on the type of test, the amount of information discovered by the tool
might differ.

The execution is integrated in Xen pipelines and triggered by commits to
`minerva/warn` branch.
The `minerva_analysis` stage will also start the evaluation that can be started
otherwise manually by executing the `log_parser.py` script with the
`<pipeline_id>` as parameter.

```
usage: log_parser [-h] [--project-id PROJECT_ID] [--force-download] [--verbose] [-e FUNCTION [FUNCTION ...]] [--comments-path COMMENTS_PATH] pipeline_id

Parse Xen Gitlab CI job log

positional arguments:
  pipeline_id           Pipeline id to download or to parse if it has already been downloaded into <pipeline_id>/ directory

options:
  -h, --help            show this help message and exit
  --project-id PROJECT_ID
                        Fetch project id instead of default 22438264
  --force-download      Force download of the logs
  --verbose             Verbose path printing
  -e FUNCTION [FUNCTION ...], --exclude FUNCTION [FUNCTION ...]
                        Set of functions to be excluded from parsing
  --comments-path COMMENTS_PATH
```

For each log file produced by each test the evaluation will produce a summary
and detailed parsed information.
The summary will be displayed on `stdout`, the detailed information saved in a
`<pipeline_id>` directory.

## Evaluation Summary

For each processed log file, a summary of allocation/frees is produced.
The summary is organized per log file (see `[LOG:
<logfile>]`).

The log is organized by domain (`[d0]` indicates domain 0), with each section
corresponding to a memory operation type.

### Example Structure

```
[d0] alloc_domheap_pages:
  sizes (B): [4096, 4096, ...]
  num calls: 27
  sum size:  110592 B
  min size:  4096 B
  max size:  4096 B
  max size path:
    alloc_domheap_pages(size=4096) [domain: d0]
      _xzalloc
        _xmalloc
```

### Common Fields

- **`[d0] <operation>:`**:
  Memory operation type for domain 0 (`alloc_domheap_pages`, `xfree`,
  `xmem_pool_alloc`).
- **`sizes (B):`**:
  List of all allocation (or deallocation) sizes.
- **`num calls:`**:
  Total number of calls.
- **`sum size:`**:
  Total size in bytes.
- **`min size:`**:
  Minimum allocation size.
- **`max size:`**:
  Maximum allocation size.
- **`max size path:`**:
  Call stack leading to the largest allocation.

`free_domheap_pages` are tracked and used in the computation but not reported in
the summary.

### Non-Freed Memory Reporting

The log includes summaries for non-freed memory:

- **`total non-freed <operation> size:`**:
  Total memory not freed.
- **`max size non-freed <operation> path:`**:
  Call stack for the largest non-freed allocation.

### Example: Non-Freed Report

```
[d0] total non-freed xmalloc size: 1840 B
[d0] max size non-freed xmalloc path:
  xmem_pool_alloc(size=1032) [domain: d0]
    _xmalloc
```

## Detailed Log Information

For each architecture and log file, the directory `<pipeline_id>/parsed`
contains detailed information on the allocations performed by each domain in
each test (parsed log file).
These files starts with similar information as the
[Evaluation Summary](#evaluation-summary), but clustered by domain.
The logs are then followed by details on the detected `xmalloc() paths without a
matching xfree()` and the `alloc_domheap_pages() paths without a matching free`.
Each of these sections contain a stack trace leading to an unmatched anonymous
allocation and possibly a comment detailing the context of the allocation, and
whether the stack trace has already been analyzed and deemed potentially safe
(or not).

### Comment Format

The unmatched anonymous allocation sections of the log files consist of comments
(prefixed with `//`) and indented stack traces.

### Format Overview

- **Comments**:
  Lines beginning with `//` provide human-readable annotations and include a
  `MINERVA:` tag with a status.

  - **Format:** `// MINERVA:
    <STATUS>` where `<STATUS>` can be:
    - `UNDECIDED`:
      Analysis is not yet fully analyzed
    - `PROB_SAFE`:
      Likely safe scenario based on known conditions.
  - Additional descriptive lines may follow the `MINERVA:` tag.

- **Domain**:
  The domain that has generated this stack trace.

- **Occurences**:
  Number of occurences of this stack trace (per domain).

- **Total size**:
  Sum of allocation sizes that the occurrences of this stack trace has caused.

- **Stack Traces**:
  Indented lines represent the call stack at the time of an event.

  - Each line shows a function call or code location.
  - Indentation reflects the call hierarchy (parent to child).

### Example: Stack Traces with Comments

```
// MINERVA: UNDECIDED
// Anonymous allocation increases per-CPU heap.
// Allocation not attributable to any domain.
Domain     : d0
Occurrences: 1
Total size : 128 B
return_to_new_vcpu64
  leave_hypervisor_to_guest
    traps.c#check_for_pcpu_work
      do_softirq
        softirq.c#__do_softirq
          timer.c#timer_softirq_action
            _xmalloc
              _xmalloc
```

In this example, a direct `_xmalloc` is triggered from an ISR.
No domain can be held accountable for such allocation and a different strategy
to ensure/motivate the boundness of such allocations should be planned.

```
// MINERVA: PROB_SAFE (under the assumption that hwdom are safe)
// the path is already attributed to a domain pool, except if the domain is
// a hardware domain, in this case the allocation is anonymous (p2m_alloc_page).
alloc_domheap_pages()
entry.o#guest_sync_slowpath
  do_trap_guest_sync
    traps.c#do_trap_hypercall
      do_memory_op
        xenmem_add_to_physmap
          xenmem_add_to_physmap_one
            p2m_set_entry
              p2m.c#__p2m_set_entry
                p2m.c#p2m_next_level
                  p2m.c#p2m_alloc_page
                    alloc_domheap_pages
                      alloc_domheap_pages
```

In this example, the allocation is marked as "probably safe" since the path is
taken only when the domain is a hardware domain.
Such domains can (under the usage-domain of interest) perform allocations.

### Example: Insufficient Information in Stack Trace

```
// MINERVA: UNDECIDED
// Not sufficient information in the stack trace.
Domain     : d0
Occurrences: 1
Total size : 392 B
_xzalloc
  _xmalloc
    _xmalloc
```

## Parser Scope and Limitations

The parser groups WARN-instrumented events into two per-pointer maps,
each tracking the same allocation/free family:

- `xmalloc` family--  alloc functions `xmem_pool_alloc`,
  `xmalloc_whole_pages`; free function `xfree`. Outstanding
  allocations live in the per-pointer deque routed through
  `xmalloc_paths` in `Log.parse`.
- `domheap` family--  alloc function `alloc_domheap_pages`; free
  functions `free_domheap_pages`, `free_xenheap_pages`. Outstanding
  allocations live in the per-pointer deque routed through
  `alloc_domheap_paths`.

Each WARN is routed by its function name. Pairing alloc with free is
done by pointer key inside one family only; the parser never moves
a free across families to match an alloc in the other.

### Cross-family allocation/free observations

A `_xmalloc(size)` of a whole-page-or-larger allocation passes
through `xmalloc_whole_pages  ->  alloc_xenheap_pages  -> 
alloc_domheap_pages(NULL, ...)`. With the current kernel
instrumentation, **both** `xmalloc_whole_pages` and
`alloc_domheap_pages` emit a WARN at the same returned pointer, so
the same logical allocation is recorded twice--  once in each
family's per-pointer map. The corresponding `xfree(p)` similarly
emits both an `xfree` WARN and a `free_xenheap_pages` WARN on the
sub-page chunks, so both maps see matching frees. Pairing succeeds
**within each family**; the parser does not need to cross families
to close these allocations.

In observed pipeline data (pipeline 1674833972, the reference
pipeline used by the surrounding analysis) the
`xmalloc_whole_pages` / `free_xenheap_pages` paths are not exercised
post-`SYS_STATE_active`, so no cross-family event is produced in
practice. A zero-cross-family-pair result on that pipeline is the
expected outcome of the current model.

### One-sided observations

Two cases can produce a one-sided alloc-or-free event with no
matching counterpart in the WARN stream:

- **Pre-`SYS_STATE_active` allocation, post-active free.** The WARN
  gate suppresses output until `system_state >= SYS_STATE_active`.
  An allocation made earlier (typically during boot, often in the
  Xen static-virtual address window such as `ffff82e0...` on
  x86_64) will not have an alloc WARN; if the corresponding free
  happens post-active, only the free WARN is observed. The parser
  reports this honestly as a `frees without matching alloc` summary
  line. It is **not** a cross-family bug, and the parser
  deliberately does not attempt to silently invent an alloc record
  for it.
- **Long-lived allocation, no free during the test window.** Many
  per-domain init-time allocations (event-channel fifo setup,
  ioreq-server creation, p2m table growth) are freed only at
  domain destroy, after the test ends and after WARN logging stops.
  These appear as outstanding allocations in the per-job parsed
  output and are the dominant population in the `non-freed` size
  totals.

### What a stronger cross-family or layered-WARN reconciliation would require

Pointer identity alone is not enough to deduplicate the two WARNs
emitted for one logical layered allocation
(`xmalloc_whole_pages  ->  alloc_domheap_pages`), nor to attribute a
one-sided free to a pre-active alloc. A WARN-side change would be
needed. The minimum useful additions:

- **Caller return address** in every WARN line (rather than
  reconstructing the call site from the post-WARN stack dump), so
  layered WARNs of the same logical operation can be correlated
  unambiguously.
- **Layered-allocation tag** on inner WARNs (for example,
  `alloc_domheap_pages` carrying a hint that it was invoked from
  `xmalloc_whole_pages`), so the parser can deduplicate inner and
  outer WARNs of the same logical alloc.
- **Memflags / allocator-kind** on `alloc_domheap_pages` WARNs, so
  the parser can distinguish deliberately-anonymous allocations
  (those using `MEMF_no_owner` or `MEMF_no_refcount`) from
  accidentally-anonymous ones.
- **Pre-active suppressed-allocation counter**, so the parser can
  explain orphan frees whose alloc predates the WARN window
  instead of treating them as anomalies.
- **Explicit per-CPU monotonic event ID** on every WARN, the
  strongest fix: enables deterministic correlation of layered WARNs
  across nested calls within the same logical operation.

Until one or more of these is added on the kernel side, the parser
remains a best-effort reconstructor from the existing WARN content,
and the within-family pairing rule described above is the
defensible default.

## Automated Generation of Comments

To avoid repeating the analysis of known stack traces at each pipeline
execution, a "database" of `<comment> + <stack trace>` can be built and used as
input to the `log_parser.py` script to automatically mark the matching stack
traces in the analysis of a pipeline.
An intial database of analyzed paths is provided in
`minerva_analysis/minerva.comments`.

### Example

`log_parser.py --comments-path minerva.comments <pipeline_id>` triggers the
automated marking of known stack traces in a newly analyzed `pipeline_id` and
also ***adds*** unknown stack traces to the `minerva.comments` database file.
