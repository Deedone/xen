# Parser tools

`minerva_analysis/log_parser.py` parses Xen runtime test logs
into call-path summaries and applies a `minerva.comments`
database that annotates known paths. The parser emits
deterministic, diff-friendly output suitable for CI
consumption.

## Inputs and outputs

```
log_parser.py <log-dir> [--comments-path <path>]
              [--out-dir <path>] [--legacy-null-is-d0]
```

- **`<log-dir>`**--  directory of `.test.log` files produced by
  the Xen test harness.
- **`--comments-path`**--  path to the `minerva.comments` file
  used to annotate known call paths. Defaults to the one
  bundled under `minerva_analysis/`.
- **`--out-dir`**--  output directory; defaults to alongside
  the log directory.
- **`--legacy-null-is-d0`**--  opt-in to the historical
  behaviour of rewriting a NULL allocator-domain argument to
  `d0`. Off by default; see "NULL domain handling" below.

## NULL domain handling

The parser previously rewrote a NULL `struct domain *`
argument in allocator call paths to `d0` (the hardware
domain). That behaviour was always wrong in the strict
sense--  NULL means "no domain context", which is distinct
from "domain zero"--  and concealed real path variants when
two allocations differed only by NULL-vs-d0 at the
allocator boundary.

The default is now to preserve NULL as NULL. The
`--legacy-null-is-d0` flag restores the previous rewrite for
compatibility with consumers that already encoded the older
form into their comments database.

## Head function in CallPath identity

The CallPath identity used to deduplicate parsed paths
includes the head function name. Two paths whose tail
sequences match but whose head functions differ are
distinct CallPath entries. This avoids collapsing paths that
the analyst intentionally distinguishes by entry point
(e.g. paths originating in different syscall handlers but
funnelling through a common allocator helper).

## Multiple outstanding allocations per pointer

The parser tracks multiple outstanding allocations per
allocated pointer rather than the latest one. A free that
matches an earlier outstanding allocation correctly retires
that allocation instead of being attributed to the most
recent allocation event for the same pointer.

This affects long-running runtime traces where a pointer is
allocated, freed, and reallocated within the same log: the
allocation accounting now reflects the full event stream
rather than the last-write-wins approximation.

## Cross-family free accounting limits

Frees that cross allocation families (for example, an
xenheap free retiring a domheap allocation) are recorded
literally--  the parser does not reconcile such pairings.
The output preserves the cross-family pair for analyst
review; downstream consumers should not assume the free
implies the allocation it appears to retire.

## Comments database

`minerva.comments` is a plain-text database mapping call-path
signatures to analyst annotations. The parser applies it
during summary generation so each call path is rendered with
its operator-supplied verdict and notes.

### Comments format

- The header is anchored at the start of the file; the
  parser rejects a malformed header rather than silently
  treating the first body line as configuration.
- Body lines that fail to parse are reported as warnings
  along with the file and line number; the parser does not
  silently skip them.
- Output produced after applying the comments database is
  emitted in a stable, deterministic order. Two runs over
  the same inputs produce byte-identical output, which is a
  prerequisite for diff-friendly CI artifacts.

## Callpath traversal

`minerva_static_analysis/callpath.py` walks a GCC `.ci`
callgraph and prints paths between caller and target
functions. The traversal continues past recursive edges
during sibling iteration so a recursive call does not
prematurely terminate the search for unrelated siblings of
the recursion site.

## Syntax checks

```
python3 -m py_compile minerva_analysis/log_parser.py
python3 -m py_compile minerva_static_analysis/callpath.py
```

`log_parser.py --help` and `callpath.py --help` document the
current flag set.
