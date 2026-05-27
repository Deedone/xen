#!/usr/bin/env python3

import itertools
import re
from argparse import ArgumentParser
from collections import defaultdict, deque
from contextlib import redirect_stdout
from pathlib import Path
from typing import Callable, DefaultDict, Iterable, Iterator, List, Tuple, TypeVar, overload

try:
    import gitlab
except ImportError:
    pass

DEFAULT_XEN_PROJECT_ID = "22438264"
arch_pattern = re.compile("(x86(_|-)64|arm64|arm32|riscv64|ppc64)")
xen_info = r"\(XEN\) \[.+\]"
debug_pattern = re.compile(rf"{xen_info} DEBUG: (?P<domain>d(\d+|\[IDLE\])|NULL): "
                           r"(?P<function>\w+) - (?P<params>.+)")
param_pattern = re.compile(r"(?P<param>\w+)(=| )(?P<value>\S+)")
trace_pattern = re.compile(rf"{xen_info} Xen call trace:")
call_pattern = re.compile(rf"{xen_info}\s+\[.+\] (. )?(?P<function>[\w\-\/#\.]+)\+.+")
header_pattern = re.compile(r"(?P<function>\w+)\(\)")

T = TypeVar("T")
R = TypeVar("R")

@overload
def groupby(iterable: Iterable[T], key: None = None) -> Iterator[Tuple[T, Iterator[T]]]: ...
@overload
def groupby(iterable: Iterable[T], key: Callable[[T], R]) -> Iterator[Tuple[R, Iterator[T]]]: ...
def groupby(iterable: Iterable[T], key: Callable[[T], R] | None = None) -> Iterator[Tuple[R, Iterator[T]]]:
    return itertools.groupby(sorted(iterable, key=key), key=key) # type: ignore

class CallPath:
    def __init__(self, function: str, params: str, domain: str):
        self.function = function
        self.params = {match.group("param"): match.group("value")
                       for match in param_pattern.finditer(params)}
        self.domain = domain
        self.path: deque[str] = deque()

    def __eq__(self, other: object):
        return (isinstance(other, CallPath) and self.path == other.path)

    def __hash__(self):
        return hash(tuple(self.path))

    def __lt__(self, other: 'CallPath'):
        return len(self.path) < len(other.path)

    def __repr__(self):
        size = f"(size={self.params.get('size')})" if "size" in self.params else ""
        domain = f" [domain: {self.domain}]" if self.domain else ""
        out = f"{self.function}{size}{domain}\n"
        return out + self.path_str

    @property
    def size(self):
        return int(self.params["size"])

    @property
    def path_str(self):
        if len(self.path) == 0:
            return "N/A"
        else:
            return "\n".join(f"{'  ' * i}{fn}" for i, fn in enumerate(reversed(self.path)))

    def appendleft(self, function: str):
        self.path.appendleft(function)

    def append(self, function: str):
        self.path.append(function)

    def indented(self, spaces: int):
        indent = " " * spaces
        return indent + str(self).replace("\n", "\n" + indent)

Paths = DefaultDict[str, DefaultDict[str, List[CallPath]]]

class Log:
    def __init__(self, file_path: Path, paths: Paths,
                 xmalloc_paths: dict[str, list[CallPath]],
                 unmatched_xfree_paths: list[CallPath],
                 alloc_domheap_paths: dict[str, list[CallPath]],
                 unmatched_free_heap_paths: list[CallPath]):
        self.file_path = file_path
        self.paths = paths
        self.xmalloc_paths = xmalloc_paths
        self.alloc_domheap_paths = alloc_domheap_paths
        self.unmatched_xfree_paths = unmatched_xfree_paths
        self.unmatched_free_heap_paths = unmatched_free_heap_paths
        match = arch_pattern.search(file_path.name)
        self.arch = match.group().replace("-", "_") if match else ""

    @classmethod
    def parse(cls, log_path: Path, exclude: list[str], skip_domain: bool = False):
        with open(log_path, "r") as file:
            log_text = file.read()

        in_trace = False
        path: CallPath | None = None
        paths: Paths = defaultdict(lambda: defaultdict(list))
        xmalloc_paths: dict[str, CallPath] = {}
        alloc_domheap_paths: dict[str, CallPath] = {}
        unmatched_xfree_paths: list[CallPath] = []
        unmatched_free_heap_paths: list[CallPath] = []
        alloc_paths = {"xmem_pool_alloc": xmalloc_paths,
                       "xmalloc_whole_pages": xmalloc_paths,
                       "alloc_domheap_pages": alloc_domheap_paths}
        free_info = {"xfree": (xmalloc_paths, unmatched_xfree_paths),
                     "free_domheap_pages": (alloc_domheap_paths, unmatched_free_heap_paths),
                     "free_xenheap_pages": (alloc_domheap_paths, unmatched_free_heap_paths)}

        for line in log_text.splitlines():
            if match := debug_pattern.search(line):
                function = match.group("function")
                params = match.group("params")
                domain = match.group("domain") if not skip_domain else ""
                if domain == "NULL":
                    domain = "d0"
                path = CallPath(function, params, domain)
                paths[domain][function].append(path)
                if function in alloc_paths:
                    alloc_paths[function][path.params["p"]] = path
                elif function in free_info:
                    free_alloc_paths, unmatched_free_paths = free_info[function]
                    if path.params["p"] in free_alloc_paths:
                        path.params["size"] = free_alloc_paths.pop(path.params["p"]).params["size"]
                    else:
                        path.params["size"] = 0
                        unmatched_free_paths.append(path)
            elif in_trace and path:
                if match := call_pattern.search(line):
                    function = match.group("function")
                    if function in exclude:
                        del paths[path.domain][path.function][-1]
                        in_trace = False
                        path = None
                    else:
                        path.append(function)
                else:
                    in_trace = False
            elif trace_pattern.search(line):
                in_trace = True

        def groupby_domain(iterable: Iterable[CallPath]):
            return {k: list(g) for k, g in groupby(iterable, lambda path: path.domain)}

        return cls(log_path, paths, groupby_domain(xmalloc_paths.values()), unmatched_xfree_paths,
                   groupby_domain(alloc_domheap_paths.values()), unmatched_free_heap_paths)

    @staticmethod
    def print_total_size(paths: dict[str, list[CallPath]], domain: str, function: str):
        domain_paths = paths.get(domain, [])
        if not domain_paths:
            return
        total_size = sum(path.size for path in domain_paths)
        max_path = max(domain_paths, key=lambda path: path.size)
        print(f"  [{domain}] total non-freed {function} size: {total_size} B")
        print(f"  [{domain}] max size non-freed {function} path:\n{max_path.indented(4)}")

    def print_stats(self):
        for i, domain in enumerate(sorted(self.paths)):
            for function in sorted(self.paths[domain]):
                fn_paths = [path for path in self.paths[domain][function]]
                if not fn_paths:
                    continue
                values = [path.size for path in fn_paths]
                max_path = max(fn_paths, key=lambda path: path.size)
                print(f"  [{domain}] {function}:")
                print(f"    sizes (B): {values}")
                print(f"    num calls: {len(values)}")
                # NOTE: sum size: sum of all requested allocations also includes allocations that
                #       are later freed
                print(f"    sum size:  {sum(values)} B")
                print(f"    min size:  {min(values)} B")
                print(f"    max size:  {max_path.params['size']} B")
                print(f"    max size path:\n{max_path.indented(6)}")

            self.print_total_size(self.xmalloc_paths, domain, "xmalloc")
            self.print_total_size(self.alloc_domheap_paths, domain, "alloc_domheap_pages")
            print()
            if i < len(self.paths) - 1:
                print("  " + "-" * 58 + "\n")

    @staticmethod
    def print_grouped_paths(paths: dict[str, list[CallPath]], comments: dict[CallPath, str]):
        for domain_paths in paths.values():
            groups: dict[CallPath, list[CallPath]] = {}
            total_sizes: dict[CallPath, int] = {}
            for path, group in groupby(domain_paths):
                group_list = list(group)
                groups[path] = group_list
                total_sizes[path] = sum(p.size for p in group_list)
            for path in sorted(groups, key=lambda path: total_sizes[path], reverse=True):
                print(comments[path], end="")
                print(f"Domain     : {path.domain}")
                print(f"Occurrences: {len(groups[path])}")
                print(f"Total size : {total_sizes[path]} B")
                print(path.path_str)

    def print_paths(self, comments: dict[CallPath, str], verbose: bool = False):
        print("############################################################")
        print("        xmalloc() paths without a matching xfree()")
        print("------------------------------------------------------------")
        print("Each path is considered only once (per domain)")
        print("Paths are sorted by descending allocation size")
        print("############################################################\n")
        self.print_grouped_paths(self.xmalloc_paths, comments)
        print()
        print("############################################################")
        print("    alloc_domheap_pages() paths without a matching free")
        print("------------------------------------------------------------")
        print("Each path is considered only once (per domain)")
        print("Paths are sorted by descending allocation size")
        print("############################################################\n")
        self.print_grouped_paths(self.alloc_domheap_paths, comments)

        if not verbose:
            return

        # Only verbose results

        print()
        print("############################################################")
        print("        xfree() paths without a matching allocation")
        print("############################################################\n")
        for xfree_path in self.unmatched_xfree_paths:
            print(xfree_path)

    def print_paths_full(self):
        print("############################################################")
        print("                      xmalloc() paths")
        print("############################################################\n")
        for domain in sorted(self.paths):
            for function in self.paths[domain]:
                if function != "xmem_pool_alloc":
                    continue
                for path in self.paths[domain][function]:
                    print(path)
        print()
        print("############################################################")
        print("               alloc_domheap_pages() paths")
        print("############################################################\n")
        for domain in sorted(self.paths):
            for function in self.paths[domain]:
                if function != "alloc_domheap_pages":
                    continue
                for path in self.paths[domain][function]:
                    print(path)

def download_job_logs(project_id: str, pipeline_id: str):
    CONFIG_SECTION = "gitlab.com"

    try:
        gl = gitlab.Gitlab.from_config(CONFIG_SECTION) # type: ignore
    except NameError:
        print("Install python-gitlab")
        exit(1)

    project = gl.projects.get(project_id)
    pipeline = project.pipelines.get(pipeline_id)
    pipeline_dir = Path(pipeline_id)
    pipeline_dir.mkdir(exist_ok=True)
    jobs = pipeline.jobs.list(get_all=True)
    for job in jobs:
        name: str = job.attributes["name"]
        if job.attributes["stage"] != "test" or name == "build-each-commit-gcc":
            continue
        print(f"Downloading job {name}")
        log = project.jobs.get(job.attributes["id"]).trace()
        path = pipeline_dir / name
        with open(path, "w") as file:
            file.write(log.decode() if isinstance(log, bytes) else "")

def update_comments(log_paths: list[Path], comments_path: Path, exclude: list[str]):
    if comments_path.exists():
        print("Comments file found")
        comments = parse_comments(comments_path)
    else:
        print(f"Comments file not found. An empty one is being created at {comments_path}")
        comments: dict[CallPath, str] = {}
    for log_path in log_paths:
        print(f"Extracing unique paths from job {log_path.name}")
        log = Log.parse(log_path, exclude, skip_domain=True)
        for domain_paths in log.xmalloc_paths.values():
            for path in set(domain_paths):
                comments.setdefault(path, "")
        for domain_paths in log.alloc_domheap_paths.values():
            for path in set(domain_paths):
                comments.setdefault(path, "")
        with open(comments_path, "w") as file, redirect_stdout(file):
            for path, comment in comments.items():
                print(comment, end="")
                print(f"{path.function}()")
                print(path.path_str)
    return comments

def parse_comments(comments_path: Path):
    comments: dict[CallPath, str] = {}
    with open(comments_path, "r") as file:
        path = None
        comment = ""
        for line in file:
            line = line.strip()
            if line.startswith("//"):
                if path:
                    comments[path] = comment
                    comment = ""
                    path = None
                comment += line + "\n"
            elif match := header_pattern.match(line):
                if path:
                    comments[path] = comment
                    comment = ""
                    path = None
                path = CallPath(match.group("function"), "", "")
            elif path and line != "N/A":
                path.appendleft(line)
    if path:
        comments[path] = comment
    return comments

def parse_pipeline(log_paths: list[Path], comments: dict[CallPath, str], output_dir: Path,
                   verbose: bool, exclude: list[str]):
    logs: DefaultDict[str, list[Log]] = defaultdict(list)
    for log_path in log_paths:
        print(f"Parsing job {log_path.name}")
        log = Log.parse(log_path, exclude)
        logs[log.arch].append(log)
        arch_dir = output_dir / log.arch
        arch_dir.mkdir(exist_ok=True)
        with open(arch_dir / log_path.name, "w") as file, redirect_stdout(file):
            log.print_stats()
            log.print_paths(comments, verbose)
        verbose_dir = arch_dir / "verbose"
        verbose_dir.mkdir(exist_ok=True)
        with open(verbose_dir / log_path.name, "w") as file, redirect_stdout(file):
            log.print_paths_full()
    print()
    for arch in sorted(logs):
        for log in sorted(logs[arch], key=lambda log: log.file_path.stem):
            print(f"[LOG: {log.file_path}]")
            print()
            log.print_stats()
            print("-" * 60)

def run(pipeline_id: str, project_id: str, force_download: bool, verbose: bool, exclude: list[str],
        comments_path: Path | None = None):
    pipeline_dir = Path(pipeline_id)
    if not pipeline_dir.exists() or force_download:
        download_job_logs(project_id, pipeline_id)
    else:
        print("Pipeline dir already exists. Use --force-download to force the download of the logs")
    log_paths = [file for file in pipeline_dir.glob("*") if file.is_file()]
    parsed_dir = pipeline_dir/ "parsed"
    parsed_dir.mkdir(exist_ok=True)
    if not comments_path:
        comments_path = parsed_dir / "comments"
    comments = update_comments(log_paths, comments_path, exclude)
    parse_pipeline(log_paths, comments, parsed_dir, verbose, exclude)

if __name__ == "__main__":
    parser = ArgumentParser("log_parser", description="Parse Xen Gitlab CI job log")
    parser.add_argument("pipeline_id", help="Pipeline id to download or to parse if it has already "
                        "been downloaded into <pipeline_id>/ directory")
    parser.add_argument("--project-id", default=DEFAULT_XEN_PROJECT_ID,
                        help="Fetch project id instead of default 22438264")
    parser.add_argument("--force-download", action="store_true", help="Force download of the logs")
    parser.add_argument("--verbose", action="store_true", help="Verbose path printing")
    parser.add_argument("-e", "--exclude", nargs="+", metavar="FUNCTION", default=[],
                        help="Set of functions to be excluded from parsing")
    parser.add_argument("--comments-path", type=Path)
    args = parser.parse_args()
    run(**vars(args))
