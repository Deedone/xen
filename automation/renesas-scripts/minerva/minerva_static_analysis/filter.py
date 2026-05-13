#!/usr/bin/env python3

from argparse import ArgumentParser
from typing import Iterable, TypeVar

T = TypeVar("T")
class OrderedSet(dict[T, None]):
    def __init__(self, iterable: Iterable[T] | None = None):
        return super().__init__(dict.fromkeys(iterable or []))

    def add(self, value: T):
        self[value] = None

    def __repr__(self):
        return f"{{{', '.join(repr(x) for x in self)}}}"

def parse_paths(filepath: str):
    paths: list[OrderedSet[str]] = []
    path: OrderedSet[str] = OrderedSet()
    with open(filepath, "r") as file:
        for line in file:
            if not line.startswith(" "):
                if path:
                    paths.append(path)
                path = OrderedSet()
            path.add(line.strip())
    return paths

def read_functions(path: str) -> list[str]:
    if not path:
        return []
    with open(path, "r") as file:
        return [fn.strip() for fn in file]

def remove_paths_containing(paths: list[OrderedSet[str]], functions: set[str]):
    return [path for path in paths if len(functions.intersection(path)) == 0]

def remove_paths_not_containing(paths: list[OrderedSet[str]],
                                functions: set[str]):
    return [path for path in paths if len(functions.intersection(path)) != 0]

def print_path(path: OrderedSet[str]):
    for i, function in enumerate(path):
        print(f"{'  ' * i}{function}")

def filter_paths(callpath_file: str, include_file: str, exclude_file: str,
                 include: list[str], exclude: list[str]):
    paths = parse_paths(callpath_file)
    included_functions = set(read_functions(include_file) + include)
    excluded_functions = set(read_functions(exclude_file) + exclude)
    if included_functions:
        paths = remove_paths_not_containing(paths, included_functions)
    if excluded_functions:
        paths = remove_paths_containing(paths, excluded_functions)
    for path in paths:
        print_path(path)

if __name__ == "__main__":
    parser = ArgumentParser("filter", description="Filter callpath files both "
                                                  "with whitelist and "
                                                  "blacklist methods")
    parser.add_argument("callpath_file")
    parser.add_argument("-I", "--include-file", metavar="FILE", default="",
                        help="Specify a file that lists functions (one for "
                             "each line) that must appear in the paths")
    parser.add_argument("-E", "--exclude-file", metavar="FILE", default="",
                        help="Specify a file that lists functions (one for "
                             "each line) that must not appear in the paths")
    parser.add_argument("-i", "--include", nargs="+", metavar="FUNCTION",
                        default=[], help="List of functions that must appear "
                                         "in the paths. If -I is used, they "
                                         "are added to the ones from the file.")
    parser.add_argument("-e", "--exclude", nargs="+", metavar="FUNCTION",
                        default=[], help="List of functions that must not "
                                         "appear in the paths. If -E is used, "
                                         "they are added to the ones from the "
                                         "file.")
    args = parser.parse_args()
    filter_paths(**vars(args))
