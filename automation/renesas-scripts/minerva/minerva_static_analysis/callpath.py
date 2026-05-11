#!/usr/bin/python3

import argparse
import asyncio
from dataclasses import dataclass
import fileinput
import json
import re
from collections import defaultdict, deque
from pathlib import Path
from typing import Generator, Iterable, Literal, TypeVar

node_pattern = re.compile(
    r"node: { title: \".+?\" label: \"(?P<name>\w+)\\n(?P<location>.+?:\d+:\d+)"
    r"\\n(?P<stack_usage>\d+) bytes \(.*\)\" }")
edge_pattern = re.compile(
    r"edge: { sourcename: \"(.+?:)?(?P<caller>.+?)\" targetname: \"(.+?:)?(?P<called>.+?)\" "
    r"label: \"(?P<location>.+?)\" }")

T = TypeVar("T")
class OrderedSet(dict[T, None]):
    def __init__(self, iterable: Iterable[T] | None = None):
        super().__init__(dict.fromkeys(iterable or []))

    def add(self, value: T):
        self[value] = None

    def __repr__(self):
        return f"{{{', '.join(repr(x) for x in self)}}}"

@dataclass(frozen=True)
class Call:
    function: 'Function'
    location: str

    def __repr__(self) -> str:
        location_str = self.location and f" @ {self.location}"
        return f"{self.function.name}{location_str}"

class Function:
    def __init__(self, name: str = "", location: str = "", stack_usage: int | str = 0):
        self.init(name, location, stack_usage)
        self.calls: OrderedSet[Call] = OrderedSet()
        self.called_by: OrderedSet[Call] = OrderedSet()

    def init(self, name: str, location: str, stack_usage: int | str):
        self.name = name
        self.location = location
        self.stack_usage = int(stack_usage)

    def __repr__(self) -> str:
        return f"{self.name} @ {self.location} [{self.stack_usage} B]"

    def add_call(self, function: 'Function', location: str):
        self.calls.add(Call(function, location))
        function.called_by.add(Call(self, location))

    def paths_from(self, length: int = -1, recursion: bool = False,
                   excluded_functions: set[str] = set(),
                   path: list[Call] | None = None) -> Generator[list[Call], list[Call], None]:
        path = path or [Call(self, self.location)]

        if len(self.calls) == 0 or length == 0:
            yield path
            return

        for call in self.calls:
            if not recursion and call in path:
                continue
            if call.function.name in excluded_functions:
                continue
            path.append(call)
            yield from call.function.paths_from(length - 1, recursion, excluded_functions, path)
            path.pop()

    def paths_to(self, length: int = -1, recursion: bool = False,
                 excluded_functions: set[str] = set(),
                 path: deque[Call] | None = None) -> Generator[deque[Call], deque[Call], None]:
        path = path or deque([Call(self, self.location)])

        if len(self.called_by) == 0 or length == 0:
            yield path
            return

        for call in self.called_by:
            if not recursion and call in path:
                continue
            if call.function.name in excluded_functions:
                continue
            path.appendleft(call)
            yield from call.function.paths_to(length - 1, recursion, excluded_functions, path)
            path.popleft()

@dataclass(frozen=True)
class Command:
    cmd: list[str]
    message: str
    stdout: int | None
    stderr: int | None
    cwd: str | None

async def run(command: Command, semaphore: asyncio.Semaphore):
    async with semaphore:
        if command.message:
            print(command.message)
        process = await asyncio.create_subprocess_exec(*command.cmd,
                                                       stdout=command.stdout,
                                                       stderr=command.stderr,
                                                       cwd=command.cwd)
        stdout, stderr = await process.communicate()
        stdout = stdout and stdout.decode()
        stderr = stderr and stderr.decode()
        return (command, stdout, stderr)

async def update_parallel(commands: list[Command], jobs: int):
    semaphore = asyncio.Semaphore(jobs)
    coroutines = (run(command, semaphore) for command in commands)
    await asyncio.gather(*coroutines)

def update(project_path: str, jobs: int, **_):
    commands: list[Command] = []
    with open(Path(project_path) / "compile_commands.json", "r") as file:
        compile_commands = json.load(file)
    commands = [Command(cc["arguments"] + ["-fcallgraph-info=su"],
                        f"Generating {Path(cc['file']).stem}.ci",
                        None, asyncio.subprocess.PIPE, cc["directory"])
                for cc in compile_commands]
    asyncio.run(update_parallel(commands, jobs))

def parse(path: str, ignore_call_location: bool):
    functions: defaultdict[str, Function] = defaultdict(Function)
    for line in fileinput.input(Path(path).rglob("*.ci")):
        if match := node_pattern.match(line):
            groups = match.groupdict()
            function = functions[groups["name"]]
            function.init(**groups)
        elif match := edge_pattern.match(line):
            groups = match.groupdict()
            caller = functions[groups["caller"]]
            called = functions[groups["called"]]
            called.name = groups["called"]
            location = "" if ignore_call_location else groups["location"]
            caller.add_call(called, location)
    return dict(functions)

def print_path(path: Iterable[Call], stack: bool):
    stack_usage = 0
    for i, call in enumerate(path):
        stack_usage += call.function.stack_usage
        stack_usage_str = f" [{stack_usage} B]" * stack
        print(f"{'  ' * i}{call}{stack_usage_str}")

def print_paths(direction: Literal["from", "to"], project_path: str, function: str,
                exclude_file: str, ignore_call_location: bool, length: int, recursion: bool,
                stack: bool, **_):
    if recursion and length < 0:
        print("To include recursive calls you must set a positive max length")
        exit(1)

    excluded_functions: set[str] = set()
    if exclude_file:
        with open(exclude_file, "r") as file:
            excluded_functions = set(fn.strip() for fn in file)

    fn = parse(project_path, ignore_call_location)[function]
    if direction == "from":
        paths = fn.paths_from(length, recursion, excluded_functions)
    else:
        paths = fn.paths_to(length, recursion, excluded_functions)
    for path in paths:
        print_path(path, stack)

def main():
    parser = argparse.ArgumentParser()
    project_path_parser = argparse.ArgumentParser(add_help=False)
    project_path_parser.add_argument("project_path", help="path where .ci files are placed")
    cmd_parser = parser.add_subparsers(title="commands", required=True)
    update_parser = cmd_parser.add_parser("update", help="generate callpath info files",
                                          parents=[project_path_parser])
    update_parser.set_defaults(command=update)
    update_parser.add_argument("-j", "--jobs", help="number of jobs to use during build",
                               type=int, default=4)
    common = argparse.ArgumentParser(add_help=False, parents=[project_path_parser])
    common.add_argument("function", help="starting or ending function")
    common.add_argument("-e", "--exclude-file",
                        help="paths that contains functions listed in this file will be excluded")
    common.add_argument("-i", "--ignore-call-location",
                        help="ignore call location (line and column)", action="store_true")
    common.add_argument("-l", "--length", help="paths length", type=int, default=-1)
    common.add_argument("-r", "--recursion", help="print also recursive paths",
                        action="store_true")
    common.add_argument("-s", "--stack", help="print stack usage", action="store_true")
    common.set_defaults(command=print_paths)
    from_parser = cmd_parser.add_parser("from", help="call paths from function", parents=[common])
    from_parser.set_defaults(direction="from")
    to_parser = cmd_parser.add_parser("to", help="call paths to function", parents=[common])
    to_parser.set_defaults(direction="to")
    args = parser.parse_args()
    args.command(**vars(args))

if __name__ == "__main__":
    main()
