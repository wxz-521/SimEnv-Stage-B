#!/usr/bin/env python3
"""Detect the run44 defect class: a function-local name read on a line that
precedes its first assignment in the same function.

This is the exact shape of the UnboundLocalError that killed the coverage
explorer's rospy.Timer thread in run44: ``heading_tolerance`` was read inside
the doorway_entry diagnostic a few lines before it was assigned, so the whole
control loop died and cmd_vel stayed (0, 0) for the rest of the mission.

The comparison is per name and line ordered, so a loop target, a branch-local
assignment or a comprehension variable does not produce a false positive.
"""

import ast
import sys


def scan(path):
    with open(path) as handle:
        tree = ast.parse(handle.read(), path)
    problems = []
    for node in ast.walk(tree):
        if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        store_first = {}
        load_first = {}
        # Names bound by constructs whose binding is structural rather than a
        # plain assignment: these are never the run44 defect.
        excluded = set()
        # Parameters are pre-bound, so rebinding one is not a defect.
        arguments = node.args
        for group in (
            arguments.args,
            arguments.posonlyargs,
            arguments.kwonlyargs,
        ):
            for argument in group:
                excluded.add(argument.arg)
        if arguments.vararg:
            excluded.add(arguments.vararg.arg)
        if arguments.kwarg:
            excluded.add(arguments.kwarg.arg)
        for sub in ast.walk(node):
            if isinstance(sub, ast.ExceptHandler) and sub.name:
                excluded.add(sub.name)
            if isinstance(sub, ast.comprehension):
                for target in ast.walk(sub.target):
                    if isinstance(target, ast.Name):
                        excluded.add(target.id)
            if isinstance(sub, ast.withitem) and sub.optional_vars is not None:
                for target in ast.walk(sub.optional_vars):
                    if isinstance(target, ast.Name):
                        excluded.add(target.id)

        def collect(current):
            for child in ast.iter_child_nodes(current):
                if isinstance(
                    child,
                    (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda, ast.ClassDef),
                ):
                    continue  # nested scopes own their names
                if isinstance(child, ast.Name):
                    if isinstance(child.ctx, ast.Store):
                        target = store_first
                    elif isinstance(child.ctx, ast.Load):
                        target = load_first
                    else:
                        target = None
                    if (
                        target is not None
                        and child.id not in target
                        and child.id not in excluded
                    ):
                        target[child.id] = child.lineno
                collect(child)

        for statement in node.body:  # body only: no decorators, no defaults
            collect(statement)

        for name, used in sorted(load_first.items(), key=lambda item: item[1]):
            stored = store_first.get(name)
            if stored is not None and used < stored:
                problems.append((node.name, name, used, stored))
    return problems


def main(paths):
    total = 0
    for path in paths:
        for func, name, used, stored in scan(path):
            total += 1
            print(
                "READ-BEFORE-ASSIGN %s: %s(): '%s' read at line %d, "
                "first store at %d" % (path, func, name, used, stored)
            )
    if total:
        print("FAILED: %d suspicious read(s)" % total)
        return 1
    print("OK: no read-before-assign local in %d file(s)" % len(paths))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
