#!/usr/bin/env python3
"""Catch ``self.foo`` on a class that never defines ``foo``.

A run costs about two hours of wall time, so a typo like
``self.stop_distance`` (an elevator-node parameter) inside the explorer's
control loop is expensive even when it only costs a few thousand guarded
exceptions.  This walks each class, collects every attribute assigned anywhere
in it, and reports attributes that are only ever read.
"""

import ast
import sys


def scan(path):
    tree = ast.parse(open(path).read(), path)
    problems = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.ClassDef):
            continue
        assigned = set()
        for sub in ast.walk(node):
            if isinstance(sub, ast.Attribute) and isinstance(sub.ctx, ast.Store):
                if isinstance(sub.value, ast.Name) and sub.value.id == "self":
                    assigned.add(sub.attr)
            # Methods are attributes of the class too.
            if isinstance(sub, (ast.FunctionDef, ast.AsyncFunctionDef)):
                assigned.add(sub.name)
            # Class-level constants and dataclass field annotations count too.
            if isinstance(sub, (ast.Assign, ast.AnnAssign)):
                targets = sub.targets if isinstance(sub, ast.Assign) else [sub.target]
                for target in targets:
                    if isinstance(target, ast.Name):
                        assigned.add(target.id)
        # attributes assigned on any object that is not self are irrelevant
        read = {}
        for sub in ast.walk(node):
            if isinstance(sub, ast.Attribute) and isinstance(sub.ctx, ast.Load):
                if isinstance(sub.value, ast.Name) and sub.value.id == "self":
                    read.setdefault(sub.attr, sub.lineno)
        for name, line in sorted(read.items(), key=lambda item: item[1]):
            if name not in assigned:
                problems.append((node.name, name, line))
    return problems


def main(paths):
    total = 0
    for path in paths:
        for cls, name, line in scan(path):
            total += 1
            print(
                "UNDEFINED-ATTR %s: %s.self.%s read at line %d"
                % (path, cls, name, line)
            )
    if total:
        print("FAILED: %d undefined attribute read(s)" % total)
        return 1
    print("OK: no undefined self attribute in %d file(s)" % len(paths))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
