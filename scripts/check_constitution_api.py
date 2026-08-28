#!/usr/bin/env python3
"""Check parity between public constitution classes and Python bindings."""

from __future__ import annotations

import re
from collections import defaultdict
from collections.abc import Sequence
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PUBLIC_HEADERS = Path("include/uipc/constitution")
DEFAULT_BINDING_ROOT = Path("src/pybind/pyuipc")

PUBLIC_CLASS_RE = re.compile(
    r"\bclass\s+UIPC_(?:CONSTITUTION|CORE)_API\s+([A-Za-z_]\w*)"
)
PY_CLASS_RE = re.compile(
    r"py::class_<\s*(?P<types>[^>]+)>\s*\(\s*[^,]+,\s*"
    r'"(?P<python_name>[A-Za-z_]\w*)"',
    re.DOTALL,
)
INITIALIZER_DEFINITION_RE = re.compile(
    r"\b(Py[A-Za-z_]\w*)::\1\s*\(\s*py::module_?\s*&"
)
INITIALIZER_CALL_RE = re.compile(r"\b(Py[A-Za-z_]\w*)\s*\{[^{};]*\}\s*;")


def _source_files(root: Path, suffix: str) -> list[Path]:
    return sorted(root.rglob(f"*{suffix}"), key=lambda path: path.as_posix())


def collect_public_classes(project_root: Path = ROOT) -> dict[str, list[Path]]:
    classes: dict[str, list[Path]] = defaultdict(list)
    header_root = project_root / PUBLIC_HEADERS
    for path in _source_files(header_root, ".h"):
        for name in PUBLIC_CLASS_RE.findall(path.read_text(encoding="utf-8")):
            classes[name].append(path.relative_to(project_root))
    return dict(classes)


def collect_python_bindings(
    project_root: Path = ROOT,
    binding_root: Path = DEFAULT_BINDING_ROOT,
) -> dict[str, list[tuple[str, Path]]]:
    bindings: dict[str, list[tuple[str, Path]]] = defaultdict(list)
    source_root = project_root / binding_root / "constitution"
    for path in _source_files(source_root, ".cpp"):
        text = path.read_text(encoding="utf-8")
        for match in PY_CLASS_RE.finditer(text):
            cpp_type = match.group("types").split(",", maxsplit=1)[0].strip()
            cpp_name = cpp_type.removeprefix("::").rsplit("::", maxsplit=1)[-1]
            bindings[cpp_name].append(
                (match.group("python_name"), path.relative_to(project_root))
            )
    return dict(bindings)


def collect_initializer_definitions(
    project_root: Path = ROOT,
    binding_root: Path = DEFAULT_BINDING_ROOT,
) -> dict[str, list[Path]]:
    definitions: dict[str, list[Path]] = defaultdict(list)
    source_root = project_root / binding_root / "constitution"
    for path in _source_files(source_root, ".cpp"):
        text = path.read_text(encoding="utf-8")
        for name in INITIALIZER_DEFINITION_RE.findall(text):
            if name != "PyModule":
                definitions[name].append(path.relative_to(project_root))
    return dict(definitions)


def collect_initializer_calls(
    project_root: Path = ROOT,
    binding_root: Path = DEFAULT_BINDING_ROOT,
) -> set[str]:
    calls: set[str] = set()
    source_root = project_root / binding_root
    for path in _source_files(source_root, ".cpp"):
        calls.update(INITIALIZER_CALL_RE.findall(path.read_text(encoding="utf-8")))
    return calls


def check_constitution_api(
    project_root: Path = ROOT,
    binding_root: Path = DEFAULT_BINDING_ROOT,
) -> list[str]:
    """Return deterministic source-level API parity errors."""

    exported = collect_public_classes(project_root)
    bindings = collect_python_bindings(project_root, binding_root)
    definitions = collect_initializer_definitions(project_root, binding_root)
    calls = collect_initializer_calls(project_root, binding_root)
    errors: list[str] = []

    for name, paths in sorted(exported.items()):
        if len(paths) > 1:
            locations = ", ".join(path.as_posix() for path in paths)
            errors.append(
                f"public class {name} is declared more than once: {locations}"
            )

    for name, entries in sorted(bindings.items()):
        if len(entries) > 1:
            locations = ", ".join(path.as_posix() for _, path in entries)
            errors.append(f"C++ class {name} is bound more than once: {locations}")
        for python_name, path in entries:
            if python_name != name:
                errors.append(
                    f"{path.as_posix()}: {name} is exposed as {python_name}; "
                    "constitution class names must stay aligned"
                )

    missing_bindings = sorted(set(exported) - set(bindings))
    extra_bindings = sorted(set(bindings) - set(exported))
    for name in missing_bindings:
        errors.append(f"public constitution class {name} has no Python binding")
    for name in extra_bindings:
        errors.append(
            f"Python binding {name} has no exported public constitution class"
        )

    for name, paths in sorted(definitions.items()):
        if len(paths) > 1:
            locations = ", ".join(path.as_posix() for path in paths)
            errors.append(
                f"binding initializer {name} is defined more than once: {locations}"
            )
        if name not in calls:
            errors.append(
                f"binding initializer {name} is never registered in a binding module"
            )

    return errors


def main(argv: Sequence[str] | None = None) -> int:
    del argv
    binding_roots = [DEFAULT_BINDING_ROOT, Path("src/nanobind/pyuipc")]
    exported = collect_public_classes()
    for binding_root in binding_roots:
        errors = check_constitution_api(ROOT, binding_root)
        if errors:
            print(f"Constitution API parity check failed for {binding_root}:")
            for error in errors:
                print(f"- {error}")
            return 1

        initializers = collect_initializer_definitions(ROOT, binding_root)
        print(
            f"Constitution API parity check passed for {binding_root}: "
            f"{len(exported)} public classes, {len(initializers)} binding initializers."
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
