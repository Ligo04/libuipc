#!/usr/bin/env python3
"""Compare XMake locks while isolating explicitly host-probed tool versions."""

from __future__ import annotations

import argparse
import difflib
import re
from collections import Counter
from pathlib import Path


HOST_VERSION_PACKAGES = frozenset({"cmake", "libffi", "ninja", "openssl"})

_PACKAGE_HEADER = re.compile(r'^        \["(?P<requirement>.+)#[0-9a-f]+"\] = \{$')
_VERSION = re.compile(r'^            version = "[^"]+"$')
_ENTRY_ENDS = frozenset({"        },", "        }"})


def _requirement_name(requirement: str) -> str:
    """Return the package name before version constraints or configs."""
    return re.split(r"[\s\[]", requirement, maxsplit=1)[0]


def _normalize_lock_text(text: str) -> tuple[str, Counter[str]]:
    normalized: list[str] = []
    manifest: Counter[str] = Counter()
    current_host_package: str | None = None
    current_version_count = 0

    for line_number, line in enumerate(text.splitlines(keepends=True), start=1):
        content = line.rstrip("\r\n")
        newline = line[len(content) :]
        header = _PACKAGE_HEADER.fullmatch(content)
        if header:
            if current_host_package is not None:
                raise RuntimeError(
                    "malformed XMake lock: package entry was not closed before "
                    f"line {line_number}"
                )
            package_name = _requirement_name(header.group("requirement"))
            if package_name in HOST_VERSION_PACKAGES:
                current_host_package = package_name
                current_version_count = 0
                manifest[package_name] += 1

        if current_host_package is not None and _VERSION.fullmatch(content):
            current_version_count += 1
            line = '            version = "<host-version>"' + newline

        normalized.append(line)

        if content in _ENTRY_ENDS and current_host_package is not None:
            if current_version_count != 1:
                raise RuntimeError(
                    "malformed XMake lock: expected exactly one version for "
                    f"host package {current_host_package!r}, found "
                    f"{current_version_count}"
                )
            current_host_package = None
            current_version_count = 0

    if current_host_package is not None:
        raise RuntimeError(
            "malformed XMake lock: unterminated host package entry "
            f"{current_host_package!r}"
        )

    actual_packages = frozenset(manifest)
    if actual_packages != HOST_VERSION_PACKAGES:
        raise RuntimeError(
            "host-version package manifest mismatch: "
            f"missing={sorted(HOST_VERSION_PACKAGES - actual_packages)!r}, "
            f"unexpected={sorted(actual_packages - HOST_VERSION_PACKAGES)!r}"
        )
    return "".join(normalized), manifest


def compare_lock_files(reference: Path, candidate: Path) -> None:
    """Require portable lock content to match across two generated locks."""
    reference_text, reference_manifest = _normalize_lock_text(
        reference.read_text(encoding="utf-8")
    )
    candidate_text, candidate_manifest = _normalize_lock_text(
        candidate.read_text(encoding="utf-8")
    )
    if reference_manifest != candidate_manifest:
        raise RuntimeError(
            "host-version package manifest count differs: "
            f"reference={dict(reference_manifest)!r}, "
            f"candidate={dict(candidate_manifest)!r}"
        )
    if reference_text == candidate_text:
        return

    diff = "".join(
        difflib.unified_diff(
            reference_text.splitlines(keepends=True),
            candidate_text.splitlines(keepends=True),
            fromfile=str(reference),
            tofile=str(candidate),
            n=3,
        )
    )
    raise RuntimeError(f"portable lock content differs:\n{diff}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference", type=Path, help="committed lock snapshot")
    parser.add_argument("candidate", type=Path, help="newly generated lock")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    compare_lock_files(args.reference, args.candidate)
    print(f"portable XMake lock content matches: {args.reference} == {args.candidate}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
