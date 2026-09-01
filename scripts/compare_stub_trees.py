#!/usr/bin/env python3
"""Compare recursive pyuipc stubs produced by two build artifacts."""

from __future__ import annotations

import argparse
import difflib
from pathlib import Path
from zipfile import ZipFile

try:
    from scripts.stubgen import EXPECTED_STUBS
except ModuleNotFoundError:  # Direct execution sets scripts/ as sys.path[0].
    from stubgen import EXPECTED_STUBS


def _normalize(text: str) -> str:
    return text.replace("\r\n", "\n")


def read_stub_tree(source: Path) -> dict[str, str]:
    """Read a pyuipc stub tree from an ``_native`` directory or wheel."""
    source = source.resolve()
    if source.is_dir():
        stubs = {
            path.relative_to(source).as_posix(): _normalize(
                path.read_text(encoding="utf-8")
            )
            for path in source.rglob("*.pyi")
        }
    elif source.is_file() and source.suffix == ".whl":
        stubs = {}
        with ZipFile(source) as archive:
            for name in archive.namelist():
                marker = "/_native/"
                if marker not in name or not name.endswith(".pyi"):
                    continue
                relative_path = name.split(marker, maxsplit=1)[1]
                stubs[relative_path] = _normalize(
                    archive.read(name).decode("utf-8")
                )
    else:
        raise RuntimeError(
            f"stub source must be an _native directory or wheel: {source}"
        )

    actual = frozenset(stubs)
    if actual != EXPECTED_STUBS:
        raise RuntimeError(
            f"stub manifest mismatch for {source}: "
            f"missing={sorted(EXPECTED_STUBS - actual)!r}, "
            f"unexpected={sorted(actual - EXPECTED_STUBS)!r}"
        )
    return stubs


def compare_stub_trees(left: Path, right: Path) -> None:
    """Raise with a focused diff when two complete stub trees differ."""
    left_stubs = read_stub_tree(left)
    right_stubs = read_stub_tree(right)

    different = [
        name
        for name in sorted(EXPECTED_STUBS)
        if left_stubs[name] != right_stubs[name]
    ]
    if not different:
        return

    first = different[0]
    diff = "".join(
        difflib.unified_diff(
            left_stubs[first].splitlines(keepends=True),
            right_stubs[first].splitlines(keepends=True),
            fromfile=f"{left}:{first}",
            tofile=f"{right}:{first}",
            n=3,
        )
    )
    raise RuntimeError(
        "pyuipc stub content differs: "
        f"files={different!r}\nfirst difference:\n{diff}"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("left", type=Path, help="first _native directory or wheel")
    parser.add_argument("right", type=Path, help="second _native directory or wheel")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    compare_stub_trees(args.left, args.right)
    print(f"pyuipc stub trees match: {args.left} == {args.right}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
