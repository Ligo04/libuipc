#!/usr/bin/env python3
"""Generate and validate recursive nanobind stubs for pyuipc."""

from __future__ import annotations

import argparse
import ast
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Iterable


MODULE_NAME = "uipc._native.pyuipc"
EXPECTED_STUBS = frozenset(
    {
        "pyuipc/__init__.pyi",
        "pyuipc/backend.pyi",
        "pyuipc/builtin.pyi",
        "pyuipc/constitution.pyi",
        "pyuipc/core.pyi",
        "pyuipc/diff_sim.pyi",
        "pyuipc/geometry/__init__.pyi",
        "pyuipc/geometry/affine_body.pyi",
        "pyuipc/unit.pyi",
        "pyuipc/usd.pyi",
    }
)
DEFAULT_PATTERN_FILE = Path(__file__).with_name("pyuipc_stubgen.patterns")
def validate_nanobind_stubs(output_dir: Path, marker_file: Path) -> None:
    """Reject incomplete or syntactically invalid stub output."""
    actual = {
        path.relative_to(output_dir).as_posix()
        for path in output_dir.rglob("*.pyi")
    }
    missing = sorted(EXPECTED_STUBS - actual)
    unexpected = sorted(actual - EXPECTED_STUBS)
    if missing or unexpected:
        raise RuntimeError(
            "nanobind stub manifest mismatch: "
            f"missing={missing!r}, unexpected={unexpected!r}"
        )

    for relative_path in sorted(actual):
        stub_path = output_dir / relative_path
        ast.parse(stub_path.read_text(encoding="utf-8"), filename=str(stub_path))

    if not marker_file.is_file():
        raise RuntimeError(f"nanobind did not generate marker file: {marker_file}")


def generate_nanobind_stubs(
    source_dir: Path,
    output_dir: Path,
    marker_file: Path,
    pattern_files: Iterable[Path],
) -> None:
    """Run nanobind's stable CLI against a fully staged Python package."""
    source_dir = source_dir.resolve()
    output_dir = output_dir.resolve()
    marker_file = marker_file.resolve()
    pattern_files = tuple(path.resolve() for path in pattern_files)

    native_dir = source_dir / "uipc" / "_native"
    if not native_dir.is_dir():
        raise RuntimeError(f"staged native module directory is missing: {native_dir}")
    for pattern_file in pattern_files:
        if not pattern_file.is_file():
            raise RuntimeError(f"stub pattern file is missing: {pattern_file}")

    for stale_stub in (source_dir / "uipc").rglob("*.pyi"):
        stale_stub.unlink()
    shutil.rmtree(output_dir / "pyuipc", ignore_errors=True)
    output_dir.mkdir(parents=True, exist_ok=True)
    marker_file.parent.mkdir(parents=True, exist_ok=True)
    marker_file.unlink(missing_ok=True)

    command = [
        sys.executable,
        "-m",
        "nanobind.stubgen",
        "-m",
        MODULE_NAME,
        "-r",
        "-i",
        str(source_dir),
        "-O",
        str(output_dir),
        "-M",
        str(marker_file),
    ]
    for pattern_file in pattern_files:
        command.extend(("-p", str(pattern_file)))

    subprocess.run(command, check=True)
    validate_nanobind_stubs(output_dir, marker_file)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source-dir",
        "--source_dir",
        dest="source_dir",
        type=Path,
        required=True,
        help="staged Python source root containing uipc/",
    )
    parser.add_argument(
        "--output-dir",
        "--output_dir",
        dest="output_dir",
        type=Path,
        required=True,
        help="directory that will contain the recursive pyuipc stubs",
    )
    parser.add_argument(
        "--marker-file",
        "--marker_file",
        dest="marker_file",
        type=Path,
        help="py.typed marker path (defaults to <source-dir>/uipc/py.typed)",
    )
    parser.add_argument(
        "--pattern-file",
        "--pattern_file",
        dest="pattern_files",
        action="append",
        type=Path,
        help="nanobind stubgen pattern file; may be repeated",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    marker_file = args.marker_file or args.source_dir / "uipc" / "py.typed"
    pattern_files = args.pattern_files or [DEFAULT_PATTERN_FILE]
    generate_nanobind_stubs(
        source_dir=args.source_dir,
        output_dir=args.output_dir,
        marker_file=marker_file,
        pattern_files=pattern_files,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
