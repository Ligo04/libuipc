#!/usr/bin/env python3
"""Smoke-test an installed pyuipc wheel without requiring a GPU."""

from __future__ import annotations

import json
import os
import sys
import tempfile
from pathlib import Path


EXPECTED_NATIVE_STUBS = {
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


def validate_typing_artifacts(package_path: Path) -> None:
    marker_file = package_path / "py.typed"
    if not marker_file.is_file():
        raise RuntimeError(f"typing marker is missing: {marker_file}")

    native_dir = package_path / "_native"
    actual_stubs = {
        path.relative_to(native_dir).as_posix()
        for path in native_dir.rglob("*.pyi")
    }
    if actual_stubs != EXPECTED_NATIVE_STUBS:
        raise RuntimeError(
            "native stub manifest mismatch: "
            f"missing={sorted(EXPECTED_NATIVE_STUBS - actual_stubs)!r}, "
            f"unexpected={sorted(actual_stubs - EXPECTED_NATIVE_STUBS)!r}"
        )


def main() -> int:
    import uipc
    from uipc import Engine

    version = str(uipc.__version__)
    expected_version = os.environ.get("UIPC_EXPECTED_VERSION")
    if expected_version and version != expected_version:
        raise RuntimeError(
            f"installed pyuipc version {version!r} does not match "
            f"expected version {expected_version!r}"
        )

    package_path = Path(uipc.__file__).resolve().parent
    native_dir = package_path / "_native"
    if not native_dir.is_dir():
        raise RuntimeError(f"native module directory is missing: {native_dir}")
    validate_typing_artifacts(package_path)

    policy_path = Path(uipc.__file__).resolve().parent / "compatibility.json"
    if not policy_path.is_file():
        raise RuntimeError(f"compatibility policy is missing: {policy_path}")
    policy = json.loads(policy_path.read_text(encoding="utf-8"))

    build_info = dict(uipc.build_info())
    expected_abi = f"cp{sys.version_info.major}{sys.version_info.minor}"
    if build_info["python_abi"] != expected_abi:
        raise RuntimeError(
            f"native ABI {build_info['python_abi']!r} does not match {expected_abi!r}"
        )
    if not build_info["cuda_backend"]:
        raise RuntimeError("published wheel was built without the CUDA backend")
    actual_architectures = str(build_info["cuda_architectures"]).split(",")
    expected_architectures = policy["wheel"]["cuda_architectures"]
    if actual_architectures != expected_architectures:
        raise RuntimeError(
            f"wheel CUDA architectures {actual_architectures!r} do not match "
            f"release policy {expected_architectures!r}"
        )
    if not str(build_info["cuda_toolkit_version"]).startswith(
        str(policy["wheel"]["cuda_toolkit"])
    ):
        raise RuntimeError(
            f"wheel CUDA toolkit {build_info['cuda_toolkit_version']!r} does not match "
            f"release policy {policy['wheel']['cuda_toolkit']!r}"
        )

    with tempfile.TemporaryDirectory(prefix="pyuipc-wheel-smoke-") as workspace:
        engine = Engine("none", workspace)
        del engine

    print(
        json.dumps(
            {
                "version": version,
                "package": str(Path(uipc.__file__).resolve()),
                "native_dir": str(native_dir),
                "backend": "none",
                "build_info": build_info,
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
