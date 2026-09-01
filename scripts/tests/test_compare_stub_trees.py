from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from zipfile import ZipFile

from scripts.compare_stub_trees import compare_stub_trees, read_stub_tree
from scripts.stubgen import EXPECTED_STUBS


class CompareStubTreesTests(unittest.TestCase):
    def make_stub_directory(self, root: Path, value: str = "value: int\n") -> Path:
        native_dir = root / "uipc" / "_native"
        for relative_path in EXPECTED_STUBS:
            path = native_dir / relative_path
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(value, encoding="utf-8")
        return native_dir

    def make_wheel(self, root: Path, native_dir: Path) -> Path:
        wheel = root / "pyuipc-0.0.0-py3-none-any.whl"
        with ZipFile(wheel, "w") as archive:
            for relative_path in EXPECTED_STUBS:
                archive.write(
                    native_dir / relative_path,
                    f"uipc/_native/{relative_path}",
                )
        return wheel

    def test_directory_and_wheel_match(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            native_dir = self.make_stub_directory(root / "directory")
            wheel = self.make_wheel(root, native_dir)

            compare_stub_trees(native_dir, wheel)

    def test_content_difference_is_reported(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            left = self.make_stub_directory(root / "left")
            right = self.make_stub_directory(root / "right")
            (right / "pyuipc/core.pyi").write_text(
                "value: str\n", encoding="utf-8"
            )

            with self.assertRaisesRegex(
                RuntimeError, r"files=\['pyuipc/core\.pyi'\]"
            ):
                compare_stub_trees(left, right)

    def test_incomplete_wheel_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            wheel = Path(directory) / "incomplete.whl"
            with ZipFile(wheel, "w") as archive:
                archive.writestr(
                    "uipc/_native/pyuipc/__init__.pyi", "value: int\n"
                )

            with self.assertRaisesRegex(RuntimeError, "manifest mismatch"):
                read_stub_tree(wheel)


if __name__ == "__main__":
    unittest.main()
