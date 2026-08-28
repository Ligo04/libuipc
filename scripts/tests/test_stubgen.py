from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from scripts.stubgen import (
    EXPECTED_STUBS,
    PYBIND11_KEYWORD_DECLARATION,
    validate_nanobind_stubs,
)


class StubgenTests(unittest.TestCase):
    def make_stub_tree(self, root: Path) -> tuple[Path, Path]:
        output_dir = root / "uipc" / "_native"
        for relative_path in EXPECTED_STUBS:
            stub_path = output_dir / relative_path
            stub_path.parent.mkdir(parents=True, exist_ok=True)
            stub_path.write_text("value: int\n", encoding="utf-8")
        marker_file = root / "uipc" / "py.typed"
        marker_file.touch()
        return output_dir, marker_file

    def test_validate_stubs_accepts_complete_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output_dir, marker_file = self.make_stub_tree(Path(directory))
            validate_nanobind_stubs(output_dir, marker_file)

    def test_validate_stubs_rejects_missing_file(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output_dir, marker_file = self.make_stub_tree(Path(directory))
            (output_dir / "pyuipc/core.pyi").unlink()

            with self.assertRaisesRegex(RuntimeError, "missing=.*core.pyi"):
                validate_nanobind_stubs(output_dir, marker_file)

    def test_validate_stubs_rejects_invalid_python(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output_dir, marker_file = self.make_stub_tree(Path(directory))
            (output_dir / "pyuipc/core.pyi").write_text(
                "None = 0\n", encoding="utf-8"
            )

            with self.assertRaises(SyntaxError):
                validate_nanobind_stubs(output_dir, marker_file)

    def test_pybind11_keyword_declarations_gain_safe_aliases(self) -> None:
        source = (
            "class Type:\n"
            "    None: int\n"
            "class Status:\n"
            "    None = 0\n"
        )

        rewritten = PYBIND11_KEYWORD_DECLARATION.sub(
            r"\g<indent>None_\g<suffix>", source
        )

        self.assertIn("    None_: int", rewritten)
        self.assertIn("    None_ = 0", rewritten)


if __name__ == "__main__":
    unittest.main()
