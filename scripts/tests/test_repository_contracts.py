from __future__ import annotations

import re
import tempfile
import unittest
from pathlib import Path

from scripts.check_constitution_api import ROOT, check_constitution_api
from scripts.check_workflow_pins import check_workflow_pins
from scripts.detect_0kb_files import detect_0kb_files


class RepositoryContractTests(unittest.TestCase):
    def test_python_wheel_declares_numpy_as_isolated_build_dependency(self) -> None:
        build_system = (
            (ROOT / "pyproject.toml")
            .read_text(encoding="utf-8")
            .split("[project]", maxsplit=1)[0]
        )

        self.assertRegex(build_system, r'(?m)^\s*"numpy",\s*$')

    def test_current_repository_contracts(self) -> None:
        self.assertEqual(detect_0kb_files(ROOT), [])
        self.assertEqual(
            check_constitution_api(ROOT, Path("src/pybind/pyuipc")), []
        )
        self.assertEqual(
            check_constitution_api(ROOT, Path("src/nanobind/pyuipc")), []
        )
        self.assertEqual(check_workflow_pins(ROOT), [])

    def test_nanobind_uses_upstream_python_version_floors(self) -> None:
        recipe = (
            ROOT / "xmake/repository/packages/n/nanobind/xmake.lua"
        ).read_text(encoding="utf-8")
        target = (ROOT / "src/nanobind/xmake.lua").read_text(encoding="utf-8")

        python_floors = (
            ('version:ge("3.0.0")', 'package:add("deps", "python >=3.10")'),
            ('version:ge("2.10.0")', 'package:add("deps", "python >=3.9")'),
            (None, 'package:add("deps", "python >=3.8")'),
        )
        for version_check, dependency in python_floors:
            if version_check is not None:
                self.assertIn(version_check, recipe)
            self.assertIn(dependency, recipe)
        self.assertLess(
            recipe.index('version:ge("3.0.0")'),
            recipe.index('version:ge("2.10.0")'),
        )
        for config in ("python_version", "python_system"):
            self.assertNotIn(f'add_configs("{config}"', recipe)
            self.assertNotIn(f'package:config("{config}")', recipe)
            self.assertNotIn(f'{config} = get_config("{config}")', target)

    def test_nanobind_version_is_synchronized(self) -> None:
        sources_and_patterns = (
            (ROOT / "pyproject.toml", r'"nanobind==([0-9.]+)"'),
            (
                ROOT / "src/nanobind/CMakeLists.txt",
                r'set\(UIPC_NANOBIND_VERSION "([0-9.]+)"\)',
            ),
            (
                ROOT / "src/nanobind/xmake.lua",
                r'add_requires\("nanobind ([0-9.]+)"',
            ),
            (
                ROOT / "xmake/repository/packages/n/nanobind/xmake.lua",
                r'add_versions\("v([0-9.]+)"',
            ),
            (ROOT / "xmake/pack.lua", r'"nanobind==([0-9.]+)"'),
        )

        versions = []
        for path, pattern in sources_and_patterns:
            match = re.search(pattern, path.read_text(encoding="utf-8"))
            self.assertIsNotNone(match, f"nanobind version missing from {path}")
            versions.append(match.group(1))

        self.assertEqual(set(versions), {"3.0.0"})

    def test_binding_implementation_selection_is_explicit(self) -> None:
        root_cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        source_cmake = (ROOT / "src/CMakeLists.txt").read_text(encoding="utf-8")
        root_xmake = (ROOT / "xmake.lua").read_text(encoding="utf-8")
        source_xmake = (ROOT / "src/xmake.lua").read_text(encoding="utf-8")
        pyproject = (ROOT / "pyproject.toml").read_text(encoding="utf-8")

        self.assertIn('set(UIPC_PYTHON_BINDING "nanobind"', root_cmake)
        self.assertIn('add_subdirectory(nanobind)', source_cmake)
        self.assertIn('add_subdirectory(pybind)', source_cmake)
        self.assertIn('option("python_binding")', root_xmake)
        self.assertIn('includes("nanobind")', source_xmake)
        self.assertIn('includes("pybind")', source_xmake)
        self.assertIn('UIPC_PYTHON_BINDING = "nanobind"', pyproject)

        self.assertIn("find_package(pybind11", (ROOT / "src/pybind/CMakeLists.txt").read_text(encoding="utf-8"))
        self.assertIn("find_package(nanobind", (ROOT / "src/nanobind/CMakeLists.txt").read_text(encoding="utf-8"))

    def test_nanobind_stubgen_pipeline_is_shared(self) -> None:
        wrapper = (ROOT / "scripts/stubgen.py").read_text(encoding="utf-8")
        post_build = (ROOT / "scripts/after_build_pyuipc.py").read_text(
            encoding="utf-8"
        )
        xmake_pack = (ROOT / "xmake/pack.lua").read_text(encoding="utf-8")
        package_init = (ROOT / "python/src/uipc/__init__.py").read_text(
            encoding="utf-8"
        )

        self.assertIn('"nanobind.stubgen"', wrapper)
        self.assertIn("scripts/stubgen.py", xmake_pack)
        self.assertIn("stubgen.py", post_build)
        self.assertNotIn("pybind11_stubgen", post_build)
        self.assertNotIn("import mypy", xmake_pack)
        self.assertIn('os.environ.get("NB_STUBGEN")', package_init)

    def test_wheel_build_preserves_active_python_environment(self) -> None:
        post_build = (ROOT / "scripts/after_build_pyuipc.py").read_text(
            encoding="utf-8"
        )

        self.assertIn("build_wheel = is_option_on(args.build_wheel)", post_build)
        self.assertRegex(
            post_build,
            r"if build_wheel:\s+print\(.+preserving the active Python environment",
        )
        self.assertRegex(
            post_build,
            r"else:\s+# Avoid importing stale package files during an in-place build\.\s+"
            r"print\('Cleaning up the old package:'\)\s+uninstall_package\(\)",
        )

    def test_polyscope_show_tests_are_marked_gui(self) -> None:
        gui_tests = []
        for path in (ROOT / "python/tests").rglob("test_*.py"):
            source = path.read_text(encoding="utf-8")
            if "ps.show()" in source:
                gui_tests.append(path)
                self.assertIn(
                    "@pytest.mark.gui",
                    source,
                    f"{path} opens a window but is not marked gui",
                )

        self.assertTrue(gui_tests, "expected at least one Polyscope GUI test")

    def test_keyword_enum_has_runtime_and_stub_alias(self) -> None:
        engine = (ROOT / "src/nanobind/pyuipc/core/engine.cpp").read_text(
            encoding="utf-8"
        )
        patterns = (ROOT / "scripts/pyuipc_stubgen.patterns").read_text(
            encoding="utf-8"
        )

        self.assertNotIn('.value("None_",', engine)
        self.assertIn('enum_Type.attr("None_")', engine)
        self.assertIn('enum_Type.attr("None")', engine)
        self.assertIn("EngineStatus\\.Type\\.None$", patterns)
        self.assertIn("None_ = 0", patterns)
        self.assertIn("None_: Type = Type.None_", patterns)

    def test_zero_byte_detector_is_scoped_and_deterministic(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "include").mkdir()
            (root / "src").mkdir()
            (root / "outside").mkdir()
            (root / "include" / "b.h").touch()
            (root / "src" / "a.cpp").touch()
            (root / "src" / "nonempty.cpp").write_text(
                "// source\n", encoding="utf-8"
            )
            (root / "outside" / "ignored.cpp").touch()

            self.assertEqual(
                [path.relative_to(root).as_posix() for path in detect_0kb_files(root)],
                ["include/b.h", "src/a.cpp"],
            )

    def test_constitution_checker_reports_missing_binding(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            headers = root / "include/uipc/constitution"
            bindings = root / "src/pybind/pyuipc/constitution"
            headers.mkdir(parents=True)
            bindings.mkdir(parents=True)
            (headers / "material.h").write_text(
                "class UIPC_CONSTITUTION_API Material {};\n", encoding="utf-8"
            )

            errors = check_constitution_api(root)
            self.assertIn(
                "public constitution class Material has no Python binding", errors
            )

    def test_constitution_checker_reports_unregistered_initializer(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            headers = root / "include/uipc/constitution"
            bindings = root / "src/pybind/pyuipc/constitution"
            headers.mkdir(parents=True)
            bindings.mkdir(parents=True)
            (headers / "material.h").write_text(
                "class UIPC_CONSTITUTION_API Material {};\n", encoding="utf-8"
            )
            (bindings / "material.cpp").write_text(
                """
                PyMaterial::PyMaterial(py::module& m)
                {
                    py::class_<Material>(m, "Material");
                }
                """,
                encoding="utf-8",
            )

            errors = check_constitution_api(root)
            self.assertIn(
                "binding initializer PyMaterial is never registered in a binding module",
                errors,
            )

    def test_workflow_checker_rejects_mutable_refs(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            workflows = root / ".github/workflows"
            workflows.mkdir(parents=True)
            (workflows / "ci.yml").write_text(
                """
                steps:
                  - uses: actions/checkout@v7
                  - uses: owner/action@0123456789abcdef0123456789abcdef01234567
                    with:
                      revision: master
                """,
                encoding="utf-8",
            )

            errors = check_workflow_pins(root)
            self.assertTrue(any("mutable ref" in error for error in errors))
            self.assertTrue(any("reviewed tag comment" in error for error in errors))
            self.assertTrue(any("revision must be" in error for error in errors))


if __name__ == "__main__":
    unittest.main()
