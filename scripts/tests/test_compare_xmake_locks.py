from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from scripts.compare_xmake_locks import compare_lock_files


def make_lock(
    *,
    cmake: str = "4.3.0",
    libffi: str = "3.4.6",
    ninja: str = "v1.13.2",
    openssl: str = "3.0.13",
    portable: str = "v1.2.3",
    commit: str = "0123456789abcdef",
) -> str:
    packages = (
        ("cmake#aaaa", cmake),
        ("libffi#bbbb", libffi),
        ("ninja#cccc", ninja),
        ("openssl >=1.1.1-a#dddd", openssl),
        ("portable#eeee", portable),
    )
    entries = []
    for requirement, version in packages:
        entries.append(
            f'''        ["{requirement}"] = {{
            repo = {{
                commit = "{commit}",
                url = "https://github.com/xmake-io/xmake-repo.git"
            }},
            version = "{version}"
        }}'''
        )
    return '{\n    ["linux|x86_64"] = {\n' + ",\n".join(entries) + "\n    }\n}\n"


class CompareXmakeLocksTests(unittest.TestCase):
    def compare(self, reference: str, candidate: str) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            reference_path = root / "reference.lock"
            candidate_path = root / "candidate.lock"
            reference_path.write_text(reference, encoding="utf-8")
            candidate_path.write_text(candidate, encoding="utf-8")
            compare_lock_files(reference_path, candidate_path)

    def test_host_tool_versions_may_differ(self) -> None:
        self.compare(
            make_lock(),
            make_lock(
                cmake="3.31.6",
                libffi="3.4.2",
                ninja="1.13.2",
                openssl="3.0.2",
            ),
        )

    def test_portable_package_version_drift_is_rejected(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "portable lock content differs"):
            self.compare(make_lock(), make_lock(portable="v9.9.9"))

    def test_repository_commit_drift_is_rejected(self) -> None:
        candidate = make_lock().replace(
            'commit = "0123456789abcdef"',
            'commit = "fedcba9876543210"',
            1,
        )
        with self.assertRaisesRegex(RuntimeError, "portable lock content differs"):
            self.compare(make_lock(), candidate)

    def test_missing_host_tool_entry_is_rejected(self) -> None:
        candidate = make_lock().replace(
            """        ["ninja#cccc"] = {
            repo = {
                commit = "0123456789abcdef",
                url = "https://github.com/xmake-io/xmake-repo.git"
            },
            version = "v1.13.2"
        },
""",
            "",
        )
        with self.assertRaisesRegex(RuntimeError, "host-version package manifest"):
            self.compare(make_lock(), candidate)


if __name__ == "__main__":
    unittest.main()
