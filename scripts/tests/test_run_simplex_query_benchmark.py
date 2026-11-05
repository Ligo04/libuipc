from __future__ import annotations

import copy
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from scripts.run_simplex_query_benchmark import (
    BenchmarkError, Cohort, COUNTERS, DEFAULT_PROFILE_PREFIX, FAMILIES, METHODS, STATES,
    classify_compute, main, parse_compute, source_inventory, state_name, step_plan, validate_driver,
    validate_replay, verification_rows,
)


def write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value) + "\n")


def write_rows(path: Path, rows: list[dict]) -> None:
    path.write_text("".join(json.dumps(row) + "\n" for row in rows))


class QueryBenchmarkGateTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.addCleanup(self.temporary.cleanup)

    def make_verification(self) -> tuple[Path, list[dict]]:
        folder = self.root / "verified"
        folder.mkdir()
        rows = []
        for i, frame in enumerate((30, 31, 31), 1):
            states = {}
            for state in STATES:
                stats = dict.fromkeys(COUNTERS, 0)
                stats.update(refreshes=i if state.endswith("refresh") else 1,
                             reuses=0 if state.endswith("refresh") else i - 1,
                             topology_builds=1, refresh_waits=i if state.endswith("refresh") else 1,
                             fused_prefixes=i)
                states[state] = {"counts": dict(pp=1, pe=0, ee=2, pt=3),
                                 "single_ef_active": state.startswith("single_ef"),
                                 "statistics": stats}
            rows.append({"request": i, "epoch": 1, "frame": frame, "newton_iteration": i,
                         "alpha": 0.0 if i == 1 else 1.0, "skin": 0.0,
                         "new_frame": i < 3, "success": True, "states": states,
                         "oracle_counts": dict(pp=1, pe=0, ee=2, pt=3),
                         "independent_oracle_equal": True})
        write_rows(folder / "requests.jsonl", rows)
        write_json(folder / "verification.json", {"trace_complete": True, "requests": 3,
                   "epochs": 1, "bytes": 100, "success": True,
                   "all_candidates_equal": True, "independent_oracle_equal": True,
                   "states": list(STATES)})
        return folder, rows

    def make_replay(self, expected: list[dict], method: str = METHODS[0],
                    policy: str = "natural") -> tuple[Path, list[dict]]:
        folder = self.root / "timed"
        folder.mkdir()
        rows = []
        before = dict.fromkeys(COUNTERS, 0)
        state = state_name(method, policy)
        for i, reference in enumerate(expected):
            after = reference["states"][state]["statistics"]
            rows.append({k: copy.deepcopy(v) for k, v in reference.items() if k not in ("states", "success")})
            rows[-1].update(measured=reference["frame"] >= 31, query_ms=(i + 5) / 10,
                            counts=reference["states"][state]["counts"],
                            single_ef_active=reference["states"][state]["single_ef_active"],
                            before=before.copy(), after=after.copy(),
                            delta={k: after[k] - before[k] for k in COUNTERS},
                            gauges={"stored_pts": 10, "ef_pairs": 0})
            before = after.copy()
        write_rows(folder / "requests.jsonl", rows)
        write_json(folder / "result.json", {"trace_complete": True, "requests": 3,
                   "measured_requests": 2, "method": method, "policy": policy, "profile": False,
                   "constructor_ms": 0.1, "first_detect_ms": 0.5, "cold_total_ms": 0.6,
                   "measured_query_ms": 1.3,
                   "trace_summary": {"complete": True, "requests": 3, "epochs": 1, "bytes": 100}})
        return folder, rows

    def test_source_inventory_records_gitlink_pins_without_opening_directories(self) -> None:
        regular = self.root / "code.cpp"
        payload = b"regular source content"
        regular.write_bytes(payload)
        (self.root / "libuipc-samples").mkdir()
        # The second gitlink is deliberately uninitialized and has no directory.
        first, second = "a" * 40, "b" * 40
        indexed = (f"100644 {'c' * 40} 0\tcode.cpp\0"
                   f"160000 {first} 0\tlibuipc-samples\0"
                   f"160000 {second} 0\tscripts/SymEigen\0")
        opened = []
        def open_regular_only(path):
            self.assertEqual(path, regular)
            opened.append(path)
            return hashlib.sha256(path.read_bytes()).hexdigest()
        with patch("scripts.run_simplex_query_benchmark.ROOT", self.root), \
                patch("scripts.run_simplex_query_benchmark.command", return_value=indexed) as git, \
                patch("scripts.run_simplex_query_benchmark.sha256", side_effect=open_regular_only):
            result = source_inventory()
        git.assert_called_once_with(["git", "ls-files", "--stage", "-z"])
        self.assertEqual(opened, [regular])
        self.assertEqual(result["files"], {str(regular): hashlib.sha256(payload).hexdigest()})
        self.assertEqual(result["gitlinks"], {
            "libuipc-samples": {"mode": "160000", "oid": first},
            "scripts/SymEigen": {"mode": "160000", "oid": second}})

    def test_dry_run_creates_nothing_and_never_queries_gpu(self) -> None:
        output = self.root / "absent"
        with patch("scripts.run_simplex_query_benchmark.gpu_sample", side_effect=AssertionError("GPU")), \
                patch("scripts.run_simplex_query_benchmark.Cohort", side_effect=AssertionError("cohort")), \
                patch("sys.stdout", new_callable=io.StringIO) as stream:
            self.assertEqual(main([str(output), "--dry-run", "--phase", "formal"]), 0)
            payload = json.loads(stream.getvalue())
        self.assertFalse(output.exists())
        self.assertEqual(len(payload["steps"]), 8)
        self.assertTrue(all(step["phase"] == "formal" for step in payload["steps"]))

    def test_plan_uses_one_capture_and_verification_before_each_replay_group(self) -> None:
        plan = step_plan(self.root, ["wb", "ex88"], list(DEFAULT_PROFILE_PREFIX))
        self.assertEqual(len(plan), 20)
        self.assertEqual([s["phase"] for s in plan[:4]], ["capture", "capture", "verify", "verify"])
        self.assertEqual(len({s["label"] for s in plan}), len(plan))
        self.assertTrue(all("xmake" not in s["command"] for s in plan))
        captures = [s for s in plan if s["phase"] == "capture"]
        self.assertTrue(all(s["environment"]["UIPC_SINGLE_EF_PERF_METHOD"] == METHODS[0] for s in captures))
        profiles = [s for s in plan if s["phase"] == "profile"]
        self.assertTrue(all(s["command"][:len(DEFAULT_PROFILE_PREFIX)] == list(DEFAULT_PROFILE_PREFIX)
                            for s in profiles))

    def test_pid_classification_retains_foreign_and_accepts_own_exit_without_name(self) -> None:
        binary = self.root / "test"
        samples = [{"utc": "first", "compute": parse_compute(f"20, {binary}, 100\n30, python, 200")},
                   {"utc": "last", "compute": parse_compute("20, [No data], 50\n31, [No data], 30")}]
        foreign, unnamed = classify_compute(samples, binary, [])
        self.assertEqual([r["processes"][0]["pid"] for r in foreign], [30, 31])
        self.assertEqual([r["pid"] for r in unnamed], [20])

    def test_unknown_compute_row_is_not_silently_dropped(self) -> None:
        with self.assertRaises(BenchmarkError):
            parse_compute("N/A")

    def test_driver_needs_full_frame_ledger_not_highest_query_frame(self) -> None:
        folder = self.root / "capture"
        folder.mkdir()
        write_json(folder / "result.json", {"requested_frames": 1000, "completed_frames": 1000,
                   "finite": True, "valid": True})
        rows = [{"frame": i, "finite": True, "valid": True, "stats": {"completed": True,
                 "converged": True}, "negative_tets": int(i == 57), "min_tet_J": -0.1 if i == 57 else 1.0}
                for i in range(1, 1001)]
        write_rows(folder / "frames.jsonl", rows)
        result = validate_driver(folder)
        self.assertEqual(result["measured_source_frames"], 970)
        self.assertEqual(result["negative_J_frames"], [57])
        write_rows(folder / "frames.jsonl", rows[:499] + rows[500:])
        with self.assertRaises(BenchmarkError):
            validate_driver(folder)

    def test_incomplete_footer_blocks_verification(self) -> None:
        folder, _ = self.make_verification()
        result = json.loads((folder / "verification.json").read_text())
        result["trace_complete"] = False
        write_json(folder / "verification.json", result)
        with self.assertRaises(BenchmarkError):
            list(verification_rows(folder))

    def test_common_output_agreement_without_independent_oracle_is_not_eligible(self) -> None:
        folder, _ = self.make_verification()
        result = json.loads((folder / "verification.json").read_text())
        result.pop("independent_oracle_equal")
        write_json(folder / "verification.json", result)
        with self.assertRaises(BenchmarkError):
            list(verification_rows(folder))

    def test_candidate_family_count_mismatch_fails_verification(self) -> None:
        folder, rows = self.make_verification()
        rows[1]["states"]["single_ef_natural"]["counts"]["ee"] += 1
        write_rows(folder / "requests.jsonl", rows)
        with self.assertRaises(BenchmarkError):
            list(verification_rows(folder))

    def test_natural_replay_keeps_warmup_history_and_source_frame_denominator(self) -> None:
        verified, expected = self.make_verification()
        timed, _ = self.make_replay(expected)
        metrics = validate_replay(timed, verified, METHODS[0], "natural", False)
        self.assertEqual(metrics["requests"], 3)
        self.assertEqual(metrics["measured_requests"], 2)
        self.assertAlmostEqual(metrics["query_ms_per_source_frame"], 1.3 / 970)
        self.assertEqual(metrics["cumulative_counter_deltas"]["reuses"], 2)
        self.assertEqual(metrics["cumulative_counter_deltas"]["refreshes"], 0)
        self.assertEqual(metrics["cold_samples"], 1)
        self.assertEqual(metrics["final_gauges"]["stored_pts"], 10)

    def test_changed_frame_start_marker_is_rejected(self) -> None:
        verified, expected = self.make_verification()
        timed, rows = self.make_replay(expected)
        rows[2]["new_frame"] = True
        write_rows(timed / "requests.jsonl", rows)
        with self.assertRaises(BenchmarkError):
            validate_replay(timed, verified, METHODS[0], "natural", False)

    def test_timed_counts_must_match_exact_reference(self) -> None:
        verified, expected = self.make_verification()
        timed, rows = self.make_replay(expected)
        rows[2]["counts"] = {k: 0 for k in FAMILIES}
        write_rows(timed / "requests.jsonl", rows)
        with self.assertRaises(BenchmarkError):
            validate_replay(timed, verified, METHODS[0], "natural", False)

    def test_truncated_replay_ledger_does_not_pass_aggregate_counts(self) -> None:
        verified, expected = self.make_verification()
        timed, rows = self.make_replay(expected)
        write_rows(timed / "requests.jsonl", rows[:-1])
        with self.assertRaises(BenchmarkError):
            validate_replay(timed, verified, METHODS[0], "natural", False)

    def test_cold_metrics_must_match_first_request_and_constructor(self) -> None:
        verified, expected = self.make_verification()
        timed, _ = self.make_replay(expected)
        result = json.loads((timed / "result.json").read_text())
        result["cold_total_ms"] = 9.0
        write_json(timed / "result.json", result)
        with self.assertRaises(BenchmarkError):
            validate_replay(timed, verified, METHODS[0], "natural", False)

    def test_profile_cannot_be_reported_as_formal(self) -> None:
        verified, expected = self.make_verification()
        timed, _ = self.make_replay(expected)
        result = json.loads((timed / "result.json").read_text())
        result["profile"] = True
        write_json(timed / "result.json", result)
        with self.assertRaises(BenchmarkError):
            validate_replay(timed, verified, METHODS[0], "natural", False)

    def test_missing_profile_report_fails_and_preserves_attempt_evidence(self) -> None:
        cohort = Cohort.__new__(Cohort)
        cohort.out = self.root
        marker = self.root / "attempt.json"
        validation = {"returncode": 0, "command": ["nsys", "profile"]}
        with self.assertRaises(BenchmarkError):
            cohort.validate_profile_collection("run", marker, validation)
        saved = json.loads(marker.read_text())
        self.assertEqual(saved["state"], "profile_collection_failed")
        self.assertEqual(saved["returncode"], 0)
        self.assertEqual(saved["profile_collection"]["collection_status"], "missing")
        self.assertEqual(saved["profile_collection"]["cuda_activity_validation"], "pending")
        self.assertIsNone(saved["profile_collection"]["sha256"])

    def test_empty_profile_report_is_rejected_with_fingerprint(self) -> None:
        cohort = Cohort.__new__(Cohort)
        cohort.out = self.root
        (self.root / "profiles").mkdir()
        report = self.root / "profiles/run.nsys-rep"
        report.touch()
        marker = self.root / "attempt.json"
        with self.assertRaises(BenchmarkError):
            cohort.validate_profile_collection("run", marker, {"returncode": 0})
        saved = json.loads(marker.read_text())["profile_collection"]
        self.assertEqual(saved["collection_status"], "empty")
        self.assertEqual(saved["bytes"], 0)
        self.assertEqual(saved["sha256"], hashlib.sha256(b"").hexdigest())
        self.assertTrue(report.exists())

    def test_nonempty_profile_report_records_collection_but_not_cuda_activity_success(self) -> None:
        cohort = Cohort.__new__(Cohort)
        cohort.out = self.root
        (self.root / "profiles").mkdir()
        report = self.root / "profiles/run.nsys-rep"
        payload = b"container presence does not certify CUDA activity"
        report.write_bytes(payload)
        marker = self.root / "attempt.json"
        metadata = cohort.validate_profile_collection("run", marker, {"returncode": 0})
        self.assertEqual(metadata["path"], str(report.resolve()))
        self.assertEqual(metadata["bytes"], len(payload))
        self.assertEqual(metadata["sha256"], hashlib.sha256(payload).hexdigest())
        self.assertEqual(metadata["collection_status"], "report_present_nonempty")
        self.assertEqual(metadata["cuda_activity_validation"], "pending")
        self.assertEqual(json.loads(marker.read_text())["state"], "profile_file_collected")

    def test_existing_attempt_marker_is_never_overwritten(self) -> None:
        cohort = Cohort.__new__(Cohort)
        cohort.out = self.root
        (self.root / "attempts").mkdir()
        marker = self.root / "attempts/old.json"
        marker.write_text('{"state":"failed"}')
        step = {"label": "old", "folder": str(self.root / "never-created")}
        with self.assertRaises(FileExistsError):
            cohort.run_process(step)
        self.assertEqual(marker.read_text(), '{"state":"failed"}')

    def test_formal_phase_cannot_launch_before_exact_verification(self) -> None:
        cohort = Cohort.__new__(Cohort)
        cohort.args = type("Arguments", (), {"phase": "formal"})()
        cohort.plan = [{"phase": "formal", "scene": "wb"}]
        with patch.object(cohort, "validate_capture"), \
                patch.object(cohort, "validate_verification", side_effect=BenchmarkError("unequal")), \
                patch.object(cohort, "run_process") as launch:
            with self.assertRaises(BenchmarkError):
                cohort.execute()
        launch.assert_not_called()

    def test_counter_history_is_checked_but_gauges_are_not_summed(self) -> None:
        verified, expected = self.make_verification()
        timed, rows = self.make_replay(expected)
        rows[-1]["delta"]["fused_prefixes"] += 1
        rows[-1]["after"]["fused_prefixes"] += 1
        write_rows(timed / "requests.jsonl", rows)
        with self.assertRaises(BenchmarkError):
            validate_replay(timed, verified, METHODS[0], "natural", False)

    def test_effective_singleef_path_must_match_reference(self) -> None:
        verified, expected = self.make_verification()
        timed, rows = self.make_replay(expected, METHODS[1])
        rows[-1]["single_ef_active"] = False
        write_rows(timed / "requests.jsonl", rows)
        with self.assertRaises(BenchmarkError):
            validate_replay(timed, verified, METHODS[1], "natural", False)


if __name__ == "__main__":
    unittest.main()
