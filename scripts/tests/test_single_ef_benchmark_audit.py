from __future__ import annotations

import copy
import json
import tempfile
import unittest
from pathlib import Path

from scripts.benchmark_single_ef_wrecking_ball import audit_summary, validate_swept_audit


def matching_records(*, optix: bool) -> list[dict]:
    family = {
        "equal": True,
        "actual_count": 1,
        "baseline_count": 1,
        "actual_duplicates": 0,
        "baseline_duplicates": 0,
        "missing": 0,
        "extra": 0,
    }
    records = []
    for sequence, alpha in ((1, 0.0), (2, 1.0)):
        detect = {
            "phase": "detect",
            "alpha": alpha,
            "detect_sequence": sequence,
            "used_single_ef": True,
            "equal": True,
            "PT": dict(family),
            "EE": dict(family),
        }
        if optix:
            detect.update(
                requested_backend="optix",
                executed_backend="optix",
                optix_builds=int(sequence == 1),
                optix_refits=int(sequence == 2),
                optix_launches=2,
            )
        records.append(detect)
        active = dict(detect, phase="active", PP=dict(family), PE=dict(family))
        records.append(active)
    toi_family = dict(
        family,
        compared_pairs=1,
        value_mismatches=0,
        nonfinite_actual=0,
        nonfinite_baseline=0,
        max_abs_diff=0.0,
        max_rel_diff=0.0,
    )
    records.append(
        dict(
            records[-2],
            phase="toi",
            toi_compared=True,
            abs_tolerance=1e-12,
            rel_tolerance=1e-9,
            actual_min=0.75,
            baseline_min=0.75,
            minimum_abs_diff=0.0,
            minimum_equal=True,
            PT=dict(toi_family),
            EE=dict(toi_family),
        )
    )
    return records


class SingleEFAuditTests(unittest.TestCase):
    def summarize(self, records: list[dict]) -> dict:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "audit.jsonl"
            path.write_text("".join(json.dumps(record) + "\n" for record in records))
            return audit_summary(path)

    def test_native_audits_remain_readable_without_optix_metadata(self) -> None:
        summary = self.summarize(matching_records(optix=False))
        validate_swept_audit(summary)
        with self.assertRaisesRegex(ValueError, "actual OptiX"):
            validate_swept_audit(summary, require_optix=True)

    def test_optix_requires_real_build_and_launch_evidence_per_detect(self) -> None:
        summary = self.summarize(matching_records(optix=True))
        validate_swept_audit(summary, require_optix=True)
        self.assertEqual(summary["optix_launches"], 4)
        self.assertEqual(summary["optix_builds"], 1)
        self.assertEqual(summary["optix_refits"], 1)
        self.assertEqual(summary["dcd_optix_launch_records"], 1)
        self.assertEqual(summary["ccd_optix_launch_records"], 1)

    def test_matching_contacts_do_not_hide_a_missing_optix_launch(self) -> None:
        records = matching_records(optix=True)
        records[2]["optix_launches"] = 0
        with self.assertRaisesRegex(ValueError, "actual OptiX"):
            validate_swept_audit(self.summarize(records), require_optix=True)

    def test_cuda_execution_cannot_be_labeled_as_optix_performance(self) -> None:
        records = matching_records(optix=True)
        records[0]["executed_backend"] = "cuda"
        with self.assertRaisesRegex(ValueError, "actual OptiX"):
            validate_swept_audit(self.summarize(records), require_optix=True)

    def test_incomplete_or_invalid_counter_records_fail(self) -> None:
        for value in (None, -1, True, 1.5):
            with self.subTest(value=value):
                records = matching_records(optix=True)
                records[0]["optix_launches"] = value
                with self.assertRaisesRegex(ValueError, "invalid optix_launches"):
                    self.summarize(records)

    def test_optix_metadata_does_not_bypass_contact_or_toi_validation(self) -> None:
        records = matching_records(optix=True)
        for phase, family, key in (("detect", "PT", "missing"), ("toi", "EE", "value_mismatches")):
            with self.subTest(phase=phase):
                changed = copy.deepcopy(records)
                record = next(record for record in changed if record["phase"] == phase)
                record[family][key] = 1
                with self.assertRaisesRegex(ValueError, "mismatches"):
                    validate_swept_audit(self.summarize(changed), require_optix=True)


if __name__ == "__main__":
    unittest.main()
