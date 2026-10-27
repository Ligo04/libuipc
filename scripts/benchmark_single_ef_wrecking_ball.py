#!/usr/bin/env python3
"""Run separate wrecking-ball throughput, timer, and candidate-audit processes."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
import math
import os
import platform
import re
import statistics
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
METHODS = ("info_stackless_bvh", "single_ef_bvh")
AVAILABLE_METHODS = (*METHODS, "single_ef_optix")
AUDIT_ENV = "UIPC_SINGLE_EF_AUDIT_FILE"


def command(command: list[str], *, timeout: int = 15) -> dict[str, Any]:
    try:
        result = subprocess.run(command, cwd=ROOT, capture_output=True, timeout=timeout)
        return {
            "returncode": result.returncode,
            "stdout": result.stdout.decode(errors="replace").strip(),
            "stderr": result.stderr.decode(errors="replace").strip(),
        }
    except (OSError, subprocess.TimeoutExpired) as error:
        return {"returncode": None, "error": str(error)}


def digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def dump(path: Path, payload: Any) -> None:
    path.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n")


def repository_state(excluded: Path) -> dict[str, Any]:
    diff = subprocess.run(
        ["git", "diff", "--binary", "HEAD"], cwd=ROOT, capture_output=True, check=True
    ).stdout
    untracked = subprocess.run(
        ["git", "ls-files", "--others", "--exclude-standard", "-z"],
        cwd=ROOT,
        capture_output=True,
        check=True,
    ).stdout
    changed = subprocess.run(
        ["git", "diff", "--name-only", "-z", "HEAD"],
        cwd=ROOT,
        capture_output=True,
        check=True,
    ).stdout
    names = set((changed + untracked).decode().split("\0"))
    prefixes = (
        "src/",
        "include/",
        "scripts/",
        "xmake/",
        "apps/examples/wrecking_ball/",
        "apps/tests/backends/cuda/",
    )
    suffixes = (".cpp", ".cu", ".h", ".cuh", ".inl", ".py", ".lua", ".cmake", ".txt")
    hashes = {}
    for name in sorted(names):
        source = ROOT / name
        relevant = name.startswith(prefixes) and name.endswith(suffixes)
        if (
            relevant or name in ("xmake.lua", "CMakeLists.txt")
        ) and not source.resolve().is_relative_to(excluded):
            hashes[name] = digest(source) if source.is_file() else "deleted"
    return {
        "sha": command(["git", "rev-parse", "HEAD"]),
        "status": command(["git", "status", "--porcelain", "--untracked-files=no"]),
        "tracked_diff_sha256": hashlib.sha256(diff).hexdigest(),
        "changed_source_sha256": hashes,
    }


def xmake_config_paths() -> list[Path]:
    root = Path(os.environ.get("XMAKE_CONFIGDIR", ROOT)).resolve()
    return sorted((root / ".xmake").glob("*/*/xmake.conf"))


def artifact_paths(build_dir: Path | None) -> tuple[Path, list[Path]]:
    if build_dir is None:
        configs = xmake_config_paths()
        if len(configs) != 1:
            raise ValueError(
                "Cannot resolve one xmake configuration; supply --build-dir"
            )
        text = configs[0].read_text()

        def setting(name: str) -> str:
            match = re.search(rf'\b{re.escape(name)}\s*=\s*"([^"]+)"', text)
            if not match:
                raise ValueError(f"Missing xmake {name}; supply --build-dir")
            return match[1]

        build_dir = (
            ROOT
            / setting("builddir")
            / setting("plat")
            / setting("arch")
            / setting("mode")
        )
    build_dir = build_dir.resolve()
    executables = [
        p
        for p in build_dir.rglob("wrecking_ball*")
        if p.is_file() and p.name in ("wrecking_ball", "wrecking_ball.exe")
    ]
    if len(executables) != 1:
        raise ValueError(
            f"Expected one built wrecking_ball in {build_dir}; found {executables}"
        )
    executable = executables[0]
    libraries = [
        p
        for p in build_dir.rglob("*")
        if p.is_file()
        and ("uipc" in p.name.lower() or "_native" in p.name.lower())
        and (".so" in p.name or p.suffix.lower() in (".dll", ".dylib", ".pyd"))
    ]
    if not any("backend_cuda" in p.name for p in libraries):
        raise ValueError(
            "CUDA backend artifact was not found in the selected build directory"
        )
    return executable, sorted(set([executable, *libraries]))


def fingerprint(paths: list[Path]) -> dict[str, Any]:
    return {
        str(p): {"sha256": digest(p), "bytes": p.stat().st_size}
        if p.is_file()
        else {"missing": True}
        for p in paths
    }


def host_facts() -> dict[str, Any]:
    return {
        "platform": platform.platform(),
        "cpu_count": os.cpu_count(),
        "nproc": command(["nproc"]),
        "uptime": command(["uptime"]),
        "loadavg": list(os.getloadavg()) if hasattr(os, "getloadavg") else None,
        "gpu": command(
            [
                "nvidia-smi",
                "--query-gpu=index,uuid,name,driver_version",
                "--format=csv,noheader",
            ]
        ),
        "initial_compute_processes": command(
            [
                "nvidia-smi",
                "--query-compute-apps=gpu_uuid,pid,process_name,used_memory",
                "--format=csv,noheader",
            ]
        ),
        "environment": {
            k: os.environ[k]
            for k in (
                "CUDA_VISIBLE_DEVICES",
                "CUDA_DEVICE_ORDER",
                "LD_LIBRARY_PATH",
                "UIPC_ENGINE_DEFAULT_CONFIG",
                "XMAKE_CONFIGDIR",
                "UIPC_OPTIX_EF_GROUP_SIZE",
            )
            if k in os.environ
        },
    }


def sample_gpu(stop: threading.Event, path: Path) -> None:
    fields = "timestamp,index,uuid,utilization.gpu,memory.used,memory.total,clocks.current.sm,clocks.current.memory,power.draw"
    start = time.monotonic()
    with path.open("w") as stream:
        while not stop.is_set():
            payload = command(
                [
                    "nvidia-smi",
                    f"--query-gpu={fields}",
                    "--format=csv,noheader,nounits",
                ],
                timeout=5,
            )
            payload["elapsed_s"] = time.monotonic() - start
            if payload.get("returncode") == 0:
                payload["columns"] = fields.split(",")
                payload["rows"] = list(
                    csv.reader(payload["stdout"].splitlines(), skipinitialspace=True)
                )
            stream.write(json.dumps(payload) + "\n")
            stream.flush()
            if stop.wait(1):
                break


def normalized_config(config: Any) -> Any:
    config = json.loads(json.dumps(config))
    if isinstance(config, dict):
        config.pop("collision_detection/method", None)
        if isinstance(config.get("collision_detection"), dict):
            config["collision_detection"].pop("method", None)
    return config


def validate_result(
    payload: dict[str, Any], method: str, frames: int, warmup: int
) -> float:
    if not isinstance(payload, dict):
        raise ValueError("Native result is not an object")
    if (payload.get("method"), payload.get("frames"), payload.get("warmup")) != (
        method,
        frames,
        warmup,
    ):
        raise ValueError(
            "Native result method/frames/warmup do not match the requested run"
        )
    if not isinstance(payload.get("config"), dict):
        raise ValueError("Native result has no config object")
    samples = payload.get("frame_samples", [])
    if not isinstance(samples, list):
        raise ValueError("Native frame_samples is not an array")
    if len(samples) != frames:
        raise ValueError(f"Expected {frames} frame samples, received {len(samples)}")
    measured = []
    for frame, sample in enumerate(samples, 1):
        if not isinstance(sample, dict) or not isinstance(sample.get("stats"), dict):
            raise ValueError(f"Frame {frame} is not a sample/stats object")
        stats = sample["stats"]
        if sample.get("frame") != frame or stats.get("completed") is not True:
            raise ValueError(f"Frame {frame} missing, out of order, or not completed")
        if stats.get("converged") is False:
            raise ValueError(f"Frame {frame} did not converge")
        for key in (
            "hit_newton_limit",
            "hit_line_search_limit",
            "failure_count",
            "error_count",
        ):
            if stats.get(key, 0):
                raise ValueError(f"Frame {frame}: {key}={stats[key]}")
        value = sample.get("wall_ms")
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value < 0
        ):
            raise ValueError(f"Frame {frame} has invalid wall_ms")
        if frame > warmup:
            measured.append(value)
    for key in ("failure_count", "error_count", "failed_frames"):
        if payload.get(key, 0):
            raise ValueError(f"Native result reports {key}={payload[key]}")
    observable = payload.get("final_observable", {})
    if not isinstance(observable, dict):
        raise ValueError("Final observable is not an object")
    translation = observable.get("ball_translation", [])
    if (
        observable.get("finite") is not True
        or not isinstance(translation, list)
        or len(translation) != 3
        or not all(
            isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
            for v in translation
        )
    ):
        raise ValueError("Final observable is nonfinite or lacks ball_translation")
    return statistics.mean(measured)


def workload_means(payload: dict[str, Any], warmup: int) -> dict[str, float]:
    samples = payload["frame_samples"][warmup:]
    means = {}
    for key in ("newton_iterations", "line_search_trials", "linear_solver_iterations"):
        values = [sample["stats"].get(key) for sample in samples]
        if any(isinstance(v, bool) or not isinstance(v, int) or v < 0 for v in values):
            raise ValueError(f"Measured frame stats lack valid {key}")
        means[key] = statistics.mean(values)
    return means


def squared_transform_difference(actual: Any, baseline: Any) -> float:
    if isinstance(actual, dict) and isinstance(baseline, dict):
        if actual.keys() != baseline.keys():
            raise ValueError("Final transform object structures differ")
        return math.fsum(
            squared_transform_difference(actual[key], baseline[key])
            for key in sorted(actual)
        )
    if isinstance(actual, list) and isinstance(baseline, list):
        if len(actual) != len(baseline):
            raise ValueError("Final transform array structures differ")
        return math.fsum(
            squared_transform_difference(a, b) for a, b in zip(actual, baseline)
        )
    if any(
        isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
        for v in (actual, baseline)
    ):
        raise ValueError(
            "Final transform contains nonnumeric/nonfinite values or structures differ"
        )
    return (actual - baseline) ** 2


def final_drift(baseline: dict[str, Any], actual: dict[str, Any]) -> dict[str, Any]:
    count = baseline.get("body_count")
    if (
        isinstance(count, bool)
        or not isinstance(count, int)
        or count <= 0
        or actual.get("body_count") != count
    ):
        raise ValueError("Final observable body counts are missing or differ")
    matrices_a, matrices_b = actual.get("transforms"), baseline.get("transforms")
    if (
        not isinstance(matrices_a, list)
        or not isinstance(matrices_b, list)
        or len(matrices_a) != count
        or len(matrices_b) != count
    ):
        raise ValueError("Final observable transforms do not match body count")
    squared = [
        squared_transform_difference(a, b) for a, b in zip(matrices_a, matrices_b)
    ]
    return {
        "body_count": count,
        "ball_translation_l2": math.dist(
            actual["ball_translation"], baseline["ball_translation"]
        ),
        "body_transform_frobenius_max": math.sqrt(max(squared)),
        "body_transform_frobenius_rms": math.sqrt(statistics.mean(squared)),
    }


def audit_summary(path: Path) -> dict[str, Any]:
    summary: dict[str, Any] = {
        "records": 0,
        "detect_records": 0,
        "active_records": 0,
        "toi_records": 0,
        "dcd_used_single_ef": 0,
        "ccd_used_single_ef": 0,
        "ccd_fallback": 0,
        "dcd_fallback": 0,
        "optix_requested_detect_records": 0,
        "dcd_optix_launch_records": 0,
        "ccd_optix_launch_records": 0,
        "optix_builds": 0,
        "optix_refits": 0,
        "optix_launches": 0,
        "optix_execution_errors": 0,
        "optix_culling_records": 0,
        "optix_grouped_detect_records": 0,
        "optix_group_sizes": [],
        "optix_group_count_range": None,
        "optix_group_tests": 0,
        "optix_body_rejections": 0,
        "optix_contact_rejections": 0,
        "optix_skipped_leaves": 0,
        "optix_leaf_tests": 0,
        "optix_emitted_pairs": 0,
        "executed_backends": {},
        "nonempty_detect_records": 0,
        "nonempty_active_records": 0,
        "nonempty_ccd_single_records": 0,
        "toi_single_ccd_records": 0,
        "nonempty_toi_single_ccd_records": 0,
        "mismatches": 0,
        "unknown_records": 0,
        "cumulative_calls": {},
        "families": {},
        "toi_compared": False,
        "toi_pairs_compared": {"PT": 0, "EE": 0},
        "toi_value_mismatches": {"PT": 0, "EE": 0},
        "toi_nonfinite_values": {"PT": 0, "EE": 0},
        "toi_max_abs_diff": {"PT": 0.0, "EE": 0.0},
        "toi_max_rel_diff": {"PT": 0.0, "EE": 0.0},
        "toi_minimum_mismatches": 0,
        "toi_minimum_max_abs_diff": 0.0,
        "toi_tolerances": [],
    }
    if not path.is_file():
        raise ValueError("Audit mode produced no audit.jsonl")
    count_keys = (
        "actual_count",
        "baseline_count",
        "actual_duplicates",
        "baseline_duplicates",
        "missing",
        "extra",
    )
    ccd_single_sequences = set()
    toi_single_sequences = set()
    for line_number, line in enumerate(path.read_text().splitlines(), 1):
        if not line.strip():
            continue
        record = json.loads(line)
        if not isinstance(record, dict):
            raise ValueError(f"Audit line {line_number} is not an object")
        summary["records"] += 1
        stage = record.get(
            "phase", record.get("stage", record.get("event", ""))
        ).lower()
        stage = "detect" if stage in ("candidates", "candidate", "detect") else stage
        if stage in ("detect", "active", "toi"):
            summary[stage + "_records"] += 1
        else:
            summary["unknown_records"] += 1
        # Count path selection on detect records only; active follows the same detect.
        alpha = record.get("alpha")
        used = record.get("used_single_ef")
        if stage == "detect":
            if alpha == 0:
                summary["dcd_used_single_ef" if used is True else "dcd_fallback"] += 1
            elif isinstance(alpha, (int, float)) and alpha > 0:
                summary["ccd_used_single_ef" if used is True else "ccd_fallback"] += 1
                if used is True and "detect_sequence" in record:
                    ccd_single_sequences.add(record["detect_sequence"])
            requested_backend = record.get("requested_backend")
            executed_backend = record.get("executed_backend")
            if executed_backend is not None:
                if not isinstance(executed_backend, str):
                    raise ValueError(f"Audit line {line_number}: invalid executed_backend")
                summary["executed_backends"][executed_backend] = (
                    summary["executed_backends"].get(executed_backend, 0) + 1
                )
            if requested_backend == "optix":
                summary["optix_requested_detect_records"] += 1
            if requested_backend == "optix" or executed_backend == "optix":
                counters = {}
                for name in ("builds", "refits", "launches"):
                    value = record.get("optix_" + name)
                    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                        raise ValueError(f"Audit line {line_number}: invalid optix_{name}")
                    counters[name] = value
                    summary["optix_" + name] += value
                valid_execution = (
                    requested_backend == "optix"
                    and executed_backend == "optix"
                    and used is True
                    and counters["launches"] > 0
                    and counters["builds"] + counters["refits"] > 0
                )
                if valid_execution:
                    if alpha == 0:
                        summary["dcd_optix_launch_records"] += 1
                    elif isinstance(alpha, (int, float)) and alpha > 0:
                        summary["ccd_optix_launch_records"] += 1
                elif (
                    used is True
                    or executed_backend != "info_stackless_fallback"
                    or any(counters.values())
                ):
                    summary["optix_execution_errors"] += 1
                if record.get("optix_culling_statistics_enabled") is True:
                    if not valid_execution:
                        raise ValueError(f"Audit line {line_number}: culling counters without OptiX execution")
                    size, groups = record.get("optix_group_size"), record.get("optix_group_count")
                    if (isinstance(size, bool) or not isinstance(size, int)
                            or not 1 <= size <= 64 or isinstance(groups, bool)
                            or not isinstance(groups, int) or groups < 1):
                        raise ValueError(f"Audit line {line_number}: invalid OptiX grouping")
                    culling = {}
                    for name in ("group_tests", "body_rejections", "contact_rejections",
                                 "skipped_leaves", "leaf_tests", "emitted_pairs"):
                        value = record.get("optix_" + name)
                        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                            raise ValueError(f"Audit line {line_number}: invalid optix_{name}")
                        culling[name] = value
                        summary["optix_" + name] += value
                    rejections = culling["body_rejections"] + culling["contact_rejections"]
                    if rejections > culling["group_tests"] or culling["skipped_leaves"] < rejections:
                        raise ValueError(f"Audit line {line_number}: inconsistent OptiX culling counters")
                    if size == 1 and any(culling[key] for key in
                                        ("group_tests", "body_rejections", "contact_rejections", "skipped_leaves")):
                        raise ValueError(f"Audit line {line_number}: flat OptiX reports group culling")
                    summary["optix_culling_records"] += 1
                    summary["optix_grouped_detect_records"] += int(size > 1)
                    if size not in summary["optix_group_sizes"]:
                        summary["optix_group_sizes"].append(size)
                        summary["optix_group_sizes"].sort()
                    bounds = summary["optix_group_count_range"]
                    summary["optix_group_count_range"] = (
                        [groups, groups] if bounds is None else
                        [min(bounds[0], groups), max(bounds[1], groups)]
                    )
        for key in (
            "dcd_single_calls",
            "dcd_fallback_calls",
            "ccd_single_calls",
            "ccd_fallback_calls",
        ):
            if key in record:
                summary["cumulative_calls"][key] = max(
                    summary["cumulative_calls"].get(key, 0), int(record[key])
                )
        if not isinstance(record.get("equal"), bool):
            raise ValueError(f"Audit line {line_number}: missing equal flag")
        required = ("PT", "EE", "PP", "PE") if stage == "active" else ("PT", "EE")
        for family in required:
            if not isinstance(record.get(family, record.get(family.lower())), dict):
                raise ValueError(f"Audit line {line_number}: missing {family} family")
        mismatch = record.get("equal") is False
        if stage == "toi":
            if record.get("toi_compared") is not True:
                raise ValueError(
                    f"Audit line {line_number}: TOI values were not compared"
                )
            tolerance = [record.get("abs_tolerance"), record.get("rel_tolerance")]
            if any(
                isinstance(v, bool)
                or not isinstance(v, (int, float))
                or not math.isfinite(v)
                or v < 0
                for v in tolerance
            ):
                raise ValueError(f"Audit line {line_number}: invalid TOI tolerances")
            if tolerance not in summary["toi_tolerances"]:
                summary["toi_tolerances"].append(tolerance)
            minimum_diff = record.get("minimum_abs_diff")
            finite_minima = all(
                isinstance(v, (int, float))
                and not isinstance(v, bool)
                and math.isfinite(v)
                for v in (
                    record.get("actual_min"),
                    record.get("baseline_min"),
                    minimum_diff,
                )
            )
            minimum_mismatch = (
                record.get("minimum_equal") is not True or not finite_minima
            )
            summary["toi_minimum_mismatches"] += int(minimum_mismatch)
            mismatch |= minimum_mismatch
            if finite_minima:
                summary["toi_minimum_max_abs_diff"] = max(
                    summary["toi_minimum_max_abs_diff"], minimum_diff
                )
        nonempty = False
        for family in ("PT", "EE", "PP", "PE"):
            data = record.get(family, record.get(family.lower()))
            if not isinstance(data, dict):
                continue
            totals = summary["families"].setdefault(
                stage + "/" + family, {key: 0 for key in count_keys}
            )
            for key in count_keys:
                value = data.get(key, 0)
                if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                    raise ValueError(
                        f"Audit line {line_number}: invalid {family}/{key}"
                    )
                totals[key] += value
            nonempty |= (
                data.get("actual_count", 0) > 0 or data.get("baseline_count", 0) > 0
            )
            mismatch |= (
                data.get("equal") is False
                or data.get("missing", 0) > 0
                or data.get("extra", 0) > 0
                or data.get("actual_count") != data.get("baseline_count")
                or data.get("actual_duplicates", 0)
                != data.get("baseline_duplicates", 0)
            )
            if stage == "toi" and family in ("PT", "EE"):
                values = {}
                for key in (
                    "compared_pairs",
                    "value_mismatches",
                    "nonfinite_actual",
                    "nonfinite_baseline",
                ):
                    value = data.get(key)
                    if (
                        isinstance(value, bool)
                        or not isinstance(value, int)
                        or value < 0
                    ):
                        raise ValueError(
                            f"Audit line {line_number}: invalid TOI {family}/{key}"
                        )
                    values[key] = value
                summary["toi_pairs_compared"][family] += values["compared_pairs"]
                summary["toi_value_mismatches"][family] += values["value_mismatches"]
                summary["toi_nonfinite_values"][family] += (
                    values["nonfinite_actual"] + values["nonfinite_baseline"]
                )
                mismatch |= any(
                    values[key]
                    for key in (
                        "value_mismatches",
                        "nonfinite_actual",
                        "nonfinite_baseline",
                    )
                )
                mismatch |= values["compared_pairs"] != data["actual_count"]
                for key in ("max_abs_diff", "max_rel_diff"):
                    value = data.get(key)
                    if (
                        isinstance(value, bool)
                        or not isinstance(value, (int, float))
                        or not math.isfinite(value)
                        or value < 0
                    ):
                        raise ValueError(
                            f"Audit line {line_number}: invalid TOI {family}/{key}"
                        )
                    summary["toi_" + key][family] = max(
                        summary["toi_" + key][family], value
                    )
        summary["mismatches"] += int(mismatch)
        if stage in ("detect", "active") and nonempty:
            summary["nonempty_" + stage + "_records"] += 1
        is_single_ccd = isinstance(alpha, (int, float)) and alpha > 0 and used is True
        if stage == "detect" and is_single_ccd and nonempty:
            summary["nonempty_ccd_single_records"] += 1
        if stage == "toi" and is_single_ccd:
            summary["toi_single_ccd_records"] += 1
            summary["nonempty_toi_single_ccd_records"] += int(nonempty)
            if "detect_sequence" in record:
                toi_single_sequences.add(record["detect_sequence"])
        summary["toi_compared"] |= record.get("toi_compared") is True
    summary["ccd_single_detect_sequences"] = len(ccd_single_sequences)
    summary["unverified_ccd_single_detect_sequences"] = len(
        ccd_single_sequences - toi_single_sequences
    )
    return summary


def validate_swept_audit(
    summary: dict[str, Any], *, require_optix: bool = False
) -> None:
    """Require the new wrecking-ball path while keeping old audit files readable."""
    if summary["mismatches"]:
        raise ValueError("Candidate/active/TOI audit reported mismatches")
    if (
        not summary["dcd_used_single_ef"]
        or not summary["ccd_used_single_ef"]
        or not summary["nonempty_ccd_single_records"]
        or summary["ccd_fallback"]
    ):
        raise ValueError(
            "Wrecking-ball audit requires nonempty single-EF DCD/CCD; CCD fallback is not swept validation"
        )
    if (
        not summary["toi_compared"]
        or not summary["toi_single_ccd_records"]
        or not summary["nonempty_toi_single_ccd_records"]
        or not summary["ccd_single_detect_sequences"]
        or summary["unverified_ccd_single_detect_sequences"]
        or any(summary["toi_pairs_compared"][family] == 0 for family in ("PT", "EE"))
    ):
        raise ValueError(
            "Wrecking-ball audit lacks complete pairwise PT/EE TOI checks for single-EF CCD"
        )
    if not summary["nonempty_detect_records"] or not summary["nonempty_active_records"]:
        raise ValueError("Audit needs nonempty candidate and active records")
    if summary["unknown_records"]:
        raise ValueError("Audit contains unrecognized record phases")
    if require_optix and (
        summary["optix_requested_detect_records"] != summary["detect_records"]
        or summary["dcd_optix_launch_records"] != summary["dcd_used_single_ef"]
        or summary["ccd_optix_launch_records"] != summary["ccd_used_single_ef"]
        or summary["optix_execution_errors"]
        or summary["dcd_fallback"]
        or not summary["optix_builds"]
        or not summary["optix_launches"]
    ):
        raise ValueError(
            "OptiX audit requires actual OptiX builds and launches for every DCD/CCD detect"
        )


def collision_timer_summary(payload: dict[str, Any]) -> dict[str, Any]:
    """Aggregate collision timers in seconds, retaining legacy fallback labels."""
    names = {
        "BVH baseline DCD detect",
        "BVH baseline CCD detect",
        "BVH single EF DCD detect",
        "BVH single EF CCD detect",
        "BVH single EF CCD fallback detect",
    }
    totals: dict[str, dict[str, Any]] = {}

    def visit(node: dict[str, Any]) -> None:
        name = node.get("name")
        if name in names:
            duration, count = node.get("duration"), node.get("count")
            if (
                isinstance(duration, bool)
                or not isinstance(duration, (int, float))
                or not math.isfinite(duration)
                or duration < 0
                or isinstance(count, bool)
                or not isinstance(count, int)
                or count < 0
            ):
                raise ValueError(
                    "Diagnostic collision timer has invalid duration/count"
                )
            data = totals.setdefault(name, {"milliseconds": 0.0, "calls": 0})
            data["milliseconds"] += duration * 1000
            data["calls"] += count
        for child in node.get("children", []):
            visit(child)

    samples = payload["frame_samples"][payload["warmup"] :]
    if not samples:
        raise ValueError("No measured diagnostic frames")
    for sample in samples:
        visit(sample["timers"])
    return {
        name: {
            "ms_per_frame": value["milliseconds"] / len(samples),
            "calls": value["calls"],
            "ms_per_call": value["milliseconds"] / value["calls"]
            if value["calls"]
            else None,
        }
        for name, value in totals.items()
    }


def runtime_identity(
    payload: dict[str, Any], executable: Path, artifacts: list[Path]
) -> dict[str, Any]:
    reported_executable = payload.get("executable")
    reported_modules = payload.get("loaded_modules")
    if reported_executable is None or reported_modules is None:
        if sys.platform.startswith("linux"):
            raise ValueError("Linux native result lacks executable/loaded_modules")
        return {"available": False, "reason": "Native runtime paths unavailable"}
    if not isinstance(reported_executable, str) or not isinstance(
        reported_modules, list
    ):
        raise ValueError("Native runtime paths have invalid types")
    actual_executable = Path(reported_executable)
    if (
        not actual_executable.is_absolute()
        or actual_executable.resolve() != executable.resolve()
    ):
        raise ValueError(
            f"Actual executable {reported_executable} differs from selected {executable}"
        )
    if not all(
        isinstance(value, str) and Path(value).is_absolute()
        for value in reported_modules
    ):
        raise ValueError("loaded_modules must contain absolute paths")
    modules = sorted({Path(value).resolve() for value in reported_modules})
    backends = [path for path in modules if "backend_cuda" in path.name]
    selected = {path.resolve() for path in artifacts}
    if not backends or any(path not in selected for path in backends):
        raise ValueError(
            f"Runtime CUDA backend is outside selected artifacts: {backends}"
        )
    if any(not path.is_file() for path in [actual_executable, *modules]):
        raise ValueError("Runtime module/executable disappeared before hashing")
    return {
        "available": True,
        "executable": str(actual_executable.resolve()),
        "loaded_modules": fingerprint(modules),
        "cuda_backends": [str(p) for p in backends],
    }


def run_one(
    args: argparse.Namespace,
    directory: Path,
    method: str,
    frames: int,
    warmup: int,
    executable: Path,
    artifacts: list[Path],
) -> dict[str, Any]:
    directory.mkdir(parents=True, exist_ok=False)
    workspace = directory / "workspace"
    workspace.mkdir()
    result_path = directory / "native-result.json"
    env = dict(os.environ)
    env.pop(AUDIT_ENV, None)
    # Throughput and audit never inherit timer instrumentation from the shell.
    env["UIPC_BENCHMARK_TIMERS"] = "1" if args.phase == "diagnostic" else "0"
    env.pop("WB_TIMER", None)
    if args.phase == "audit":
        env[AUDIT_ENV] = str(directory / "audit.jsonl")
    argv = [
        "xmake",
        "run",
        "wrecking_ball",
        "--benchmark",
        "--method",
        method,
        "--frames",
        str(frames),
        "--warmup",
        str(warmup),
        "--result-json",
        str(result_path),
        "--workspace",
        str(workspace),
    ]
    if args.phase == "diagnostic":
        argv.append("--timers")
    result: dict[str, Any] = {
        "method": method,
        "directory": str(directory),
        "command": argv,
        "audit_enabled": args.phase == "audit",
    }
    dump(
        directory / "command.json",
        {
            **result,
            "environment_overrides": {
                AUDIT_ENV: env.get(AUDIT_ENV),
                "UIPC_BENCHMARK_TIMERS": env["UIPC_BENCHMARK_TIMERS"],
                "UIPC_OPTIX_EF_GROUP_SIZE": env.get("UIPC_OPTIX_EF_GROUP_SIZE"),
            },
        },
    )
    stop = threading.Event()
    sampler = threading.Thread(
        target=sample_gpu, args=(stop, directory / "gpu-samples.jsonl"), daemon=True
    )
    sampler.start()
    started = time.monotonic()
    try:
        with (
            (directory / "stdout.log").open("w") as stdout,
            (directory / "stderr.log").open("w") as stderr,
        ):
            process = subprocess.run(
                argv, cwd=ROOT, env=env, stdout=stdout, stderr=stderr, check=False
            )
        result["returncode"] = process.returncode
        if process.returncode:
            raise ValueError(f"Native process exited {process.returncode}")
        payload = json.loads(result_path.read_text())
        result["mean_ms_per_frame"] = validate_result(payload, method, frames, warmup)
        result["workload_means"] = workload_means(payload, warmup)
        result["runtime_identity"] = runtime_identity(payload, executable, artifacts)
        result["config"] = normalized_config(payload["config"])
        result["final_observable"] = payload["final_observable"]
        if args.phase == "diagnostic" and not all(
            "timers" in s for s in payload["frame_samples"]
        ):
            raise ValueError("Diagnostic run omitted per-frame timers")
        if args.phase == "diagnostic":
            result["collision_timers"] = collision_timer_summary(payload)
        if args.phase == "audit":
            result["audit"] = audit_summary(directory / "audit.jsonl")
            validate_swept_audit(
                result["audit"], require_optix=method == "single_ef_optix"
            )
        result["success"] = True
    except (OSError, ValueError, TypeError, KeyError) as error:
        result.update(success=False, error=str(error))
    finally:
        result["process_wall_s"] = time.monotonic() - started
        stop.set()
        sampler.join(timeout=6)
    dump(directory / "run-summary.json", result)
    print(
        json.dumps(
            {
                "run": directory.name,
                "method": method,
                "mean_ms_per_frame": result.get("mean_ms_per_frame"),
                "success": result["success"],
                "error": result.get("error"),
            }
        ),
        flush=True,
    )
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--phase", choices=("smoke", "throughput", "diagnostic", "audit"), required=True
    )
    parser.add_argument("--frames", type=int, default=300)
    parser.add_argument("--warmup", type=int, default=30)
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument(
        "--methods",
        nargs=2,
        choices=AVAILABLE_METHODS,
        default=METHODS,
        metavar=("BASELINE", "COMPARISON"),
        help="Two methods run in alternating order; audit runs selected Single-EF methods",
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--build-dir",
        type=Path,
        help="Artifact directory; native executable and CUDA backend must match its files",
    )
    args = parser.parse_args(argv)
    if args.methods[0] == args.methods[1]:
        parser.error("--methods must select two different methods")
    if args.phase == "smoke":
        args.frames, args.warmup = 3, 1
    if args.frames <= 0 or args.warmup < 0 or args.warmup >= args.frames:
        parser.error("require 0 <= warmup < frames")
    if args.phase == "throughput" and args.repeats < 5:
        parser.error("throughput requires --repeats >= 5 for each method")
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    session = output / (
        args.phase
        + "-"
        + dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    )
    session.mkdir()
    report: dict[str, Any] = {
        "benchmark": "native C++ wrecking_ball",
        "scene_note": "Original C++ scene ignores JSON density; ABD density=1000 and kappa=10MPa on both methods.",
        "phase": args.phase,
        "frames": args.frames,
        "warmup": args.warmup,
        "requested_methods": list(args.methods),
        "session": str(session),
        "runs": [],
        "failures": [],
    }
    try:
        executable, artifacts = artifact_paths(args.build_dir)
        inputs = [
            ROOT / "assets/sim_data/tetmesh" / f"{name}.msh"
            for name in ("cube", "ball", "link")
        ]
        inputs.append(ROOT / "apps/examples/wrecking_ball/wrecking_ball.json")
        report["before"] = {
            "repository": repository_state(output),
            "inputs": fingerprint(inputs),
            "artifacts": fingerprint(artifacts),
            "host": host_facts(),
            "executable": str(executable),
            "ldd": command(["ldd", str(executable)]),
            "xmake_config": fingerprint(xmake_config_paths()),
            "artifact_scope": "Selected build directory executable and local libuipc/native libraries; ldd is recorded separately. No claim that hashing alone proves runtime loading.",
        }
        if any(v.get("missing") for v in report["before"]["inputs"].values()):
            raise ValueError("Scene inputs are missing")
        dump(session / "metadata-before.json", report["before"])
        methods = (
            tuple(method for method in args.methods if method.startswith("single_ef_"))
            if args.phase == "audit"
            else tuple(args.methods)
        )
        report["executed_methods"] = list(methods)
        repeats = args.repeats if args.phase == "throughput" else 1
        for repeat in range(repeats):
            for method in methods:
                directory = session / f"{len(report['runs']) + 1:02d}-{method}"
                report["runs"].append(
                    run_one(
                        args,
                        directory,
                        method,
                        args.frames,
                        args.warmup,
                        executable,
                        artifacts,
                    )
                )
        report["after"] = {
            "repository": repository_state(output),
            "inputs": fingerprint(inputs),
            "artifacts": fingerprint(artifacts),
            "xmake_config": fingerprint(xmake_config_paths()),
            "host": host_facts(),
        }
        dump(session / "metadata-after.json", report["after"])
        for key in ("repository", "inputs", "artifacts", "xmake_config"):
            if report["before"][key] != report["after"][key]:
                report["failures"].append(f"{key} changed during the benchmark")
        valid = [r for r in report["runs"] if r["success"]]
        report["failures"].extend(
            r["directory"] + ": " + r.get("error", "failed")
            for r in report["runs"]
            if not r["success"]
        )
        if valid and any(r["config"] != valid[0]["config"] for r in valid[1:]):
            report["failures"].append(
                "Native configs differ beyond collision_detection/method"
            )
        if args.phase == "throughput":
            report["independent_final_drift"] = []
            for index in range(0, len(report["runs"]), 2):
                baseline, actual = report["runs"][index : index + 2]
                pair = {
                    "pair": index // 2 + 1,
                    "baseline_run": baseline["directory"],
                    "single_ef_run": actual["directory"],
                    "baseline_method": baseline["method"],
                    "comparison_method": actual["method"],
                }
                if baseline["success"] and actual["success"]:
                    try:
                        pair.update(
                            final_drift(
                                baseline["final_observable"], actual["final_observable"]
                            )
                        )
                    except (ValueError, TypeError, KeyError, OverflowError) as error:
                        pair["error"] = str(error)
                        report["failures"].append(
                            f"Final drift pair {pair['pair']}: {error}"
                        )
                else:
                    pair["unavailable"] = "At least one native run failed"
                report["independent_final_drift"].append(pair)
            report["drift_interpretation"] = (
                "These compare final states of independent simulations, not identical-state candidate sets. "
                "Finite numeric drift alone does not fail the benchmark. Same-state candidate/active audit establishes PT/EE equality. "
                "The transform RMS is the root mean square of per-body Frobenius norms."
            )
        report["summary"] = {}
        for method in methods:
            values = [r["mean_ms_per_frame"] for r in valid if r["method"] == method]
            if values:
                report["summary"][method] = {
                    "runs": len(values),
                    "run_means_ms": values,
                    "median_run_mean_ms": statistics.median(values),
                    "range_ms": [min(values), max(values)],
                    "run_workload_means": [
                        r["workload_means"] for r in valid if r["method"] == method
                    ],
                }
        report["interpretation"] = (
            "Use the median and range of per-run frame means. "
            "Small differences are inconclusive; inspect contention, clocks, trajectories and all frame samples. "
            "Timer and audit phases are instrumentation runs, not throughput results."
        )
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        report["failures"].append(str(error))
    dump(session / "summary.json", report)
    print(
        json.dumps(
            {
                "session": str(session),
                "summary": report.get("summary", {}),
                "failures": report["failures"],
            },
            indent=2,
        )
    )
    return 1 if report["failures"] else 0


if __name__ == "__main__":
    sys.exit(main())
