#!/usr/bin/env python3
"""Capture and replay identical production simplex queries without evolving A/B worlds."""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import statistics
import subprocess
import sys
import time
from typing import Any, Iterable, Mapping

ROOT = Path(__file__).resolve().parents[1]
METHODS = ("info_stackless_bvh", "single_ef_bvh")
POLICIES = ("natural", "refresh")
STATES = ("info_natural", "single_ef_natural", "info_refresh", "single_ef_refresh")
FAMILIES = ("pp", "pe", "ee", "pt")
COUNTERS = ("refreshes", "reuses", "topology_builds", "refresh_waits", "ef_retries",
            "typed_refills", "cross_frame_reuses", "fused_prefixes")
IDENTITY = ("request", "epoch", "frame", "newton_iteration", "alpha", "skin", "new_frame")
DEFAULT_INPUTS = ROOT / "output/benchmark-runs/xsquare-shared-query-cuda135-20261105/inputs"
DEFAULT_PROFILE_PREFIX = (
    "/opt/nvidia/nsight-systems/2026.5.1/target-linux-x64/nsys", "profile",
    "--trace=cuda", "--sample=none", "--cpuctxsw=none",
    "--capture-range=cudaProfilerApi", "--capture-range-end=stop", "--stats=false",
)


class BenchmarkError(RuntimeError):
    """A failed gate preserves its cohort and prevents automatic repetition."""


def require(condition: bool, message: str) -> None:
    if not condition:
        raise BenchmarkError(message)


def load(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, value: Any) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def sha256(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def hashes(paths: Iterable[Path]) -> dict[str, str]:
    return {str(p): sha256(p) for p in sorted(set(paths)) if p.is_file()}


def jsonl(path: Path) -> Iterable[dict[str, Any]]:
    with path.open(encoding="utf-8") as stream:
        for number, line in enumerate(stream, 1):
            require(bool(line.strip()), f"blank ledger row: {path}:{number}")
            yield json.loads(line)


def command(args: list[str], **kwargs: Any) -> str:
    return subprocess.check_output(args, cwd=ROOT, text=True, **kwargs).strip()


def nonnegative_integer(value: Any) -> bool:
    return type(value) is int and value >= 0


def checked_counts(value: Mapping[str, Any]) -> dict[str, int]:
    require(set(value) == set(FAMILIES), "candidate counts must name PP/PE/EE/PT")
    require(all(nonnegative_integer(value[k]) for k in FAMILIES), "invalid candidate count")
    return dict(value)


def state_name(method: str, policy: str) -> str:
    return ("info" if method == METHODS[0] else "single_ef") + "_" + policy


def validate_driver(folder: Path) -> dict[str, Any]:
    result = load(folder / "result.json")
    require(result.get("requested_frames") == 1000 and result.get("completed_frames") == 1000,
            "capture driver did not complete the requested 1000 frames")
    require(result.get("finite") is True and result.get("valid") is True,
            "capture driver has nonfinite or invalid state")
    rows = list(jsonl(folder / "frames.jsonl"))
    require([r.get("frame") for r in rows] == list(range(1, 1001)),
            "capture driver frame ledger is missing, repeated or reordered")
    require(all(r.get("finite") is True and r.get("valid") is True
                and r.get("stats", {}).get("completed") is True for r in rows),
            "capture driver has incomplete/invalid frame records")
    measured = rows[30:]
    return {"completed_frames": 1000, "measured_source_frames": 970,
            "converged_frames": sum(r["stats"].get("converged") is True for r in measured),
            "newton_limit_frames": [r["frame"] for r in measured if r["stats"].get("hit_newton_limit")],
            "line_search_limit_frames": [r["frame"] for r in measured if r["stats"].get("hit_line_search_limit")],
            "negative_J_frames": [r["frame"] for r in rows if (r.get("negative_tets") or 0) > 0],
            "minimum_tet_J": min((r["min_tet_J"] for r in rows if r.get("min_tet_J") is not None), default=None)}


def verification_rows(folder: Path) -> Iterable[dict[str, Any]]:
    result = load(folder / "verification.json")
    require(result.get("trace_complete") is True and result.get("success") is True
            and result.get("all_candidates_equal") is True
            and result.get("independent_oracle_equal") is True,
            "trace completeness or independent exact candidate equality failed")
    require(set(result.get("states", [])) == set(STATES), "verification omitted a method/policy state")
    total = 0
    previous = None
    for row in jsonl(folder / "requests.jsonl"):
        require(row.get("success") is True and row.get("independent_oracle_equal") is True,
                "independent candidate verification row failed")
        require(all(k in row for k in IDENTITY), "verification identity is incomplete")
        require(nonnegative_integer(row["request"]) and (previous is None or row["request"] == previous + 1),
                "verification requests are not contiguous")
        require(type(row["frame"]) is int and 1 <= row["frame"] <= 1000,
                "query frame is outside the completed driver ledger")
        require(set(row.get("states", {})) == set(STATES), "verification states are incomplete")
        expected = checked_counts(row["oracle_counts"])
        for name in STATES:
            state = row["states"][name]
            counts = checked_counts(state["counts"])
            require(expected is None or counts == expected, "verification family counts disagree")
            require(type(state.get("single_ef_active")) is bool, "effective strategy is missing")
            expected = counts
        previous = row["request"]
        total += 1
        yield row
    require(total > 0 and result.get("requests") == total, "trace request total disagrees with verification")


def validate_replay(folder: Path, verified: Path, method: str, policy: str,
                    profile: bool) -> dict[str, Any]:
    result = load(folder / "result.json")
    require(result.get("trace_complete") is True, "replay trace has no clean footer")
    require(result.get("method") == method and result.get("policy") == policy
            and result.get("profile") is profile, "replay settings differ from the requested experiment")
    ref = load(verified / "verification.json")
    values: list[float] = []
    bins: dict[str, list[float]] = {"refresh": [], "reuse": []}
    totals = dict.fromkeys(COUNTERS, 0)
    state = state_name(method, policy)
    request_count = fallback_count = measured_fallback_count = 0
    first_query_ms = None
    final_gauges: dict[str, Any] = {}
    for row, expected in itertools.zip_longest(jsonl(folder / "requests.jsonl"), verification_rows(verified)):
        require(row is not None and expected is not None, "timed and verified request counts differ")
        require(all(row.get(k) == expected[k] for k in IDENTITY), "timed request identity changed")
        require(checked_counts(row["counts"]) == expected["states"][state]["counts"],
                "timed output counts differ from exact verification")
        require(row.get("single_ef_active") == expected["states"][state]["single_ef_active"],
                "effective strategy differs from exact verification")
        measured = 31 <= row["frame"] <= 1000
        require(row.get("measured") is measured, "measured flag disagrees with source frame window")
        milliseconds = row.get("query_ms")
        require(type(milliseconds) in (float, int) and math.isfinite(milliseconds) and milliseconds >= 0,
                "invalid query duration")
        if first_query_ms is None:
            first_query_ms = float(milliseconds)
        for key in COUNTERS:
            before, after, delta = row["before"][key], row["after"][key], row["delta"][key]
            require(all(nonnegative_integer(v) for v in (before, after, delta)) and after - before == delta,
                    f"inconsistent cumulative counter {key}")
            require(after == expected["states"][state]["statistics"][key],
                    f"cache history differs from verification: {key}")
            if measured:
                totals[key] += delta
        if measured:
            values.append(float(milliseconds))
            if row["delta"]["refreshes"]:
                bins["refresh"].append(float(milliseconds))
            elif row["delta"]["reuses"]:
                bins["reuse"].append(float(milliseconds))
        fallback = int(method == METHODS[1] and not row["single_ef_active"])
        fallback_count += fallback
        measured_fallback_count += fallback if measured else 0
        final_gauges = row["gauges"]
        request_count += 1
    require(result.get("requests") == request_count == ref["requests"], "replay request total changed")
    require(result.get("measured_requests") == len(values) and values, "replay measured no expected requests")
    require(math.isclose(result["measured_query_ms"], sum(values), rel_tol=1e-10, abs_tol=1e-6),
            "replay aggregate duration disagrees with its ledger")
    require(result["trace_summary"]["complete"] is True
            and result["trace_summary"]["requests"] == ref["requests"], "trace summary is incomplete")
    for key in ("constructor_ms", "first_detect_ms", "cold_total_ms"):
        require(type(result.get(key)) in (int, float) and math.isfinite(result[key]) and result[key] >= 0,
                f"invalid cold metric: {key}")
    require(math.isclose(result["first_detect_ms"], first_query_ms, rel_tol=1e-10, abs_tol=1e-6)
            and math.isclose(result["cold_total_ms"], result["constructor_ms"] + result["first_detect_ms"],
                             rel_tol=1e-10, abs_tol=1e-6), "cold metrics disagree with query ledger")
    ordered = sorted(values)
    position = (len(ordered) - 1) * 0.95
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    p95 = ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)
    return {"requests": request_count, "measured_requests": len(values),
            "query_ms_total": sum(values), "mean_ms_per_query": statistics.mean(values),
            "p50_ms_per_query": statistics.median(values), "p95_ms_per_query": p95,
            "queries_per_second": len(values) * 1000 / sum(values) if sum(values) else None,
            "query_ms_per_source_frame": sum(values) / 970,
            "constructor_ms": result["constructor_ms"], "first_detect_ms": result["first_detect_ms"],
            "cold_total_ms": result["cold_total_ms"], "cold_samples": 1,
            "cumulative_counter_deltas": totals, "final_gauges": final_gauges,
            "gauges_note": "Snapshots, not accumulated traversal work; EF-only gauges are N/A for Info.",
            "effective_fallback_requests": fallback_count,
            "measured_fallback_requests": measured_fallback_count,
            "bins": {k: {"requests": len(v), "query_ms": sum(v)} for k, v in bins.items()},
            "full_scene_fps_newton_pcg": "not measured by query replay"}


def parse_compute(text: str) -> list[dict[str, Any]]:
    rows = []
    for line in text.splitlines():
        if not line.strip():
            continue
        fields = line.split(",", 2)
        require(len(fields) == 3 and fields[0].strip().isdigit(), "unrecognized nvidia-smi compute row")
        rows.append({"pid": int(fields[0]), "name": fields[1].strip(), "memory_mib": fields[2].strip()})
    return rows


def classify_compute(samples: list[dict[str, Any]], binary: Path,
                     known_pids: Iterable[int]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    own = set(known_pids)
    for sample in samples:
        own.update(r["pid"] for r in sample["compute"] if r["name"] == str(binary))
    foreign, unnamed = [], []
    for sample in samples:
        outsiders = [r for r in sample["compute"] if r["pid"] not in own]
        if outsiders:
            foreign.append({"utc": sample["utc"], "processes": outsiders})
        unnamed.extend({"utc": sample["utc"], **r} for r in sample["compute"]
                       if r["pid"] in own and r["name"] != str(binary))
    return foreign, unnamed


def gpu_sample() -> dict[str, Any]:
    gpu = command(["nvidia-smi", "--query-gpu=temperature.gpu,clocks.sm,clocks.mem,power.draw,utilization.gpu,memory.used",
                   "--format=csv,noheader,nounits"])
    require(len(gpu.splitlines()) == 1, "this benchmark requires one visible GPU")
    return {"utc": dt.datetime.now(dt.timezone.utc).isoformat(), "gpu": gpu,
            "compute": parse_compute(command(["nvidia-smi", "--query-compute-apps=pid,process_name,used_memory",
                                               "--format=csv,noheader,nounits"]))}


def process_children(pid: int) -> set[int]:
    found, pending = set(), [pid]
    while pending:
        current = pending.pop()
        for path in Path(f"/proc/{current}/task").glob("*/children"):
            try:
                children = {int(p) for p in path.read_text().split()}
            except (OSError, ValueError):
                continue
            new = children - found
            found.update(new)
            pending.extend(new)
    return found


def native_environment(runtime: Path) -> dict[str, str]:
    env = {k: v for k, v in os.environ.items()
           if not k.startswith(("UIPC_", "NSYS_", "CUDA_INJECTION"))}
    env["LD_LIBRARY_PATH"] = os.pathsep.join(map(str, [runtime, runtime / "dependencies"]))
    return env


def copy_immutable(source: Path, target: Path) -> None:
    if target.exists():
        require(sha256(source) == sha256(target), f"runtime basename collision: {target}")
        return
    shutil.copy2(source.resolve(), target)
    require(sha256(source) == sha256(target), f"runtime copy failed: {source}")
    target.chmod(0o555)


def source_inventory() -> dict[str, Any]:
    files: dict[str, str] = {}
    gitlinks: dict[str, dict[str, str]] = {}
    for entry in command(["git", "ls-files", "--stage", "-z"]).split("\0"):
        if not entry:
            continue
        indexed, relative = entry.split("\t", 1)
        mode, oid, stage = indexed.split()
        require(stage == "0", "unmerged index entry cannot qualify as frozen source")
        if mode == "160000":
            # A gitlink names a pinned commit, including when its directory is uninitialized.
            gitlinks[relative] = {"mode": mode, "oid": oid}
        else:
            path = ROOT / relative
            require(path.is_file(), f"indexed source file is missing: {relative}")
            files[str(path)] = sha256(path)
    return {"files": files, "gitlinks": gitlinks}


def profile_prefix(args: argparse.Namespace) -> list[str]:
    prefix = list(DEFAULT_PROFILE_PREFIX) if args.profile_prefix_json is None else load(args.profile_prefix_json)
    require(isinstance(prefix, list) and len(prefix) > 1 and all(isinstance(v, str) for v in prefix),
            "profile prefix must be a nonempty JSON argument list")
    require(Path(prefix[0]).is_absolute() and prefix[1] == "profile", "profile prefix must name an absolute nsys profile command")
    return prefix


def step_plan(output: Path, scenes: list[str], prefix: list[str]) -> list[dict[str, Any]]:
    binary = str(output / "runtime/uipc_test_backend_cuda")
    steps = []
    for phase in ("capture", "verify", "formal", "profile"):
        settings = [(s, None, None) for s in scenes] if phase in ("capture", "verify") else [
            (s, m, p) for s in scenes for p in POLICIES for m in METHODS]
        for scene, method, policy in settings:
            label = "-".join(str(v) for v in (phase, scene, policy, method) if v)
            folder = output / label
            env = {}
            if phase == "capture":
                tag = "[.single_ef_bvh_performance]"
                env = {"UIPC_SINGLE_EF_PERF_SCENE": scene, "UIPC_SINGLE_EF_PERF_METHOD": METHODS[0],
                       "UIPC_SINGLE_EF_PERF_PROFILE": "0", "UIPC_SINGLE_EF_PERF_OUTPUT": str(folder),
                       "UIPC_SINGLE_EF_PERF_ASSETS": str(output / "inputs/assets"),
                       "UIPC_SIMPLEX_QUERY_TRACE_OUTPUT": str(output / "traces" / (scene + ".bin"))}
            else:
                tag = "[.simplex_query_replay]"
                env = {"UIPC_SIMPLEX_QUERY_REPLAY_INPUT": str(output / "traces" / (scene + ".bin")),
                       "UIPC_SIMPLEX_QUERY_REPLAY_OUTPUT": str(folder),
                       "UIPC_SIMPLEX_QUERY_REPLAY_METHOD": method or METHODS[0],
                       "UIPC_SIMPLEX_QUERY_REPLAY_POLICY": policy or "natural",
                       "UIPC_SIMPLEX_QUERY_REPLAY_VERIFY": str(int(phase == "verify")),
                       "UIPC_SIMPLEX_QUERY_REPLAY_PROFILE": str(int(phase == "profile"))}
            argv = [binary, tag, "--reporter", "compact"]
            if phase == "profile":
                argv = [*prefix, "--output=" + str(output / "profiles" / label), *argv]
            steps.append({"label": label, "phase": phase, "scene": scene, "method": method,
                          "policy": policy, "folder": str(folder), "command": argv, "environment": env})
    return steps


class Cohort:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.out = args.output.resolve()
        self.manifest_path = self.out / "manifest.json"
        self.prefix = profile_prefix(args)
        self.plan = step_plan(self.out, args.scenes, self.prefix)
        if not self.out.exists():
            require(args.phase != "analyze", "analysis requires an existing cohort")
            self.freeze()
        else:
            require(args.phase not in ("all", "capture"), "existing cohort cannot be overwritten or automatically rerun")
            require(self.manifest_path.exists(), "existing output has no frozen manifest")
        self.manifest = load(self.manifest_path)
        require(self.manifest["scenes"] == args.scenes, "scene selection changed within the cohort")
        self.prefix = self.manifest["profile_prefix"]
        self.plan = step_plan(self.out, args.scenes, self.prefix)
        self.env = native_environment(self.out / "runtime")

    def freeze(self) -> None:
        require(not command(["git", "status", "--porcelain=v1"]), "commit source changes before freezing a cohort")
        require(self.args.build_provenance is not None, "--build-provenance is required for new measurements")
        provenance = load(self.args.build_provenance)
        require(bool(provenance.get("source_commit")), "build provenance has no source commit")
        subprocess.run(["git", "merge-base", "--is-ancestor", provenance["source_commit"], "HEAD"], cwd=ROOT, check=True)
        for relative, digest in provenance["source_files_sha256"].items():
            require(sha256(ROOT / relative) == digest, f"compiled source changed: {relative}")
        build = self.args.build_directory.resolve()
        for key, name in (("backend", "libuipc_backend_cuda.so"), ("test", "uipc_test_backend_cuda")):
            require(sha256(build / name) == provenance["native"][key]["sha256"], f"build artifact mismatch: {name}")
        self.out.parent.mkdir(parents=True, exist_ok=True)
        self.out.mkdir()
        for name in ("runtime", "runtime/dependencies", "traces", "attempts", "processes", "profiles"):
            (self.out / name).mkdir(exist_ok=True)
        shutil.copytree(self.args.inputs.resolve(), self.out / "inputs")
        origin = self.args.inputs.resolve().parent / "sample-snapshot.json"
        if origin.exists():
            snapshot = load(origin)
            for relative, value in snapshot.get("files", {}).items():
                require(sha256(self.out / "inputs" / relative) == value["sha256"],
                        f"frozen sample input changed: {relative}")
            write_json(self.out / "input-origin.json", snapshot)
        write_json(self.out / "build-provenance.json", provenance)
        runtime = self.out / "runtime"
        natives = [p for p in build.iterdir() if p.is_file() and (".so" in p.name or p.name == "uipc_test_backend_cuda")]
        require(any(p.name == "uipc_test_backend_cuda" for p in natives), "native CUDA test executable is absent")
        for p in natives:
            copy_immutable(p, runtime / p.name)
        discovery_env = native_environment(runtime)
        discovery_env["LD_LIBRARY_PATH"] += os.pathsep + os.pathsep.join(
            sorted({str(p.parent) for p in Path("/home/ligo/.xmake/packages").glob("**/libspdlog.so*")}))
        dependencies = set()
        for p in natives:
            linked = command(["ldd", str(runtime / p.name)], env=discovery_env)
            require("not found" not in linked, f"unresolved runtime dependency: {p.name}")
            (self.out / "processes" / (p.name + "-ldd.txt")).write_text(linked + "\n")
            for alias, raw in re.findall(r"^\s*(\S+)\s+=>\s+(/\S+)\s+\(", linked, re.M):
                dependency = Path(raw).resolve()
                if dependency.parent == runtime:
                    continue
                dependencies.add(dependency)
                if not str(dependency).startswith(("/usr/lib/", "/lib/", "/lib64/")):
                    copy_immutable(dependency, runtime / "dependencies" / alias)
            dependencies.update(Path(p).resolve() for p in re.findall(r"(/\S+)\s+\(", linked))
        final_env = native_environment(runtime)
        for native in (runtime / "uipc_test_backend_cuda", runtime / "libuipc_backend_cuda.so"):
            linked = command(["ldd", str(native)], env=final_env)
            require("not found" not in linked, "private runtime has unresolved dependencies")
            for line in linked.splitlines():
                if "libuipc_" in line:
                    require(str(runtime) in line, "native dependency escaped private runtime")
            dependencies.update(Path(p).resolve() for p in re.findall(r"(/\S+)\s+\(", linked))
        for pattern in ("libnvidia*.so*", "libcuda.so*"):
            dependencies.update(p.resolve() for p in Path("/usr/lib/x86_64-linux-gnu").glob(pattern))
        dependencies.update(p.resolve() for p in runtime.rglob("*.so*"))
        dependencies.add(runtime / "uipc_test_backend_cuda")
        profiler_root = Path(self.prefix[0]).parent
        profile_files = {p.resolve() for p in profiler_root.rglob("*.so*") if p.is_file()}
        profile_files.add(Path(self.prefix[0]))
        # Injection libraries may link additional system libraries absent from the native app.
        # Keep those in the profiler inventory rather than weakening formal-runtime coverage.
        tool_dependencies = set()
        for path in sorted(profile_files):
            if path == Path(self.prefix[0]) or any(token in path.name.lower()
                                                  for token in ("injection", "cupti", "nvperf", "nsys")):
                linked = subprocess.run(["ldd", str(path)], cwd=ROOT, env=final_env,
                                        text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
                tool_dependencies.update(Path(p).resolve() for p in re.findall(r"(/\S+)\s+\(", linked.stdout))
        profile_files.update(tool_dependencies)
        sources = source_inventory()
        before = {"schema_version": 1, "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
                  "head": command(["git", "rev-parse", "HEAD"]), "branch": command(["git", "branch", "--show-current"]),
                  "compiled_source_commit": provenance["source_commit"], "source_files": sources["files"],
                  "source_gitlinks": sources["gitlinks"],
                  "build_provenance_sha256": sha256(self.out / "build-provenance.json"),
                  "submodules": command(["git", "submodule", "status"]),
                  "runtime": hashes(dependencies), "profiler_runtime": hashes(profile_files),
                  "inputs": hashes(p for p in (self.out / "inputs").rglob("*") if p.is_file()),
                  "profile_prefix": self.prefix, "scenes": self.args.scenes,
                  "python": {"executable": sys.executable, "version": sys.version,
                             "sha256": sha256(Path(sys.executable))},
                  "relevant_environment": {k: v for k, v in os.environ.items()
                                           if k.startswith(("CUDA_", "UIPC_", "LD_", "NSYS_"))},
                  "cuda_module_loading": os.environ.get("CUDA_MODULE_LOADING"),
                  "removed_environment_prefixes": ["UIPC_", "NSYS_", "CUDA_INJECTION"],
                  "gpu": command(["nvidia-smi", "--query-gpu=index,uuid,name,driver_version,power.limit", "--format=csv"]),
                  "host": {"uname": command(["uname", "-a"]), "load": command(["uptime"]), "cpus": os.cpu_count()},
                  "protocol": {"driver_frames": 1000, "warmup_source_frames": 30, "measured_source_frames": [31, 1000],
                               "methods": METHODS, "policies": POLICIES, "runs_per_setting": 1,
                               "independent_profile": True, "idle_seconds": 15, "max_start_temperature_c": 45,
                               "telemetry_seconds": 1, "repeatability": "not assessed"}}
        write_json(self.manifest_path, before)
        write_json(self.out / "plan.json", self.plan)

    def guard(self) -> None:
        require(command(["git", "rev-parse", "HEAD"]) == self.manifest["head"], "HEAD changed during cohort")
        require(not command(["git", "status", "--porcelain=v1"]), "worktree changed during cohort")
        require(command(["git", "submodule", "status"]) == self.manifest["submodules"],
                "submodule checkout status changed")
        sources = source_inventory()
        require(sources["files"] == self.manifest["source_files"], "source fingerprints changed")
        require(sources["gitlinks"] == self.manifest["source_gitlinks"], "indexed gitlink pins changed")
        require(sha256(self.out / "build-provenance.json") == self.manifest["build_provenance_sha256"],
                "archived build proof changed")
        require(sha256(Path(self.manifest["python"]["executable"])) == self.manifest["python"]["sha256"],
                "Python interpreter changed")
        for key in ("runtime", "inputs", "profiler_runtime"):
            require(hashes(Path(p) for p in self.manifest[key]) == self.manifest[key], f"{key} fingerprints changed")
        for path in (self.out / "traces").glob("*-capture.json"):
            captured = load(path)
            require(sha256(Path(captured["trace"])) == captured["trace_sha256"], "frozen query trace changed")

    def wait_for_gpu(self, label: str) -> None:
        samples, idle_since = [], None
        while True:
            sample = gpu_sample()
            samples.append(sample)
            now = time.monotonic()
            if sample["compute"]:
                idle_since = None
            elif idle_since is None:
                idle_since = now
            write_json(self.out / "processes" / (label + "-preflight.json"), samples)
            if idle_since is not None and now - idle_since >= 15 and float(sample["gpu"].split(",")[0]) <= 45:
                return
            write_json(self.out / "status.json", {"waiting_for_gpu": label, "sample": sample,
                                                  "idle_seconds": 0 if idle_since is None else now - idle_since})
            time.sleep(5)

    def loaded_runtime(self, folder: Path, profile: bool) -> dict[str, Any]:
        files = set()
        maps = list(folder.glob("loaded-files-*.json"))
        require(bool(maps), "native process did not emit loaded-file provenance")
        for path in maps:
            files.update(Path(raw) for raw in load(path) if ".so" in Path(raw).name
                         or Path(raw).name == "uipc_test_backend_cuda")
        actual = hashes(p.resolve() for p in files if p.is_file())
        baseline = self.manifest["runtime"]
        injected = self.manifest["profiler_runtime"]
        allowed_injection_hashes = set(injected.values())
        extra = {p: h for p, h in actual.items() if p not in baseline}
        profiler_files = {p: h for p, h in extra.items() if profile and h in allowed_injection_hashes}
        uncovered = {p: h for p, h in extra.items() if p not in profiler_files}
        mismatch = {p: h for p, h in actual.items() if p in baseline and baseline[p] != h}
        runtime = self.out / "runtime"
        native = str(runtime / "libuipc_backend_cuda.so")
        escaped = [p for p in actual if Path(p).name.startswith("libuipc_") and Path(p).parent != runtime]
        require(actual.get(native) == baseline.get(native) and not escaped, "process loaded the wrong native backend")
        require(not uncovered and not mismatch, "loaded runtime escaped frozen native/profiler manifests")
        unreadable = sorted(str(p) for p in files if not p.is_file())
        require(profile or not unreadable, "formal process has unreadable shared-library provenance")
        return {"files": actual, "profiler_injection": profiler_files,
                "unreadable_mappings": unreadable,
                "complete_shared_library_provenance": not unreadable}

    def validate_profile_collection(self, label: str, marker: Path,
                                    validation: dict[str, Any]) -> dict[str, Any]:
        path = (self.out / "profiles" / (label + ".nsys-rep")).resolve()
        present = path.is_file()
        size = path.stat().st_size if present else 0
        status = "report_present_nonempty" if size else ("empty" if present else "missing")
        report = {"path": str(path), "bytes": size,
                  "sha256": sha256(path) if present else None,
                  "collection_status": status, "cuda_activity_validation": "pending",
                  "activity_validation_note": "File presence does not certify usable CUDA activity; inspect with supported Nsight report tools."}
        validation["profile_collection"] = report
        validation["state"] = "profile_file_collected" if size else "profile_collection_failed"
        write_json(marker, validation)
        require(size > 0, f"Nsight report is missing or empty: {path}; no automatic rerun")
        return report

    def run_process(self, step: dict[str, Any]) -> dict[str, Any]:
        label, folder = step["label"], Path(step["folder"])
        marker = self.out / "attempts" / (label + ".json")
        require(not folder.exists(), f"run output already exists: {label}")
        with marker.open("x", encoding="utf-8") as stream:
            json.dump({"state": "reserved", **step}, stream, indent=2)
        self.guard()
        self.wait_for_gpu(label)
        self.guard()
        env = {**self.env, **step["environment"]}
        samples, known = [], set()
        start = time.monotonic()
        print("START", label, flush=True)
        with (self.out / "processes" / (label + ".log")).open("w") as log, \
                (self.out / "processes" / (label + "-gpu.jsonl")).open("w") as telemetry:
            process = subprocess.Popen(step["command"], cwd=ROOT, env=env, stdout=log,
                                       stderr=subprocess.STDOUT, start_new_session=True)
            if step["phase"] != "profile":
                known.add(process.pid)
            try:
                while process.poll() is None:
                    candidates = process_children(process.pid) | {process.pid}
                    for pid in candidates:
                        try:
                            if Path(os.readlink(f"/proc/{pid}/exe")) == self.out / "runtime/uipc_test_backend_cuda":
                                known.add(pid)
                        except OSError:
                            pass
                    sample = gpu_sample()
                    samples.append(sample)
                    telemetry.write(json.dumps(sample) + "\n")
                    telemetry.flush()
                    write_json(self.out / "status.json", {"run": label, "pid": process.pid,
                                                          "elapsed_seconds": time.monotonic() - start})
                    time.sleep(1)
            except BaseException:
                # Stop only the process group created by this runner, preserving its artifacts.
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGTERM)
                        process.wait(timeout=15)
                raise
        foreign, unnamed = classify_compute(samples, self.out / "runtime/uipc_test_backend_cuda", known)
        validation = {"label": label, "returncode": process.returncode, "command": step["command"],
                      "environment": step["environment"], "elapsed_seconds": time.monotonic() - start,
                      "known_native_pids": sorted(known), "foreign_compute_samples": foreign,
                      "own_name_unavailable_samples": unnamed, "state": "process_failed"}
        write_json(marker, validation)
        require(process.returncode == 0, f"native process failed: {label}; no automatic rerun")
        if step["phase"] == "profile":
            self.validate_profile_collection(label, marker, validation)
        validation["loaded_runtime"] = self.loaded_runtime(folder, step["phase"] == "profile")
        self.guard()
        validation["state"] = "process_validated"
        validation["sources_runtime_inputs_unchanged"] = True
        write_json(marker, validation)
        print("END", label, "foreign_samples=" + str(len(foreign)), flush=True)
        return validation

    def validate_capture(self, scene: str) -> dict[str, Any]:
        result = validate_driver(self.out / ("capture-" + scene))
        trace = self.out / "traces" / (scene + ".bin")
        require(trace.is_file() and trace.stat().st_size > 0, "capture produced no query trace")
        result.update({"trace": str(trace), "trace_sha256": sha256(trace), "trace_bytes": trace.stat().st_size})
        saved = self.out / "traces" / (scene + "-capture.json")
        if saved.exists():
            require(load(saved) == result, "capture inputs or completion evidence changed")
        else:
            write_json(saved, result)
        return result

    def validate_verification(self, scene: str) -> dict[str, Any]:
        self.validate_capture(scene)
        folder = self.out / ("verify-" + scene)
        count = sum(1 for _ in verification_rows(folder))
        result = load(folder / "verification.json")
        require(result["requests"] == count, "verification request count changed")
        return result

    def execute(self) -> None:
        phases = ("capture", "verify", "formal", "profile") if self.args.phase == "all" else (self.args.phase,)
        for phase in phases:
            for step in (s for s in self.plan if s["phase"] == phase):
                scene = step["scene"]
                if phase != "capture":
                    self.validate_capture(scene)
                if phase in ("formal", "profile"):
                    self.validate_verification(scene)
                self.run_process(step)
                if phase == "capture":
                    self.validate_capture(scene)
                elif phase == "verify":
                    self.validate_verification(scene)
                else:
                    metrics = validate_replay(Path(step["folder"]), self.out / ("verify-" + scene),
                                              step["method"], step["policy"], phase == "profile")
                    write_json(Path(step["folder"]) / "validated-metrics.json", metrics)
            self.summarize()
        self.guard()
        write_json(self.out / "status.json", {"finished_phase": self.args.phase, "successful": True})

    def summarize(self) -> None:
        result: dict[str, Any] = {"head": self.manifest["head"], "compiled_source_commit": self.manifest["compiled_source_commit"],
                                  "protocol": self.manifest["protocol"], "captures": {}, "runs": {}, "comparisons": {}}
        for scene in self.args.scenes:
            if (self.out / "traces" / (scene + "-capture.json")).exists():
                result["captures"][scene] = load(self.out / "traces" / (scene + "-capture.json"))
        for step in self.plan:
            path = Path(step["folder"]) / "validated-metrics.json"
            if path.exists():
                validation = load(self.out / "attempts" / (step["label"] + ".json"))
                result["runs"][step["label"]] = {**load(path), "profile": step["phase"] == "profile",
                                                  "foreign_compute_samples": len(validation["foreign_compute_samples"])}
        for phase, scene, policy in itertools.product(("formal", "profile"), self.args.scenes, POLICIES):
            labels = [f"{phase}-{scene}-{policy}-{m}" for m in METHODS]
            if all(label in result["runs"] for label in labels):
                a, b = [result["runs"][label] for label in labels]
                before, after = a["query_ms_total"], b["query_ms_total"]
                require(before > 0 and after > 0, "comparison measured zero query cost")
                result["comparisons"][f"{phase}-{scene}-{policy}"] = {
                    "query_time_reduction_percent": 100 * (before - after) / before,
                    "speedup": before / after,
                    "sampled_foreign_compute": a["foreign_compute_samples"] + b["foreign_compute_samples"],
                    "effective_fallback_requests": a["effective_fallback_requests"] + b["effective_fallback_requests"],
                    "clean_sampled_compute_comparison": a["foreign_compute_samples"] + b["foreign_compute_samples"] == 0,
                    "actual_measured_ef_comparison": b["measured_fallback_requests"] == 0,
                    "stable_or_full_scene_speedup_established": False}
        write_json(self.out / "summary.json", result)


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("output", type=Path, help="fresh cohort directory, or an existing cohort for a later explicit phase")
    result.add_argument("--phase", choices=("all", "capture", "verify", "formal", "profile", "analyze"), default="all")
    result.add_argument("--scenes", nargs="+", choices=("wb", "ex88"), default=["wb", "ex88"])
    result.add_argument("--build-directory", type=Path, default=ROOT / "build/linux/x86_64/release")
    result.add_argument("--build-provenance", type=Path)
    result.add_argument("--inputs", type=Path, default=DEFAULT_INPUTS, help="frozen inputs directory containing assets/")
    result.add_argument("--profile-prefix-json", type=Path, help="approved nsys profile argument list, before --output and target")
    result.add_argument("--dry-run", action="store_true", help="print the complete plan without creating files or querying GPU")
    return result


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    require(len(set(args.scenes)) == len(args.scenes), "scene selection contains duplicates")
    if args.dry_run:
        steps = step_plan(args.output.resolve(), args.scenes, profile_prefix(args))
        if args.phase != "all":
            steps = [step for step in steps if step["phase"] == args.phase]
        print(json.dumps({"output": str(args.output.resolve()), "steps": steps}, indent=2))
        return 0
    cohort = Cohort(args)
    if args.phase == "analyze":
        cohort.summarize()
    else:
        cohort.execute()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (BenchmarkError, OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        print(f"Benchmark stopped: {error}", file=sys.stderr)
        raise SystemExit(1)
