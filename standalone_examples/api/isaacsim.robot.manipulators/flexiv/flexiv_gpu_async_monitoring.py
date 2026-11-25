"""Low-overhead host/GPU resource sampling for GPU async fleet benchmarks."""

from __future__ import annotations

import csv
from collections import deque
from datetime import datetime, timezone
import math
import os
from pathlib import Path
import shutil
import subprocess
import threading
import time
from typing import Any, Iterable


RESOURCE_FIELDS = (
    "schema_version",
    "run_id",
    "manifest_generated_at",
    "config",
    "manifest",
    "num_envs",
    "robot_count",
    "timestamp_utc",
    "monotonic_s",
    "phase",
    "servo_cycle",
    "logical_cpu_count",
    "host_cpu_percent",
    "host_memory_used_percent",
    "host_memory_available_mb",
    "isaac_cpu_core_percent",
    "isaac_rss_mb",
    "controller_cpu_core_percent",
    "controller_rss_mb",
    "pipeline_cpu_core_percent",
    "controller_processes_expected",
    "controller_processes_alive",
    "gpu_index",
    "gpu_util_percent",
    "gpu_memory_used_mb",
    "gpu_power_w",
    "gpu_temperature_c",
    "gpu_query_ok",
    "gpu_query_error",
)


def append_csv_rows(path: Path, fieldnames: Iterable[str], rows: Iterable[dict[str, Any]]) -> None:
    """Append rows while emitting exactly one schema-checked header."""
    names = tuple(fieldnames)
    rendered = list(rows)
    if not rendered:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    write_header = not path.exists() or path.stat().st_size == 0
    if not write_header:
        with path.open(encoding="utf-8", newline="") as stream:
            existing = next(csv.reader(stream), [])
        if tuple(existing) != names:
            raise ValueError(
                f"CSV schema mismatch for [{path}]: existing={existing}, expected={list(names)}"
            )
    with path.open("a", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=names)
        if write_header:
            writer.writeheader()
        writer.writerows(rendered)


def _finite_values(samples: Iterable[dict[str, Any]], key: str) -> list[float]:
    result: list[float] = []
    for sample in samples:
        value = sample.get(key)
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            rendered = float(value)
            if math.isfinite(rendered):
                result.append(rendered)
    return result


def _percentile(values: list[float], percentile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    position = (len(ordered) - 1) * percentile / 100.0
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def summarize_resource_samples(samples: Iterable[dict[str, Any]]) -> dict[str, float | int]:
    """Return stable aggregate fields for one benchmark measurement window."""
    rows = list(samples)
    result: dict[str, float | int] = {"resource_samples": len(rows)}
    metrics = {
        "host_cpu_percent": "host_cpu",
        "host_memory_used_percent": "host_memory_used",
        "isaac_cpu_core_percent": "isaac_cpu_core",
        "isaac_rss_mb": "isaac_rss_mb",
        "controller_cpu_core_percent": "controller_cpu_core",
        "controller_rss_mb": "controller_rss_mb",
        "pipeline_cpu_core_percent": "pipeline_cpu_core",
        "gpu_util_percent": "gpu_util",
        "gpu_memory_used_mb": "gpu_memory_used_mb",
        "gpu_power_w": "gpu_power_w",
        "gpu_temperature_c": "gpu_temperature_c",
    }
    for source, prefix in metrics.items():
        values = _finite_values(rows, source)
        result[f"{prefix}_mean"] = sum(values) / len(values) if values else 0.0
        result[f"{prefix}_p95"] = _percentile(values, 95.0)
        result[f"{prefix}_max"] = max(values, default=0.0)
    result["gpu_resource_samples"] = sum(bool(row.get("gpu_query_ok")) for row in rows)
    result["controller_processes_alive_min"] = min(
        (int(row.get("controller_processes_alive", 0)) for row in rows),
        default=0,
    )
    result["controller_processes_expected_max"] = max(
        (int(row.get("controller_processes_expected", 0)) for row in rows),
        default=0,
    )
    return result


class ResourceMonitor:
    """Sample this Isaac process, its controller fleet, host memory, and one GPU."""

    def __init__(
        self,
        *,
        run_id: str,
        manifest_generated_at: str,
        config_path: Path,
        manifest_path: Path,
        num_envs: int,
        robot_count: int,
        output_path: Path,
        sample_interval_s: float,
        gpu_index: int,
        max_samples: int = 100_000,
    ) -> None:
        self._run_id = run_id
        self._manifest_generated_at = manifest_generated_at
        self._config_path = config_path
        self._manifest_path = manifest_path
        self._num_envs = num_envs
        self._robot_count = robot_count
        self._output_path = output_path
        self._sample_interval_s = sample_interval_s
        self._gpu_index = gpu_index
        self._logical_cpu_count = os.cpu_count() or 1
        self._clock_ticks = int(os.sysconf("SC_CLK_TCK"))
        self._page_size = int(os.sysconf("SC_PAGE_SIZE"))
        self._isaac_pid = os.getpid()
        self._controller_pids: tuple[int, ...] = ()
        self._phase = "startup"
        self._servo_cycle = 0
        self._context_lock = threading.Lock()
        self._sample_lock = threading.Lock()
        self._samples: deque[dict[str, Any]] = deque(maxlen=max_samples)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._previous_time: float | None = None
        self._previous_host: tuple[int, int] | None = None
        self._previous_process_ticks: dict[int, tuple[int, int]] = {}
        self._errors: deque[str] = deque(maxlen=32)
        self._nvidia_smi = shutil.which("nvidia-smi") or "/usr/bin/nvidia-smi"

    @property
    def errors(self) -> tuple[str, ...]:
        with self._context_lock:
            return tuple(self._errors)

    def set_controller_processes(self, launcher_pid: int, process_pids: Iterable[int]) -> None:
        pids = {int(pid) for pid in process_pids if int(pid) > 0}
        if launcher_pid > 0:
            pids.add(int(launcher_pid))
        with self._context_lock:
            self._controller_pids = tuple(sorted(pids))

    def set_context(self, phase: str, servo_cycle: int) -> None:
        with self._context_lock:
            self._phase = phase
            self._servo_cycle = int(servo_cycle)

    def begin_phase(self, phase: str, servo_cycle: int) -> None:
        """Start a measurement phase with fresh CPU utilization baselines."""
        self.set_context(phase, servo_cycle)
        with self._sample_lock:
            self._previous_time = None
            self._previous_host = None
            self._previous_process_ticks = {}
        self.sample_now()

    def start(self) -> None:
        if self._thread is not None:
            return
        self.sample_now()
        self._thread = threading.Thread(
            target=self._run, name="gpu-async-resource-monitor", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=max(2.0, self._sample_interval_s * 3.0))
            self._thread = None

    def samples(self, phase: str | None = None) -> list[dict[str, Any]]:
        with self._sample_lock:
            rows = list(self._samples)
        if phase is None:
            return rows
        return [row for row in rows if row.get("phase") == phase]

    def sample_now(self) -> dict[str, Any] | None:
        try:
            with self._sample_lock:
                sample = self._collect_sample()
                append_csv_rows(self._output_path, RESOURCE_FIELDS, [sample])
                self._samples.append(sample)
                return sample
        except Exception as error:  # Monitoring must not strand the simulation cleanup path.
            with self._context_lock:
                self._errors.append(f"{type(error).__name__}: {error}")
            return None

    def _run(self) -> None:
        while not self._stop.wait(self._sample_interval_s):
            self.sample_now()

    def _read_host_cpu(self) -> tuple[int, int]:
        fields = Path("/proc/stat").read_text(encoding="utf-8").splitlines()[0].split()
        values = [int(value) for value in fields[1:]]
        total = sum(values)
        idle = values[3] + (values[4] if len(values) > 4 else 0)
        return total, idle

    def _read_process(self, pid: int) -> tuple[int, int, float] | None:
        try:
            stat = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
            tail = stat[stat.rfind(")") + 2 :].split()
            ticks = int(tail[11]) + int(tail[12])
            start_ticks = int(tail[19])
            resident_pages = int(
                Path(f"/proc/{pid}/statm").read_text(encoding="utf-8").split()[1]
            )
        except (OSError, ValueError, IndexError):
            return None
        return ticks, start_ticks, resident_pages * self._page_size / (1024.0 * 1024.0)

    def _read_memory(self) -> tuple[float, float]:
        values: dict[str, int] = {}
        for line in Path("/proc/meminfo").read_text(encoding="utf-8").splitlines():
            label, raw = line.split(":", 1)
            if label in ("MemTotal", "MemAvailable"):
                values[label] = int(raw.split()[0])
        total = values["MemTotal"]
        available = values["MemAvailable"]
        return (total - available) * 100.0 / total, available / 1024.0

    @staticmethod
    def _number(value: str) -> float | None:
        try:
            rendered = float(value.strip())
        except ValueError:
            return None
        return rendered if math.isfinite(rendered) else None

    def _query_gpu(self) -> dict[str, Any]:
        try:
            result = subprocess.run(
                [
                    self._nvidia_smi,
                    f"--id={self._gpu_index}",
                    "--query-gpu=utilization.gpu,memory.used,power.draw,temperature.gpu",
                    "--format=csv,noheader,nounits",
                ],
                check=True,
                capture_output=True,
                text=True,
                timeout=3.0,
            )
            values = [self._number(value) for value in result.stdout.strip().split(",")]
            if len(values) != 4 or any(value is None for value in values):
                raise ValueError(f"unexpected nvidia-smi output: {result.stdout.strip()}")
            return {
                "gpu_util_percent": values[0],
                "gpu_memory_used_mb": values[1],
                "gpu_power_w": values[2],
                "gpu_temperature_c": values[3],
                "gpu_query_ok": True,
                "gpu_query_error": "",
            }
        except (OSError, subprocess.SubprocessError, ValueError) as error:
            return {
                "gpu_util_percent": None,
                "gpu_memory_used_mb": None,
                "gpu_power_w": None,
                "gpu_temperature_c": None,
                "gpu_query_ok": False,
                "gpu_query_error": f"{type(error).__name__}: {error}",
            }

    def _collect_sample(self) -> dict[str, Any]:
        now = time.monotonic()
        host = self._read_host_cpu()
        memory_used, memory_available = self._read_memory()
        with self._context_lock:
            phase = self._phase
            servo_cycle = self._servo_cycle
            controller_pids = self._controller_pids

        process_data: dict[int, tuple[int, int, float]] = {}
        for pid in (self._isaac_pid, *controller_pids):
            observed = self._read_process(pid)
            if observed is not None:
                process_data[pid] = observed

        elapsed = now - self._previous_time if self._previous_time is not None else None

        def cpu_for(pids: Iterable[int]) -> float | None:
            if elapsed is None or elapsed <= 0:
                return None
            delta_ticks = 0
            comparable = False
            for pid in pids:
                current = process_data.get(pid)
                previous = self._previous_process_ticks.get(pid)
                if current is None or previous is None or current[1] != previous[1]:
                    continue
                comparable = True
                delta_ticks += max(0, current[0] - previous[0])
            return delta_ticks * 100.0 / (self._clock_ticks * elapsed) if comparable else None

        host_cpu: float | None = None
        if self._previous_host is not None:
            total_delta = host[0] - self._previous_host[0]
            idle_delta = host[1] - self._previous_host[1]
            if total_delta > 0:
                host_cpu = (total_delta - idle_delta) * 100.0 / total_delta

        isaac_cpu = cpu_for((self._isaac_pid,))
        controller_cpu = cpu_for(controller_pids)
        pipeline_cpu = (
            isaac_cpu + controller_cpu
            if isaac_cpu is not None and controller_cpu is not None
            else None
        )
        isaac_rss = process_data.get(self._isaac_pid, (0, 0, 0.0))[2]
        controller_rss = sum(process_data[pid][2] for pid in controller_pids if pid in process_data)
        gpu = self._query_gpu()
        sample: dict[str, Any] = {
            "schema_version": 1,
            "run_id": self._run_id,
            "manifest_generated_at": self._manifest_generated_at,
            "config": str(self._config_path),
            "manifest": str(self._manifest_path),
            "num_envs": self._num_envs,
            "robot_count": self._robot_count,
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "monotonic_s": now,
            "phase": phase,
            "servo_cycle": servo_cycle,
            "logical_cpu_count": self._logical_cpu_count,
            "host_cpu_percent": host_cpu,
            "host_memory_used_percent": memory_used,
            "host_memory_available_mb": memory_available,
            "isaac_cpu_core_percent": isaac_cpu,
            "isaac_rss_mb": isaac_rss,
            "controller_cpu_core_percent": controller_cpu,
            "controller_rss_mb": controller_rss,
            "pipeline_cpu_core_percent": pipeline_cpu,
            "controller_processes_expected": len(controller_pids),
            "controller_processes_alive": sum(pid in process_data for pid in controller_pids),
            "gpu_index": self._gpu_index,
            **gpu,
        }
        self._previous_time = now
        self._previous_host = host
        self._previous_process_ticks = {
            pid: (data[0], data[1]) for pid, data in process_data.items()
        }
        return sample
