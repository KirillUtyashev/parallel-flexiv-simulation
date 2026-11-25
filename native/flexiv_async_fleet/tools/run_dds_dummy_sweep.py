#!/usr/bin/env python3
"""Sweep exact Sim Plugin DDS peers against the shared async UserNode participant."""

from __future__ import annotations

import argparse
import csv
import importlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import time

RESULT_PREFIX = "DDS_DUMMY_JSON="
PEER_RESULT_RE = re.compile(
    r"DDS_DUMMY_RESULT requested=(\d+) created=(\d+) connected=(\d+) "
    r"creation_error=(\S+) creation_errors=(\d+)"
)
DEFAULT_COUNTS = (1, 32, 64, 96, 100, 101, 112, 128, 256)
_FAST_EXIT_KEEPALIVE: list[object] = []


def serials(count: int) -> list[str]:
    return [f"Rizon4s-D{i:05d}" for i in range(count)]


def terminate(process: subprocess.Popen[str]) -> None:
    if process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5)


def run_single(args: argparse.Namespace) -> dict[str, object]:
    sys.path.insert(0, str(args.module_dir))
    module = importlib.import_module("_flexiv_async_fleet")
    if not getattr(module, "dds_shared_participant", False):
        raise RuntimeError("async module was not built with a shared DDS participant")
    if not args.peer_executable.is_file():
        raise RuntimeError(f"dummy peer executable not found: {args.peer_executable}")

    robot_serials = serials(args.single)
    maximum_user_connected = 0
    connected_user_serials: set[str] = set()
    maximum_discovered = 0
    start = time.monotonic()

    fleet = module.AsyncFleet(
        robot_serials, 7, queue_capacity=4, command_wait_timeout_ms=2
    )
    peers: list[subprocess.Popen[str]] = []
    peer_logs: list[object] = []

    def sample_connections() -> None:
        nonlocal maximum_user_connected, maximum_discovered
        statistics = fleet.statistics()
        connected_user_serials.update(
            item["serial"] for item in statistics if item["connected"]
        )
        maximum_user_connected = max(
            maximum_user_connected,
            sum(bool(item["connected"]) for item in statistics),
        )
        diagnostics = module.shared_dds_diagnostics()
        maximum_discovered = max(
            maximum_discovered, int(diagnostics["discovered_participants"])
        )

    def launch_peer(peer_serials: list[str]) -> None:
        peer_log = tempfile.TemporaryFile(mode="w+", encoding="utf-8")
        peer_logs.append(peer_log)
        peers.append(
            subprocess.Popen(
                [
                    str(args.peer_executable),
                    "--timeout-ms",
                    str(args.timeout_ms),
                    "--hold-ms",
                    str(args.hold_ms),
                    "--creation-parallelism",
                    str(args.creation_parallelism),
                    *peer_serials,
                ],
                stdout=peer_log,
                stderr=subprocess.STDOUT,
                text=True,
                env=os.environ.copy(),
            )
        )

    try:
        worker_deadline = time.monotonic() + 10
        while time.monotonic() < worker_deadline:
            statistics = fleet.statistics()
            if len(statistics) == args.single and all(
                item["worker_running"] and item["phase"] != "initializing"
                for item in statistics
            ):
                break
            time.sleep(0.01)
        else:
            raise RuntimeError("not all UserNode workers initialized within 10 seconds")

        if args.peer_layout == "packed":
            launch_peer(robot_serials)
        else:
            wave_size = args.peer_wave_size or len(robot_serials)
            for index, serial in enumerate(robot_serials, start=1):
                launch_peer([serial])
                sample_connections()
                if (
                    index < len(robot_serials)
                    and index % wave_size == 0
                    and args.peer_inter_wave_delay_ms > 0
                ):
                    delay_deadline = (
                        time.monotonic() + args.peer_inter_wave_delay_ms / 1000
                    )
                    while time.monotonic() < delay_deadline:
                        sample_connections()
                        time.sleep(0.01)

        deadline = time.monotonic() + args.timeout_ms / 1000 + 15
        while any(peer.poll() is None for peer in peers) and time.monotonic() < deadline:
            sample_connections()
            time.sleep(0.01)
        if any(peer.poll() is None for peer in peers):
            for peer in peers:
                terminate(peer)
            raise RuntimeError("dummy peer process exceeded its experiment deadline")

        peer_outputs: list[str] = []
        peer_returncodes: list[int] = []
        for peer, peer_log in zip(peers, peer_logs):
            peer.wait(timeout=5)
            peer_returncodes.append(peer.returncode)
            peer_log.flush()
            peer_log.seek(0)
            peer_outputs.append(peer_log.read())

        # Take one last snapshot while the shared participant is still alive.
        sample_connections()
        diagnostics = module.shared_dds_diagnostics()
        peer_requested = 0
        peer_created = 0
        peer_connected = 0
        creation_error_count = 0
        failed_logs: list[str] = []
        for peer_output, returncode in zip(peer_outputs, peer_returncodes):
            peer_matches = list(PEER_RESULT_RE.finditer(peer_output))
            if len(peer_matches) != 1:
                creation_error_count += 1
                failed_logs.append(peer_output[-1000:])
                continue
            match = peer_matches[0]
            requested, created, connected = map(int, match.groups()[:3])
            peer_requested += requested
            peer_created += created
            peer_connected += connected
            creation_error_count += int(match.group(5))
            if returncode != 0 or match.group(4) != "none":
                failed_logs.append(peer_output[-1000:])

        peer_process_failures = sum(returncode != 0 for returncode in peer_returncodes)
        allocation = diagnostics["participant_allocation"]
        passed = (
            peer_process_failures == 0
            and peer_requested == args.single
            and peer_created == args.single
            and peer_connected == args.single
            and len(connected_user_serials) == args.single
            and creation_error_count == 0
        )
        return {
            "robots": args.single,
            "peer_layout": args.peer_layout,
            "peer_processes": len(peers),
            "peer_process_failures": peer_process_failures,
            "peer_created": peer_created,
            "peer_creation_errors": creation_error_count,
            "peer_connected": peer_connected,
            "user_connected": len(connected_user_serials),
            "user_connected_simultaneous_max": maximum_user_connected,
            "shared_discovered_participants": maximum_discovered,
            "participant_allocation_initial": int(allocation[0]),
            "participant_allocation_maximum": int(allocation[1]),
            "participant_allocation_increment": int(allocation[2]),
            "passed": passed,
            "elapsed_s": round(time.monotonic() - start, 3),
            "peer_log_tail": "" if passed else "\n---\n".join(failed_logs)[-4000:],
        }
    finally:
        for peer in peers:
            terminate(peer)
        for peer_log in peer_logs:
            peer_log.close()
        if args.fast_child_exit:
            # The sweep runs every size in a disposable process. Keep the fleet
            # alive until os._exit() so per-UserNode DDS teardown cannot dominate
            # the measurement or add minutes to large diagnostic sweeps.
            _FAST_EXIT_KEEPALIVE.append(fleet)
        else:
            fleet.stop()


def parse_child_result(stdout: str) -> dict[str, object]:
    markers = [line for line in stdout.splitlines() if line.startswith(RESULT_PREFIX)]
    if len(markers) != 1:
        raise RuntimeError(f"child emitted {len(markers)} result markers")
    return json.loads(markers[0][len(RESULT_PREFIX):])


def run_sweep(args: argparse.Namespace) -> int:
    rows: list[dict[str, object]] = []
    for count in args.counts:
        command = [
            sys.executable,
            str(Path(__file__).resolve()),
            "--single",
            str(count),
            "--module-dir",
            str(args.module_dir),
            "--peer-executable",
            str(args.peer_executable),
            "--timeout-ms",
            str(args.timeout_ms),
            "--hold-ms",
            str(args.hold_ms),
            "--creation-parallelism",
            str(args.creation_parallelism),
            "--peer-layout",
            args.peer_layout,
            "--peer-wave-size",
            str(args.peer_wave_size),
            "--peer-inter-wave-delay-ms",
            str(args.peer_inter_wave_delay_ms),
            "--fast-child-exit",
        ]
        result = subprocess.run(
            command,
            text=True,
            capture_output=True,
            timeout=args.timeout_ms / 1000 + 30,
        )
        try:
            row = parse_child_result(result.stdout)
        except (RuntimeError, json.JSONDecodeError) as error:
            raise RuntimeError(
                f"DDS dummy child failed for {count} robots: {error}; "
                f"stderr tail: {result.stderr[-2000:]}"
            ) from error
        rows.append(row)
        print(
            f"robots={count} discovered={row['shared_discovered_participants']} "
            f"matched={row['user_connected']} passed={row['passed']} "
            f"elapsed_s={row['elapsed_s']}",
            flush=True,
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        if not row["passed"]:
            print(f"stopping at first failed size; partial CSV: {args.output}")
            return 2
    print(f"DDS dummy sweep passed; CSV: {args.output}")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--single", type=int)
    parser.add_argument("--counts", type=int, nargs="+", default=DEFAULT_COUNTS)
    parser.add_argument(
        "--module-dir",
        type=Path,
        default=Path(__file__).resolve().parents[3] / "build/async_fleet_py311",
    )
    parser.add_argument("--peer-executable", type=Path)
    parser.add_argument("--timeout-ms", type=int, default=10_000)
    parser.add_argument("--hold-ms", type=int, default=1_000)
    parser.add_argument("--creation-parallelism", type=int, default=16)
    parser.add_argument(
        "--peer-layout", choices=("packed", "processes"), default="packed"
    )
    parser.add_argument("--peer-wave-size", type=int, default=0)
    parser.add_argument("--peer-inter-wave-delay-ms", type=int, default=0)
    parser.add_argument("--fast-child-exit", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("/tmp/flexiv-dds-dummy-sweep.csv"),
    )
    args = parser.parse_args()
    args.module_dir = args.module_dir.expanduser().resolve()
    if args.peer_executable is None:
        args.peer_executable = args.module_dir / "flexiv_dds_dummy_peers"
    else:
        args.peer_executable = args.peer_executable.expanduser().resolve()
    args.output = args.output.expanduser().resolve()
    if args.single is not None and args.single < 1:
        parser.error("--single must be positive")
    if any(count < 1 for count in args.counts):
        parser.error("--counts must contain only positive values")
    if args.timeout_ms < 100 or args.hold_ms < 0:
        parser.error("timeout must be at least 100 ms and hold cannot be negative")
    if args.creation_parallelism < 1:
        parser.error("--creation-parallelism must be positive")
    if args.peer_wave_size < 0 or args.peer_inter_wave_delay_ms < 0:
        parser.error("peer wave size and delay cannot be negative")
    return args


def main() -> int:
    args = parse_args()
    try:
        if args.single is not None:
            print(
                RESULT_PREFIX + json.dumps(run_single(args), sort_keys=True),
                flush=True,
            )
            if args.fast_child_exit:
                os._exit(0)
            return 0
        return run_sweep(args)
    except (OSError, RuntimeError, subprocess.SubprocessError) as error:
        print(f"ERROR: {error}", file=sys.stderr, flush=True)
        if getattr(locals().get("args"), "fast_child_exit", False):
            os._exit(1)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
