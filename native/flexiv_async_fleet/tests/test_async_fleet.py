#!/usr/bin/env python3
"""Disconnected-endpoint smoke test for the native asynchronous fleet module."""

from __future__ import annotations

import argparse
import time

import numpy as np

from _flexiv_async_fleet import AsyncFleet


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robots", type=int, default=4)
    parser.add_argument("--cycles", type=int, default=200)
    args = parser.parse_args()
    if args.robots < 1 or args.cycles < 1:
        parser.error("--robots and --cycles must be positive")

    serials = [f"Rizon4s-ASY{i:03d}" for i in range(args.robots)]
    q = np.zeros((args.robots, 7), dtype=np.float32)
    dq = np.zeros_like(q)
    efforts = np.empty_like(q)
    cycles = np.empty(args.robots, dtype=np.uint64)
    connected = np.empty(args.robots, dtype=np.uint8)
    valid = np.empty(args.robots, dtype=np.uint8)

    start = time.perf_counter()
    with AsyncFleet(serials, 7, queue_capacity=8, command_wait_timeout_ms=2) as fleet:
        for cycle in range(1, args.cycles + 1):
            fleet.publish_states(cycle, q, dq)
            assert fleet.wait_for_command_cycle(cycle, 100) == 0
            fleet.read_commands(efforts, cycles, connected, valid)
            time.sleep(0.001)
        statistics = fleet.statistics()
    elapsed = time.perf_counter() - start

    assert len(statistics) == args.robots
    assert all(item["worker_error"] == "" for item in statistics)
    assert all(item["states_published"] == args.cycles for item in statistics)
    assert all(item["queue_high_watermark"] <= 1 for item in statistics)
    assert all(item["state_queue_overflows"] == 0 for item in statistics)
    assert all(item["commands_received"] == 0 for item in statistics)
    assert all(item["command_packets_received"] == 0 for item in statistics)
    assert all(item["synchronization_events"] == 0 for item in statistics)
    assert all(item["synchronization_command_discards"] == 0 for item in statistics)
    assert all(item["recovery_events"] == 0 for item in statistics)
    assert all(item["phase"] != "error" for item in statistics)
    assert not connected.any()
    assert not valid.any()
    print(
        f"ASYNC_FLEET_SMOKE_PASS robots={args.robots} cycles={args.cycles} "
        f"elapsed_s={elapsed:.3f}"
    )


if __name__ == "__main__":
    main()
