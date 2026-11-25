#!/usr/bin/env python3
"""Create an optimized private FlexivSimulation binary for one fleet run."""

from __future__ import annotations

import argparse
import hashlib
import math
import mmap
from pathlib import Path
import shutil
import struct
import sys


DEFAULT_MAX_MESSAGE_SIZE = 65_500
DEFAULT_INITIAL_PEERS_RANGE = 4
DEFAULT_MUTATION_TRIES = 100
DEFAULT_SIMULATION_TIMESTEP_S = 0.0005
DEFAULT_MIDDLEWARE_TIMESTEP_S = 0.001
# UDPTransportDescriptor's first four uint32 fields in the supported binary.
# Requiring this complete, unique signature makes the private patch fail closed
# if Elements Studio changes its embedded Fast DDS layout.
UDP_DESCRIPTOR_SIGNATURE = struct.pack(
    "<IIII", DEFAULT_MAX_MESSAGE_SIZE, DEFAULT_INITIAL_PEERS_RANGE, 0, 0
)
# In the supported FlexivSimulation build, DefaultNode's inlined
# BuiltinAttributes constructor writes mutation_tries=100 to its stack QoS.
# The full instruction (including its unique stack offset) is required so a
# different Elements Studio build fails closed instead of patching an
# unrelated integer constant.
DEFAULT_NODE_MUTATION_SIGNATURE = bytes.fromhex("c78424b802000064000000")

# The supported FlexivSimulation build keeps its 500 us simulation/joint-control
# timestep immediately before the 1 ms RobotControlApp middleware timestep. The
# complete pair is unique. The simulation value feeds rigid-body, flexible-joint,
# and joint-control timestep setters.
TIMESTEP_CONSTANT_SIGNATURE = struct.pack(
    "<dd", DEFAULT_SIMULATION_TIMESTEP_S, DEFAULT_MIDDLEWARE_TIMESTEP_S
)

# The middleware and interaction-middleware setters load the shared 1 ms constant
# from .rodata. That constant is also used by embedded Fast DDS time conversions,
# so changing it globally would corrupt unrelated timing. For a non-default
# simulation timestep, redirect only these two unique loads to the simulation
# constant eight bytes earlier.
MIDDLEWARE_TIMESTEP_LOAD_SIGNATURE = bytes.fromhex("488b0568a24d00")
MIDDLEWARE_TIMESTEP_LOAD_PATCH = bytes.fromhex("488b0560a24d00")
INTERACTION_TIMESTEP_LOAD_SIGNATURE = bytes.fromhex("488b054da24d00")
INTERACTION_TIMESTEP_LOAD_PATCH = bytes.fromhex("488b0545a24d00")

# ArmSimulation::Run starts RunBlocking in a worker thread, stores that thread,
# then spins on its running atomic before tail-calling std::thread::join(). The
# join already provides the required blocking behavior. Replacing only the
# backward conditional branch with NOPs makes the parent thread enter join
# immediately instead of consuming one complete CPU core per simulated arm.
# Require the surrounding instructions as a unique signature so an updated
# Elements Studio binary fails closed.
RUN_BUSY_WAIT_SIGNATURE = bytes.fromhex(
    "0fb60384c075f9488b44241864482b042528000000"
)
RUN_BUSY_WAIT_PATCH = bytes.fromhex(
    "0fb60384c09090488b44241864482b042528000000"
)


class PreparationError(RuntimeError):
    """Raised when a private FlexivSimulation binary cannot be prepared safely."""


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def unique_offset(image: mmap.mmap, signature: bytes, label: str) -> int:
    offset = image.find(signature)
    if offset < 0:
        raise PreparationError(f"supported {label} signature was not found")
    if image.find(signature, offset + 1) >= 0:
        raise PreparationError(f"supported {label} signature is ambiguous")
    return offset


def prepare(
    source: Path,
    output: Path,
    max_initial_peers_range: int,
    mutation_tries: int,
    simulation_timestep_s: float = DEFAULT_SIMULATION_TIMESTEP_S,
    optimize_run_wait: bool = True,
) -> tuple[str, str]:
    source = source.resolve()
    output = output.resolve()
    if not source.is_file():
        raise PreparationError(f"FlexivSimulation source not found: {source}")
    if source == output:
        raise PreparationError("the installed FlexivSimulation binary must remain untouched")
    if not 4 <= max_initial_peers_range <= 65_535:
        raise PreparationError("DDS maximum initial peers range must be in [4, 65535]")
    if not DEFAULT_MUTATION_TRIES <= mutation_tries <= 65_535:
        raise PreparationError("DDS mutation tries must be in [100, 65535]")
    if not math.isfinite(simulation_timestep_s) or simulation_timestep_s <= 0.0:
        raise PreparationError("simulation timestep must be finite and positive")
    output.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, output)
    try:
        with output.open("r+b") as stream:
            with mmap.mmap(stream.fileno(), 0) as image:
                if max_initial_peers_range != DEFAULT_INITIAL_PEERS_RANGE:
                    offset = unique_offset(
                        image, UDP_DESCRIPTOR_SIGNATURE, "UDP transport descriptor"
                    )
                    image[offset + 4 : offset + 8] = struct.pack(
                        "<I", max_initial_peers_range
                    )
                if mutation_tries != DEFAULT_MUTATION_TRIES:
                    offset = unique_offset(
                        image, DEFAULT_NODE_MUTATION_SIGNATURE, "DefaultNode mutation"
                    )
                    image[offset + 7 : offset + 11] = struct.pack("<I", mutation_tries)
                if simulation_timestep_s != DEFAULT_SIMULATION_TIMESTEP_S:
                    constant_offset = unique_offset(
                        image,
                        TIMESTEP_CONSTANT_SIGNATURE,
                        "simulation timestep constant",
                    )
                    middleware_load_offset = unique_offset(
                        image,
                        MIDDLEWARE_TIMESTEP_LOAD_SIGNATURE,
                        "middleware timestep load",
                    )
                    interaction_load_offset = unique_offset(
                        image,
                        INTERACTION_TIMESTEP_LOAD_SIGNATURE,
                        "interaction-middleware timestep load",
                    )
                    image[constant_offset : constant_offset + 8] = struct.pack(
                        "<d", simulation_timestep_s
                    )
                    image[
                        middleware_load_offset : middleware_load_offset
                        + len(MIDDLEWARE_TIMESTEP_LOAD_SIGNATURE)
                    ] = MIDDLEWARE_TIMESTEP_LOAD_PATCH
                    image[
                        interaction_load_offset : interaction_load_offset
                        + len(INTERACTION_TIMESTEP_LOAD_SIGNATURE)
                    ] = INTERACTION_TIMESTEP_LOAD_PATCH
                if optimize_run_wait:
                    busy_wait_offset = unique_offset(
                        image,
                        RUN_BUSY_WAIT_SIGNATURE,
                        "ArmSimulation::Run busy-wait",
                    )
                    image[
                        busy_wait_offset : busy_wait_offset
                        + len(RUN_BUSY_WAIT_SIGNATURE)
                    ] = RUN_BUSY_WAIT_PATCH
                image.flush()
    except Exception:
        output.unlink(missing_ok=True)
        raise

    source_digest = sha256(source)
    output_digest = sha256(output)
    change_expected = (
        max_initial_peers_range != DEFAULT_INITIAL_PEERS_RANGE
        or mutation_tries != DEFAULT_MUTATION_TRIES
        or simulation_timestep_s != DEFAULT_SIMULATION_TIMESTEP_S
        or optimize_run_wait
    )
    if change_expected and source_digest == output_digest:
        output.unlink(missing_ok=True)
        raise PreparationError("private FlexivSimulation binary was not changed")
    return source_digest, output_digest


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--max-initial-peers-range", required=True, type=int)
    parser.add_argument("--mutation-tries", required=True, type=int)
    parser.add_argument(
        "--simulation-timestep-s",
        type=float,
        default=DEFAULT_SIMULATION_TIMESTEP_S,
    )
    parser.add_argument(
        "--preserve-run-busy-wait",
        action="store_true",
        help="retain the stock ArmSimulation::Run busy-spin for controlled A/B tests",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        source_digest, output_digest = prepare(
            args.source,
            args.output,
            args.max_initial_peers_range,
            args.mutation_tries,
            args.simulation_timestep_s,
            not args.preserve_run_busy_wait,
        )
    except (OSError, PreparationError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print(
        f"prepared {args.output.resolve()}: "
        f"max_initial_peers_range={args.max_initial_peers_range} "
        f"mutation_tries={args.mutation_tries} "
        f"simulation_timestep_s={args.simulation_timestep_s:.12g} "
        f"run_wait={'busy_spin' if args.preserve_run_busy_wait else 'blocking_join'} "
        f"source_sha256={source_digest} output_sha256={output_digest}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
