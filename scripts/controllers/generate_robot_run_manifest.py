#!/usr/bin/env python3
"""Allocate synthetic Rizon 4s identities and dense port blocks for one run."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import ipaddress
import json
import math
import os
from pathlib import Path
import re
import socket
import sys
import tempfile


BASE62 = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"
INTERNAL_SERIAL_PATTERN = re.compile(r"A02LS-05-M1-([0-9A-Za-z]{6})")


class ManifestError(RuntimeError):
    """Raised when a run manifest cannot be allocated safely."""


LEGACY_DEFAULTS = {
    "run_id": "synthetic-rizon-run",
    "num_environments": 1,
    "host_ip": "142.1.44.134",
    "port_start": 12000,
    "port_block_size": 64,
    "ports_per_robot": 32,
    "suffix_prefix": "SYN",
    "start_index": 0,
    "source_profile": "simulator0",
    "source_internal_serial": "A02LS-05-M1-cBDY86",
    "check_availability": False,
    "controller_startup_wave_size": 0,
    "controller_startup_inter_wave_delay_s": 0.0,
    "dds_max_initial_peers_range": 4,
    "dds_mutation_tries": 100,
    "physics_frequency_hz": 2000.0,
}


def controller_ready_file(run_id: str, generated_at: str) -> Path:
    """Return the run-specific local controller-readiness signal path."""
    digest = hashlib.sha256(f"{run_id}\0{generated_at}".encode("utf-8")).hexdigest()[:20]
    return Path("/tmp") / f"flexiv-controller-fleet-{digest}.ready.json"


def _require_mapping(value: object, name: str) -> dict[str, object]:
    if not isinstance(value, dict):
        raise ManifestError(f"{name} must be a YAML mapping")
    return value


def _require_list(value: object, name: str) -> list[object]:
    if not isinstance(value, list):
        raise ManifestError(f"{name} must be a YAML list")
    return value


def _require_string(value: object, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ManifestError(f"{name} must be a non-empty string")
    return value


def _require_integer(value: object, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ManifestError(f"{name} must be an integer")
    return value


def _require_number(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ManifestError(f"{name} must be a number")
    return float(value)


def _require_boolean(value: object, name: str) -> bool:
    if not isinstance(value, bool):
        raise ManifestError(f"{name} must be true or false")
    return value


def _resolve_config_path(value: object, name: str, config_path: Path) -> Path:
    path = Path(_require_string(value, name)).expanduser()
    if not path.is_absolute():
        path = config_path.parent / path
    return path.resolve()


def load_isaac_config(path: Path, output: Path) -> argparse.Namespace:
    """Translate one Isaac fleet YAML into the allocator's canonical inputs."""
    config_path = path.expanduser().resolve()
    try:
        import yaml
    except ImportError as error:
        raise ManifestError(
            "PyYAML is required when --isaac-config is used"
        ) from error

    try:
        document = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise ManifestError(f"cannot read Isaac config [{config_path}]: {error}") from error

    root = _require_mapping(document, "config root")
    simulation = _require_mapping(root.get("simulation"), "simulation")
    scene = _require_mapping(root.get("scene"), "scene")
    fleet = _require_mapping(root.get("fleet"), "fleet")
    robot_entries = _require_list(root.get("robots"), "robots")
    roles = []
    role_initial_q: dict[str, list[float]] = {}
    for index, entry_value in enumerate(robot_entries):
        entry = _require_mapping(entry_value, f"robots[{index}]")
        role = _require_string(entry.get("role"), f"robots[{index}].role")
        initial_q_values = _require_list(
            entry.get("initial_q"), f"robots[{index}].initial_q"
        )
        if len(initial_q_values) != 7:
            raise ManifestError(f"robots[{index}].initial_q must contain seven values")
        roles.append(role)
        role_initial_q[role] = [
            _require_number(value, f"robots[{index}].initial_q[{joint_index}]")
            for joint_index, value in enumerate(initial_q_values)
        ]

    profile_root_value = fleet.get("existing_profile_root")
    profile_root = None
    if profile_root_value is not None:
        profile_root = _resolve_config_path(
            profile_root_value, "fleet.existing_profile_root", config_path
        )

    return argparse.Namespace(
        output=output.expanduser().resolve(),
        run_id=_require_string(fleet.get("run_id"), "fleet.run_id"),
        num_environments=_require_integer(scene.get("num_envs"), "scene.num_envs"),
        role=roles,
        role_initial_q=role_initial_q,
        host_ip=_require_string(fleet.get("host_ip"), "fleet.host_ip"),
        port_start=_require_integer(fleet.get("port_start"), "fleet.port_start"),
        port_block_size=_require_integer(
            fleet.get("port_block_size"), "fleet.port_block_size"
        ),
        ports_per_robot=_require_integer(
            fleet.get("ports_per_robot"), "fleet.ports_per_robot"
        ),
        suffix_prefix=_require_string(
            fleet.get("suffix_prefix"), "fleet.suffix_prefix"
        ),
        start_index=_require_integer(fleet.get("start_index"), "fleet.start_index"),
        existing_profile_root=profile_root,
        source_profile=_require_string(
            fleet.get("source_profile"), "fleet.source_profile"
        ),
        source_internal_serial=_require_string(
            fleet.get("source_internal_serial"), "fleet.source_internal_serial"
        ),
        check_availability=_require_boolean(
            fleet.get("check_availability"), "fleet.check_availability"
        ),
        controller_startup_wave_size=_require_integer(
            fleet.get("controller_startup_wave_size", 0),
            "fleet.controller_startup_wave_size",
        ),
        controller_startup_inter_wave_delay_s=_require_number(
            fleet.get("controller_startup_inter_wave_delay_s", 0.0),
            "fleet.controller_startup_inter_wave_delay_s",
        ),
        dds_max_initial_peers_range=_require_integer(
            fleet.get("dds_max_initial_peers_range", 4),
            "fleet.dds_max_initial_peers_range",
        ),
        dds_mutation_tries=_require_integer(
            fleet.get("dds_mutation_tries", 100),
            "fleet.dds_mutation_tries",
        ),
        physics_frequency_hz=_require_number(
            simulation.get("physics_frequency_hz"),
            "simulation.physics_frequency_hz",
        ),
        isaac_config=config_path,
    )


def encode_base62(value: int, width: int) -> str:
    if value < 0:
        raise ManifestError(f"identity index must be non-negative, got {value}")
    characters = [BASE62[0]] * width
    remaining = value
    for index in range(width - 1, -1, -1):
        remaining, digit = divmod(remaining, len(BASE62))
        characters[index] = BASE62[digit]
    if remaining:
        raise ManifestError(f"identity index {value} does not fit in {width} base-62 digits")
    return "".join(characters)


def existing_suffixes(profile_root: Path | None) -> set[str]:
    if profile_root is None:
        return set()
    if not profile_root.is_dir():
        raise ManifestError(f"existing profile root not found: {profile_root}")

    suffixes: set[str] = set()
    for path in profile_root.glob("*/A02LS-05-M1-*"):
        if not path.is_dir():
            continue
        match = INTERNAL_SERIAL_PATTERN.fullmatch(path.name)
        if match:
            suffixes.add(match.group(1))
    return suffixes


def local_ephemeral_range() -> tuple[int, int] | None:
    path = Path("/proc/sys/net/ipv4/ip_local_port_range")
    try:
        values = [int(value) for value in path.read_text(encoding="ascii").split()]
    except (OSError, ValueError):
        return None
    if len(values) != 2:
        return None
    return values[0], values[1]


def assert_available(host_ip: str, ports: list[int]) -> None:
    # Availability is a point-in-time preflight, not a port reservation.  Close
    # each probe immediately so large manifests do not exhaust the process file
    # descriptor limit merely by checking thousands of ports.
    for port in ports:
        for socket_type in (socket.SOCK_STREAM, socket.SOCK_DGRAM):
            candidate = socket.socket(socket.AF_INET, socket_type)
            try:
                candidate.bind((host_ip, port))
                if socket_type == socket.SOCK_STREAM:
                    candidate.listen(1)
            except OSError as error:
                raise ManifestError(
                    f"port preflight failed for {host_ip}:{port} "
                    f"({'TCP' if socket_type == socket.SOCK_STREAM else 'UDP'}): {error}"
                ) from error
            finally:
                candidate.close()


def atomic_write_json(path: Path, manifest: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temp_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(manifest, stream, indent=2)
            stream.write("\n")
        os.replace(temp_name, path)
    finally:
        Path(temp_name).unlink(missing_ok=True)


def generate(args: argparse.Namespace) -> dict[str, object]:
    roles = args.role or ["left", "right"]
    if args.num_environments < 1:
        raise ManifestError("number of environments must be positive")
    if not roles or any(not role.strip() for role in roles):
        raise ManifestError("robot roles must be non-empty")
    if len(roles) != len(set(roles)):
        raise ManifestError("robot roles must be unique")
    try:
        host_ip = str(ipaddress.IPv4Address(args.host_ip))
    except ipaddress.AddressValueError as error:
        raise ManifestError(f"invalid host IPv4 address: {args.host_ip}") from error

    if not re.fullmatch(r"[0-9A-Za-z]{1,5}", args.suffix_prefix):
        raise ManifestError("suffix prefix must contain 1-5 base-62 characters")
    if not INTERNAL_SERIAL_PATTERN.fullmatch(args.source_internal_serial):
        raise ManifestError(
            f"invalid source Rizon 4s internal serial: {args.source_internal_serial}"
        )
    suffix_width = 6 - len(args.suffix_prefix)
    robot_count = args.num_environments * len(roles)
    startup_wave_size = getattr(args, "controller_startup_wave_size", 0)
    startup_delay_s = getattr(
        args, "controller_startup_inter_wave_delay_s", 0.0
    )
    if startup_wave_size < 0:
        raise ManifestError("controller startup wave size cannot be negative")
    if startup_delay_s < 0:
        raise ManifestError("controller startup inter-wave delay cannot be negative")
    dds_max_initial_peers_range = getattr(args, "dds_max_initial_peers_range", 4)
    if not 4 <= dds_max_initial_peers_range <= 65_535:
        raise ManifestError("DDS maximum initial peers range must be in [4, 65535]")
    dds_mutation_tries = getattr(args, "dds_mutation_tries", 100)
    if not 100 <= dds_mutation_tries <= 65_535:
        raise ManifestError("DDS mutation tries must be in [100, 65535]")
    physics_frequency_hz = getattr(args, "physics_frequency_hz", 2000.0)
    if not math.isfinite(physics_frequency_hz) or physics_frequency_hz <= 0.0:
        raise ManifestError("physics frequency must be finite and positive")
    if args.port_start < 1024:
        raise ManifestError("port start must be at least 1024")
    if args.port_block_size <= args.ports_per_robot:
        raise ManifestError(
            "port block must include one time-sync port plus every config port"
        )
    if args.ports_per_robot < 1:
        raise ManifestError("ports per robot must be positive")

    final_block_end = (
        args.port_start + robot_count * args.port_block_size - 1
    )
    if final_block_end > 65535:
        raise ManifestError(
            f"requested blocks end at port {final_block_end}, above 65535"
        )
    ephemeral_range = local_ephemeral_range()
    if ephemeral_range is not None:
        ephemeral_start, ephemeral_end = ephemeral_range
        if not (final_block_end < ephemeral_start or args.port_start > ephemeral_end):
            raise ManifestError(
                f"allocated range {args.port_start}-{final_block_end} overlaps local "
                f"ephemeral range {ephemeral_start}-{ephemeral_end}"
            )

    reserved = existing_suffixes(args.existing_profile_root)
    allocated: set[str] = set()
    next_index = args.start_index
    robots: list[dict[str, object]] = []
    all_used_ports: list[int] = []

    for env_index in range(args.num_environments):
        for role_index, role in enumerate(roles):
            while True:
                suffix = args.suffix_prefix + encode_base62(next_index, suffix_width)
                next_index += 1
                if suffix not in reserved and suffix not in allocated:
                    allocated.add(suffix)
                    break

            slot = env_index * len(roles) + role_index
            block_base = args.port_start + slot * args.port_block_size
            block_end = block_base + args.port_block_size - 1
            time_sync_port = block_base
            config_ports = list(
                range(block_base + 1, block_base + 1 + args.ports_per_robot)
            )
            all_used_ports.extend([time_sync_port, *config_ports])

            internal_serial = f"A02LS-05-M1-{suffix}"
            public_serial = f"Rizon4s-{suffix}"
            robot = {
                "slot": slot,
                "env_index": env_index,
                "role": role,
                "suffix": suffix,
                "internal_serial": internal_serial,
                "public_serial": public_serial,
                "normalized_internal_serial": internal_serial.replace("-", "_"),
                "normalized_public_serial": public_serial.replace("-", "_"),
                "network": {
                    "host_ip": host_ip,
                    "port_block_base": block_base,
                    "port_block_end": block_end,
                    "port_block_size": args.port_block_size,
                    "time_sync_port": time_sync_port,
                    "config_ports": config_ports,
                },
            }
            role_initial_q = getattr(args, "role_initial_q", {})
            if role in role_initial_q:
                robot["initial_q"] = list(role_initial_q[role])
            robots.append(robot)

    if len(all_used_ports) != len(set(all_used_ports)):
        raise ManifestError("allocator produced duplicate ports")
    if args.check_availability:
        assert_available(host_ip, all_used_ports)

    generated_at = datetime.now(timezone.utc).isoformat()
    return {
        "schema_version": 1,
        "run_id": args.run_id,
        "generated_at": generated_at,
        "controller_ready_file": str(
            controller_ready_file(args.run_id, generated_at)
        ),
        "source_profile": {
            "name": args.source_profile,
            "internal_serial": args.source_internal_serial,
        },
        "allocation": {
            "num_environments": args.num_environments,
            "roles": roles,
            "robot_count": robot_count,
            "host_ip": host_ip,
            "port_start": args.port_start,
            "port_end": final_block_end,
            "port_block_size": args.port_block_size,
            "ports_per_robot": args.ports_per_robot,
            "local_ephemeral_range": list(ephemeral_range) if ephemeral_range else None,
        },
        "controller_startup": {
            "wave_size": startup_wave_size,
            "inter_wave_delay_s": startup_delay_s,
        },
        "simulation": {
            "physics_frequency_hz": physics_frequency_hz,
        },
        "middleware": {
            "dds_max_initial_peers_range": dds_max_initial_peers_range,
            "dds_mutation_tries": dds_mutation_tries,
        },
        "robots": robots,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--isaac-config",
        type=Path,
        help="load topology and allocation inputs from one Isaac fleet YAML",
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument("--run-id")
    parser.add_argument("--num-environments", type=int)
    parser.add_argument("--role", action="append")
    parser.add_argument("--host-ip")
    parser.add_argument("--port-start", type=int)
    parser.add_argument("--port-block-size", type=int)
    parser.add_argument("--ports-per-robot", type=int)
    parser.add_argument("--suffix-prefix")
    parser.add_argument("--start-index", type=int)
    parser.add_argument("--existing-profile-root", type=Path)
    parser.add_argument("--source-profile")
    parser.add_argument("--source-internal-serial")
    parser.add_argument("--check-availability", action="store_true", default=None)
    parser.add_argument("--controller-startup-wave-size", type=int)
    parser.add_argument("--controller-startup-inter-wave-delay-s", type=float)
    parser.add_argument("--dds-max-initial-peers-range", type=int)
    parser.add_argument("--dds-mutation-tries", type=int)
    parser.add_argument("--physics-frequency-hz", type=float)
    args = parser.parse_args()

    if args.isaac_config is not None:
        allocator_options = (
            "run_id",
            "num_environments",
            "role",
            "host_ip",
            "port_start",
            "port_block_size",
            "ports_per_robot",
            "suffix_prefix",
            "start_index",
            "existing_profile_root",
            "source_profile",
            "source_internal_serial",
            "check_availability",
            "controller_startup_wave_size",
            "controller_startup_inter_wave_delay_s",
            "dds_max_initial_peers_range",
            "dds_mutation_tries",
            "physics_frequency_hz",
        )
        conflicts = [
            f"--{name.replace('_', '-')}"
            for name in allocator_options
            if getattr(args, name) is not None
        ]
        if conflicts:
            parser.error(
                "--isaac-config is authoritative and cannot be combined with "
                + ", ".join(conflicts)
            )
        if args.output is None:
            parser.error("--output is required with --isaac-config")
        return load_isaac_config(args.isaac_config, args.output)

    if args.output is None:
        parser.error("--output is required")
    for name, default in LEGACY_DEFAULTS.items():
        if getattr(args, name) is None:
            setattr(args, name, default)
    args.isaac_config = None
    return args


def main() -> int:
    try:
        args = parse_args()
        manifest = generate(args)
        if args.isaac_config is not None:
            config_bytes = args.isaac_config.read_bytes()
            manifest["isaac_config"] = {
                "path": str(args.isaac_config),
                "sha256": hashlib.sha256(config_bytes).hexdigest(),
            }
        atomic_write_json(args.output, manifest)
    except (ManifestError, OSError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    allocation = manifest["allocation"]
    print(
        f"generated {args.output}: robots={allocation['robot_count']} "
        f"ports={allocation['port_start']}-{allocation['port_end']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
