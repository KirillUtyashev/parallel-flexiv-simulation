#!/usr/bin/env python3
"""Launch an arbitrary manifest-defined fleet of simulated Flexiv arms.

The installed Elements Studio tree is read-only.  Each arm receives a private
profile, RobotControlApp binary, communication-port block, bridge, logs, and
cleanup lifecycle under one retained runtime directory.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import fcntl
import ipaddress
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from typing import Any

from clone_simulator_profile_identity import CloneError, clone_profile
from generate_robot_run_manifest import (
    ManifestError,
    assert_available,
    atomic_write_json,
    controller_ready_file,
    generate,
    local_ephemeral_range,
)
from prepare_robot_control_app_runtime import (
    PORT_PATTERN,
    PreparationError,
    prepare,
)
from prepare_flexiv_simulation_binary import (
    PreparationError as SimulationPreparationError,
    prepare as prepare_flexiv_simulation,
)


INTERNAL_SERIAL_RE = re.compile(r"A02LS-05-M1-([0-9A-Za-z]{6})")
PUBLIC_SERIAL_RE = re.compile(r"Rizon4s-([0-9A-Za-z]{6})")
SHM_PREFIXES = (
    "flexiv_arm_command_",
    "flexiv_arm_state_",
    "flexiv_simulation_motion_bar_",
    "flexiv_simulation_rca_motion_bar_",
    "flexiv_simulation_tool_payload_",
)


class FleetError(RuntimeError):
    """Raised when the fleet cannot be launched safely."""


class StopRequested(RuntimeError):
    """Raised when SIGINT or SIGTERM requests an orderly shutdown."""


@dataclass(frozen=True)
class RobotSpec:
    slot: int
    env_index: int
    role: str
    suffix: str
    internal_serial: str
    public_serial: str
    normalized_internal_serial: str
    normalized_public_serial: str
    host_ip: str
    block_base: int
    block_end: int
    block_size: int
    time_sync_port: int
    config_ports: tuple[int, ...]


@dataclass(frozen=True)
class RobotRuntime:
    spec: RobotSpec
    path: Path


@dataclass
class ManagedProcess:
    name: str
    kind: str
    robot: RobotSpec
    log_path: Path
    process: subprocess.Popen[bytes]


def log(message: str) -> None:
    print(f"[generated-fleet] {message}", flush=True)


def require_mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise FleetError(f"{label} must be a JSON object")
    return value


def require_list(value: Any, label: str) -> list[Any]:
    if not isinstance(value, list):
        raise FleetError(f"{label} must be a JSON array")
    return value


def require_string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise FleetError(f"{label} must be a non-empty string")
    return value


def require_integer(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise FleetError(f"{label} must be an integer")
    return value


def require_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise FleetError(f"{label} must be a number")
    return float(value)


def load_manifest(path: Path) -> tuple[dict[str, Any], list[RobotSpec]]:
    try:
        manifest = require_mapping(
            json.loads(path.read_text(encoding="utf-8")), "manifest"
        )
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise FleetError(f"cannot read manifest {path}: {error}") from error

    if require_integer(manifest.get("schema_version"), "schema_version") != 1:
        raise FleetError("only robot-run manifest schema_version 1 is supported")

    allocation = require_mapping(manifest.get("allocation"), "allocation")
    source_profile = require_mapping(manifest.get("source_profile"), "source_profile")
    require_string(source_profile.get("name"), "source_profile.name")
    source_serial = require_string(
        source_profile.get("internal_serial"), "source_profile.internal_serial"
    )
    if not INTERNAL_SERIAL_RE.fullmatch(source_serial):
        raise FleetError(f"invalid source profile serial: {source_serial}")

    robot_count = require_integer(allocation.get("robot_count"), "allocation.robot_count")
    num_environments = require_integer(
        allocation.get("num_environments"), "allocation.num_environments"
    )
    roles = tuple(
        require_string(role, f"allocation.roles[{index}]")
        for index, role in enumerate(require_list(allocation.get("roles"), "allocation.roles"))
    )
    ports_per_robot = require_integer(
        allocation.get("ports_per_robot"), "allocation.ports_per_robot"
    )
    block_size = require_integer(
        allocation.get("port_block_size"), "allocation.port_block_size"
    )
    allocation_host = require_string(allocation.get("host_ip"), "allocation.host_ip")
    try:
        allocation_host = str(ipaddress.IPv4Address(allocation_host))
    except ipaddress.AddressValueError as error:
        raise FleetError(f"invalid allocation host IPv4 address: {allocation_host}") from error

    robot_entries = require_list(manifest.get("robots"), "robots")
    if robot_count < 1 or robot_count != len(robot_entries):
        raise FleetError(
            f"allocation.robot_count={robot_count} does not match {len(robot_entries)} entries"
        )
    if num_environments < 1 or not roles or len(roles) != len(set(roles)):
        raise FleetError("allocation environments and roles must be non-empty and unique")
    if robot_count != num_environments * len(roles):
        raise FleetError("allocation robot count does not match environments times roles")
    if ports_per_robot < 1 or block_size <= ports_per_robot:
        raise FleetError("manifest port block cannot hold time-sync and all config ports")

    robots: list[RobotSpec] = []
    identity_values: set[str] = set()
    active_ports: set[int] = set()
    for expected_slot, raw_entry in enumerate(robot_entries):
        label = f"robots[{expected_slot}]"
        entry = require_mapping(raw_entry, label)
        slot = require_integer(entry.get("slot"), f"{label}.slot")
        if slot != expected_slot:
            raise FleetError(f"{label}.slot must be {expected_slot}, got {slot}")
        env_index = require_integer(entry.get("env_index"), f"{label}.env_index")
        role = require_string(entry.get("role"), f"{label}.role")
        expected_env_index, role_index = divmod(slot, len(roles))
        if env_index != expected_env_index or role != roles[role_index]:
            raise FleetError(f"{label} environment/role assignment is inconsistent")
        suffix = require_string(entry.get("suffix"), f"{label}.suffix")
        internal_serial = require_string(
            entry.get("internal_serial"), f"{label}.internal_serial"
        )
        public_serial = require_string(
            entry.get("public_serial"), f"{label}.public_serial"
        )
        normalized_internal = require_string(
            entry.get("normalized_internal_serial"),
            f"{label}.normalized_internal_serial",
        )
        normalized_public = require_string(
            entry.get("normalized_public_serial"),
            f"{label}.normalized_public_serial",
        )

        internal_match = INTERNAL_SERIAL_RE.fullmatch(internal_serial)
        public_match = PUBLIC_SERIAL_RE.fullmatch(public_serial)
        if not internal_match or not public_match:
            raise FleetError(f"{label} has an invalid Rizon 4s identity")
        if internal_match.group(1) != suffix or public_match.group(1) != suffix:
            raise FleetError(f"{label} serials do not match suffix {suffix}")
        if normalized_internal != internal_serial.replace("-", "_"):
            raise FleetError(f"{label}.normalized_internal_serial is inconsistent")
        if normalized_public != public_serial.replace("-", "_"):
            raise FleetError(f"{label}.normalized_public_serial is inconsistent")
        for identity in (
            internal_serial,
            public_serial,
            normalized_internal,
            normalized_public,
        ):
            if identity in identity_values:
                raise FleetError(f"manifest reuses robot identity {identity}")
            identity_values.add(identity)

        network = require_mapping(entry.get("network"), f"{label}.network")
        host_ip = require_string(network.get("host_ip"), f"{label}.network.host_ip")
        try:
            host_ip = str(ipaddress.IPv4Address(host_ip))
        except ipaddress.AddressValueError as error:
            raise FleetError(f"invalid host IPv4 address for {label}: {host_ip}") from error
        if host_ip != allocation_host:
            raise FleetError(
                "one fleet-launcher invocation must use one controller host IP; "
                f"{label} uses {host_ip}, expected {allocation_host}"
            )
        robot_block_size = require_integer(
            network.get("port_block_size"), f"{label}.network.port_block_size"
        )
        block_base = require_integer(
            network.get("port_block_base"), f"{label}.network.port_block_base"
        )
        block_end = require_integer(
            network.get("port_block_end"), f"{label}.network.port_block_end"
        )
        time_sync_port = require_integer(
            network.get("time_sync_port"), f"{label}.network.time_sync_port"
        )
        raw_ports = require_list(
            network.get("config_ports"), f"{label}.network.config_ports"
        )
        config_ports = tuple(
            require_integer(port, f"{label}.network.config_ports[{index}]")
            for index, port in enumerate(raw_ports)
        )
        if robot_block_size != block_size:
            raise FleetError(f"{label} block size differs from allocation")
        expected_base = require_integer(
            allocation.get("port_start"), "allocation.port_start"
        ) + slot * block_size
        if block_base != expected_base or block_end != block_base + block_size - 1:
            raise FleetError(f"{label} has a non-contiguous or malformed port block")
        if not 1024 <= block_base <= block_end <= 65535:
            raise FleetError(f"{label} port block is outside 1024-65535")
        expected_ports = tuple(range(block_base + 1, block_base + 1 + ports_per_robot))
        if time_sync_port != block_base or config_ports != expected_ports:
            raise FleetError(f"{label} active ports do not match the dense-block contract")
        for port in (time_sync_port, *config_ports):
            if port in active_ports:
                raise FleetError(f"manifest reuses active port {port}")
            active_ports.add(port)

        robots.append(
            RobotSpec(
                slot=slot,
                env_index=env_index,
                role=role,
                suffix=suffix,
                internal_serial=internal_serial,
                public_serial=public_serial,
                normalized_internal_serial=normalized_internal,
                normalized_public_serial=normalized_public,
                host_ip=host_ip,
                block_base=block_base,
                block_end=block_end,
                block_size=robot_block_size,
                time_sync_port=time_sync_port,
                config_ports=config_ports,
            )
        )

    port_start = require_integer(allocation.get("port_start"), "allocation.port_start")
    port_end = require_integer(allocation.get("port_end"), "allocation.port_end")
    if port_end != port_start + robot_count * block_size - 1:
        raise FleetError("allocation.port_end does not match the fleet block span")
    ephemeral_range = local_ephemeral_range()
    if ephemeral_range is not None:
        ephemeral_start, ephemeral_end = ephemeral_range
        if not (port_end < ephemeral_start or port_start > ephemeral_end):
            raise FleetError(
                f"fleet range {port_start}-{port_end} overlaps local ephemeral range "
                f"{ephemeral_start}-{ephemeral_end}"
            )
    return manifest, robots


def command_exists(command: str) -> bool:
    return shutil.which(command) is not None


def process_matches(arguments: list[str]) -> bool:
    result = subprocess.run(arguments, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if result.returncode not in (0, 1):
        raise FleetError(f"process preflight failed: {' '.join(arguments)}")
    return result.returncode == 0


def active_ipv4_addresses() -> set[str]:
    try:
        result = subprocess.run(
            ["ip", "-4", "-o", "address", "show"],
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError) as error:
        raise FleetError(f"cannot inspect local IPv4 addresses: {error}") from error
    addresses: set[str] = set()
    for line in result.stdout.splitlines():
        fields = line.split()
        if "inet" in fields:
            addresses.add(fields[fields.index("inet") + 1].split("/", 1)[0])
    return addresses


def tail(path: Path, line_count: int = 25) -> str:
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return "<log unavailable>"
    return "\n".join(lines[-line_count:]) or "<log empty>"


class FleetLauncher:
    def __init__(
        self,
        args: argparse.Namespace,
        manifest: dict[str, Any],
        robots: list[RobotSpec],
        manifest_path: Path,
        work_root: Path,
    ) -> None:
        self.args = args
        self.manifest = manifest
        self.robots = robots
        self.manifest_path = manifest_path
        self.work_root = work_root
        self.elem_root = args.elem_root
        self.runtime_python = args.runtime_python
        self.repo_root = Path(__file__).resolve().parents[2]
        self.runtimes: list[RobotRuntime] = []
        self.processes: list[ManagedProcess] = []
        self.lock_stream: Any = None
        self.stop_signal: int | None = None
        self.ready_signal_owned = False
        startup = manifest.get("controller_startup", {})
        if startup is None:
            startup = {}
        startup = require_mapping(startup, "controller_startup")
        manifest_wave_size = require_integer(
            startup.get("wave_size", 0), "controller_startup.wave_size"
        )
        manifest_wave_delay_s = startup.get("inter_wave_delay_s", 0.0)
        if isinstance(manifest_wave_delay_s, bool) or not isinstance(
            manifest_wave_delay_s, (int, float)
        ):
            raise FleetError("controller_startup.inter_wave_delay_s must be a number")
        requested_wave_size = (
            args.startup_wave_size
            if args.startup_wave_size is not None
            else manifest_wave_size
        )
        requested_wave_delay_s = (
            args.startup_wave_delay_s
            if args.startup_wave_delay_s is not None
            else float(manifest_wave_delay_s)
        )
        if requested_wave_size < 0:
            raise FleetError("controller startup wave size cannot be negative")
        if requested_wave_delay_s < 0:
            raise FleetError("controller startup wave delay cannot be negative")
        self.startup_wave_size = requested_wave_size or len(robots)
        self.startup_wave_size = min(self.startup_wave_size, len(robots))
        self.startup_wave_delay_s = requested_wave_delay_s
        middleware = manifest.get("middleware", {})
        if middleware is None:
            middleware = {}
        middleware = require_mapping(middleware, "middleware")
        self.dds_max_initial_peers_range = require_integer(
            middleware.get("dds_max_initial_peers_range", 4),
            "middleware.dds_max_initial_peers_range",
        )
        if not 4 <= self.dds_max_initial_peers_range <= 65_535:
            raise FleetError("DDS maximum initial peers range must be in [4, 65535]")
        self.dds_mutation_tries = require_integer(
            middleware.get("dds_mutation_tries", 100),
            "middleware.dds_mutation_tries",
        )
        if not 100 <= self.dds_mutation_tries <= 65_535:
            raise FleetError("DDS mutation tries must be in [100, 65535]")
        simulation = manifest.get("simulation", {})
        if simulation is None:
            simulation = {}
        simulation = require_mapping(simulation, "simulation")
        self.physics_frequency_hz = require_number(
            simulation.get("physics_frequency_hz", 2000.0),
            "simulation.physics_frequency_hz",
        )
        if (
            not math.isfinite(self.physics_frequency_hz)
            or self.physics_frequency_hz <= 0.0
        ):
            raise FleetError("simulation physics frequency must be finite and positive")
        self.flexiv_simulation_timestep_s = 1.0 / self.physics_frequency_hz
        self.flexiv_simulation_binary = self.elem_root / "FlexivSimulation"
        run_id = require_string(manifest.get("run_id"), "run_id")
        generated_at = require_string(manifest.get("generated_at"), "generated_at")
        expected_ready_file = controller_ready_file(run_id, generated_at)
        declared_ready_file = manifest.get("controller_ready_file")
        if declared_ready_file is not None:
            declared_ready_file = Path(
                require_string(declared_ready_file, "controller_ready_file")
            )
            if declared_ready_file != expected_ready_file:
                raise FleetError(
                    "controller_ready_file does not match run_id/generated_at"
                )
        self.ready_file = expected_ready_file
        source = require_mapping(manifest["source_profile"], "source_profile")
        self.source_profile_name = require_string(source["name"], "source_profile.name")
        self.source_internal_serial = require_string(
            source["internal_serial"], "source_profile.internal_serial"
        )
        self.source_profile = (
            self.elem_root / "user_data_ui" / "simDir" / self.source_profile_name
        )

    def request_stop(self, signum: int, _frame: Any) -> None:
        self.stop_signal = signum

    def check_stop(self) -> None:
        if self.stop_signal is not None:
            raise StopRequested(signal.Signals(self.stop_signal).name)

    def preflight(self) -> None:
        required_files = [
            self.elem_root / "RobotControlApp",
            self.elem_root / "FlexivSimulation",
        ]
        if self.args.physics == "mock":
            required_files.extend(
                (
                    self.runtime_python,
                    self.repo_root / "scripts" / "rdk" / "mock_external_simulator.py",
                )
            )
        if self.args.rdk_smoke_seconds:
            required_files.extend(
                (
                    self.runtime_python,
                    self.repo_root / "scripts" / "rdk" / "rdk_read_joint_states.py",
                )
            )
        for path in required_files:
            if not path.is_file():
                raise FleetError(f"required file not found: {path}")
        for path in (self.elem_root / "specs_enc", self.source_profile):
            if not path.is_dir():
                raise FleetError(f"required directory not found: {path}")
        if not (self.source_profile / self.source_internal_serial).is_dir():
            raise FleetError(
                f"source identity is absent from profile: {self.source_internal_serial}"
            )
        for command in (
            "ip",
            "pgrep",
            "fuser",
            "mountpoint",
            "fusermount",
            "gocryptfs",
        ):
            if not command_exists(command):
                raise FleetError(f"required command not found: {command}")

        if process_matches(["pgrep", "-x", "RobotControlApp"]):
            raise FleetError("RobotControlApp is already running")
        if process_matches(["pgrep", "-f", r"(^|/)FlexivSimulation( |$)"]):
            raise FleetError("FlexivSimulation is already running")
        if process_matches(["pgrep", "-f", r"[m]ock_external_simulator.py"]):
            raise FleetError("a mock external simulator is already running")

        host_ip = self.robots[0].host_ip
        if host_ip != "0.0.0.0" and host_ip not in active_ipv4_addresses():
            raise FleetError(f"fleet host IP is not active on this machine: {host_ip}")

        sim_root = self.elem_root / "user_data_ui" / "simDir"
        for robot in self.robots:
            collisions = list(sim_root.glob(f"*/{robot.internal_serial}"))
            if collisions:
                raise FleetError(
                    f"generated identity already exists in an installed profile: {collisions[0]}"
                )

        imports: list[str] = []
        if self.args.physics == "mock":
            imports.append("flexivsimplugin")
        if self.args.rdk_smoke_seconds:
            imports.append("flexivrdk")
        if imports:
            result = subprocess.run(
                [str(self.runtime_python), "-c", "; ".join(f"import {name}" for name in imports)],
                capture_output=True,
                text=True,
            )
            if result.returncode != 0:
                raise FleetError(
                    f"runtime Python is missing required packages {imports}: "
                    f"{result.stderr.strip()}"
                )

        active_ports = [
            port
            for robot in self.robots
            for port in (robot.time_sync_port, *robot.config_ports)
        ]
        try:
            assert_available(host_ip, active_ports)
        except ManifestError as error:
            raise FleetError(str(error)) from error

    def acquire_lock(self) -> None:
        token = re.sub(r"[^0-9A-Za-z]+", "_", self.robots[0].host_ip).strip("_")
        lock_path = Path("/tmp") / f"flexiv_robot_fleet_{token}.lock"
        self.lock_stream = lock_path.open("a+", encoding="utf-8")
        try:
            fcntl.flock(self.lock_stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            self.lock_stream.seek(0)
            owner = self.lock_stream.read().strip() or "unknown owner"
            raise FleetError(
                f"another fleet launcher owns {self.robots[0].host_ip}: {owner}"
            ) from error
        self.lock_stream.seek(0)
        self.lock_stream.truncate()
        self.lock_stream.write(f"pid={os.getpid()} manifest={self.manifest_path}\n")
        self.lock_stream.flush()

    def clean_serial_shm(self, robot: RobotSpec, fail_if_held: bool) -> None:
        for prefix in SHM_PREFIXES:
            path = Path("/dev/shm") / f"{prefix}{robot.normalized_internal_serial}"
            if not path.exists():
                continue
            held = subprocess.run(
                ["fuser", str(path)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            ).returncode == 0
            if held:
                if fail_if_held:
                    raise FleetError(f"shared-memory resource has a live holder: {path}")
                log(f"leaving shared-memory resource with a live holder: {path}")
                continue
            path.unlink(missing_ok=True)

    @staticmethod
    def replace_first(text: str, source: str, target: str, label: str) -> str:
        if source not in text:
            raise FleetError(f"{label} was not found in the source profile")
        return text.replace(source, target, 1)

    def provision_robot(self, robot: RobotSpec) -> RobotRuntime:
        runtime = self.work_root / f"robot_{robot.slot:03d}_{robot.public_serial}"
        clone_profile(
            self.source_profile,
            runtime,
            self.source_internal_serial,
            robot.internal_serial,
        )

        shutil.copytree(
            self.elem_root / "specs_enc",
            runtime / "specs_enc",
            copy_function=shutil.copy2,
        )
        (runtime / "specs_enc" / "gocryptfs.conf").unlink(missing_ok=True)
        (runtime / "specs").mkdir()
        for name in ("bin", "lib", "plugins"):
            (runtime / name).symlink_to(self.elem_root / name, target_is_directory=True)

        comm_config = runtime / "user_data" / "settings" / "commCfg.prototxt"
        ext_config = runtime / "user_data" / "settings" / "robotExtInterfaceCfg.xml"
        try:
            comm_text = comm_config.read_text(encoding="utf-8")
            comm_text = self.replace_first(
                comm_text,
                'ip: "0.0.0.0"',
                f'ip: "{robot.host_ip}"',
                "RobotPC wildcard IP",
            )
            comm_text = self.replace_first(
                comm_text,
                'ip: "127.0.0.1"',
                f'ip: "{robot.host_ip}"',
                "UIPC loopback IP",
            )
            comm_config.write_text(comm_text, encoding="utf-8")

            ext_text = ext_config.read_text(encoding="utf-8")
            ext_text, enable_count = re.subn(
                r"<enable>[01]</enable>", "<enable>1</enable>", ext_text, count=1
            )
            ext_text, interface_count = re.subn(
                r"<interface_config_file>[^<]+</interface_config_file>",
                "<interface_config_file>externalEthernetConfig.xml</interface_config_file>",
                ext_text,
                count=1,
            )
            if enable_count != 1 or interface_count != 1:
                raise FleetError(f"cannot enable Ethernet external interface in {ext_config}")
            ext_config.write_text(ext_text, encoding="utf-8")
        except (OSError, UnicodeError) as error:
            raise FleetError(f"cannot configure private profile {runtime}: {error}") from error

        prepare(
            source_binary=self.elem_root / "RobotControlApp",
            output_binary=runtime / "RobotControlApp",
            comm_config=comm_config,
            time_sync_port=robot.time_sync_port,
            port_offset=None,
            port_base=robot.block_base,
            port_block_size=robot.block_size,
            dds_mutation_tries=self.dds_mutation_tries,
        )
        rendered_ports = tuple(
            int(match.group(2))
            for match in PORT_PATTERN.finditer(comm_config.read_text(encoding="utf-8"))
        )
        if rendered_ports != robot.config_ports:
            raise FleetError(
                f"{robot.public_serial} config ports differ from its manifest block"
            )
        self.clean_serial_shm(robot, fail_if_held=True)
        result = RobotRuntime(robot, runtime)
        self.runtimes.append(result)
        return result

    def prepare_flexiv_simulation_binary(self) -> None:
        private_binary = self.work_root / "FlexivSimulation"
        source_digest, output_digest = prepare_flexiv_simulation(
            self.elem_root / "FlexivSimulation",
            private_binary,
            self.dds_max_initial_peers_range,
            self.dds_mutation_tries,
            self.flexiv_simulation_timestep_s,
            not self.args.preserve_flexiv_simulation_busy_wait,
        )
        self.flexiv_simulation_binary = private_binary
        log(
            "prepared one shared private FlexivSimulation binary: "
            f"dds_max_initial_peers_range={self.dds_max_initial_peers_range} "
            f"dds_mutation_tries={self.dds_mutation_tries} "
            f"simulation_timestep_s={self.flexiv_simulation_timestep_s:.12g} "
            f"run_wait={'busy_spin' if self.args.preserve_flexiv_simulation_busy_wait else 'blocking_join'} "
            f"source_sha256={source_digest} output_sha256={output_digest}"
        )

    def spawn(
        self,
        name: str,
        kind: str,
        runtime: RobotRuntime,
        command: list[str],
        cwd: Path,
        environment: dict[str, str],
        log_name: str,
    ) -> ManagedProcess:
        log_path = runtime.path / log_name
        with log_path.open("wb") as stream:
            process = subprocess.Popen(
                command,
                cwd=cwd,
                env=environment,
                stdout=stream,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        managed = ManagedProcess(name, kind, runtime.spec, log_path, process)
        self.processes.append(managed)
        return managed

    def base_environment(self) -> dict[str, str]:
        environment = os.environ.copy()
        library_path = str(self.elem_root / "lib")
        inherited_library_path = environment.get("LD_LIBRARY_PATH")
        if inherited_library_path:
            library_path = f"{library_path}:{inherited_library_path}"
        environment.update(
            {
                "LD_LIBRARY_PATH": library_path,
                "QT_QPA_PLATFORM_PLUGIN_PATH": str(self.elem_root / "plugins"),
                "QTWEBENGINE_DISABLE_SANDBOX": "1",
            }
        )
        return environment

    def process_environment(
        self, runtime: RobotRuntime, process_kind: str
    ) -> dict[str, str]:
        environment = self.base_environment()
        if self.args.cpu_profiler_library is None:
            return environment

        inherited_preload = environment.get("LD_PRELOAD")
        profiler_library = str(self.args.cpu_profiler_library)
        environment["LD_PRELOAD"] = (
            f"{profiler_library}:{inherited_preload}"
            if inherited_preload
            else profiler_library
        )
        environment["CPUPROFILE"] = str(
            self.args.cpu_profile_dir
            / f"{process_kind}_{runtime.spec.slot:03d}_{runtime.spec.public_serial}.prof"
        )
        environment["CPUPROFILE_FREQUENCY"] = str(self.args.cpu_profile_frequency_hz)
        environment["CPUPROFILESIGNAL"] = str(self.args.cpu_profile_signal)
        return environment

    def start_mock(self, runtime: RobotRuntime) -> ManagedProcess:
        environment = os.environ.copy()
        environment["PYTHONUNBUFFERED"] = "1"
        return self.spawn(
            f"mock {runtime.spec.public_serial}",
            "mock",
            runtime,
            [
                str(self.runtime_python),
                str(self.repo_root / "scripts" / "rdk" / "mock_external_simulator.py"),
                runtime.spec.public_serial,
            ],
            self.repo_root,
            environment,
            "mock.log",
        )

    def start_app(self, runtime: RobotRuntime) -> ManagedProcess:
        return self.spawn(
            f"RobotControlApp {runtime.spec.public_serial}",
            "app",
            runtime,
            [
                str(runtime.path / "RobotControlApp"),
                "-u",
                "./user_data/",
                "-c",
                "./specs/robots/FlexivA02L/flexivCfg.xml",
                "-m",
                "MotionBarSimulation",
                "-s",
                runtime.spec.internal_serial,
                "-x",
                "CX01-02-P1-00034",
                "-n",
                "-g",
            ],
            runtime.path,
            self.process_environment(runtime, "robot_control_app"),
            "rca.log",
        )

    def start_bridge(self, runtime: RobotRuntime) -> ManagedProcess:
        return self.spawn(
            f"FlexivSimulation {runtime.spec.public_serial}",
            "bridge",
            runtime,
            [
                str(self.flexiv_simulation_binary),
                "--robot_urdf",
                str(runtime.path / "user_data/settings/generated_robot_A02LS-M1_abs_path.urdf"),
                "--robot_srdf",
                str(runtime.path / "user_data/settings/generated_robot_A02LS-M1_abs_path.srdf"),
                "--scene_urdf",
                str(runtime.path / "user_data/settings/user_scene_abs_path.urdf"),
                "--param",
                str(runtime.path / runtime.spec.internal_serial / "arm_driver_param.xml"),
                "--group_state",
                "home",
            ],
            self.elem_root,
            self.process_environment(runtime, "flexiv_simulation"),
            "bridge.log",
        )

    def process_for(self, robot: RobotSpec, kind: str) -> ManagedProcess:
        for process in self.processes:
            if process.robot.slot == robot.slot and process.kind == kind:
                return process
        raise FleetError(f"internal error: no {kind} process for {robot.public_serial}")

    def assert_core_processes_running(self) -> None:
        for managed in self.processes:
            if managed.kind == "reader":
                continue
            return_code = managed.process.poll()
            if return_code is not None:
                raise FleetError(
                    f"{managed.name} exited unexpectedly with status {return_code}; "
                    f"see {managed.log_path}\n{tail(managed.log_path)}"
                )

    def wait_for_log(
        self,
        managed: ManagedProcess,
        pattern: str,
        timeout_seconds: int,
        description: str,
    ) -> None:
        self.wait_for_logs([(managed, pattern)], timeout_seconds, description)

    def wait_for_logs(
        self,
        expectations: list[tuple[ManagedProcess, str]],
        timeout_seconds: int,
        description: str,
    ) -> None:
        """Wait for a whole fleet phase using one deadline and parallel log polling."""
        deadline = time.monotonic() + timeout_seconds
        pending = {
            managed.process.pid: (managed, re.compile(pattern))
            for managed, pattern in expectations
        }
        while time.monotonic() < deadline:
            self.check_stop()
            self.assert_core_processes_running()
            completed: list[int] = []
            for pid, (managed, compiled) in pending.items():
                try:
                    contents = managed.log_path.read_text(
                        encoding="utf-8", errors="replace"
                    )
                except OSError:
                    contents = ""
                if compiled.search(contents):
                    completed.append(pid)
            for pid in completed:
                pending.pop(pid)
            if not pending:
                return
            time.sleep(0.25)
        unresolved = [managed for managed, _pattern in pending.values()]
        names = ", ".join(managed.robot.public_serial for managed in unresolved)
        first = unresolved[0]
        raise FleetError(
            f"timed out waiting for {description} for {len(unresolved)}/"
            f"{len(expectations)} robots: {names}; first log: {first.log_path}\n"
            f"{tail(first.log_path)}"
        )

    def write_controller_signal(self, state: str) -> None:
        if state not in ("STARTING", "STOPPED_EXTERNAL_AUTO"):
            raise FleetError(f"unsupported controller signal state: {state}")
        controller_processes = [
            managed
            for managed in self.processes
            if managed.kind in ("app", "bridge")
        ]
        process_kinds_by_slot: dict[int, set[str]] = {}
        for managed in controller_processes:
            process_kinds_by_slot.setdefault(managed.robot.slot, set()).add(managed.kind)
        started_robot_count = sum(
            kinds == {"app", "bridge"} for kinds in process_kinds_by_slot.values()
        )
        if state == "STOPPED_EXTERNAL_AUTO" and started_robot_count != len(self.runtimes):
            raise FleetError("cannot publish controller readiness before all waves start")
        atomic_write_json(
            self.ready_file,
            {
                "schema_version": 1,
                "run_id": self.manifest["run_id"],
                "manifest_generated_at": self.manifest["generated_at"],
                "launcher_pid": os.getpid(),
                "runtime_root": str(self.work_root),
                "state": state,
                "robots": [runtime.spec.public_serial for runtime in self.runtimes],
                "startup": {
                    "wave_size": self.startup_wave_size,
                    "inter_wave_delay_s": self.startup_wave_delay_s,
                    "started_robot_count": started_robot_count,
                    "robot_count": len(self.runtimes),
                },
                "middleware": {
                    "dds_max_initial_peers_range": self.dds_max_initial_peers_range,
                    "dds_mutation_tries": self.dds_mutation_tries,
                },
                "simulation": {
                    "physics_frequency_hz": self.physics_frequency_hz,
                    "flexiv_simulation_timestep_s": self.flexiv_simulation_timestep_s,
                },
                "processes": [
                    {
                        "pid": managed.process.pid,
                        "kind": managed.kind,
                        "serial": managed.robot.public_serial,
                        "log_path": str(managed.log_path),
                    }
                    for managed in controller_processes
                ],
            },
        )

    def wait_between_startup_waves(self) -> None:
        deadline = time.monotonic() + self.startup_wave_delay_s
        while time.monotonic() < deadline:
            self.check_stop()
            self.assert_core_processes_running()
            time.sleep(min(0.25, max(0.0, deadline - time.monotonic())))

    def start_reader(self, runtime: RobotRuntime) -> ManagedProcess:
        environment = os.environ.copy()
        environment["PYTHONUNBUFFERED"] = "1"
        return self.spawn(
            f"RDK reader {runtime.spec.public_serial}",
            "reader",
            runtime,
            [
                str(self.runtime_python),
                str(self.repo_root / "scripts" / "rdk" / "rdk_read_joint_states.py"),
                runtime.spec.public_serial,
            ],
            self.repo_root,
            environment,
            "rdk.log",
        )

    @staticmethod
    def signal_process(managed: ManagedProcess, signum: int) -> None:
        if managed.process.poll() is not None:
            return
        try:
            os.killpg(managed.process.pid, signum)
        except ProcessLookupError:
            pass

    def run_rdk_smoke(self) -> None:
        log(f"starting one read-only RDK client per arm for {self.args.rdk_smoke_seconds}s")
        readers = [self.start_reader(runtime) for runtime in self.runtimes]
        deadline = time.monotonic() + self.args.rdk_smoke_seconds
        while time.monotonic() < deadline:
            self.check_stop()
            self.assert_core_processes_running()
            time.sleep(min(0.25, max(0.0, deadline - time.monotonic())))
        for reader in readers:
            self.signal_process(reader, signal.SIGINT)
        reader_deadline = time.monotonic() + 5
        while time.monotonic() < reader_deadline and any(
            reader.process.poll() is None for reader in readers
        ):
            time.sleep(0.1)
        for reader in readers:
            if reader.process.poll() is None:
                self.signal_process(reader, signal.SIGKILL)
            try:
                reader.process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                pass

        failures: list[str] = []
        for reader in readers:
            contents = reader.log_path.read_text(encoding="utf-8", errors="replace")
            connected = contents.count("Connected to the robot")
            q_count = sum(line.startswith("q:") for line in contents.splitlines())
            tau_count = sum(line.startswith("tau:") for line in contents.splitlines())
            log(
                f"{reader.robot.public_serial}: connected={connected} "
                f"q={q_count} tau={tau_count}"
            )
            if connected < 1 or q_count < 3 or tau_count < 3:
                failures.append(reader.robot.public_serial)
        if failures:
            raise FleetError(
                "RDK smoke did not receive enough samples for: " + ", ".join(failures)
            )
        log(f"PASS: all {len(readers)} serial-specific RDK clients received states")

    def run(self) -> None:
        self.acquire_lock()
        self.ready_signal_owned = True
        self.ready_file.unlink(missing_ok=True)
        self.preflight()
        self.prepare_flexiv_simulation_binary()
        signal.signal(signal.SIGINT, self.request_stop)
        signal.signal(signal.SIGTERM, self.request_stop)

        log(
            f"provisioning {len(self.robots)} arms from {self.source_profile_name} "
            f"under {self.work_root}"
        )
        for robot in self.robots:
            self.check_stop()
            log(
                f"[{robot.slot + 1}/{len(self.robots)}] {robot.public_serial} "
                f"env={robot.env_index} role={robot.role} "
                f"ports={robot.block_base}-{robot.block_end}"
            )
            self.provision_robot(robot)

        if self.args.physics == "mock":
            log("starting one mock physics peer per arm")
            for runtime in self.runtimes:
                self.start_mock(runtime)

        wave_count = (
            len(self.runtimes) + self.startup_wave_size - 1
        ) // self.startup_wave_size
        log(
            f"starting controllers in {wave_count} wave(s) of at most "
            f"{self.startup_wave_size} arm(s)"
        )
        apps: list[ManagedProcess] = []
        for wave_index, start in enumerate(
            range(0, len(self.runtimes), self.startup_wave_size), start=1
        ):
            wave = self.runtimes[start : start + self.startup_wave_size]
            slot_range = f"{wave[0].spec.slot}-{wave[-1].spec.slot}"
            log(
                f"startup wave {wave_index}/{wave_count}: "
                f"arms={len(wave)} slots={slot_range}"
            )
            wave_apps = [self.start_app(runtime) for runtime in wave]
            apps.extend(wave_apps)
            self.wait_for_logs(
                [(app, r"gRPC server listening on") for app in wave_apps],
                self.args.ready_timeout,
                f"wave {wave_index}/{wave_count} gRPC readiness",
            )
            self.wait_for_logs(
                [
                    (
                        app,
                        re.escape(
                            f"basic_stream_data/{runtime.spec.normalized_public_serial}"
                        ),
                    )
                    for runtime, app in zip(wave, wave_apps)
                ],
                15,
                f"wave {wave_index}/{wave_count} serial-specific RDK advertisement",
            )
            self.wait_for_logs(
                [
                    (app, re.escape(f"Listenning on {runtime.spec.time_sync_port}"))
                    for runtime, app in zip(wave, wave_apps)
                ],
                15,
                f"wave {wave_index}/{wave_count} time-sync listener",
            )
            for runtime, app in zip(wave, wave_apps):
                contents = app.log_path.read_text(encoding="utf-8", errors="replace")
                if re.search(
                    r"Lost time sync slave.*bind|Address already in use", contents
                ):
                    raise FleetError(
                        f"{runtime.spec.public_serial} reported a port bind collision; "
                        f"see {app.log_path}"
                    )

            for runtime in wave:
                self.start_bridge(runtime)
            # Publish each admitted wave so monitoring can track startup growth.
            # STOPPED_EXTERNAL_AUTO remains gated on the complete fleet.
            self.write_controller_signal("STARTING")
            self.wait_for_logs(
                [(app, r"STOPPED_EXTERNAL_AUTO") for app in wave_apps],
                self.args.ready_timeout,
                f"wave {wave_index}/{wave_count} external-auto mode",
            )
            log(f"startup wave {wave_index}/{wave_count} is operational")
            if wave_index < wave_count and self.startup_wave_delay_s > 0:
                self.wait_between_startup_waves()
        for runtime in self.runtimes:
            if self.args.physics == "mock":
                mock = self.process_for(runtime.spec, "mock")
                self.wait_for_log(
                    mock,
                    r"Connected with Flexiv Elements Studio",
                    30,
                    "mock physics connection",
                )

        self.write_controller_signal("STOPPED_EXTERNAL_AUTO")
        log(f"READY: {len(self.runtimes)} arms are in STOPPED_EXTERNAL_AUTO")
        for runtime in self.runtimes:
            robot = runtime.spec
            log(
                f"  slot={robot.slot} env={robot.env_index} role={robot.role} "
                f"serial={robot.public_serial} time_sync={robot.time_sync_port}"
            )
        log(f"manifest snapshot: {self.work_root / 'robot_run_manifest.json'}")
        log(f"controller-ready signal: {self.ready_file}")

        if self.args.rdk_smoke_seconds:
            self.run_rdk_smoke()
            return

        log("fleet will remain active until Ctrl+C")
        while True:
            self.check_stop()
            self.assert_core_processes_running()
            time.sleep(0.5)

    def cleanup(self) -> None:
        if self.ready_signal_owned:
            self.ready_file.unlink(missing_ok=True)
        if self.processes:
            log("stopping fleet processes")
        for managed in self.processes:
            initial_signal = signal.SIGINT if managed.kind in ("reader", "mock") else signal.SIGTERM
            self.signal_process(managed, initial_signal)
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline and any(
            managed.process.poll() is None for managed in self.processes
        ):
            time.sleep(0.25)
        for managed in self.processes:
            self.signal_process(managed, signal.SIGKILL)
            try:
                managed.process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                pass

        for runtime in self.runtimes:
            specs = runtime.path / "specs"
            mounted = subprocess.run(
                ["mountpoint", "-q", str(specs)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            ).returncode == 0
            if mounted:
                subprocess.run(
                    ["fusermount", "-uz", str(specs)],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            try:
                self.clean_serial_shm(runtime.spec, fail_if_held=False)
            except OSError as error:
                log(f"warning: cannot clean shared memory for {runtime.spec.public_serial}: {error}")

        if self.lock_stream is not None:
            try:
                fcntl.flock(self.lock_stream, fcntl.LOCK_UN)
            finally:
                self.lock_stream.close()
                self.lock_stream = None
        log(f"logs and private runtimes retained at: {self.work_root}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--manifest", type=Path, help="existing generated run manifest")
    source.add_argument("--num-arms", type=int, help="generate and launch this many arms")
    parser.add_argument(
        "--physics",
        choices=("isaac", "mock"),
        default="isaac",
        help="external physics peer; Isaac peers must already be running",
    )
    parser.add_argument(
        "--rdk-smoke-seconds",
        type=int,
        default=0,
        help="run concurrent read-only RDK clients, validate, and exit",
    )
    parser.add_argument("--ready-timeout", type=int, default=90)
    parser.add_argument(
        "--startup-wave-size",
        type=int,
        help="controller arms admitted per startup wave; overrides the manifest",
    )
    parser.add_argument(
        "--startup-wave-delay-s",
        type=float,
        help="settling delay between operational waves; overrides the manifest",
    )
    parser.add_argument(
        "--elem-root",
        type=Path,
        default=Path(os.environ.get("ELEM_ROOT", "~/flexiv_elements_studio/FlexivElementsStudio")),
    )
    parser.add_argument(
        "--runtime-python",
        type=Path,
        default=Path(
            os.environ.get(
                "PYTHON", "~/miniforge3/envs/flexiv-sim-plugin/bin/python"
            )
        ),
    )
    parser.add_argument(
        "--work-parent",
        type=Path,
        default=Path(os.environ.get("WORK_PARENT", "/tmp")),
    )
    parser.add_argument(
        "--preserve-flexiv-simulation-busy-wait",
        action="store_true",
        help="retain the stock FlexivSimulation busy-spin for controlled A/B tests",
    )
    parser.add_argument(
        "--cpu-profiler-library",
        type=Path,
        help="optional libprofiler.so path for per-process CPU sampling",
    )
    parser.add_argument(
        "--cpu-profile-dir",
        type=Path,
        help="directory for per-process CPU profiles (required with profiler library)",
    )
    parser.add_argument(
        "--cpu-profile-frequency-hz",
        type=int,
        default=100,
        help="CPU profiler sampling frequency (default: 100 Hz)",
    )
    parser.add_argument(
        "--cpu-profile-signal",
        type=int,
        default=12,
        help="unused signal that toggles CPU profiling and flushes output (default: 12)",
    )

    # Options used only by the single-command --num-arms allocation path.
    parser.add_argument("--run-id", default="generated-flexiv-fleet")
    parser.add_argument("--host-ip", default="142.1.44.134")
    parser.add_argument("--port-start", type=int, default=12000)
    parser.add_argument("--port-block-size", type=int, default=64)
    parser.add_argument("--ports-per-robot", type=int, default=32)
    parser.add_argument("--suffix-prefix", default="FLT")
    parser.add_argument("--start-index", type=int, default=0)
    parser.add_argument("--source-profile", default="simulator0")
    parser.add_argument("--source-internal-serial", default="A02LS-05-M1-cBDY86")
    args = parser.parse_args()
    args.elem_root = args.elem_root.expanduser().resolve()
    args.runtime_python = args.runtime_python.expanduser().resolve()
    args.work_parent = args.work_parent.expanduser().resolve()
    if (args.cpu_profiler_library is None) != (args.cpu_profile_dir is None):
        parser.error("--cpu-profiler-library and --cpu-profile-dir must be used together")
    if args.cpu_profiler_library is not None:
        args.cpu_profiler_library = args.cpu_profiler_library.expanduser().resolve()
        args.cpu_profile_dir = args.cpu_profile_dir.expanduser().resolve()
        if not args.cpu_profiler_library.is_file():
            parser.error(f"CPU profiler library not found: {args.cpu_profiler_library}")
        args.cpu_profile_dir.mkdir(parents=True, exist_ok=True)
    if args.cpu_profile_frequency_hz < 1:
        parser.error("--cpu-profile-frequency-hz must be positive")
    if not 1 <= args.cpu_profile_signal <= 64:
        parser.error("--cpu-profile-signal must be in [1, 64]")
    if args.num_arms is not None and args.num_arms < 1:
        parser.error("--num-arms must be positive")
    if args.rdk_smoke_seconds and args.rdk_smoke_seconds < 4:
        parser.error("--rdk-smoke-seconds must be 0 or at least 4")
    if args.ready_timeout < 1:
        parser.error("--ready-timeout must be positive")
    if args.startup_wave_size is not None and args.startup_wave_size < 1:
        parser.error("--startup-wave-size must be positive")
    if args.startup_wave_delay_s is not None and args.startup_wave_delay_s < 0:
        parser.error("--startup-wave-delay-s cannot be negative")
    return args


def make_automatic_manifest(args: argparse.Namespace, output: Path) -> None:
    generator_args = argparse.Namespace(
        output=output,
        run_id=args.run_id,
        num_environments=args.num_arms,
        role=["arm"],
        host_ip=args.host_ip,
        port_start=args.port_start,
        port_block_size=args.port_block_size,
        ports_per_robot=args.ports_per_robot,
        suffix_prefix=args.suffix_prefix,
        start_index=args.start_index,
        existing_profile_root=args.elem_root / "user_data_ui" / "simDir",
        source_profile=args.source_profile,
        source_internal_serial=args.source_internal_serial,
        check_availability=True,
        physics_frequency_hz=2000.0,
    )
    manifest = generate(generator_args)
    atomic_write_json(output, manifest)


def main() -> int:
    args = parse_args()
    if not args.work_parent.is_dir():
        print(f"ERROR: work parent not found: {args.work_parent}", file=sys.stderr)
        return 1
    work_root = Path(tempfile.mkdtemp(prefix="flexiv-generated-fleet.", dir=args.work_parent))
    manifest_snapshot = work_root / "robot_run_manifest.json"
    launcher: FleetLauncher | None = None
    try:
        if args.num_arms is not None:
            make_automatic_manifest(args, manifest_snapshot)
            source_manifest = manifest_snapshot
        else:
            source_manifest = args.manifest.expanduser().resolve()
        manifest, robots = load_manifest(source_manifest)
        if source_manifest != manifest_snapshot:
            shutil.copy2(source_manifest, manifest_snapshot)
        launcher = FleetLauncher(args, manifest, robots, manifest_snapshot, work_root)
        launcher.run()
        return 0
    except StopRequested as error:
        log(f"shutdown requested by {error}")
        return 130
    except (
        FleetError,
        ManifestError,
        CloneError,
        PreparationError,
        SimulationPreparationError,
        OSError,
    ) as error:
        print(f"[generated-fleet] ERROR: {error}", file=sys.stderr, flush=True)
        return 1
    finally:
        if launcher is not None:
            launcher.cleanup()
        else:
            log(f"partial runtime retained at: {work_root}")


if __name__ == "__main__":
    raise SystemExit(main())
