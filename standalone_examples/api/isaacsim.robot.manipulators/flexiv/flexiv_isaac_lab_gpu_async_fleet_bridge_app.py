"""CUDA-native Isaac Lab fleet bridge with asynchronous controller transport."""

from __future__ import annotations

APP_VERSION = "2.13-gpu-async-mounted-cameras"
ASYNC_TRANSPORT_VERSION = "1.4.0-async6"

import hashlib
import json
import math
import os
import re
import time
import traceback
from argparse import ArgumentParser
from collections import deque
from dataclasses import dataclass
from multiprocessing import shared_memory
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from isaaclab.app import AppLauncher

import _flexiv_async_fleet
from flexiv_gpu_async_monitoring import (
    ResourceMonitor,
    append_csv_rows,
    summarize_resource_samples,
)


if _flexiv_async_fleet.__version__ != ASYNC_TRANSPORT_VERSION:
    raise ImportError(
        f"_flexiv_async_fleet=={ASYNC_TRANSPORT_VERSION} is required, "
        f"but found {_flexiv_async_fleet.__version__}"
    )
if not getattr(_flexiv_async_fleet, "dds_shared_participant", False):
    raise ImportError(
        "_flexiv_async_fleet must use the shared DDS participant transport"
    )


@dataclass(frozen=True)
class ManifestRobot:
    slot: int
    env_index: int
    role: str
    public_serial: str
    normalized_public_serial: str


@dataclass
class EndpointRuntime:
    connected: bool = False
    effort_active: bool = False
    fresh_since_s: float | None = None
    ever_activated: bool = False
    stale_events: int = 0
    unsafe_command_active: bool = False
    unsafe_command_events: int = 0


def _mapping(value: Any, label: str) -> dict:
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a mapping")
    return value


def _list(value: Any, label: str) -> list:
    if not isinstance(value, list):
        raise ValueError(f"{label} must be a list")
    return value


def _string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{label} must be a non-empty string")
    return value


def _integer(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{label} must be an integer")
    return value


def _positive_integer(value: Any, label: str) -> int:
    result = _integer(value, label)
    if result < 1:
        raise ValueError(f"{label} must be positive")
    return result


def _positive_float(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be a number")
    result = float(value)
    if result <= 0:
        raise ValueError(f"{label} must be positive")
    return result


def _boolean(value: Any, label: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{label} must be true or false")
    return value


def _load_yaml(path: Path) -> dict:
    try:
        result = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise ValueError(f"cannot read config [{path}]: {error}") from error
    return _mapping(result, "config")


def load_manifest(
    path: Path,
) -> tuple[int, tuple[str, ...], list[ManifestRobot], str, str, Path]:
    """Load the generated controller fleet contract."""
    try:
        manifest = _mapping(json.loads(path.read_text(encoding="utf-8")), "manifest")
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ValueError(f"cannot read manifest [{path}]: {error}") from error
    if _integer(manifest.get("schema_version"), "schema_version") != 1:
        raise ValueError("only robot-run manifest schema_version 1 is supported")
    run_id = _string(manifest.get("run_id"), "run_id")
    generated_at = _string(manifest.get("generated_at"), "generated_at")
    ready_digest = hashlib.sha256(
        f"{run_id}\0{generated_at}".encode("utf-8")
    ).hexdigest()[:20]
    ready_file = Path(
        f"/tmp/flexiv-controller-fleet-{ready_digest}.ready.json"
    )
    declared_ready_file = manifest.get("controller_ready_file")
    if declared_ready_file is not None:
        declared_path = Path(_string(declared_ready_file, "controller_ready_file"))
        if declared_path != ready_file:
            raise ValueError(
                "controller_ready_file does not match manifest run_id/generated_at"
            )

    allocation = _mapping(manifest.get("allocation"), "allocation")
    num_envs = _positive_integer(
        allocation.get("num_environments"), "allocation.num_environments"
    )
    roles = tuple(
        _string(role, f"allocation.roles[{index}]")
        for index, role in enumerate(_list(allocation.get("roles"), "allocation.roles"))
    )
    robot_count = _positive_integer(
        allocation.get("robot_count"), "allocation.robot_count"
    )
    if not roles or len(roles) != len(set(roles)):
        raise ValueError("manifest roles must be non-empty and unique")
    if robot_count != num_envs * len(roles):
        raise ValueError("manifest robot_count must equal environments times roles")

    entries = _list(manifest.get("robots"), "robots")
    if len(entries) != robot_count:
        raise ValueError(f"manifest has {len(entries)} robots; expected {robot_count}")
    result: list[ManifestRobot] = []
    serials: set[str] = set()
    assignments: set[tuple[int, str]] = set()
    for expected_slot, raw in enumerate(entries):
        label = f"robots[{expected_slot}]"
        entry = _mapping(raw, label)
        slot = _integer(entry.get("slot"), f"{label}.slot")
        env_index = _integer(entry.get("env_index"), f"{label}.env_index")
        role = _string(entry.get("role"), f"{label}.role")
        public_serial = _string(entry.get("public_serial"), f"{label}.public_serial")
        normalized = _string(
            entry.get("normalized_public_serial"),
            f"{label}.normalized_public_serial",
        )
        expected_env, role_index = divmod(expected_slot, len(roles))
        if slot != expected_slot:
            raise ValueError(f"{label}.slot must be {expected_slot}")
        if env_index != expected_env or role != roles[role_index]:
            raise ValueError(f"{label} environment/role order is inconsistent")
        if re.fullmatch(r"Rizon4s-[0-9A-Za-z]{6}", public_serial) is None:
            raise ValueError(f"{label} has an invalid Rizon 4s public serial")
        if normalized != public_serial.replace("-", "_"):
            raise ValueError(f"{label}.normalized_public_serial is inconsistent")
        if public_serial in serials or (env_index, role) in assignments:
            raise ValueError(f"{label} duplicates a serial or assignment")
        serials.add(public_serial)
        assignments.add((env_index, role))
        result.append(
            ManifestRobot(
                slot=slot,
                env_index=env_index,
                role=role,
                public_serial=public_serial,
                normalized_public_serial=normalized,
            )
        )
    return num_envs, roles, result, run_id, generated_at, ready_file


argparser = ArgumentParser(description=__doc__)
argparser.add_argument("--config", required=True, type=Path)
argparser.add_argument("--manifest", required=True, type=Path)
args = argparser.parse_args()

config_path = args.config.expanduser().resolve()
manifest_path = args.manifest.expanduser().resolve()
config = _load_yaml(config_path)
(
    manifest_num_envs,
    manifest_roles,
    manifest_robots,
    manifest_run_id,
    manifest_generated_at,
    controller_ready_file,
) = load_manifest(manifest_path)

simulation_config = _mapping(config.get("simulation"), "simulation")
physics_device = _string(simulation_config.get("device"), "simulation.device")
if re.fullmatch(r"cuda(?::[0-9]+)?", physics_device) is None:
    argparser.error("the GPU async bridge requires simulation.device cuda or cuda:<index>")
headless = _boolean(simulation_config.get("headless"), "simulation.headless")
scene_config = _mapping(config.get("scene"), "scene")
scene_graph_instancing_requested = _boolean(
    scene_config.get("scene_graph_instancing"),
    "scene.scene_graph_instancing",
)
camera_rendering_requested = bool(_list(config.get("cameras", []), "cameras"))

launcher_options = {
    "headless": headless,
    "width": 1920,
    "height": 1080,
    "device": physics_device,
    "enable_cameras": camera_rendering_requested,
}
if not camera_rendering_requested:
    launcher_options["experience"] = "isaacsim.exp.base.python.kit"
app_launcher = AppLauncher(launcher_options)
simulation_app = app_launcher.app

import omni.kit.app
import omni.usd

if camera_rendering_requested:
    extension_manager = omni.kit.app.get_app().get_extension_manager()
    if not extension_manager.is_extension_enabled("isaacsim.sensors.camera"):
        extension_manager.set_extension_enabled_immediate(
            "isaacsim.sensors.camera", True
        )

import spdlog
import torch
from flexiv_isaac_runtime_settings import configure_scene_graph_instancing
import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils import configclass
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.rotations import gf_quat_to_np_array, lookat_to_quatf
from isaacsim.sensors.camera import Camera
from pxr import Gf


scene_graph_instancing_enabled = configure_scene_graph_instancing(
    scene_graph_instancing_requested
)

CAMERA_SHM_MAGIC = 0x495341414343414D
CAMERA_SHM_VERSION = 1
CAMERA_SHM_HEADER_FIELDS = 8
CAMERA_SHM_HEADER_BYTES = CAMERA_SHM_HEADER_FIELDS * np.dtype(np.int64).itemsize
CAMERA_SHM_CHANNELS = 3
CAMERA_SHM_DTYPE = np.uint8
CAMERA_SHM_DTYPE_CODE = 2

CLOCK_SHM_MAGIC = 0x4953414143434C4B
CLOCK_SHM_VERSION = 1
CLOCK_SHM_HEADER_FIELDS = 8
CLOCK_SHM_HEADER_BYTES = CLOCK_SHM_HEADER_FIELDS * np.dtype(np.int64).itemsize


@configclass
class GpuFleetSceneCfg(InteractiveSceneCfg):
    """Cloned workcells whose articulations are controlled through one CUDA view."""


def _role_configs(document: dict) -> tuple[dict[str, dict], list[str]]:
    result: dict[str, dict] = {}
    ordered: list[str] = []
    for index, raw in enumerate(_list(document.get("robots"), "robots")):
        robot = _mapping(raw, f"robots[{index}]")
        role = _string(robot.get("role"), f"robots[{index}].role")
        prim_name = _string(
            robot.get("prim_name", role), f"robots[{index}].prim_name"
        )
        if re.fullmatch(r"[A-Za-z0-9_]+", role) is None:
            raise ValueError(f"robots[{index}].role is invalid")
        if re.fullmatch(r"[A-Za-z0-9_]+", prim_name) is None:
            raise ValueError(f"robots[{index}].prim_name is invalid")
        if role in result:
            raise ValueError(f"duplicate robot role [{role}]")
        rendered = dict(robot)
        rendered["prim_name"] = prim_name
        initial_q = rendered.get(
            "initial_q", [0.0, -0.698132, 0.0, 1.5708, 0.0, 0.698132, 0.0]
        )
        if not isinstance(initial_q, list) or len(initial_q) != 7:
            raise ValueError(f"robots[{index}].initial_q must contain seven values")
        rendered["initial_q"] = [float(value) for value in initial_q]
        initial_gripper_state = rendered.get("initial_gripper_state")
        if initial_gripper_state is not None:
            if not isinstance(initial_gripper_state, str):
                raise ValueError(
                    f"robots[{index}].initial_gripper_state must be a string"
                )
            initial_gripper_state = initial_gripper_state.strip().lower()
            if initial_gripper_state not in {"open", "closed"}:
                raise ValueError(
                    f"robots[{index}].initial_gripper_state must be open or closed"
                )
            rendered["initial_gripper_state"] = initial_gripper_state
        result[role] = rendered
        ordered.append(role)
    if not ordered:
        raise ValueError("config must define at least one robot role")
    return result, ordered


role_configs, ordered_roles = _role_configs(config)
if tuple(ordered_roles) != manifest_roles:
    raise ValueError(
        f"config roles {ordered_roles} do not match manifest roles {list(manifest_roles)}"
    )


def _make_scene_config(document: dict) -> GpuFleetSceneCfg:
    scene = _mapping(document.get("scene"), "scene")
    num_envs = _positive_integer(scene.get("num_envs"), "scene.num_envs")
    if num_envs != manifest_num_envs:
        raise ValueError(
            f"scene.num_envs={num_envs} does not match manifest {manifest_num_envs}"
        )
    scene_cfg = GpuFleetSceneCfg(
        num_envs=num_envs,
        env_spacing=float(scene.get("env_spacing", 5.0)),
        replicate_physics=bool(scene.get("replicate_physics", True)),
        filter_collisions=bool(scene.get("filter_collisions", True)),
        clone_in_fabric=bool(scene.get("clone_in_fabric", False)),
    )
    def usd_path(value: Any, field: str) -> str:
        path = _string(value, field)
        if "://" in path:
            return path
        return str((config_path.parent / Path(path).expanduser()).resolve())

    env_usd = usd_path(scene.get("env_usd"), "scene.env_usd")
    setattr(
        scene_cfg,
        "workspace",
        AssetBaseCfg(
            prim_path="{ENV_REGEX_NS}/Workspace",
            spawn=sim_utils.UsdFileCfg(usd_path=env_usd),
        ),
    )
    for index, role in enumerate(ordered_roles):
        robot = role_configs[role]
        position = tuple(float(robot["position"][axis]) for axis in ("x", "y", "z"))
        orientation = tuple(
            float(robot["orientation"][axis]) for axis in ("w", "x", "y", "z")
        )
        setattr(
            scene_cfg,
            f"flexiv_{index}_{role}",
            AssetBaseCfg(
                prim_path=f"{{ENV_REGEX_NS}}/Flexiv_{robot['prim_name']}",
                spawn=sim_utils.UsdFileCfg(usd_path=usd_path(robot["usd"], "robot.usd")),
                init_state=AssetBaseCfg.InitialStateCfg(pos=position, rot=orientation),
            ),
        )
    return scene_cfg


@dataclass
class StateStagingSlot:
    storage: torch.Tensor
    start_event: torch.cuda.Event
    end_event: torch.cuda.Event
    cycle: int = 0
    busy: bool = False


@dataclass
class TransferTimingSlot:
    start_event: torch.cuda.Event
    end_event: torch.cuda.Event
    busy: bool = False


@dataclass
class CameraSharedMemoryData:
    name: str
    camera: Camera
    shm: shared_memory.SharedMemory
    header: np.ndarray
    rgb: np.ndarray
    last_rendering_time: float = -1.0


class GpuAsyncFleetBridge:
    ROBOT_DOF = 7
    # Grav authors every other finger linkage as a PhysX mimic of this single
    # driven joint. Commanding a mimic joint independently leaves the mechanism
    # partially open.
    GRIPPER_JOINT_NAMES = ("finger_joint",)
    GRIPPER_STATE_JOINT_SIGNS = {
        "finger_joint": 1.0,
        "left_inner_knuckle_joint": 1.0,
        "right_inner_knuckle_joint": 1.0,
        "right_outer_knuckle_joint": 1.0,
        "left_outer_finger_joint": -1.0,
        "right_outer_finger_joint": -1.0,
    }
    GRIPPER_POSITIONS_RAD = {
        "open": (math.radians(45.0),),
        "closed": (math.radians(-8.88),),
    }

    def __init__(self) -> None:
        self._logger = spdlog.ConsoleLogger("Flexiv-Isaac GPU Async Fleet Bridge")
        self._logger.info("————————————————————————————————————————————————————————————")
        self._logger.info(f"——— Flexiv-Isaac GPU Async Fleet Bridge v{APP_VERSION} ———")
        self._logger.info("————————————————————————————————————————————————————————————")
        self._logger.info(
            "OmniHydra scene-graph instancing: "
            f"{str(scene_graph_instancing_enabled).lower()}"
        )

        self._physics_frequency = _positive_float(
            simulation_config.get("physics_frequency_hz"),
            "simulation.physics_frequency_hz",
        )
        self._rendering_frequency = _positive_float(
            simulation_config.get("rendering_frequency_hz"),
            "simulation.rendering_frequency_hz",
        )
        self._render = _boolean(simulation_config.get("render"), "simulation.render")
        raw_clock_shm_name = simulation_config.get("clock_shared_memory_name", "")
        if not isinstance(raw_clock_shm_name, str):
            raise ValueError("simulation.clock_shared_memory_name must be a string")
        self._clock_shm_name = raw_clock_shm_name.removeprefix("/")
        if self._clock_shm_name and (
            "/" in self._clock_shm_name
            or re.fullmatch(r"[A-Za-z0-9_]+", self._clock_shm_name) is None
        ):
            raise ValueError(
                "simulation.clock_shared_memory_name must be one POSIX "
                "shared-memory name"
            )
        self._camera_configs = _list(config.get("cameras", []), "cameras")
        if self._camera_configs and not self._render:
            raise ValueError("camera capture requires simulation.render=true")

        monitoring = _mapping(config.get("monitoring"), "monitoring")
        self._monitoring_enabled = _boolean(
            monitoring.get("enabled"), "monitoring.enabled"
        )
        self._resource_sample_interval_s = _positive_float(
            monitoring.get("resource_sample_interval_s"),
            "monitoring.resource_sample_interval_s",
        )
        self._resource_csv = Path(
            _string(
                monitoring.get("resource_samples_csv"),
                "monitoring.resource_samples_csv",
            )
        ).expanduser().resolve()
        self._correctness_enabled = _boolean(
            monitoring.get("correctness_enabled"),
            "monitoring.correctness_enabled",
        )
        self._correctness_interval = _positive_integer(
            monitoring.get("correctness_sample_interval_steps"),
            "monitoring.correctness_sample_interval_steps",
        )
        self._correctness_csv = Path(
            _string(
                monitoring.get("correctness_csv"), "monitoring.correctness_csv"
            )
        ).expanduser().resolve()
        self._effort_error_tolerance_nm = _positive_float(
            monitoring.get("effort_error_tolerance_nm"),
            "monitoring.effort_error_tolerance_nm",
        )
        self._require_resource_samples = _boolean(
            monitoring.get("require_resource_samples"),
            "monitoring.require_resource_samples",
        )
        self._require_correctness_pass = _boolean(
            monitoring.get("require_correctness_pass"),
            "monitoring.require_correctness_pass",
        )
        self._torque_trace_enabled = _boolean(
            monitoring.get("torque_trace_enabled", False),
            "monitoring.torque_trace_enabled",
        )
        self._torque_trace_label = _string(
            monitoring.get("torque_trace_label", manifest_run_id),
            "monitoring.torque_trace_label",
        )
        self._torque_trace_csv = (
            Path(
                _string(
                    monitoring.get("torque_trace_csv"),
                    "monitoring.torque_trace_csv",
                )
            )
            .expanduser()
            .resolve()
            if self._torque_trace_enabled
            else None
        )
        if self._require_resource_samples and not self._monitoring_enabled:
            raise ValueError(
                "monitoring.require_resource_samples needs monitoring.enabled"
            )
        if self._require_correctness_pass and not self._correctness_enabled:
            raise ValueError(
                "monitoring.require_correctness_pass needs correctness_enabled"
            )

        transport = _mapping(config.get("transport"), "transport")
        self._queue_capacity = _positive_integer(
            transport.get("state_queue_capacity"), "transport.state_queue_capacity"
        )
        self._wait_timeout_ms = _positive_integer(
            transport.get("command_wait_timeout_ms"),
            "transport.command_wait_timeout_ms",
        )
        self._same_cycle_command_deadline_us = _positive_integer(
            transport.get("same_cycle_command_deadline_us"),
            "transport.same_cycle_command_deadline_us",
        )
        self._require_controller_ready = _boolean(
            transport.get("require_controller_ready_signal"),
            "transport.require_controller_ready_signal",
        )
        self._controller_ready_poll_s = _positive_float(
            transport.get("controller_ready_poll_s"),
            "transport.controller_ready_poll_s",
        )
        self._max_command_age_steps = _positive_integer(
            transport.get("max_command_age_steps"),
            "transport.max_command_age_steps",
        )
        raw_command_limits = transport.get("max_abs_command_nm")
        if isinstance(raw_command_limits, list):
            if len(raw_command_limits) != self.ROBOT_DOF:
                raise ValueError(
                    "transport.max_abs_command_nm must contain seven joint limits"
                )
            self._max_abs_command_nm = np.asarray(
                [
                    _positive_float(value, f"transport.max_abs_command_nm[{index}]")
                    for index, value in enumerate(raw_command_limits)
                ],
                dtype=np.float32,
            )
        else:
            self._max_abs_command_nm = np.full(
                self.ROBOT_DOF,
                _positive_float(
                    raw_command_limits, "transport.max_abs_command_nm"
                ),
                dtype=np.float32,
            )
        self._activation_delay_s = _positive_float(
            transport.get("effort_activation_delay_s"),
            "transport.effort_activation_delay_s",
        )
        self._recovery_delay_s = _positive_float(
            transport.get("effort_recovery_delay_s"),
            "transport.effort_recovery_delay_s",
        )
        self._health_interval = _positive_integer(
            transport.get("health_report_cycles"), "transport.health_report_cycles"
        )
        self._performance_interval = _positive_integer(
            transport.get("performance_report_cycles"),
            "transport.performance_report_cycles",
        )
        staging_slots = _positive_integer(
            transport.get("gpu_state_staging_slots"),
            "transport.gpu_state_staging_slots",
        )

        # Start endpoint workers before scene construction so their one-time DDS
        # initialization overlaps Isaac/USD setup rather than filling state queues.
        self._transport = _flexiv_async_fleet.AsyncFleet(
            [robot.normalized_public_serial for robot in manifest_robots],
            self.ROBOT_DOF,
            self._queue_capacity,
            self._wait_timeout_ms,
        )

        SimulationManager.set_physics_sim_device(physics_device)
        self._world = World(
            stage_units_in_meters=1.0,
            physics_dt=1.0 / self._physics_frequency,
            rendering_dt=1.0 / self._rendering_frequency,
            set_defaults=False,
            backend="torch",
            device=physics_device,
        )
        self._lab_scene = InteractiveScene(_make_scene_config(config))
        self._world.reset()

        self._view = self._world.scene.add(
            Articulation(
                prim_paths_expr="/World/envs/env_.*/Flexiv_.*",
                name="gpu_async_all_fleet_robots",
                reset_xform_properties=False,
            )
        )
        self._world.reset()
        self._validate_gpu_runtime()
        self._initialize_view_mapping()
        self._cameras: list[Camera] = []
        self._camera_shms: list[CameraSharedMemoryData] = []
        self._initialize_cameras()

        self._joint_indices = torch.arange(
            self.ROBOT_DOF, dtype=torch.int64, device=physics_device
        )
        self._all_view_rows = torch.arange(
            len(manifest_robots), dtype=torch.int64, device=physics_device
        )
        self._initial_manifest = torch.tensor(
            [role_configs[robot.role]["initial_q"] for robot in manifest_robots],
            dtype=torch.float32,
            device=physics_device,
        )
        self._initial_view = torch.empty_like(self._initial_manifest)
        self._initial_view.index_copy_(
            0, self._manifest_to_view, self._initial_manifest
        )
        self._hold_positions_manifest = self._initial_manifest.clone()
        self._initialize_gripper_positions()

        robot_count = len(manifest_robots)
        self._state_slots = [
            StateStagingSlot(
                storage=torch.empty(
                    (2, robot_count, self.ROBOT_DOF),
                    dtype=torch.float32,
                    pin_memory=True,
                ),
                start_event=torch.cuda.Event(enable_timing=True),
                end_event=torch.cuda.Event(enable_timing=True),
            )
            for _ in range(staging_slots)
        ]
        self._pending_state_slots: deque[int] = deque()
        self._command_cpu = torch.zeros(
            (robot_count, self.ROBOT_DOF), dtype=torch.float32, pin_memory=True
        )
        self._command_numpy = self._command_cpu.numpy()
        self._command_cycles = np.zeros(robot_count, dtype=np.uint64)
        self._connected = np.zeros(robot_count, dtype=np.uint8)
        self._command_valid = np.zeros(robot_count, dtype=np.uint8)
        self._command_gpu = torch.zeros(
            (robot_count, self.ROBOT_DOF),
            dtype=torch.float32,
            device=physics_device,
        )
        self._h2d_slots = [
            TransferTimingSlot(
                start_event=torch.cuda.Event(enable_timing=True),
                end_event=torch.cuda.Event(enable_timing=True),
            )
            for _ in range(16)
        ]
        self._endpoint_runtime = [EndpointRuntime() for _ in manifest_robots]
        self._active_manifest_rows: list[int] = []
        self._controller_process_metadata: list[dict[str, Any]] = []
        self._controller_runtime_root: Path | None = None
        self._servo_cycle = 0
        self._clock_shm: shared_memory.SharedMemory | None = None
        self._clock_header: np.ndarray | None = None
        self._initialize_simulation_clock()
        self._state_staging_overruns = 0
        self._native_state_drops = 0
        self._command_deadline_misses = 0
        self._d2h_samples_ms: list[float] = []
        self._h2d_samples_ms: list[float] = []
        self._command_deadline_samples_ms: list[float] = []
        self._controller_ready = not self._require_controller_ready
        self._last_controller_ready_poll_s = 0.0
        self._controller_ready_signature: tuple[int, int] | None = None
        self._controller_launcher_pid = 0
        self._controller_signal_state: str | None = None
        self._next_health_cycle = self._health_interval
        self._next_performance_cycle = self._performance_interval
        self._performance_start_cycle = 0
        self._performance_start_time = time.perf_counter()
        self._last_q_manifest = self._initial_manifest
        self._last_dq_manifest = torch.zeros_like(self._initial_manifest)
        self._measurement_active = False
        self._measurement_start_cycle = 0
        self._torque_trace_rows: list[dict[str, Any]] = []
        self._minimum_active_rows = robot_count
        self._inactive_row_cycles = 0
        self._measurement_max_command_nm = 0.0
        self._reset_correctness_statistics()
        gpu_index = int(physics_device.split(":", 1)[1]) if ":" in physics_device else 0
        self._resource_monitor = (
            ResourceMonitor(
                run_id=manifest_run_id,
                manifest_generated_at=manifest_generated_at,
                config_path=config_path,
                manifest_path=manifest_path,
                num_envs=manifest_num_envs,
                robot_count=robot_count,
                output_path=self._resource_csv,
                sample_interval_s=self._resource_sample_interval_s,
                gpu_index=gpu_index,
            )
            if self._monitoring_enabled
            else None
        )
        ratio = self._physics_frequency / self._rendering_frequency
        self._render_interval = max(1, int(round(ratio)))
        self._reset_all_rows()
        if self._resource_monitor is not None:
            self._resource_monitor.set_context("startup", self._servo_cycle)
            self._resource_monitor.start()
        self._logger.info(
            f"READY: CUDA fleet rows={robot_count}, device={physics_device}, "
            f"state staging slots={staging_slots}, native queue={self._queue_capacity}, "
            f"same-cycle command deadline={self._same_cycle_command_deadline_us} us, "
            f"controller ready signal={controller_ready_file}, "
            f"max command age={self._max_command_age_steps} cycles, "
            f"resource monitoring={self._monitoring_enabled}, "
            f"correctness monitoring={self._correctness_enabled}"
        )

    @staticmethod
    def _camera_shm_name(camera_name: str) -> str:
        return "isaac_sim_camera_" + re.sub(r"[^0-9A-Za-z_]", "_", camera_name)

    @staticmethod
    def _camera_vector(value: Any, label: str) -> np.ndarray:
        mapping = _mapping(value, label)
        result = np.asarray(
            [float(mapping[axis]) for axis in ("x", "y", "z")], dtype=np.float64
        )
        if not np.all(np.isfinite(result)):
            raise ValueError(f"{label} must contain finite values")
        return result

    @staticmethod
    def _camera_quaternion(value: Any, label: str) -> np.ndarray:
        mapping = _mapping(value, label)
        result = np.asarray(
            [float(mapping[axis]) for axis in ("w", "x", "y", "z")],
            dtype=np.float64,
        )
        if not np.all(np.isfinite(result)):
            raise ValueError(f"{label} must contain finite values")
        norm = float(np.linalg.norm(result))
        if norm < 1.0e-12:
            raise ValueError(f"{label} must be a nonzero quaternion")
        return result / norm

    def _camera_mount_path(self, value: Any, label: str) -> str:
        mount = _mapping(value, label)
        env_index = _integer(mount.get("env_index"), f"{label}.env_index")
        if env_index < 0 or env_index >= len(self._lab_scene.env_prim_paths):
            raise ValueError(
                f"{label}.env_index [{env_index}] is outside [0, "
                f"{len(self._lab_scene.env_prim_paths) - 1}]"
            )
        robot_role = _string(mount.get("robot_role"), f"{label}.robot_role")
        if robot_role not in role_configs:
            raise ValueError(
                f"{label}.robot_role [{robot_role}] is not one of "
                f"{list(role_configs)}"
            )
        link_prim = _string(mount.get("link_prim"), f"{label}.link_prim")
        if re.fullmatch(r"[A-Za-z0-9_]+", link_prim) is None:
            raise ValueError(f"{label}.link_prim must be one USD prim name")
        robot_prim_name = role_configs[robot_role]["prim_name"]
        env_path = self._lab_scene.env_prim_paths[env_index]
        parent_path = f"{env_path}/Flexiv_{robot_prim_name}/{link_prim}"
        parent_prim = omni.usd.get_context().get_stage().GetPrimAtPath(parent_path)
        if not parent_prim.IsValid():
            raise ValueError(
                f"{label} resolves to missing camera parent prim [{parent_path}]"
            )
        return parent_path

    @staticmethod
    def _create_shared_memory(name: str, size: int) -> shared_memory.SharedMemory:
        try:
            return shared_memory.SharedMemory(create=True, size=size, name=name)
        except FileExistsError:
            stale = shared_memory.SharedMemory(name=name)
            stale.close()
            stale.unlink()
            return shared_memory.SharedMemory(create=True, size=size, name=name)

    def _initialize_simulation_clock(self) -> None:
        if not self._clock_shm_name:
            return
        shm = self._create_shared_memory(
            self._clock_shm_name, CLOCK_SHM_HEADER_BYTES
        )
        header = np.ndarray(
            (CLOCK_SHM_HEADER_FIELDS,), dtype=np.int64, buffer=shm.buf
        )
        header[:] = (
            CLOCK_SHM_MAGIC,
            CLOCK_SHM_VERSION,
            0,
            0,
            0,
            round(self._physics_frequency * 1000.0),
            time.monotonic_ns(),
            0,
        )
        self._clock_shm = shm
        self._clock_header = header
        self._logger.info(
            f"Simulation clock publishes {self._physics_frequency:.3f} Hz "
            f"through shared memory [{self._clock_shm_name}]"
        )

    def _write_simulation_clock(self) -> None:
        header = self._clock_header
        if header is None:
            return
        sequence = int(header[2])
        write_sequence = sequence + 1 if sequence % 2 == 0 else sequence + 2
        header[2] = write_sequence
        header[3] = self._servo_cycle
        header[4] = round(
            self._servo_cycle * 1_000_000_000.0 / self._physics_frequency
        )
        header[6] = time.monotonic_ns()
        header[2] = write_sequence + 1

    def _initialize_cameras(self) -> None:
        observed_names: set[str] = set()
        for index, raw in enumerate(self._camera_configs):
            label = f"cameras[{index}]"
            camera_config = _mapping(raw, label)
            name = _string(camera_config.get("name"), f"{label}.name")
            if re.fullmatch(r"[A-Za-z0-9_]+", name) is None:
                raise ValueError(f"{label}.name is invalid")
            if name in observed_names:
                raise ValueError(f"duplicate camera name [{name}]")
            observed_names.add(name)
            fps = _positive_integer(camera_config.get("fps"), f"{label}.fps")
            raw_resolution = _list(
                camera_config.get("resolution"), f"{label}.resolution"
            )
            if len(raw_resolution) != 2:
                raise ValueError(f"{label}.resolution must contain width and height")
            resolution = tuple(
                _positive_integer(value, f"{label}.resolution[{axis}]")
                for axis, value in enumerate(raw_resolution)
            )
            if any(value % 2 for value in resolution):
                raise ValueError(f"{label}.resolution values must be even")
            raw_mount = camera_config.get("mount")
            if raw_mount is None:
                if (
                    "local_position" in camera_config
                    or "local_orientation" in camera_config
                ):
                    raise ValueError(
                        f"{label}.local_position/local_orientation require "
                        f"{label}.mount"
                    )
                position = self._camera_vector(
                    camera_config.get("position"), f"{label}.position"
                )
                target = self._camera_vector(
                    camera_config.get("target"), f"{label}.target"
                )
                if np.linalg.norm(position - target) < 1.0e-6:
                    raise ValueError(f"{label}.position and target must differ")
                # USD cameras look down local -Z. This helper is intentionally
                # called target-first so local +Z points from the target back to
                # the camera.
                orientation = gf_quat_to_np_array(
                    lookat_to_quatf(
                        Gf.Vec3d(*target),
                        Gf.Vec3d(*position),
                        Gf.Vec3d(0, 0, 1),
                    )
                )
                camera = Camera(
                    prim_path=f"/World/{name}",
                    name=name,
                    frequency=fps,
                    resolution=resolution,
                    position=position,
                    orientation=orientation,
                    annotator_device="cpu",
                )
                camera.set_world_pose(
                    position=position,
                    orientation=orientation,
                    camera_axes="usd",
                )
                pose_description = (
                    f"world position={position.tolist()}, target={target.tolist()}"
                )
            else:
                if "position" in camera_config or "target" in camera_config:
                    raise ValueError(
                        f"{label}.mount cannot be combined with position/target"
                    )
                parent_path = self._camera_mount_path(raw_mount, f"{label}.mount")
                translation = self._camera_vector(
                    camera_config.get("local_position"), f"{label}.local_position"
                )
                orientation = self._camera_quaternion(
                    camera_config.get("local_orientation"),
                    f"{label}.local_orientation",
                )
                camera = Camera(
                    prim_path=f"{parent_path}/{name}",
                    name=name,
                    frequency=fps,
                    resolution=resolution,
                    annotator_device="cpu",
                )
                # MuJoCo and USD cameras both look down local -Z with +Y up, so
                # a camera transform authored in the robot attachment frame can
                # be applied directly in USD camera axes.
                camera.set_local_pose(
                    translation=translation,
                    orientation=orientation,
                    camera_axes="usd",
                )
                pose_description = (
                    f"mounted on [{parent_path}] at local position="
                    f"{translation.tolist()}, local orientation wxyz="
                    f"{orientation.tolist()}"
                )
            near_clipping_distance = _positive_float(
                camera_config.get("near_clipping_distance_m", 0.01),
                f"{label}.near_clipping_distance_m",
            )
            _, far_clipping_distance = camera.get_clipping_range()
            if near_clipping_distance >= far_clipping_distance:
                raise ValueError(
                    f"{label}.near_clipping_distance_m must be below the "
                    f"far clipping distance [{far_clipping_distance}]"
                )
            camera.set_clipping_range(near_distance=near_clipping_distance)
            raw_focal_length = camera_config.get("focal_length")
            raw_vertical_fov = camera_config.get("vertical_fov_deg")
            if (raw_focal_length is None) == (raw_vertical_fov is None):
                raise ValueError(
                    f"{label} must define exactly one of focal_length or "
                    "vertical_fov_deg"
                )
            if raw_vertical_fov is not None:
                vertical_fov_deg = _positive_float(
                    raw_vertical_fov, f"{label}.vertical_fov_deg"
                )
                if vertical_fov_deg >= 180.0:
                    raise ValueError(f"{label}.vertical_fov_deg must be below 180")
                focal_length = camera.get_vertical_aperture() / (
                    2.0 * math.tan(math.radians(vertical_fov_deg) / 2.0)
                )
                camera.set_focal_length(focal_length)
                lens_description = f"vertical FOV={vertical_fov_deg:.3f} deg"
            else:
                focal_length = _positive_float(
                    raw_focal_length, f"{label}.focal_length"
                )
                camera.set_focal_length(focal_length)
                lens_description = f"focal length={focal_length:.6f} stage units"
            camera.initialize()
            effective_vertical_fov_deg = math.degrees(
                2.0
                * math.atan(
                    camera.get_vertical_aperture()
                    / (2.0 * camera.get_focal_length())
                )
            )
            if raw_vertical_fov is not None:
                lens_description = (
                    f"requested vertical FOV={vertical_fov_deg:.3f} deg, "
                    f"effective vertical FOV={effective_vertical_fov_deg:.3f} deg"
                )
            else:
                lens_description += (
                    f", effective vertical FOV={effective_vertical_fov_deg:.3f} deg"
                )
            near_clipping_distance, far_clipping_distance = camera.get_clipping_range()
            self._cameras.append(camera)

            width, height = camera.get_resolution()
            shm_name = self._camera_shm_name(name)
            payload_bytes = height * width * CAMERA_SHM_CHANNELS
            shm = self._create_shared_memory(
                shm_name, CAMERA_SHM_HEADER_BYTES + payload_bytes
            )
            header = np.ndarray(
                (CAMERA_SHM_HEADER_FIELDS,), dtype=np.int64, buffer=shm.buf
            )
            rgb = np.ndarray(
                (height, width, CAMERA_SHM_CHANNELS),
                dtype=CAMERA_SHM_DTYPE,
                buffer=shm.buf,
                offset=CAMERA_SHM_HEADER_BYTES,
            )
            header[:] = (
                CAMERA_SHM_MAGIC,
                CAMERA_SHM_VERSION,
                0,
                0,
                height,
                width,
                CAMERA_SHM_CHANNELS,
                CAMERA_SHM_DTYPE_CODE,
            )
            self._camera_shms.append(
                CameraSharedMemoryData(shm_name, camera, shm, header, rgb)
            )
            self._logger.info(
                f"Camera [{name}] publishes {width}x{height} RGB at {fps} Hz "
                f"with {lens_description}, clipping range="
                f"[{near_clipping_distance:.6f}, {far_clipping_distance:.6f}] m "
                f"through shared memory [{shm_name}]; {pose_description}"
            )

    def _write_camera_frames(self) -> None:
        for camera_shm in self._camera_shms:
            frame = camera_shm.camera.get_current_frame()
            rendering_time = frame.get("rendering_time")
            if rendering_time is None or rendering_time == camera_shm.last_rendering_time:
                continue
            rgba = frame.get("rgb")
            if isinstance(rgba, dict) and "data" in rgba:
                rgba = rgba["data"]
            if rgba is None:
                continue
            rgb = np.ascontiguousarray(rgba[..., :3], dtype=CAMERA_SHM_DTYPE)
            if rgb.shape != camera_shm.rgb.shape:
                transposed = (
                    camera_shm.rgb.shape[1],
                    camera_shm.rgb.shape[0],
                    camera_shm.rgb.shape[2],
                )
                if rgb.shape != transposed:
                    continue
                rgb = np.ascontiguousarray(np.transpose(rgb, (1, 0, 2)))
            sequence = int(camera_shm.header[2])
            write_sequence = sequence + 1 if sequence % 2 == 0 else sequence + 2
            camera_shm.header[2] = write_sequence
            np.copyto(camera_shm.rgb, rgb)
            camera_shm.header[3] = time.monotonic_ns()
            camera_shm.header[2] = write_sequence + 1
            camera_shm.last_rendering_time = float(rendering_time)

    def _validate_gpu_runtime(self) -> None:
        context = self._world.get_physics_context()
        actual_device = SimulationManager.get_physics_sim_device()
        gpu_dynamics = context.is_gpu_dynamics_enabled()
        broadphase = context.get_broadphase_type()
        fabric_enabled = (
            omni.kit.app.get_app()
            .get_extension_manager()
            .is_extension_enabled("omni.physx.fabric")
        )
        if (
            not actual_device.startswith("cuda")
            or not gpu_dynamics
            or broadphase != "GPU"
            or not fabric_enabled
        ):
            raise RuntimeError(
                "GPU runtime activation failed: "
                f"requested={physics_device} actual={actual_device} "
                f"gpu_dynamics={gpu_dynamics} broadphase={broadphase} "
                f"fabric={fabric_enabled}"
            )
        self._logger.info(
            f"Validated GPU runtime: device={actual_device}, "
            f"gpu_dynamics={gpu_dynamics}, broadphase={broadphase}, "
            f"fabric={fabric_enabled}"
        )

    def _initialize_view_mapping(self) -> None:
        expected_paths: list[str] = []
        for robot in manifest_robots:
            prim_name = role_configs[robot.role]["prim_name"]
            env_path = self._lab_scene.env_prim_paths[robot.env_index]
            expected_paths.append(f"{env_path}/Flexiv_{prim_name}")
        paths = list(self._view.prim_paths)
        if self._view.count != len(manifest_robots):
            raise RuntimeError(
                f"global articulation view has {self._view.count} rows; "
                f"expected {len(manifest_robots)}"
            )
        if self._view.num_dof < self.ROBOT_DOF:
            raise RuntimeError(
                f"global articulation view has {self._view.num_dof} DoF; "
                f"expected at least {self.ROBOT_DOF}"
            )
        if set(paths) != set(expected_paths):
            raise RuntimeError(
                f"global articulation paths do not match manifest: "
                f"observed={paths}, expected={expected_paths}"
            )
        mapping = [paths.index(path) for path in expected_paths]
        self._manifest_to_view = torch.tensor(
            mapping, dtype=torch.int64, device=physics_device
        )
        self._logger.info(
            f"Validated global CUDA articulation view shape="
            f"({self._view.count}, {self.ROBOT_DOF}), "
            f"manifest_to_view={mapping}"
        )

    def _initialize_gripper_positions(self) -> None:
        states = [
            role_configs[robot.role].get("initial_gripper_state")
            for robot in manifest_robots
        ]
        self._gripper_joint_indices: torch.Tensor | None = None
        self._initial_gripper_view: torch.Tensor | None = None
        self._gripper_state_joint_indices: torch.Tensor | None = None
        self._initial_gripper_state_view: torch.Tensor | None = None
        self._gripper_report_pending = False
        if not any(state is not None for state in states):
            return
        if any(state is None for state in states):
            raise ValueError(
                "initial_gripper_state must be defined for every robot or omitted "
                "for every robot"
            )
        dof_names = list(self._view.dof_names)
        missing = [
            name for name in self.GRIPPER_JOINT_NAMES if name not in dof_names
        ]
        if missing:
            raise RuntimeError(
                f"configured gripper state needs missing articulation DoFs {missing}; "
                f"available={dof_names}"
            )
        self._gripper_joint_indices = torch.tensor(
            [dof_names.index(name) for name in self.GRIPPER_JOINT_NAMES],
            dtype=torch.int64,
            device=physics_device,
        )
        initial_manifest = torch.tensor(
            [self.GRIPPER_POSITIONS_RAD[state] for state in states],
            dtype=torch.float32,
            device=physics_device,
        )
        self._initial_gripper_view = torch.empty_like(initial_manifest)
        self._initial_gripper_view.index_copy_(
            0, self._manifest_to_view, initial_manifest
        )
        diagnostic_names = list(self.GRIPPER_STATE_JOINT_SIGNS)
        missing_state_joints = [
            name for name in diagnostic_names if name not in dof_names
        ]
        if missing_state_joints:
            raise RuntimeError(
                "configured gripper state needs missing coupled articulation DoFs "
                f"{missing_state_joints}; available={dof_names}"
            )
        self._gripper_state_joint_indices = torch.tensor(
            [dof_names.index(name) for name in diagnostic_names],
            dtype=torch.int64,
            device=physics_device,
        )
        state_manifest = torch.tensor(
            [
                [
                    self.GRIPPER_POSITIONS_RAD[state][0] * sign
                    for sign in self.GRIPPER_STATE_JOINT_SIGNS.values()
                ]
                for state in states
            ],
            dtype=torch.float32,
            device=physics_device,
        )
        self._initial_gripper_state_view = torch.empty_like(state_manifest)
        self._initial_gripper_state_view.index_copy_(
            0, self._manifest_to_view, state_manifest
        )
        self._gripper_diagnostic_names = diagnostic_names
        self._gripper_diagnostic_indices = self._gripper_state_joint_indices
        self._gripper_report_pending = True
        self._logger.info(
            "Configured initial Grav gripper states "
            + ", ".join(
                f"{robot.public_serial}={state}"
                for robot, state in zip(manifest_robots, states)
            )
        )

    def _report_gripper_positions_once(self) -> None:
        if not self._gripper_report_pending:
            return
        positions_view = self._view.get_joint_positions(
            joint_indices=self._gripper_diagnostic_indices
        )
        positions_manifest = positions_view.index_select(0, self._manifest_to_view)
        positions_deg = torch.rad2deg(positions_manifest).cpu().numpy()
        self._logger.info(
            "Live Grav gripper positions after first rendered physics step: "
            + "; ".join(
                f"{robot.public_serial}="
                + ", ".join(
                    f"{name}:{value:.3f}deg"
                    for name, value in zip(
                        self._gripper_diagnostic_names, positions_deg[row]
                    )
                )
                for row, robot in enumerate(manifest_robots)
            )
        )
        self._gripper_report_pending = False

    def _reset_all_rows(self) -> None:
        velocities = torch.zeros_like(self._initial_view)
        self._view.switch_control_mode(
            "position", indices=self._all_view_rows, joint_indices=self._joint_indices
        )
        self._view.set_joint_positions(
            self._initial_view,
            indices=self._all_view_rows,
            joint_indices=self._joint_indices,
        )
        self._view.set_joint_velocities(
            velocities,
            indices=self._all_view_rows,
            joint_indices=self._joint_indices,
        )
        self._view.set_joint_position_targets(
            self._initial_view,
            indices=self._all_view_rows,
            joint_indices=self._joint_indices,
        )
        if (
            self._gripper_joint_indices is not None
            and self._initial_gripper_view is not None
            and self._gripper_state_joint_indices is not None
            and self._initial_gripper_state_view is not None
        ):
            gripper_velocities = torch.zeros_like(self._initial_gripper_state_view)
            self._view.switch_control_mode(
                "position",
                indices=self._all_view_rows,
                joint_indices=self._gripper_joint_indices,
            )
            self._view.set_joint_positions(
                self._initial_gripper_state_view,
                indices=self._all_view_rows,
                joint_indices=self._gripper_state_joint_indices,
            )
            self._view.set_joint_velocities(
                gripper_velocities,
                indices=self._all_view_rows,
                joint_indices=self._gripper_state_joint_indices,
            )
            self._view.set_joint_position_targets(
                self._initial_gripper_view,
                indices=self._all_view_rows,
                joint_indices=self._gripper_joint_indices,
            )
        self._hold_positions_manifest.copy_(self._initial_manifest)
        for runtime in self._endpoint_runtime:
            runtime.connected = False
            runtime.effort_active = False
            runtime.fresh_since_s = None
            runtime.unsafe_command_active = False
        self._logger.info("Batched CUDA reset placed every controller row in position hold")

    def _reset_correctness_statistics(self) -> None:
        robot_count = len(manifest_robots)
        self._correctness_samples = np.zeros(robot_count, dtype=np.int64)
        self._correctness_effort_samples = np.zeros(robot_count, dtype=np.int64)
        self._correctness_nonfinite_state = np.zeros(robot_count, dtype=np.int64)
        self._correctness_nonfinite_effort = np.zeros(robot_count, dtype=np.int64)
        self._correctness_max_initial_delta = np.zeros(robot_count, dtype=np.float64)
        self._correctness_max_velocity = np.zeros(robot_count, dtype=np.float64)
        self._correctness_max_effort_error = np.zeros(robot_count, dtype=np.float64)
        self._correctness_effort_error_sum_sq = np.zeros(
            robot_count, dtype=np.float64
        )
        self._correctness_effort_value_count = np.zeros(robot_count, dtype=np.int64)
        self._correctness_start_stale = np.asarray(
            [runtime.stale_events for runtime in self._endpoint_runtime],
            dtype=np.int64,
        )
        self._correctness_start_unsafe = np.asarray(
            [runtime.unsafe_command_events for runtime in self._endpoint_runtime],
            dtype=np.int64,
        )

    def _enforce_position_holds(self) -> None:
        """Publish an exact zero-velocity state for rows not owned by fresh effort."""
        manifest_rows = [
            row
            for row, runtime in enumerate(self._endpoint_runtime)
            if not runtime.effort_active
        ]
        if not manifest_rows:
            return
        manifest_indices = torch.tensor(
            manifest_rows, dtype=torch.int64, device=physics_device
        )
        view_indices = self._manifest_to_view.index_select(0, manifest_indices)
        positions = self._hold_positions_manifest.index_select(0, manifest_indices)
        velocities = torch.zeros_like(positions)
        self._view.set_joint_positions(
            positions, indices=view_indices, joint_indices=self._joint_indices
        )
        self._view.set_joint_velocities(
            velocities, indices=view_indices, joint_indices=self._joint_indices
        )
        self._view.set_joint_position_targets(
            positions, indices=view_indices, joint_indices=self._joint_indices
        )

    def _read_gpu_states(self) -> tuple[torch.Tensor, torch.Tensor]:
        q_view = self._view.get_joint_positions(
            joint_indices=self._joint_indices, clone=False
        )
        dq_view = self._view.get_joint_velocities(
            joint_indices=self._joint_indices, clone=False
        )
        if not isinstance(q_view, torch.Tensor) or not isinstance(dq_view, torch.Tensor):
            raise RuntimeError("articulation state backend is not Torch")
        if not q_view.is_cuda or not dq_view.is_cuda:
            raise RuntimeError(
                f"articulation states are not CUDA tensors: "
                f"q={q_view.device}, dq={dq_view.device}"
            )
        return (
            q_view.index_select(0, self._manifest_to_view),
            dq_view.index_select(0, self._manifest_to_view),
        )

    def _drain_state_staging(self) -> None:
        while self._pending_state_slots:
            index = self._pending_state_slots[0]
            slot = self._state_slots[index]
            if not slot.end_event.query():
                break
            self._pending_state_slots.popleft()
            self._publish_staged_state(slot)

    def _publish_staged_state(self, slot: StateStagingSlot) -> None:
        self._d2h_samples_ms.append(
            float(slot.start_event.elapsed_time(slot.end_event))
        )
        dropped = self._transport.publish_states(
            slot.cycle,
            slot.storage[0].numpy(),
            slot.storage[1].numpy(),
        )
        self._native_state_drops += int(dropped)
        slot.busy = False

    def _flush_state_staging(self) -> None:
        """Publish the current state before physics starts.

        The single batched D2H copy is allowed to overlap command snapshot handling,
        then completed here. Controller workers can consequently compute command N
        in parallel with GPU physics step N while command N-1 drives that step.
        """
        while self._pending_state_slots:
            index = self._pending_state_slots.popleft()
            slot = self._state_slots[index]
            slot.end_event.synchronize()
            self._publish_staged_state(slot)

    def _stage_states(
        self, servo_cycle: int, q_manifest: torch.Tensor, dq_manifest: torch.Tensor
    ) -> None:
        free_index = next(
            (index for index, slot in enumerate(self._state_slots) if not slot.busy),
            None,
        )
        if free_index is None:
            self._state_staging_overruns += 1
            return
        slot = self._state_slots[free_index]
        slot.busy = True
        slot.cycle = servo_cycle
        slot.start_event.record()
        slot.storage[0].copy_(q_manifest, non_blocking=True)
        slot.storage[1].copy_(dq_manifest, non_blocking=True)
        slot.end_event.record()
        self._pending_state_slots.append(free_index)

    def _drain_h2d_timing(self) -> None:
        for slot in self._h2d_slots:
            if slot.busy and slot.end_event.query():
                self._h2d_samples_ms.append(
                    float(slot.start_event.elapsed_time(slot.end_event))
                )
                slot.busy = False

    def _copy_commands_to_gpu(self) -> None:
        timing = next((slot for slot in self._h2d_slots if not slot.busy), None)
        if timing is not None:
            timing.busy = True
            timing.start_event.record()
        self._command_gpu.copy_(self._command_cpu, non_blocking=True)
        if timing is not None:
            timing.end_event.record()

    def _validated_controller_processes(
        self, payload: dict
    ) -> tuple[Path, list[dict[str, Any]]]:
        runtime_root = Path(
            _string(payload.get("runtime_root"), "controller ready runtime_root")
        )
        if not runtime_root.is_absolute():
            raise ValueError("controller ready runtime_root must be absolute")
        raw_processes = _list(
            payload.get("processes"), "controller ready processes"
        )
        expected = {
            (robot.public_serial, kind)
            for robot in manifest_robots
            for kind in ("app", "bridge")
        }
        observed: set[tuple[str, str]] = set()
        pids: set[int] = set()
        processes: list[dict[str, Any]] = []
        for index, raw in enumerate(raw_processes):
            entry = _mapping(raw, f"controller ready processes[{index}]")
            pid = _positive_integer(
                entry.get("pid"), f"controller ready processes[{index}].pid"
            )
            kind = _string(
                entry.get("kind"), f"controller ready processes[{index}].kind"
            )
            serial = _string(
                entry.get("serial"), f"controller ready processes[{index}].serial"
            )
            log_path = Path(
                _string(
                    entry.get("log_path"),
                    f"controller ready processes[{index}].log_path",
                )
            )
            if kind not in ("app", "bridge") or (serial, kind) not in expected:
                raise ValueError("controller ready process assignment is invalid")
            if (serial, kind) in observed or pid in pids:
                raise ValueError("controller ready processes contain a duplicate")
            if not log_path.is_absolute() or runtime_root not in log_path.parents:
                raise ValueError("controller ready process log is outside runtime_root")
            observed.add((serial, kind))
            pids.add(pid)
            processes.append(
                {"pid": pid, "kind": kind, "serial": serial, "log_path": log_path}
            )
        signal_state = payload.get("state")
        if signal_state == "STARTING":
            observed_serials = {serial for serial, _kind in observed}
            incomplete_serials = [
                serial
                for serial in observed_serials
                if (serial, "app") not in observed or (serial, "bridge") not in observed
            ]
            if incomplete_serials:
                raise ValueError(
                    "controller starting signal contains incomplete app/bridge pairs"
                )
        elif observed != expected:
            raise ValueError(
                f"controller ready signal describes {len(observed)} processes; "
                f"expected {len(expected)}"
            )
        return runtime_root, processes

    def _refresh_controller_ready(self) -> None:
        if not self._require_controller_ready:
            return
        now = time.monotonic()
        if now - self._last_controller_ready_poll_s < self._controller_ready_poll_s:
            return
        self._last_controller_ready_poll_s = now
        ready = False
        metadata_available = False
        process_metadata: list[dict[str, Any]] = []
        runtime_root: Path | None = None
        launcher_pid = 0
        signal_state: str | None = None
        metadata_changed = False
        try:
            ready_stat = controller_ready_file.stat()
            signature = (ready_stat.st_mtime_ns, ready_stat.st_size)
            if (
                signature == self._controller_ready_signature
                and self._controller_launcher_pid > 0
                and self._controller_process_metadata
            ):
                launcher_pid = self._controller_launcher_pid
                process_metadata = self._controller_process_metadata
                runtime_root = self._controller_runtime_root
                signal_state = self._controller_signal_state
                os.kill(launcher_pid, 0)
                metadata_available = True
                ready = signal_state == "STOPPED_EXTERNAL_AUTO"
            else:
                payload = _mapping(
                    json.loads(controller_ready_file.read_text(encoding="utf-8")),
                    "controller ready signal",
                )
                raw_launcher_pid = payload.get("launcher_pid")
                expected_serials = [robot.public_serial for robot in manifest_robots]
                identity_valid = (
                    payload.get("schema_version") == 1
                    and payload.get("run_id") == manifest_run_id
                    and payload.get("manifest_generated_at") == manifest_generated_at
                    and payload.get("robots") == expected_serials
                    and isinstance(raw_launcher_pid, int)
                    and raw_launcher_pid > 0
                )
                signal_state = payload.get("state")
                metadata_available = identity_valid and signal_state in (
                    "STARTING",
                    "STOPPED_EXTERNAL_AUTO",
                )
                if metadata_available:
                    launcher_pid = raw_launcher_pid
                    runtime_root, process_metadata = (
                        self._validated_controller_processes(payload)
                    )
                    os.kill(launcher_pid, 0)
                    self._controller_ready_signature = signature
                    self._controller_launcher_pid = launcher_pid
                    self._controller_signal_state = signal_state
                    metadata_changed = True
                    ready = signal_state == "STOPPED_EXTERNAL_AUTO"
        except (OSError, UnicodeError, json.JSONDecodeError, ValueError):
            ready = False
            metadata_available = False
        if metadata_available:
            self._controller_process_metadata = process_metadata
            self._controller_runtime_root = runtime_root
            if self._resource_monitor is not None and metadata_changed:
                self._resource_monitor.set_controller_processes(
                    launcher_pid,
                    (entry["pid"] for entry in process_metadata),
                )
        else:
            self._controller_ready_signature = None
            self._controller_launcher_pid = 0
            self._controller_signal_state = None
            self._controller_process_metadata = []
            self._controller_runtime_root = None
            if self._resource_monitor is not None:
                self._resource_monitor.set_controller_processes(0, ())
        if metadata_changed and not ready:
            self._logger.info(
                f"Accepted controller fleet process metadata in state "
                f"[{signal_state}] from [{controller_ready_file}]"
            )
        if ready != self._controller_ready:
            if ready:
                self._logger.info(
                    f"Accepted controller fleet readiness signal [{controller_ready_file}]"
                )
            else:
                self._logger.error(
                    f"Controller fleet readiness signal lost [{controller_ready_file}]"
                )
            self._controller_ready = ready

    def _enter_position_hold(
        self, manifest_rows: list[int], q_manifest: torch.Tensor
    ) -> None:
        if not manifest_rows:
            return
        manifest_indices = torch.tensor(
            manifest_rows, dtype=torch.int64, device=physics_device
        )
        view_indices = self._manifest_to_view.index_select(0, manifest_indices)
        positions = q_manifest.index_select(0, manifest_indices)
        self._hold_positions_manifest.index_copy_(
            0, manifest_indices, positions
        )
        self._view.switch_control_mode(
            "position", indices=view_indices, joint_indices=self._joint_indices
        )
        self._view.set_joint_position_targets(
            positions, indices=view_indices, joint_indices=self._joint_indices
        )

    def _update_control(self, q_manifest: torch.Tensor) -> None:
        self._transport.read_commands(
            self._command_numpy,
            self._command_cycles,
            self._connected,
            self._command_valid,
        )
        now = time.monotonic()
        newly_active: list[int] = []
        enter_hold: list[int] = []
        active_healthy: list[int] = []

        for row, (robot, runtime) in enumerate(
            zip(manifest_robots, self._endpoint_runtime)
        ):
            connected = bool(self._connected[row])
            command_cycle = int(self._command_cycles[row])
            valid = bool(self._command_valid[row]) and command_cycle <= self._servo_cycle
            command_age = self._servo_cycle - command_cycle if valid else None
            absolute_command = np.abs(self._command_numpy[row])
            command_magnitude = float(np.max(absolute_command))
            if self._measurement_active:
                self._measurement_max_command_nm = max(
                    self._measurement_max_command_nm, command_magnitude
                )
            unsafe_joints = np.flatnonzero(
                absolute_command > self._max_abs_command_nm
            )
            command_safe = len(unsafe_joints) == 0
            healthy = (
                connected
                and valid
                and command_safe
                and self._controller_ready
                and command_age is not None
                and command_age <= self._max_command_age_steps
            )

            if connected and valid and not command_safe:
                if not runtime.unsafe_command_active:
                    runtime.unsafe_command_events += 1
                    joint = int(unsafe_joints[0])
                    self._logger.error(
                        f"Rejected unsafe command for [{robot.public_serial}]: "
                        f"joint={joint + 1}, "
                        f"|tau|={absolute_command[joint]:.3f} Nm exceeds "
                        f"{self._max_abs_command_nm[joint]:.3f} Nm"
                    )
                runtime.unsafe_command_active = True
            else:
                runtime.unsafe_command_active = False

            if connected and not runtime.connected:
                self._logger.info(
                    f"Connected endpoint [{robot.public_serial}] env={robot.env_index} "
                    f"role={robot.role}; awaiting fresh asynchronous commands"
                )
                runtime.fresh_since_s = None
            elif not connected and runtime.connected:
                self._logger.error(
                    f"Disconnected endpoint [{robot.public_serial}] env={robot.env_index} "
                    f"role={robot.role}"
                )
                if runtime.effort_active:
                    enter_hold.append(row)
                runtime.effort_active = False
                runtime.fresh_since_s = None

            runtime.connected = connected
            if healthy:
                if runtime.fresh_since_s is None:
                    runtime.fresh_since_s = now
                required_delay = (
                    self._recovery_delay_s
                    if runtime.ever_activated
                    else self._activation_delay_s
                )
                if (
                    not runtime.effort_active
                    and now - runtime.fresh_since_s >= required_delay
                ):
                    runtime.effort_active = True
                    runtime.ever_activated = True
                    newly_active.append(row)
                    self._logger.info(
                        f"Activated effort row for [{robot.public_serial}] after "
                        f"{required_delay:.3f} s of fresh asynchronous exchange"
                    )
                if runtime.effort_active:
                    active_healthy.append(row)
            else:
                runtime.fresh_since_s = None
                if runtime.effort_active:
                    runtime.effort_active = False
                    runtime.stale_events += 1
                    enter_hold.append(row)
                    if not connected:
                        reason = "endpoint disconnected"
                    elif not valid:
                        reason = "no valid command for the current servo timeline"
                    elif not command_safe:
                        reason = (
                            f"unsafe torque magnitude {command_magnitude:.3f} Nm"
                        )
                    elif not self._controller_ready:
                        reason = "controller fleet readiness signal is absent"
                    else:
                        reason = (
                            f"command age {command_age} exceeds "
                            f"{self._max_command_age_steps} cycles"
                        )
                    self._logger.error(
                        f"Command became unusable for [{robot.public_serial}]: "
                        f"cycle={self._servo_cycle}, command_cycle={command_cycle}, "
                        f"reason={reason}; entering position hold"
                    )

        if newly_active:
            rows = torch.tensor(
                newly_active, dtype=torch.int64, device=physics_device
            )
            view_rows = self._manifest_to_view.index_select(0, rows)
            self._view.switch_control_mode(
                "effort", indices=view_rows, joint_indices=self._joint_indices
            )
        self._enter_position_hold(sorted(set(enter_hold)), q_manifest)

        self._copy_commands_to_gpu()
        if active_healthy:
            manifest_rows = torch.tensor(
                active_healthy, dtype=torch.int64, device=physics_device
            )
            view_rows = self._manifest_to_view.index_select(0, manifest_rows)
            efforts = self._command_gpu.index_select(0, manifest_rows)
            self._view.set_joint_efforts(
                efforts, indices=view_rows, joint_indices=self._joint_indices
            )
        self._active_manifest_rows = active_healthy
        if self._measurement_active:
            active_count = len(active_healthy)
            self._minimum_active_rows = min(self._minimum_active_rows, active_count)
            self._inactive_row_cycles += len(manifest_robots) - active_count

    def _sample_correctness(
        self, q_manifest: torch.Tensor, dq_manifest: torch.Tensor
    ) -> None:
        if (
            not self._correctness_enabled
            or not self._measurement_active
            or (self._servo_cycle + 1) % self._correctness_interval != 0
        ):
            return

        robot_count = len(manifest_robots)
        finite_q = torch.isfinite(q_manifest)
        finite_dq = torch.isfinite(dq_manifest)
        nonfinite_state = (~finite_q).sum(dim=1) + (~finite_dq).sum(dim=1)
        initial_delta = torch.where(
            finite_q,
            torch.abs(q_manifest - self._initial_manifest),
            torch.zeros_like(q_manifest),
        ).amax(dim=1)
        maximum_velocity = torch.where(
            finite_dq,
            torch.abs(dq_manifest),
            torch.zeros_like(dq_manifest),
        ).amax(dim=1)

        effort_samples = torch.zeros(
            robot_count, dtype=torch.float32, device=physics_device
        )
        nonfinite_effort = torch.zeros_like(effort_samples)
        maximum_effort_error = torch.zeros_like(effort_samples)
        effort_error_sum_sq = torch.zeros_like(effort_samples)
        effort_value_count = torch.zeros_like(effort_samples)
        if self._active_manifest_rows:
            manifest_rows = torch.tensor(
                self._active_manifest_rows,
                dtype=torch.int64,
                device=physics_device,
            )
            view_rows = self._manifest_to_view.index_select(0, manifest_rows)
            applied = self._view.get_applied_joint_efforts(
                indices=view_rows, joint_indices=self._joint_indices
            )
            if not isinstance(applied, torch.Tensor) or not applied.is_cuda:
                raise RuntimeError(
                    "applied effort readback is not a CUDA Torch tensor"
                )
            expected = self._command_gpu.index_select(0, manifest_rows)
            finite_effort = torch.isfinite(applied) & torch.isfinite(expected)
            safe_error = torch.where(
                finite_effort, torch.abs(applied - expected), torch.zeros_like(applied)
            )
            effort_samples.index_fill_(0, manifest_rows, 1.0)
            nonfinite_effort.index_copy_(
                0, manifest_rows, (~finite_effort).sum(dim=1).to(torch.float32)
            )
            maximum_effort_error.index_copy_(
                0, manifest_rows, safe_error.amax(dim=1)
            )
            effort_error_sum_sq.index_copy_(
                0, manifest_rows, torch.square(safe_error).sum(dim=1)
            )
            effort_value_count.index_copy_(
                0, manifest_rows, finite_effort.sum(dim=1).to(torch.float32)
            )

        packed = torch.stack(
            (
                nonfinite_state.to(torch.float32),
                initial_delta,
                maximum_velocity,
                effort_samples,
                nonfinite_effort,
                maximum_effort_error,
                effort_error_sum_sq,
                effort_value_count,
            ),
            dim=1,
        ).cpu().numpy()
        self._correctness_samples += 1
        self._correctness_nonfinite_state += packed[:, 0].astype(np.int64)
        np.maximum(
            self._correctness_max_initial_delta,
            packed[:, 1],
            out=self._correctness_max_initial_delta,
        )
        np.maximum(
            self._correctness_max_velocity,
            packed[:, 2],
            out=self._correctness_max_velocity,
        )
        self._correctness_effort_samples += packed[:, 3].astype(np.int64)
        self._correctness_nonfinite_effort += packed[:, 4].astype(np.int64)
        np.maximum(
            self._correctness_max_effort_error,
            packed[:, 5],
            out=self._correctness_max_effort_error,
        )
        self._correctness_effort_error_sum_sq += packed[:, 6]
        self._correctness_effort_value_count += packed[:, 7].astype(np.int64)

    def _sample_torque_trace(
        self, q_manifest: torch.Tensor, dq_manifest: torch.Tensor
    ) -> None:
        if not self._torque_trace_enabled or not self._measurement_active:
            return

        q_cpu = q_manifest.detach().cpu().numpy()
        dq_cpu = dq_manifest.detach().cpu().numpy()
        torque_cpu = np.asarray(self._command_numpy, dtype=np.float32).copy()
        torque_bits = torque_cpu.view(np.uint32)
        measurement_step = self._servo_cycle - self._measurement_start_cycle
        for row, (robot, runtime) in enumerate(
            zip(manifest_robots, self._endpoint_runtime)
        ):
            command_cycle = int(self._command_cycles[row])
            command_valid = bool(self._command_valid[row])
            result: dict[str, Any] = {
                "schema_version": 1,
                "run_id": manifest_run_id,
                "manifest_generated_at": manifest_generated_at,
                "config": str(config_path),
                "trace_label": self._torque_trace_label,
                "measurement_step": measurement_step,
                "servo_cycle": self._servo_cycle,
                "slot": robot.slot,
                "env_index": robot.env_index,
                "role": robot.role,
                "serial": robot.public_serial,
                "command_cycle": command_cycle,
                "command_age_steps": (
                    self._servo_cycle - command_cycle if command_valid else -1
                ),
                "connected": bool(self._connected[row]),
                "command_valid": command_valid,
                "effort_active": runtime.effort_active,
            }
            for joint in range(self.ROBOT_DOF):
                number = joint + 1
                result[f"q_j{number}_rad"] = float(q_cpu[row, joint])
                result[f"dq_j{number}_rad_s"] = float(dq_cpu[row, joint])
                result[f"tau_j{number}_nm"] = float(torque_cpu[row, joint])
                result[f"tau_j{number}_bits"] = int(torque_bits[row, joint])
            self._torque_trace_rows.append(result)

    def _write_torque_trace(self) -> None:
        if not self._torque_trace_enabled:
            return
        if not self._torque_trace_rows or self._torque_trace_csv is None:
            raise RuntimeError("enabled torque trace did not collect any samples")
        append_csv_rows(
            self._torque_trace_csv,
            tuple(self._torque_trace_rows[0]),
            self._torque_trace_rows,
        )
        self._logger.info(
            f"Torque trace rows appended to [{self._torque_trace_csv}]"
        )

    def _percentiles(self, values: list[float]) -> tuple[float, float, float]:
        if not values:
            return 0.0, 0.0, 0.0
        array = np.asarray(values, dtype=float)
        return tuple(float(value) for value in np.percentile(array, [50, 95, 99]))

    def _transport_summary(self) -> dict[str, float | int]:
        statistics = self._transport.statistics()
        turnaround = [
            float(value)
            for endpoint in statistics
            for value in endpoint["recent_command_turnaround_us"]
        ]
        p50, p95, p99 = self._percentiles(turnaround)
        latest_completed_cycle = max(0, self._servo_cycle - 1)
        command_ages = [
            latest_completed_cycle - int(cycle)
            for cycle, valid in zip(self._command_cycles, self._command_valid)
            if valid and int(cycle) <= latest_completed_cycle
        ]
        return {
            "connected": sum(bool(endpoint["connected"]) for endpoint in statistics),
            "worker_errors": sum(bool(endpoint["worker_error"]) for endpoint in statistics),
            "commands_received": sum(
                int(endpoint["commands_received"]) for endpoint in statistics
            ),
            "command_packets_received": sum(
                int(endpoint["command_packets_received"]) for endpoint in statistics
            ),
            "synchronization_events": sum(
                int(endpoint["synchronization_events"]) for endpoint in statistics
            ),
            "synchronization_command_discards": sum(
                int(endpoint["synchronization_command_discards"])
                for endpoint in statistics
            ),
            "recovery_events": sum(
                int(endpoint["recovery_events"]) for endpoint in statistics
            ),
            "wait_timeouts": sum(
                int(endpoint["command_wait_timeouts"]) for endpoint in statistics
            ),
            "queue_overflows": sum(
                int(endpoint["state_queue_overflows"]) for endpoint in statistics
            ),
            "state_frames_superseded": sum(
                int(endpoint["state_frames_superseded"]) for endpoint in statistics
            ),
            "max_consecutive_command_wait_timeouts": max(
                (
                    int(endpoint["maximum_consecutive_command_wait_timeouts"])
                    for endpoint in statistics
                ),
                default=0,
            ),
            "waiting_for_command_endpoints": sum(
                endpoint["phase"] == "waiting_for_command" for endpoint in statistics
            ),
            "max_publish_send_lag": max(
                (
                    max(
                        0,
                        int(endpoint["last_published_cycle"])
                        - int(endpoint["last_sent_cycle"]),
                    )
                    for endpoint in statistics
                ),
                default=0,
            ),
            "queue_high_watermark": max(
                (int(endpoint["queue_high_watermark"]) for endpoint in statistics),
                default=0,
            ),
            "max_command_age": max(command_ages, default=0),
            "turnaround_p50_us": p50,
            "turnaround_p95_us": p95,
            "turnaround_p99_us": p99,
            "turnaround_max_us": max(
                (
                    float(endpoint["maximum_command_turnaround_us"])
                    for endpoint in statistics
                ),
                default=0.0,
            ),
        }

    def _report_health(self) -> None:
        if self._servo_cycle < self._next_health_cycle:
            return
        summary = self._transport_summary()
        active = sum(runtime.effort_active for runtime in self._endpoint_runtime)
        maximum_initial_drift = float(
            torch.max(torch.abs(self._last_q_manifest - self._initial_manifest)).item()
        )
        maximum_velocity = float(torch.max(torch.abs(self._last_dq_manifest)).item())
        maximum_command = float(np.max(np.abs(self._command_numpy)))
        self._logger.info(
            "Async fleet health: "
            f"connected={summary['connected']}/{len(manifest_robots)}, "
            f"controller_ready={self._controller_ready}, "
            f"effort_active={active}/{len(manifest_robots)}, "
            f"max_command_age={summary['max_command_age']} cycles, "
            f"queue_overflows={summary['queue_overflows']}, "
            f"state_frames_superseded={summary['state_frames_superseded']}, "
            f"command_sync_events={summary['synchronization_events']}, "
            f"command_sync_discards={summary['synchronization_command_discards']}, "
            f"command_recoveries={summary['recovery_events']}, "
            f"max_publish_send_lag={summary['max_publish_send_lag']} cycles, "
            f"waiting_for_command={summary['waiting_for_command_endpoints']}, "
            f"max_consecutive_command_timeouts="
            f"{summary['max_consecutive_command_wait_timeouts']}, "
            f"queue_high_watermark={summary['queue_high_watermark']}, "
            f"state_staging_overruns={self._state_staging_overruns}, "
            f"command_deadline_misses={self._command_deadline_misses}, "
            f"max_initial_drift={maximum_initial_drift:.6f} rad, "
            f"max_velocity={maximum_velocity:.6f} rad/s, "
            f"max_command={maximum_command:.3f} Nm, "
            f"turnaround_us[p50/p95/p99/max]="
            f"{summary['turnaround_p50_us']:.1f}/"
            f"{summary['turnaround_p95_us']:.1f}/"
            f"{summary['turnaround_p99_us']:.1f}/"
            f"{summary['turnaround_max_us']:.1f}"
        )
        self._next_health_cycle += self._health_interval

    def _report_performance(self) -> None:
        if self._servo_cycle < self._next_performance_cycle:
            return
        now = time.perf_counter()
        cycles = self._servo_cycle - self._performance_start_cycle
        elapsed = now - self._performance_start_time
        effective_hz = cycles / elapsed
        d2h = self._d2h_samples_ms[-self._performance_interval :]
        h2d = self._h2d_samples_ms[-self._performance_interval :]
        deadline = self._command_deadline_samples_ms[-self._performance_interval :]
        self._logger.info(
            f"GPU async performance: physics={effective_hz:.1f} Hz, "
            f"real-time factor={effective_hz / self._physics_frequency:.3f}, "
            f"environment-steps/s={effective_hz * manifest_num_envs:.1f}, "
            f"D2H mean={np.mean(d2h) if d2h else 0.0:.4f} ms, "
            f"H2D mean={np.mean(h2d) if h2d else 0.0:.4f} ms, "
            f"command deadline wait mean="
            f"{np.mean(deadline) if deadline else 0.0:.4f} ms"
        )
        self._performance_start_cycle = self._servo_cycle
        self._performance_start_time = now
        self._next_performance_cycle += self._performance_interval

    def step(self, diagnostics: bool = True) -> None:
        self._drain_state_staging()
        self._drain_h2d_timing()
        self._enforce_position_holds()
        q_manifest, dq_manifest = self._read_gpu_states()
        self._last_q_manifest = q_manifest
        self._last_dq_manifest = dq_manifest
        self._stage_states(self._servo_cycle, q_manifest, dq_manifest)
        self._flush_state_staging()
        deadline_start = time.perf_counter()
        ready_count = self._transport.wait_for_command_cycle(
            self._servo_cycle, self._same_cycle_command_deadline_us
        )
        self._command_deadline_samples_ms.append(
            (time.perf_counter() - deadline_start) * 1000.0
        )
        self._refresh_controller_ready()
        self._update_control(q_manifest)
        self._sample_torque_trace(q_manifest, dq_manifest)
        connected_count = int(np.count_nonzero(self._connected))
        if connected_count and ready_count < connected_count:
            self._command_deadline_misses += 1
        render_this_cycle = bool(
            self._render
            and (self._servo_cycle + 1) % self._render_interval == 0
        )
        self._world.step(render=False, update_fabric=render_this_cycle)
        self._sample_correctness(q_manifest, dq_manifest)
        self._servo_cycle += 1
        self._write_simulation_clock()
        if render_this_cycle:
            self._world.render()
            self._write_camera_frames()
            self._report_gripper_positions_once()
        if diagnostics:
            self._report_health()
            self._report_performance()

    def _benchmark_config(self) -> dict:
        raw = config.get("benchmark", {})
        return _mapping(raw, "benchmark")

    def _all_effort_active(self) -> bool:
        return all(runtime.effort_active for runtime in self._endpoint_runtime)

    def _controller_log_counts(self) -> dict[str, dict[str, int]]:
        counts = {
            robot.public_serial: {"fault": 0, "torque_limit": 0, "error": 0}
            for robot in manifest_robots
        }
        patterns = {
            "fault": re.compile(
                r"fault state|transit system state[^\n]*\bFAULT\b|entered?[^\n]*\bFAULT\b",
                re.IGNORECASE,
            ),
            "torque_limit": re.compile(
                r"current torque cmd|torque command[^\n]*limit|torque[^\n]*safety check",
                re.IGNORECASE,
            ),
            "error": re.compile(r"\[(?:error|critical)\]", re.IGNORECASE),
        }
        for entry in self._controller_process_metadata:
            if entry["kind"] != "app":
                continue
            try:
                contents = entry["log_path"].read_text(
                    encoding="utf-8", errors="replace"
                )
            except OSError:
                counts[entry["serial"]]["error"] += 1
                continue
            for label, pattern in patterns.items():
                counts[entry["serial"]][label] += len(pattern.findall(contents))
        return counts

    def _correctness_summary(
        self, log_counts: dict[str, dict[str, int]]
    ) -> dict[str, float | int | bool]:
        effort_count = int(np.sum(self._correctness_effort_value_count))
        effort_sum_sq = float(np.sum(self._correctness_effort_error_sum_sq))
        stale_delta = np.asarray(
            [runtime.stale_events for runtime in self._endpoint_runtime],
            dtype=np.int64,
        ) - self._correctness_start_stale
        unsafe_delta = np.asarray(
            [runtime.unsafe_command_events for runtime in self._endpoint_runtime],
            dtype=np.int64,
        ) - self._correctness_start_unsafe
        process_alive = 0
        for entry in self._controller_process_metadata:
            try:
                os.kill(entry["pid"], 0)
                process_alive += 1
            except OSError:
                pass
        fault_entries = sum(item["fault"] for item in log_counts.values())
        torque_entries = sum(item["torque_limit"] for item in log_counts.values())
        error_entries = sum(item["error"] for item in log_counts.values())
        maximum_effort_error = float(
            np.max(self._correctness_max_effort_error, initial=0.0)
        )
        summary: dict[str, float | int | bool] = {
            "correctness_samples_min": int(
                np.min(self._correctness_samples)
            ),
            "effort_readback_samples_min": int(
                np.min(self._correctness_effort_samples)
            ),
            "nonfinite_state_values": int(
                np.sum(self._correctness_nonfinite_state)
            ),
            "nonfinite_effort_values": int(
                np.sum(self._correctness_nonfinite_effort)
            ),
            "max_initial_pose_delta_rad": float(
                np.max(self._correctness_max_initial_delta, initial=0.0)
            ),
            "max_velocity_rad_s": float(
                np.max(self._correctness_max_velocity, initial=0.0)
            ),
            "max_effort_target_error_nm": maximum_effort_error,
            "rms_effort_target_error_nm": (
                math.sqrt(effort_sum_sq / effort_count) if effort_count else 0.0
            ),
            "minimum_effort_active_rows": self._minimum_active_rows,
            "inactive_effort_row_cycles": self._inactive_row_cycles,
            "stale_command_events": int(np.sum(stale_delta)),
            "unsafe_command_events": int(np.sum(unsafe_delta)),
            "controller_processes_expected": len(self._controller_process_metadata),
            "controller_processes_alive_final": process_alive,
            "controller_fault_log_entries": fault_entries,
            "controller_torque_limit_log_entries": torque_entries,
            "controller_error_log_entries": error_entries,
        }
        summary["correctness_pass"] = bool(
            summary["correctness_samples_min"] > 0
            and summary["effort_readback_samples_min"] > 0
            and summary["nonfinite_state_values"] == 0
            and summary["nonfinite_effort_values"] == 0
            and maximum_effort_error <= self._effort_error_tolerance_nm
            and self._minimum_active_rows == len(manifest_robots)
            and self._inactive_row_cycles == 0
            and summary["stale_command_events"] == 0
            and summary["unsafe_command_events"] == 0
            and process_alive == len(self._controller_process_metadata)
            and fault_entries == 0
            and torque_entries == 0
            and error_entries == 0
        )
        return summary

    def _write_correctness_rows(
        self, log_counts: dict[str, dict[str, int]]
    ) -> None:
        stale_delta = np.asarray(
            [runtime.stale_events for runtime in self._endpoint_runtime],
            dtype=np.int64,
        ) - self._correctness_start_stale
        unsafe_delta = np.asarray(
            [runtime.unsafe_command_events for runtime in self._endpoint_runtime],
            dtype=np.int64,
        ) - self._correctness_start_unsafe
        rows: list[dict[str, Any]] = []
        for row, robot in enumerate(manifest_robots):
            effort_count = int(self._correctness_effort_value_count[row])
            effort_rms = (
                math.sqrt(
                    self._correctness_effort_error_sum_sq[row] / effort_count
                )
                if effort_count
                else 0.0
            )
            logs = log_counts[robot.public_serial]
            row_pass = bool(
                self._correctness_samples[row] > 0
                and self._correctness_effort_samples[row] > 0
                and self._correctness_nonfinite_state[row] == 0
                and self._correctness_nonfinite_effort[row] == 0
                and self._correctness_max_effort_error[row]
                <= self._effort_error_tolerance_nm
                and stale_delta[row] == 0
                and unsafe_delta[row] == 0
                and logs["fault"] == 0
                and logs["torque_limit"] == 0
                and logs["error"] == 0
            )
            rows.append(
                {
                    "schema_version": 1,
                    "run_id": manifest_run_id,
                    "manifest_generated_at": manifest_generated_at,
                    "config": str(config_path),
                    "manifest": str(manifest_path),
                    "num_envs": manifest_num_envs,
                    "robot_count": len(manifest_robots),
                    "slot": robot.slot,
                    "env_index": robot.env_index,
                    "role": robot.role,
                    "serial": robot.public_serial,
                    "state_samples": int(self._correctness_samples[row]),
                    "effort_readback_samples": int(
                        self._correctness_effort_samples[row]
                    ),
                    "nonfinite_state_values": int(
                        self._correctness_nonfinite_state[row]
                    ),
                    "nonfinite_effort_values": int(
                        self._correctness_nonfinite_effort[row]
                    ),
                    "max_initial_pose_delta_rad": self._correctness_max_initial_delta[
                        row
                    ],
                    "max_velocity_rad_s": self._correctness_max_velocity[row],
                    "max_effort_target_error_nm": self._correctness_max_effort_error[
                        row
                    ],
                    "rms_effort_target_error_nm": effort_rms,
                    "stale_command_events": int(stale_delta[row]),
                    "unsafe_command_events": int(unsafe_delta[row]),
                    "controller_fault_log_entries": logs["fault"],
                    "controller_torque_limit_log_entries": logs["torque_limit"],
                    "controller_error_log_entries": logs["error"],
                    "correctness_pass": row_pass,
                }
            )
        append_csv_rows(self._correctness_csv, tuple(rows[0]), rows)
        self._logger.info(
            f"Correctness rows appended to [{self._correctness_csv}]"
        )

    def _append_benchmark_csv(self, result: dict[str, Any], path: Path) -> None:
        append_csv_rows(path, tuple(result), [result])
        self._logger.info(f"Benchmark row appended to [{path}]")

    def _run_benchmark(self, benchmark: dict) -> None:
        warmup = _positive_integer(
            benchmark.get("warmup_steps"), "benchmark.warmup_steps"
        )
        measured = _positive_integer(
            benchmark.get("measured_steps"), "benchmark.measured_steps"
        )
        require_active = _boolean(
            benchmark.get("require_all_effort_active"),
            "benchmark.require_all_effort_active",
        )
        if self._resource_monitor is not None:
            self._resource_monitor.set_context("controller_wait", self._servo_cycle)
        if require_active:
            timeout_s = _positive_float(
                benchmark.get("controller_ready_timeout_s"),
                "benchmark.controller_ready_timeout_s",
            )
            deadline = time.monotonic() + timeout_s
            while not self._all_effort_active():
                if not simulation_app.is_running():
                    raise RuntimeError("simulation stopped while awaiting controllers")
                if time.monotonic() >= deadline:
                    raise TimeoutError("not all controller rows became effort-active")
                self.step()

        if self._resource_monitor is not None:
            self._resource_monitor.set_context("warmup", self._servo_cycle)
        for _ in range(warmup):
            if not simulation_app.is_running():
                raise RuntimeError("simulation stopped during benchmark warmup")
            self.step(diagnostics=False)

        start_summary = self._transport_summary()
        start_staging_overruns = self._state_staging_overruns
        start_native_drops = self._native_state_drops
        start_deadline_misses = self._command_deadline_misses
        start_deadline_sample = len(self._command_deadline_samples_ms)
        start_d2h_sample = len(self._d2h_samples_ms)
        start_h2d_sample = len(self._h2d_samples_ms)
        self._reset_correctness_statistics()
        self._minimum_active_rows = len(manifest_robots)
        self._inactive_row_cycles = 0
        self._measurement_max_command_nm = 0.0
        self._torque_trace_rows.clear()
        self._measurement_start_cycle = self._servo_cycle
        self._measurement_active = True
        if self._resource_monitor is not None:
            self._resource_monitor.begin_phase("measurement", self._servo_cycle)
        start_cycle = self._servo_cycle
        start_time = time.perf_counter()
        for _ in range(measured):
            if not simulation_app.is_running():
                raise RuntimeError("simulation stopped during benchmark")
            self.step(diagnostics=False)
        elapsed = time.perf_counter() - start_time
        if self._resource_monitor is not None:
            self._resource_monitor.set_context("measurement", self._servo_cycle)
            self._resource_monitor.sample_now()
        self._measurement_active = False
        actual_steps = self._servo_cycle - start_cycle
        end_summary = self._transport_summary()
        effective_hz = actual_steps / elapsed
        log_counts = self._controller_log_counts()
        correctness = self._correctness_summary(log_counts)
        resource_samples = (
            self._resource_monitor.samples("measurement")
            if self._resource_monitor is not None
            else []
        )
        resources = summarize_resource_samples(resource_samples)
        monitor_errors = (
            len(self._resource_monitor.errors)
            if self._resource_monitor is not None
            else 0
        )
        resources["resource_monitor_errors"] = monitor_errors
        resources["resource_logging_pass"] = bool(
            resources["resource_samples"] >= 2
            and resources["gpu_resource_samples"] >= 2
            and resources["controller_processes_expected_max"] > 0
            and resources["controller_processes_alive_min"]
            == resources["controller_processes_expected_max"]
            and monitor_errors == 0
        )
        if self._resource_monitor is not None:
            self._resource_monitor.set_context("complete", self._servo_cycle)
        result = {
            "schema_version": 3,
            "bridge_version": APP_VERSION,
            "run_id": manifest_run_id,
            "manifest_generated_at": manifest_generated_at,
            "config": str(config_path),
            "manifest": str(manifest_path),
            "device": physics_device,
            "num_envs": manifest_num_envs,
            "robot_count": len(manifest_robots),
            "physics_frequency_hz": self._physics_frequency,
            "render": self._render,
            "scene_graph_instancing": scene_graph_instancing_enabled,
            "controllers_required": require_active,
            "controller_ready": self._controller_ready,
            "warmup_steps": warmup,
            "measured_steps": actual_steps,
            "elapsed_s": elapsed,
            "effective_physics_hz": effective_hz,
            "real_time_factor": effective_hz / self._physics_frequency,
            "environment_steps_per_s": effective_hz * manifest_num_envs,
            "state_staging_overruns": (
                self._state_staging_overruns - start_staging_overruns
            ),
            "native_state_drops": self._native_state_drops - start_native_drops,
            "command_deadline_us": self._same_cycle_command_deadline_us,
            "command_deadline_misses": (
                self._command_deadline_misses - start_deadline_misses
            ),
            "queue_overflows": (
                int(end_summary["queue_overflows"])
                - int(start_summary["queue_overflows"])
            ),
            "state_frames_superseded": (
                int(end_summary["state_frames_superseded"])
                - int(start_summary["state_frames_superseded"])
            ),
            "max_command_age": end_summary["max_command_age"],
            "turnaround_p50_us": end_summary["turnaround_p50_us"],
            "turnaround_p95_us": end_summary["turnaround_p95_us"],
            "turnaround_p99_us": end_summary["turnaround_p99_us"],
            "turnaround_max_us": end_summary["turnaround_max_us"],
            "max_initial_drift_rad": correctness["max_initial_pose_delta_rad"],
            "max_velocity_rad_s": correctness["max_velocity_rad_s"],
            "max_command_nm": self._measurement_max_command_nm,
            "d2h_mean_ms": float(np.mean(self._d2h_samples_ms[start_d2h_sample:]))
            if len(self._d2h_samples_ms) > start_d2h_sample
            else 0.0,
            "h2d_mean_ms": float(np.mean(self._h2d_samples_ms[start_h2d_sample:]))
            if len(self._h2d_samples_ms) > start_h2d_sample
            else 0.0,
            "command_deadline_wait_mean_ms": float(
                np.mean(self._command_deadline_samples_ms[start_deadline_sample:])
            )
            if len(self._command_deadline_samples_ms) > start_deadline_sample
            else 0.0,
            **correctness,
            **resources,
        }
        self._logger.info(
            "GPU_ASYNC_BENCHMARK_RESULT "
            + " ".join(f"{key}={value}" for key, value in result.items())
        )
        output = Path(_string(benchmark.get("result_csv"), "benchmark.result_csv"))
        self._write_torque_trace()
        if self._correctness_enabled:
            self._write_correctness_rows(log_counts)
        self._append_benchmark_csv(result, output.expanduser().resolve())
        if self._require_resource_samples and not resources["resource_logging_pass"]:
            raise RuntimeError("resource logging did not produce a complete valid window")
        if self._require_correctness_pass and not correctness["correctness_pass"]:
            raise RuntimeError("controller/state correctness checks failed")

    def run(self) -> None:
        benchmark = self._benchmark_config()
        enabled = _boolean(benchmark.get("enabled", False), "benchmark.enabled")
        if enabled:
            self._run_benchmark(benchmark)
            return
        while simulation_app.is_running():
            self.step()

    def cleanup(self) -> None:
        self._clock_header = None
        clock_shm = getattr(self, "_clock_shm", None)
        if clock_shm is not None:
            clock_shm.close()
            try:
                clock_shm.unlink()
            except FileNotFoundError:
                pass
            self._clock_shm = None
        for camera_shm in getattr(self, "_camera_shms", []):
            camera_shm.header = None
            camera_shm.rgb = None
            camera_shm.shm.close()
            try:
                camera_shm.shm.unlink()
            except FileNotFoundError:
                pass
        self._camera_shms = []
        monitor = getattr(self, "_resource_monitor", None)
        if monitor is not None:
            monitor.set_context("shutdown", self._servo_cycle)
            monitor.stop()
        transport = getattr(self, "_transport", None)
        if transport is not None:
            transport.stop()
        if torch.cuda.is_available():
            torch.cuda.synchronize()


def main() -> None:
    bridge = None
    try:
        bridge = GpuAsyncFleetBridge()
        bridge.run()
    except Exception:
        traceback.print_exc()
        raise
    finally:
        if bridge is not None:
            bridge.cleanup()
        simulation_app.close()


if __name__ == "__main__":
    main()
