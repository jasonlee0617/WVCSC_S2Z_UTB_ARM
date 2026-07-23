"""Schema and validation helpers for the real five-point spray route.

This route is deliberately separate from ``site_mission.py``.  The latter is
the shared manual-target mission format used by simulation and the Nav2 Qt
tools; changing it would make the real demonstration route an unnecessary
breaking change for those users.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Mapping

import yaml

from .site_mission import (
    DEFAULT_ARM_BASE_FORWARD_OFFSET,
    DEFAULT_ARM_BASE_LEFT_OFFSET,
    MAX_CAPTURE_POSITION_SPREAD_M,
    MAX_CAPTURE_POSITION_STDDEV_M,
    MAX_CAPTURE_YAW_SPREAD_RAD,
    MAX_CAPTURE_YAW_STDDEV_RAD,
    MAX_ARM_BASE_FORWARD_ERROR_M,
    MAX_TREE_DISTANCE_M,
    MIN_TREE_DISTANCE_M,
    footprint_is_free,
    load_map_grid,
    map_hashes,
)


FIELD_ROUTE_SCHEMA_VERSION = 4
ROUTE_POINT_IDS = ("point_1", "point_2", "point_3", "point_4", "point_5")
ROUTE_ROLES = ("wide_start", "inspect", "inspect", "wide_stop", "finish")
INSPECT_POINT_IDS = frozenset(("point_2", "point_3"))
ARM_SPRAY_DURATION_SEC = 3.0


@dataclass(frozen=True)
class FieldRouteStep:
    point_id: str
    role: str
    navigation_pose: dict[str, float]
    tree_id: str | None = None
    tree_offset_arm_base_m: tuple[float, float] | None = None
    tree_base_z_m: float | None = None
    arm_spray_duration: float | None = None


def _finite_number(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be a finite number")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{field} must be a finite number")
    return value


def _pose(raw: Any, field: str) -> dict[str, float]:
    if not isinstance(raw, Mapping):
        raise ValueError(f"{field} must be a mapping")
    pose = {
        key: _finite_number(raw.get(key), f"{field}.{key}")
        for key in ("x", "y", "yaw")
    }
    return pose


def _capture_quality(raw: Any, field: str) -> dict[str, float]:
    if not isinstance(raw, Mapping):
        raise ValueError(f"{field} is required")
    samples = raw.get("samples")
    if isinstance(samples, bool) or not isinstance(samples, int) or samples < 30:
        raise ValueError(f"{field}.samples must be an integer of at least 30")
    quality = {
        "samples": samples,
        **{
            key: _finite_number(raw.get(key), f"{field}.{key}")
            for key in (
                "position_spread_m", "yaw_spread_rad",
                "max_position_stddev_m", "max_yaw_stddev_rad")
        },
    }
    if quality["position_spread_m"] > MAX_CAPTURE_POSITION_SPREAD_M:
        raise ValueError(f"{field}.position_spread_m exceeds capture limit")
    if quality["yaw_spread_rad"] > MAX_CAPTURE_YAW_SPREAD_RAD:
        raise ValueError(f"{field}.yaw_spread_rad exceeds capture limit")
    if quality["max_position_stddev_m"] > MAX_CAPTURE_POSITION_STDDEV_M:
        raise ValueError(f"{field}.max_position_stddev_m exceeds capture limit")
    if quality["max_yaw_stddev_rad"] > MAX_CAPTURE_YAW_STDDEV_RAD:
        raise ValueError(f"{field}.max_yaw_stddev_rad exceeds capture limit")
    return quality


def new_field_route_document(site_id: str, mission_id: str, map_yaml: str) -> dict[str, Any]:
    if not isinstance(site_id, str) or not site_id.strip():
        raise ValueError("site_id must be a non-empty string")
    if not isinstance(mission_id, str) or not mission_id.strip():
        raise ValueError("mission_id must be a non-empty string")

    hashes = map_hashes(map_yaml)
    return {
        "schema_version": FIELD_ROUTE_SCHEMA_VERSION,
        "site_id": site_id.strip(),
        "map": {
            "frame_id": hashes["frame_id"],
            "yaml_sha256": hashes["yaml_sha256"],
            "image_sha256": hashes["image_sha256"],
        },
        "arm_base_mount": {
            "x_m": DEFAULT_ARM_BASE_FORWARD_OFFSET,
            "y_m": DEFAULT_ARM_BASE_LEFT_OFFSET,
        },
        "mission": {
            "mission_id": mission_id.strip(),
            "route_steps": [
                {"point_id": point_id, "role": role, "navigation_pose": None}
                for point_id, role in zip(ROUTE_POINT_IDS, ROUTE_ROLES)
            ],
        },
    }


def load_field_route_document(path: str) -> dict[str, Any]:
    source = Path(path).expanduser().resolve()
    if not source.is_file():
        raise ValueError(f"field route file does not exist: {source}")
    try:
        document = yaml.safe_load(source.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ValueError(f"invalid YAML in {source}: {exc}") from exc
    if not isinstance(document, dict):
        raise ValueError("field route root must be a mapping")
    if document.get("schema_version") != FIELD_ROUTE_SCHEMA_VERSION:
        raise ValueError(
            f"field route schema_version must be {FIELD_ROUTE_SCHEMA_VERSION}; "
            "this is not a five-point real-field route"
        )
    return document


def route_steps(document: Mapping[str, Any]) -> tuple[FieldRouteStep, ...]:
    mission = document.get("mission")
    if not isinstance(mission, Mapping):
        raise ValueError("mission must be a mapping")
    raw_steps = mission.get("route_steps")
    if not isinstance(raw_steps, list) or len(raw_steps) != len(ROUTE_ROLES):
        raise ValueError("mission.route_steps must contain exactly five ordered steps")

    parsed: list[FieldRouteStep] = []
    for index, (raw, expected_id, expected_role) in enumerate(
        zip(raw_steps, ROUTE_POINT_IDS, ROUTE_ROLES), start=1
    ):
        field = f"mission.route_steps[{index - 1}]"
        if not isinstance(raw, Mapping):
            raise ValueError(f"{field} must be a mapping")
        if raw.get("point_id") != expected_id:
            raise ValueError(f"{field}.point_id must be {expected_id}")
        if raw.get("role") != expected_role:
            raise ValueError(f"{field}.role must be {expected_role}")
        pose = _pose(raw.get("navigation_pose"), f"{field}.navigation_pose")

        if expected_id not in INSPECT_POINT_IDS:
            parsed.append(FieldRouteStep(expected_id, expected_role, pose))
            continue

        tree_id = raw.get("tree_id")
        if not isinstance(tree_id, str) or not tree_id.strip():
            raise ValueError(f"{field}.tree_id must be a non-empty string")
        offset = raw.get("tree_offset_arm_base_m")
        if not isinstance(offset, (list, tuple)) or len(offset) != 2:
            raise ValueError(f"{field}.tree_offset_arm_base_m must be [x, y]")
        tree_x = _finite_number(offset[0], f"{field}.tree_offset_arm_base_m[0]")
        tree_y = _finite_number(offset[1], f"{field}.tree_offset_arm_base_m[1]")
        if abs(tree_x) > MAX_ARM_BASE_FORWARD_ERROR_M:
            raise ValueError(f"{field}.tree_offset_arm_base_m[0] exceeds arm-base forward limit")
        distance = math.hypot(tree_x, tree_y)
        if not MIN_TREE_DISTANCE_M <= distance <= MAX_TREE_DISTANCE_M:
            raise ValueError(f"{field}.tree_offset_arm_base_m is outside supported tree distance")
        tree_z = _finite_number(raw.get("tree_base_z_m"), f"{field}.tree_base_z_m")
        duration = _finite_number(raw.get("arm_spray_duration"), f"{field}.arm_spray_duration")
        if not math.isclose(duration, ARM_SPRAY_DURATION_SEC, abs_tol=1e-9):
            raise ValueError(f"{field}.arm_spray_duration must be {ARM_SPRAY_DURATION_SEC}")
        parsed.append(
            FieldRouteStep(
                expected_id,
                expected_role,
                pose,
                tree_id.strip(),
                (tree_x, tree_y),
                tree_z,
                duration,
            )
        )
    return tuple(parsed)


def validate_field_route_document(
    document: Mapping[str, Any],
    map_yaml: str,
    *,
    require_capture_quality: bool = True,
    require_free_space: bool = True,
) -> tuple[FieldRouteStep, ...]:
    if document.get("schema_version") != FIELD_ROUTE_SCHEMA_VERSION:
        raise ValueError(f"schema_version must be {FIELD_ROUTE_SCHEMA_VERSION}")
    if not isinstance(document.get("site_id"), str) or not document["site_id"].strip():
        raise ValueError("site_id must be a non-empty string")
    mission = document.get("mission")
    if not isinstance(mission, Mapping) or not isinstance(mission.get("mission_id"), str) or not mission["mission_id"].strip():
        raise ValueError("mission.mission_id must be a non-empty string")

    route_map = document.get("map")
    if not isinstance(route_map, Mapping):
        raise ValueError("map must be a mapping")
    expected_hashes = map_hashes(map_yaml)
    if (route_map.get("frame_id") != expected_hashes["frame_id"]
            or route_map.get("yaml_sha256") != expected_hashes["yaml_sha256"]
            or route_map.get("image_sha256") != expected_hashes["image_sha256"]):
        raise ValueError("field route map binding does not match the selected map")

    mount = document.get("arm_base_mount")
    if not isinstance(mount, Mapping):
        raise ValueError("arm_base_mount must be a mapping")
    arm_x = _finite_number(mount.get("x_m"), "arm_base_mount.x_m")
    arm_y = _finite_number(mount.get("y_m"), "arm_base_mount.y_m")
    if (not math.isclose(arm_x, DEFAULT_ARM_BASE_FORWARD_OFFSET, abs_tol=1e-9)
            or not math.isclose(arm_y, DEFAULT_ARM_BASE_LEFT_OFFSET, abs_tol=1e-9)):
        raise ValueError("arm_base_mount does not match robot geometry")

    steps = route_steps(document)
    raw_steps = mission["route_steps"]
    seen_tree_ids: set[str] = set()
    grid = load_map_grid(map_yaml) if require_free_space else None
    for index, (step, raw) in enumerate(zip(steps, raw_steps), start=1):
        if require_capture_quality:
            _capture_quality(raw.get("capture_quality"), f"mission.route_steps[{index - 1}].capture_quality")
        if grid is not None and not footprint_is_free(
                grid, (step.navigation_pose["x"], step.navigation_pose["y"],
                       step.navigation_pose["yaw"])):
            raise ValueError(f"route step {step.point_id} is not in free map space")
        if step.tree_id:
            if step.tree_id in seen_tree_ids:
                raise ValueError(f"duplicate inspect tree_id: {step.tree_id}")
            seen_tree_ids.add(step.tree_id)
    return steps
