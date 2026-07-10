#!/usr/bin/env python3
"""RAMMS Newton bridge worker.

This process speaks newline-delimited JSON over stdin/stdout so the Unreal
plugin can drive the selected Python/Warp-based Newton checkout without relying
on UE's editor-only embedded Python runtime.
"""

from __future__ import annotations

import json
import math
import sys
from collections import OrderedDict
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


def _plugin_root() -> Path:
    return Path(__file__).resolve().parent.parent


def _configure_import_paths() -> None:
    newton_root = _plugin_root() / "ThirdParty" / "newton"
    if newton_root.exists():
        sys.path.insert(0, str(newton_root))


_configure_import_paths()

try:
    import newton  # type: ignore
    import warp as wp  # type: ignore

    NEWTON_IMPORT_ERROR: str | None = None
except Exception as exc:  # noqa: BLE001
    newton = None  # type: ignore[assignment]
    wp = None  # type: ignore[assignment]
    NEWTON_IMPORT_ERROR = f"{type(exc).__name__}: {exc}"


def _vec_cm_to_m(vec: dict[str, Any]) -> tuple[float, float, float]:
    return (
        float(vec.get("x", 0.0)) * 0.01,
        float(vec.get("y", 0.0)) * 0.01,
        float(vec.get("z", 0.0)) * 0.01,
    )


def _transform_to_newton(transform: dict[str, Any]) -> Any:
    translation = _vec_cm_to_m(transform.get("translation_cm", {}))
    rotation = transform.get("rotation", {})
    return wp.transform(
        p=wp.vec3(*translation),
        q=wp.quat(
            float(rotation.get("x", 0.0)),
            float(rotation.get("y", 0.0)),
            float(rotation.get("z", 0.0)),
            float(rotation.get("w", 1.0)),
        ),
    )


def _identity_transform() -> Any:
    return wp.transform(wp.vec3(0.0, 0.0, 0.0), wp.quat_identity())


def _default_scale() -> dict[str, float]:
    return {"x": 1.0, "y": 1.0, "z": 1.0}


def _export_transform_to_body_transform(transform: dict[str, Any]) -> dict[str, Any]:
    translation = list(transform.get("translation_m", [0.0, 0.0, 0.0]))
    rotation = list(transform.get("rotation_xyzw", [0.0, 0.0, 0.0, 1.0]))
    scale = list(transform.get("scale", [1.0, 1.0, 1.0]))
    return {
        "translation_cm": {
            "x": float(translation[0] if len(translation) > 0 else 0.0) * 100.0,
            "y": float(translation[1] if len(translation) > 1 else 0.0) * 100.0,
            "z": float(translation[2] if len(translation) > 2 else 0.0) * 100.0,
        },
        "rotation": {
            "x": float(rotation[0] if len(rotation) > 0 else 0.0),
            "y": float(rotation[1] if len(rotation) > 1 else 0.0),
            "z": float(rotation[2] if len(rotation) > 2 else 0.0),
            "w": float(rotation[3] if len(rotation) > 3 else 1.0),
        },
        "scale3d": {
            "x": float(scale[0] if len(scale) > 0 else 1.0),
            "y": float(scale[1] if len(scale) > 1 else 1.0),
            "z": float(scale[2] if len(scale) > 2 else 1.0),
        },
    }


def _export_transform_to_newton(transform: dict[str, Any] | None) -> Any:
    if not transform:
        return _identity_transform()

    translation = list(transform.get("translation_m", [0.0, 0.0, 0.0]))
    rotation = list(transform.get("rotation_xyzw", [0.0, 0.0, 0.0, 1.0]))
    return wp.transform(
        p=wp.vec3(
            float(translation[0] if len(translation) > 0 else 0.0),
            float(translation[1] if len(translation) > 1 else 0.0),
            float(translation[2] if len(translation) > 2 else 0.0),
        ),
        q=wp.quat(
            float(rotation[0] if len(rotation) > 0 else 0.0),
            float(rotation[1] if len(rotation) > 1 else 0.0),
            float(rotation[2] if len(rotation) > 2 else 0.0),
            float(rotation[3] if len(rotation) > 3 else 1.0),
        ),
    )


def _axis_from_values(values: list[Any] | tuple[Any, ...] | None) -> Any:
    if not values:
        return wp.vec3(1.0, 0.0, 0.0)

    x = float(values[0] if len(values) > 0 else 1.0)
    y = float(values[1] if len(values) > 1 else 0.0)
    z = float(values[2] if len(values) > 2 else 0.0)
    length = math.sqrt((x * x) + (y * y) + (z * z))
    if length <= 1.0e-6:
        return wp.vec3(1.0, 0.0, 0.0)
    return wp.vec3(x / length, y / length, z / length)


def _revolute_degrees_to_radians(value: float) -> float:
    return math.radians(float(value))


@dataclass
class BodyState:
    body_id: int
    name: str
    owner_name: str
    mass_kg: float
    kinematic: bool
    transform: dict[str, Any]
    shape: dict[str, Any]
    material: dict[str, Any]
    model_index: int | None = None
    newton_mesh: Any | None = None


@dataclass
class JointState:
    joint_id: int
    name: str
    parent_body_id: int
    child_body_id: int
    joint_type: str
    drive_mode: str
    axis: tuple[float, float, float]
    use_limits: bool
    min_limit_degrees: float
    max_limit_degrees: float
    max_effort: float
    target_angle_degrees: float
    target_velocity_degrees_per_second: float
    position_gain: float
    damping_gain: float
    feedforward_effort: float
    parent_anchor_transform: dict[str, Any]
    child_anchor_transform: dict[str, Any]
    model_joint_index: int | None = None
    dof_index: int | None = None
    coord_index: int | None = None


class BridgeWorld:
    def __init__(self, world_id: int, create_desc: dict[str, Any]) -> None:
        self.world_id = world_id
        self.create_desc = create_desc
        self.bodies: "OrderedDict[int, BodyState]" = OrderedDict()
        self.next_body_id = 1
        self.joints: "OrderedDict[int, JointState]" = OrderedDict()
        self.next_joint_id = 1
        self.scene_dirty = True
        self.sim_time_seconds = 0.0
        self.model = None
        self.solver = None
        self.collision_pipeline = None
        self.state_0 = None
        self.state_1 = None
        self.control = None
        self.contacts = None

    @property
    def newton_available(self) -> bool:
        return newton is not None and wp is not None

    def _make_shape_cfg(self, material: dict[str, Any]) -> Any:
        cfg = newton.ModelBuilder.ShapeConfig()
        cfg.density = float(material.get("density", 1000.0))
        cfg.ke = float(material.get("ke", 1.0e4))
        cfg.kd = float(material.get("kd", 1.0e3))
        cfg.kf = float(material.get("kf", 0.0))
        cfg.ka = float(material.get("ka", 0.0))
        cfg.mu = float(material.get("mu", 0.5))
        cfg.restitution = float(material.get("restitution", 0.0))
        cfg.mu_torsional = float(material.get("mu_torsional", 0.005))
        cfg.mu_rolling = float(material.get("mu_rolling", 0.0001))
        cfg.margin = float(material.get("margin_cm", 0.0)) * 0.01
        cfg.is_solid = bool(material.get("is_solid", True))
        cfg.collision_group = max(int(material.get("collision_group", 1)), 0)
        return cfg

    def _get_or_create_mesh(self, body: BodyState) -> Any | None:
        if body.newton_mesh is not None:
            return body.newton_mesh

        vertex_count = int(body.shape.get("vertex_count", 0))
        vertices_cm = body.shape.get("vertices_cm", [])
        indices = body.shape.get("indices", [])
        if vertex_count <= 0 or len(vertices_cm) != vertex_count * 3 or len(indices) < 3:
            return None

        vertices_m = np.array(vertices_cm, dtype=np.float32).reshape(vertex_count, 3) * 0.01
        triangle_indices = np.array(indices, dtype=np.int32)
        body.newton_mesh = newton.Mesh(vertices=vertices_m, indices=triangle_indices, compute_inertia=False)
        return body.newton_mesh

    def _add_shape(self, builder: Any, body_index: int, body: BodyState, xform: Any | None = None) -> None:
        shape = body.shape
        shape_type = str(shape.get("type", "box")).lower()
        shape_xform = xform if xform is not None else _identity_transform()
        shape_cfg = self._make_shape_cfg(body.material)

        if shape_type == "mesh":
            mesh = self._get_or_create_mesh(body)
            if mesh is not None:
                builder.add_shape_mesh(body_index, xform=shape_xform, mesh=mesh, cfg=shape_cfg)
                return

        if shape_type == "convex_hull":
            mesh = self._get_or_create_mesh(body)
            if mesh is not None:
                builder.add_shape_convex_hull(body_index, xform=shape_xform, mesh=mesh, cfg=shape_cfg)
                return

        if shape_type == "sphere":
            radius_m = max(float(shape.get("radius_cm", 10.0)) * 0.01, 0.001)
            builder.add_shape_sphere(body_index, xform=shape_xform, radius=radius_m, cfg=shape_cfg)
            return

        if shape_type == "capsule":
            radius_m = max(float(shape.get("radius_cm", 10.0)) * 0.01, 0.001)
            half_height_m = max(float(shape.get("half_height_cm", 20.0)) * 0.01, radius_m)
            builder.add_shape_capsule(body_index, xform=shape_xform, radius=radius_m, half_height=half_height_m, cfg=shape_cfg)
            return

        half_extents = shape.get("half_extents_cm", {})
        hx = max(float(half_extents.get("x", 10.0)) * 0.01, 0.001)
        hy = max(float(half_extents.get("y", 10.0)) * 0.01, 0.001)
        hz = max(float(half_extents.get("z", 10.0)) * 0.01, 0.001)
        builder.add_shape_box(body_index, xform=shape_xform, hx=hx, hy=hy, hz=hz, cfg=shape_cfg)

    def _joint_target_mode(self, joint: JointState) -> Any:
        drive_mode = joint.drive_mode.lower()
        if drive_mode == "position_control":
            return (
                newton.JointTargetMode.POSITION_VELOCITY
                if joint.position_gain > 0.0 and joint.damping_gain > 0.0
                else newton.JointTargetMode.POSITION
            )
        if drive_mode == "velocity_control":
            return newton.JointTargetMode.VELOCITY
        if drive_mode == "torque_control":
            return newton.JointTargetMode.EFFORT
        return newton.JointTargetMode.NONE

    def _apply_joint_control_to_runtime(self, joint: JointState) -> None:
        if self.control is None or joint.dof_index is None:
            return

        drive_mode = joint.drive_mode.lower()
        dof_index = joint.dof_index

        if self.control.joint_target_pos is not None:
            target_pos = 0.0
            if drive_mode in ("position_control", "velocity_control"):
                target_pos = _revolute_degrees_to_radians(joint.target_angle_degrees) if joint.joint_type == "revolute" else float(joint.target_angle_degrees)
            self.control.joint_target_pos.numpy()[dof_index] = target_pos

        if self.control.joint_target_vel is not None:
            target_vel = 0.0
            if drive_mode in ("position_control", "velocity_control"):
                target_vel = (
                    _revolute_degrees_to_radians(joint.target_velocity_degrees_per_second)
                    if joint.joint_type == "revolute"
                    else float(joint.target_velocity_degrees_per_second)
                )
            self.control.joint_target_vel.numpy()[dof_index] = target_vel

        if self.control.joint_act is not None:
            self.control.joint_act.numpy()[dof_index] = float(joint.feedforward_effort)

        if self.control.joint_f is not None:
            self.control.joint_f.numpy()[dof_index] = float(joint.feedforward_effort if drive_mode == "torque_control" else 0.0)

    def _apply_all_joint_controls_to_runtime(self) -> None:
        if self.control is None:
            return

        for joint in self.joints.values():
            self._apply_joint_control_to_runtime(joint)

    def _sync_joint_states_from_state(self) -> list[dict[str, Any]]:
        if self.state_0 is None or self.state_0.joint_q is None or self.state_0.joint_qd is None:
            return []

        joint_q = self.state_0.joint_q.numpy()
        joint_qd = self.state_0.joint_qd.numpy()
        result: list[dict[str, Any]] = []
        for joint in self.joints.values():
            position = 0.0
            velocity = 0.0
            if joint.coord_index is not None and joint.coord_index < len(joint_q):
                position = float(joint_q[joint.coord_index])
            if joint.dof_index is not None and joint.dof_index < len(joint_qd):
                velocity = float(joint_qd[joint.dof_index])

            if joint.joint_type == "revolute":
                position = math.degrees(position)
                velocity = math.degrees(velocity)

            result.append(
                {
                    "joint_id": joint.joint_id,
                    "name": joint.name,
                    "position": position,
                    "velocity": velocity,
                }
            )
        return result

    def _rebuild_scene_if_needed(self) -> None:
        if not self.newton_available or not self.scene_dirty:
            return

        if not self.bodies:
            self.model = None
            self.solver = None
            self.collision_pipeline = None
            self.state_0 = None
            self.state_1 = None
            self.control = None
            self.contacts = None
            self.scene_dirty = False
            return

        if self.model is not None and self.state_0 is not None and self.state_0.body_q is not None:
            self._sync_transforms_from_state(list(self.bodies))

        gravity = self.create_desc.get("gravity_cm_per_second_squared", {})
        gravity_m = _vec_cm_to_m(gravity)

        builder = newton.ModelBuilder(
            up_axis=newton.Axis.Z,
            gravity=float(gravity_m[2]),
        )

        articulated_body_ids: set[int] = set()
        articulated_child_body_ids: set[int] = set()
        for joint in self.joints.values():
            if joint.parent_body_id > 0:
                articulated_body_ids.add(joint.parent_body_id)
            if joint.child_body_id > 0:
                articulated_body_ids.add(joint.child_body_id)
                articulated_child_body_ids.add(joint.child_body_id)

        for body in self.bodies.values():
            world_xform = _transform_to_newton(body.transform)
            if body.body_id in articulated_body_ids:
                body_index = builder.add_link(
                    xform=world_xform,
                    mass=0.0 if body.kinematic else max(float(body.mass_kg), 0.001),
                    label=body.name or f"body_{body.body_id}",
                    is_kinematic=bool(body.kinematic and body.body_id not in articulated_child_body_ids),
                )
                body.model_index = int(body_index)
                self._add_shape(builder, int(body_index), body)
                continue

            if body.kinematic:
                body.model_index = None
                self._add_shape(builder, -1, body, xform=world_xform)
                continue

            body_index = builder.add_body(
                xform=world_xform,
                mass=0.0 if body.kinematic else max(float(body.mass_kg), 0.001),
                label=body.name or f"body_{body.body_id}",
            )
            body.model_index = int(body_index)
            self._add_shape(builder, int(body_index), body)

        graph_neighbors: dict[int, set[int]] = {}
        for joint in self.joints.values():
            child_body = self.bodies.get(joint.child_body_id)
            if child_body is None or child_body.model_index is None:
                joint.model_joint_index = None
                joint.dof_index = None
                joint.coord_index = None
                continue

            parent_index = -1
            if joint.parent_body_id > 0:
                parent_body = self.bodies.get(joint.parent_body_id)
                if parent_body is None or parent_body.model_index is None:
                    joint.model_joint_index = None
                    joint.dof_index = None
                    joint.coord_index = None
                    continue
                parent_index = int(parent_body.model_index)

            joint_kwargs: dict[str, Any] = {
                "parent": parent_index,
                "child": int(child_body.model_index),
                "parent_xform": _export_transform_to_newton(joint.parent_anchor_transform),
                "child_xform": _export_transform_to_newton(joint.child_anchor_transform),
                "axis": _axis_from_values(joint.axis),
                "label": joint.name,
                "effort_limit": max(float(joint.max_effort), 0.0),
                "target_ke": max(float(joint.position_gain), 0.0),
                "target_kd": max(float(joint.damping_gain), 0.0),
                "actuator_mode": self._joint_target_mode(joint),
            }
            if joint.use_limits:
                joint_kwargs["limit_lower"] = _revolute_degrees_to_radians(joint.min_limit_degrees) if joint.joint_type == "revolute" else float(joint.min_limit_degrees)
                joint_kwargs["limit_upper"] = _revolute_degrees_to_radians(joint.max_limit_degrees) if joint.joint_type == "revolute" else float(joint.max_limit_degrees)

            if joint.joint_type == "fixed":
                joint_index = builder.add_joint_fixed(
                    parent=parent_index,
                    child=int(child_body.model_index),
                    parent_xform=_export_transform_to_newton(joint.parent_anchor_transform),
                    child_xform=_export_transform_to_newton(joint.child_anchor_transform),
                    label=joint.name,
                )
            elif joint.joint_type == "prismatic":
                joint_index = builder.add_joint_prismatic(**joint_kwargs)
            elif joint.joint_type == "spherical":
                joint_index = builder.add_joint_ball(
                    parent=parent_index,
                    child=int(child_body.model_index),
                    parent_xform=_export_transform_to_newton(joint.parent_anchor_transform),
                    child_xform=_export_transform_to_newton(joint.child_anchor_transform),
                    label=joint.name,
                    actuator_mode=newton.JointTargetMode.NONE,
                )
            else:
                joint_index = builder.add_joint_revolute(**joint_kwargs)

            joint.model_joint_index = int(joint_index)
            graph_neighbors.setdefault(joint.child_body_id, set())
            if joint.parent_body_id > 0:
                graph_neighbors.setdefault(joint.parent_body_id, set()).add(joint.child_body_id)
                graph_neighbors[joint.child_body_id].add(joint.parent_body_id)

        visited_bodies: set[int] = set()
        for root_body_id in graph_neighbors:
            if root_body_id in visited_bodies:
                continue

            stack = [root_body_id]
            component_body_ids: set[int] = set()
            while stack:
                body_id = stack.pop()
                if body_id in visited_bodies:
                    continue
                visited_bodies.add(body_id)
                component_body_ids.add(body_id)
                stack.extend(graph_neighbors.get(body_id, ()))

            joint_indices = [
                int(joint.model_joint_index)
                for joint in self.joints.values()
                if joint.model_joint_index is not None
                and joint.child_body_id in component_body_ids
            ]
            if joint_indices:
                builder.add_articulation(joint_indices)

        self.model = builder.finalize()
        self.collision_pipeline = newton.CollisionPipeline(
            self.model,
            broad_phase="explicit",
        )
        self.solver = newton.solvers.SolverXPBD(
            self.model,
            iterations=8,
            rigid_contact_relaxation=0.8,
            enable_restitution=True,
        )
        self.state_0 = self.model.state()
        self.state_1 = self.model.state()
        self.control = self.model.control()
        self.contacts = self.collision_pipeline.contacts()
        for joint in self.joints.values():
            if joint.model_joint_index is None:
                joint.dof_index = None
                joint.coord_index = None
                continue

            joint_index = int(joint.model_joint_index)
            joint.coord_index = int(builder.joint_q_start[joint_index]) if joint_index < len(builder.joint_q_start) else None
            next_dof_start = builder.joint_dof_count
            if joint_index + 1 < len(builder.joint_qd_start):
                next_dof_start = int(builder.joint_qd_start[joint_index + 1])
            joint.dof_index = int(builder.joint_qd_start[joint_index]) if next_dof_start > int(builder.joint_qd_start[joint_index]) else None

        self._apply_all_joint_controls_to_runtime()
        self.scene_dirty = False

    def _sync_body_transform_from_pose(self, body: BodyState, pose: Any) -> None:
        body.transform = {
            "translation_cm": {
                "x": float(pose[0]) * 100.0,
                "y": float(pose[1]) * 100.0,
                "z": float(pose[2]) * 100.0,
            },
            "rotation": {
                "x": float(pose[3]),
                "y": float(pose[4]),
                "z": float(pose[5]),
                "w": float(pose[6]),
            },
            "scale3d": body.transform.get("scale3d", _default_scale()),
        }

    def _sync_transforms_from_state(self, body_ids: list[int] | None = None) -> None:
        if self.state_0 is None or self.state_0.body_q is None:
            return

        body_q = self.state_0.body_q.numpy()
        requested_body_ids = body_ids if body_ids is not None else list(self.bodies)
        for body_id in requested_body_ids:
            body = self.bodies.get(int(body_id))
            if body is None or body.model_index is None or body.model_index >= len(body_q):
                continue

            self._sync_body_transform_from_pose(body, body_q[body.model_index])

    def create_joint(self, params: dict[str, Any]) -> int:
        joint_id = self.next_joint_id
        self.next_joint_id += 1

        axis_values = params.get("axis", [1.0, 0.0, 0.0])
        self.joints[joint_id] = JointState(
            joint_id=joint_id,
            name=str(params.get("joint_name", f"joint_{joint_id}")),
            parent_body_id=int(params.get("parent_body_id", 0)),
            child_body_id=int(params.get("child_body_id", 0)),
            joint_type=str(params.get("joint_type", "revolute")).lower(),
            drive_mode=str(params.get("drive_mode", "passive")).lower(),
            axis=(
                float(axis_values[0] if len(axis_values) > 0 else 1.0),
                float(axis_values[1] if len(axis_values) > 1 else 0.0),
                float(axis_values[2] if len(axis_values) > 2 else 0.0),
            ),
            use_limits=bool(params.get("use_limits", False)),
            min_limit_degrees=float(params.get("min_limit_degrees", -180.0)),
            max_limit_degrees=float(params.get("max_limit_degrees", 180.0)),
            max_effort=float(params.get("max_effort", 0.0)),
            target_angle_degrees=float(params.get("target_angle_degrees", 0.0)),
            target_velocity_degrees_per_second=float(params.get("target_velocity_degrees_per_second", 0.0)),
            position_gain=float(params.get("position_gain", 0.0)),
            damping_gain=float(params.get("damping_gain", 0.0)),
            feedforward_effort=float(params.get("feedforward_effort", 0.0)),
            parent_anchor_transform=dict(params.get("parent_anchor_transform", {})),
            child_anchor_transform=dict(params.get("child_anchor_transform", {})),
        )
        self.scene_dirty = True
        return joint_id

    def destroy_joint(self, joint_id: int) -> None:
        if joint_id in self.joints:
            del self.joints[joint_id]
            self.scene_dirty = True

    def set_joint_controls(self, joint_controls: list[dict[str, Any]]) -> int:
        updated_count = 0
        joints_by_name = {joint.name: joint for joint in self.joints.values()}
        for joint_control in joint_controls:
            joint_name = str(joint_control.get("name", ""))
            joint = joints_by_name.get(joint_name)
            if joint is None:
                continue

            structural_change = False
            if "drive_mode" in joint_control:
                new_drive_mode = str(joint_control.get("drive_mode", joint.drive_mode)).lower()
                structural_change = structural_change or new_drive_mode != joint.drive_mode
                joint.drive_mode = new_drive_mode
            if "target_angle_degrees" in joint_control:
                joint.target_angle_degrees = float(joint_control.get("target_angle_degrees", joint.target_angle_degrees))
            if "target_velocity_degrees_per_second" in joint_control:
                joint.target_velocity_degrees_per_second = float(
                    joint_control.get("target_velocity_degrees_per_second", joint.target_velocity_degrees_per_second)
                )
            if "position_gain" in joint_control:
                new_position_gain = float(joint_control.get("position_gain", joint.position_gain))
                structural_change = structural_change or not math.isclose(new_position_gain, joint.position_gain, rel_tol=1.0e-6, abs_tol=1.0e-6)
                joint.position_gain = new_position_gain
            if "damping_gain" in joint_control:
                new_damping_gain = float(joint_control.get("damping_gain", joint.damping_gain))
                structural_change = structural_change or not math.isclose(new_damping_gain, joint.damping_gain, rel_tol=1.0e-6, abs_tol=1.0e-6)
                joint.damping_gain = new_damping_gain
            if "feedforward_effort" in joint_control:
                joint.feedforward_effort = float(joint_control.get("feedforward_effort", joint.feedforward_effort))
            if "max_effort" in joint_control:
                new_max_effort = float(joint_control.get("max_effort", joint.max_effort))
                structural_change = structural_change or not math.isclose(new_max_effort, joint.max_effort, rel_tol=1.0e-6, abs_tol=1.0e-6)
                joint.max_effort = new_max_effort

            updated_count += 1
            if structural_change:
                self.scene_dirty = True
            elif not self.scene_dirty:
                self._apply_joint_control_to_runtime(joint)

        return updated_count

    def create_body(self, params: dict[str, Any]) -> int:
        body_id = self.next_body_id
        self.next_body_id += 1
        self.bodies[body_id] = BodyState(
            body_id=body_id,
            name=str(params.get("name", "")),
            owner_name=str(params.get("owner_name", "")),
            mass_kg=float(params.get("mass_kg", 1.0)),
            kinematic=bool(params.get("kinematic", False)),
            transform=dict(params.get("transform", {})),
            shape=dict(params.get("shape", {})),
            material=dict(params.get("material", {})),
        )
        self.scene_dirty = True
        return body_id

    def destroy_body(self, body_id: int) -> None:
        if body_id in self.bodies:
            del self.bodies[body_id]
            self.scene_dirty = True

    def set_body_transform(self, body_id: int, transform: dict[str, Any]) -> bool:
        body = self.bodies.get(body_id)
        if body is None:
            return False
        body.transform = dict(transform)
        self.scene_dirty = True
        return True

    def get_body_transform(self, body_id: int) -> dict[str, Any] | None:
        body = self.bodies.get(body_id)
        if body is not None and body.model_index is not None:
            self._sync_transforms_from_state([body_id])
        return None if body is None else body.transform

    def step(self, fixed_step_seconds: float, body_ids_to_sync: list[int] | None = None) -> dict[str, Any]:
        self._rebuild_scene_if_needed()

        if not self.newton_available:
            return {
                "simulated": False,
                "reason": NEWTON_IMPORT_ERROR or "Newton Python package is unavailable.",
                "body_count": len(self.bodies),
            }

        if self.model is None or self.solver is None or self.state_0 is None or self.state_1 is None:
            return {
                "simulated": False,
                "reason": "No Newton bodies are currently registered.",
                "body_count": len(self.bodies),
            }

        self.state_0.clear_forces()
        self.contacts = self.model.collide(self.state_0, collision_pipeline=self.collision_pipeline)
        self.solver.step(self.state_0, self.state_1, self.control, self.contacts, float(fixed_step_seconds))
        self.state_0, self.state_1 = self.state_1, self.state_0
        self.sim_time_seconds += float(fixed_step_seconds)

        body_transforms: list[dict[str, Any]] = []
        if body_ids_to_sync:
            self._sync_transforms_from_state(body_ids_to_sync)
            for body_id in body_ids_to_sync:
                body = self.bodies.get(int(body_id))
                if body is None:
                    continue
                body_transforms.append(
                    {
                        "body_id": int(body_id),
                        "transform": body.transform,
                    }
                )

        result = {
            "simulated": True,
            "body_count": len(self.bodies),
            "joint_count": len(self.joints),
            "sim_time_seconds": self.sim_time_seconds,
        }
        if body_transforms:
            result["body_transforms"] = body_transforms
        joint_states = self._sync_joint_states_from_state()
        if joint_states:
            result["joint_states"] = joint_states
        return result


class BridgeServer:
    def __init__(self) -> None:
        self.next_world_id = 1
        self.worlds: dict[int, BridgeWorld] = {}

    def handle(self, method: str, params: dict[str, Any]) -> dict[str, Any]:
        if method == "ping":
            return {
                "backend": "newton-python-worker",
                "newton_available": newton is not None,
                "newton_version": getattr(newton, "__version__", ""),
                "import_error": NEWTON_IMPORT_ERROR or "",
            }

        if method == "shutdown":
            return {"shutdown": True}

        if method == "create_world":
            world_id = self.next_world_id
            self.next_world_id += 1
            self.worlds[world_id] = BridgeWorld(world_id, params)
            return {"world_id": world_id}

        world_id = int(params.get("world_id", 0))
        world = self.worlds.get(world_id)
        if world is None:
            raise RuntimeError(f"Unknown world_id {world_id}")

        if method == "destroy_world":
            del self.worlds[world_id]
            return {"destroyed": True}
        if method == "create_body":
            return {"body_id": world.create_body(params)}
        if method == "destroy_body":
            world.destroy_body(int(params.get("body_id", 0)))
            return {"destroyed": True}
        if method == "create_joint":
            return {"joint_id": world.create_joint(params)}
        if method == "destroy_joint":
            world.destroy_joint(int(params.get("joint_id", 0)))
            return {"destroyed": True}
        if method == "set_joint_controls":
            joint_controls = [dict(value) for value in params.get("joints", [])]
            return {"updated": world.set_joint_controls(joint_controls)}
        if method == "set_body_transform":
            return {"updated": world.set_body_transform(int(params.get("body_id", 0)), dict(params.get("transform", {})))}
        if method == "get_body_transform":
            transform = world.get_body_transform(int(params.get("body_id", 0)))
            if transform is None:
                raise RuntimeError(f"Unknown body_id {params.get('body_id', 0)}")
            return {"transform": transform}
        if method == "step_world":
            return world.step(
                float(params.get("fixed_step_seconds", 0.0)),
                [int(body_id) for body_id in params.get("body_ids_to_sync", [])],
            )

        raise RuntimeError(f"Unsupported method '{method}'")


def _write_response(payload: dict[str, Any]) -> None:
    sys.stdout.write(json.dumps(payload, separators=(",", ":")) + "\n")
    sys.stdout.flush()


def main() -> int:
    server = BridgeServer()

    for raw_line in sys.stdin:
        line = raw_line.strip()
        if not line:
            continue

        request_id: int | None = None
        shutdown_requested = False
        try:
            message = json.loads(line)
            request_id = int(message.get("id", 0))
            method = str(message.get("method", ""))
            params = dict(message.get("params", {}))
            result = server.handle(method, params)
            shutdown_requested = bool(result.get("shutdown", False))
            _write_response({"id": request_id, "ok": True, "result": result})
        except Exception as exc:  # noqa: BLE001
            _write_response(
                {
                    "id": request_id or 0,
                    "ok": False,
                    "error": f"{type(exc).__name__}: {exc}",
                }
            )

        if shutdown_requested:
            break

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
