#!/usr/bin/env python3
"""RAMMS Newton bridge worker.

This process speaks newline-delimited JSON over stdin/stdout so the Unreal
plugin can drive the selected Python/Warp-based Newton checkout without relying
on UE's editor-only embedded Python runtime.
"""

from __future__ import annotations

import json
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


class BridgeWorld:
    def __init__(self, world_id: int, create_desc: dict[str, Any]) -> None:
        self.world_id = world_id
        self.create_desc = create_desc
        self.bodies: "OrderedDict[int, BodyState]" = OrderedDict()
        self.next_body_id = 1
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

        for body in self.bodies.values():
            world_xform = _transform_to_newton(body.transform)
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
            "sim_time_seconds": self.sim_time_seconds,
        }
        if body_transforms:
            result["body_transforms"] = body_transforms
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
