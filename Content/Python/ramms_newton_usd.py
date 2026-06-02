from __future__ import annotations

import argparse
import json
import math
import pathlib
import re
from typing import Any

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

import newton_usd_schemas  # noqa: F401


_SAFE_NAME_RE = re.compile(r"[^A-Za-z0-9_]+")


def _safe_name(value: str | None, fallback: str) -> str:
    text = (value or "").strip()
    if not text:
        text = fallback
    text = _SAFE_NAME_RE.sub("_", text).strip("_")
    if not text:
        text = fallback
    if text[0].isdigit():
        text = f"_{text}"
    return text


def _vec3(values: list[float] | tuple[float, float, float] | None, default: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> Gf.Vec3f:
    source = values if values is not None else default
    return Gf.Vec3f(float(source[0]), float(source[1]), float(source[2]))


def _quatf_xyzw(values: list[float] | tuple[float, float, float, float] | None) -> Gf.Quatf:
    if not values:
        return Gf.Quatf(1.0, 0.0, 0.0, 0.0)
    return Gf.Quatf(float(values[3]), float(values[0]), float(values[1]), float(values[2]))


def _apply_transform(prim: Usd.Prim, transform_data: dict[str, Any] | None) -> None:
    if not transform_data:
        return

    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(_vec3(transform_data.get("translation_m")))
    xformable.AddOrientOp().Set(_quatf_xyzw(transform_data.get("rotation_xyzw")))
    xformable.AddScaleOp().Set(_vec3(transform_data.get("scale"), (1.0, 1.0, 1.0)))


def _set_custom_attr(prim: Usd.Prim, name: str, value: Any) -> None:
    if value is None:
        return
    if isinstance(value, bool):
        prim.CreateAttribute(name, Sdf.ValueTypeNames.Bool, custom=True).Set(value)
        return
    if isinstance(value, int) and not isinstance(value, bool):
        prim.CreateAttribute(name, Sdf.ValueTypeNames.Int, custom=True).Set(value)
        return
    if isinstance(value, float):
        prim.CreateAttribute(name, Sdf.ValueTypeNames.Double, custom=True).Set(value)
        return
    if isinstance(value, str):
        prim.CreateAttribute(name, Sdf.ValueTypeNames.String, custom=True).Set(value)
        return
    if isinstance(value, list):
        if not value:
            prim.CreateAttribute(name, Sdf.ValueTypeNames.StringArray, custom=True).Set([])
            return
        first = value[0]
        if isinstance(first, str):
            prim.CreateAttribute(name, Sdf.ValueTypeNames.StringArray, custom=True).Set(value)
            return
        if isinstance(first, bool):
            prim.CreateAttribute(name, Sdf.ValueTypeNames.BoolArray, custom=True).Set(value)
            return
        if isinstance(first, int):
            prim.CreateAttribute(name, Sdf.ValueTypeNames.IntArray, custom=True).Set(value)
            return
        prim.CreateAttribute(name, Sdf.ValueTypeNames.DoubleArray, custom=True).Set([float(item) for item in value])


def _axis_token(axis: list[float] | None):
    if not axis:
        return UsdPhysics.Tokens.x
    magnitudes = {"x": abs(float(axis[0])), "y": abs(float(axis[1])), "z": abs(float(axis[2]))}
    axis_name = max(magnitudes, key=magnitudes.get)
    return getattr(UsdPhysics.Tokens, axis_name)


def _find_root_links(links: list[dict[str, Any]]) -> list[dict[str, Any]]:
    names = {link.get("name", "") for link in links}
    return [link for link in links if not link.get("parent_name") or link.get("parent_name") not in names]


def _define_default_material(stage: Usd.Stage, robot_path: Sdf.Path) -> UsdShade.Material:
    material = UsdShade.Material.Define(stage, robot_path.AppendPath("Materials/DefaultPhysicsMaterial"))
    material_api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    material_api.CreateDensityAttr().Set(1000.0)
    material_api.CreateDynamicFrictionAttr().Set(0.5)
    material_api.CreateStaticFrictionAttr().Set(0.5)
    material_api.CreateRestitutionAttr().Set(0.0)
    material.GetPrim().ApplyAPI("NewtonMaterialAPI")
    material.GetPrim().GetAttribute("newton:torsionalFriction").Set(0.005)
    material.GetPrim().GetAttribute("newton:rollingFriction").Set(0.0001)
    return material


def _bind_material(geom_prim: Usd.Prim, material: UsdShade.Material | None) -> None:
    if not material:
        return
    UsdShade.MaterialBindingAPI(geom_prim).Bind(material)


def _define_collision_geom(
    stage: Usd.Stage,
    link_prim: Usd.Prim,
    primitive_data: dict[str, Any] | None,
    material: UsdShade.Material | None,
) -> None:
    if not primitive_data:
        return

    primitive_type = primitive_data.get("type")
    geom_path = link_prim.GetPath().AppendChild("Collision")
    geom_prim: Usd.Prim | None = None

    if primitive_type == "box":
        cube = UsdGeom.Cube.Define(stage, geom_path)
        cube.GetSizeAttr().Set(1.0)
        scale = primitive_data.get("size_m", [1.0, 1.0, 1.0])
        xformable = UsdGeom.Xformable(cube)
        xformable.AddScaleOp().Set(_vec3(scale))
        geom_prim = cube.GetPrim()
    elif primitive_type == "sphere":
        sphere = UsdGeom.Sphere.Define(stage, geom_path)
        sphere.GetRadiusAttr().Set(float(primitive_data.get("radius_m", 0.5)))
        geom_prim = sphere.GetPrim()
    elif primitive_type == "capsule":
        capsule = UsdGeom.Capsule.Define(stage, geom_path)
        capsule.GetAxisAttr().Set(UsdGeom.Tokens.z)
        capsule.GetRadiusAttr().Set(float(primitive_data.get("radius_m", 0.1)))
        capsule.GetHeightAttr().Set(float(primitive_data.get("cylinder_height_m", 0.0)))
        geom_prim = capsule.GetPrim()

    if not geom_prim:
        return

    UsdGeom.Imageable(geom_prim).GetPurposeAttr().Set(UsdGeom.Tokens.guide)
    UsdPhysics.CollisionAPI.Apply(geom_prim)
    geom_prim.ApplyAPI("NewtonCollisionAPI")
    geom_prim.GetAttribute("physics:collisionEnabled").Set(True)
    geom_prim.GetAttribute("newton:contactMargin").Set(0.0)
    _bind_material(geom_prim, material)


def _define_links(
    stage: Usd.Stage,
    geometry_scope_path: Sdf.Path,
    links: list[dict[str, Any]],
    material: UsdShade.Material | None,
) -> dict[str, Sdf.Path]:
    links_by_name = {link.get("name", ""): link for link in links if link.get("name")}
    link_paths: dict[str, Sdf.Path] = {}
    child_name_counts: dict[str, dict[str, int]] = {}

    def ensure_link(link_name: str) -> Sdf.Path:
        if link_name in link_paths:
            return link_paths[link_name]

        link = links_by_name[link_name]
        parent_name = link.get("parent_name") or ""
        if parent_name and parent_name in links_by_name:
            parent_path = ensure_link(parent_name)
        else:
            parent_path = geometry_scope_path

        parent_key = str(parent_path)
        safe_name = _safe_name(link_name, "Link")
        name_counts = child_name_counts.setdefault(parent_key, {})
        count = name_counts.get(safe_name, 0)
        name_counts[safe_name] = count + 1
        if count:
            safe_name = f"{safe_name}_{count + 1}"

        prim = UsdGeom.Xform.Define(stage, parent_path.AppendChild(safe_name)).GetPrim()
        _apply_transform(prim, link.get("relative_transform"))

        rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(prim)
        rigid_body_api.CreateRigidBodyEnabledAttr().Set(True)
        if link.get("kinematic") or link.get("treat_as_terrain"):
            rigid_body_api.CreateKinematicEnabledAttr().Set(True)

        mass_api = UsdPhysics.MassAPI.Apply(prim)
        mass_api.CreateMassAttr().Set(float(link.get("mass_kg", 1.0)))

        _set_custom_attr(prim, "ramms:sourceComponent", link.get("component_name") or "")
        _set_custom_attr(prim, "ramms:transformSource", link.get("transform_source") or "")
        _set_custom_attr(prim, "ramms:linearDamping", float(link.get("linear_damping", 0.0)))
        _set_custom_attr(prim, "ramms:angularDamping", float(link.get("angular_damping", 0.0)))

        primitive_data = link.get("primitive") or {}
        _set_custom_attr(prim, "ramms:primitiveType", primitive_data.get("type") or "")
        _set_custom_attr(prim, "ramms:renderAssetPath", primitive_data.get("render_asset_path"))
        _set_custom_attr(prim, "ramms:materialPaths", primitive_data.get("material_paths"))

        _define_collision_geom(stage, prim, primitive_data, material)

        link_paths[link_name] = prim.GetPath()
        return prim.GetPath()

    for link_name in links_by_name:
        ensure_link(link_name)

    return link_paths


def _joint_local_pos0(joint: dict[str, Any], links_by_name: dict[str, dict[str, Any]]) -> Gf.Vec3f:
    child_name = joint.get("child_link_name") or ""
    child_link = links_by_name.get(child_name)
    if not child_link:
        return Gf.Vec3f(0.0, 0.0, 0.0)
    relative = child_link.get("relative_transform") or {}
    return _vec3(relative.get("translation_m"))


def _create_joint_prim(stage: Usd.Stage, joint_path: Sdf.Path, joint_type: str):
    if joint_type == "fixed":
        return UsdPhysics.FixedJoint.Define(stage, joint_path)
    if joint_type == "prismatic":
        return UsdPhysics.PrismaticJoint.Define(stage, joint_path)
    if joint_type == "spherical":
        return UsdPhysics.SphericalJoint.Define(stage, joint_path)
    return UsdPhysics.RevoluteJoint.Define(stage, joint_path)


def _controller_joint_lookup(data: dict[str, Any]) -> dict[str, dict[str, Any]]:
    lookup: dict[str, dict[str, Any]] = {}
    controllers = data.get("controllers") or {}

    kinova = controllers.get("kinova") or {}
    for joint in kinova.get("joints") or []:
        joint_name = joint.get("name")
        if joint_name:
            lookup[joint_name] = {"kind": "kinova", **joint}

    gripper = controllers.get("gripper") or {}
    for finger in gripper.get("fingers") or []:
        joint_name = finger.get("constraint_name")
        if joint_name:
            lookup[joint_name] = {"kind": "gripper", **finger}

    return lookup


def _define_joints(
    stage: Usd.Stage,
    robot_prim: Usd.Prim,
    physics_scope_path: Sdf.Path,
    links: list[dict[str, Any]],
    joints: list[dict[str, Any]],
    link_paths: dict[str, Sdf.Path],
) -> dict[str, Sdf.Path]:
    links_by_name = {link.get("name", ""): link for link in links if link.get("name")}
    joint_paths: dict[str, Sdf.Path] = {}
    name_counts: dict[str, int] = {}

    for joint in joints:
        joint_name = joint.get("name") or "joint"
        safe_name = _safe_name(joint_name, "Joint")
        count = name_counts.get(safe_name, 0)
        name_counts[safe_name] = count + 1
        if count:
            safe_name = f"{safe_name}_{count + 1}"

        joint_prim_schema = _create_joint_prim(stage, physics_scope_path.AppendChild(safe_name), joint.get("joint_type", "revolute"))
        joint_prim = joint_prim_schema.GetPrim()

        body0_target = link_paths.get(joint.get("parent_link_name") or "", robot_prim.GetPath())
        body1_target = link_paths.get(joint.get("child_link_name") or "")
        if not body1_target:
            continue

        joint_prim_schema.CreateBody0Rel().SetTargets([body0_target])
        joint_prim_schema.CreateBody1Rel().SetTargets([body1_target])
        joint_prim_schema.CreateLocalPos0Attr().Set(_joint_local_pos0(joint, links_by_name))
        joint_prim_schema.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint_prim_schema.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint_prim_schema.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint_prim_schema.CreateJointEnabledAttr().Set(True)

        if hasattr(joint_prim_schema, "CreateAxisAttr"):
            joint_prim_schema.CreateAxisAttr().Set(_axis_token(joint.get("axis")))

        if joint.get("use_limits") and hasattr(joint_prim_schema, "CreateLowerLimitAttr"):
            joint_prim_schema.CreateLowerLimitAttr().Set(float(joint.get("min_limit_degrees", 0.0)))
            joint_prim_schema.CreateUpperLimitAttr().Set(float(joint.get("max_limit_degrees", 0.0)))

        max_effort = float(joint.get("max_effort", 0.0))
        if max_effort > 0.0:
            _set_custom_attr(joint_prim, "urdf:limit:effort", max_effort)

        _set_custom_attr(joint_prim, "ramms:driveMode", joint.get("drive_mode") or "")
        _set_custom_attr(joint_prim, "ramms:sourceConstraint", joint.get("constraint_name") or "")
        _set_custom_attr(joint_prim, "ramms:sourceBone", joint.get("bone_name") or "")

        joint_paths[joint_name] = joint_prim.GetPath()

    return joint_paths


def _define_root_joints(
    stage: Usd.Stage,
    robot_prim: Usd.Prim,
    physics_scope_path: Sdf.Path,
    root_links: list[dict[str, Any]],
    link_paths: dict[str, Sdf.Path],
) -> None:
    for index, link in enumerate(root_links):
        link_name = link.get("name") or ""
        body1_target = link_paths.get(link_name)
        if not body1_target:
            continue

        joint_name = "root_joint" if index == 0 else f"root_joint_{_safe_name(link_name, 'Link')}"
        joint = UsdPhysics.FixedJoint.Define(stage, physics_scope_path.AppendChild(joint_name))
        joint.CreateBody0Rel().SetTargets([robot_prim.GetPath()])
        joint.CreateBody1Rel().SetTargets([body1_target])
        joint.CreateLocalPos0Attr().Set(_vec3((0.0, 0.0, 0.0)))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint.CreateJointEnabledAttr().Set(True)


def _define_scene(stage: Usd.Stage, scene_path: Sdf.Path, scene_data: dict[str, Any]) -> None:
    scene = UsdPhysics.Scene.Define(stage, scene_path)
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81 if scene_data.get("gravity_enabled", True) else 0.0)

    prim = scene.GetPrim()
    prim.ApplyAPI("NewtonSceneAPI")
    prim.ApplyAPI("NewtonXpbdSceneAPI")
    prim.GetAttribute("newton:gravityEnabled").Set(bool(scene_data.get("gravity_enabled", True)))
    prim.GetAttribute("newton:timeStepsPerSecond").Set(int(scene_data.get("time_steps_per_second", 60)))
    prim.GetAttribute("newton:maxSolverIterations").Set(int(scene_data.get("max_solver_iterations", 8)))
    prim.GetAttribute("newton:xpbd:rigidContactRelaxation").Set(float(scene_data.get("rigid_contact_relaxation", 0.8)))
    prim.GetAttribute("newton:xpbd:restitutionEnabled").Set(bool(scene_data.get("restitution_enabled", True)))


def _define_actuators(
    stage: Usd.Stage,
    actuator_scope_path: Sdf.Path,
    data: dict[str, Any],
    joint_paths: dict[str, Sdf.Path],
) -> None:
    controller_lookup = _controller_joint_lookup(data)
    name_counts: dict[str, int] = {}

    for joint in data.get("joints") or []:
        joint_name = joint.get("name") or ""
        drive_mode = joint.get("drive_mode") or "passive"
        joint_path = joint_paths.get(joint_name)
        if not joint_path or drive_mode == "passive":
            continue

        safe_name = _safe_name(f"{joint_name}_actuator", "Actuator")
        count = name_counts.get(safe_name, 0)
        name_counts[safe_name] = count + 1
        if count:
            safe_name = f"{safe_name}_{count + 1}"

        prim = stage.DefinePrim(actuator_scope_path.AppendChild(safe_name), "NewtonActuator")
        prim.GetRelationship("newton:targets").SetTargets([joint_path])
        _set_custom_attr(prim, "ramms:driveMode", drive_mode)

        controller_data = controller_lookup.get(joint_name, {})
        if controller_data:
            _set_custom_attr(prim, "ramms:controllerKind", controller_data.get("kind"))

        if drive_mode == "position_control":
            kp = controller_data.get("position_strength", controller_data.get("motor_strength"))
            kd = controller_data.get("position_damping", controller_data.get("motor_damping"))
            if kp is not None or kd is not None:
                prim.ApplyAPI("NewtonPDControlAPI")
                if kp is not None:
                    prim.GetAttribute("newton:kp").Set(float(kp))
                if kd is not None:
                    prim.GetAttribute("newton:kd").Set(float(kd))

        max_effort = float(joint.get("max_effort", 0.0))
        if max_effort > 0.0:
            prim.ApplyAPI("NewtonMaxEffortClampingAPI")
            prim.GetAttribute("newton:maxEffort").Set(max_effort)

        max_speed_deg = controller_data.get("max_speed_degrees_per_second")
        if max_speed_deg is not None:
            _set_custom_attr(prim, "ramms:maxSpeedRadiansPerSecond", math.radians(float(max_speed_deg)))
            joint_prim = stage.GetPrimAtPath(joint_path)
            _set_custom_attr(joint_prim, "urdf:limit:velocity", math.radians(float(max_speed_deg)))


def export_robot_data(data: dict[str, Any], output_path: str | pathlib.Path) -> pathlib.Path:
    output_path = pathlib.Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    robot_name = _safe_name(data.get("robot_name"), "RammsRobot")
    stage = Usd.Stage.CreateNew(str(output_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, float((data.get("stage") or {}).get("meters_per_unit", 1.0)))
    UsdPhysics.SetStageKilogramsPerUnit(stage, float((data.get("stage") or {}).get("kilograms_per_unit", 1.0)))

    robot = UsdGeom.Xform.Define(stage, f"/{robot_name}")
    stage.SetDefaultPrim(robot.GetPrim())
    _apply_transform(robot.GetPrim(), data.get("actor_transform"))

    geometry_scope = stage.DefinePrim(robot.GetPath().AppendChild("Geometry"), "Scope")
    physics_scope = stage.DefinePrim(robot.GetPath().AppendChild("Physics"), "Scope")
    stage.DefinePrim(robot.GetPath().AppendChild("Actuators"), "Scope")
    stage.DefinePrim(robot.GetPath().AppendChild("Materials"), "Scope")

    material = _define_default_material(stage, robot.GetPath())

    links = data.get("links") or []
    link_paths = _define_links(stage, geometry_scope.GetPath(), links, material)
    root_links = _find_root_links(links)
    if root_links:
        first_root_name = root_links[0].get("name") or ""
        first_root_path = link_paths.get(first_root_name)
        if first_root_path:
            stage.GetPrimAtPath(first_root_path).ApplyAPI("NewtonArticulationRootAPI")

    _define_scene(stage, robot.GetPath().AppendChild("Scene"), data.get("scene") or {})
    joint_paths = _define_joints(stage, robot.GetPrim(), physics_scope.GetPath(), links, data.get("joints") or [], link_paths)
    _define_root_joints(stage, robot.GetPrim(), physics_scope.GetPath(), root_links, link_paths)
    _define_actuators(stage, robot.GetPath().AppendChild("Actuators"), data, joint_paths)

    stage.GetRootLayer().Save()
    return output_path


def export_robot_json(json_text: str, output_path: str | pathlib.Path) -> pathlib.Path:
    return export_robot_data(json.loads(json_text), output_path)


def export_robot_json_file(input_path: str | pathlib.Path, output_path: str | pathlib.Path) -> pathlib.Path:
    return export_robot_data(json.loads(pathlib.Path(input_path).read_text(encoding="utf-8")), output_path)


def _main() -> int:
    parser = argparse.ArgumentParser(description="Export RAMMS articulated robot JSON to Newton-compatible USD.")
    parser.add_argument("--input", required=True, help="Path to the RAMMS robot export JSON file.")
    parser.add_argument("--output", required=True, help="Destination .usd/.usda path.")
    args = parser.parse_args()

    export_robot_json_file(args.input, args.output)
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
