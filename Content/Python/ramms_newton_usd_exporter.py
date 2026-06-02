from __future__ import annotations

import json
from pathlib import Path

import unreal

import ramms_newton_usd


def _find_articulated_component(actor: unreal.Actor, component_name: str = ""):
    components = actor.get_components_by_class(unreal.RammsNewtonArticulatedRobotComponent)
    if component_name:
        for component in components:
            if component.get_name() == component_name:
                return component
    return components[0] if components else None


def export_actor_to_newton_usd(actor: unreal.Actor, output_path: str, component_name: str = "") -> str:
    component = _find_articulated_component(actor, component_name)
    if not component:
        raise RuntimeError(f"Actor '{actor.get_name()}' has no RammsNewtonArticulatedRobotComponent.")

    export_data = json.loads(component.get_effective_robot_export_json())
    result_path = ramms_newton_usd.export_robot_data(export_data, output_path)
    unreal.log(f"[RammsNewtonPhysics] Exported Newton USD: {result_path}")
    return str(result_path)


def export_selected_actors_to_newton_usd(output_directory: str) -> list[str]:
    output_dir = Path(output_directory)
    output_dir.mkdir(parents=True, exist_ok=True)

    exported_paths: list[str] = []
    for actor in unreal.EditorLevelLibrary.get_selected_level_actors():
        component = _find_articulated_component(actor)
        if not component:
            continue
        output_path = output_dir / f"{actor.get_name()}.usda"
        exported_paths.append(export_actor_to_newton_usd(actor, str(output_path)))

    if not exported_paths:
        unreal.log_warning("[RammsNewtonPhysics] No selected actors had a RammsNewtonArticulatedRobotComponent.")

    return exported_paths
