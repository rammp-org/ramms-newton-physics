using UnrealBuildTool;

public class RammsNewtonPhysics : ModuleRules
{
	public RammsNewtonPhysics(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		// zmq.h pulls in windows.h whose macros (GetObject, ...) leak across
		// unity TUs and break engine headers — same setting URLab uses.
		bUseUnity = false;

		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				"CoreUObject",
				"Engine",
				"DeveloperSettings",
				// URLab exposes <mujoco/mujoco.h> and <zmq.h> via public
				// include paths and links both libs publicly, so this one
				// dependency provides the MuJoCo C API, libzmq, and the
				// UMjPhysicsEngine/AAMjManager integration surface.
				"URLab",
			});

		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"Json",
				"Projects",
			});
	}
}
