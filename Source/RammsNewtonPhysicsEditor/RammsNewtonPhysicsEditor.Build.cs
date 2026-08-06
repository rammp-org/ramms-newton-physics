// Copyright RAMMP. All Rights Reserved.

using UnrealBuildTool;

public class RammsNewtonPhysicsEditor : ModuleRules
{
	public RammsNewtonPhysicsEditor(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		// Matches the runtime module: URLab pulls in windows.h via its
		// third-party headers.
		bUseUnity = false;

		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				"CoreUObject",
				"Engine",
				"Projects",
				"RammsNewtonPhysics",
				"Slate",
				"SlateCore",
				"ToolMenus",
				"UnrealEd",
				"URLab",
			});
	}
}
