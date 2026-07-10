using UnrealBuildTool;

public class RammsNewtonPhysics : ModuleRules
{
	public RammsNewtonPhysics(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				"CoreUObject",
				"Engine",
				"DeveloperSettings",
			});

		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"Json",
				"Projects",
				"RammsCore",
				"RammsNewtonPhysicsThirdParty",
			});
	}
}
