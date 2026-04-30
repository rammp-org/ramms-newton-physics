using System;
using System.IO;
using System.Linq;
using UnrealBuildTool;

public class RammsNewtonPhysicsThirdParty : ModuleRules
{
	public RammsNewtonPhysicsThirdParty(ReadOnlyTargetRules Target) : base(Target)
	{
		Type = ModuleType.External;

		string PluginDir = Path.GetFullPath(Path.Combine(ModuleDirectory, "..", ".."));
		string SourceCheckoutA = Path.Combine(PluginDir, "ThirdParty", "newton-dynamics");
		string SourceCheckoutB = Path.Combine(PluginDir, "ThirdParty", "NewtonDynamics");
		string PrebuiltRoot = Path.Combine(PluginDir, "ThirdParty", "Prebuilt", Target.Platform.ToString());

		bool HasSourceCheckout = Directory.Exists(SourceCheckoutA) || Directory.Exists(SourceCheckoutB);
		string[] IncludeCandidates =
		{
			Path.Combine(SourceCheckoutA, "include"),
			Path.Combine(SourceCheckoutA, "sdk"),
			Path.Combine(SourceCheckoutB, "include"),
			Path.Combine(SourceCheckoutB, "sdk"),
			Path.Combine(PrebuiltRoot, "include"),
		};

		string IncludeDir = IncludeCandidates.FirstOrDefault(Directory.Exists);
		if (!string.IsNullOrEmpty(IncludeDir))
		{
			PublicSystemIncludePaths.Add(IncludeDir);
		}

		bool HasPrebuilt = false;
		string LibDir = Path.Combine(PrebuiltRoot, "lib");
		string BinDir = Path.Combine(PrebuiltRoot, "bin");

		if (Target.Platform == UnrealTargetPlatform.Win64)
		{
			string ImportLib = Path.Combine(LibDir, "newton.lib");
			string DelayLoadDll = Path.Combine(BinDir, "newton.dll");
			if (File.Exists(ImportLib))
			{
				PublicAdditionalLibraries.Add(ImportLib);
				HasPrebuilt = true;
			}
			if (File.Exists(DelayLoadDll))
			{
				PublicDelayLoadDLLs.Add("newton.dll");
				RuntimeDependencies.Add(DelayLoadDll);
			}
		}
		else if (Target.Platform == UnrealTargetPlatform.Linux)
		{
			string SharedObject = Path.Combine(LibDir, "libnewton.so");
			if (File.Exists(SharedObject))
			{
				PublicAdditionalLibraries.Add(SharedObject);
				RuntimeDependencies.Add(SharedObject);
				HasPrebuilt = true;
			}
		}
		else if (Target.Platform == UnrealTargetPlatform.Mac)
		{
			string Dylib = Path.Combine(LibDir, "libnewton.dylib");
			if (File.Exists(Dylib))
			{
				PublicAdditionalLibraries.Add(Dylib);
				RuntimeDependencies.Add(Dylib);
				HasPrebuilt = true;
			}
		}

		PublicDefinitions.Add($"RAMMS_NEWTON_HAS_SOURCE_CHECKOUT={(HasSourceCheckout ? 1 : 0)}");
		PublicDefinitions.Add($"RAMMS_NEWTON_HAS_HEADERS={(!string.IsNullOrEmpty(IncludeDir) ? 1 : 0)}");
		PublicDefinitions.Add($"RAMMS_NEWTON_HAS_PREBUILT={(HasPrebuilt ? 1 : 0)}");
	}
}
