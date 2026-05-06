// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsNewtonPhysicsModule.h"

#include "HAL/PlatformProcess.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"
#include "RammsNewtonPhysicsSettings.h"

DEFINE_LOG_CATEGORY_STATIC(LogRammsNewtonPhysics, Log, All);

namespace
{
	FString GetRammsNewtonPluginBaseDir()
	{
		const TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("RammsNewtonPhysics"));
		return Plugin.IsValid()
			? Plugin->GetBaseDir()
			: FPaths::Combine(FPaths::ProjectPluginsDir(), TEXT("RammsNewtonPhysics"));
	}

	FString ResolvePythonWorkerScriptPath(const URammsNewtonPhysicsSettings* Settings)
	{
		if (Settings && !Settings->PythonWorkerScriptPath.IsEmpty())
		{
			return FPaths::IsRelative(Settings->PythonWorkerScriptPath)
				? FPaths::ConvertRelativePathToFull(FPaths::ProjectDir(), Settings->PythonWorkerScriptPath)
				: Settings->PythonWorkerScriptPath;
		}

		return FPaths::Combine(GetRammsNewtonPluginBaseDir(), TEXT("Scripts"), TEXT("ramms_newton_worker.py"));
	}
} // namespace

void FRammsNewtonPhysicsModule::StartupModule()
{
	RefreshBackendStatus();
	UE_LOG(LogRammsNewtonPhysics, Log, TEXT("%s"), *CachedStatus.Summary);
}

void FRammsNewtonPhysicsModule::ShutdownModule()
{
	if (RuntimeLibraryHandle)
	{
		FPlatformProcess::FreeDllHandle(RuntimeLibraryHandle);
		RuntimeLibraryHandle = nullptr;
	}
}

FRammsNewtonBackendStatus FRammsNewtonPhysicsModule::GetBackendStatus() const
{
	return CachedStatus;
}

bool FRammsNewtonPhysicsModule::IsBackendReady() const
{
	return CachedStatus.bRuntimeReady;
}

void* FRammsNewtonPhysicsModule::GetRuntimeSymbol(const TCHAR* SymbolName) const
{
	return RuntimeLibraryHandle ? FPlatformProcess::GetDllExport(RuntimeLibraryHandle, SymbolName) : nullptr;
}

void FRammsNewtonPhysicsModule::RefreshBackendStatus()
{
	CachedStatus = FRammsNewtonBackendStatus();
	CachedStatus.bSourceCheckoutDetected = !!RAMMS_NEWTON_HAS_SOURCE_CHECKOUT;
	CachedStatus.bHeadersDetected = !!RAMMS_NEWTON_HAS_HEADERS;

	FString	   LoadedLibraryPath;
	const bool bRuntimeLoaded = TryLoadRuntimeLibrary(LoadedLibraryPath);
	CachedStatus.bRuntimeLibraryLoaded = bRuntimeLoaded;
	CachedStatus.RuntimeLibraryPath = LoadedLibraryPath;
	const URammsNewtonPhysicsSettings* Settings = GetDefault<URammsNewtonPhysicsSettings>();
	const bool						   bPreferPythonBridge = Settings && Settings->bPreferPythonWorkerBridge;
	const FString					   PythonWorkerScriptPath = ResolvePythonWorkerScriptPath(Settings);
	const bool						   bPythonWorkerScriptExists = FPaths::FileExists(PythonWorkerScriptPath);

#if RAMMS_NEWTON_HAS_PREBUILT
	if (bRuntimeLoaded)
	{
		CachedStatus.Mode = ERammsNewtonBackendMode::PrebuiltLibrary;
		CachedStatus.bRuntimeReady = true;
		CachedStatus.Summary = FString::Printf(TEXT("RammsNewtonPhysics: prebuilt Newton runtime loaded from '%s'."), *LoadedLibraryPath);
	}
	else
	{
		CachedStatus.Mode = ERammsNewtonBackendMode::BuiltInAdapter;
		CachedStatus.bRuntimeReady = true;
		CachedStatus.Summary = TEXT("RammsNewtonPhysics: prebuilt Newton library was detected at build time, but no runtime binary was loaded. Falling back to the built-in RAMMS adapter.");
	}
#elif RAMMS_NEWTON_HAS_HEADERS
	if (bRuntimeLoaded)
	{
		CachedStatus.Mode = ERammsNewtonBackendMode::DynamicLibrary;
		CachedStatus.bRuntimeReady = true;
		CachedStatus.Summary = FString::Printf(TEXT("RammsNewtonPhysics: Newton runtime loaded dynamically from '%s'."), *LoadedLibraryPath);
	}
	else if (bPreferPythonBridge && CachedStatus.bSourceCheckoutDetected && bPythonWorkerScriptExists)
	{
		CachedStatus.Mode = ERammsNewtonBackendMode::ExternalPythonBridge;
		CachedStatus.bRuntimeReady = true;
		CachedStatus.Summary = FString::Printf(
			TEXT("RammsNewtonPhysics: Newton source checkout detected. Using the external Python worker bridge at '%s' and falling back to the built-in RAMMS adapter if the worker cannot start."),
			*PythonWorkerScriptPath);
	}
	else
	{
		CachedStatus.Mode = ERammsNewtonBackendMode::BuiltInAdapter;
		CachedStatus.bRuntimeReady = true;
	#if RAMMS_NEWTON_HAS_SOURCE_CHECKOUT
		if (bPreferPythonBridge && !bPythonWorkerScriptExists)
		{
			CachedStatus.Summary = FString::Printf(
				TEXT("RammsNewtonPhysics: Newton source checkout detected, but the configured Python worker script was not found at '%s'. Falling back to the built-in RAMMS adapter."),
				*PythonWorkerScriptPath);
		}
		else
		{
			CachedStatus.Summary = TEXT("RammsNewtonPhysics: Newton source checkout detected, but no native runtime is loaded yet. Falling back to the built-in RAMMS adapter.");
		}
	#else
		CachedStatus.Summary = TEXT("RammsNewtonPhysics: Newton headers detected, but no prebuilt runtime library was found. Falling back to the built-in RAMMS adapter.");
	#endif
	}
#else
	if (bRuntimeLoaded)
	{
		CachedStatus.Mode = ERammsNewtonBackendMode::DynamicLibrary;
		CachedStatus.bRuntimeReady = true;
		CachedStatus.Summary = FString::Printf(TEXT("RammsNewtonPhysics: Newton runtime loaded dynamically from '%s'."), *LoadedLibraryPath);
	}
	else if (bPreferPythonBridge && CachedStatus.bSourceCheckoutDetected && bPythonWorkerScriptExists)
	{
		CachedStatus.Mode = ERammsNewtonBackendMode::ExternalPythonBridge;
		CachedStatus.bRuntimeReady = true;
		CachedStatus.Summary = FString::Printf(
			TEXT("RammsNewtonPhysics: Newton source checkout detected. Using the external Python worker bridge at '%s' and falling back to the built-in RAMMS adapter if the worker cannot start."),
			*PythonWorkerScriptPath);
	}
	else
	{
		CachedStatus.Mode = ERammsNewtonBackendMode::BuiltInAdapter;
		CachedStatus.bRuntimeReady = true;
		CachedStatus.Summary = TEXT("RammsNewtonPhysics: no external Newton runtime detected. Using the built-in RAMMS adapter.");
	}
#endif
}

bool FRammsNewtonPhysicsModule::TryLoadRuntimeLibrary(FString& OutLoadedPath)
{
	if (RuntimeLibraryHandle)
	{
		OutLoadedPath = CachedStatus.RuntimeLibraryPath;
		return true;
	}

	for (const FString& Candidate : GetRuntimeLibraryCandidates())
	{
		if (!FPaths::FileExists(Candidate))
		{
			continue;
		}

		RuntimeLibraryHandle = FPlatformProcess::GetDllHandle(*Candidate);
		if (RuntimeLibraryHandle)
		{
			OutLoadedPath = Candidate;
			return true;
		}
	}

	return false;
}

TArray<FString> FRammsNewtonPhysicsModule::GetRuntimeLibraryCandidates() const
{
	TArray<FString> Candidates;

	TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("RammsNewtonPhysics"));
	const FString		PluginBaseDir = Plugin.IsValid()
			  ? Plugin->GetBaseDir()
			  : FPaths::Combine(FPaths::ProjectPluginsDir(), TEXT("RammsNewtonPhysics"));

#if PLATFORM_WINDOWS
	const FString PlatformDir = TEXT("Win64");
#elif PLATFORM_LINUX
	const FString PlatformDir = TEXT("Linux");
#elif PLATFORM_MAC
	const FString PlatformDir = TEXT("Mac");
#else
	const FString PlatformDir;
#endif

	if (PlatformDir.IsEmpty())
	{
		return Candidates;
	}

	const FString BinDir = FPaths::Combine(PluginBaseDir, TEXT("ThirdParty"), TEXT("Prebuilt"), PlatformDir, TEXT("bin"));
	const FString LibDir = FPaths::Combine(PluginBaseDir, TEXT("ThirdParty"), TEXT("Prebuilt"), PlatformDir, TEXT("lib"));

#if PLATFORM_WINDOWS
	Candidates.Add(FPaths::Combine(BinDir, TEXT("newton.dll")));
	Candidates.Add(FPaths::Combine(BinDir, TEXT("Newton.dll")));
#elif PLATFORM_LINUX
	Candidates.Add(FPaths::Combine(LibDir, TEXT("libnewton.so")));
#elif PLATFORM_MAC
	Candidates.Add(FPaths::Combine(LibDir, TEXT("libnewton.dylib")));
#endif

	return Candidates;
}

IMPLEMENT_MODULE(FRammsNewtonPhysicsModule, RammsNewtonPhysics)
