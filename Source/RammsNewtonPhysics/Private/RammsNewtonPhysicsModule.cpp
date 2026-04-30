// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsNewtonPhysicsModule.h"

#include "Modules/ModuleManager.h"

DEFINE_LOG_CATEGORY_STATIC(LogRammsNewtonPhysics, Log, All);

void FRammsNewtonPhysicsModule::StartupModule()
{
	RefreshBackendStatus();
	UE_LOG(LogRammsNewtonPhysics, Log, TEXT("%s"), *CachedStatus.Summary);
}

void FRammsNewtonPhysicsModule::ShutdownModule()
{
}

FRammsNewtonBackendStatus FRammsNewtonPhysicsModule::GetBackendStatus() const
{
	return CachedStatus;
}

bool FRammsNewtonPhysicsModule::IsBackendReady() const
{
	return CachedStatus.bRuntimeReady;
}

void FRammsNewtonPhysicsModule::RefreshBackendStatus()
{
	CachedStatus = FRammsNewtonBackendStatus();

#if RAMMS_NEWTON_HAS_PREBUILT
	CachedStatus.Mode = ERammsNewtonBackendMode::PrebuiltLibrary;
	CachedStatus.bRuntimeReady = true;
	CachedStatus.Summary = TEXT("RammsNewtonPhysics: prebuilt Newton runtime detected and linked.");
#elif RAMMS_NEWTON_HAS_HEADERS
	CachedStatus.Mode = ERammsNewtonBackendMode::HeadersOnly;
	CachedStatus.bRuntimeReady = false;
	#if RAMMS_NEWTON_HAS_SOURCE_CHECKOUT
		CachedStatus.Summary = TEXT("RammsNewtonPhysics: Newton source checkout detected, but direct UBT compilation is not wired yet. Add a prebuilt runtime or extend the ThirdParty module.");
	#else
		CachedStatus.Summary = TEXT("RammsNewtonPhysics: Newton headers detected, but no prebuilt runtime library was found.");
	#endif
#else
	CachedStatus.Mode = ERammsNewtonBackendMode::StubOnly;
	CachedStatus.bRuntimeReady = false;
	CachedStatus.Summary = TEXT("RammsNewtonPhysics: no Newton SDK detected. Plugin is running in scaffold mode only.");
#endif
}

IMPLEMENT_MODULE(FRammsNewtonPhysicsModule, RammsNewtonPhysics)
