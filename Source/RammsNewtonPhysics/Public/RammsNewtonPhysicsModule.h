// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Modules/ModuleManager.h"
#include "RammsNewtonPhysicsTypes.h"

class RAMMSNEWTONPHYSICS_API FRammsNewtonPhysicsModule : public IModuleInterface
{
public:
	static inline FRammsNewtonPhysicsModule& Get()
	{
		return FModuleManager::LoadModuleChecked<FRammsNewtonPhysicsModule>("RammsNewtonPhysics");
	}

	static inline bool IsAvailable()
	{
		return FModuleManager::Get().IsModuleLoaded("RammsNewtonPhysics");
	}

	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

	FRammsNewtonBackendStatus GetBackendStatus() const;
	bool					  IsBackendReady() const;

private:
	void RefreshBackendStatus();

	FRammsNewtonBackendStatus CachedStatus;
};
