// Copyright RAMMP. All Rights Reserved.

#include "RammsNewtonPhysicsModule.h"

#include "Modules/ModuleManager.h"
#include "RammsNewtonPhysicsTypes.h"

DEFINE_LOG_CATEGORY(LogRammsNewton);

void FRammsNewtonPhysicsModule::StartupModule()
{
	UE_LOG(LogRammsNewton, Log,
		TEXT("RammsNewtonPhysics loaded (Newton runs out-of-process; availability is probed at runtime)"));
}

void FRammsNewtonPhysicsModule::ShutdownModule()
{
}

IMPLEMENT_MODULE(FRammsNewtonPhysicsModule, RammsNewtonPhysics)
