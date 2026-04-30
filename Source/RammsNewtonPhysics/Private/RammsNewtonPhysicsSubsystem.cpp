// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsNewtonPhysicsSubsystem.h"

#include "RammsNewtonPhysicsComponent.h"
#include "RammsNewtonPhysicsModule.h"
#include "RammsNewtonPhysicsSettings.h"

DEFINE_LOG_CATEGORY_STATIC(LogRammsNewtonPhysicsSubsystem, Log, All);

void URammsNewtonPhysicsSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);
	RegisteredBridges.Reset();
	AccumulatedTimeSeconds = 0.0f;
	StepCounter = 0;
	bLoggedUnavailable = false;
}

void URammsNewtonPhysicsSubsystem::Deinitialize()
{
	RegisteredBridges.Reset();
	Super::Deinitialize();
}

void URammsNewtonPhysicsSubsystem::Tick(float DeltaTime)
{
	const URammsNewtonPhysicsSettings* Settings = GetDefault<URammsNewtonPhysicsSettings>();
	if (!Settings || !Settings->bEnableSubsystemStepping)
	{
		return;
	}

	RegisteredBridges.RemoveAllSwap([](const TWeakObjectPtr<URammsNewtonPhysicsComponent>& Bridge) {
		return !Bridge.IsValid();
	});

	if (RegisteredBridges.Num() == 0)
	{
		return;
	}

	const FRammsNewtonBackendStatus BackendStatus = GetBackendStatus();
	if (!BackendStatus.bRuntimeReady)
	{
		if (Settings->bLogBackendWarnings && !bLoggedUnavailable)
		{
			bLoggedUnavailable = true;
			UE_LOG(LogRammsNewtonPhysicsSubsystem, Warning, TEXT("%s"), *BackendStatus.Summary);
		}
		return;
	}

	const float FixedStepSeconds = (Settings->FixedStepHz > KINDA_SMALL_NUMBER)
		? (1.0f / Settings->FixedStepHz)
		: (1.0f / 60.0f);

	AccumulatedTimeSeconds += DeltaTime;
	int32 Substeps = 0;
	const int32 MaxSubsteps = FMath::Max(1, Settings->MaxSubstepsPerTick);
	while (AccumulatedTimeSeconds >= FixedStepSeconds && Substeps < MaxSubsteps)
	{
		StepSimulation(FixedStepSeconds);
		AccumulatedTimeSeconds -= FixedStepSeconds;
		++Substeps;
	}
}

TStatId URammsNewtonPhysicsSubsystem::GetStatId() const
{
	RETURN_QUICK_DECLARE_CYCLE_STAT(URammsNewtonPhysicsSubsystem, STATGROUP_Tickables);
}

void URammsNewtonPhysicsSubsystem::RegisterBridge(URammsNewtonPhysicsComponent* Bridge)
{
	if (Bridge)
	{
		RegisteredBridges.AddUnique(Bridge);
	}
}

void URammsNewtonPhysicsSubsystem::UnregisterBridge(URammsNewtonPhysicsComponent* Bridge)
{
	RegisteredBridges.RemoveAllSwap([Bridge](const TWeakObjectPtr<URammsNewtonPhysicsComponent>& Candidate) {
		return !Candidate.IsValid() || Candidate.Get() == Bridge;
	});
}

int32 URammsNewtonPhysicsSubsystem::GetRegisteredBridgeCount() const
{
	int32 Count = 0;
	for (const TWeakObjectPtr<URammsNewtonPhysicsComponent>& Bridge : RegisteredBridges)
	{
		if (Bridge.IsValid())
		{
			++Count;
		}
	}
	return Count;
}

FRammsNewtonBackendStatus URammsNewtonPhysicsSubsystem::GetBackendStatus() const
{
	return FRammsNewtonPhysicsModule::Get().GetBackendStatus();
}

void URammsNewtonPhysicsSubsystem::StepSimulation(float FixedStepSeconds)
{
	for (const TWeakObjectPtr<URammsNewtonPhysicsComponent>& Bridge : RegisteredBridges)
	{
		if (Bridge.IsValid())
		{
			Bridge->HandleSimulationStep(FixedStepSeconds);
		}
	}

	++StepCounter;
}
