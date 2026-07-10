// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RammsNewtonNativeBackend.h"
#include "Subsystems/WorldSubsystem.h"
#include "RammsNewtonPhysicsTypes.h"
#include "RammsNewtonPhysicsSubsystem.generated.h"

class URammsNewtonPhysicsComponent;

UCLASS()
class RAMMSNEWTONPHYSICS_API URammsNewtonPhysicsSubsystem : public UTickableWorldSubsystem
{
	GENERATED_BODY()

public:
	virtual void	Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void	Deinitialize() override;
	virtual void	Tick(float DeltaTime) override;
	virtual TStatId GetStatId() const override;
	virtual bool	IsTickable() const override { return true; }

	void RegisterBridge(URammsNewtonPhysicsComponent* Bridge);
	void UnregisterBridge(URammsNewtonPhysicsComponent* Bridge);

	UFUNCTION(BlueprintPure, Category = "Ramms|Physics|Newton")
	int32 GetRegisteredBridgeCount() const;

	UFUNCTION(BlueprintPure, Category = "Ramms|Physics|Newton")
	FRammsNewtonBackendStatus GetBackendStatus() const;

	UFUNCTION(BlueprintPure, Category = "Ramms|Physics|Newton")
	FRammsNewtonNativeWorldStatus GetNativeWorldStatus() const;

private:
	void InitializeNativeBackendIfNeeded();
	void StepSimulation(float FixedStepSeconds);

	TArray<TWeakObjectPtr<URammsNewtonPhysicsComponent>> RegisteredBridges;
	FRammsNewtonNativeBackend							 NativeBackend;
	float												 AccumulatedTimeSeconds = 0.0f;
	int64												 StepCounter = 0;
	bool												 bAttemptedNativeBackendInit = false;
	bool												 bLoggedUnavailable = false;
};
