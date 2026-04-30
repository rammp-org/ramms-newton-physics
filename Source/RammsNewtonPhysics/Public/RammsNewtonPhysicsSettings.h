// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DeveloperSettings.h"
#include "RammsNewtonPhysicsSettings.generated.h"

UCLASS(Config = Game, DefaultConfig, meta = (DisplayName = "RAMMS Newton Physics"))
class RAMMSNEWTONPHYSICS_API URammsNewtonPhysicsSettings : public UDeveloperSettings
{
	GENERATED_BODY()

public:
	UPROPERTY(Config, EditAnywhere, BlueprintReadOnly, Category = "Runtime", meta = (ClampMin = "1.0"))
	float FixedStepHz = 240.0f;

	UPROPERTY(Config, EditAnywhere, BlueprintReadOnly, Category = "Runtime", meta = (ClampMin = "1"))
	int32 MaxSubstepsPerTick = 4;

	UPROPERTY(Config, EditAnywhere, BlueprintReadOnly, Category = "Runtime")
	bool bEnableSubsystemStepping = true;

	UPROPERTY(Config, EditAnywhere, BlueprintReadOnly, Category = "Runtime")
	bool bLogBackendWarnings = true;

	UPROPERTY(Config, EditAnywhere, BlueprintReadOnly, Category = "Runtime")
	FVector GravityCmPerSecondSquared = FVector(0.0f, 0.0f, -980.0f);
};
