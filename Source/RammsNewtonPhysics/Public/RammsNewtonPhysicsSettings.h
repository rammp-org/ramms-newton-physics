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
	int32 MaxSubstepsPerTick = 1;

	UPROPERTY(Config, EditAnywhere, BlueprintReadOnly, Category = "Runtime")
	bool bEnableSubsystemStepping = true;

	UPROPERTY(Config, EditAnywhere, BlueprintReadOnly, Category = "Runtime")
	bool bLogBackendWarnings = true;

	UPROPERTY(Config, EditAnywhere, BlueprintReadOnly, Category = "Runtime")
	FVector GravityCmPerSecondSquared = FVector(0.0f, 0.0f, -980.0f);

	UPROPERTY(Config, EditAnywhere, BlueprintReadOnly, Category = "Python Bridge")
	bool bPreferPythonWorkerBridge = true;

	UPROPERTY(Config, EditAnywhere, BlueprintReadOnly, Category = "Python Bridge")
	bool bAllowBuiltInAdapterFallback = true;

	UPROPERTY(Config, EditAnywhere, BlueprintReadOnly, Category = "Python Bridge")
	FString PythonExecutablePath;

	UPROPERTY(Config, EditAnywhere, BlueprintReadOnly, Category = "Python Bridge")
	FString PythonWorkerScriptPath;

	UPROPERTY(Config, EditAnywhere, BlueprintReadOnly, Category = "Python Bridge")
	FString PythonExtraArguments;

	UPROPERTY(Config, EditAnywhere, BlueprintReadOnly, Category = "Python Bridge", meta = (ClampMin = "1.0"))
	float PythonRequestTimeoutSeconds = 5.0f;
};
