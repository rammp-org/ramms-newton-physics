// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RammsNewtonPhysicsTypes.generated.h"

UENUM(BlueprintType)
enum class ERammsNewtonBackendMode : uint8
{
	StubOnly		UMETA(DisplayName = "Stub Only"),
	HeadersOnly		UMETA(DisplayName = "Headers Detected"),
	PrebuiltLibrary UMETA(DisplayName = "Prebuilt Library Linked"),
};

UENUM(BlueprintType)
enum class ERammsNewtonSimulationRole : uint8
{
	GenericRigidBody UMETA(DisplayName = "Generic Rigid Body"),
	MobilityBase	 UMETA(DisplayName = "Mobility Base"),
	Manipulator		 UMETA(DisplayName = "Manipulator"),
	HybridRobot		 UMETA(DisplayName = "Hybrid Robot"),
};

USTRUCT(BlueprintType)
struct RAMMSNEWTONPHYSICS_API FRammsNewtonBridgeDescription
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	ERammsNewtonSimulationRole SimulationRole = ERammsNewtonSimulationRole::HybridRobot;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	bool bAutoCollectChildPrimitiveComponents = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	bool bAutoCollectSkeletalMeshComponents = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	bool bPushUnrealPosesToSolver = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	bool bPullSolverPosesBackToUnreal = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	FName RootPrimitiveComponentName;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	FName SkeletalMeshComponentName;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	TArray<FName> IncludedPrimitiveComponents;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	TArray<FName> ExcludedPrimitiveComponents;
};

USTRUCT(BlueprintType)
struct RAMMSNEWTONPHYSICS_API FRammsNewtonBackendStatus
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Newton")
	ERammsNewtonBackendMode Mode = ERammsNewtonBackendMode::StubOnly;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Newton")
	bool bRuntimeReady = false;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Newton")
	FString Summary;
};
