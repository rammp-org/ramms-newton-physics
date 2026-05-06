// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RammsNewtonPhysicsComponent.h"
#include "RammsNewtonArticulatedRobotComponent.generated.h"

UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSNEWTONPHYSICS_API URammsNewtonArticulatedRobotComponent : public URammsNewtonPhysicsComponent
{
	GENERATED_BODY()

public:
	URammsNewtonArticulatedRobotComponent();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton|Robot")
	FRammsNewtonRobotDescription RobotDescription;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton|Robot")
	bool bAutoInferLinksFromManagedComponents = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton|Robot")
	bool bLogValidationWarnings = true;

	UFUNCTION(BlueprintPure, Category = "Ramms|Physics|Newton")
	TArray<FName> GetConfiguredLinkNames() const;

	UFUNCTION(BlueprintPure, Category = "Ramms|Physics|Newton")
	TArray<FName> GetConfiguredJointNames() const;

	UFUNCTION(BlueprintPure, Category = "Ramms|Physics|Newton")
	bool HasValidRobotDescription(FString& OutIssue) const;

	virtual void HandleSimulationStep(float FixedStepSeconds) override;
};
