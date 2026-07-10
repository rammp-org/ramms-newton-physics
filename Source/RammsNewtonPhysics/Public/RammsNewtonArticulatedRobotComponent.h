// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RammsNewtonPhysicsComponent.h"
#include "RammsNewtonArticulatedRobotComponent.generated.h"

class UPoseableMeshComponent;
class URammsSkeletalPoseComponent;
class USkeletalMeshComponent;

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

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton|Robot|Inference")
	bool bAutoInferArmFromKinovaController = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton|Robot|Inference")
	bool bAutoInferGripperFromController = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton|Robot|Inference")
	FName KinovaControllerComponentName;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton|Robot|Inference")
	FName GripperControllerComponentName;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton|Robot|Runtime")
	bool bUseEffectiveRobotDescriptionForManagedComponents = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton|Robot|Runtime")
	bool bEnableSkeletalPoseWriteback = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton|Robot|Runtime")
	bool bAutoCreatePoseableMirrors = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton|Robot|Runtime")
	bool bHideSourceSkeletalMeshesWhenUsingPoseMirrors = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton|Robot")
	bool bLogValidationWarnings = true;

	UFUNCTION(BlueprintPure, Category = "Ramms|Physics|Newton")
	TArray<FName> GetConfiguredLinkNames() const;

	UFUNCTION(BlueprintPure, Category = "Ramms|Physics|Newton")
	TArray<FName> GetConfiguredJointNames() const;

	UFUNCTION(BlueprintPure, Category = "Ramms|Physics|Newton")
	FRammsNewtonRobotDescription GetEffectiveRobotDescription() const;

	UFUNCTION(BlueprintPure, Category = "Ramms|Physics|Newton")
	FString GetEffectiveRobotExportJson() const;

	UFUNCTION(BlueprintPure, Category = "Ramms|Physics|Newton")
	FString GetCurrentJointControlJson() const;

	UFUNCTION(BlueprintPure, Category = "Ramms|Physics|Newton")
	bool HasValidRobotDescription(FString& OutIssue) const;

	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Ramms|Physics|Newton")
	void AutoPopulateRobotDescription();

	UFUNCTION(BlueprintCallable, Category = "Ramms|Physics|Newton")
	void AutoPopulateRobotDescriptionFromControllers(bool bOverwriteExisting = true);

	virtual TArray<FName> GetManagedComponentNames() const override;
	virtual void		  GetManagedPrimitiveComponents(TArray<UPrimitiveComponent*>& OutPrimitiveComponents) const override;
	virtual void		  HandleSimulationStep(float FixedStepSeconds) override;
	virtual void		  ApplySolvedJointStates(const TMap<FName, float>& JointPositions, const TMap<FName, float>& JointVelocities) override;

private:
	void AppendInferredRobotDescription(FRammsNewtonRobotDescription& InOutDescription) const;
	void EnsureSkeletalPoseWritebackSetup(const FRammsNewtonRobotDescription& EffectiveDescription);

	UPROPERTY(Transient)
	TObjectPtr<URammsSkeletalPoseComponent> RuntimeSkeletalPoseComponent = nullptr;

	UPROPERTY(Transient)
	TMap<FName, TObjectPtr<UPoseableMeshComponent>> RuntimePoseMirrorsBySourceMeshName;

	UPROPERTY(Transient)
	TMap<FName, TObjectPtr<USkeletalMeshComponent>> RuntimeSourceSkeletalMeshes;

	TMap<FName, ECollisionEnabled::Type> RuntimeSourceCollisionModes;
	TMap<FName, bool>					 RuntimeSourceGenerateOverlapEvents;
	TMap<FName, bool>					 RuntimeSourceVisibility;
	TMap<FName, bool>					 RuntimeSourceHiddenInGame;
};
