// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RammsNewtonPhysicsTypes.h"
#include "RammsNewtonPhysicsComponent.generated.h"

UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSNEWTONPHYSICS_API URammsNewtonPhysicsComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	URammsNewtonPhysicsComponent();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton|Configuration")
	bool bAutoRegisterWithSubsystem = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton|Configuration")
	FRammsNewtonBridgeDescription BridgeDescription;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton|Debug")
	bool bLogRegistration = true;

	UPROPERTY(BlueprintReadOnly, Category = "Newton|State")
	bool bRegisteredWithSubsystem = false;

	UFUNCTION(BlueprintCallable, Category = "Ramms|Physics|Newton")
	bool RegisterWithSubsystem();

	UFUNCTION(BlueprintCallable, Category = "Ramms|Physics|Newton")
	void UnregisterFromSubsystem();

	UFUNCTION(BlueprintPure, Category = "Ramms|Physics|Newton")
	FRammsNewtonBackendStatus GetBackendStatus() const;

	UFUNCTION(BlueprintPure, Category = "Ramms|Physics|Newton")
	TArray<FName> GetManagedComponentNames() const;

	void HandleSimulationStep(float FixedStepSeconds);

private:
	bool ShouldIncludeComponentName(FName ComponentName) const;
};
