// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsNewtonPhysicsComponent.h"

#include "Components/PrimitiveComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "GameFramework/Actor.h"
#include "RammsNewtonPhysicsModule.h"
#include "RammsNewtonPhysicsSubsystem.h"

DEFINE_LOG_CATEGORY_STATIC(LogRammsNewtonPhysicsComponent, Log, All);

URammsNewtonPhysicsComponent::URammsNewtonPhysicsComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void URammsNewtonPhysicsComponent::BeginPlay()
{
	Super::BeginPlay();

	if (bAutoRegisterWithSubsystem)
	{
		RegisterWithSubsystem();
	}
}

void URammsNewtonPhysicsComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	UnregisterFromSubsystem();
	Super::EndPlay(EndPlayReason);
}

bool URammsNewtonPhysicsComponent::RegisterWithSubsystem()
{
	if (bRegisteredWithSubsystem)
	{
		return true;
	}

	UWorld* World = GetWorld();
	if (!World)
	{
		return false;
	}

	URammsNewtonPhysicsSubsystem* Subsystem = World->GetSubsystem<URammsNewtonPhysicsSubsystem>();
	if (!Subsystem)
	{
		return false;
	}

	Subsystem->RegisterBridge(this);
	bRegisteredWithSubsystem = true;

	if (bLogRegistration)
	{
		UE_LOG(LogRammsNewtonPhysicsComponent, Log, TEXT("[Newton] Registered '%s' with %d managed components"),
			*GetNameSafe(GetOwner()),
			GetManagedComponentNames().Num());
	}

	return true;
}

void URammsNewtonPhysicsComponent::UnregisterFromSubsystem()
{
	if (!bRegisteredWithSubsystem)
	{
		return;
	}

	if (UWorld* World = GetWorld())
	{
		if (URammsNewtonPhysicsSubsystem* Subsystem = World->GetSubsystem<URammsNewtonPhysicsSubsystem>())
		{
			Subsystem->UnregisterBridge(this);
		}
	}

	bRegisteredWithSubsystem = false;
}

FRammsNewtonBackendStatus URammsNewtonPhysicsComponent::GetBackendStatus() const
{
	return FRammsNewtonPhysicsModule::Get().GetBackendStatus();
}

TArray<FName> URammsNewtonPhysicsComponent::GetManagedComponentNames() const
{
	TSet<FName> ManagedNames;

	for (const FName IncludedName : BridgeDescription.IncludedPrimitiveComponents)
	{
		if (ShouldIncludeComponentName(IncludedName))
		{
			ManagedNames.Add(IncludedName);
		}
	}

	if (const AActor* Owner = GetOwner())
	{
		if (BridgeDescription.bAutoCollectChildPrimitiveComponents)
		{
			TInlineComponentArray<UPrimitiveComponent*> PrimitiveComponents;
			Owner->GetComponents(PrimitiveComponents);
			for (const UPrimitiveComponent* PrimitiveComponent : PrimitiveComponents)
			{
				if (PrimitiveComponent && ShouldIncludeComponentName(PrimitiveComponent->GetFName()))
				{
					ManagedNames.Add(PrimitiveComponent->GetFName());
				}
			}
		}

		if (BridgeDescription.bAutoCollectSkeletalMeshComponents)
		{
			TInlineComponentArray<USkeletalMeshComponent*> SkeletalMeshComponents;
			Owner->GetComponents(SkeletalMeshComponents);
			for (const USkeletalMeshComponent* SkeletalMeshComponent : SkeletalMeshComponents)
			{
				if (SkeletalMeshComponent && ShouldIncludeComponentName(SkeletalMeshComponent->GetFName()))
				{
					ManagedNames.Add(SkeletalMeshComponent->GetFName());
				}
			}
		}
	}

	if (ShouldIncludeComponentName(BridgeDescription.RootPrimitiveComponentName))
	{
		ManagedNames.Add(BridgeDescription.RootPrimitiveComponentName);
	}
	if (ShouldIncludeComponentName(BridgeDescription.SkeletalMeshComponentName))
	{
		ManagedNames.Add(BridgeDescription.SkeletalMeshComponentName);
	}

	TArray<FName> Result = ManagedNames.Array();
	Result.Sort([](const FName& A, const FName& B) {
		return A.LexicalLess(B);
	});
	return Result;
}

void URammsNewtonPhysicsComponent::SetNativeRegistrationState(bool bInNativeRegistered, int32 InNativeBodyCount, const FString& InSummary)
{
	bNativeRuntimeRegistered = bInNativeRegistered;
	NativeRegisteredBodyCount = InNativeBodyCount;
	NativeRegistrationSummary = InSummary;
}

void URammsNewtonPhysicsComponent::HandleSimulationStep(float FixedStepSeconds)
{
	(void)FixedStepSeconds;
	// Placeholder for the eventual Newton sync pipeline:
	//  1. Push selected actor/component state into Newton
	//  2. Step the Newton world / articulation graph
	//  3. Pull transforms, joint state, and contact data back into UE
}

bool URammsNewtonPhysicsComponent::ShouldIncludeComponentName(FName ComponentName) const
{
	if (ComponentName.IsNone())
	{
		return false;
	}

	return !BridgeDescription.ExcludedPrimitiveComponents.Contains(ComponentName);
}

void URammsNewtonPhysicsComponent::GetManagedPrimitiveComponents(TArray<UPrimitiveComponent*>& OutPrimitiveComponents) const
{
	OutPrimitiveComponents.Reset();

	const AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}

	const TArray<FName> ManagedNames = GetManagedComponentNames();
	if (ManagedNames.Num() == 0)
	{
		return;
	}

	TSet<FName> ManagedNameSet;
	ManagedNameSet.Append(ManagedNames);

	TInlineComponentArray<UPrimitiveComponent*> PrimitiveComponents;
	Owner->GetComponents(PrimitiveComponents);

	TSet<FName> SeenNames;
	for (UPrimitiveComponent* PrimitiveComponent : PrimitiveComponents)
	{
		if (!PrimitiveComponent)
		{
			continue;
		}

		const FName ComponentName = PrimitiveComponent->GetFName();
		if (!ManagedNameSet.Contains(ComponentName) || SeenNames.Contains(ComponentName))
		{
			continue;
		}

		SeenNames.Add(ComponentName);
		OutPrimitiveComponents.Add(PrimitiveComponent);
	}
}
