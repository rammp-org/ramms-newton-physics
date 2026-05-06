// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsNewtonArticulatedRobotComponent.h"

DEFINE_LOG_CATEGORY_STATIC(LogRammsNewtonArticulatedRobotComponent, Log, All);

URammsNewtonArticulatedRobotComponent::URammsNewtonArticulatedRobotComponent()
{
	BridgeDescription.SimulationRole = ERammsNewtonSimulationRole::HybridRobot;
}

TArray<FName> URammsNewtonArticulatedRobotComponent::GetConfiguredLinkNames() const
{
	TArray<FName> LinkNames;

	if (bAutoInferLinksFromManagedComponents)
	{
		LinkNames = GetManagedComponentNames();
	}

	for (const FRammsNewtonLinkDescription& Link : RobotDescription.Links)
	{
		if (!Link.LinkName.IsNone())
		{
			LinkNames.AddUnique(Link.LinkName);
		}
	}

	LinkNames.Sort([](const FName& A, const FName& B) {
		return A.LexicalLess(B);
	});
	return LinkNames;
}

TArray<FName> URammsNewtonArticulatedRobotComponent::GetConfiguredJointNames() const
{
	TArray<FName> JointNames;
	for (const FRammsNewtonJointDescription& Joint : RobotDescription.Joints)
	{
		if (!Joint.JointName.IsNone())
		{
			JointNames.Add(Joint.JointName);
		}
	}

	JointNames.Sort([](const FName& A, const FName& B) {
		return A.LexicalLess(B);
	});
	return JointNames;
}

bool URammsNewtonArticulatedRobotComponent::HasValidRobotDescription(FString& OutIssue) const
{
	const TArray<FName> LinkNames = GetConfiguredLinkNames();
	if (LinkNames.Num() == 0)
	{
		OutIssue = TEXT("No Newton links are configured or inferable from the actor.");
		return false;
	}

	for (const FRammsNewtonJointDescription& Joint : RobotDescription.Joints)
	{
		if (Joint.ParentLinkName.IsNone() || Joint.ChildLinkName.IsNone())
		{
			OutIssue = FString::Printf(TEXT("Joint '%s' is missing a parent or child link."), *Joint.JointName.ToString());
			return false;
		}

		if (!LinkNames.Contains(Joint.ParentLinkName))
		{
			OutIssue = FString::Printf(TEXT("Joint '%s' references missing parent link '%s'."), *Joint.JointName.ToString(), *Joint.ParentLinkName.ToString());
			return false;
		}

		if (!LinkNames.Contains(Joint.ChildLinkName))
		{
			OutIssue = FString::Printf(TEXT("Joint '%s' references missing child link '%s'."), *Joint.JointName.ToString(), *Joint.ChildLinkName.ToString());
			return false;
		}
	}

	OutIssue.Reset();
	return true;
}

void URammsNewtonArticulatedRobotComponent::HandleSimulationStep(float FixedStepSeconds)
{
	FString ValidationIssue;
	if (!HasValidRobotDescription(ValidationIssue))
	{
		if (bLogValidationWarnings)
		{
			UE_LOG(LogRammsNewtonArticulatedRobotComponent, Verbose, TEXT("[Newton] %s"), *ValidationIssue);
		}
		return;
	}

	Super::HandleSimulationStep(FixedStepSeconds);

	// Future Newton articulation flow:
	//  1. Create one coupled Newton model for the base, arm, gripper, and manipulated props
	//  2. Map links to UE components / bones and joints to named constraints or URDF joints
	//  3. Step Newton and write back the solved transforms and joint states
}
