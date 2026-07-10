// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsNewtonArticulatedRobotComponent.h"

#include "Components/BoxComponent.h"
#include "Components/CapsuleComponent.h"
#include "Components/MeshComponent.h"
#include "Components/PoseableMeshComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/SphereComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Dom/JsonObject.h"
#include "GripperControllerComponent.h"
#include "KinovaGen3ControllerComponent.h"
#include "Materials/MaterialInterface.h"
#include "RammsSkeletalPoseComponent.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"

#include "Engine/SkeletalMesh.h"
#include "Engine/StaticMesh.h"
#include "PhysicsEngine/ConstraintInstance.h"
#include "ReferenceSkeleton.h"

DEFINE_LOG_CATEGORY_STATIC(LogRammsNewtonArticulatedRobotComponent, Log, All);

namespace
{
	constexpr double CentimetersToMeters = 0.01;

	template <typename TComponent>
	TComponent* FindNamedComponent(const AActor* Owner, const FName ExactName)
	{
		if (!Owner || ExactName.IsNone())
		{
			return nullptr;
		}

		TInlineComponentArray<TComponent*> Components;
		Owner->GetComponents(Components);
		for (TComponent* Component : Components)
		{
			if (Component && Component->GetFName() == ExactName)
			{
				return Component;
			}
		}

		return nullptr;
	}

	bool ShouldAutoInferManagedLinkFromComponent(const AActor* Owner, const FName ComponentName)
	{
		if (!Owner || ComponentName.IsNone())
		{
			return false;
		}

		if (FindNamedComponent<USkeletalMeshComponent>(Owner, ComponentName) != nullptr
			|| FindNamedComponent<UPoseableMeshComponent>(Owner, ComponentName) != nullptr)
		{
			return false;
		}

		const UPrimitiveComponent* PrimitiveComponent = FindNamedComponent<UPrimitiveComponent>(Owner, ComponentName);
		if (!PrimitiveComponent)
		{
			return false;
		}

		const ECollisionEnabled::Type CollisionEnabled = PrimitiveComponent->GetCollisionEnabled();
		return CollisionEnabled == ECollisionEnabled::PhysicsOnly
			|| CollisionEnabled == ECollisionEnabled::QueryAndPhysics;
	}

	template <typename TComponent>
	TComponent* FindNamedOrFirstComponent(const AActor* Owner, const FName PreferredName)
	{
		if (!Owner)
		{
			return nullptr;
		}

		TInlineComponentArray<TComponent*> Components;
		Owner->GetComponents(Components);
		if (PreferredName != NAME_None)
		{
			for (TComponent* Component : Components)
			{
				if (Component && Component->GetFName() == PreferredName)
				{
					return Component;
				}
			}
		}

		for (TComponent* Component : Components)
		{
			if (Component)
			{
				return Component;
			}
		}

		return nullptr;
	}

	// UGripperControllerComponent no longer exposes getters for its configuration
	// (the arm/gripper integration is pending a RammsCore API rework). Until then,
	// read the still-present protected UPROPERTY fields via reflection so the
	// exported robot description keeps carrying the configured values.
	template <typename TValue>
	TValue GetGripperConfigValue(const UGripperControllerComponent& Gripper, const FName PropertyName, const TValue& DefaultValue)
	{
		const FProperty* Property = UGripperControllerComponent::StaticClass()->FindPropertyByName(PropertyName);
		if (Property != nullptr && Property->GetSize() == sizeof(TValue))
		{
			return *Property->ContainerPtrToValuePtr<TValue>(&Gripper);
		}
		return DefaultValue;
	}

	FName GetGripperMeshComponentName(const UGripperControllerComponent& Gripper)
	{
		return GetGripperConfigValue<FName>(Gripper, TEXT("GripperMeshName"), NAME_None);
	}

	float GetGripperOpenAngle(const UGripperControllerComponent& Gripper)
	{
		return GetGripperConfigValue<float>(Gripper, TEXT("OpenAngle"), 0.0f);
	}

	float GetGripperClosedAngle(const UGripperControllerComponent& Gripper)
	{
		return GetGripperConfigValue<float>(Gripper, TEXT("ClosedAngle"), 0.0f);
	}

	TArray<FAngularMotorConfig> GetGripperFingerMotors(const UGripperControllerComponent& Gripper)
	{
		return {
			GetGripperConfigValue<FAngularMotorConfig>(Gripper, TEXT("Finger1Motor"), FAngularMotorConfig()),
			GetGripperConfigValue<FAngularMotorConfig>(Gripper, TEXT("Finger2Motor"), FAngularMotorConfig()),
		};
	}

	FVector ToNewtonAxis(const EConstraintAxis Axis)
	{
		switch (Axis)
		{
			case EConstraintAxis::Swing1:
				return FVector::UpVector;
			case EConstraintAxis::Swing2:
				return FVector::RightVector;
			case EConstraintAxis::Twist:
			default:
				return FVector::ForwardVector;
		}
	}

	FVector ToNewtonAxis(const EMotorAxis Axis)
	{
		switch (Axis)
		{
			case EMotorAxis::Y:
				return FVector::RightVector;
			case EMotorAxis::Z:
				return FVector::UpVector;
			case EMotorAxis::X:
			default:
				return FVector::ForwardVector;
		}
	}

	void FindChildBoneAndFrame(const FConstraintInstance& ConstraintInstance, const FReferenceSkeleton& ReferenceSkeleton,
		FName& OutChildBone, EConstraintFrame::Type& OutChildFrame)
	{
		OutChildBone = ConstraintInstance.ConstraintBone1;
		OutChildFrame = EConstraintFrame::Frame1;

		const int32 Bone1Index = ReferenceSkeleton.FindBoneIndex(ConstraintInstance.ConstraintBone1);
		const int32 Bone2Index = ReferenceSkeleton.FindBoneIndex(ConstraintInstance.ConstraintBone2);
		if (Bone1Index == INDEX_NONE || Bone2Index == INDEX_NONE)
		{
			return;
		}

		for (int32 Index = ReferenceSkeleton.GetParentIndex(Bone1Index); Index != INDEX_NONE; Index = ReferenceSkeleton.GetParentIndex(Index))
		{
			if (Index == Bone2Index)
			{
				OutChildBone = ConstraintInstance.ConstraintBone1;
				OutChildFrame = EConstraintFrame::Frame1;
				return;
			}
		}

		for (int32 Index = ReferenceSkeleton.GetParentIndex(Bone2Index); Index != INDEX_NONE; Index = ReferenceSkeleton.GetParentIndex(Index))
		{
			if (Index == Bone1Index)
			{
				OutChildBone = ConstraintInstance.ConstraintBone2;
				OutChildFrame = EConstraintFrame::Frame2;
				return;
			}
		}
	}

	struct FSkeletalConstraintJointInfo
	{
		FName	   ParentBoneName;
		FName	   ChildBoneName;
		FVector	   PoseAxis = FVector::ForwardVector;
		FVector	   AxisInParentFrame = FVector::ForwardVector;
		FTransform ParentAnchorTransform = FTransform::Identity;
		FTransform ChildAnchorTransform = FTransform::Identity;
		bool	   bValid = false;
	};

	template <typename TAxis>
	bool TryBuildConstraintJointInfo(const USkeletalMeshComponent* SkeletalMeshComponent, const FName ConstraintName, const TAxis AxisKind,
		const float DirectionSign, FSkeletalConstraintJointInfo& OutInfo)
	{
		OutInfo = {};
		if (!SkeletalMeshComponent || ConstraintName.IsNone())
		{
			return false;
		}

		FConstraintInstance* ConstraintInstance = const_cast<USkeletalMeshComponent*>(SkeletalMeshComponent)->FindConstraintInstance(ConstraintName);
		const USkeletalMesh* SkeletalMeshAsset = SkeletalMeshComponent->GetSkeletalMeshAsset();
		if (!ConstraintInstance || !SkeletalMeshAsset)
		{
			return false;
		}

		const FReferenceSkeleton& ReferenceSkeleton = SkeletalMeshAsset->GetRefSkeleton();
		FName					  ChildBoneName;
		EConstraintFrame::Type	  ChildFrame = EConstraintFrame::Frame1;
		FindChildBoneAndFrame(*ConstraintInstance, ReferenceSkeleton, ChildBoneName, ChildFrame);
		if (ChildBoneName.IsNone())
		{
			return false;
		}

		const EConstraintFrame::Type ParentFrame =
			ChildFrame == EConstraintFrame::Frame1 ? EConstraintFrame::Frame2 : EConstraintFrame::Frame1;
		const FName ParentBoneName =
			ChildFrame == EConstraintFrame::Frame1 ? ConstraintInstance->ConstraintBone2 : ConstraintInstance->ConstraintBone1;
		const FVector CanonicalAxis = ToNewtonAxis(AxisKind).GetSafeNormal() * (DirectionSign >= 0.0f ? 1.0f : -1.0f);

		OutInfo.ParentBoneName = ParentBoneName;
		OutInfo.ChildBoneName = ChildBoneName;
		OutInfo.PoseAxis = ConstraintInstance->GetRefFrame(ChildFrame).GetRotation().RotateVector(CanonicalAxis);
		OutInfo.AxisInParentFrame = CanonicalAxis;
		OutInfo.ParentAnchorTransform = ConstraintInstance->GetRefFrame(ParentFrame);
		OutInfo.ChildAnchorTransform = ConstraintInstance->GetRefFrame(ChildFrame);
		OutInfo.bValid = true;
		return true;
	}

	ERammsNewtonJointDriveMode ToNewtonDriveMode(const EJointControlMode Mode)
	{
		switch (Mode)
		{
			case EJointControlMode::VelocityControl:
				return ERammsNewtonJointDriveMode::VelocityControl;
			case EJointControlMode::TorqueControl:
				return ERammsNewtonJointDriveMode::TorqueControl;
			case EJointControlMode::PositionControl:
			default:
				return ERammsNewtonJointDriveMode::PositionControl;
		}
	}

	void AddOrMergeLink(TArray<FRammsNewtonLinkDescription>& Links, const FRammsNewtonLinkDescription& Candidate)
	{
		if (Candidate.LinkName.IsNone())
		{
			return;
		}

		if (FRammsNewtonLinkDescription* ExistingLink = Links.FindByPredicate([&Candidate](const FRammsNewtonLinkDescription& Link) {
				return Link.LinkName == Candidate.LinkName;
			}))
		{
			if (ExistingLink->ComponentName.IsNone())
			{
				ExistingLink->ComponentName = Candidate.ComponentName;
			}
			if (ExistingLink->ParentLinkName.IsNone())
			{
				ExistingLink->ParentLinkName = Candidate.ParentLinkName;
			}
			return;
		}

		Links.Add(Candidate);
	}

	void AddJointIfMissing(TArray<FRammsNewtonJointDescription>& Joints, const FRammsNewtonJointDescription& Candidate)
	{
		if (Candidate.JointName.IsNone())
		{
			return;
		}

		if (Joints.ContainsByPredicate([&Candidate](const FRammsNewtonJointDescription& Joint) {
				return Joint.JointName == Candidate.JointName;
			}))
		{
			return;
		}

		Joints.Add(Candidate);
	}

	FName ResolveSkeletalMeshName(const AActor* Owner, const FName PreferredName)
	{
		if (PreferredName != NAME_None)
		{
			return PreferredName;
		}

		if (const USkeletalMeshComponent* SkeletalMeshComponent = FindNamedOrFirstComponent<USkeletalMeshComponent>(Owner, NAME_None))
		{
			return SkeletalMeshComponent->GetFName();
		}

		return NAME_None;
	}

	FString ToJsonJointType(const ERammsNewtonJointType JointType)
	{
		switch (JointType)
		{
			case ERammsNewtonJointType::Fixed:
				return TEXT("fixed");
			case ERammsNewtonJointType::Prismatic:
				return TEXT("prismatic");
			case ERammsNewtonJointType::Spherical:
				return TEXT("spherical");
			case ERammsNewtonJointType::Revolute:
			default:
				return TEXT("revolute");
		}
	}

	FString ToJsonDriveMode(const ERammsNewtonJointDriveMode DriveMode)
	{
		switch (DriveMode)
		{
			case ERammsNewtonJointDriveMode::Passive:
				return TEXT("passive");
			case ERammsNewtonJointDriveMode::VelocityControl:
				return TEXT("velocity_control");
			case ERammsNewtonJointDriveMode::TorqueControl:
				return TEXT("torque_control");
			case ERammsNewtonJointDriveMode::PositionControl:
			default:
				return TEXT("position_control");
		}
	}

	FString ToJsonControlMode(const EJointControlMode ControlMode)
	{
		switch (ControlMode)
		{
			case EJointControlMode::VelocityControl:
				return TEXT("velocity_control");
			case EJointControlMode::TorqueControl:
				return TEXT("torque_control");
			case EJointControlMode::PositionControl:
			default:
				return TEXT("position_control");
		}
	}

	FString ToJsonConstraintAxis(const EConstraintAxis Axis)
	{
		switch (Axis)
		{
			case EConstraintAxis::Swing1:
				return TEXT("swing1");
			case EConstraintAxis::Swing2:
				return TEXT("swing2");
			case EConstraintAxis::Twist:
			default:
				return TEXT("twist");
		}
	}

	FString ToJsonMotorAxis(const EMotorAxis Axis)
	{
		switch (Axis)
		{
			case EMotorAxis::Y:
				return TEXT("y");
			case EMotorAxis::Z:
				return TEXT("z");
			case EMotorAxis::X:
			default:
				return TEXT("x");
		}
	}

	TEnumAsByte<EAxis::Type> ToPoseAxis(const FVector& Axis, bool& bOutInvertDirection)
	{
		const FVector AbsAxis = Axis.GetAbs();
		bOutInvertDirection = false;

		if (AbsAxis.X >= AbsAxis.Y && AbsAxis.X >= AbsAxis.Z)
		{
			bOutInvertDirection = Axis.X < 0.0f;
			return EAxis::X;
		}
		if (AbsAxis.Y >= AbsAxis.Z)
		{
			bOutInvertDirection = Axis.Y < 0.0f;
			return EAxis::Y;
		}

		bOutInvertDirection = Axis.Z < 0.0f;
		return EAxis::Z;
	}

	TArray<TSharedPtr<FJsonValue>> MakeJsonNumberArray(std::initializer_list<double> Values)
	{
		TArray<TSharedPtr<FJsonValue>> Result;
		Result.Reserve(static_cast<int32>(Values.size()));
		for (const double Value : Values)
		{
			Result.Add(MakeShared<FJsonValueNumber>(Value));
		}
		return Result;
	}

	TArray<TSharedPtr<FJsonValue>> MakeJsonStringArray(const TArray<FString>& Values)
	{
		TArray<TSharedPtr<FJsonValue>> Result;
		Result.Reserve(Values.Num());
		for (const FString& Value : Values)
		{
			Result.Add(MakeShared<FJsonValueString>(Value));
		}
		return Result;
	}

	TSharedRef<FJsonObject> MakeTransformJson(const FTransform& Transform)
	{
		const FVector TranslationMeters = Transform.GetTranslation() * CentimetersToMeters;
		const FQuat	  Rotation = Transform.GetRotation();
		const FVector Scale = Transform.GetScale3D();

		TSharedRef<FJsonObject> Json = MakeShared<FJsonObject>();
		Json->SetArrayField(TEXT("translation_m"), MakeJsonNumberArray({
													   TranslationMeters.X,
													   TranslationMeters.Y,
													   TranslationMeters.Z,
												   }));
		Json->SetArrayField(TEXT("rotation_xyzw"), MakeJsonNumberArray({
													   Rotation.X,
													   Rotation.Y,
													   Rotation.Z,
													   Rotation.W,
												   }));
		Json->SetArrayField(TEXT("scale"), MakeJsonNumberArray({
											   Scale.X,
											   Scale.Y,
											   Scale.Z,
										   }));
		return Json;
	}

	bool TryResolveLinkWorldTransform(const AActor* Owner, const FRammsNewtonLinkDescription& Link, FTransform& OutWorldTransform, FString& OutSource)
	{
		if (!Owner)
		{
			return false;
		}

		auto TryResolveFromSkeletalMesh = [&](const USkeletalMeshComponent* SkeletalMeshComponent, const bool bAllowComponentFallback) -> bool {
			if (!SkeletalMeshComponent)
			{
				return false;
			}

			if (!Link.LinkName.IsNone())
			{
				const int32 BoneIndex = SkeletalMeshComponent->GetBoneIndex(Link.LinkName);
				if (BoneIndex != INDEX_NONE)
				{
					OutWorldTransform = SkeletalMeshComponent->GetBoneTransform(BoneIndex);
					OutSource = FString::Printf(TEXT("bone:%s"), *Link.LinkName.ToString());
					return true;
				}
			}

			if (!bAllowComponentFallback)
			{
				return false;
			}

			OutWorldTransform = SkeletalMeshComponent->GetComponentTransform();
			OutSource = FString::Printf(TEXT("component:%s"), *SkeletalMeshComponent->GetFName().ToString());
			return true;
		};

		if (Link.ComponentName != NAME_None)
		{
			if (const USkeletalMeshComponent* SkeletalMeshComponent = FindNamedOrFirstComponent<USkeletalMeshComponent>(Owner, Link.ComponentName))
			{
				if (TryResolveFromSkeletalMesh(SkeletalMeshComponent, true))
				{
					return true;
				}
			}

			if (const USceneComponent* SceneComponent = FindNamedOrFirstComponent<USceneComponent>(Owner, Link.ComponentName))
			{
				OutWorldTransform = SceneComponent->GetComponentTransform();
				OutSource = FString::Printf(TEXT("component:%s"), *SceneComponent->GetFName().ToString());
				return true;
			}
		}

		if (Link.LinkName != NAME_None)
		{
			if (const USceneComponent* SceneComponent = FindNamedOrFirstComponent<USceneComponent>(Owner, Link.LinkName))
			{
				OutWorldTransform = SceneComponent->GetComponentTransform();
				OutSource = FString::Printf(TEXT("component:%s"), *SceneComponent->GetFName().ToString());
				return true;
			}

			TInlineComponentArray<USkeletalMeshComponent*> SkeletalMeshComponents;
			Owner->GetComponents(SkeletalMeshComponents);
			for (const USkeletalMeshComponent* SkeletalMeshComponent : SkeletalMeshComponents)
			{
				if (TryResolveFromSkeletalMesh(SkeletalMeshComponent, false))
				{
					return true;
				}
			}
		}

		OutWorldTransform = Owner->GetActorTransform();
		OutSource = TEXT("actor");
		return false;
	}

	const UPrimitiveComponent* ResolvePrimitiveForLink(const AActor* Owner, const FRammsNewtonLinkDescription& Link)
	{
		if (!Owner)
		{
			return nullptr;
		}

		if (Link.ComponentName != NAME_None)
		{
			if (const UPrimitiveComponent* PrimitiveComponent = FindNamedOrFirstComponent<UPrimitiveComponent>(Owner, Link.ComponentName))
			{
				return PrimitiveComponent;
			}
		}

		if (Link.LinkName != NAME_None)
		{
			if (const UPrimitiveComponent* PrimitiveComponent = FindNamedOrFirstComponent<UPrimitiveComponent>(Owner, Link.LinkName))
			{
				return PrimitiveComponent;
			}
		}

		return nullptr;
	}

	TSharedRef<FJsonObject> MakePrimitiveMetadataJson(const UPrimitiveComponent* PrimitiveComponent)
	{
		TSharedRef<FJsonObject> Json = MakeShared<FJsonObject>();
		if (!PrimitiveComponent)
		{
			Json->SetStringField(TEXT("type"), TEXT("none"));
			return Json;
		}

		Json->SetStringField(TEXT("component_class"), PrimitiveComponent->GetClass()->GetName());
		Json->SetStringField(TEXT("component_name"), PrimitiveComponent->GetFName().ToString());

		if (const UBoxComponent* BoxComponent = Cast<UBoxComponent>(PrimitiveComponent))
		{
			const FVector ExtentMeters = BoxComponent->GetScaledBoxExtent() * 2.0 * CentimetersToMeters;
			Json->SetStringField(TEXT("type"), TEXT("box"));
			Json->SetArrayField(TEXT("size_m"), MakeJsonNumberArray({
													ExtentMeters.X,
													ExtentMeters.Y,
													ExtentMeters.Z,
												}));
		}
		else if (const USphereComponent* SphereComponent = Cast<USphereComponent>(PrimitiveComponent))
		{
			Json->SetStringField(TEXT("type"), TEXT("sphere"));
			Json->SetNumberField(TEXT("radius_m"), SphereComponent->GetScaledSphereRadius() * CentimetersToMeters);
		}
		else if (const UCapsuleComponent* CapsuleComponent = Cast<UCapsuleComponent>(PrimitiveComponent))
		{
			const double RadiusMeters = CapsuleComponent->GetScaledCapsuleRadius() * CentimetersToMeters;
			const double HalfHeightMeters = CapsuleComponent->GetScaledCapsuleHalfHeight() * CentimetersToMeters;
			Json->SetStringField(TEXT("type"), TEXT("capsule"));
			Json->SetNumberField(TEXT("radius_m"), RadiusMeters);
			Json->SetNumberField(TEXT("cylinder_height_m"), FMath::Max(0.0, (HalfHeightMeters * 2.0) - (RadiusMeters * 2.0)));
		}
		else if (const UStaticMeshComponent* StaticMeshComponent = Cast<UStaticMeshComponent>(PrimitiveComponent))
		{
			Json->SetStringField(TEXT("type"), TEXT("static_mesh"));
			if (const UStaticMesh* StaticMesh = StaticMeshComponent->GetStaticMesh())
			{
				Json->SetStringField(TEXT("asset_path"), StaticMesh->GetPathName());
			}
		}
		else
		{
			Json->SetStringField(TEXT("type"), TEXT("primitive"));
		}

		if (const UMeshComponent* MeshComponent = Cast<UMeshComponent>(PrimitiveComponent))
		{
			TArray<FString> MaterialPaths;
			for (int32 MaterialIndex = 0; MaterialIndex < MeshComponent->GetNumMaterials(); ++MaterialIndex)
			{
				if (const UMaterialInterface* Material = MeshComponent->GetMaterial(MaterialIndex))
				{
					MaterialPaths.Add(Material->GetPathName());
				}
			}

			if (MaterialPaths.Num() > 0)
			{
				Json->SetArrayField(TEXT("material_paths"), MakeJsonStringArray(MaterialPaths));
			}

			if (const UStaticMeshComponent* StaticMeshComponent = Cast<UStaticMeshComponent>(MeshComponent))
			{
				if (const UStaticMesh* StaticMesh = StaticMeshComponent->GetStaticMesh())
				{
					Json->SetStringField(TEXT("render_asset_path"), StaticMesh->GetPathName());
				}
			}
			else if (const USkeletalMeshComponent* SkeletalMeshComponent = Cast<USkeletalMeshComponent>(MeshComponent))
			{
				if (const USkeletalMesh* SkeletalMesh = SkeletalMeshComponent->GetSkeletalMeshAsset())
				{
					Json->SetStringField(TEXT("render_asset_path"), SkeletalMesh->GetPathName());
				}
			}
		}

		return Json;
	}
} // namespace

URammsNewtonArticulatedRobotComponent::URammsNewtonArticulatedRobotComponent()
{
	bAutoRegisterWithSubsystem = false;
	BridgeDescription.SimulationRole = ERammsNewtonSimulationRole::HybridRobot;
}

FRammsNewtonRobotDescription URammsNewtonArticulatedRobotComponent::GetEffectiveRobotDescription() const
{
	FRammsNewtonRobotDescription EffectiveDescription = RobotDescription;
	if (EffectiveDescription.RobotName.IsNone())
	{
		if (const AActor* Owner = GetOwner())
		{
			EffectiveDescription.RobotName = Owner->GetFName();
		}
	}

	if (bAutoInferLinksFromManagedComponents)
	{
		for (const FName ManagedComponentName : GetManagedComponentNamesFromBridgeDescription())
		{
			FRammsNewtonLinkDescription Link;
			Link.LinkName = ManagedComponentName;
			Link.ComponentName = ManagedComponentName;
			AddOrMergeLink(EffectiveDescription.Links, Link);
		}
	}

	AppendInferredRobotDescription(EffectiveDescription);
	return EffectiveDescription;
}

TArray<FName> URammsNewtonArticulatedRobotComponent::GetConfiguredLinkNames() const
{
	TArray<FName>					   LinkNames;
	const FRammsNewtonRobotDescription EffectiveDescription = GetEffectiveRobotDescription();
	for (const FRammsNewtonLinkDescription& Link : EffectiveDescription.Links)
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
	TArray<FName>					   JointNames;
	const FRammsNewtonRobotDescription EffectiveDescription = GetEffectiveRobotDescription();
	for (const FRammsNewtonJointDescription& Joint : EffectiveDescription.Joints)
	{
		if (!Joint.JointName.IsNone())
		{
			JointNames.AddUnique(Joint.JointName);
		}
	}

	JointNames.Sort([](const FName& A, const FName& B) {
		return A.LexicalLess(B);
	});
	return JointNames;
}

TArray<FName> URammsNewtonArticulatedRobotComponent::GetManagedComponentNames() const
{
	if (!bUseEffectiveRobotDescriptionForManagedComponents)
	{
		return Super::GetManagedComponentNames();
	}

	TSet<FName>					 ManagedNames;
	FRammsNewtonRobotDescription RuntimeDescription = RobotDescription;
	if (RuntimeDescription.RobotName.IsNone())
	{
		if (const AActor* Owner = GetOwner())
		{
			RuntimeDescription.RobotName = Owner->GetFName();
		}
	}

	AppendInferredRobotDescription(RuntimeDescription);
	for (const FRammsNewtonLinkDescription& Link : RuntimeDescription.Links)
	{
		if (ShouldIncludeComponentName(Link.ComponentName))
		{
			ManagedNames.Add(Link.ComponentName);
		}
	}

	if (ManagedNames.Num() == 0)
	{
		return Super::GetManagedComponentNames();
	}

	TArray<FName> Result = ManagedNames.Array();
	Result.Sort([](const FName& A, const FName& B) {
		return A.LexicalLess(B);
	});
	return Result;
}

void URammsNewtonArticulatedRobotComponent::GetManagedPrimitiveComponents(TArray<UPrimitiveComponent*>& OutPrimitiveComponents) const
{
	Super::GetManagedPrimitiveComponents(OutPrimitiveComponents);
	if (!bUseEffectiveRobotDescriptionForManagedComponents)
	{
		return;
	}

	OutPrimitiveComponents.RemoveAll([](const UPrimitiveComponent* PrimitiveComponent) {
		return Cast<USkeletalMeshComponent>(PrimitiveComponent) != nullptr
			|| Cast<UPoseableMeshComponent>(PrimitiveComponent) != nullptr;
	});
}

FString URammsNewtonArticulatedRobotComponent::GetEffectiveRobotExportJson() const
{
	const AActor*					   Owner = GetOwner();
	const FRammsNewtonRobotDescription EffectiveDescription = GetEffectiveRobotDescription();

	TSharedRef<FJsonObject> RootJson = MakeShared<FJsonObject>();
	RootJson->SetStringField(TEXT("robot_name"), EffectiveDescription.RobotName.ToString());
	RootJson->SetBoolField(TEXT("treat_as_single_coupled_system"), EffectiveDescription.bTreatAsSingleCoupledSystem);
	RootJson->SetBoolField(TEXT("include_ground_interaction"), EffectiveDescription.bIncludeGroundInteraction);
	RootJson->SetBoolField(TEXT("include_manipulated_objects"), EffectiveDescription.bIncludeManipulatedObjects);
	RootJson->SetStringField(TEXT("actor_name"), Owner ? Owner->GetName() : FString());
	RootJson->SetStringField(TEXT("actor_path"), Owner ? Owner->GetPathName() : FString());

	const FTransform ActorTransform = Owner ? Owner->GetActorTransform() : FTransform::Identity;
	RootJson->SetObjectField(TEXT("actor_transform"), MakeTransformJson(ActorTransform));

	TSharedRef<FJsonObject> StageJson = MakeShared<FJsonObject>();
	StageJson->SetStringField(TEXT("up_axis"), TEXT("z"));
	StageJson->SetNumberField(TEXT("meters_per_unit"), 1.0);
	StageJson->SetNumberField(TEXT("kilograms_per_unit"), 1.0);
	RootJson->SetObjectField(TEXT("stage"), StageJson);

	TSharedRef<FJsonObject> SceneJson = MakeShared<FJsonObject>();
	SceneJson->SetBoolField(TEXT("gravity_enabled"), true);
	SceneJson->SetNumberField(TEXT("time_steps_per_second"), 60.0);
	SceneJson->SetNumberField(TEXT("max_solver_iterations"), 8.0);
	SceneJson->SetNumberField(TEXT("rigid_contact_relaxation"), 0.8);
	SceneJson->SetBoolField(TEXT("restitution_enabled"), true);
	RootJson->SetObjectField(TEXT("scene"), SceneJson);

	TMap<FName, FTransform> ResolvedWorldTransforms;
	TMap<FName, FString>	ResolvedTransformSources;

	TArray<TSharedPtr<FJsonValue>> LinkValues;
	LinkValues.Reserve(EffectiveDescription.Links.Num());
	for (const FRammsNewtonLinkDescription& Link : EffectiveDescription.Links)
	{
		FTransform WorldTransform = ActorTransform;
		FString	   TransformSource;
		const bool bResolvedTransform = TryResolveLinkWorldTransform(Owner, Link, WorldTransform, TransformSource);
		ResolvedWorldTransforms.Add(Link.LinkName, WorldTransform);
		ResolvedTransformSources.Add(Link.LinkName, TransformSource);

		FTransform RelativeTransform = WorldTransform.GetRelativeTransform(ActorTransform);
		if (!Link.ParentLinkName.IsNone())
		{
			if (const FTransform* ParentWorldTransform = ResolvedWorldTransforms.Find(Link.ParentLinkName))
			{
				RelativeTransform = WorldTransform.GetRelativeTransform(*ParentWorldTransform);
			}
		}

		TSharedRef<FJsonObject> LinkJson = MakeShared<FJsonObject>();
		LinkJson->SetStringField(TEXT("name"), Link.LinkName.ToString());
		LinkJson->SetStringField(TEXT("component_name"), Link.ComponentName.ToString());
		LinkJson->SetStringField(TEXT("parent_name"), Link.ParentLinkName.ToString());
		LinkJson->SetBoolField(TEXT("kinematic"), Link.bKinematic);
		LinkJson->SetBoolField(TEXT("treat_as_terrain"), Link.bTreatAsTerrain);
		LinkJson->SetNumberField(TEXT("mass_kg"), Link.MassKg);
		LinkJson->SetNumberField(TEXT("linear_damping"), Link.LinearDamping);
		LinkJson->SetNumberField(TEXT("angular_damping"), Link.AngularDamping);
		LinkJson->SetBoolField(TEXT("transform_resolved"), bResolvedTransform);
		LinkJson->SetStringField(TEXT("transform_source"), TransformSource);
		LinkJson->SetObjectField(TEXT("world_transform"), MakeTransformJson(WorldTransform));
		LinkJson->SetObjectField(TEXT("relative_transform"), MakeTransformJson(RelativeTransform));
		LinkJson->SetObjectField(TEXT("primitive"), MakePrimitiveMetadataJson(ResolvePrimitiveForLink(Owner, Link)));
		LinkValues.Add(MakeShared<FJsonValueObject>(LinkJson));
	}
	RootJson->SetArrayField(TEXT("links"), LinkValues);

	TArray<TSharedPtr<FJsonValue>> JointValues;
	JointValues.Reserve(EffectiveDescription.Joints.Num());
	for (const FRammsNewtonJointDescription& Joint : EffectiveDescription.Joints)
	{
		TSharedRef<FJsonObject> JointJson = MakeShared<FJsonObject>();
		JointJson->SetStringField(TEXT("name"), Joint.JointName.ToString());
		JointJson->SetStringField(TEXT("parent_link_name"), Joint.ParentLinkName.ToString());
		JointJson->SetStringField(TEXT("child_link_name"), Joint.ChildLinkName.ToString());
		JointJson->SetStringField(TEXT("joint_type"), ToJsonJointType(Joint.JointType));
		JointJson->SetStringField(TEXT("drive_mode"), ToJsonDriveMode(Joint.DriveMode));
		JointJson->SetStringField(TEXT("constraint_name"), Joint.ConstraintName.ToString());
		JointJson->SetStringField(TEXT("bone_name"), Joint.BoneName.ToString());
		JointJson->SetBoolField(TEXT("use_limits"), Joint.bUseLimits);
		JointJson->SetNumberField(TEXT("min_limit_degrees"), Joint.MinLimitDegrees);
		JointJson->SetNumberField(TEXT("max_limit_degrees"), Joint.MaxLimitDegrees);
		JointJson->SetNumberField(TEXT("max_effort"), Joint.MaxEffort);
		JointJson->SetArrayField(TEXT("axis"), MakeJsonNumberArray({
												   Joint.LocalAxis.X,
												   Joint.LocalAxis.Y,
												   Joint.LocalAxis.Z,
											   }));
		JointJson->SetBoolField(TEXT("use_explicit_joint_frames"), Joint.bUseExplicitJointFrames);
		if (Joint.bUseExplicitJointFrames)
		{
			JointJson->SetArrayField(TEXT("axis_in_parent_frame"), MakeJsonNumberArray({
																	   Joint.AxisInParentFrame.X,
																	   Joint.AxisInParentFrame.Y,
																	   Joint.AxisInParentFrame.Z,
																   }));
			JointJson->SetObjectField(TEXT("parent_anchor_transform"), MakeTransformJson(Joint.ParentAnchorTransform));
			JointJson->SetObjectField(TEXT("child_anchor_transform"), MakeTransformJson(Joint.ChildAnchorTransform));
		}
		JointValues.Add(MakeShared<FJsonValueObject>(JointJson));
	}
	RootJson->SetArrayField(TEXT("joints"), JointValues);

	TSharedRef<FJsonObject> ControllersJson = MakeShared<FJsonObject>();
	if (Owner && bAutoInferArmFromKinovaController)
	{
		if (const UKinovaGen3ControllerComponent* ArmController =
				FindNamedOrFirstComponent<UKinovaGen3ControllerComponent>(Owner, KinovaControllerComponentName))
		{
			TSharedRef<FJsonObject> ArmJson = MakeShared<FJsonObject>();
			ArmJson->SetStringField(TEXT("component_name"), ArmController->GetFName().ToString());
			ArmJson->SetStringField(TEXT("skeletal_mesh_component_name"), ArmController->SkeletalMeshComponentName.ToString());
			ArmJson->SetStringField(TEXT("end_effector_bone_name"), ArmController->EndEffectorBoneName.ToString());
			ArmJson->SetStringField(TEXT("control_mode"), ToJsonControlMode(ArmController->ControlMode));

			TArray<TSharedPtr<FJsonValue>> ArmJointValues;
			ArmJointValues.Reserve(ArmController->Joints.Num());
			for (const FRevoluteJointConfig& JointConfig : ArmController->Joints)
			{
				TSharedRef<FJsonObject> JointJson = MakeShared<FJsonObject>();
				JointJson->SetStringField(TEXT("name"),
					(JointConfig.ConstraintName != NAME_None ? JointConfig.ConstraintName : JointConfig.BoneName).ToString());
				JointJson->SetStringField(TEXT("bone_name"), JointConfig.BoneName.ToString());
				JointJson->SetStringField(TEXT("constraint_name"), JointConfig.ConstraintName.ToString());
				JointJson->SetStringField(TEXT("axis"), ToJsonConstraintAxis(JointConfig.ControlledAxis));
				JointJson->SetBoolField(TEXT("invert_axis"), JointConfig.bInvertAxisForIK);
				JointJson->SetNumberField(TEXT("angle_offset_degrees"), JointConfig.AngleOffset);
				JointJson->SetNumberField(TEXT("max_speed_degrees_per_second"), JointConfig.MaxAngularSpeed * JointConfig.SpeedMultiplier);
				JointJson->SetNumberField(TEXT("max_torque"), JointConfig.MaxTorque);
				JointJson->SetNumberField(TEXT("position_strength"), JointConfig.PositionStrength);
				JointJson->SetNumberField(TEXT("position_damping"), JointConfig.PositionDamping);
				JointJson->SetBoolField(TEXT("software_limits_enabled"), JointConfig.bEnableSoftwareLimits);
				JointJson->SetNumberField(TEXT("min_limit_degrees"), JointConfig.MinAngleLimit);
				JointJson->SetNumberField(TEXT("max_limit_degrees"), JointConfig.MaxAngleLimit);
				ArmJointValues.Add(MakeShared<FJsonValueObject>(JointJson));
			}
			ArmJson->SetArrayField(TEXT("joints"), ArmJointValues);
			ControllersJson->SetObjectField(TEXT("kinova"), ArmJson);
		}
	}

	if (Owner && bAutoInferGripperFromController)
	{
		if (const UGripperControllerComponent* GripperController =
				FindNamedOrFirstComponent<UGripperControllerComponent>(Owner, GripperControllerComponentName))
		{
			TSharedRef<FJsonObject> GripperJson = MakeShared<FJsonObject>();
			GripperJson->SetStringField(TEXT("component_name"), GripperController->GetFName().ToString());
			GripperJson->SetStringField(TEXT("skeletal_mesh_component_name"), GetGripperMeshComponentName(*GripperController).ToString());
			GripperJson->SetNumberField(TEXT("open_angle_degrees"), GetGripperOpenAngle(*GripperController));
			GripperJson->SetNumberField(TEXT("closed_angle_degrees"), GetGripperClosedAngle(*GripperController));

			const TArray<FAngularMotorConfig> FingerMotors = GetGripperFingerMotors(*GripperController);
			TArray<TSharedPtr<FJsonValue>> FingerValues;
			FingerValues.Reserve(FingerMotors.Num());
			for (const FAngularMotorConfig& FingerMotor : FingerMotors)
			{
				TSharedRef<FJsonObject> FingerJson = MakeShared<FJsonObject>();
				FingerJson->SetStringField(TEXT("constraint_name"), FingerMotor.ConstraintName.ToString());
				FingerJson->SetStringField(TEXT("axis"), ToJsonMotorAxis(FingerMotor.ControlAxis));
				FingerJson->SetBoolField(TEXT("enabled"), FingerMotor.bEnabled);
				FingerJson->SetBoolField(TEXT("invert_direction"), FingerMotor.bInvertDirection);
				FingerJson->SetNumberField(TEXT("target_angle_degrees"), FingerMotor.TargetAngle);
				FingerJson->SetNumberField(TEXT("max_speed_degrees_per_second"), FingerMotor.MaxSpeed * FingerMotor.SpeedMultiplier);
				FingerJson->SetNumberField(TEXT("motor_strength"), FingerMotor.MotorStrength);
				FingerJson->SetNumberField(TEXT("motor_damping"), FingerMotor.MotorDamping);
				FingerValues.Add(MakeShared<FJsonValueObject>(FingerJson));
			}
			GripperJson->SetArrayField(TEXT("fingers"), FingerValues);
			ControllersJson->SetObjectField(TEXT("gripper"), GripperJson);
		}
	}
	RootJson->SetObjectField(TEXT("controllers"), ControllersJson);

	FString					  Output;
	TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&Output);
	FJsonSerializer::Serialize(RootJson, Writer);
	return Output;
}

FString URammsNewtonArticulatedRobotComponent::GetCurrentJointControlJson() const
{
	const AActor*					   Owner = GetOwner();
	const FRammsNewtonRobotDescription EffectiveDescription = GetEffectiveRobotDescription();

	TMap<FName, TSharedPtr<FJsonObject>> JointControlsByName;
	auto								 FindOrAddJointControl = [&JointControlsByName](const FName JointName) -> TSharedRef<FJsonObject> {
		if (TSharedPtr<FJsonObject>* Existing = JointControlsByName.Find(JointName))
		{
			return Existing->ToSharedRef();
		}

		TSharedRef<FJsonObject> JointJson = MakeShared<FJsonObject>();
		JointJson->SetStringField(TEXT("name"), JointName.ToString());
		JointControlsByName.Add(JointName, JointJson);
		return JointJson;
	};

	for (const FRammsNewtonJointDescription& Joint : EffectiveDescription.Joints)
	{
		if (Joint.JointName.IsNone())
		{
			continue;
		}

		TSharedRef<FJsonObject> JointJson = FindOrAddJointControl(Joint.JointName);
		JointJson->SetStringField(TEXT("drive_mode"), ToJsonDriveMode(Joint.DriveMode));
		JointJson->SetNumberField(TEXT("max_effort"), Joint.MaxEffort);
	}

	if (Owner && bAutoInferArmFromKinovaController)
	{
		if (const UKinovaGen3ControllerComponent* ArmController =
				FindNamedOrFirstComponent<UKinovaGen3ControllerComponent>(Owner, KinovaControllerComponentName))
		{
			for (const FRevoluteJointConfig& JointConfig : ArmController->Joints)
			{
				const FName JointName = JointConfig.ConstraintName != NAME_None ? JointConfig.ConstraintName : JointConfig.BoneName;
				if (JointName.IsNone())
				{
					continue;
				}

				TSharedRef<FJsonObject> JointJson = FindOrAddJointControl(JointName);
				JointJson->SetStringField(TEXT("drive_mode"), ToJsonDriveMode(ToNewtonDriveMode(ArmController->ControlMode)));
				JointJson->SetNumberField(TEXT("target_angle_degrees"), JointConfig.TargetAngle);
				JointJson->SetNumberField(TEXT("current_angle_degrees"), JointConfig.CurrentAngle);
				JointJson->SetNumberField(TEXT("position_gain"), JointConfig.PositionStrength * 1.0e-5);
				JointJson->SetNumberField(TEXT("damping_gain"), JointConfig.PositionDamping * 1.0e-5);
				JointJson->SetNumberField(TEXT("max_effort"), JointConfig.MaxTorque);

				const float DeltaDegrees = FMath::FindDeltaAngleDegrees(JointConfig.CurrentAngle, JointConfig.TargetAngle);
				const float TargetVelocityDegreesPerSecond =
					FMath::Abs(DeltaDegrees) > KINDA_SMALL_NUMBER
					? FMath::Sign(DeltaDegrees) * (JointConfig.MaxAngularSpeed * JointConfig.SpeedMultiplier)
					: 0.0f;
				JointJson->SetNumberField(TEXT("target_velocity_degrees_per_second"), TargetVelocityDegreesPerSecond);
				JointJson->SetNumberField(
					TEXT("feedforward_effort"),
					FMath::Abs(DeltaDegrees) > KINDA_SMALL_NUMBER ? FMath::Sign(DeltaDegrees) * JointConfig.MaxTorque : 0.0f);
			}
		}
	}

	if (Owner && bAutoInferGripperFromController)
	{
		if (const UGripperControllerComponent* GripperController =
				FindNamedOrFirstComponent<UGripperControllerComponent>(Owner, GripperControllerComponentName))
		{
			const TArray<FAngularMotorConfig> FingerMotors = GetGripperFingerMotors(*GripperController);

			for (const FAngularMotorConfig& FingerMotor : FingerMotors)
			{
				if (FingerMotor.ConstraintName.IsNone())
				{
					continue;
				}

				TSharedRef<FJsonObject> JointJson = FindOrAddJointControl(FingerMotor.ConstraintName);
				JointJson->SetStringField(TEXT("drive_mode"), TEXT("position_control"));
				JointJson->SetNumberField(TEXT("target_angle_degrees"), FingerMotor.TargetAngle);
				JointJson->SetNumberField(TEXT("current_angle_degrees"), FingerMotor.CurrentAngle);
				JointJson->SetNumberField(TEXT("position_gain"), FingerMotor.MotorStrength * 1.0e-4);
				JointJson->SetNumberField(TEXT("damping_gain"), FingerMotor.MotorDamping * 1.0e-4);
				JointJson->SetNumberField(TEXT("max_effort"), FingerMotor.MotorStrength * 1.0e-3);

				const float DeltaDegrees = FingerMotor.TargetAngle - FingerMotor.CurrentAngle;
				const float TargetVelocityDegreesPerSecond =
					FMath::Abs(DeltaDegrees) > KINDA_SMALL_NUMBER
					? FMath::Sign(DeltaDegrees) * (FingerMotor.MaxSpeed * FingerMotor.SpeedMultiplier)
					: 0.0f;
				JointJson->SetNumberField(TEXT("target_velocity_degrees_per_second"), TargetVelocityDegreesPerSecond);
				JointJson->SetNumberField(
					TEXT("feedforward_effort"),
					FMath::Abs(DeltaDegrees) > KINDA_SMALL_NUMBER ? FMath::Sign(DeltaDegrees) * (FingerMotor.MotorStrength * 1.0e-3) : 0.0f);
			}
		}
	}

	TSharedRef<FJsonObject> RootJson = MakeShared<FJsonObject>();
	RootJson->SetStringField(TEXT("robot_name"), EffectiveDescription.RobotName.ToString());

	TArray<TSharedPtr<FJsonValue>> JointValues;
	JointValues.Reserve(JointControlsByName.Num());
	for (const TPair<FName, TSharedPtr<FJsonObject>>& Pair : JointControlsByName)
	{
		if (Pair.Value.IsValid())
		{
			JointValues.Add(MakeShared<FJsonValueObject>(Pair.Value.ToSharedRef()));
		}
	}

	RootJson->SetArrayField(TEXT("joints"), JointValues);

	FString					  Output;
	TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&Output);
	FJsonSerializer::Serialize(RootJson, Writer);
	return Output;
}

bool URammsNewtonArticulatedRobotComponent::HasValidRobotDescription(FString& OutIssue) const
{
	const FRammsNewtonRobotDescription EffectiveDescription = GetEffectiveRobotDescription();
	TSet<FName>						   LinkNames;
	for (const FRammsNewtonLinkDescription& Link : EffectiveDescription.Links)
	{
		if (!Link.LinkName.IsNone())
		{
			LinkNames.Add(Link.LinkName);
		}
	}

	if (LinkNames.Num() == 0)
	{
		OutIssue = TEXT("No Newton links are configured or inferable from the actor/controllers.");
		return false;
	}

	for (const FRammsNewtonJointDescription& Joint : EffectiveDescription.Joints)
	{
		if (Joint.JointName.IsNone())
		{
			OutIssue = TEXT("A Newton joint is missing its joint name.");
			return false;
		}

		if (Joint.ParentLinkName.IsNone() || Joint.ChildLinkName.IsNone())
		{
			OutIssue = FString::Printf(TEXT("Joint '%s' is missing a parent or child link."), *Joint.JointName.ToString());
			return false;
		}

		if (!LinkNames.Contains(Joint.ParentLinkName))
		{
			OutIssue = FString::Printf(
				TEXT("Joint '%s' references missing parent link '%s'."),
				*Joint.JointName.ToString(),
				*Joint.ParentLinkName.ToString());
			return false;
		}

		if (!LinkNames.Contains(Joint.ChildLinkName))
		{
			OutIssue = FString::Printf(
				TEXT("Joint '%s' references missing child link '%s'."),
				*Joint.JointName.ToString(),
				*Joint.ChildLinkName.ToString());
			return false;
		}
	}

	OutIssue.Reset();
	return true;
}

void URammsNewtonArticulatedRobotComponent::AutoPopulateRobotDescription()
{
	AutoPopulateRobotDescriptionFromControllers(true);
}

void URammsNewtonArticulatedRobotComponent::AutoPopulateRobotDescriptionFromControllers(bool bOverwriteExisting)
{
	if (bOverwriteExisting)
	{
		FRammsNewtonRobotDescription PopulatedDescription;
		const AActor*				 Owner = GetOwner();
		PopulatedDescription.RobotName = RobotDescription.RobotName;
		PopulatedDescription.bTreatAsSingleCoupledSystem = RobotDescription.bTreatAsSingleCoupledSystem;
		PopulatedDescription.bIncludeGroundInteraction = RobotDescription.bIncludeGroundInteraction;
		PopulatedDescription.bIncludeManipulatedObjects = RobotDescription.bIncludeManipulatedObjects;

		if (PopulatedDescription.RobotName.IsNone())
		{
			if (Owner)
			{
				PopulatedDescription.RobotName = Owner->GetFName();
			}
		}

		if (bAutoInferLinksFromManagedComponents)
		{
			for (const FName ManagedComponentName : GetManagedComponentNamesFromBridgeDescription())
			{
				if (!ShouldAutoInferManagedLinkFromComponent(Owner, ManagedComponentName))
				{
					continue;
				}

				FRammsNewtonLinkDescription Link;
				Link.LinkName = ManagedComponentName;
				Link.ComponentName = ManagedComponentName;
				AddOrMergeLink(PopulatedDescription.Links, Link);
			}
		}

		AppendInferredRobotDescription(PopulatedDescription);
		RobotDescription = MoveTemp(PopulatedDescription);
		return;
	}

	RobotDescription = GetEffectiveRobotDescription();
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

void URammsNewtonArticulatedRobotComponent::ApplySolvedJointStates(
	const TMap<FName, float>& JointPositions,
	const TMap<FName, float>& JointVelocities)
{
	(void)JointVelocities;

	if (!bEnableSkeletalPoseWriteback || JointPositions.Num() == 0)
	{
		return;
	}

	const FRammsNewtonRobotDescription EffectiveDescription = GetEffectiveRobotDescription();
	EnsureSkeletalPoseWritebackSetup(EffectiveDescription);

	URammsSkeletalPoseComponent* PoseComponent = RuntimeSkeletalPoseComponent.Get();
	if (!PoseComponent && GetOwner())
	{
		PoseComponent = GetOwner()->FindComponentByClass<URammsSkeletalPoseComponent>();
	}
	if (!PoseComponent)
	{
		return;
	}

	for (const TPair<FName, TObjectPtr<USkeletalMeshComponent>>& Pair : RuntimeSourceSkeletalMeshes)
	{
		if (USkeletalMeshComponent* SourceMesh = Pair.Value)
		{
			if (const TObjectPtr<UPoseableMeshComponent>* MirrorPtr = RuntimePoseMirrorsBySourceMeshName.Find(Pair.Key))
			{
				if (UPoseableMeshComponent* Mirror = MirrorPtr->Get())
				{
					Mirror->SetWorldTransform(SourceMesh->GetComponentTransform());
				}
			}
		}
	}

	bool bUpdatedAnyJoint = false;
	for (FKinematicJointConfig& PoseJoint : PoseComponent->Joints)
	{
		if (const float* SolvedPosition = JointPositions.Find(PoseJoint.GetEffectiveName()))
		{
			PoseJoint.TargetValue = *SolvedPosition;
			bUpdatedAnyJoint = true;
		}
	}

	if (bUpdatedAnyJoint)
	{
		PoseComponent->SnapToTargets();
	}
}

void URammsNewtonArticulatedRobotComponent::EnsureSkeletalPoseWritebackSetup(
	const FRammsNewtonRobotDescription& EffectiveDescription)
{
	if (!bEnableSkeletalPoseWriteback)
	{
		return;
	}

	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}

	URammsSkeletalPoseComponent* PoseComponent = RuntimeSkeletalPoseComponent.Get();
	if (!PoseComponent)
	{
		PoseComponent = Owner->FindComponentByClass<URammsSkeletalPoseComponent>();
	}
	if (!PoseComponent)
	{
		PoseComponent = NewObject<URammsSkeletalPoseComponent>(Owner, TEXT("RammsNewtonRuntimePose"));
		if (PoseComponent)
		{
			Owner->AddInstanceComponent(PoseComponent);
			PoseComponent->RegisterComponent();
		}
	}

	if (!PoseComponent)
	{
		return;
	}

	RuntimeSkeletalPoseComponent = PoseComponent;

	TMap<FName, FRammsNewtonLinkDescription> LinksByName;
	for (const FRammsNewtonLinkDescription& Link : EffectiveDescription.Links)
	{
		LinksByName.Add(Link.LinkName, Link);
	}

	RuntimeSourceSkeletalMeshes.Reset();
	for (const FRammsNewtonLinkDescription& Link : EffectiveDescription.Links)
	{
		if (Link.ComponentName.IsNone())
		{
			continue;
		}

		USkeletalMeshComponent* SourceMesh = FindNamedOrFirstComponent<USkeletalMeshComponent>(Owner, Link.ComponentName);
		if (!SourceMesh)
		{
			continue;
		}

		const bool bIsBoneLink = !Link.LinkName.IsNone() && SourceMesh->GetBoneIndex(Link.LinkName) != INDEX_NONE;
		const bool bIsRootMeshLink = Link.LinkName == Link.ComponentName;
		if (!bIsBoneLink && !bIsRootMeshLink)
		{
			continue;
		}

		RuntimeSourceSkeletalMeshes.Add(Link.ComponentName, SourceMesh);
		RuntimeSourceCollisionModes.FindOrAdd(Link.ComponentName, SourceMesh->GetCollisionEnabled());
		RuntimeSourceGenerateOverlapEvents.FindOrAdd(Link.ComponentName, SourceMesh->GetGenerateOverlapEvents());
		RuntimeSourceVisibility.FindOrAdd(Link.ComponentName, SourceMesh->GetVisibleFlag());
		RuntimeSourceHiddenInGame.FindOrAdd(Link.ComponentName, SourceMesh->bHiddenInGame);

		UPoseableMeshComponent* PoseMirror = RuntimePoseMirrorsBySourceMeshName.FindRef(Link.ComponentName);
		if (!PoseMirror)
		{
			const FName MirrorName = *FString::Printf(TEXT("%s_NewtonPose"), *SourceMesh->GetFName().ToString());
			PoseMirror = FindNamedOrFirstComponent<UPoseableMeshComponent>(Owner, MirrorName);
			if (!PoseMirror && bAutoCreatePoseableMirrors)
			{
				PoseMirror = NewObject<UPoseableMeshComponent>(Owner, UPoseableMeshComponent::StaticClass(), MirrorName);
				if (PoseMirror)
				{
					if (USceneComponent* ParentComponent = SourceMesh->GetAttachParent())
					{
						PoseMirror->SetupAttachment(ParentComponent);
						PoseMirror->SetRelativeTransform(SourceMesh->GetRelativeTransform());
					}

					if (USkeletalMesh* SourceSkeletalMesh = SourceMesh->GetSkeletalMeshAsset())
					{
						PoseMirror->SetSkinnedAssetAndUpdate(SourceSkeletalMesh, true);
					}
					for (int32 MaterialIndex = 0; MaterialIndex < SourceMesh->GetNumMaterials(); ++MaterialIndex)
					{
						if (UMaterialInterface* Material = SourceMesh->GetMaterial(MaterialIndex))
						{
							PoseMirror->SetMaterial(MaterialIndex, Material);
						}
					}
					PoseMirror->SetWorldTransform(SourceMesh->GetComponentTransform());

					Owner->AddInstanceComponent(PoseMirror);
					PoseMirror->RegisterComponent();
				}
			}
		}

		if (PoseMirror)
		{
			RuntimePoseMirrorsBySourceMeshName.Add(Link.ComponentName, PoseMirror);
			PoseMirror->SetCollisionProfileName(SourceMesh->GetCollisionProfileName());
			PoseMirror->SetCollisionObjectType(SourceMesh->GetCollisionObjectType());
			PoseMirror->SetCollisionResponseToChannels(SourceMesh->GetCollisionResponseToChannels());
			PoseMirror->SetGenerateOverlapEvents(
				bHideSourceSkeletalMeshesWhenUsingPoseMirrors
					? RuntimeSourceGenerateOverlapEvents.FindRef(Link.ComponentName)
					: false);
			PoseMirror->SetCollisionEnabled(
				bHideSourceSkeletalMeshesWhenUsingPoseMirrors
					? RuntimeSourceCollisionModes.FindRef(Link.ComponentName)
					: ECollisionEnabled::NoCollision);

			if (bHideSourceSkeletalMeshesWhenUsingPoseMirrors)
			{
				SourceMesh->SetVisibility(false, false);
				SourceMesh->SetHiddenInGame(true, false);
				SourceMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
				SourceMesh->SetGenerateOverlapEvents(false);
			}
			else
			{
				SourceMesh->SetVisibility(RuntimeSourceVisibility.FindRef(Link.ComponentName), false);
				SourceMesh->SetHiddenInGame(RuntimeSourceHiddenInGame.FindRef(Link.ComponentName), false);
				SourceMesh->SetCollisionEnabled(RuntimeSourceCollisionModes.FindRef(Link.ComponentName));
				SourceMesh->SetGenerateOverlapEvents(RuntimeSourceGenerateOverlapEvents.FindRef(Link.ComponentName));
			}
		}
	}

	PoseComponent->RefreshPoseableMeshes();

	TSet<FName> RuntimePoseMirrorNames;
	for (const TPair<FName, TObjectPtr<UPoseableMeshComponent>>& Pair : RuntimePoseMirrorsBySourceMeshName)
	{
		if (Pair.Value)
		{
			RuntimePoseMirrorNames.Add(Pair.Value->GetFName());
		}
	}

	PoseComponent->Joints.RemoveAll([&RuntimePoseMirrorNames](const FKinematicJointConfig& ExistingJoint) {
		return RuntimePoseMirrorNames.Contains(ExistingJoint.MeshComponentName);
	});

	for (const FRammsNewtonJointDescription& Joint : EffectiveDescription.Joints)
	{
		const FRammsNewtonLinkDescription* ChildLink = LinksByName.Find(Joint.ChildLinkName);
		if (!ChildLink)
		{
			continue;
		}

		USkeletalMeshComponent* SourceMesh = RuntimeSourceSkeletalMeshes.FindRef(ChildLink->ComponentName);
		UPoseableMeshComponent* PoseMirror = RuntimePoseMirrorsBySourceMeshName.FindRef(ChildLink->ComponentName);
		if (!SourceMesh || !PoseMirror)
		{
			continue;
		}

		const FName BoneName = !Joint.BoneName.IsNone() ? Joint.BoneName : ChildLink->LinkName;
		if (BoneName.IsNone() || SourceMesh->GetBoneIndex(BoneName) == INDEX_NONE)
		{
			continue;
		}

		bool				  bInvertDirection = false;
		FKinematicJointConfig PoseJoint;
		PoseJoint.MeshComponentName = PoseMirror->GetFName();
		PoseJoint.BoneName = BoneName;
		PoseJoint.JointName = Joint.JointName;
		PoseJoint.JointType = Joint.JointType == ERammsNewtonJointType::Prismatic
			? EKinematicJointType::Prismatic
			: EKinematicJointType::Revolute;
		PoseJoint.Axis = ToPoseAxis(Joint.LocalAxis, bInvertDirection);
		PoseJoint.bInvertDirection = bInvertDirection;
		PoseJoint.MinValue = PoseJoint.JointType == EKinematicJointType::Prismatic
			? Joint.MinLimitDegrees
			: Joint.MinLimitDegrees;
		PoseJoint.MaxValue = PoseJoint.JointType == EKinematicJointType::Prismatic
			? Joint.MaxLimitDegrees
			: Joint.MaxLimitDegrees;
		PoseJoint.bEnforceLimits = Joint.bUseLimits;
		PoseComponent->Joints.Add(PoseJoint);
	}
}

void URammsNewtonArticulatedRobotComponent::AppendInferredRobotDescription(FRammsNewtonRobotDescription& InOutDescription) const
{
	const AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}

	const UKinovaGen3ControllerComponent* ArmController = bAutoInferArmFromKinovaController
		? FindNamedOrFirstComponent<UKinovaGen3ControllerComponent>(Owner, KinovaControllerComponentName)
		: nullptr;
	const UGripperControllerComponent*	  GripperController = bAutoInferGripperFromController
		   ? FindNamedOrFirstComponent<UGripperControllerComponent>(Owner, GripperControllerComponentName)
		   : nullptr;

	FName ArmRootLinkName = NAME_None;
	FName EndEffectorLinkName = NAME_None;

	if (ArmController)
	{
		const USkeletalMeshComponent* ArmMeshComponent =
			FindNamedOrFirstComponent<USkeletalMeshComponent>(Owner, ArmController->SkeletalMeshComponentName);
		const FName ArmMeshName = ArmMeshComponent ? ArmMeshComponent->GetFName() : ResolveSkeletalMeshName(Owner, ArmController->SkeletalMeshComponentName);
		ArmRootLinkName = ArmMeshName;

		if (ArmMeshName != NAME_None)
		{
			FRammsNewtonLinkDescription RootLink;
			RootLink.LinkName = ArmMeshName;
			RootLink.ComponentName = ArmMeshName;
			AddOrMergeLink(InOutDescription.Links, RootLink);
		}

		for (const FRevoluteJointConfig& JointConfig : ArmController->Joints)
		{
			FName ChildLinkName = JointConfig.BoneName;
			if (ChildLinkName.IsNone())
			{
				continue;
			}

			FName						 ParentLinkName = ArmMeshName;
			FSkeletalConstraintJointInfo ConstraintInfo;
			const float					 ConstraintDirectionSign = FMath::IsNearlyZero(JointConfig.ConstraintAngleSign) ? 1.0f : JointConfig.ConstraintAngleSign;
			const bool					 bHasConstraintInfo =
				ArmMeshComponent
				&& JointConfig.ConstraintName != NAME_None
				&& TryBuildConstraintJointInfo(ArmMeshComponent, JointConfig.ConstraintName, JointConfig.ControlledAxis, ConstraintDirectionSign, ConstraintInfo);
			if (bHasConstraintInfo)
			{
				ChildLinkName = ConstraintInfo.ChildBoneName;
				if (ConstraintInfo.ParentBoneName != NAME_None)
				{
					ParentLinkName = ConstraintInfo.ParentBoneName;
				}
			}

			if (ArmMeshComponent)
			{
				const FName ParentBoneName = ArmMeshComponent->GetParentBone(ChildLinkName);
				if (ParentBoneName != NAME_None)
				{
					ParentLinkName = ParentBoneName;
				}
			}

			if (ParentLinkName != NAME_None)
			{
				FRammsNewtonLinkDescription ParentLink;
				ParentLink.LinkName = ParentLinkName;
				ParentLink.ComponentName = ArmMeshName;
				AddOrMergeLink(InOutDescription.Links, ParentLink);
			}

			FRammsNewtonLinkDescription ChildLink;
			ChildLink.LinkName = ChildLinkName;
			ChildLink.ComponentName = ArmMeshName;
			ChildLink.ParentLinkName = ParentLinkName;
			AddOrMergeLink(InOutDescription.Links, ChildLink);

			FRammsNewtonJointDescription Joint;
			Joint.JointName = JointConfig.ConstraintName != NAME_None ? JointConfig.ConstraintName : JointConfig.BoneName;
			Joint.ParentLinkName = ParentLinkName;
			Joint.ChildLinkName = ChildLinkName;
			Joint.JointType = ERammsNewtonJointType::Revolute;
			Joint.DriveMode = ToNewtonDriveMode(ArmController->ControlMode);
			Joint.LocalAxis = bHasConstraintInfo ? ConstraintInfo.PoseAxis : (ToNewtonAxis(JointConfig.ControlledAxis) * ConstraintDirectionSign);
			Joint.bUseExplicitJointFrames = bHasConstraintInfo;
			Joint.AxisInParentFrame = bHasConstraintInfo ? ConstraintInfo.AxisInParentFrame : (ToNewtonAxis(JointConfig.ControlledAxis) * ConstraintDirectionSign);
			Joint.ParentAnchorTransform = bHasConstraintInfo ? ConstraintInfo.ParentAnchorTransform : FTransform::Identity;
			Joint.ChildAnchorTransform = bHasConstraintInfo ? ConstraintInfo.ChildAnchorTransform : FTransform::Identity;
			Joint.ConstraintName = JointConfig.ConstraintName;
			Joint.BoneName = ChildLinkName;
			Joint.bUseLimits = JointConfig.bEnableSoftwareLimits;
			Joint.MinLimitDegrees = JointConfig.MinAngleLimit;
			Joint.MaxLimitDegrees = JointConfig.MaxAngleLimit;
			Joint.MaxEffort = JointConfig.MaxTorque;
			AddJointIfMissing(InOutDescription.Joints, Joint);

			EndEffectorLinkName = ChildLinkName;
		}

		if (ArmController->EndEffectorBoneName != NAME_None)
		{
			FName ParentLinkName = EndEffectorLinkName;
			if (ArmMeshComponent)
			{
				const FName ParentBoneName = ArmMeshComponent->GetParentBone(ArmController->EndEffectorBoneName);
				if (ParentBoneName != NAME_None)
				{
					ParentLinkName = ParentBoneName;
				}
			}

			FRammsNewtonLinkDescription EndEffectorLink;
			EndEffectorLink.LinkName = ArmController->EndEffectorBoneName;
			EndEffectorLink.ComponentName = ArmMeshName;
			EndEffectorLink.ParentLinkName = ParentLinkName;
			AddOrMergeLink(InOutDescription.Links, EndEffectorLink);
			EndEffectorLinkName = ArmController->EndEffectorBoneName;
		}
	}

	if (GripperController)
	{
		const FName					  ConfiguredGripperMeshName = GetGripperMeshComponentName(*GripperController);
		const USkeletalMeshComponent* GripperMeshComponent =
			FindNamedOrFirstComponent<USkeletalMeshComponent>(Owner, ConfiguredGripperMeshName);
		const FName GripperMeshName = GripperMeshComponent ? GripperMeshComponent->GetFName()
														   : ResolveSkeletalMeshName(Owner, ConfiguredGripperMeshName);
		FName		ParentLinkName = EndEffectorLinkName != NAME_None ? EndEffectorLinkName : ArmRootLinkName;
		if (ParentLinkName == NAME_None)
		{
			ParentLinkName = GripperMeshName;
		}

		if (ParentLinkName != NAME_None)
		{
			FRammsNewtonLinkDescription ParentLink;
			ParentLink.LinkName = ParentLinkName;
			ParentLink.ComponentName = GripperMeshName;
			AddOrMergeLink(InOutDescription.Links, ParentLink);
		}

		const float						  GripperOpenAngle = GetGripperOpenAngle(*GripperController);
		const float						  GripperClosedAngle = GetGripperClosedAngle(*GripperController);
		const float						  MinFingerAngle = FMath::Min(GripperOpenAngle, GripperClosedAngle);
		const float						  MaxFingerAngle = FMath::Max(GripperOpenAngle, GripperClosedAngle);
		const TArray<FAngularMotorConfig> FingerMotors = GetGripperFingerMotors(*GripperController);

		for (const FAngularMotorConfig& FingerMotor : FingerMotors)
		{
			if (FingerMotor.ConstraintName.IsNone())
			{
				continue;
			}

			FSkeletalConstraintJointInfo ConstraintInfo;
			const bool					 bHasConstraintInfo =
				GripperMeshComponent
				&& TryBuildConstraintJointInfo(
					GripperMeshComponent,
					FingerMotor.ConstraintName,
					FingerMotor.ControlAxis,
					FingerMotor.bInvertDirection ? -1.0f : 1.0f,
					ConstraintInfo);
			const FName FingerLinkName = bHasConstraintInfo ? ConstraintInfo.ChildBoneName : FingerMotor.ConstraintName;
			FName		FingerParentLinkName = ParentLinkName;
			if (bHasConstraintInfo && ConstraintInfo.ParentBoneName != NAME_None)
			{
				FingerParentLinkName = ConstraintInfo.ParentBoneName;

				FRammsNewtonLinkDescription ParentBoneLink;
				ParentBoneLink.LinkName = FingerParentLinkName;
				ParentBoneLink.ComponentName = GripperMeshName;
				if (GripperMeshComponent)
				{
					const FName ParentBoneParent = GripperMeshComponent->GetParentBone(FingerParentLinkName);
					if (ParentBoneParent != NAME_None)
					{
						ParentBoneLink.ParentLinkName = ParentBoneParent;
					}
					else if (ParentLinkName != FingerParentLinkName)
					{
						ParentBoneLink.ParentLinkName = ParentLinkName;
					}
				}
				AddOrMergeLink(InOutDescription.Links, ParentBoneLink);
			}

			FRammsNewtonLinkDescription FingerLink;
			FingerLink.LinkName = FingerLinkName;
			FingerLink.ComponentName = GripperMeshName;
			FingerLink.ParentLinkName = FingerParentLinkName;
			AddOrMergeLink(InOutDescription.Links, FingerLink);

			FRammsNewtonJointDescription FingerJoint;
			FingerJoint.JointName = FingerMotor.ConstraintName;
			FingerJoint.ParentLinkName = FingerParentLinkName;
			FingerJoint.ChildLinkName = FingerLinkName;
			FingerJoint.JointType = ERammsNewtonJointType::Revolute;
			FingerJoint.DriveMode = ERammsNewtonJointDriveMode::PositionControl;
			FingerJoint.LocalAxis = bHasConstraintInfo ? ConstraintInfo.PoseAxis
													   : (ToNewtonAxis(FingerMotor.ControlAxis) * (FingerMotor.bInvertDirection ? -1.0f : 1.0f));
			FingerJoint.bUseExplicitJointFrames = bHasConstraintInfo;
			FingerJoint.AxisInParentFrame = bHasConstraintInfo ? ConstraintInfo.AxisInParentFrame
															   : (ToNewtonAxis(FingerMotor.ControlAxis) * (FingerMotor.bInvertDirection ? -1.0f : 1.0f));
			FingerJoint.ParentAnchorTransform = bHasConstraintInfo ? ConstraintInfo.ParentAnchorTransform : FTransform::Identity;
			FingerJoint.ChildAnchorTransform = bHasConstraintInfo ? ConstraintInfo.ChildAnchorTransform : FTransform::Identity;
			FingerJoint.ConstraintName = FingerMotor.ConstraintName;
			FingerJoint.BoneName = FingerLinkName;
			FingerJoint.bUseLimits = true;
			FingerJoint.MinLimitDegrees = MinFingerAngle;
			FingerJoint.MaxLimitDegrees = MaxFingerAngle;
			FingerJoint.MaxEffort = FingerMotor.MotorStrength;
			AddJointIfMissing(InOutDescription.Joints, FingerJoint);
		}
	}
}
