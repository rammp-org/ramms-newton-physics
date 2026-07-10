// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RammsNewtonPhysicsTypes.generated.h"

UENUM(BlueprintType)
enum class ERammsNewtonBackendMode : uint8
{
	StubOnly			 UMETA(DisplayName = "Stub Only"),
	HeadersOnly			 UMETA(DisplayName = "Headers Detected"),
	PrebuiltLibrary		 UMETA(DisplayName = "Prebuilt Library Linked"),
	DynamicLibrary		 UMETA(DisplayName = "Dynamic Library Loaded"),
	ExternalPythonBridge UMETA(DisplayName = "External Python Bridge"),
	BuiltInAdapter		 UMETA(DisplayName = "Built-In Adapter"),
};

UENUM(BlueprintType)
enum class ERammsNewtonSimulationRole : uint8
{
	GenericRigidBody UMETA(DisplayName = "Generic Rigid Body"),
	MobilityBase	 UMETA(DisplayName = "Mobility Base"),
	Manipulator		 UMETA(DisplayName = "Manipulator"),
	HybridRobot		 UMETA(DisplayName = "Hybrid Robot"),
};

UENUM(BlueprintType)
enum class ERammsNewtonCollisionGeometryMode : uint8
{
	Auto				   UMETA(DisplayName = "Auto"),
	PrimitiveApproximation UMETA(DisplayName = "Primitive Approximation"),
	TriangleMesh		   UMETA(DisplayName = "Triangle Mesh"),
	ConvexHull			   UMETA(DisplayName = "Convex Hull"),
};

UENUM(BlueprintType)
enum class ERammsNewtonJointType : uint8
{
	Fixed	  UMETA(DisplayName = "Fixed"),
	Revolute  UMETA(DisplayName = "Revolute"),
	Prismatic UMETA(DisplayName = "Prismatic"),
	Spherical UMETA(DisplayName = "Spherical"),
};

UENUM(BlueprintType)
enum class ERammsNewtonJointDriveMode : uint8
{
	Passive			UMETA(DisplayName = "Passive"),
	PositionControl UMETA(DisplayName = "Position Control"),
	VelocityControl UMETA(DisplayName = "Velocity Control"),
	TorqueControl	UMETA(DisplayName = "Torque / Force Control"),
};

USTRUCT(BlueprintType)
struct RAMMSNEWTONPHYSICS_API FRammsNewtonLinkDescription
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	FName LinkName;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	FName ComponentName;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	FName ParentLinkName;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	bool bKinematic = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	bool bTreatAsTerrain = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton", meta = (ClampMin = "0.0"))
	float MassKg = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton", meta = (ClampMin = "0.0"))
	float LinearDamping = 0.01f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton", meta = (ClampMin = "0.0"))
	float AngularDamping = 0.01f;
};

USTRUCT(BlueprintType)
struct RAMMSNEWTONPHYSICS_API FRammsNewtonJointDescription
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	FName JointName;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	FName ParentLinkName;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	FName ChildLinkName;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	ERammsNewtonJointType JointType = ERammsNewtonJointType::Revolute;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	ERammsNewtonJointDriveMode DriveMode = ERammsNewtonJointDriveMode::PositionControl;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	FVector LocalAxis = FVector::ForwardVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	bool bUseExplicitJointFrames = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton", meta = (EditCondition = "bUseExplicitJointFrames"))
	FVector AxisInParentFrame = FVector::ForwardVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton", meta = (EditCondition = "bUseExplicitJointFrames"))
	FTransform ParentAnchorTransform = FTransform::Identity;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton", meta = (EditCondition = "bUseExplicitJointFrames"))
	FTransform ChildAnchorTransform = FTransform::Identity;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	FName ConstraintName;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	FName BoneName;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	bool bUseLimits = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton", meta = (EditCondition = "bUseLimits"))
	float MinLimitDegrees = -180.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton", meta = (EditCondition = "bUseLimits"))
	float MaxLimitDegrees = 180.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton", meta = (ClampMin = "0.0"))
	float MaxEffort = 0.0f;
};

USTRUCT(BlueprintType)
struct RAMMSNEWTONPHYSICS_API FRammsNewtonRobotDescription
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	FName RobotName;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	bool bTreatAsSingleCoupledSystem = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	bool bIncludeGroundInteraction = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	bool bIncludeManipulatedObjects = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	TArray<FRammsNewtonLinkDescription> Links;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	TArray<FRammsNewtonJointDescription> Joints;
};

USTRUCT(BlueprintType)
struct RAMMSNEWTONPHYSICS_API FRammsNewtonMaterialDescription
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton", meta = (ClampMin = "0.0"))
	float Density = 1000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton", meta = (ClampMin = "0.0"))
	float ContactElasticStiffness = 10000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton", meta = (ClampMin = "0.0"))
	float ContactDamping = 1000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton", meta = (ClampMin = "0.0"))
	float FrictionDamping = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton", meta = (ClampMin = "0.0"))
	float AdhesionDistance = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton", meta = (ClampMin = "0.0"))
	float Friction = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton", meta = (ClampMin = "0.0"))
	float Restitution = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton", meta = (ClampMin = "0.0"))
	float TorsionalFriction = 0.005f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton", meta = (ClampMin = "0.0"))
	float RollingFriction = 0.0001f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton", meta = (ClampMin = "0.0"))
	float CollisionMarginCm = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	bool bSolid = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton", meta = (ClampMin = "0"))
	int32 CollisionGroup = 1;
};

USTRUCT(BlueprintType)
struct RAMMSNEWTONPHYSICS_API FRammsNewtonComponentOverride
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	FName ComponentName;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	bool bOverrideCollisionGeometry = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton", meta = (EditCondition = "bOverrideCollisionGeometry"))
	ERammsNewtonCollisionGeometryMode CollisionGeometryMode = ERammsNewtonCollisionGeometryMode::Auto;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	bool bOverrideMaterial = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton", meta = (EditCondition = "bOverrideMaterial"))
	FRammsNewtonMaterialDescription Material;
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
	bool bTreatManagedPrimitivesAsDynamic = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	ERammsNewtonCollisionGeometryMode DefaultCollisionGeometryMode = ERammsNewtonCollisionGeometryMode::Auto;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	FRammsNewtonMaterialDescription DefaultMaterial;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	FName RootPrimitiveComponentName;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	FName SkeletalMeshComponentName;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	TArray<FName> IncludedPrimitiveComponents;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	TArray<FName> ExcludedPrimitiveComponents;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	TArray<FRammsNewtonComponentOverride> ComponentOverrides;
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
	bool bRuntimeLibraryLoaded = false;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Newton")
	bool bSourceCheckoutDetected = false;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Newton")
	bool bHeadersDetected = false;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Newton")
	FString RuntimeLibraryPath;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Newton")
	FString Summary;
};

USTRUCT(BlueprintType)
struct RAMMSNEWTONPHYSICS_API FRammsNewtonNativeWorldStatus
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Newton")
	bool bBackendInitialized = false;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Newton")
	bool bUsingBuiltInAdapter = false;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Newton")
	bool bUsingPythonBridge = false;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Newton")
	bool bWorldCreated = false;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Newton")
	bool bHasWorldLifecycleExports = false;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Newton")
	bool bHasBodySyncExports = false;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Newton")
	int32 RegisteredBridgeCount = 0;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Newton")
	int32 RegisteredBodyCount = 0;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Newton")
	int64 StepCount = 0;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Newton")
	FString RuntimeSummary;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Newton")
	FString LastError;
};
