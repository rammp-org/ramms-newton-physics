// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsNewtonNativeBackend.h"

#include "Async/Async.h"
#include "Components/BoxComponent.h"
#include "Components/CapsuleComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Components/SphereComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Containers/StringConv.h"
#include "Dom/JsonObject.h"
#include "Engine/StaticMesh.h"
#include "GameFramework/Actor.h"
#include "Interfaces/IPluginManager.h"
#include "Policies/CondensedJsonPrintPolicy.h"
#include "RammsNewtonPhysicsComponent.h"
#include "RammsNewtonPhysicsModule.h"
#include "RammsNewtonPhysicsSettings.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Misc/ScopeLock.h"
#include "StaticMeshResources.h"

#if PLATFORM_WINDOWS
	#include "Windows/WindowsHWrapper.h"
#endif

DEFINE_LOG_CATEGORY_STATIC(LogRammsNewtonNativeBackend, Log, All);

namespace RammsNewtonNative
{
	using FCreateWorldFn = void* (*)(const FRammsNewtonNativeWorldCreateDesc*);
	using FDestroyWorldFn = void (*)(void*);
	using FStepWorldFn = void (*)(void*, float);
	using FCreateBodyFn = uint64 (*)(void*, const FRammsNewtonNativeBodyCreateDesc*);
	using FDestroyBodyFn = void (*)(void*, uint64);
	using FSetBodyTransformFn = bool (*)(void*, uint64, const FRammsNewtonNativeTransform*);
	using FGetBodyTransformFn = bool (*)(void*, uint64, FRammsNewtonNativeTransform*);

	static constexpr TCHAR CreateWorldExport[] = TEXT("RammsNewtonCreateWorld");
	static constexpr TCHAR DestroyWorldExport[] = TEXT("RammsNewtonDestroyWorld");
	static constexpr TCHAR StepWorldExport[] = TEXT("RammsNewtonStepWorld");
	static constexpr TCHAR CreateBodyExport[] = TEXT("RammsNewtonCreateBody");
	static constexpr TCHAR DestroyBodyExport[] = TEXT("RammsNewtonDestroyBody");
	static constexpr TCHAR SetBodyTransformExport[] = TEXT("RammsNewtonSetBodyTransform");
	static constexpr TCHAR GetBodyTransformExport[] = TEXT("RammsNewtonGetBodyTransform");

	struct FBuiltInBodyState
	{
		uint64						Id = 0;
		FString						Name;
		FString						OwnerName;
		float						MassKg = 1.0f;
		bool						bKinematic = false;
		FRammsNewtonNativeTransform Transform;
	};

	struct FBuiltInWorldState
	{
		FRammsNewtonNativeWorldCreateDesc CreateDesc;
		TMap<uint64, FBuiltInBodyState>	  Bodies;
		uint64							  NextBodyId = 1;
		float							  SimulatedTimeSeconds = 0.0f;
	};

	static FRammsNewtonNativeVec3 ToNativeVec3(const FVector& Vector)
	{
		return { static_cast<float>(Vector.X), static_cast<float>(Vector.Y), static_cast<float>(Vector.Z) };
	}

	static FRammsNewtonNativeQuat ToNativeQuat(const FQuat& Quat)
	{
		return { static_cast<float>(Quat.X), static_cast<float>(Quat.Y), static_cast<float>(Quat.Z), static_cast<float>(Quat.W) };
	}

	static FRammsNewtonNativeTransform ToNativeTransform(const FTransform& Transform)
	{
		FRammsNewtonNativeTransform NativeTransform;
		NativeTransform.TranslationCm = ToNativeVec3(Transform.GetLocation());
		NativeTransform.Rotation = ToNativeQuat(Transform.GetRotation());
		NativeTransform.Scale3D = ToNativeVec3(Transform.GetScale3D());
		return NativeTransform;
	}

	static FTransform ToUnrealTransform(const FRammsNewtonNativeTransform& Transform)
	{
		return FTransform(
			FQuat(Transform.Rotation.X, Transform.Rotation.Y, Transform.Rotation.Z, Transform.Rotation.W),
			FVector(Transform.TranslationCm.X, Transform.TranslationCm.Y, Transform.TranslationCm.Z),
			FVector(Transform.Scale3D.X, Transform.Scale3D.Y, Transform.Scale3D.Z));
	}

	static FString ShapeTypeToString(const ERammsNewtonNativeShapeType ShapeType)
	{
		switch (ShapeType)
		{
			case ERammsNewtonNativeShapeType::Mesh:
				return TEXT("mesh");
			case ERammsNewtonNativeShapeType::ConvexHull:
				return TEXT("convex_hull");
			case ERammsNewtonNativeShapeType::Sphere:
				return TEXT("sphere");
			case ERammsNewtonNativeShapeType::Capsule:
				return TEXT("capsule");
			case ERammsNewtonNativeShapeType::Box:
				return TEXT("box");
			case ERammsNewtonNativeShapeType::Unknown:
			default:
				return TEXT("unknown");
		}
	}

	static TSharedRef<FJsonObject> MakeVec3Json(const FRammsNewtonNativeVec3& Vector)
	{
		TSharedRef<FJsonObject> Object = MakeShared<FJsonObject>();
		Object->SetNumberField(TEXT("x"), Vector.X);
		Object->SetNumberField(TEXT("y"), Vector.Y);
		Object->SetNumberField(TEXT("z"), Vector.Z);
		return Object;
	}

	static TSharedRef<FJsonObject> MakeQuatJson(const FRammsNewtonNativeQuat& Quat)
	{
		TSharedRef<FJsonObject> Object = MakeShared<FJsonObject>();
		Object->SetNumberField(TEXT("x"), Quat.X);
		Object->SetNumberField(TEXT("y"), Quat.Y);
		Object->SetNumberField(TEXT("z"), Quat.Z);
		Object->SetNumberField(TEXT("w"), Quat.W);
		return Object;
	}

	static TSharedRef<FJsonObject> MakeTransformJson(const FRammsNewtonNativeTransform& Transform)
	{
		TSharedRef<FJsonObject> Object = MakeShared<FJsonObject>();
		Object->SetObjectField(TEXT("translation_cm"), MakeVec3Json(Transform.TranslationCm));
		Object->SetObjectField(TEXT("rotation"), MakeQuatJson(Transform.Rotation));
		Object->SetObjectField(TEXT("scale3d"), MakeVec3Json(Transform.Scale3D));
		return Object;
	}

	static TSharedRef<FJsonObject> MakeShapeJson(const FRammsNewtonNativeBodyShapeDesc& Shape)
	{
		TSharedRef<FJsonObject> Object = MakeShared<FJsonObject>();
		Object->SetStringField(TEXT("type"), ShapeTypeToString(Shape.ShapeType));
		Object->SetObjectField(TEXT("half_extents_cm"), MakeVec3Json(Shape.HalfExtentsCm));
		Object->SetNumberField(TEXT("radius_cm"), Shape.RadiusCm);
		Object->SetNumberField(TEXT("half_height_cm"), Shape.HalfHeightCm);
		if (Shape.MeshVerticesCm && Shape.MeshVertexCount > 0)
		{
			TArray<TSharedPtr<FJsonValue>> VertexValues;
			VertexValues.Reserve(Shape.MeshVertexCount * 3);
			for (int32 VertexIndex = 0; VertexIndex < Shape.MeshVertexCount * 3; ++VertexIndex)
			{
				VertexValues.Add(MakeShared<FJsonValueNumber>(Shape.MeshVerticesCm[VertexIndex]));
			}

			Object->SetNumberField(TEXT("vertex_count"), Shape.MeshVertexCount);
			Object->SetArrayField(TEXT("vertices_cm"), VertexValues);
		}

		if (Shape.MeshIndices && Shape.MeshIndexCount > 0)
		{
			TArray<TSharedPtr<FJsonValue>> IndexValues;
			IndexValues.Reserve(Shape.MeshIndexCount);
			for (int32 Index = 0; Index < Shape.MeshIndexCount; ++Index)
			{
				IndexValues.Add(MakeShared<FJsonValueNumber>(Shape.MeshIndices[Index]));
			}

			Object->SetArrayField(TEXT("indices"), IndexValues);
		}
		return Object;
	}

	static TSharedRef<FJsonObject> MakeMaterialJson(const FRammsNewtonNativeBodyMaterialDesc& Material)
	{
		TSharedRef<FJsonObject> Object = MakeShared<FJsonObject>();
		Object->SetNumberField(TEXT("density"), Material.Density);
		Object->SetNumberField(TEXT("ke"), Material.ContactElasticStiffness);
		Object->SetNumberField(TEXT("kd"), Material.ContactDamping);
		Object->SetNumberField(TEXT("kf"), Material.FrictionDamping);
		Object->SetNumberField(TEXT("ka"), Material.AdhesionDistance);
		Object->SetNumberField(TEXT("mu"), Material.Friction);
		Object->SetNumberField(TEXT("restitution"), Material.Restitution);
		Object->SetNumberField(TEXT("mu_torsional"), Material.TorsionalFriction);
		Object->SetNumberField(TEXT("mu_rolling"), Material.RollingFriction);
		Object->SetNumberField(TEXT("margin_cm"), Material.CollisionMarginCm);
		Object->SetBoolField(TEXT("is_solid"), Material.bSolid);
		Object->SetNumberField(TEXT("collision_group"), Material.CollisionGroup);
		return Object;
	}

	static bool ReadVec3Json(const TSharedPtr<FJsonObject>& Object, const TCHAR* FieldName, FRammsNewtonNativeVec3& OutVector)
	{
		if (!Object.IsValid())
		{
			return false;
		}

		const TSharedPtr<FJsonObject>* VectorObject = nullptr;
		if (!Object->TryGetObjectField(FieldName, VectorObject) || !VectorObject || !VectorObject->IsValid())
		{
			return false;
		}

		double X = 0.0;
		double Y = 0.0;
		double Z = 0.0;
		if (!(*VectorObject)->TryGetNumberField(TEXT("x"), X)
			|| !(*VectorObject)->TryGetNumberField(TEXT("y"), Y)
			|| !(*VectorObject)->TryGetNumberField(TEXT("z"), Z))
		{
			return false;
		}

		OutVector.X = static_cast<float>(X);
		OutVector.Y = static_cast<float>(Y);
		OutVector.Z = static_cast<float>(Z);
		return true;
	}

	static bool ReadQuatJson(const TSharedPtr<FJsonObject>& Object, const TCHAR* FieldName, FRammsNewtonNativeQuat& OutQuat)
	{
		if (!Object.IsValid())
		{
			return false;
		}

		const TSharedPtr<FJsonObject>* QuatObject = nullptr;
		if (!Object->TryGetObjectField(FieldName, QuatObject) || !QuatObject || !QuatObject->IsValid())
		{
			return false;
		}

		double X = 0.0;
		double Y = 0.0;
		double Z = 0.0;
		double W = 1.0;
		if (!(*QuatObject)->TryGetNumberField(TEXT("x"), X)
			|| !(*QuatObject)->TryGetNumberField(TEXT("y"), Y)
			|| !(*QuatObject)->TryGetNumberField(TEXT("z"), Z)
			|| !(*QuatObject)->TryGetNumberField(TEXT("w"), W))
		{
			return false;
		}

		OutQuat.X = static_cast<float>(X);
		OutQuat.Y = static_cast<float>(Y);
		OutQuat.Z = static_cast<float>(Z);
		OutQuat.W = static_cast<float>(W);
		return true;
	}

	static bool ReadTransformJson(const TSharedPtr<FJsonObject>& Object, FRammsNewtonNativeTransform& OutTransform)
	{
		return ReadVec3Json(Object, TEXT("translation_cm"), OutTransform.TranslationCm)
			&& ReadQuatJson(Object, TEXT("rotation"), OutTransform.Rotation)
			&& ReadVec3Json(Object, TEXT("scale3d"), OutTransform.Scale3D);
	}

	static FRammsNewtonNativeBodyShapeDesc MakeShapeDesc(const UPrimitiveComponent& PrimitiveComponent)
	{
		FRammsNewtonNativeBodyShapeDesc Shape;

		if (const UBoxComponent* BoxComponent = Cast<UBoxComponent>(&PrimitiveComponent))
		{
			Shape.ShapeType = ERammsNewtonNativeShapeType::Box;
			Shape.HalfExtentsCm = ToNativeVec3(BoxComponent->GetScaledBoxExtent());
			return Shape;
		}

		if (const USphereComponent* SphereComponent = Cast<USphereComponent>(&PrimitiveComponent))
		{
			const float RadiusCm = SphereComponent->GetScaledSphereRadius();
			Shape.ShapeType = ERammsNewtonNativeShapeType::Sphere;
			Shape.RadiusCm = RadiusCm;
			Shape.HalfExtentsCm = { RadiusCm, RadiusCm, RadiusCm };
			return Shape;
		}

		if (const UCapsuleComponent* CapsuleComponent = Cast<UCapsuleComponent>(&PrimitiveComponent))
		{
			const float RadiusCm = CapsuleComponent->GetScaledCapsuleRadius();
			const float HalfHeightCm = CapsuleComponent->GetScaledCapsuleHalfHeight();
			Shape.ShapeType = ERammsNewtonNativeShapeType::Capsule;
			Shape.RadiusCm = RadiusCm;
			Shape.HalfHeightCm = HalfHeightCm;
			Shape.HalfExtentsCm = { RadiusCm, RadiusCm, HalfHeightCm };
			return Shape;
		}

		const FVector HalfExtentsCm = PrimitiveComponent.Bounds.BoxExtent.ComponentMax(FVector(1.0f, 1.0f, 1.0f));
		Shape.ShapeType = ERammsNewtonNativeShapeType::Box;
		Shape.HalfExtentsCm = ToNativeVec3(HalfExtentsCm);
		return Shape;
	}

	static FRammsNewtonNativeBodyMaterialDesc ToNativeMaterialDesc(const FRammsNewtonMaterialDescription& Material)
	{
		FRammsNewtonNativeBodyMaterialDesc NativeMaterial;
		NativeMaterial.Density = Material.Density;
		NativeMaterial.ContactElasticStiffness = Material.ContactElasticStiffness;
		NativeMaterial.ContactDamping = Material.ContactDamping;
		NativeMaterial.FrictionDamping = Material.FrictionDamping;
		NativeMaterial.AdhesionDistance = Material.AdhesionDistance;
		NativeMaterial.Friction = Material.Friction;
		NativeMaterial.Restitution = Material.Restitution;
		NativeMaterial.TorsionalFriction = Material.TorsionalFriction;
		NativeMaterial.RollingFriction = Material.RollingFriction;
		NativeMaterial.CollisionMarginCm = Material.CollisionMarginCm;
		NativeMaterial.bSolid = Material.bSolid;
		NativeMaterial.CollisionGroup = Material.CollisionGroup;
		return NativeMaterial;
	}

	static const FRammsNewtonComponentOverride* FindComponentOverride(
		const FRammsNewtonBridgeDescription& BridgeDescription,
		const FName							 ComponentName)
	{
		for (const FRammsNewtonComponentOverride& ComponentOverride : BridgeDescription.ComponentOverrides)
		{
			if (ComponentOverride.ComponentName == ComponentName)
			{
				return &ComponentOverride;
			}
		}

		return nullptr;
	}

	static FRammsNewtonMaterialDescription ResolveMaterialDescription(
		const URammsNewtonPhysicsComponent& Bridge,
		const UPrimitiveComponent&			PrimitiveComponent)
	{
		FRammsNewtonMaterialDescription Material = Bridge.BridgeDescription.DefaultMaterial;
		if (const FRammsNewtonComponentOverride* ComponentOverride =
				FindComponentOverride(Bridge.BridgeDescription, PrimitiveComponent.GetFName()))
		{
			if (ComponentOverride->bOverrideMaterial)
			{
				Material = ComponentOverride->Material;
			}
		}

		return Material;
	}

	static ERammsNewtonCollisionGeometryMode ResolveCollisionGeometryMode(
		const URammsNewtonPhysicsComponent& Bridge,
		const UPrimitiveComponent&			PrimitiveComponent,
		const bool							bTreatAsDynamicInNewton)
	{
		if (const FRammsNewtonComponentOverride* ComponentOverride =
				FindComponentOverride(Bridge.BridgeDescription, PrimitiveComponent.GetFName()))
		{
			if (ComponentOverride->bOverrideCollisionGeometry)
			{
				return ComponentOverride->CollisionGeometryMode;
			}
		}

		if (Bridge.BridgeDescription.DefaultCollisionGeometryMode != ERammsNewtonCollisionGeometryMode::Auto)
		{
			return Bridge.BridgeDescription.DefaultCollisionGeometryMode;
		}

		if (Cast<const UStaticMeshComponent>(&PrimitiveComponent))
		{
			return bTreatAsDynamicInNewton
				? ERammsNewtonCollisionGeometryMode::ConvexHull
				: ERammsNewtonCollisionGeometryMode::TriangleMesh;
		}

		return ERammsNewtonCollisionGeometryMode::PrimitiveApproximation;
	}

	static bool ExtractStaticMeshGeometry(
		const UStaticMeshComponent& StaticMeshComponent,
		TArray<float>&				OutVerticesCm,
		TArray<int32>&				OutIndices)
	{
		const UStaticMesh* StaticMesh = StaticMeshComponent.GetStaticMesh();
		if (!StaticMesh)
		{
			return false;
		}

		const FStaticMeshRenderData* RenderData = StaticMesh->GetRenderData();
		if (!RenderData || RenderData->LODResources.Num() == 0)
		{
			return false;
		}

		const FStaticMeshLODResources& LODResources = RenderData->LODResources[0];
		const uint32				   VertexCount = LODResources.VertexBuffers.PositionVertexBuffer.GetNumVertices();
		const FIndexArrayView		   IndexArray = LODResources.IndexBuffer.GetArrayView();
		if (VertexCount == 0 || IndexArray.Num() < 3)
		{
			return false;
		}

		const FVector Scale3D = StaticMeshComponent.GetComponentTransform().GetScale3D().GetAbs();
		OutVerticesCm.SetNumUninitialized(static_cast<int32>(VertexCount) * 3);
		for (uint32 VertexIndex = 0; VertexIndex < VertexCount; ++VertexIndex)
		{
			const FVector3f Position = LODResources.VertexBuffers.PositionVertexBuffer.VertexPosition(VertexIndex);
			const int32		BaseIndex = static_cast<int32>(VertexIndex) * 3;
			OutVerticesCm[BaseIndex] = Position.X * Scale3D.X;
			OutVerticesCm[BaseIndex + 1] = Position.Y * Scale3D.Y;
			OutVerticesCm[BaseIndex + 2] = Position.Z * Scale3D.Z;
		}

		OutIndices.SetNumUninitialized(IndexArray.Num());
		for (int32 Index = 0; Index < IndexArray.Num(); Index += 3)
		{
			if (Index + 2 >= IndexArray.Num())
			{
				break;
			}

			// UE static meshes are authored in a different handedness convention than
			// the Newton mesh collision path expects, so flip winding on export to keep
			// outward-facing normals and avoid back-face contact culling.
			OutIndices[Index] = static_cast<int32>(IndexArray[Index]);
			OutIndices[Index + 1] = static_cast<int32>(IndexArray[Index + 2]);
			OutIndices[Index + 2] = static_cast<int32>(IndexArray[Index + 1]);
		}

		return true;
	}

	static void* BuiltInCreateWorld(const FRammsNewtonNativeWorldCreateDesc* CreateDesc)
	{
		FBuiltInWorldState* WorldState = new FBuiltInWorldState();
		if (CreateDesc)
		{
			WorldState->CreateDesc = *CreateDesc;
		}
		return WorldState;
	}

	static void BuiltInDestroyWorld(void* WorldHandle)
	{
		delete static_cast<FBuiltInWorldState*>(WorldHandle);
	}

	static void BuiltInStepWorld(void* WorldHandle, float FixedStepSeconds)
	{
		if (FBuiltInWorldState* WorldState = static_cast<FBuiltInWorldState*>(WorldHandle))
		{
			WorldState->SimulatedTimeSeconds += FixedStepSeconds;
		}
	}

	static uint64 BuiltInCreateBody(void* WorldHandle, const FRammsNewtonNativeBodyCreateDesc* BodyDesc)
	{
		FBuiltInWorldState* WorldState = static_cast<FBuiltInWorldState*>(WorldHandle);
		if (!WorldState || !BodyDesc)
		{
			return 0;
		}

		const uint64	   BodyId = WorldState->NextBodyId++;
		FBuiltInBodyState& BodyState = WorldState->Bodies.Add(BodyId);
		BodyState.Id = BodyId;
		BodyState.Name = BodyDesc->NameAnsi ? UTF8_TO_TCHAR(BodyDesc->NameAnsi) : TEXT("");
		BodyState.OwnerName = BodyDesc->OwnerNameAnsi ? UTF8_TO_TCHAR(BodyDesc->OwnerNameAnsi) : TEXT("");
		BodyState.MassKg = BodyDesc->MassKg;
		BodyState.bKinematic = BodyDesc->bKinematic;
		BodyState.Transform = BodyDesc->Transform;
		return BodyId;
	}

	static void BuiltInDestroyBody(void* WorldHandle, uint64 BodyId)
	{
		if (FBuiltInWorldState* WorldState = static_cast<FBuiltInWorldState*>(WorldHandle))
		{
			WorldState->Bodies.Remove(BodyId);
		}
	}

	static bool BuiltInSetBodyTransform(void* WorldHandle, uint64 BodyId, const FRammsNewtonNativeTransform* Transform)
	{
		FBuiltInWorldState* WorldState = static_cast<FBuiltInWorldState*>(WorldHandle);
		if (!WorldState || !Transform)
		{
			return false;
		}

		if (FBuiltInBodyState* BodyState = WorldState->Bodies.Find(BodyId))
		{
			BodyState->Transform = *Transform;
			return true;
		}

		return false;
	}

	static bool BuiltInGetBodyTransform(void* WorldHandle, uint64 BodyId, FRammsNewtonNativeTransform* Transform)
	{
		FBuiltInWorldState* WorldState = static_cast<FBuiltInWorldState*>(WorldHandle);
		if (!WorldState || !Transform)
		{
			return false;
		}

		if (const FBuiltInBodyState* BodyState = WorldState->Bodies.Find(BodyId))
		{
			*Transform = BodyState->Transform;
			return true;
		}

		return false;
	}
} // namespace RammsNewtonNative

struct FRammsNewtonNativeBackend::FNativeExports
{
	RammsNewtonNative::FCreateWorldFn	   CreateWorld = nullptr;
	RammsNewtonNative::FDestroyWorldFn	   DestroyWorld = nullptr;
	RammsNewtonNative::FStepWorldFn		   StepWorld = nullptr;
	RammsNewtonNative::FCreateBodyFn	   CreateBody = nullptr;
	RammsNewtonNative::FDestroyBodyFn	   DestroyBody = nullptr;
	RammsNewtonNative::FSetBodyTransformFn SetBodyTransform = nullptr;
	RammsNewtonNative::FGetBodyTransformFn GetBodyTransform = nullptr;

	bool HasWorldLifecycle() const
	{
		return CreateWorld && DestroyWorld && StepWorld;
	}

	bool HasBodySync() const
	{
		return CreateBody && DestroyBody && SetBodyTransform && GetBodyTransform;
	}
};

FRammsNewtonNativeBackend::FRammsNewtonNativeBackend()
{
	Exports = new FNativeExports();
}

FRammsNewtonNativeBackend::~FRammsNewtonNativeBackend()
{
	Shutdown();
	delete Exports;
	Exports = nullptr;
}

bool FRammsNewtonNativeBackend::Initialize(const URammsNewtonPhysicsSettings& Settings, const FRammsNewtonBackendStatus& BackendStatus)
{
	Shutdown();
	ResetWorldStatus();
	WorldStatus.RuntimeSummary = BackendStatus.Summary;
	PythonRequestTimeoutSeconds = Settings.PythonRequestTimeoutSeconds;

	if (!BackendStatus.bRuntimeReady)
	{
		SetLastError(BackendStatus.Summary);
		return false;
	}

	bool bBackendBound = false;
	if (BackendStatus.Mode == ERammsNewtonBackendMode::ExternalPythonBridge)
	{
		if (InitializePythonBridge(Settings))
		{
			bUsePythonBridge = true;
			WorldStatus.bUsingPythonBridge = true;
			WorldStatus.bHasWorldLifecycleExports = true;
			WorldStatus.bHasBodySyncExports = true;
			bBackendBound = true;
		}
		else if (Settings.bAllowBuiltInAdapterFallback)
		{
			UE_LOG(
				LogRammsNewtonNativeBackend,
				Warning,
				TEXT("RammsNewtonPhysics: Python worker bridge failed to initialize. Falling back to the built-in RAMMS adapter."));
			ShutdownPythonBridge();
			bBackendBound = BindExports();
		}
	}
	else
	{
		bBackendBound = BindExports();
	}

	if (!bBackendBound)
	{
		return false;
	}

	FRammsNewtonNativeWorldCreateDesc WorldDesc;
	WorldDesc.FixedStepHz = Settings.FixedStepHz;
	WorldDesc.MaxSubstepsPerTick = Settings.MaxSubstepsPerTick;
	WorldDesc.GravityCmPerSecondSquared = RammsNewtonNative::ToNativeVec3(Settings.GravityCmPerSecondSquared);

	if (!CreateWorld(WorldDesc))
	{
		return false;
	}

	WorldStatus.bBackendInitialized = true;
	WorldStatus.bWorldCreated = true;
	WorldStatus.LastError.Reset();
	return true;
}

void FRammsNewtonNativeBackend::Shutdown()
{
	for (TPair<FObjectKey, FBridgeRecord>& Pair : BridgeRecords)
	{
		if (FBridgeRecord* BridgeRecord = &Pair.Value)
		{
			if (BridgeRecord->Bridge.IsValid())
			{
				BridgeRecord->Bridge->SetNativeRegistrationState(false, 0, TEXT("Newton native backend shut down."));
			}

			for (const FRammsNewtonNativeBodyRecord& BodyRecord : BridgeRecord->Bodies)
			{
				if (BodyRecord.NativeBodyId != 0)
				{
					DestroyBody(BodyRecord.NativeBodyId);
				}
			}
		}
	}
	BridgeRecords.Reset();

	DestroyWorld();
	ShutdownPythonBridge();
	ResetWorldStatus();
}

bool FRammsNewtonNativeBackend::IsInitialized() const
{
	return WorldStatus.bBackendInitialized && (NativeWorldHandle != nullptr || PythonWorkerWorldId != 0);
}

const FRammsNewtonNativeWorldStatus& FRammsNewtonNativeBackend::GetWorldStatus() const
{
	return WorldStatus;
}

bool FRammsNewtonNativeBackend::RegisterBridge(URammsNewtonPhysicsComponent& Bridge)
{
	if (!IsInitialized())
	{
		Bridge.SetNativeRegistrationState(false, 0, TEXT("Newton native world is not initialized."));
		return false;
	}

	if (!WorldStatus.bHasBodySyncExports)
	{
		SetLastError(TEXT("RammsNewtonPhysics: the active backend does not provide body sync support yet."));
		Bridge.SetNativeRegistrationState(false, 0, TEXT("Newton runtime is active, but body sync is not available yet."));
		return false;
	}

	const FObjectKey BridgeKey(&Bridge);
	if (BridgeRecords.Contains(BridgeKey))
	{
		return true;
	}

	TArray<UPrimitiveComponent*> ManagedPrimitiveComponents;
	Bridge.GetManagedPrimitiveComponents(ManagedPrimitiveComponents);
	if (ManagedPrimitiveComponents.Num() == 0)
	{
		Bridge.SetNativeRegistrationState(false, 0, TEXT("No managed primitive components were found for Newton registration."));
		return false;
	}

	FBridgeRecord& BridgeRecord = BridgeRecords.Add(BridgeKey);
	BridgeRecord.Bridge = &Bridge;

	for (UPrimitiveComponent* PrimitiveComponent : ManagedPrimitiveComponents)
	{
		if (!PrimitiveComponent)
		{
			continue;
		}

		FRammsNewtonNativeBodyRecord BodyRecord;
		if (CreateNativeBody(Bridge, *PrimitiveComponent, BodyRecord))
		{
			BridgeRecord.Bodies.Add(BodyRecord);
		}
	}

	UpdateRegistrationCounts();

	if (BridgeRecord.Bodies.Num() == 0)
	{
		Bridge.SetNativeRegistrationState(false, 0, TEXT("Managed components were found, but no native Newton bodies were created."));
		BridgeRecords.Remove(BridgeKey);
		UpdateRegistrationCounts();
		return false;
	}

	Bridge.SetNativeRegistrationState(
		true,
		BridgeRecord.Bodies.Num(),
		FString::Printf(TEXT("Registered %d managed component(s) with the Newton backend."), BridgeRecord.Bodies.Num()));
	return true;
}

void FRammsNewtonNativeBackend::UnregisterBridge(URammsNewtonPhysicsComponent& Bridge)
{
	const FObjectKey BridgeKey(&Bridge);
	FBridgeRecord*	 BridgeRecord = BridgeRecords.Find(BridgeKey);
	if (!BridgeRecord)
	{
		Bridge.SetNativeRegistrationState(false, 0, TEXT("Not registered with the native Newton backend."));
		return;
	}

	for (const FRammsNewtonNativeBodyRecord& BodyRecord : BridgeRecord->Bodies)
	{
		if (BodyRecord.NativeBodyId != 0)
		{
			DestroyBody(BodyRecord.NativeBodyId);
		}
	}

	Bridge.SetNativeRegistrationState(false, 0, TEXT("Removed from the native Newton backend."));
	BridgeRecords.Remove(BridgeKey);
	UpdateRegistrationCounts();
}

bool FRammsNewtonNativeBackend::StepSimulation(float FixedStepSeconds)
{
	if (!IsInitialized())
	{
		return false;
	}

	SyncAllBridgesToNative();
	if (!StepWorld(FixedStepSeconds))
	{
		return false;
	}
	if (!bUsePythonBridge)
	{
		SyncAllBridgesFromNative();
	}
	++WorldStatus.StepCount;
	return true;
}

void FRammsNewtonNativeBackend::SyncAllBridgesToNative()
{
	for (const TPair<FObjectKey, FBridgeRecord>& Pair : BridgeRecords)
	{
		const FBridgeRecord& BridgeRecord = Pair.Value;
		if (!BridgeRecord.Bridge.IsValid())
		{
			continue;
		}

		if (!BridgeRecord.Bridge->BridgeDescription.bPushUnrealPosesToSolver)
		{
			continue;
		}

		for (const FRammsNewtonNativeBodyRecord& BodyRecord : BridgeRecord.Bodies)
		{
			PushBodyTransformToNative(BodyRecord);
		}
	}
}

void FRammsNewtonNativeBackend::SyncAllBridgesFromNative()
{
	for (const TPair<FObjectKey, FBridgeRecord>& Pair : BridgeRecords)
	{
		const FBridgeRecord& BridgeRecord = Pair.Value;
		if (!BridgeRecord.Bridge.IsValid())
		{
			continue;
		}

		if (!BridgeRecord.Bridge->BridgeDescription.bPullSolverPosesBackToUnreal)
		{
			continue;
		}

		for (const FRammsNewtonNativeBodyRecord& BodyRecord : BridgeRecord.Bodies)
		{
			PullBodyTransformFromNative(BodyRecord);
		}
	}
}

bool FRammsNewtonNativeBackend::BindExports()
{
	check(Exports);
	*Exports = FNativeExports();
	bUsePythonBridge = false;

	FRammsNewtonPhysicsModule& Module = FRammsNewtonPhysicsModule::Get();
	Exports->CreateWorld = reinterpret_cast<RammsNewtonNative::FCreateWorldFn>(Module.GetRuntimeSymbol(RammsNewtonNative::CreateWorldExport));
	Exports->DestroyWorld = reinterpret_cast<RammsNewtonNative::FDestroyWorldFn>(Module.GetRuntimeSymbol(RammsNewtonNative::DestroyWorldExport));
	Exports->StepWorld = reinterpret_cast<RammsNewtonNative::FStepWorldFn>(Module.GetRuntimeSymbol(RammsNewtonNative::StepWorldExport));
	Exports->CreateBody = reinterpret_cast<RammsNewtonNative::FCreateBodyFn>(Module.GetRuntimeSymbol(RammsNewtonNative::CreateBodyExport));
	Exports->DestroyBody = reinterpret_cast<RammsNewtonNative::FDestroyBodyFn>(Module.GetRuntimeSymbol(RammsNewtonNative::DestroyBodyExport));
	Exports->SetBodyTransform = reinterpret_cast<RammsNewtonNative::FSetBodyTransformFn>(Module.GetRuntimeSymbol(RammsNewtonNative::SetBodyTransformExport));
	Exports->GetBodyTransform = reinterpret_cast<RammsNewtonNative::FGetBodyTransformFn>(Module.GetRuntimeSymbol(RammsNewtonNative::GetBodyTransformExport));

	if (!Exports->HasWorldLifecycle())
	{
		Exports->CreateWorld = &RammsNewtonNative::BuiltInCreateWorld;
		Exports->DestroyWorld = &RammsNewtonNative::BuiltInDestroyWorld;
		Exports->StepWorld = &RammsNewtonNative::BuiltInStepWorld;
		Exports->CreateBody = &RammsNewtonNative::BuiltInCreateBody;
		Exports->DestroyBody = &RammsNewtonNative::BuiltInDestroyBody;
		Exports->SetBodyTransform = &RammsNewtonNative::BuiltInSetBodyTransform;
		Exports->GetBodyTransform = &RammsNewtonNative::BuiltInGetBodyTransform;
		WorldStatus.bUsingBuiltInAdapter = true;
	}

	WorldStatus.bHasWorldLifecycleExports = Exports->HasWorldLifecycle();
	WorldStatus.bHasBodySyncExports = Exports->HasBodySync();

	if (!WorldStatus.bHasWorldLifecycleExports)
	{
		SetLastError(TEXT("RammsNewtonPhysics: runtime library is loaded, but required world lifecycle exports were not found. Expected adapter exports such as RammsNewtonCreateWorld/RammsNewtonStepWorld."));
		return false;
	}

	return true;
}

bool FRammsNewtonNativeBackend::InitializePythonBridge(const URammsNewtonPhysicsSettings& Settings)
{
	ShutdownPythonBridge();

	PythonExecutablePath = ResolvePythonExecutablePath(Settings);
	PythonWorkerScriptPath = ResolvePythonWorkerScriptPath(Settings);
	if (PythonExecutablePath.IsEmpty() || PythonWorkerScriptPath.IsEmpty())
	{
		SetLastError(TEXT("RammsNewtonPhysics: Python bridge is enabled, but the Python executable or worker script path is empty."));
		return false;
	}

	if (!FPaths::FileExists(PythonWorkerScriptPath))
	{
		SetLastError(FString::Printf(TEXT("RammsNewtonPhysics: Python worker script was not found at '%s'."), *PythonWorkerScriptPath));
		return false;
	}

	if (!FPlatformProcess::CreatePipe(PythonStdOutReadPipe, PythonStdOutWritePipe)
		|| !FPlatformProcess::CreatePipe(PythonStdInReadPipe, PythonStdInWritePipe, true))
	{
		SetLastError(TEXT("RammsNewtonPhysics: failed to create pipes for the Python worker bridge."));
		ShutdownPythonBridge();
		return false;
	}

	const FString Arguments = FString::Printf(
		TEXT("-u \"%s\" %s"),
		*PythonWorkerScriptPath,
		*Settings.PythonExtraArguments);

	PythonWorkerProcess = FPlatformProcess::CreateProc(
		*PythonExecutablePath,
		*Arguments,
		false,
		true,
		true,
		nullptr,
		0,
		*GetPluginBaseDir(),
		PythonStdOutWritePipe,
		PythonStdInReadPipe);

	if (!PythonWorkerProcess.IsValid())
	{
		SetLastError(FString::Printf(TEXT("RammsNewtonPhysics: failed to launch Python worker '%s' with script '%s'."), *PythonExecutablePath, *PythonWorkerScriptPath));
		ShutdownPythonBridge();
		return false;
	}

#if PLATFORM_WINDOWS
	if (PythonStdOutWritePipe)
	{
		::CloseHandle(static_cast<HANDLE>(PythonStdOutWritePipe));
		PythonStdOutWritePipe = nullptr;
	}

	if (PythonStdInReadPipe)
	{
		::CloseHandle(static_cast<HANDLE>(PythonStdInReadPipe));
		PythonStdInReadPipe = nullptr;
	}
#endif

	StartPythonOutputReader();

	TSharedPtr<FJsonObject> Result;
	if (!SendPythonRequest(TEXT("ping"), MakeShared<FJsonObject>(), Result))
	{
		ShutdownPythonBridge();
		return false;
	}

	bool bNewtonAvailable = false;
	Result->TryGetBoolField(TEXT("newton_available"), bNewtonAvailable);
	if (!bNewtonAvailable)
	{
		FString ImportError;
		Result->TryGetStringField(TEXT("import_error"), ImportError);
		SetLastError(FString::Printf(TEXT("RammsNewtonPhysics: Python worker started, but the Newton Python package is not ready: %s"), *ImportError));
		ShutdownPythonBridge();
		return false;
	}

	FString NewtonVersion;
	Result->TryGetStringField(TEXT("newton_version"), NewtonVersion);
	WorldStatus.RuntimeSummary = FString::Printf(
		TEXT("%s Python worker bridge active (%s, Newton %s)."),
		*WorldStatus.RuntimeSummary,
		*PythonExecutablePath,
		NewtonVersion.IsEmpty() ? TEXT("unknown") : *NewtonVersion);
	return true;
}

void FRammsNewtonNativeBackend::ShutdownPythonBridge()
{
	if (PythonWorkerProcess.IsValid() && FPlatformProcess::IsProcRunning(PythonWorkerProcess))
	{
		if (PythonWorkerWorldId != 0)
		{
			DestroyWorld();
		}

		TSharedPtr<FJsonObject> UnusedResult;
		SendPythonRequest(TEXT("shutdown"), MakeShared<FJsonObject>(), UnusedResult);

		if (FPlatformProcess::IsProcRunning(PythonWorkerProcess))
		{
			FPlatformProcess::TerminateProc(PythonWorkerProcess, true);
		}
	}

	StopPythonOutputReader();

	if (PythonWorkerProcess.IsValid())
	{
		FPlatformProcess::CloseProc(PythonWorkerProcess);
		PythonWorkerProcess = FProcHandle();
	}

	if (PythonStdOutReadPipe || PythonStdOutWritePipe)
	{
		FPlatformProcess::ClosePipe(PythonStdOutReadPipe, PythonStdOutWritePipe);
		PythonStdOutReadPipe = nullptr;
		PythonStdOutWritePipe = nullptr;
	}

	if (PythonStdInReadPipe || PythonStdInWritePipe)
	{
		FPlatformProcess::ClosePipe(PythonStdInReadPipe, PythonStdInWritePipe);
		PythonStdInReadPipe = nullptr;
		PythonStdInWritePipe = nullptr;
	}

	PythonWorkerWorldId = 0;
	PendingPythonOutput.Reset();
	{
		FScopeLock Lock(&PythonResponseMutex);
		PythonResponses.Reset();
	}
	bUsePythonBridge = false;
}

void FRammsNewtonNativeBackend::ResetWorldStatus()
{
	WorldStatus = FRammsNewtonNativeWorldStatus();
	WorldStatus.RuntimeSummary.Reset();
}

void FRammsNewtonNativeBackend::UpdateRegistrationCounts()
{
	WorldStatus.RegisteredBridgeCount = 0;
	WorldStatus.RegisteredBodyCount = 0;

	for (const TPair<FObjectKey, FBridgeRecord>& Pair : BridgeRecords)
	{
		if (Pair.Value.Bridge.IsValid())
		{
			++WorldStatus.RegisteredBridgeCount;
			WorldStatus.RegisteredBodyCount += Pair.Value.Bodies.Num();
		}
	}
}

void FRammsNewtonNativeBackend::SetLastError(const FString& ErrorText)
{
	WorldStatus.LastError = ErrorText;
	if (!ErrorText.IsEmpty())
	{
		UE_LOG(LogRammsNewtonNativeBackend, Warning, TEXT("%s"), *ErrorText);
	}
}

FRammsNewtonNativeBackend::FBridgeRecord* FRammsNewtonNativeBackend::FindBridgeRecord(URammsNewtonPhysicsComponent& Bridge)
{
	return BridgeRecords.Find(FObjectKey(&Bridge));
}

const FRammsNewtonNativeBackend::FBridgeRecord* FRammsNewtonNativeBackend::FindBridgeRecord(const URammsNewtonPhysicsComponent& Bridge) const
{
	return BridgeRecords.Find(FObjectKey(const_cast<URammsNewtonPhysicsComponent*>(&Bridge)));
}

bool FRammsNewtonNativeBackend::CreateWorld(const FRammsNewtonNativeWorldCreateDesc& WorldDesc)
{
	if (bUsePythonBridge)
	{
		TSharedRef<FJsonObject> Params = MakeShared<FJsonObject>();
		Params->SetNumberField(TEXT("fixed_step_hz"), WorldDesc.FixedStepHz);
		Params->SetNumberField(TEXT("max_substeps_per_tick"), WorldDesc.MaxSubstepsPerTick);
		Params->SetObjectField(TEXT("gravity_cm_per_second_squared"), RammsNewtonNative::MakeVec3Json(WorldDesc.GravityCmPerSecondSquared));

		TSharedPtr<FJsonObject> Result;
		if (!SendPythonRequest(TEXT("create_world"), Params, Result))
		{
			return false;
		}

		double WorldId = 0.0;
		if (!Result->TryGetNumberField(TEXT("world_id"), WorldId))
		{
			SetLastError(TEXT("RammsNewtonPhysics: Python worker did not return a world_id."));
			return false;
		}

		PythonWorkerWorldId = static_cast<uint64>(WorldId);
		return PythonWorkerWorldId != 0;
	}

	if (!Exports || !Exports->CreateWorld)
	{
		SetLastError(TEXT("RammsNewtonPhysics: native backend is not bound to a world creation function."));
		return false;
	}

	NativeWorldHandle = Exports->CreateWorld(&WorldDesc);
	if (!NativeWorldHandle)
	{
		SetLastError(TEXT("RammsNewtonPhysics: native runtime exports were found, but world creation returned null."));
		return false;
	}

	return true;
}

void FRammsNewtonNativeBackend::DestroyWorld()
{
	if (bUsePythonBridge)
	{
		if (PythonWorkerWorldId != 0 && PythonWorkerProcess.IsValid())
		{
			TSharedRef<FJsonObject> Params = MakeShared<FJsonObject>();
			Params->SetNumberField(TEXT("world_id"), static_cast<double>(PythonWorkerWorldId));
			TSharedPtr<FJsonObject> UnusedResult;
			SendPythonRequest(TEXT("destroy_world"), Params, UnusedResult);
		}
		PythonWorkerWorldId = 0;
		return;
	}

	if (NativeWorldHandle && Exports && Exports->DestroyWorld)
	{
		Exports->DestroyWorld(NativeWorldHandle);
	}
	NativeWorldHandle = nullptr;
}

bool FRammsNewtonNativeBackend::StepWorld(float FixedStepSeconds)
{
	if (bUsePythonBridge)
	{
		if (PythonWorkerWorldId == 0)
		{
			return false;
		}

		TSharedRef<FJsonObject> Params = MakeShared<FJsonObject>();
		Params->SetNumberField(TEXT("world_id"), static_cast<double>(PythonWorkerWorldId));
		Params->SetNumberField(TEXT("fixed_step_seconds"), FixedStepSeconds);
		const TArray<uint64> PullBodyIds = GatherPythonPullBodyIds();
		if (PullBodyIds.Num() > 0)
		{
			TArray<TSharedPtr<FJsonValue>> PullBodyIdValues;
			PullBodyIdValues.Reserve(PullBodyIds.Num());
			for (const uint64 BodyId : PullBodyIds)
			{
				PullBodyIdValues.Add(MakeShared<FJsonValueNumber>(static_cast<double>(BodyId)));
			}
			Params->SetArrayField(TEXT("body_ids_to_sync"), PullBodyIdValues);
		}

		TSharedPtr<FJsonObject> Result;
		if (!SendPythonRequest(TEXT("step_world"), Params, Result))
		{
			return false;
		}

		return ApplyPythonStepTransforms(Result);
	}

	if (!Exports || !Exports->StepWorld)
	{
		return false;
	}

	Exports->StepWorld(NativeWorldHandle, FixedStepSeconds);
	return true;
}

uint64 FRammsNewtonNativeBackend::CreateBody(const FRammsNewtonNativeBodyCreateDesc& BodyDesc)
{
	if (bUsePythonBridge)
	{
		TSharedRef<FJsonObject> Params = MakeShared<FJsonObject>();
		Params->SetNumberField(TEXT("world_id"), static_cast<double>(PythonWorkerWorldId));
		Params->SetStringField(TEXT("name"), BodyDesc.NameAnsi ? UTF8_TO_TCHAR(BodyDesc.NameAnsi) : TEXT(""));
		Params->SetStringField(TEXT("owner_name"), BodyDesc.OwnerNameAnsi ? UTF8_TO_TCHAR(BodyDesc.OwnerNameAnsi) : TEXT(""));
		Params->SetNumberField(TEXT("mass_kg"), BodyDesc.MassKg);
		Params->SetBoolField(TEXT("kinematic"), BodyDesc.bKinematic);
		Params->SetObjectField(TEXT("transform"), RammsNewtonNative::MakeTransformJson(BodyDesc.Transform));
		Params->SetObjectField(TEXT("shape"), RammsNewtonNative::MakeShapeJson(BodyDesc.Shape));
		Params->SetObjectField(TEXT("material"), RammsNewtonNative::MakeMaterialJson(BodyDesc.Material));

		TSharedPtr<FJsonObject> Result;
		if (!SendPythonRequest(TEXT("create_body"), Params, Result))
		{
			return 0;
		}

		double BodyId = 0.0;
		return Result->TryGetNumberField(TEXT("body_id"), BodyId) ? static_cast<uint64>(BodyId) : 0;
	}

	return Exports ? Exports->CreateBody(NativeWorldHandle, &BodyDesc) : 0;
}

void FRammsNewtonNativeBackend::DestroyBody(uint64 BodyId)
{
	if (BodyId == 0)
	{
		return;
	}

	if (bUsePythonBridge)
	{
		TSharedRef<FJsonObject> Params = MakeShared<FJsonObject>();
		Params->SetNumberField(TEXT("world_id"), static_cast<double>(PythonWorkerWorldId));
		Params->SetNumberField(TEXT("body_id"), static_cast<double>(BodyId));
		TSharedPtr<FJsonObject> UnusedResult;
		SendPythonRequest(TEXT("destroy_body"), Params, UnusedResult);
		return;
	}

	if (NativeWorldHandle && Exports && Exports->DestroyBody)
	{
		Exports->DestroyBody(NativeWorldHandle, BodyId);
	}
}

bool FRammsNewtonNativeBackend::SetBodyTransform(uint64 BodyId, const FRammsNewtonNativeTransform& Transform)
{
	if (bUsePythonBridge)
	{
		TSharedRef<FJsonObject> Params = MakeShared<FJsonObject>();
		Params->SetNumberField(TEXT("world_id"), static_cast<double>(PythonWorkerWorldId));
		Params->SetNumberField(TEXT("body_id"), static_cast<double>(BodyId));
		Params->SetObjectField(TEXT("transform"), RammsNewtonNative::MakeTransformJson(Transform));

		TSharedPtr<FJsonObject> Result;
		if (!SendPythonRequest(TEXT("set_body_transform"), Params, Result))
		{
			return false;
		}

		bool bUpdated = false;
		return Result->TryGetBoolField(TEXT("updated"), bUpdated) && bUpdated;
	}

	return Exports && Exports->SetBodyTransform
		? Exports->SetBodyTransform(NativeWorldHandle, BodyId, &Transform)
		: false;
}

bool FRammsNewtonNativeBackend::GetBodyTransform(uint64 BodyId, FRammsNewtonNativeTransform& OutTransform)
{
	if (bUsePythonBridge)
	{
		TSharedRef<FJsonObject> Params = MakeShared<FJsonObject>();
		Params->SetNumberField(TEXT("world_id"), static_cast<double>(PythonWorkerWorldId));
		Params->SetNumberField(TEXT("body_id"), static_cast<double>(BodyId));

		TSharedPtr<FJsonObject> Result;
		if (!SendPythonRequest(TEXT("get_body_transform"), Params, Result))
		{
			return false;
		}

		const TSharedPtr<FJsonObject>* TransformObject = nullptr;
		return Result->TryGetObjectField(TEXT("transform"), TransformObject)
			&& TransformObject
			&& RammsNewtonNative::ReadTransformJson(*TransformObject, OutTransform);
	}

	return Exports && Exports->GetBodyTransform
		? Exports->GetBodyTransform(NativeWorldHandle, BodyId, &OutTransform)
		: false;
}

bool FRammsNewtonNativeBackend::CreateNativeBody(URammsNewtonPhysicsComponent& Bridge, UPrimitiveComponent& PrimitiveComponent, FRammsNewtonNativeBodyRecord& OutRecord)
{
	FTCHARToUTF8 ComponentNameUtf8(*PrimitiveComponent.GetName());
	FTCHARToUTF8 OwnerNameUtf8(*GetNameSafe(Bridge.GetOwner()));

	FRammsNewtonNativeBodyCreateDesc BodyDesc;
	BodyDesc.NameAnsi = ComponentNameUtf8.Get();
	BodyDesc.OwnerNameAnsi = OwnerNameUtf8.Get();
	const bool bTreatAsDynamicInNewton =
		Bridge.BridgeDescription.bTreatManagedPrimitivesAsDynamic || PrimitiveComponent.IsSimulatingPhysics();
	const ERammsNewtonCollisionGeometryMode CollisionGeometryMode =
		RammsNewtonNative::ResolveCollisionGeometryMode(Bridge, PrimitiveComponent, bTreatAsDynamicInNewton);
	BodyDesc.bKinematic = !bTreatAsDynamicInNewton;
	const ECollisionEnabled::Type CollisionEnabled = PrimitiveComponent.GetCollisionEnabled();
	const bool					  bPhysicsCollisionEnabled =
		CollisionEnabled == ECollisionEnabled::PhysicsOnly
		|| CollisionEnabled == ECollisionEnabled::QueryAndPhysics;
	BodyDesc.MassKg = 0.0f;
	if (!BodyDesc.bKinematic)
	{
		BodyDesc.MassKg = (PrimitiveComponent.IsSimulatingPhysics() && bPhysicsCollisionEnabled)
			? FMath::Max(PrimitiveComponent.GetMass(), 0.001f)
			: 1.0f;
	}
	BodyDesc.Transform = RammsNewtonNative::ToNativeTransform(PrimitiveComponent.GetComponentTransform());
	BodyDesc.Shape = RammsNewtonNative::MakeShapeDesc(PrimitiveComponent);
	BodyDesc.Material = RammsNewtonNative::ToNativeMaterialDesc(
		RammsNewtonNative::ResolveMaterialDescription(Bridge, PrimitiveComponent));

	TArray<float> MeshVerticesCm;
	TArray<int32> MeshIndices;
	if (CollisionGeometryMode != ERammsNewtonCollisionGeometryMode::PrimitiveApproximation)
	{
		if (const UStaticMeshComponent* StaticMeshComponent = Cast<UStaticMeshComponent>(&PrimitiveComponent))
		{
			if (RammsNewtonNative::ExtractStaticMeshGeometry(*StaticMeshComponent, MeshVerticesCm, MeshIndices))
			{
				BodyDesc.Shape.ShapeType =
					CollisionGeometryMode == ERammsNewtonCollisionGeometryMode::ConvexHull
					? ERammsNewtonNativeShapeType::ConvexHull
					: ERammsNewtonNativeShapeType::Mesh;
				BodyDesc.Shape.MeshVerticesCm = MeshVerticesCm.GetData();
				BodyDesc.Shape.MeshVertexCount = MeshVerticesCm.Num() / 3;
				BodyDesc.Shape.MeshIndices = MeshIndices.GetData();
				BodyDesc.Shape.MeshIndexCount = MeshIndices.Num();
			}
			else
			{
				BodyDesc.Shape = RammsNewtonNative::MakeShapeDesc(PrimitiveComponent);
			}
		}
		else
		{
			BodyDesc.Shape = RammsNewtonNative::MakeShapeDesc(PrimitiveComponent);
		}
	}
	else
	{
		BodyDesc.Shape = RammsNewtonNative::MakeShapeDesc(PrimitiveComponent);
	}

	const uint64 NativeBodyId = CreateBody(BodyDesc);
	if (NativeBodyId == 0)
	{
		return false;
	}

	OutRecord.NativeBodyId = NativeBodyId;
	OutRecord.ComponentName = PrimitiveComponent.GetFName();
	OutRecord.PrimitiveComponent = &PrimitiveComponent;
	return true;
}

bool FRammsNewtonNativeBackend::PushBodyTransformToNative(const FRammsNewtonNativeBodyRecord& BodyRecord)
{
	if (!BodyRecord.PrimitiveComponent.IsValid())
	{
		return false;
	}

	const FRammsNewtonNativeTransform NativeTransform =
		RammsNewtonNative::ToNativeTransform(BodyRecord.PrimitiveComponent->GetComponentTransform());
	return SetBodyTransform(BodyRecord.NativeBodyId, NativeTransform);
}

bool FRammsNewtonNativeBackend::PullBodyTransformFromNative(const FRammsNewtonNativeBodyRecord& BodyRecord)
{
	if (!BodyRecord.PrimitiveComponent.IsValid())
	{
		return false;
	}

	FRammsNewtonNativeTransform NativeTransform;
	if (!GetBodyTransform(BodyRecord.NativeBodyId, NativeTransform))
	{
		return false;
	}

	BodyRecord.PrimitiveComponent->SetWorldTransform(
		RammsNewtonNative::ToUnrealTransform(NativeTransform),
		false,
		nullptr,
		ETeleportType::TeleportPhysics);
	return true;
}

void FRammsNewtonNativeBackend::StartPythonOutputReader()
{
	StopPythonOutputReader();

	if (!PythonWorkerProcess.IsValid() || !PythonStdOutReadPipe)
	{
		return;
	}

	PythonResponseEvent = FPlatformProcess::GetSynchEventFromPool(false);
	bStopPythonOutputReader = false;
	PythonOutputReaderTask = Async(EAsyncExecution::Thread, [this]() {
		RunPythonOutputReader();
	});
}

void FRammsNewtonNativeBackend::StopPythonOutputReader()
{
	bStopPythonOutputReader = true;
	if (PythonResponseEvent)
	{
		PythonResponseEvent->Trigger();
	}

	if (PythonOutputReaderTask.IsValid())
	{
		PythonOutputReaderTask.Wait();
		PythonOutputReaderTask = TFuture<void>();
	}

	if (PythonResponseEvent)
	{
		FPlatformProcess::ReturnSynchEventToPool(PythonResponseEvent);
		PythonResponseEvent = nullptr;
	}
}

void FRammsNewtonNativeBackend::RunPythonOutputReader()
{
	FString BufferedOutput;

#if PLATFORM_WINDOWS
	const HANDLE StdOutHandle = static_cast<HANDLE>(PythonStdOutReadPipe);
	ANSICHAR	 Buffer[4096];
	while (!bStopPythonOutputReader && StdOutHandle)
	{
		DWORD	   BytesRead = 0;
		const BOOL bReadOk = ::ReadFile(StdOutHandle, Buffer, sizeof(Buffer), &BytesRead, nullptr);
		if (!bReadOk || BytesRead == 0)
		{
			break;
		}

		const FUTF8ToTCHAR Converted(Buffer, BytesRead);
		BufferedOutput.AppendChars(Converted.Get(), Converted.Length());

		int32 NewlineIndex = INDEX_NONE;
		while ((NewlineIndex = BufferedOutput.Find(TEXT("\n"))) != INDEX_NONE)
		{
			FString OutputLine = BufferedOutput.Left(NewlineIndex).TrimStartAndEnd();
			BufferedOutput.RightChopInline(NewlineIndex + 1, EAllowShrinking::No);
			if (!OutputLine.IsEmpty())
			{
				HandlePythonOutputLine(OutputLine);
			}
		}
	}
#else
	while (!bStopPythonOutputReader && PythonWorkerProcess.IsValid() && FPlatformProcess::IsProcRunning(PythonWorkerProcess))
	{
		const FString Chunk = FPlatformProcess::ReadPipe(PythonStdOutReadPipe);
		if (Chunk.IsEmpty())
		{
			FPlatformProcess::Sleep(0.001f);
			continue;
		}

		BufferedOutput += Chunk;
		int32 NewlineIndex = INDEX_NONE;
		while ((NewlineIndex = BufferedOutput.Find(TEXT("\n"))) != INDEX_NONE)
		{
			FString OutputLine = BufferedOutput.Left(NewlineIndex).TrimStartAndEnd();
			BufferedOutput.RightChopInline(NewlineIndex + 1, EAllowShrinking::No);
			if (!OutputLine.IsEmpty())
			{
				HandlePythonOutputLine(OutputLine);
			}
		}
	}
#endif

	if (!BufferedOutput.TrimStartAndEnd().IsEmpty())
	{
		HandlePythonOutputLine(BufferedOutput.TrimStartAndEnd());
	}

	if (PythonResponseEvent)
	{
		PythonResponseEvent->Trigger();
	}
}

void FRammsNewtonNativeBackend::HandlePythonOutputLine(const FString& OutputLine)
{
	if (!OutputLine.StartsWith(TEXT("{")))
	{
		return;
	}

	TSharedPtr<FJsonObject>			Response;
	const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(OutputLine);
	if (!FJsonSerializer::Deserialize(Reader, Response) || !Response.IsValid())
	{
		UE_LOG(LogRammsNewtonNativeBackend, Warning, TEXT("RammsNewtonPhysics: failed to parse Python worker response '%s'."), *OutputLine);
		return;
	}

	QueuePythonResponse(Response);
}

void FRammsNewtonNativeBackend::QueuePythonResponse(const TSharedPtr<FJsonObject>& Response)
{
	if (!Response.IsValid())
	{
		return;
	}

	double ResponseId = 0.0;
	if (!Response->TryGetNumberField(TEXT("id"), ResponseId))
	{
		return;
	}

	{
		FScopeLock Lock(&PythonResponseMutex);
		PythonResponses.Add(static_cast<uint64>(ResponseId), Response);
	}

	if (PythonResponseEvent)
	{
		PythonResponseEvent->Trigger();
	}
}

bool FRammsNewtonNativeBackend::WaitForPythonResponse(uint64 RequestId, TSharedPtr<FJsonObject>& OutResponse)
{
	const double StartTime = FPlatformTime::Seconds();
	while (FPlatformTime::Seconds() - StartTime <= PythonRequestTimeoutSeconds)
	{
		{
			FScopeLock Lock(&PythonResponseMutex);
			if (TSharedPtr<FJsonObject>* Response = PythonResponses.Find(RequestId))
			{
				OutResponse = *Response;
				PythonResponses.Remove(RequestId);
				return true;
			}
		}

		if (!PythonWorkerProcess.IsValid() || !FPlatformProcess::IsProcRunning(PythonWorkerProcess))
		{
			SetLastError(TEXT("RammsNewtonPhysics: Python worker bridge exited unexpectedly."));
			return false;
		}

		if (!PythonResponseEvent)
		{
			break;
		}

		const double RemainingSeconds = PythonRequestTimeoutSeconds - (FPlatformTime::Seconds() - StartTime);
		if (RemainingSeconds <= 0.0)
		{
			break;
		}

		const uint32 WaitMilliseconds = static_cast<uint32>(FMath::Max(1.0, RemainingSeconds * 1000.0));
		PythonResponseEvent->Wait(WaitMilliseconds);
	}

	SetLastError(FString::Printf(TEXT("RammsNewtonPhysics: timed out waiting for Python worker response to '%s'."), TEXT("request")));
	return false;
}

bool FRammsNewtonNativeBackend::ApplyPythonStepTransforms(const TSharedPtr<FJsonObject>& Result)
{
	if (!Result.IsValid())
	{
		return true;
	}

	const TArray<TSharedPtr<FJsonValue>>* BodyTransforms = nullptr;
	if (!Result->TryGetArrayField(TEXT("body_transforms"), BodyTransforms) || !BodyTransforms)
	{
		return true;
	}

	TMap<uint64, FRammsNewtonNativeTransform> TransformsByBodyId;
	for (const TSharedPtr<FJsonValue>& EntryValue : *BodyTransforms)
	{
		if (!EntryValue.IsValid())
		{
			continue;
		}

		const TSharedPtr<FJsonObject>* EntryObject = nullptr;
		if (!EntryValue->TryGetObject(EntryObject) || !EntryObject || !EntryObject->IsValid())
		{
			continue;
		}

		double						   BodyIdValue = 0.0;
		const TSharedPtr<FJsonObject>* TransformObject = nullptr;
		if (!(*EntryObject)->TryGetNumberField(TEXT("body_id"), BodyIdValue)
			|| !(*EntryObject)->TryGetObjectField(TEXT("transform"), TransformObject)
			|| !TransformObject)
		{
			continue;
		}

		FRammsNewtonNativeTransform Transform;
		if (RammsNewtonNative::ReadTransformJson(*TransformObject, Transform))
		{
			TransformsByBodyId.Add(static_cast<uint64>(BodyIdValue), Transform);
		}
	}

	for (const TPair<FObjectKey, FBridgeRecord>& Pair : BridgeRecords)
	{
		const FBridgeRecord& BridgeRecord = Pair.Value;
		if (!BridgeRecord.Bridge.IsValid() || !BridgeRecord.Bridge->BridgeDescription.bPullSolverPosesBackToUnreal)
		{
			continue;
		}

		for (const FRammsNewtonNativeBodyRecord& BodyRecord : BridgeRecord.Bodies)
		{
			if (!BodyRecord.PrimitiveComponent.IsValid())
			{
				continue;
			}

			if (const FRammsNewtonNativeTransform* Transform = TransformsByBodyId.Find(BodyRecord.NativeBodyId))
			{
				BodyRecord.PrimitiveComponent->SetWorldTransform(
					RammsNewtonNative::ToUnrealTransform(*Transform),
					false,
					nullptr,
					ETeleportType::TeleportPhysics);
			}
		}
	}

	return true;
}

TArray<uint64> FRammsNewtonNativeBackend::GatherPythonPullBodyIds() const
{
	TArray<uint64> BodyIds;
	for (const TPair<FObjectKey, FBridgeRecord>& Pair : BridgeRecords)
	{
		const FBridgeRecord& BridgeRecord = Pair.Value;
		if (!BridgeRecord.Bridge.IsValid() || !BridgeRecord.Bridge->BridgeDescription.bPullSolverPosesBackToUnreal)
		{
			continue;
		}

		for (const FRammsNewtonNativeBodyRecord& BodyRecord : BridgeRecord.Bodies)
		{
			if (BodyRecord.NativeBodyId != 0)
			{
				BodyIds.Add(BodyRecord.NativeBodyId);
			}
		}
	}

	return BodyIds;
}

bool FRammsNewtonNativeBackend::SendPythonRequest(const FString& Method, const TSharedPtr<FJsonObject>& Params, TSharedPtr<FJsonObject>& OutResult)
{
	if (!PythonWorkerProcess.IsValid() || !FPlatformProcess::IsProcRunning(PythonWorkerProcess) || !PythonStdInWritePipe || !PythonStdOutReadPipe)
	{
		SetLastError(TEXT("RammsNewtonPhysics: Python worker bridge is not running."));
		return false;
	}

	const uint64			RequestId = PythonRequestId++;
	TSharedRef<FJsonObject> Request = MakeShared<FJsonObject>();
	Request->SetNumberField(TEXT("id"), static_cast<double>(RequestId));
	Request->SetStringField(TEXT("method"), Method);
	Request->SetObjectField(TEXT("params"), Params.IsValid() ? Params.ToSharedRef() : MakeShared<FJsonObject>());

	FString															 Payload;
	TSharedRef<TJsonWriter<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>> Writer =
		TJsonWriterFactory<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>::Create(&Payload);
	FJsonSerializer::Serialize(Request, Writer);
	Payload.AppendChar(TEXT('\n'));

	{
		FScopeLock Lock(&PythonResponseMutex);
		PythonResponses.Remove(RequestId);
	}

	FPlatformProcess::WritePipe(PythonStdInWritePipe, Payload);
	TSharedPtr<FJsonObject> Response;
	if (!WaitForPythonResponse(RequestId, Response))
	{
		SetLastError(FString::Printf(TEXT("RammsNewtonPhysics: timed out waiting for Python worker response to '%s'."), *Method));
		return false;
	}

	bool bOk = false;
	Response->TryGetBoolField(TEXT("ok"), bOk);
	if (!bOk)
	{
		FString ErrorMessage;
		Response->TryGetStringField(TEXT("error"), ErrorMessage);
		SetLastError(FString::Printf(TEXT("RammsNewtonPhysics: Python worker request '%s' failed: %s"), *Method, *ErrorMessage));
		return false;
	}

	const TSharedPtr<FJsonObject>* ResultObject = nullptr;
	if (Response->TryGetObjectField(TEXT("result"), ResultObject) && ResultObject)
	{
		OutResult = *ResultObject;
	}
	else
	{
		OutResult = MakeShared<FJsonObject>();
	}

	return true;
}

FString FRammsNewtonNativeBackend::ResolvePythonExecutablePath(const URammsNewtonPhysicsSettings& Settings) const
{
	if (!Settings.PythonExecutablePath.IsEmpty())
	{
		return FPaths::IsRelative(Settings.PythonExecutablePath)
			? FPaths::ConvertRelativePathToFull(FPaths::ProjectDir(), Settings.PythonExecutablePath)
			: Settings.PythonExecutablePath;
	}

#if PLATFORM_WINDOWS
	return TEXT("python.exe");
#else
	return TEXT("python3");
#endif
}

FString FRammsNewtonNativeBackend::ResolvePythonWorkerScriptPath(const URammsNewtonPhysicsSettings& Settings) const
{
	if (!Settings.PythonWorkerScriptPath.IsEmpty())
	{
		return FPaths::IsRelative(Settings.PythonWorkerScriptPath)
			? FPaths::ConvertRelativePathToFull(FPaths::ProjectDir(), Settings.PythonWorkerScriptPath)
			: Settings.PythonWorkerScriptPath;
	}

	return FPaths::Combine(GetPluginBaseDir(), TEXT("Scripts"), TEXT("ramms_newton_worker.py"));
}

FString FRammsNewtonNativeBackend::GetPluginBaseDir() const
{
	const TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("RammsNewtonPhysics"));
	return Plugin.IsValid()
		? Plugin->GetBaseDir()
		: FPaths::Combine(FPaths::ProjectPluginsDir(), TEXT("RammsNewtonPhysics"));
}
