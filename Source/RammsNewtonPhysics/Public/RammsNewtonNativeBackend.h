// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Async/Future.h"
#include "CoreMinimal.h"
#include "HAL/PlatformProcess.h"
#include "RammsNewtonPhysicsTypes.h"
#include "UObject/ObjectKey.h"

class FJsonObject;
class FEvent;
class UPrimitiveComponent;
class URammsNewtonPhysicsComponent;
class URammsNewtonPhysicsSettings;

struct FRammsNewtonNativeVec3
{
	float X = 0.0f;
	float Y = 0.0f;
	float Z = 0.0f;
};

struct FRammsNewtonNativeQuat
{
	float X = 0.0f;
	float Y = 0.0f;
	float Z = 0.0f;
	float W = 1.0f;
};

struct FRammsNewtonNativeTransform
{
	FRammsNewtonNativeVec3 TranslationCm;
	FRammsNewtonNativeQuat Rotation;
	FRammsNewtonNativeVec3 Scale3D{ 1.0f, 1.0f, 1.0f };
};

struct FRammsNewtonNativeWorldCreateDesc
{
	float				   FixedStepHz = 240.0f;
	int32				   MaxSubstepsPerTick = 1;
	FRammsNewtonNativeVec3 GravityCmPerSecondSquared;
};

enum class ERammsNewtonNativeShapeType : uint8
{
	Unknown,
	Box,
	Sphere,
	Capsule,
	Mesh,
	ConvexHull,
};

struct FRammsNewtonNativeBodyShapeDesc
{
	ERammsNewtonNativeShapeType ShapeType = ERammsNewtonNativeShapeType::Unknown;
	FRammsNewtonNativeVec3		HalfExtentsCm;
	float						RadiusCm = 0.0f;
	float						HalfHeightCm = 0.0f;
	const float*				MeshVerticesCm = nullptr;
	int32						MeshVertexCount = 0;
	const int32*				MeshIndices = nullptr;
	int32						MeshIndexCount = 0;
};

struct FRammsNewtonNativeBodyMaterialDesc
{
	float Density = 1000.0f;
	float ContactElasticStiffness = 10000.0f;
	float ContactDamping = 1000.0f;
	float FrictionDamping = 0.0f;
	float AdhesionDistance = 0.0f;
	float Friction = 0.5f;
	float Restitution = 0.0f;
	float TorsionalFriction = 0.005f;
	float RollingFriction = 0.0001f;
	float CollisionMarginCm = 0.0f;
	bool  bSolid = true;
	int32 CollisionGroup = 1;
};

struct FRammsNewtonNativeBodyCreateDesc
{
	const char*						   NameAnsi = nullptr;
	const char*						   OwnerNameAnsi = nullptr;
	float							   MassKg = 1.0f;
	bool							   bKinematic = false;
	FRammsNewtonNativeTransform		   Transform;
	FRammsNewtonNativeBodyShapeDesc	   Shape;
	FRammsNewtonNativeBodyMaterialDesc Material;
};

struct FRammsNewtonNativeBodyRecord
{
	uint64								NativeBodyId = 0;
	FName								ComponentName;
	TWeakObjectPtr<UPrimitiveComponent> PrimitiveComponent;

	bool IsValid() const
	{
		return NativeBodyId != 0 && PrimitiveComponent.IsValid();
	}
};

class RAMMSNEWTONPHYSICS_API FRammsNewtonNativeBackend
{
public:
	FRammsNewtonNativeBackend();
	~FRammsNewtonNativeBackend();

	bool Initialize(const URammsNewtonPhysicsSettings& Settings, const FRammsNewtonBackendStatus& BackendStatus);
	void Shutdown();

	bool								 IsInitialized() const;
	const FRammsNewtonNativeWorldStatus& GetWorldStatus() const;

	bool RegisterBridge(URammsNewtonPhysicsComponent& Bridge);
	void UnregisterBridge(URammsNewtonPhysicsComponent& Bridge);

	bool StepSimulation(float FixedStepSeconds);
	void SyncAllBridgesToNative();
	void SyncAllBridgesFromNative();

private:
	struct FNativeExports;

	struct FBridgeRecord
	{
		TWeakObjectPtr<URammsNewtonPhysicsComponent> Bridge;
		TArray<FRammsNewtonNativeBodyRecord>		 Bodies;
	};

	bool				 BindExports();
	bool				 InitializePythonBridge(const URammsNewtonPhysicsSettings& Settings);
	void				 ShutdownPythonBridge();
	void				 ResetWorldStatus();
	void				 UpdateRegistrationCounts();
	void				 SetLastError(const FString& ErrorText);
	FBridgeRecord*		 FindBridgeRecord(URammsNewtonPhysicsComponent& Bridge);
	const FBridgeRecord* FindBridgeRecord(const URammsNewtonPhysicsComponent& Bridge) const;
	bool				 CreateWorld(const FRammsNewtonNativeWorldCreateDesc& WorldDesc);
	void				 DestroyWorld();
	bool				 StepWorld(float FixedStepSeconds);
	uint64				 CreateBody(const FRammsNewtonNativeBodyCreateDesc& BodyDesc);
	void				 DestroyBody(uint64 BodyId);
	bool				 SetBodyTransform(uint64 BodyId, const FRammsNewtonNativeTransform& Transform);
	bool				 GetBodyTransform(uint64 BodyId, FRammsNewtonNativeTransform& OutTransform);
	bool				 CreateNativeBody(URammsNewtonPhysicsComponent& Bridge, UPrimitiveComponent& PrimitiveComponent, FRammsNewtonNativeBodyRecord& OutRecord);
	bool				 PushBodyTransformToNative(const FRammsNewtonNativeBodyRecord& BodyRecord);
	bool				 PullBodyTransformFromNative(const FRammsNewtonNativeBodyRecord& BodyRecord);
	void				 StartPythonOutputReader();
	void				 StopPythonOutputReader();
	void				 RunPythonOutputReader();
	void				 HandlePythonOutputLine(const FString& OutputLine);
	void				 QueuePythonResponse(const TSharedPtr<FJsonObject>& Response);
	bool				 WaitForPythonResponse(uint64 RequestId, TSharedPtr<FJsonObject>& OutResponse);
	bool				 ApplyPythonStepTransforms(const TSharedPtr<FJsonObject>& Result);
	TArray<uint64>		 GatherPythonPullBodyIds() const;
	bool				 SendPythonRequest(const FString& Method, const TSharedPtr<FJsonObject>& Params, TSharedPtr<FJsonObject>& OutResult);
	FString				 ResolvePythonExecutablePath(const URammsNewtonPhysicsSettings& Settings) const;
	FString				 ResolvePythonWorkerScriptPath(const URammsNewtonPhysicsSettings& Settings) const;
	FString				 GetPluginBaseDir() const;

	void*								  NativeWorldHandle = nullptr;
	FNativeExports*						  Exports = nullptr;
	FRammsNewtonNativeWorldStatus		  WorldStatus;
	TMap<FObjectKey, FBridgeRecord>		  BridgeRecords;
	FProcHandle							  PythonWorkerProcess;
	void*								  PythonStdOutReadPipe = nullptr;
	void*								  PythonStdOutWritePipe = nullptr;
	void*								  PythonStdInReadPipe = nullptr;
	void*								  PythonStdInWritePipe = nullptr;
	uint64								  PythonWorkerWorldId = 0;
	uint64								  PythonRequestId = 1;
	double								  PythonRequestTimeoutSeconds = 5.0;
	bool								  bUsePythonBridge = false;
	TAtomic<bool>						  bStopPythonOutputReader = false;
	FString								  PythonExecutablePath;
	FString								  PythonWorkerScriptPath;
	FString								  PendingPythonOutput;
	FCriticalSection					  PythonResponseMutex;
	TMap<uint64, TSharedPtr<FJsonObject>> PythonResponses;
	FEvent*								  PythonResponseEvent = nullptr;
	TFuture<void>						  PythonOutputReaderTask;
};
