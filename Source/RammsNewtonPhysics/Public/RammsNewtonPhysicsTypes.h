// Copyright RAMMP. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RammsNewtonPhysicsTypes.generated.h"

RAMMSNEWTONPHYSICS_API DECLARE_LOG_CATEGORY_EXTERN(LogRammsNewton, Log, All);

/** Which Newton solver flavor the worker should construct. */
UENUM(BlueprintType)
enum class ERammsNewtonSolver : uint8
{
	/** MuJoCo-Warp on the GPU (requires CUDA). Wire name: "mujoco". */
	MuJoCoWarp UMETA(DisplayName = "MuJoCo-Warp (GPU)"),
	/** Plain MuJoCo under the SolverMuJoCo wrapper (portable). Wire name: "mujoco_cpu". */
	MuJoCoCpu UMETA(DisplayName = "MuJoCo (CPU)"),
};

/** Lifecycle of the out-of-process Newton worker. */
UENUM(BlueprintType)
enum class ERammsNewtonWorkerState : uint8
{
	Stopped,
	Starting,
	Ready,
	Failed,
};

/**
 * Result of the availability probe (`python -m newton_worker --probe`).
 * Availability is a runtime property of the machine (Python env, Newton
 * import, CUDA), never a link-time one — the plugin loads everywhere.
 */
USTRUCT(BlueprintType)
struct RAMMSNEWTONPHYSICS_API FRammsNewtonCapabilities
{
	GENERATED_BODY()

	/** True once a probe has actually run (successfully or not). */
	UPROPERTY(BlueprintReadOnly, Category = "Newton")
	bool bProbed = false;

	/** True when the worker env imports Newton and offers at least one solver. */
	UPROPERTY(BlueprintReadOnly, Category = "Newton")
	bool bAvailable = false;

	UPROPERTY(BlueprintReadOnly, Category = "Newton")
	FString NewtonVersion;

	UPROPERTY(BlueprintReadOnly, Category = "Newton")
	FString PythonVersion;

	UPROPERTY(BlueprintReadOnly, Category = "Newton")
	bool bCudaAvailable = false;

	UPROPERTY(BlueprintReadOnly, Category = "Newton")
	FString CudaDeviceName;

	/** Wire names of usable solvers ("mujoco", "mujoco_cpu"). */
	UPROPERTY(BlueprintReadOnly, Category = "Newton")
	TArray<FString> Solvers;

	/** Human-readable reason when bAvailable is false. */
	UPROPERTY(BlueprintReadOnly, Category = "Newton")
	FString Error;

	bool SupportsSolver(const FString& WireName) const { return Solvers.Contains(WireName); }
};

/** Model description returned by the worker's load_model (original-MJCF terms). */
USTRUCT(BlueprintType)
struct RAMMSNEWTONPHYSICS_API FRammsNewtonModelInfo
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "Newton")
	FString Solver;

	UPROPERTY(BlueprintReadOnly, Category = "Newton")
	double Timestep = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Newton")
	int32 Nq = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Newton")
	int32 Nv = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Newton")
	int32 Nu = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Newton")
	int32 Na = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Newton")
	int32 Nmocap = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Newton")
	TArray<FString> JointNames;

	UPROPERTY(BlueprintReadOnly, Category = "Newton")
	TArray<FString> ActuatorNames;

	UPROPERTY(BlueprintReadOnly, Category = "Newton")
	TArray<FString> Warnings;
};

/** One step's worth of state read back from the worker (original-MJCF layout). */
struct FRammsNewtonStepResult
{
	double		   Time = 0.0;
	int64		   StepCount = 0;
	TArray<double> Qpos;
	TArray<double> Qvel;
	TArray<double> Act;
};

namespace RammsNewton
{
	/** UI enum -> wire name expected by the worker. */
	inline const TCHAR* SolverWireName(ERammsNewtonSolver Solver)
	{
		return Solver == ERammsNewtonSolver::MuJoCoWarp ? TEXT("mujoco") : TEXT("mujoco_cpu");
	}
} // namespace RammsNewton
