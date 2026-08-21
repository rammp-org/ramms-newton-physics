// Copyright RAMMP. All Rights Reserved.

#pragma once

#include "Components/ActorComponent.h"
#include "CoreMinimal.h"
#include "RammsNewtonPhysicsTypes.h"
#include "Templates/SharedPointer.h"

#include <atomic>

#include "RammsNewtonSolverComponent.generated.h"

class FRammsNewtonWorkerClient;
class UMjPhysicsEngine;
struct mjModel_;
struct mjData_;

/**
 * Swaps URLab's mj_step for Newton stepping (plan §5.3, "Newton steps,
 * MuJoCo renders/senses").
 *
 * Place on any actor in a level with an AAMjManager. When active it:
 *  1. serializes the manager's compiled model (mj_saveXMLString + VFS
 *     assets) and loads it into the out-of-process Newton worker;
 *  2. installs a CustomStepHandler that, per physics iteration, forwards
 *     d->ctrl (and mocap) to the worker, steps Newton, writes qpos/qvel/act
 *     back into mjData, and runs mj_forward — so every URLab consumer
 *     (sensors, publishers, render pump) works unchanged.
 *
 * Degradation is always graceful: any failure (worker missing/dead/timeout,
 * layout mismatch, model recompile) falls back to local mj_step for that
 * step, and the component fully deactivates on the game thread with a
 * warning. Newton being unavailable on this machine just means the
 * component stays inactive — URLab runs stock.
 *
 * Mutually exclusive with URLab replay and Direct/Puppet step modes (all
 * share the single CustomStepHandler slot).
 */
UCLASS(ClassGroup = (MuJoCo), meta = (BlueprintSpawnableComponent))
class RAMMSNEWTONPHYSICS_API URammsNewtonSolverComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	URammsNewtonSolverComponent();

	/** Bind to the manager and take over stepping as soon as a compiled model exists. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	bool bActivateOnBeginPlay = true;

	/**
	 * Mid-run attach seeds the worker with the engine's CURRENT state via
	 * set_state, so no reset is needed. This flag is now only the FALLBACK:
	 * if seeding fails and it is true, the simulation is reset so both sides
	 * align at t=0; if false, activation is refused instead.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton")
	bool bResetSimOnActivate = true;

	/** Steps per second achieved by the Newton bridge (5 s sliding window). */
	UFUNCTION(BlueprintPure, Category = "Newton")
	float GetAchievedHz() const { return AchievedHz.load(); }

	/** Begin (or retry) taking over stepping. Loads happen asynchronously. */
	UFUNCTION(BlueprintCallable, Category = "Newton")
	void ActivateNewtonSolver();

	/** Return stepping to URLab's local mj_step. */
	UFUNCTION(BlueprintCallable, Category = "Newton")
	void DeactivateNewtonSolver();

	/** True while the CustomStepHandler is installed and healthy. */
	UFUNCTION(BlueprintPure, Category = "Newton")
	bool IsNewtonStepping() const;

	UFUNCTION(BlueprintPure, Category = "Newton")
	FString GetStatusText() const;

	UFUNCTION(BlueprintPure, Category = "Newton")
	FRammsNewtonModelInfo GetModelInfo() const { return ModelInfo; }

	/**
	 * Serialize the engine's compiled model (mj_saveXMLString on the live
	 * mjSpec, asset references flattened to bare filenames) plus the VFS
	 * asset blobs — the exact payload the worker's load_model expects and the
	 * scene-export artifact for headless training (plan §5.4). Takes the
	 * engine's CallbackMutex briefly. Also used by the editor module
	 * (validate / export actions).
	 */
	static bool SerializeCompiledModel(
		UMjPhysicsEngine*			  Engine,
		FString&					  OutXml,
		TMap<FString, TArray<uint8>>& OutAssets,
		FString&					  OutError);

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(
		float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
	UMjPhysicsEngine* FindEngine() const;
	void			  BeginBind(UMjPhysicsEngine* Engine);
	void			  InstallHandler(UMjPhysicsEngine* Engine);
	void			  FinishInstall(UMjPhysicsEngine* Engine);
	void			  UninstallHandler();
	void			  SetStatus(const FString& InStatus);

	/**
	 * Push the engine's current qpos/qvel/act/time into the worker
	 * (thread-pool task; captures state under CallbackMutex, then RPCs).
	 * bResetWorkerFirst mirrors a sim reset with a worker rebuild before the
	 * injection. OnDone runs on the game thread with the outcome.
	 */
	void BeginStateSync(bool bResetWorkerFirst, TFunction<void(bool, FString)> OnDone);

	/**
	 * Validate a worker state (layout + finiteness) and write it into mjData,
	 * advancing time one step and running mj_forward. Physics thread only.
	 * On failure sets status + bStepFailed and returns false (caller should
	 * mj_step locally).
	 */
	bool ValidateAndWriteback(mjModel_* Model, mjData_* Data, const FRammsNewtonStepResult& Result);

	/** Worker connection; shared so background load/teardown tasks can outlive the component. */
	TSharedPtr<FRammsNewtonWorkerClient, ESPMode::ThreadSafe> Client;

	TWeakObjectPtr<UMjPhysicsEngine> BoundEngine;

	/** The mjModel the worker currently mirrors; the step handler ignores any other. */
	std::atomic<mjModel_*> ExpectedModel{ nullptr };

	/** Set by the step handler (physics thread) on any failure; serviced in Tick. */
	std::atomic<bool> bStepFailed{ false };

	/** Forward d->mocap_* each step; auto-disabled if the worker rejects it. */
	std::atomic<bool> bForwardMocap{ true };

	/**
	 * Lifecycle resync requests raised by the step handler when it observes a
	 * time discontinuity in mjData (see TickComponent for servicing).
	 */
	enum class EResyncRequest : uint8
	{
		None = 0,
		/** d->time snapped to ~0: URLab reset — worker reset + state injection. */
		Reset = 1,
		/** d->time jumped mid-run (snapshot restore) — worker set_state injection. */
		SetState = 2,
	};
	std::atomic<uint8> PendingResync{ 0 };
	bool			   bResyncInFlight = false;

	/**
	 * d->time as of our last writeback (physics thread). -1 = no step yet.
	 * The handler compares mjData's time against this to detect resets and
	 * restores that happened between our steps.
	 */
	std::atomic<double> LastSteppedTime{ -1.0 };

	bool bWantActive = false;
	bool bHandlerInstalled = false;
	bool bLoadInFlight = false;

	FRammsNewtonModelInfo ModelInfo;

	mutable FCriticalSection StatusMutex;
	FString					 StatusText;

	/** Physics-thread scratch (only the step handler touches these). */
	TArray<double> CtrlScratch;
	TArray<double> MocapPosScratch;
	TArray<double> MocapQuatScratch;

	/** One-step pipelining enabled for the installed handler (settings copy). */
	bool bPipelineActive = true;

	/** Whether the in-flight pipelined request carried mocap data (physics thread only). */
	bool bPendingStepHadMocap = false;

	/** Achieved step rate (5 s window, computed in Tick from StepCounter). */
	std::atomic<uint64> StepCounter{ 0 };
	std::atomic<float>	AchievedHz{ 0.0f };
	double				HzWindowStart = 0.0;
	uint64				HzWindowStartCount = 0;
};
