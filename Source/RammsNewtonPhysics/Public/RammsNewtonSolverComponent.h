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
	 * Serialize the engine's compiled scene plus its VFS blobs — the exact
	 * payload the worker's load_model expects, and the scene-export artifact
	 * for headless training (plan §5.4). Also used by the editor module
	 * (validate / export actions).
	 *
	 * Sourced from UMjPhysicsEngine::BuildCompiledScene, which is the same
	 * place the bridge handshake's `mjcf_compiled` comes from, so the worker
	 * sees exactly what a remote client would. That hands back the MJCF the
	 * compiler was given rather than a re-serialisation of the model, which
	 * matters because a scene may reference participant specs by VFS name:
	 * re-serialising would flatten that structure away. Participants travel
	 * as assets under the names the scene references them by, alongside the
	 * asset files, with every file= reference flattened to a bare filename
	 * for the worker's flat scene directory.
	 */
	static bool SerializeCompiledModel(
		UMjPhysicsEngine*			  Engine,
		FString&					  OutXml,
		TMap<FString, TArray<uint8>>& OutAssets,
		FString&					  OutError);

protected:
	/** The qpos of our last writeback, and its lock. Compared at the top of the
	 *  next step to notice an external edit that left d->time untouched — a
	 *  keyframe reset, say, which would otherwise be silently overwritten. */
	TArray<double>			 LastWrittenQpos;
	mutable FCriticalSection LastWrittenMutex;

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
	/**
	 * @param OnInjectedUnderLock  Runs on the worker thread with the engine's
	 *        CallbackMutex still held, right after a successful injection, so
	 *        the caller can retire its request before physics can step again.
	 *        Must touch nothing that needs the game thread.
	 */
	/** Outcome of a BeginStateSync. StaleModel is not a failure: the compiled
	 *  model moved under the sync, so the right answer is to rebind, not to
	 *  drop the backend. */
	enum class EStateSyncResult : uint8
	{
		Ok,
		Failed,
		StaleModel,
	};

	void BeginStateSync(bool bResetWorkerFirst, const mjModel_* ExpectedModelForSync,
		TFunction<void(EStateSyncResult, FString)> OnDone,
		TFunction<void(double)>					   OnInjectedUnderLock = nullptr);

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
	/**
	 * Packed request: low 8 bits an EResyncRequest, high 24 a counter bumped by
	 * every raise. The kind alone cannot tell "still the request I am
	 * servicing" from "a fresh one of the same kind arrived while I was gone",
	 * and the completion clears by compare-exchange -- so without the counter a
	 * second reset raised mid-RPC would be cleared unserviced, and the next
	 * writeback would push the pre-reset pose back into the engine.
	 *
	 * The counter wraps at 2^24; a collision needs exactly 16.7M intervening
	 * raises during one RPC, and raises are edge-triggered (one per observed
	 * discontinuity, not one per step).
	 */
	/**
	 * Held by shared ref, not as plain members, because a resync retires from
	 * the worker thread while it still holds the engine's CallbackMutex (see
	 * BeginStateSync). Reaching back through the UObject there would mean
	 * resolving a weak pointer off the game thread; a ref-counted struct the
	 * in-flight lambda owns a reference to is valid whatever happens to the
	 * component.
	 */
	struct FResyncState
	{
		/** Packed kind + occurrence; see PendingResync notes above. */
		std::atomic<uint32> Pending{ 0 };
		/** d->time as of our last writeback. -1 = no step yet. */
		std::atomic<double> LastSteppedTime{ -1.0 };
	};
	/**
	 * Replaced -- not cleared -- by every install. A retire from a previous
	 * installation still holds a reference to the old object, and clearing in
	 * place would restart the occurrence counter at zero, letting that stale
	 * retire's ticket collide with the new session's first request and clear
	 * it unserviced. Safe to swap because the handler is installed at the end
	 * of FinishInstall and the previous one was removed by UninstallHandler
	 * under CallbackMutex, so nothing is reading it.
	 */
	TSharedRef<FResyncState, ESPMode::ThreadSafe> ResyncState = MakeShared<FResyncState, ESPMode::ThreadSafe>();

	bool bResyncInFlight = false;

	static constexpr uint32 ResyncCountShift = 8;

	static EResyncRequest ResyncKind(uint32 Packed)
	{
		return static_cast<EResyncRequest>(Packed & 0xFFu);
	}

	/** Raise a resync from the step handler, never weakening a pending Reset. */
	void RaiseResync(EResyncRequest Kind);

	/** Bumped by every install. A resync completion carries the generation it
	 *  started in and does nothing if that no longer matches, so a reply
	 *  arriving after a deactivate/reactivate cycle cannot clear a request
	 *  belonging to the new session or re-arm it against the old model. */
	std::atomic<uint32> InstallGeneration{ 0 };

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
