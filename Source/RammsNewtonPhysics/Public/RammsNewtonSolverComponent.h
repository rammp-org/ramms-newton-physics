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

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(
		float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
	UMjPhysicsEngine* FindEngine() const;
	void			  BeginBind(UMjPhysicsEngine* Engine);
	void			  InstallHandler(UMjPhysicsEngine* Engine);
	void			  UninstallHandler();
	void			  SetStatus(const FString& InStatus);

	static bool SerializeCompiledModel(
		UMjPhysicsEngine*			  Engine,
		FString&					  OutXml,
		TMap<FString, TArray<uint8>>& OutAssets,
		FString&					  OutError);

	/** Worker connection; shared so background load/teardown tasks can outlive the component. */
	TSharedPtr<FRammsNewtonWorkerClient, ESPMode::ThreadSafe> Client;

	TWeakObjectPtr<UMjPhysicsEngine> BoundEngine;

	/** The mjModel the worker currently mirrors; the step handler ignores any other. */
	std::atomic<mjModel_*> ExpectedModel{ nullptr };

	/** Set by the step handler (physics thread) on any failure; serviced in Tick. */
	std::atomic<bool> bStepFailed{ false };

	/** Forward d->mocap_* each step; auto-disabled if the worker rejects it. */
	std::atomic<bool> bForwardMocap{ true };

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
};
