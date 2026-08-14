// Copyright RAMMP. All Rights Reserved.

#include "RammsNewtonSolverComponent.h"

#include "Async/Async.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "Internationalization/Regex.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/ScopeLock.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "RammsNewtonPhysicsSettings.h"
#include "RammsNewtonWorkerClient.h"

#include <mujoco/mujoco.h>

URammsNewtonSolverComponent::URammsNewtonSolverComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickInterval = 0.0f;
}

void URammsNewtonSolverComponent::BeginPlay()
{
	Super::BeginPlay();
	if (bActivateOnBeginPlay)
	{
		ActivateNewtonSolver();
	}
}

void URammsNewtonSolverComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	DeactivateNewtonSolver();
	Super::EndPlay(EndPlayReason);
}

void URammsNewtonSolverComponent::ActivateNewtonSolver()
{
	bWantActive = true;
	bStepFailed.store(false);
	SetStatus(TEXT("Waiting for a compiled MuJoCo model"));
}

void URammsNewtonSolverComponent::DeactivateNewtonSolver()
{
	bWantActive = false;
	UninstallHandler();

	if (Client.IsValid())
	{
		// Stop() can block behind an in-flight load_model (its request mutex),
		// so retire the client on a worker thread; background tasks hold their
		// own shared refs, making this safe even mid-load.
		TSharedPtr<FRammsNewtonWorkerClient, ESPMode::ThreadSafe> Retired = MoveTemp(Client);
		Client.Reset();
		Async(EAsyncExecution::ThreadPool, [Retired]() { Retired->Stop(); });
	}
	bLoadInFlight = false;
	bResyncInFlight = false;
	PendingResync.store(static_cast<uint8>(EResyncRequest::None));
	LastSteppedTime.store(-1.0);
	SetStatus(TEXT("Inactive (URLab stepping locally)"));
}

bool URammsNewtonSolverComponent::IsNewtonStepping() const
{
	return bHandlerInstalled && !bStepFailed.load();
}

FString URammsNewtonSolverComponent::GetStatusText() const
{
	FScopeLock Lock(&StatusMutex);
	return StatusText;
}

void URammsNewtonSolverComponent::SetStatus(const FString& InStatus)
{
	{
		FScopeLock Lock(&StatusMutex);
		if (StatusText == InStatus)
		{
			return;
		}
		StatusText = InStatus;
	}
	UE_LOG(LogRammsNewton, Log, TEXT("[NewtonSolver] %s"), *InStatus);
}

UMjPhysicsEngine* URammsNewtonSolverComponent::FindEngine() const
{
	AAMjManager* Manager = AAMjManager::GetManager();
	return Manager ? Manager->PhysicsEngine : nullptr;
}

void URammsNewtonSolverComponent::TickComponent(
	float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!bWantActive)
	{
		return;
	}

	// Handler-reported failure or a dead worker → return stepping to URLab.
	if (bHandlerInstalled && (bStepFailed.load() || !Client.IsValid() || !Client->IsWorkerProcessAlive()))
	{
		UE_LOG(LogRammsNewton, Warning,
			TEXT("[NewtonSolver] Newton stepping failed — falling back to local mj_step. %s"),
			*GetStatusText());
		const bool bWasWanted = bWantActive;
		DeactivateNewtonSolver();
		// Stay deactivated; the user can retry via ActivateNewtonSolver().
		bWantActive = false;
		if (bWasWanted && GEngine)
		{
			GEngine->AddOnScreenDebugMessage(
				reinterpret_cast<uint64>(this), 10.0f, FColor::Orange,
				TEXT("Newton solver failed — URLab resumed local MuJoCo stepping"));
		}
		return;
	}

	UMjPhysicsEngine* Engine = FindEngine();
	if (!Engine || !Engine->GetModel())
	{
		return; // manager not up / model not compiled yet — keep waiting
	}

	AAMjManager* Manager = AAMjManager::GetManager();
	if (Manager && Manager->EffectiveStepMode.load() != EStepMode::Live)
	{
		SetStatus(TEXT("Manager is in Direct/Puppet step mode — Newton requires Live mode"));
		return;
	}

	// Service lifecycle resyncs raised by the step handler. While a resync is
	// pending or in flight, the handler steps locally (mj_step) so the sim
	// keeps running; the worker rejoins once its state is aligned again.
	const EResyncRequest Resync = static_cast<EResyncRequest>(PendingResync.load());
	if (bHandlerInstalled && Resync != EResyncRequest::None && !bResyncInFlight)
	{
		if (Resync == EResyncRequest::Unsupported)
		{
			UE_LOG(LogRammsNewton, Warning,
				TEXT("[NewtonSolver] Snapshot restore detected — not supported under Newton yet (needs worker set_state). Falling back to local stepping."));
			DeactivateNewtonSolver();
			bWantActive = false;
			SetStatus(TEXT("Deactivated: snapshot restore requires worker set_state (URLab stepping locally)"));
			return;
		}

		// Simulation reset: mirror it in the worker (v1 reset = full model
		// rebuild — slow-ish cold, seconds warm), then let the handler resume.
		// The few locally-stepped frames in between cause a bounded, tiny
		// divergence that the first writeback reconciles.
		bResyncInFlight = true;
		SetStatus(TEXT("Sim reset — resetting Newton worker"));
		TSharedPtr<FRammsNewtonWorkerClient, ESPMode::ThreadSafe> LocalClient = Client;
		TWeakObjectPtr<URammsNewtonSolverComponent>				  WeakThis(this);
		Async(EAsyncExecution::ThreadPool,
			[LocalClient, WeakThis]() {
				FRammsNewtonStepResult Unused;
				FString				   Error;
				const bool			   bOk = LocalClient->ResetSim(Unused, Error);
				AsyncTask(ENamedThreads::GameThread,
					[WeakThis, bOk, Error]() {
						URammsNewtonSolverComponent* This = WeakThis.Get();
						if (!This)
						{
							return;
						}
						This->bResyncInFlight = false;
						if (!This->bWantActive || !This->bHandlerInstalled)
						{
							return;
						}
						if (!bOk)
						{
							This->SetStatus(FString::Printf(TEXT("Worker reset failed: %s"), *Error));
							This->bStepFailed.store(true); // next Tick falls back cleanly
							return;
						}
						// Re-arm the handler at the engine's current time so the
						// discontinuity check doesn't re-trigger on the local
						// steps taken while the worker was rebuilding.
						if (UMjPhysicsEngine* BoundEnginePtr = This->BoundEngine.Get())
						{
							FScopeLock Lock(&BoundEnginePtr->CallbackMutex);
							if (mjData* Data = BoundEnginePtr->GetData())
							{
								This->LastSteppedTime.store(Data->time);
							}
						}
						This->PendingResync.store(static_cast<uint8>(EResyncRequest::None));
						This->SetStatus(TEXT("Newton stepping active (resynced after sim reset)"));
					});
			});
		return;
	}

	// (Re)bind whenever URLab's compiled model is not the one the worker mirrors.
	if (Engine->GetModel() != ExpectedModel.load() && !bLoadInFlight)
	{
		if (bHandlerInstalled)
		{
			UE_LOG(LogRammsNewton, Log, TEXT("[NewtonSolver] Model recompiled — rebinding worker"));
			UninstallHandler();
		}
		BeginBind(Engine);
	}
}

bool URammsNewtonSolverComponent::SerializeCompiledModel(
	UMjPhysicsEngine* Engine, FString& OutXml, TMap<FString, TArray<uint8>>& OutAssets, FString& OutError)
{
	{
		// Brief lock: keep the physics loop from stepping while we read the spec.
		FScopeLock Lock(&Engine->CallbackMutex);
		mjSpec*	   Spec = Engine->GetSpec();
		if (!Spec)
		{
			OutError = TEXT("engine has no mjSpec");
			return false;
		}

		// Two-pass growing buffer, mirroring URLab's handshake serializer.
		TArray<uint8> XmlBuf;
		for (int32 Capacity = 256 * 1024; Capacity <= 32 * 1024 * 1024; Capacity *= 2)
		{
			XmlBuf.SetNumUninitialized(Capacity);
			FMemory::Memzero(XmlBuf.GetData(), Capacity);
			char	  SaveError[1024] = "";
			const int XmlResult = mj_saveXMLString(
				Spec, reinterpret_cast<char*>(XmlBuf.GetData()), Capacity, SaveError, sizeof(SaveError));
			if (XmlResult == 0)
			{
				OutXml = UTF8_TO_TCHAR(reinterpret_cast<const char*>(XmlBuf.GetData()));
				break;
			}
			const FString Error = UTF8_TO_TCHAR(SaveError);
			if (!Error.Contains(TEXT("buffer"), ESearchCase::IgnoreCase))
			{
				OutError = FString::Printf(TEXT("mj_saveXMLString failed: %s"), *Error);
				return false;
			}
		}
	}
	if (OutXml.IsEmpty())
	{
		OutError = TEXT("mj_saveXMLString produced no XML (model too large?)");
		return false;
	}

	// Flatten asset references to bare filenames so the worker's scene dir
	// (assets written flat) resolves them — same transform URLab's bridge does.
	{
		FString Flattened;
		Flattened.Reserve(OutXml.Len());
		const FRegexPattern Pattern(TEXT("file=\"([^\"]*?)([^/\\\\\"]+)\""));
		FRegexMatcher		Matcher(Pattern, OutXml);
		int32				Cursor = 0;
		while (Matcher.FindNext())
		{
			Flattened += OutXml.Mid(Cursor, Matcher.GetMatchBeginning() - Cursor);
			Flattened += FString::Printf(TEXT("file=\"%s\""), *Matcher.GetCaptureGroup(2));
			Cursor = Matcher.GetMatchEnding();
		}
		Flattened += OutXml.Mid(Cursor);
		OutXml = MoveTemp(Flattened);
	}

	for (const FString& AssetPath : Engine->ActiveAssetPaths)
	{
		TArray<uint8> Data;
		if (!FFileHelper::LoadFileToArray(Data, *AssetPath))
		{
			OutError = FString::Printf(TEXT("failed to read asset '%s'"), *AssetPath);
			return false;
		}
		OutAssets.Add(FPaths::GetCleanFilename(AssetPath), MoveTemp(Data));
	}
	return true;
}

void URammsNewtonSolverComponent::BeginBind(UMjPhysicsEngine* Engine)
{
	mjModel* Model = Engine->GetModel();

	FString						 Xml;
	TMap<FString, TArray<uint8>> Assets;
	FString						 Error;
	if (!SerializeCompiledModel(Engine, Xml, Assets, Error))
	{
		SetStatus(FString::Printf(TEXT("Model serialization failed: %s"), *Error));
		bWantActive = false;
		return;
	}

	if (!Client.IsValid())
	{
		Client = MakeShared<FRammsNewtonWorkerClient, ESPMode::ThreadSafe>();
	}

	bLoadInFlight = true;
	SetStatus(TEXT("Loading model into Newton worker (first load may compile GPU kernels)"));

	// Snapshot everything the background task needs; it must not touch the
	// component (except via the weak ptr on the game thread at the end).
	TSharedPtr<FRammsNewtonWorkerClient, ESPMode::ThreadSafe> LocalClient = Client;
	TWeakObjectPtr<URammsNewtonSolverComponent>				  WeakThis(this);
	TWeakObjectPtr<UMjPhysicsEngine>						  WeakEngine(Engine);
	const int32												  ModelNq = Model->nq;
	const int32												  ModelNv = Model->nv;
	const int32												  ModelNu = Model->nu;
	const double											  ModelTimestep = Model->opt.timestep;

	Async(EAsyncExecution::ThreadPool,
		[LocalClient, WeakThis, WeakEngine, Model, ModelNq, ModelNv, ModelNu, ModelTimestep,
			Xml = MoveTemp(Xml), Assets = MoveTemp(Assets)]() {
			const URammsNewtonPhysicsSettings& Settings = URammsNewtonPhysicsSettings::Get();
			FString							   BindError;
			FRammsNewtonModelInfo			   Info;
			bool							   bBound = false;

			do
			{
				if (!LocalClient->Start(BindError))
				{
					break;
				}

				FRammsNewtonCapabilities Caps;
				if (!LocalClient->Hello(Caps, BindError))
				{
					break;
				}

				FString Solver = RammsNewton::SolverWireName(Settings.Solver);
				if (!Caps.SupportsSolver(Solver))
				{
					if (Settings.bFallBackToCpuSolver && Caps.SupportsSolver(TEXT("mujoco_cpu")))
					{
						UE_LOG(LogRammsNewton, Warning,
							TEXT("[NewtonSolver] Solver '%s' unavailable (%s) — using mujoco_cpu"),
							*Solver, *Caps.Error);
						Solver = TEXT("mujoco_cpu");
					}
					else
					{
						BindError = FString::Printf(
							TEXT("solver '%s' unavailable: %s"), *Solver, *Caps.Error);
						break;
					}
				}

				if (!LocalClient->LoadModel(Xml, Assets, Solver, Info, BindError))
				{
					break;
				}

				if (Info.Nq != ModelNq || Info.Nv != ModelNv || Info.Nu != ModelNu)
				{
					BindError = FString::Printf(
						TEXT("layout mismatch: worker nq/nv/nu %d/%d/%d vs engine %d/%d/%d"),
						Info.Nq, Info.Nv, Info.Nu, ModelNq, ModelNv, ModelNu);
					break;
				}
				if (!FMath::IsNearlyEqual(Info.Timestep, ModelTimestep, 1e-9))
				{
					UE_LOG(LogRammsNewton, Warning,
						TEXT("[NewtonSolver] Worker timestep %g differs from engine %g"),
						Info.Timestep, ModelTimestep);
				}

				if (Settings.bVerifyLivenessAfterLoad)
				{
					// A miscompiled kernel cache yields a loaded-but-frozen or
					// NaN sim; a few free steps expose the worst of it.
					FRammsNewtonStepResult Probe;
					FString				   LivenessError;
					if (!LocalClient->Step({}, {}, {}, 3, Probe, LivenessError))
					{
						BindError = FString::Printf(TEXT("liveness step failed: %s"), *LivenessError);
						break;
					}
					bool bFinite = true;
					for (double Value : Probe.Qpos)
					{
						bFinite &= FMath::IsFinite(Value);
					}
					for (double Value : Probe.Qvel)
					{
						bFinite &= FMath::IsFinite(Value);
					}
					if (!bFinite)
					{
						BindError = TEXT("liveness check produced non-finite state (miscompiled kernels?)");
						break;
					}
					FRammsNewtonStepResult Unused;
					if (!LocalClient->ResetSim(Unused, LivenessError))
					{
						BindError = FString::Printf(TEXT("post-liveness reset failed: %s"), *LivenessError);
						break;
					}
				}

				bBound = true;
			}
			while (false);

			AsyncTask(ENamedThreads::GameThread,
				[WeakThis, WeakEngine, Model, Info, BindError, bBound]() {
					URammsNewtonSolverComponent* This = WeakThis.Get();
					if (!This)
					{
						return;
					}
					This->bLoadInFlight = false;
					if (!This->bWantActive)
					{
						return; // deactivated while loading
					}
					UMjPhysicsEngine* Engine = WeakEngine.Get();
					if (!bBound)
					{
						This->SetStatus(FString::Printf(TEXT("Newton bind failed: %s"), *BindError));
						This->bWantActive = false;
						return;
					}
					if (!Engine || Engine->GetModel() != Model)
					{
						// Recompiled while we were loading; Tick will rebind.
						This->SetStatus(TEXT("Model changed during load — rebinding"));
						return;
					}
					This->ModelInfo = Info;
					This->InstallHandler(Engine);
				});
		});
}

void URammsNewtonSolverComponent::InstallHandler(UMjPhysicsEngine* Engine)
{
	// The worker starts from the model's initial state, so the engine must be
	// there too or the first writeback would visibly snap the sim backwards.
	double EngineTime = 0.0;
	{
		FScopeLock Lock(&Engine->CallbackMutex);
		if (mjData* Data = Engine->GetData())
		{
			EngineTime = Data->time;
		}
	}
	if (EngineTime > UE_KINDA_SMALL_NUMBER)
	{
		if (bResetSimOnActivate)
		{
			UE_LOG(LogRammsNewton, Log,
				TEXT("[NewtonSolver] Sim advanced to t=%.3fs during worker load — resetting so Newton takes over from the initial state"),
				EngineTime);
			Engine->ResetSimulation();
		}
		else
		{
			SetStatus(FString::Printf(
				TEXT("Refusing to attach at t=%.3fs (bResetSimOnActivate is off; Newton can only start from the initial state until set_state lands)"),
				EngineTime));
			bWantActive = false;
			return;
		}
	}

	BoundEngine = Engine;
	ExpectedModel.store(Engine->GetModel());
	bStepFailed.store(false);
	bForwardMocap.store(true);
	PendingResync.store(static_cast<uint8>(EResyncRequest::None));
	LastSteppedTime.store(-1.0);

	// Runs on URLab's physics thread, inside CallbackMutex. It must never
	// call back into Set/ClearCustomStepHandler (self-deadlock) — failures
	// raise bStepFailed and Tick services them on the game thread.
	Engine->SetCustomStepHandler(
		[this](mjModel* Model, mjData* Data) {
			if (Model != ExpectedModel.load() || bStepFailed.load() || !Client.IsValid()
				|| PendingResync.load() != static_cast<uint8>(EResyncRequest::None))
			{
				mj_step(Model, Data);
				return;
			}

			// Detect external time changes since our last writeback: a reset
			// (time back to ~0) or a snapshot restore (arbitrary jump). URLab
			// applies both inside this same physics iteration before the step
			// handler runs, so mjData is already in the new state here.
			const double Prev = LastSteppedTime.load();
			if (Prev >= 0.0 && !FMath::IsNearlyEqual(Data->time, Prev, Model->opt.timestep * 0.5))
			{
				const bool bIsReset = Data->time < Model->opt.timestep * 0.5;
				PendingResync.store(static_cast<uint8>(
					bIsReset ? EResyncRequest::Reset : EResyncRequest::Unsupported));
				mj_step(Model, Data);
				return;
			}

			CtrlScratch.SetNum(Model->nu, EAllowShrinking::No);
			if (Model->nu > 0)
			{
				FMemory::Memcpy(CtrlScratch.GetData(), Data->ctrl, Model->nu * sizeof(double));
			}

			MocapPosScratch.Reset();
			MocapQuatScratch.Reset();
			if (bForwardMocap.load() && Model->nmocap > 0 && ModelInfo.Nmocap == Model->nmocap)
			{
				MocapPosScratch.Append(Data->mocap_pos, Model->nmocap * 3);
				MocapQuatScratch.Append(Data->mocap_quat, Model->nmocap * 4);
			}

			FRammsNewtonStepResult Result;
			FString				   Error;
			if (!Client->Step(CtrlScratch, MocapPosScratch, MocapQuatScratch, 1, Result, Error))
			{
				// The solver's internal re-export can change nmocap; drop mocap
				// forwarding rather than the whole backend when that is the cause.
				if (MocapPosScratch.Num() > 0 && Error.Contains(TEXT("mocap")))
				{
					UE_LOG(LogRammsNewton, Warning,
						TEXT("[NewtonSolver] Worker rejected mocap data (%s) — mocap forwarding disabled"),
						*Error);
					bForwardMocap.store(false);
					mj_step(Model, Data);
					return;
				}
				SetStatus(FString::Printf(TEXT("Step failed: %s"), *Error));
				bStepFailed.store(true);
				mj_step(Model, Data);
				return;
			}

			if (Result.Qpos.Num() != Model->nq || Result.Qvel.Num() != Model->nv)
			{
				SetStatus(FString::Printf(
					TEXT("Step reply layout mismatch (qpos %d vs nq %d)"), Result.Qpos.Num(), Model->nq));
				bStepFailed.store(true);
				mj_step(Model, Data);
				return;
			}

			// Never write a diverged solver state into URLab: NaN qpos would
			// propagate through mj_forward into every sensor/publisher and
			// leave nothing for the local fallback to resume from.
			bool bFinite = true;
			for (double Value : Result.Qpos)
			{
				bFinite &= FMath::IsFinite(Value);
			}
			for (double Value : Result.Qvel)
			{
				bFinite &= FMath::IsFinite(Value);
			}
			if (!bFinite)
			{
				SetStatus(TEXT("Worker returned non-finite state (solver diverged)"));
				bStepFailed.store(true);
				mj_step(Model, Data);
				return;
			}

			FMemory::Memcpy(Data->qpos, Result.Qpos.GetData(), Model->nq * sizeof(double));
			FMemory::Memcpy(Data->qvel, Result.Qvel.GetData(), Model->nv * sizeof(double));
			if (Model->na > 0 && Result.Act.Num() == Model->na)
			{
				FMemory::Memcpy(Data->act, Result.Act.GetData(), Model->na * sizeof(double));
			}
			Data->time += Model->opt.timestep;
			LastSteppedTime.store(Data->time);

			// Recompute all derived quantities (sites, sensors, contacts) at
			// Newton's state so URLab's sensors/publishers stay consistent.
			mj_forward(Model, Data);
		});

	bHandlerInstalled = true;
	SetStatus(FString::Printf(
		TEXT("Newton stepping active (%s, nq=%d nu=%d)"), *ModelInfo.Solver, ModelInfo.Nq, ModelInfo.Nu));
}

void URammsNewtonSolverComponent::UninstallHandler()
{
	if (UMjPhysicsEngine* Engine = BoundEngine.Get())
	{
		// Takes CallbackMutex internally, so returning guarantees no step
		// handler invocation (which captures `this`) is still running.
		Engine->ClearCustomStepHandler();
	}
	bHandlerInstalled = false;
	BoundEngine = nullptr;
	ExpectedModel.store(nullptr);
}
