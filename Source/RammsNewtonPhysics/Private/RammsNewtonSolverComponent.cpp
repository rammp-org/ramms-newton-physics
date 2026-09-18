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

	// Beta resolves Auto -> Live/Direct/Puppet on the engine; the manager only
	// carries the authored preference, so asking it would miss the resolution.
	if (Engine->GetStepMode() != EStepMode::Live)
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

namespace
{
	/**
	 * Rewrite `file="some/dir/name.ext"` to `file="name.ext"`.
	 *
	 * The worker writes every asset flat into one scene directory, so a
	 * reference carrying a directory would not resolve there. This is the same
	 * transform URLab's own bridge applies before shipping a scene.
	 */
	void FlattenAssetPaths(FString& Xml)
	{
		FString				Flattened;
		Flattened.Reserve(Xml.Len());
		const FRegexPattern Pattern(TEXT("file=\"([^\"]*?)([^/\\\\\"]+)\""));
		FRegexMatcher		Matcher(Pattern, Xml);
		int32				Cursor = 0;
		while (Matcher.FindNext())
		{
			Flattened += Xml.Mid(Cursor, Matcher.GetMatchBeginning() - Cursor);
			Flattened += FString::Printf(TEXT("file=\"%s\""), *Matcher.GetCaptureGroup(2));
			Cursor = Matcher.GetMatchEnding();
		}
		Flattened += Xml.Mid(Cursor);
		Xml = MoveTemp(Flattened);
	}
} // namespace

bool URammsNewtonSolverComponent::SerializeCompiledModel(
	UMjPhysicsEngine* Engine, FString& OutXml, TMap<FString, TArray<uint8>>& OutAssets, FString& OutError)
{
	// Beta removed the engine's raw mjSpec accessor and its ActiveAssetPaths
	// list. BuildCompiledScene replaces both, and is the same source the bridge
	// handshake's `mjcf_compiled` comes from, so the worker now receives exactly
	// the scene a remote client would be given. It hands back the MJCF text the
	// compiler was actually handed rather than a re-serialisation of the model,
	// which matters here: re-serialising flattens the participant structure away.
	FMjCompiledScene Scene;
	if (!Engine->BuildCompiledScene(Scene, OutError))
	{
		if (OutError.IsEmpty())
		{
			OutError = TEXT("engine could not build a compiled scene");
		}
		return false;
	}

	OutXml = Scene.Xml;
	FlattenAssetPaths(OutXml);

	// A beta scene is not always one spec. Participants are referenced by the
	// VFS name they were mounted under, so each one has to travel with the
	// scene or the worker's compile fails on a missing <model file=...>.
	// Shipping them as assets puts them in the worker's flat scene dir under
	// exactly the name the scene references.
	for (const TPair<FString, FString>& Participant : Scene.ParticipantXml)
	{
		FString ParticipantXml = Participant.Value;
		FlattenAssetPaths(ParticipantXml);
		FTCHARToUTF8 Utf8(*ParticipantXml);
		TArray<uint8> Bytes;
		Bytes.Append(reinterpret_cast<const uint8*>(Utf8.Get()), Utf8.Length());
		OutAssets.Add(Participant.Key, MoveTemp(Bytes));
	}

	// AssetFiles maps the name the MJCF mounts an asset under to the file it
	// came from, so the key is what to ship it as — not the file's own name,
	// which need not match.
	for (const TPair<FString, FString>& Asset : Scene.AssetFiles)
	{
		TArray<uint8> Data;
		if (!FFileHelper::LoadFileToArray(Data, *Asset.Value))
		{
			OutError = FString::Printf(
				TEXT("failed to read asset '%s' (mounted as '%s')"), *Asset.Value, *Asset.Key);
			return false;
		}
		OutAssets.Add(Asset.Key, MoveTemp(Data));
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
		[this, Engine](mjModel* Model, mjData* Data) -> bool {
			// Beta's contract: return true iff this call advanced sim state,
			// and fire OnPostStep ourselves — the engine loop only does that
			// for the plain mj_step path, so a handler that stays silent
			// leaves recorders and replay never notified. Every path below
			// advances, either through Newton or the mj_step fallback.
			const auto Advanced = [Engine, Model, Data]() -> bool {
				if (Engine->OnPostStep)
				{
					Engine->OnPostStep(Model, Data);
				}
				return true;
			};

			if (Model != ExpectedModel.load() || bStepFailed.load() || !Client.IsValid()
				|| PendingResync.load() != static_cast<uint8>(EResyncRequest::None))
			{
				mj_step(Model, Data);
				return Advanced();
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
				return Advanced();
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
					return Advanced();
				}
				SetStatus(FString::Printf(TEXT("Step failed: %s"), *Error));
				bStepFailed.store(true);
				mj_step(Model, Data);
				return Advanced();
			}

			if (Result.Qpos.Num() != Model->nq || Result.Qvel.Num() != Model->nv)
			{
				SetStatus(FString::Printf(
					TEXT("Step reply layout mismatch (qpos %d vs nq %d)"), Result.Qpos.Num(), Model->nq));
				bStepFailed.store(true);
				mj_step(Model, Data);
				return Advanced();
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
			return Advanced();
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
