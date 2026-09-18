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
#include "Misc/ScopeExit.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"

static TAutoConsoleVariable<int32> CVarNewtonDumpShippedScene(
	TEXT("Ramms.Newton.DumpShippedScene"),
	0,
	TEXT("Write the MJCF and assets handed to the Newton worker into ")
	TEXT("Saved/NewtonExport/ before loading. Off by default; turn on to diff ")
	TEXT("against URLab's own Saved/URLab/scene_compiled.xml."),
	ECVF_Default);
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

	// AssetFiles maps the name the MJCF mounts an asset under to the file it
	// came from, so the key is what to ship it as — not the file's own name,
	// which need not match. Read first: the flatten pass below needs them
	// mounted, or a scene referencing meshes will not compile.
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

	// Newton's MJCF importer does not implement MuJoCo's <attach> / <model
	// file=...> composition: handed such a scene it silently yields zero
	// bodies and zero joints, and the only symptom downstream is "the model
	// must have at least one joint". Plain MuJoCo reads the same file
	// correctly, so this is an importer gap rather than a bad scene.
	//
	// URLab beta emits exactly that shape — each articulation is a participant
	// attached into the scene — so the scene has to be flattened into a single
	// document before it goes on the wire. Parsing it with the participants and
	// assets mounted in a VFS, compiling, and re-serialising resolves every
	// attach; it is the same round trip URLab performs for its own debug
	// artifact, and the compile is not optional because MuJoCo refuses to write
	// XML for a spec it has not compiled ("Only compiled model can be written").
	{
		mjVFS Vfs;
		mj_defaultVFS(&Vfs);
		ON_SCOPE_EXIT
		{
			mj_deleteVFS(&Vfs);
		};

		TArray<FTCHARToUTF8> Held;
		Held.Reserve(Scene.ParticipantXml.Num());
		for (const TPair<FString, FString>& Participant : Scene.ParticipantXml)
		{
			FString ParticipantXml = Participant.Value;
			FlattenAssetPaths(ParticipantXml);
			const int32 Index = Held.Emplace(*ParticipantXml);
			mj_addBufferVFS(&Vfs, TCHAR_TO_UTF8(*Participant.Key), Held[Index].Get(), Held[Index].Length());
		}
		for (const TPair<FString, TArray<uint8>>& Asset : OutAssets)
		{
			mj_addBufferVFS(&Vfs, TCHAR_TO_UTF8(*Asset.Key), Asset.Value.GetData(), Asset.Value.Num());
		}

		FString SceneXml = Scene.Xml;
		FlattenAssetPaths(SceneXml);

		char	SpecError[1024] = {0};
		mjSpec* Spec = mj_parseXMLString(TCHAR_TO_UTF8(*SceneXml), &Vfs, SpecError, sizeof(SpecError));
		if (!Spec)
		{
			OutError = FString::Printf(TEXT("could not parse the compiled scene: %s"), UTF8_TO_TCHAR(SpecError));
			return false;
		}
		ON_SCOPE_EXIT
		{
			mj_deleteSpec(Spec);
		};

		// Compiled purely to satisfy the writer; the model itself is discarded.
		if (mjModel* Compiled = mj_compile(Spec, &Vfs))
		{
			mj_deleteModel(Compiled);
		}
		else
		{
			OutError = FString::Printf(
				TEXT("the compiled scene did not recompile for flattening: %s"), UTF8_TO_TCHAR(mjs_getError(Spec)));
			return false;
		}

		// Sized before the call, then once against the size MuJoCo asks for: a
		// positive return is the length it wants, not a failure.
		TArray<char> Buffer;
		Buffer.SetNumZeroed(1 << 16);
		char  SaveError[1024] = {0};
		int32 Result = mj_saveXMLString(Spec, Buffer.GetData(), Buffer.Num(), SaveError, sizeof(SaveError));
		if (Result > 0)
		{
			Buffer.SetNumZeroed(Result + 1);
			Result = mj_saveXMLString(Spec, Buffer.GetData(), Buffer.Num(), SaveError, sizeof(SaveError));
		}
		if (Result != 0)
		{
			OutError = FString::Printf(TEXT("could not flatten the compiled scene: %s"), UTF8_TO_TCHAR(SaveError));
			return false;
		}
		OutXml = UTF8_TO_TCHAR(Buffer.GetData());
	}
	if (OutXml.IsEmpty())
	{
		OutError = TEXT("flattening produced no XML");
		return false;
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


	// What we actually put on the wire, written next to the engine's own
	// scene_compiled.xml so the two can be diffed. A model that loads in the
	// worker standalone but fails through the bridge is a shipping bug, and
	// without this the only evidence is the worker's error text.
	if (CVarNewtonDumpShippedScene.GetValueOnAnyThread() != 0)
	{
		const FString DumpDir = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("NewtonExport"));
		FFileHelper::SaveStringToFile(Xml, *FPaths::Combine(DumpDir, TEXT("shipped_scene.xml")),
			FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
		for (const TPair<FString, TArray<uint8>>& Asset : Assets)
		{
			FFileHelper::SaveArrayToFile(Asset.Value, *FPaths::Combine(DumpDir, Asset.Key));
		}
		UE_LOG(LogRammsNewton, Log,
			TEXT("[NewtonSolver] Dumped shipped scene (%d chars, %d asset(s)) to %s"),
			Xml.Len(), Assets.Num(), *DumpDir);
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
