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
	bPendingStepHadMocap = false;
	AchievedHz.store(0.0f);
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

	// Achieved worker step rate, 5 s sliding window (bridge health metric —
	// pipelined stepping should sit near the physics rate, not ~0.1x of it).
	if (bHandlerInstalled)
	{
		const double Now = FPlatformTime::Seconds();
		const double Window = Now - HzWindowStart;
		if (Window >= 5.0)
		{
			const uint64 Count = StepCounter.load();
			const float	 Hz = static_cast<float>(static_cast<double>(Count - HzWindowStartCount) / Window);
			AchievedHz.store(Hz);
			HzWindowStart = Now;
			HzWindowStartCount = Count;
			UE_LOG(LogRammsNewton, Verbose, TEXT("[NewtonSolver] Achieved %.1f worker steps/s"), Hz);
		}
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
	// Reset mirrors the reset in the worker (v1 = full model rebuild) and
	// then injects the engine's CURRENT state via set_state, so the local
	// steps taken during the rebuild cause no divergence. A snapshot restore
	// skips the rebuild and just injects.
	const EResyncRequest Resync = static_cast<EResyncRequest>(PendingResync.load());
	if (bHandlerInstalled && Resync != EResyncRequest::None && !bResyncInFlight)
	{
		const bool bIsReset = Resync == EResyncRequest::Reset;
		bResyncInFlight = true;
		SetStatus(bIsReset
				? TEXT("Sim reset — resetting Newton worker + injecting state")
				: TEXT("Snapshot restore — injecting state into Newton worker"));
		TWeakObjectPtr<URammsNewtonSolverComponent> WeakThis(this);
		BeginStateSync(bIsReset,
			[WeakThis, bIsReset](bool bOk, FString Error) {
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
					This->SetStatus(FString::Printf(
						TEXT("Worker %s failed: %s"),
						bIsReset ? TEXT("reset resync") : TEXT("snapshot-restore set_state"), *Error));
					This->bStepFailed.store(true); // next Tick falls back cleanly
					return;
				}
				// Re-arm the handler at the engine's current time so the
				// discontinuity check doesn't re-trigger on the local steps
				// taken while the worker was syncing.
				if (UMjPhysicsEngine* BoundEnginePtr = This->BoundEngine.Get())
				{
					FScopeLock Lock(&BoundEnginePtr->CallbackMutex);
					if (mjData* Data = BoundEnginePtr->GetData())
					{
						This->LastSteppedTime.store(Data->time);
					}
				}
				This->PendingResync.store(static_cast<uint8>(EResyncRequest::None));
				This->SetStatus(bIsReset
						? TEXT("Newton stepping active (reset + state resync)")
						: TEXT("Newton stepping active (snapshot restored via set_state)"));
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
		FString Flattened;
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
		// Ship it under the name the flattened XML will ask for. FlattenAssetPaths
		// rewrites every file= reference down to its basename, so a mounted name
		// carrying a directory would be written to disk at a path the scene no
		// longer mentions and the worker's load would fail on a missing file.
		const FString ShippedName = FPaths::GetCleanFilename(Asset.Key);
		if (const TArray<uint8>* Existing = OutAssets.Find(ShippedName))
		{
			// Two mounts colliding on one basename is unrecoverable rather than
			// something to resolve by picking a winner: flattening has already
			// made the scene's two references indistinguishable. Identical bytes
			// are the one harmless case.
			if (*Existing != Data)
			{
				OutError = FString::Printf(
					TEXT("two different assets flatten to '%s' (one of them is '%s'); ")
						TEXT("rename one in the scene so the flattened references stay distinct"),
					*ShippedName, *Asset.Value);
				return false;
			}
			continue;
		}
		OutAssets.Add(ShippedName, MoveTemp(Data));
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

		char	SpecError[1024] = { 0 };
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
		char  SaveError[1024] = { 0 };
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
		bool		  bWrote = FFileHelper::SaveStringToFile(Xml,
			*FPaths::Combine(DumpDir, TEXT("shipped_scene.xml")),
			FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
		for (const TPair<FString, TArray<uint8>>& Asset : Assets)
		{
			bWrote &= FFileHelper::SaveArrayToFile(Asset.Value, *FPaths::Combine(DumpDir, Asset.Key));
		}
		// A diagnostic that reports success it did not have is worse than no
		// diagnostic: the next person reads the line and trusts the files.
		UE_CLOG(bWrote, LogRammsNewton, Log,
			TEXT("[NewtonSolver] Dumped shipped scene (%d chars, %d asset(s)) to %s"),
			Xml.Len(), Assets.Num(), *DumpDir);
		UE_CLOG(!bWrote, LogRammsNewton, Warning,
			TEXT("[NewtonSolver] Failed to write part of the shipped-scene dump to %s"), *DumpDir);
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

void URammsNewtonSolverComponent::BeginStateSync(
	bool bResetWorkerFirst, TFunction<void(bool, FString)> OnDone)
{
	TSharedPtr<FRammsNewtonWorkerClient, ESPMode::ThreadSafe> LocalClient = Client;
	TWeakObjectPtr<UMjPhysicsEngine>						  WeakEngine = BoundEngine;
	if (!LocalClient.IsValid())
	{
		OnDone(false, TEXT("no worker client"));
		return;
	}
	Async(EAsyncExecution::ThreadPool,
		[LocalClient, WeakEngine, OnDone, bResetWorkerFirst]() {
			FRammsNewtonStepResult Unused;
			FString				   Error;
			bool				   bOk = true;

			if (bResetWorkerFirst)
			{
				bOk = LocalClient->ResetSim(Unused, Error);
			}

			if (bOk)
			{
				// Capture AND inject under CallbackMutex. Releasing it between
				// the two lets the fallback handler keep calling mj_step while
				// set_state is in flight, so the worker is seeded from a state
				// the engine has already moved past — and the first Newton
				// writeback then rolls the pose back by every step taken during
				// the RPC. Holding the lock stalls the physics thread for the
				// duration instead, which is a hitch on a rare event rather
				// than a silent rewind on every one.
				TArray<double> Qpos, Qvel, Act;
				double		   Time = -1.0;
				if (UMjPhysicsEngine* Engine = WeakEngine.Get())
				{
					FScopeLock Lock(&Engine->CallbackMutex);
					mjModel*   Model = Engine->GetModel();
					mjData*	   Data = Engine->GetData();
					if (Model && Data)
					{
						Qpos.Append(Data->qpos, Model->nq);
						Qvel.Append(Data->qvel, Model->nv);
						if (Model->na > 0)
						{
							Act.Append(Data->act, Model->na);
						}
						Time = Data->time;
						bOk = LocalClient->SetState(Qpos, Qvel, Act, Time, Unused, Error);
					}
				}
				if (Time < 0.0)
				{
					bOk = false;
					Error = TEXT("engine/model unavailable while capturing state");
				}
			}

			AsyncTask(ENamedThreads::GameThread,
				[OnDone, bOk, Error]() { OnDone(bOk, Error); });
		});
}

void URammsNewtonSolverComponent::InstallHandler(UMjPhysicsEngine* Engine)
{
	// The worker sits at the model's initial state after load. If the engine
	// already advanced (long GPU-kernel compile, mid-run activation), seed
	// the worker with the engine's CURRENT state via set_state instead of
	// resetting the whole simulation.
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
		UE_LOG(LogRammsNewton, Log,
			TEXT("[NewtonSolver] Sim advanced to t=%.3fs during worker load — seeding worker via set_state"),
			EngineTime);
		SetStatus(FString::Printf(TEXT("Seeding worker state at t=%.3fs"), EngineTime));
		BoundEngine = Engine; // BeginStateSync captures from BoundEngine
		TWeakObjectPtr<URammsNewtonSolverComponent> WeakThis(this);
		TWeakObjectPtr<UMjPhysicsEngine>			WeakEngine(Engine);
		mjModel*									Model = Engine->GetModel();
		BeginStateSync(/*bResetWorkerFirst=*/false,
			[WeakThis, WeakEngine, Model](bool bOk, FString Error) {
				URammsNewtonSolverComponent* This = WeakThis.Get();
				if (!This || !This->bWantActive)
				{
					return;
				}
				UMjPhysicsEngine* Engine = WeakEngine.Get();
				if (!Engine || Engine->GetModel() != Model)
				{
					This->SetStatus(TEXT("Model changed during state seeding — rebinding"));
					return; // Tick's rebind picks it up
				}
				if (bOk)
				{
					This->FinishInstall(Engine);
					return;
				}
				if (This->bResetSimOnActivate)
				{
					UE_LOG(LogRammsNewton, Warning,
						TEXT("[NewtonSolver] set_state seeding failed (%s) — falling back to a sim reset"),
						*Error);
					// Reset BOTH sides. A set_state that timed out may still have
					// been applied, and one that failed part-way leaves the worker
					// somewhere unknown; resetting only the engine would let the
					// first reply overwrite the freshly reset state with that
					// unknown one. A worker reset that itself fails is worth
					// refusing the attach over, since nothing after it is trusted.
					FString				   ResetError;
					FRammsNewtonStepResult ResetState;
					if (This->Client.IsValid() && !This->Client->ResetSim(ResetState, ResetError))
					{
						This->SetStatus(FString::Printf(
							TEXT("Refusing to attach: worker reset failed after a failed seed (%s)"),
							*ResetError));
						This->bWantActive = false;
						return;
					}
					Engine->ResetSimulation();
					This->FinishInstall(Engine);
					return;
				}
				This->SetStatus(FString::Printf(
					TEXT("Refusing to attach: set_state seeding failed (%s) and bResetSimOnActivate is off"),
					*Error));
				This->bWantActive = false;
			});
		return;
	}

	FinishInstall(Engine);
}

void URammsNewtonSolverComponent::FinishInstall(UMjPhysicsEngine* Engine)
{
	BoundEngine = Engine;
	ExpectedModel.store(Engine->GetModel());
	bStepFailed.store(false);
	bForwardMocap.store(true);
	PendingResync.store(static_cast<uint8>(EResyncRequest::None));
	LastSteppedTime.store(-1.0);
	bPipelineActive = URammsNewtonPhysicsSettings::Get().bPipelineSteps;
	bPendingStepHadMocap = false;
	StepCounter.store(0);
	{
		// Belongs to the installation that wrote it. Carried across an
		// uninstall into a new session with the same nq, it would be compared
		// against the previous run's state and request a needless sync on the
		// very first step. Empty means "the first Advanced() establishes it".
		FScopeLock Lock(&LastWrittenMutex);
		LastWrittenQpos.Reset();
	}
	HzWindowStart = FPlatformTime::Seconds();
	HzWindowStartCount = 0;

	// Runs on URLab's physics thread, inside CallbackMutex. It must never
	// call back into Set/ClearCustomStepHandler (self-deadlock) — failures
	// raise bStepFailed and Tick services them on the game thread.
	//
	// Pipelined mode (§6.8 option 1): each invocation first collects the
	// PREVIOUS step's reply (usually already arrived — the worker computed it
	// during UE's frame) and writes it back, then sends the next request with
	// the CURRENT ctrl/mocap. The written-back state is one model timestep
	// stale; time still advances exactly one step per invocation, so URLab's
	// accumulator bookkeeping is unchanged. The very first invocation after
	// install has nothing to collect and steps locally once to prime the
	// pipeline (the next writeback overwrites the full state anyway).
	Engine->SetCustomStepHandler(
		[this, Engine](mjModel* Model, mjData* Data) -> bool {
			// Beta's contract: return true iff this call advanced sim state,
			// and fire OnPostStep ourselves — the engine loop only does that
			// for the plain mj_step path, so a handler that stays silent
			// leaves recorders and replay never notified. Every path below
			// advances, either through Newton or the mj_step fallback.
			const auto Advanced = [this, Engine, Model, Data]() -> bool {
				// Re-baseline on every exit, not just the writeback: the
				// mj_step fallbacks move qpos too, and leaving the baseline
				// stale makes the external-edit check below fire on our own
				// output — which loops resync -> fallback -> resync and stops
				// the sim advancing at all.
				{
					FScopeLock Lock(&LastWrittenMutex);
					LastWrittenQpos.SetNumUninitialized(Model->nq, EAllowShrinking::No);
					FMemory::Memcpy(LastWrittenQpos.GetData(), Data->qpos, Model->nq * sizeof(double));
				}
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
			// handler runs, so mjData is already in the new state here. Any
			// pipelined request still in flight is drained (and discarded) by
			// the resync RPC itself.
			const double Prev = LastSteppedTime.load();
			if (Prev >= 0.0 && !FMath::IsNearlyEqual(Data->time, Prev, Model->opt.timestep * 0.5))
			{
				const bool bIsReset = Data->time < Model->opt.timestep * 0.5;
				PendingResync.store(static_cast<uint8>(
					bIsReset ? EResyncRequest::Reset : EResyncRequest::SetState));
				mj_step(Model, Data);
				return Advanced();
			}

			// A keyframe reset, a Blueprint pose write or anything else that
			// edits qpos without touching time slips past the check above: it
			// leaves d->time exactly where our last writeback put it. Left
			// undetected the next writeback simply overwrites the edit, so the
			// pose silently snaps back and the caller is told nothing.
			// Comparing against what we last wrote is the cheap way to notice,
			// and it cannot false-positive on our own output.
			{
				FScopeLock Lock(&LastWrittenMutex);
				if (LastWrittenQpos.Num() == Model->nq)
				{
					for (int32 i = 0; i < Model->nq; ++i)
					{
						if (!FMath::IsNearlyEqual(Data->qpos[i], LastWrittenQpos[i], 1e-9))
						{
							PendingResync.store(static_cast<uint8>(EResyncRequest::SetState));
							break;
						}
					}
				}
			}
			if (PendingResync.load() != static_cast<uint8>(EResyncRequest::None))
			{
				mj_step(Model, Data);
				return Advanced();
			}

			// ---- collect the previous pipelined step, if any ----
			bool bWroteBack = false;
			if (bPipelineActive && Client->HasPendingStep())
			{
				FRammsNewtonStepResult Result;
				FString				   Error;
				if (!Client->StepCollect(Result, Error))
				{
					// The solver's internal re-export can change nmocap; drop
					// mocap forwarding rather than the whole backend.
					if (bPendingStepHadMocap && Error.Contains(TEXT("mocap")))
					{
						UE_LOG(LogRammsNewton, Warning,
							TEXT("[NewtonSolver] Worker rejected mocap data (%s) — mocap forwarding disabled"),
							*Error);
						bForwardMocap.store(false);
						// The worker rejected the request before it stepped, so
						// it is now a step behind the engine we are about to
						// advance locally. Resume only after pushing state, or
						// the next reply pairs a stale pose with a newer time.
						PendingResync.store(static_cast<uint8>(EResyncRequest::SetState));
						mj_step(Model, Data);
						LastSteppedTime.store(Data->time);
						return Advanced();
					}
					SetStatus(FString::Printf(TEXT("Step failed: %s"), *Error));
					bStepFailed.store(true);
					mj_step(Model, Data);
					return Advanced();
				}
				if (!ValidateAndWriteback(Model, Data, Result))
				{
					mj_step(Model, Data);
					return Advanced();
				}
				bWroteBack = true;
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

			if (bPipelineActive)
			{
				FString Error;
				if (!Client->StepBegin(CtrlScratch, MocapPosScratch, MocapQuatScratch, 1, Error))
				{
					SetStatus(FString::Printf(TEXT("Step send failed: %s"), *Error));
					bStepFailed.store(true);
					if (!bWroteBack)
					{
						mj_step(Model, Data);
					}
					return Advanced();
				}
				bPendingStepHadMocap = MocapPosScratch.Num() > 0;
				if (!bWroteBack)
				{
					// Prime the pipeline: keep the sim advancing this call; the
					// next writeback fully overwrites qpos/qvel/act anyway.
					mj_step(Model, Data);
				}
				LastSteppedTime.store(Data->time);
				StepCounter.fetch_add(1);
				return Advanced();
			}

			// ---- synchronous exchange (bPipelineSteps off) ----
			FRammsNewtonStepResult Result;
			FString				   Error;
			if (!Client->Step(CtrlScratch, MocapPosScratch, MocapQuatScratch, 1, Result, Error))
			{
				if (MocapPosScratch.Num() > 0 && Error.Contains(TEXT("mocap")))
				{
					UE_LOG(LogRammsNewton, Warning,
						TEXT("[NewtonSolver] Worker rejected mocap data (%s) — mocap forwarding disabled"),
						*Error);
					bForwardMocap.store(false);
					// As above: rejected before the worker stepped, so it is a
					// step behind. Push state before resuming.
					PendingResync.store(static_cast<uint8>(EResyncRequest::SetState));
					mj_step(Model, Data);
					LastSteppedTime.store(Data->time);
					return Advanced();
				}
				SetStatus(FString::Printf(TEXT("Step failed: %s"), *Error));
				bStepFailed.store(true);
				mj_step(Model, Data);
				return Advanced();
			}
			if (!ValidateAndWriteback(Model, Data, Result))
			{
				mj_step(Model, Data);
				return Advanced();
			}
			LastSteppedTime.store(Data->time);
			StepCounter.fetch_add(1);
			return Advanced();
		});

	bHandlerInstalled = true;
	SetStatus(FString::Printf(
		TEXT("Newton stepping active (%s, nq=%d nu=%d, %s)"), *ModelInfo.Solver, ModelInfo.Nq,
		ModelInfo.Nu, bPipelineActive ? TEXT("pipelined") : TEXT("synchronous")));
}

bool URammsNewtonSolverComponent::ValidateAndWriteback(
	mjModel_* Model, mjData_* Data, const FRammsNewtonStepResult& Result)
{
	// act is part of the state, not an optional extra: accepting a reply whose
	// act is missing or the wrong length and then leaving Data->act alone pairs
	// the worker's qpos/qvel with this side's stale activation.
	if (Result.Qpos.Num() != Model->nq || Result.Qvel.Num() != Model->nv
		|| Result.Act.Num() != Model->na)
	{
		SetStatus(FString::Printf(
			TEXT("Step reply layout mismatch (qpos %d vs nq %d, qvel %d vs nv %d, act %d vs na %d)"),
			Result.Qpos.Num(), Model->nq, Result.Qvel.Num(), Model->nv, Result.Act.Num(), Model->na));
		bStepFailed.store(true);
		return false;
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
	// act is copied into mjData alongside qpos/qvel, so a NaN activation poisons
	// the dynamics just as surely; checking only the other two would let it past
	// a function whose whole job is to refuse a diverged state.
	for (double Value : Result.Act)
	{
		bFinite &= FMath::IsFinite(Value);
	}
	if (!bFinite)
	{
		SetStatus(TEXT("Worker returned non-finite state (solver diverged)"));
		bStepFailed.store(true);
		return false;
	}

	FMemory::Memcpy(Data->qpos, Result.Qpos.GetData(), Model->nq * sizeof(double));
	FMemory::Memcpy(Data->qvel, Result.Qvel.GetData(), Model->nv * sizeof(double));
	if (Model->na > 0)
	{
		FMemory::Memcpy(Data->act, Result.Act.GetData(), Model->na * sizeof(double));
	}
	Data->time += Model->opt.timestep;
	LastSteppedTime.store(Data->time);

	// Recompute all derived quantities (sites, sensors, contacts) at
	// Newton's state so URLab's sensors/publishers stay consistent.
	mj_forward(Model, Data);
	return true;
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
