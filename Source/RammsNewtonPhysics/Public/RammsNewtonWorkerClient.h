// Copyright RAMMP. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "HAL/PlatformProcess.h"
#include "RammsNewtonPhysicsTypes.h"
#include "Templates/SharedPointer.h"

class FJsonObject;

/**
 * Client for the out-of-process Newton worker
 * (Scripts/newton_worker, ZMQ REQ/REP, JSON codec).
 *
 * The worker owns the Newton/warp stack in a subprocess ON PURPOSE: warp's
 * native kernel compiler can hard-crash the hosting process, so it must
 * never run inside the editor. Every entry point here therefore tolerates a
 * dead or wedged worker: requests time out, the REQ socket is rebuilt, and
 * the caller gets a clean failure instead of a hang.
 *
 * Thread-safety: all RPC calls serialize on an internal mutex and may be
 * made from any thread (Step is expected on URLab's physics thread).
 * Start/Stop must not be called concurrently with RPCs from another thread
 * in a way that overlaps a Stop — the owning component guarantees that.
 */
class RAMMSNEWTONPHYSICS_API FRammsNewtonWorkerClient
{
public:
	FRammsNewtonWorkerClient() = default;
	~FRammsNewtonWorkerClient();

	FRammsNewtonWorkerClient(const FRammsNewtonWorkerClient&) = delete;
	FRammsNewtonWorkerClient& operator=(const FRammsNewtonWorkerClient&) = delete;

	/** Settings override, else the pinned venv under <Plugin>/Scripts/.venv. Empty if none found. */
	static FString ResolvePythonExecutable();

	/** Directory containing the newton_worker package (<Plugin>/Scripts). */
	static FString GetScriptsDir();

	/**
	 * Run `python -m newton_worker --probe` and parse the capability line.
	 * Returns false only when the probe could not run at all (no python,
	 * timeout); an unavailable-Newton env still returns true with
	 * OutCapabilities.bAvailable == false and the reason in .Error.
	 */
	static bool RunProbe(FRammsNewtonCapabilities& OutCapabilities);

	/** Spawn the worker (`serve`) and wait for its READY line. */
	bool Start(FString& OutError);

	/** Shut the worker down (polite `shutdown` op, then terminate). Safe to call twice. */
	void Stop(bool bRequestShutdown = true);

	bool					IsWorkerProcessAlive() const;
	ERammsNewtonWorkerState GetState() const { return State; }

	bool Hello(FRammsNewtonCapabilities& OutCapabilities, FString& OutError);

	bool LoadModel(
		const FString&						MjcfXml,
		const TMap<FString, TArray<uint8>>& Assets,
		const FString&						SolverWireName,
		FRammsNewtonModelInfo&				OutInfo,
		FString&							OutError);

	/**
	 * Advance the worker sim. Ctrl must be empty or nu-sized; MocapPos /
	 * MocapQuat must be empty or nmocap*3 / nmocap*4 flat arrays.
	 */
	bool Step(
		TConstArrayView<double> Ctrl,
		TConstArrayView<double> MocapPos,
		TConstArrayView<double> MocapQuat,
		int32					Nsteps,
		FRammsNewtonStepResult& OutState,
		FString&				OutError);

	bool ResetSim(FRammsNewtonStepResult& OutState, FString& OutError);

private:
	bool Request(
		const TCHAR*				   Op,
		const TSharedPtr<FJsonObject>& Params,
		double						   TimeoutSeconds,
		TSharedPtr<FJsonObject>&	   OutResult,
		FString&					   OutError);

	bool CreateSocket(FString& OutError);
	void DestroySocket();
	void CloseProcess();

	static bool ParseStepResult(const TSharedPtr<FJsonObject>& Result, FRammsNewtonStepResult& Out);
	static void ParseCapabilities(const TSharedPtr<FJsonObject>& Json, FRammsNewtonCapabilities& Out);

	void*	ZmqContext = nullptr;
	void*	ZmqSocket = nullptr;
	FString Endpoint;

	FProcHandle WorkerProc;
	void*		StdOutReadPipe = nullptr;
	void*		StdOutWritePipe = nullptr;

	int32					 NextRequestId = 1;
	ERammsNewtonWorkerState	 State = ERammsNewtonWorkerState::Stopped;
	mutable FCriticalSection RequestMutex;
};
