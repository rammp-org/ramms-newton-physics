// Copyright RAMMP. All Rights Reserved.

#include "RammsNewtonWorkerClient.h"

#include "Dom/JsonObject.h"
#include "HAL/PlatformTime.h"
#include "Interfaces/IPluginManager.h"
#include "Math/UnrealMathUtility.h"
#include "Misc/Base64.h"
#include "Misc/Paths.h"
#include "Misc/ScopeLock.h"
#include "RammsNewtonPhysicsSettings.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"

#if PLATFORM_WINDOWS
	#include "Windows/AllowWindowsPlatformTypes.h"
#endif
#include <zmq.h>
#if PLATFORM_WINDOWS
	#include "Windows/HideWindowsPlatformTypes.h"
#endif

namespace
{
	constexpr double WorkerReadySeconds = 30.0;
	constexpr double ShutdownOpSeconds = 2.0;
	constexpr double ShutdownGraceSeconds = 2.0;

	FString ReadAllAvailable(void* Pipe, FString& Accumulator)
	{
		if (Pipe)
		{
			Accumulator += FPlatformProcess::ReadPipe(Pipe);
		}
		return Accumulator;
	}

	bool GetDoubleArrayField(const TSharedPtr<FJsonObject>& Object, const TCHAR* Field, TArray<double>& Out)
	{
		const TArray<TSharedPtr<FJsonValue>>* Values = nullptr;
		if (!Object.IsValid() || !Object->TryGetArrayField(Field, Values))
		{
			return false;
		}
		Out.Reset(Values->Num());
		for (const TSharedPtr<FJsonValue>& Value : *Values)
		{
			Out.Add(Value->AsNumber());
		}
		return true;
	}

	void GetStringArrayField(const TSharedPtr<FJsonObject>& Object, const TCHAR* Field, TArray<FString>& Out)
	{
		Out.Reset();
		if (Object.IsValid())
		{
			Object->TryGetStringArrayField(Field, Out);
		}
	}
} // namespace

FRammsNewtonWorkerClient::~FRammsNewtonWorkerClient()
{
	Stop(/*bRequestShutdown=*/true);
}

FString FRammsNewtonWorkerClient::GetScriptsDir()
{
	const TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("RammsNewtonPhysics"));
	if (!Plugin.IsValid())
	{
		return FString();
	}
	return FPaths::ConvertRelativePathToFull(FPaths::Combine(Plugin->GetBaseDir(), TEXT("Scripts")));
}

FString FRammsNewtonWorkerClient::ResolvePythonExecutable()
{
	const URammsNewtonPhysicsSettings& Settings = URammsNewtonPhysicsSettings::Get();
	if (!Settings.PythonExecutablePath.IsEmpty())
	{
		if (FPaths::FileExists(Settings.PythonExecutablePath))
		{
			return Settings.PythonExecutablePath;
		}
		UE_LOG(LogRammsNewton, Warning,
			TEXT("PythonExecutablePath '%s' does not exist — falling back to the pinned venv"),
			*Settings.PythonExecutablePath);
	}

	const FString ScriptsDir = GetScriptsDir();
	if (ScriptsDir.IsEmpty())
	{
		return FString();
	}
#if PLATFORM_WINDOWS
	const FString Candidate = FPaths::Combine(ScriptsDir, TEXT(".venv"), TEXT("Scripts"), TEXT("python.exe"));
#else
	const FString Candidate = FPaths::Combine(ScriptsDir, TEXT(".venv"), TEXT("bin"), TEXT("python"));
#endif
	return FPaths::FileExists(Candidate) ? Candidate : FString();
}

bool FRammsNewtonWorkerClient::RunProbe(FRammsNewtonCapabilities& OutCapabilities)
{
	OutCapabilities = FRammsNewtonCapabilities();
	OutCapabilities.bProbed = true;

	const FString Python = ResolvePythonExecutable();
	const FString ScriptsDir = GetScriptsDir();
	if (Python.IsEmpty())
	{
		OutCapabilities.Error = TEXT(
			"No worker Python found. Set PythonExecutablePath in RAMMS Newton Physics "
			"settings or create the pinned venv at <Plugin>/Scripts/.venv (see Scripts/README.md).");
		return true; // definitive answer: unavailable
	}

	void* ReadPipe = nullptr;
	void* WritePipe = nullptr;
	if (!FPlatformProcess::CreatePipe(ReadPipe, WritePipe))
	{
		OutCapabilities.Error = TEXT("CreatePipe failed");
		return false;
	}

	FProcHandle Proc = FPlatformProcess::CreateProc(
		*Python,
		TEXT("-u -m newton_worker --probe"),
		/*bLaunchDetached=*/false,
		/*bLaunchHidden=*/true,
		/*bLaunchReallyHidden=*/true,
		/*OutProcessID=*/nullptr,
		/*PriorityModifier=*/0,
		*ScriptsDir,
		WritePipe);
	if (!Proc.IsValid())
	{
		FPlatformProcess::ClosePipe(ReadPipe, WritePipe);
		OutCapabilities.Error = FString::Printf(TEXT("Failed to launch '%s'"), *Python);
		return false;
	}

	const double Deadline =
		FPlatformTime::Seconds() + URammsNewtonPhysicsSettings::Get().ProbeTimeoutSeconds;
	FString Output;
	bool	bTimedOut = false;
	while (FPlatformProcess::IsProcRunning(Proc))
	{
		ReadAllAvailable(ReadPipe, Output);
		if (FPlatformTime::Seconds() > Deadline)
		{
			bTimedOut = true;
			FPlatformProcess::TerminateProc(Proc, /*KillTree=*/true);
			break;
		}
		FPlatformProcess::Sleep(0.05f);
	}
	ReadAllAvailable(ReadPipe, Output);
	FPlatformProcess::CloseProc(Proc);
	FPlatformProcess::ClosePipe(ReadPipe, WritePipe);

	if (bTimedOut)
	{
		OutCapabilities.Error = TEXT("Probe timed out");
		return false;
	}

	// The contract is one JSON object line on stdout; be tolerant of stray
	// lines and take the last one that parses.
	TArray<FString> Lines;
	Output.ParseIntoArrayLines(Lines);
	TSharedPtr<FJsonObject> Json;
	for (int32 Index = Lines.Num() - 1; Index >= 0; --Index)
	{
		const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Lines[Index]);
		TSharedPtr<FJsonObject>			Candidate;
		if (FJsonSerializer::Deserialize(Reader, Candidate) && Candidate.IsValid())
		{
			Json = Candidate;
			break;
		}
	}
	if (!Json.IsValid())
	{
		OutCapabilities.Error =
			FString::Printf(TEXT("Probe emitted no parseable JSON. Output: %s"), *Output.Left(512));
		return false;
	}

	ParseCapabilities(Json, OutCapabilities);
	return true;
}

void FRammsNewtonWorkerClient::ParseCapabilities(
	const TSharedPtr<FJsonObject>& Json, FRammsNewtonCapabilities& Out)
{
	Out.bProbed = true;
	Json->TryGetStringField(TEXT("python"), Out.PythonVersion);
	GetStringArrayField(Json, TEXT("solvers"), Out.Solvers);
	Out.bAvailable = Out.Solvers.Num() > 0;

	const TSharedPtr<FJsonObject>* Package = nullptr;
	if (Json->TryGetObjectField(TEXT("newton"), Package))
	{
		(*Package)->TryGetStringField(TEXT("version"), Out.NewtonVersion);
		if (!Out.bAvailable)
		{
			(*Package)->TryGetStringField(TEXT("error"), Out.Error);
		}
	}
	const TSharedPtr<FJsonObject>* Cuda = nullptr;
	if (Json->TryGetObjectField(TEXT("cuda"), Cuda))
	{
		(*Cuda)->TryGetBoolField(TEXT("available"), Out.bCudaAvailable);
		(*Cuda)->TryGetStringField(TEXT("device"), Out.CudaDeviceName);
	}
	if (!Out.bAvailable && Out.Error.IsEmpty())
	{
		Out.Error = TEXT("Worker env offers no solvers (newton or mujoco import failed?)");
	}
}

bool FRammsNewtonWorkerClient::Start(FString& OutError)
{
	if (State == ERammsNewtonWorkerState::Ready && IsWorkerProcessAlive())
	{
		return true;
	}
	Stop(/*bRequestShutdown=*/false);

	const FString Python = ResolvePythonExecutable();
	const FString ScriptsDir = GetScriptsDir();
	if (Python.IsEmpty())
	{
		OutError = TEXT("No worker Python interpreter found (settings or Scripts/.venv)");
		State = ERammsNewtonWorkerState::Failed;
		return false;
	}

	State = ERammsNewtonWorkerState::Starting;

	// Random localhost port; on a bind collision the worker exits without
	// printing READY and we retry on a fresh port.
	for (int32 Attempt = 0; Attempt < 3; ++Attempt)
	{
		const int32 Port = FMath::RandRange(30500, 30999);
		Endpoint = FString::Printf(TEXT("tcp://127.0.0.1:%d"), Port);

		if (!FPlatformProcess::CreatePipe(StdOutReadPipe, StdOutWritePipe))
		{
			OutError = TEXT("CreatePipe failed");
			State = ERammsNewtonWorkerState::Failed;
			return false;
		}
		const FString Args =
			FString::Printf(TEXT("-u -m newton_worker serve --endpoint %s"), *Endpoint);
		WorkerProc = FPlatformProcess::CreateProc(
			*Python, *Args,
			/*bLaunchDetached=*/false,
			/*bLaunchHidden=*/true,
			/*bLaunchReallyHidden=*/true,
			/*OutProcessID=*/nullptr,
			/*PriorityModifier=*/0,
			*ScriptsDir,
			StdOutWritePipe);
		if (!WorkerProc.IsValid())
		{
			CloseProcess();
			OutError = FString::Printf(TEXT("Failed to launch '%s'"), *Python);
			State = ERammsNewtonWorkerState::Failed;
			return false;
		}

		const double Deadline = FPlatformTime::Seconds() + WorkerReadySeconds;
		FString		 Output;
		bool		 bReady = false;
		while (FPlatformTime::Seconds() < Deadline)
		{
			ReadAllAvailable(StdOutReadPipe, Output);
			if (Output.Contains(TEXT("READY ")))
			{
				bReady = true;
				break;
			}
			if (!FPlatformProcess::IsProcRunning(WorkerProc))
			{
				break; // likely port collision or import failure — retry
			}
			FPlatformProcess::Sleep(0.05f);
		}

		if (bReady)
		{
			if (!CreateSocket(OutError))
			{
				Stop(/*bRequestShutdown=*/false);
				State = ERammsNewtonWorkerState::Failed;
				return false;
			}
			State = ERammsNewtonWorkerState::Ready;
			UE_LOG(LogRammsNewton, Log, TEXT("Newton worker ready on %s"), *Endpoint);
			return true;
		}

		const bool bDied = !FPlatformProcess::IsProcRunning(WorkerProc);
		UE_LOG(LogRammsNewton, Warning,
			TEXT("Newton worker did not report READY on %s (%s), attempt %d"),
			*Endpoint, bDied ? TEXT("process exited") : TEXT("timeout"), Attempt + 1);
		CloseProcess();
		if (!bDied)
		{
			// Alive but silent is an env problem, not a port collision — no retry.
			break;
		}
	}

	OutError = TEXT("Worker failed to start (see log)");
	State = ERammsNewtonWorkerState::Failed;
	return false;
}

void FRammsNewtonWorkerClient::Stop(bool bRequestShutdown)
{
	if (bRequestShutdown && ZmqSocket && IsWorkerProcessAlive())
	{
		TSharedPtr<FJsonObject> Result;
		FString					Error;
		Request(TEXT("shutdown"), nullptr, ShutdownOpSeconds, Result, Error);
	}

	{
		FScopeLock Lock(&RequestMutex);
		DestroySocket();
	}

	if (WorkerProc.IsValid())
	{
		const double Deadline = FPlatformTime::Seconds() + ShutdownGraceSeconds;
		while (FPlatformProcess::IsProcRunning(WorkerProc) && FPlatformTime::Seconds() < Deadline)
		{
			DrainWorkerOutput(); // a blocked write would otherwise stall its exit
			FPlatformProcess::Sleep(0.05f);
		}
		if (FPlatformProcess::IsProcRunning(WorkerProc))
		{
			FPlatformProcess::TerminateProc(WorkerProc, /*KillTree=*/true);
		}
	}
	CloseProcess();
	State = ERammsNewtonWorkerState::Stopped;
}

void FRammsNewtonWorkerClient::CloseProcess()
{
	if (WorkerProc.IsValid())
	{
		FPlatformProcess::CloseProc(WorkerProc);
		WorkerProc.Reset();
	}
	if (StdOutReadPipe || StdOutWritePipe)
	{
		FPlatformProcess::ClosePipe(StdOutReadPipe, StdOutWritePipe);
		StdOutReadPipe = nullptr;
		StdOutWritePipe = nullptr;
	}
}

bool FRammsNewtonWorkerClient::IsWorkerProcessAlive() const
{
	// IsProcRunning takes a non-const ref in the platform API.
	FProcHandle& Handle = const_cast<FProcHandle&>(WorkerProc);
	return Handle.IsValid() && FPlatformProcess::IsProcRunning(Handle);
}

bool FRammsNewtonWorkerClient::CreateSocket(FString& OutError)
{
	if (!ZmqContext)
	{
		ZmqContext = zmq_ctx_new();
		if (!ZmqContext)
		{
			OutError = TEXT("zmq_ctx_new failed");
			return false;
		}
	}
	ZmqSocket = zmq_socket(ZmqContext, ZMQ_REQ);
	if (!ZmqSocket)
	{
		OutError = TEXT("zmq_socket failed");
		return false;
	}
	const int Linger = 0;
	zmq_setsockopt(ZmqSocket, ZMQ_LINGER, &Linger, sizeof(Linger));
	if (zmq_connect(ZmqSocket, TCHAR_TO_ANSI(*Endpoint)) != 0)
	{
		OutError = FString::Printf(TEXT("zmq_connect(%s) failed: %hs"), *Endpoint, zmq_strerror(zmq_errno()));
		DestroySocket();
		return false;
	}
	return true;
}

void FRammsNewtonWorkerClient::DrainWorkerOutput()
{
	if (!StdOutReadPipe)
	{
		return;
	}
	const FString Chunk = FPlatformProcess::ReadPipe(StdOutReadPipe);
	if (!Chunk.IsEmpty())
	{
		UE_LOG(LogRammsNewton, VeryVerbose, TEXT("[worker] %s"), *Chunk.Left(2048));
	}
}

void FRammsNewtonWorkerClient::DestroySocket()
{
	bStepPending.store(false);
	if (ZmqSocket)
	{
		zmq_close(ZmqSocket);
		ZmqSocket = nullptr;
	}
	if (ZmqContext)
	{
		zmq_ctx_term(ZmqContext);
		ZmqContext = nullptr;
	}
}

bool FRammsNewtonWorkerClient::SendRequest(
	const TCHAR*				   Op,
	const TSharedPtr<FJsonObject>& Params,
	double						   TimeoutSeconds,
	int32&						   OutRequestId,
	FString&					   OutError)
{
	if (!ZmqSocket)
	{
		OutError = TEXT("worker not connected");
		return false;
	}

	OutRequestId = NextRequestId++;
	const TSharedRef<FJsonObject> Message = MakeShared<FJsonObject>();
	Message->SetNumberField(TEXT("id"), OutRequestId);
	Message->SetStringField(TEXT("op"), Op);
	if (Params.IsValid())
	{
		Message->SetObjectField(TEXT("params"), Params);
	}

	FString																   Payload;
	const TSharedRef<TJsonWriter<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>> Writer =
		TJsonWriterFactory<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>::Create(&Payload);
	FJsonSerializer::Serialize(Message, Writer);

	const FTCHARToUTF8 Utf8(*Payload);
	const int		   SendTimeoutMs = FMath::Max(1, static_cast<int>(TimeoutSeconds * 1000.0));
	// Receive in short slices so we can service the worker's output pipe
	// between polls: the child's stdout AND stderr feed our pipe, and a full
	// pipe buffer blocks the worker mid-write (observed: trimesh warning spam
	// during mesh-heavy load_model deadlocked the whole load). Draining here
	// is what keeps long ops safe; it also gives dead-worker fast-fail.
	const int PollTimeoutMs = 250;
	zmq_setsockopt(ZmqSocket, ZMQ_SNDTIMEO, &SendTimeoutMs, sizeof(SendTimeoutMs));
	zmq_setsockopt(ZmqSocket, ZMQ_RCVTIMEO, &PollTimeoutMs, sizeof(PollTimeoutMs));

	if (zmq_send(ZmqSocket, Utf8.Get(), Utf8.Length(), 0) < 0)
	{
		OutError = FString::Printf(TEXT("send '%s' failed: %hs"), Op, zmq_strerror(zmq_errno()));
		// A REQ socket is poisoned after a failed exchange — rebuild it.
		bStepPending.store(false);
		DestroySocket();
		FString Unused;
		CreateSocket(Unused);
		return false;
	}
	return true;
}

bool FRammsNewtonWorkerClient::RecvReply(
	int32					 RequestId,
	const TCHAR*			 Op,
	double					 TimeoutSeconds,
	TSharedPtr<FJsonObject>& OutResult,
	FString&				 OutError)
{
	if (!ZmqSocket)
	{
		OutError = TEXT("worker not connected");
		return false;
	}

	// A stale reply (from a previous timed-out exchange that the server still
	// answered) carries the wrong id; skip past it within the deadline.
	const double Deadline = FPlatformTime::Seconds() + TimeoutSeconds;
	for (;;)
	{
		zmq_msg_t Reply;
		zmq_msg_init(&Reply);
		if (zmq_msg_recv(&Reply, ZmqSocket, 0) < 0)
		{
			zmq_msg_close(&Reply);
			DrainWorkerOutput();
			const bool bAlive = IsWorkerProcessAlive();
			if (bAlive && FPlatformTime::Seconds() < Deadline)
			{
				continue; // still within budget — keep polling (and draining)
			}
			OutError = FString::Printf(
				TEXT("no reply to '%s' within %.1fs (%s)"),
				Op, TimeoutSeconds,
				bAlive ? TEXT("worker alive but unresponsive") : TEXT("worker process is dead"));
			bStepPending.store(false);
			DestroySocket();
			FString Unused;
			CreateSocket(Unused);
			return false;
		}
		DrainWorkerOutput();

		FUTF8ToTCHAR  Wide(static_cast<const ANSICHAR*>(zmq_msg_data(&Reply)), zmq_msg_size(&Reply));
		const FString ReplyText(Wide.Length(), Wide.Get());
		zmq_msg_close(&Reply);

		TSharedPtr<FJsonObject>			ReplyJson;
		const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(ReplyText);
		if (!FJsonSerializer::Deserialize(Reader, ReplyJson) || !ReplyJson.IsValid())
		{
			OutError = FString::Printf(TEXT("unparseable reply to '%s'"), Op);
			return false;
		}

		int32 ReplyId = -1;
		ReplyJson->TryGetNumberField(TEXT("id"), ReplyId);
		if (ReplyId != RequestId)
		{
			if (FPlatformTime::Seconds() > Deadline)
			{
				OutError = FString::Printf(TEXT("only stale replies to '%s'"), Op);
				return false;
			}
			continue; // stale — but REQ/REP strictly alternates, so this is at most once
		}

		bool bOk = false;
		ReplyJson->TryGetBoolField(TEXT("ok"), bOk);
		if (!bOk)
		{
			FString						   Code = TEXT("unknown");
			FString						   ErrorMessage;
			const TSharedPtr<FJsonObject>* ErrorObject = nullptr;
			if (ReplyJson->TryGetObjectField(TEXT("error"), ErrorObject))
			{
				(*ErrorObject)->TryGetStringField(TEXT("code"), Code);
				(*ErrorObject)->TryGetStringField(TEXT("message"), ErrorMessage);
			}
			OutError = FString::Printf(TEXT("[%s] %s"), *Code, *ErrorMessage);
			return false;
		}

		const TSharedPtr<FJsonObject>* Result = nullptr;
		if (ReplyJson->TryGetObjectField(TEXT("result"), Result))
		{
			OutResult = *Result;
		}
		else
		{
			OutResult = MakeShared<FJsonObject>();
		}
		return true;
	}
}

void FRammsNewtonWorkerClient::DrainPendingStep()
{
	if (!bStepPending.load())
	{
		return;
	}
	// Discarding the reply is safe: the callers that get here are lifecycle
	// ops (reset / set_state / shutdown) that re-establish state themselves.
	TSharedPtr<FJsonObject> Discarded;
	FString					Error;
	RecvReply(PendingStepId, TEXT("step(pipelined)"),
		URammsNewtonPhysicsSettings::Get().StepTimeoutSeconds, Discarded, Error);
	bStepPending.store(false);
}

bool FRammsNewtonWorkerClient::Request(
	const TCHAR*				   Op,
	const TSharedPtr<FJsonObject>& Params,
	double						   TimeoutSeconds,
	TSharedPtr<FJsonObject>&	   OutResult,
	FString&					   OutError)
{
	FScopeLock Lock(&RequestMutex);
	DrainPendingStep();

	int32 RequestId = 0;
	if (!SendRequest(Op, Params, TimeoutSeconds, RequestId, OutError))
	{
		return false;
	}
	return RecvReply(RequestId, Op, TimeoutSeconds, OutResult, OutError);
}

bool FRammsNewtonWorkerClient::Hello(FRammsNewtonCapabilities& OutCapabilities, FString& OutError)
{
	const TSharedRef<FJsonObject> Params = MakeShared<FJsonObject>();
	Params->SetNumberField(TEXT("protocol"), 1);
	TSharedPtr<FJsonObject> Result;
	if (!Request(TEXT("hello"), Params, URammsNewtonPhysicsSettings::Get().ProbeTimeoutSeconds, Result, OutError))
	{
		return false;
	}
	ParseCapabilities(Result, OutCapabilities);
	bool bCompatible = false;
	Result->TryGetBoolField(TEXT("protocol_compatible"), bCompatible);
	if (!bCompatible)
	{
		OutError = TEXT("worker protocol version mismatch");
		return false;
	}
	return true;
}

bool FRammsNewtonWorkerClient::LoadModel(
	const FString&						MjcfXml,
	const TMap<FString, TArray<uint8>>& Assets,
	const FString&						SolverWireName,
	FRammsNewtonModelInfo&				OutInfo,
	FString&							OutError)
{
	const TSharedRef<FJsonObject> Params = MakeShared<FJsonObject>();
	Params->SetStringField(TEXT("mjcf_xml"), MjcfXml);
	Params->SetStringField(TEXT("solver"), SolverWireName);
	if (Assets.Num() > 0)
	{
		const TSharedRef<FJsonObject> AssetsJson = MakeShared<FJsonObject>();
		for (const TPair<FString, TArray<uint8>>& Asset : Assets)
		{
			AssetsJson->SetStringField(Asset.Key, FBase64::Encode(Asset.Value));
		}
		Params->SetObjectField(TEXT("assets"), AssetsJson);
	}

	TSharedPtr<FJsonObject> Result;
	if (!Request(TEXT("load_model"), Params, URammsNewtonPhysicsSettings::Get().LoadTimeoutSeconds, Result, OutError))
	{
		return false;
	}

	OutInfo = FRammsNewtonModelInfo();
	Result->TryGetStringField(TEXT("solver"), OutInfo.Solver);
	Result->TryGetNumberField(TEXT("timestep"), OutInfo.Timestep);
	Result->TryGetNumberField(TEXT("nq"), OutInfo.Nq);
	Result->TryGetNumberField(TEXT("nv"), OutInfo.Nv);
	Result->TryGetNumberField(TEXT("nu"), OutInfo.Nu);
	Result->TryGetNumberField(TEXT("na"), OutInfo.Na);
	Result->TryGetNumberField(TEXT("nmocap"), OutInfo.Nmocap);
	GetStringArrayField(Result, TEXT("joint_names"), OutInfo.JointNames);
	GetStringArrayField(Result, TEXT("actuator_names"), OutInfo.ActuatorNames);
	GetStringArrayField(Result, TEXT("warnings"), OutInfo.Warnings);
	return true;
}

bool FRammsNewtonWorkerClient::ParseStepResult(
	const TSharedPtr<FJsonObject>& Result, FRammsNewtonStepResult& Out)
{
	if (!Result.IsValid())
	{
		return false;
	}
	Result->TryGetNumberField(TEXT("time"), Out.Time);
	double StepCount = 0.0;
	Result->TryGetNumberField(TEXT("step_count"), StepCount);
	Out.StepCount = static_cast<int64>(StepCount);
	const bool bHaveQpos = GetDoubleArrayField(Result, TEXT("qpos"), Out.Qpos);
	const bool bHaveQvel = GetDoubleArrayField(Result, TEXT("qvel"), Out.Qvel);
	GetDoubleArrayField(Result, TEXT("act"), Out.Act);
	return bHaveQpos && bHaveQvel;
}

TSharedRef<FJsonObject> FRammsNewtonWorkerClient::BuildStepParams(
	TConstArrayView<double> Ctrl,
	TConstArrayView<double> MocapPos,
	TConstArrayView<double> MocapQuat,
	int32					Nsteps)
{
	const TSharedRef<FJsonObject> Params = MakeShared<FJsonObject>();
	Params->SetNumberField(TEXT("nsteps"), Nsteps);
	if (Ctrl.Num() > 0)
	{
		TArray<TSharedPtr<FJsonValue>> CtrlJson;
		CtrlJson.Reserve(Ctrl.Num());
		for (double Value : Ctrl)
		{
			CtrlJson.Add(MakeShared<FJsonValueNumber>(Value));
		}
		Params->SetArrayField(TEXT("ctrl"), CtrlJson);
	}
	auto AddNestedArray = [&Params](const TCHAR* Field, TConstArrayView<double> Flat, int32 Stride) {
		if (Flat.Num() == 0)
		{
			return;
		}
		TArray<TSharedPtr<FJsonValue>> Rows;
		for (int32 Base = 0; Base + Stride <= Flat.Num(); Base += Stride)
		{
			TArray<TSharedPtr<FJsonValue>> Row;
			Row.Reserve(Stride);
			for (int32 Offset = 0; Offset < Stride; ++Offset)
			{
				Row.Add(MakeShared<FJsonValueNumber>(Flat[Base + Offset]));
			}
			Rows.Add(MakeShared<FJsonValueArray>(Row));
		}
		Params->SetArrayField(Field, Rows);
	};
	AddNestedArray(TEXT("mocap_pos"), MocapPos, 3);
	AddNestedArray(TEXT("mocap_quat"), MocapQuat, 4);
	return Params;
}

bool FRammsNewtonWorkerClient::Step(
	TConstArrayView<double> Ctrl,
	TConstArrayView<double> MocapPos,
	TConstArrayView<double> MocapQuat,
	int32					Nsteps,
	FRammsNewtonStepResult& OutState,
	FString&				OutError)
{
	TSharedPtr<FJsonObject> Result;
	if (!Request(TEXT("step"), BuildStepParams(Ctrl, MocapPos, MocapQuat, Nsteps),
			URammsNewtonPhysicsSettings::Get().StepTimeoutSeconds, Result, OutError))
	{
		return false;
	}
	if (!ParseStepResult(Result, OutState))
	{
		OutError = TEXT("step reply missing qpos/qvel");
		return false;
	}
	return true;
}

bool FRammsNewtonWorkerClient::StepBegin(
	TConstArrayView<double> Ctrl,
	TConstArrayView<double> MocapPos,
	TConstArrayView<double> MocapQuat,
	int32					Nsteps,
	FString&				OutError)
{
	FScopeLock Lock(&RequestMutex);
	DrainPendingStep(); // at most one step in flight (REQ/REP alternation)

	int32 RequestId = 0;
	if (!SendRequest(TEXT("step"), BuildStepParams(Ctrl, MocapPos, MocapQuat, Nsteps),
			URammsNewtonPhysicsSettings::Get().StepTimeoutSeconds, RequestId, OutError))
	{
		return false;
	}
	PendingStepId = RequestId;
	bStepPending.store(true);
	return true;
}

bool FRammsNewtonWorkerClient::StepCollect(FRammsNewtonStepResult& OutState, FString& OutError)
{
	FScopeLock Lock(&RequestMutex);
	if (!bStepPending.load())
	{
		OutError = TEXT("no step in flight");
		return false;
	}
	TSharedPtr<FJsonObject> Result;
	const bool				bOk = RecvReply(PendingStepId, TEXT("step"),
		URammsNewtonPhysicsSettings::Get().StepTimeoutSeconds, Result, OutError);
	bStepPending.store(false);
	if (!bOk)
	{
		return false;
	}
	if (!ParseStepResult(Result, OutState))
	{
		OutError = TEXT("step reply missing qpos/qvel");
		return false;
	}
	return true;
}

bool FRammsNewtonWorkerClient::SetState(
	TConstArrayView<double> Qpos,
	TConstArrayView<double> Qvel,
	TConstArrayView<double> Act,
	double					Time,
	FRammsNewtonStepResult& OutState,
	FString&				OutError)
{
	const TSharedRef<FJsonObject> Params = MakeShared<FJsonObject>();
	auto						  AddArray = [&Params](const TCHAR* Field, TConstArrayView<double> Values) {
		if (Values.Num() == 0)
		{
			return;
		}
		TArray<TSharedPtr<FJsonValue>> Json;
		Json.Reserve(Values.Num());
		for (double Value : Values)
		{
			Json.Add(MakeShared<FJsonValueNumber>(Value));
		}
		Params->SetArrayField(Field, Json);
	};
	AddArray(TEXT("qpos"), Qpos);
	AddArray(TEXT("qvel"), Qvel);
	AddArray(TEXT("act"), Act);
	if (Time >= 0.0)
	{
		Params->SetNumberField(TEXT("time"), Time);
	}

	TSharedPtr<FJsonObject> Result;
	if (!Request(TEXT("set_state"), Params,
			URammsNewtonPhysicsSettings::Get().StepTimeoutSeconds, Result, OutError))
	{
		return false;
	}
	if (!ParseStepResult(Result, OutState))
	{
		OutError = TEXT("set_state reply missing qpos/qvel");
		return false;
	}
	return true;
}

bool FRammsNewtonWorkerClient::ResetSim(FRammsNewtonStepResult& OutState, FString& OutError)
{
	TSharedPtr<FJsonObject> Result;
	// reset is a full model rebuild in the worker — allow the load budget.
	if (!Request(TEXT("reset"), nullptr, URammsNewtonPhysicsSettings::Get().LoadTimeoutSeconds, Result, OutError))
	{
		return false;
	}
	if (!ParseStepResult(Result, OutState))
	{
		OutError = TEXT("reset reply missing qpos/qvel");
		return false;
	}
	return true;
}
