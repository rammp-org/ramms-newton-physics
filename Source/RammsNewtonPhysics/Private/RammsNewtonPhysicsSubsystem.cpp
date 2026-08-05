// Copyright RAMMP. All Rights Reserved.

#include "RammsNewtonPhysicsSubsystem.h"

#include "Async/Async.h"
#include "Misc/ScopeLock.h"
#include "RammsNewtonWorkerClient.h"

FRammsNewtonCapabilities URammsNewtonPhysicsSubsystem::GetCachedCapabilities() const
{
	FScopeLock Lock(&CacheMutex);
	return Cached;
}

bool URammsNewtonPhysicsSubsystem::IsNewtonAvailable() const
{
	FScopeLock Lock(&CacheMutex);
	return Cached.bProbed && Cached.bAvailable;
}

FRammsNewtonCapabilities URammsNewtonPhysicsSubsystem::ProbeAvailability(bool bForceReprobe)
{
	{
		FScopeLock Lock(&CacheMutex);
		if (Cached.bProbed && !bForceReprobe)
		{
			return Cached;
		}
	}
	FRammsNewtonCapabilities Capabilities;
	FRammsNewtonWorkerClient::RunProbe(Capabilities);
	StoreCapabilities(Capabilities);
	return Capabilities;
}

void URammsNewtonPhysicsSubsystem::ProbeAvailabilityAsync(bool bForceReprobe)
{
	if (bProbeInFlight)
	{
		return;
	}
	{
		FScopeLock Lock(&CacheMutex);
		if (Cached.bProbed && !bForceReprobe)
		{
			OnProbeCompleted.Broadcast(Cached);
			return;
		}
	}

	bProbeInFlight = true;
	TWeakObjectPtr<URammsNewtonPhysicsSubsystem> WeakThis(this);
	Async(EAsyncExecution::ThreadPool,
		[WeakThis]() {
			FRammsNewtonCapabilities Capabilities;
			FRammsNewtonWorkerClient::RunProbe(Capabilities);
			AsyncTask(ENamedThreads::GameThread,
				[WeakThis, Capabilities]() {
					if (URammsNewtonPhysicsSubsystem* This = WeakThis.Get())
					{
						This->StoreCapabilities(Capabilities);
						This->bProbeInFlight = false;
						This->OnProbeCompleted.Broadcast(Capabilities);
					}
				});
		});
}

void URammsNewtonPhysicsSubsystem::StoreCapabilities(const FRammsNewtonCapabilities& InCapabilities)
{
	FScopeLock Lock(&CacheMutex);
	Cached = InCapabilities;
	UE_LOG(LogRammsNewton, Log,
		TEXT("Newton availability: %s (newton %s, cuda %s%s%s)"),
		InCapabilities.bAvailable ? TEXT("AVAILABLE") : TEXT("unavailable"),
		*InCapabilities.NewtonVersion,
		InCapabilities.bCudaAvailable ? TEXT("yes: ") : TEXT("no"),
		InCapabilities.bCudaAvailable ? *InCapabilities.CudaDeviceName : TEXT(""),
		InCapabilities.Error.IsEmpty()
			? TEXT("")
			: *FString::Printf(TEXT(", error: %s"), *InCapabilities.Error));
}
