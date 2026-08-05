// Copyright RAMMP. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RammsNewtonPhysicsTypes.h"
#include "Subsystems/EngineSubsystem.h"
#include "RammsNewtonPhysicsSubsystem.generated.h"

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(
	FOnRammsNewtonProbeCompleted, const FRammsNewtonCapabilities&, Capabilities);

/**
 * App-wide Newton availability cache (probing spawns a Python subprocess and
 * imports Newton, so results are cached for the session).
 *
 * Diagnostics/tooling surface only — URammsNewtonSolverComponent talks to
 * the worker directly and does not depend on this subsystem.
 */
UCLASS()
class RAMMSNEWTONPHYSICS_API URammsNewtonPhysicsSubsystem : public UEngineSubsystem
{
	GENERATED_BODY()

public:
	/** Cached result; unprobed default (bProbed=false) until a probe ran. */
	UFUNCTION(BlueprintPure, Category = "RAMMS|Newton")
	FRammsNewtonCapabilities GetCachedCapabilities() const;

	UFUNCTION(BlueprintPure, Category = "RAMMS|Newton")
	bool IsNewtonAvailable() const;

	/** True while a background probe is running. */
	UFUNCTION(BlueprintPure, Category = "RAMMS|Newton")
	bool IsProbing() const { return bProbeInFlight; }

	/**
	 * Probe on a background thread; OnProbeCompleted broadcasts on the game
	 * thread when done. No-op if a probe is already in flight, or if a cached
	 * result exists and bForceReprobe is false (the cache is broadcast).
	 */
	UFUNCTION(BlueprintCallable, Category = "RAMMS|Newton")
	void ProbeAvailabilityAsync(bool bForceReprobe = false);

	/** Blocking probe (up to ProbeTimeoutSeconds) — prefer the async variant. */
	UFUNCTION(BlueprintCallable, Category = "RAMMS|Newton")
	FRammsNewtonCapabilities ProbeAvailability(bool bForceReprobe = false);

	UPROPERTY(BlueprintAssignable, Category = "RAMMS|Newton")
	FOnRammsNewtonProbeCompleted OnProbeCompleted;

private:
	void StoreCapabilities(const FRammsNewtonCapabilities& InCapabilities);

	mutable FCriticalSection CacheMutex;
	FRammsNewtonCapabilities Cached;
	bool					 bProbeInFlight = false;
};
