// Copyright RAMMP. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DeveloperSettings.h"
#include "RammsNewtonPhysicsTypes.h"
#include "RammsNewtonPhysicsSettings.generated.h"

/**
 * Project settings for the Newton solver backend.
 *
 * Newton runs out-of-process (Python worker under Scripts/newton_worker);
 * nothing here affects platforms where the worker env is absent — the
 * availability probe simply reports Unavailable and URLab keeps stepping
 * MuJoCo locally.
 */
UCLASS(Config = Game, DefaultConfig, meta = (DisplayName = "RAMMS Newton Physics"))
class RAMMSNEWTONPHYSICS_API URammsNewtonPhysicsSettings : public UDeveloperSettings
{
	GENERATED_BODY()

public:
	URammsNewtonPhysicsSettings()
	{
		CategoryName = TEXT("Plugins");
	}

	static const URammsNewtonPhysicsSettings& Get()
	{
		return *GetDefault<URammsNewtonPhysicsSettings>();
	}

	/**
	 * Python interpreter for the worker. Empty = auto-locate the pinned venv
	 * at <Plugin>/Scripts/.venv (Scripts/python.exe on Windows, bin/python
	 * elsewhere).
	 */
	UPROPERTY(EditAnywhere, Config, Category = "Worker")
	FString PythonExecutablePath;

	/** Solver the worker constructs when a Newton solver component activates. */
	UPROPERTY(EditAnywhere, Config, Category = "Worker")
	ERammsNewtonSolver Solver = ERammsNewtonSolver::MuJoCoWarp;

	/**
	 * When the preferred solver is not offered by the probe (e.g. no CUDA),
	 * fall back to the CPU solver instead of reporting unavailable.
	 */
	UPROPERTY(EditAnywhere, Config, Category = "Worker")
	bool bFallBackToCpuSolver = true;

	/** Seconds allowed for `--probe` (imports Newton; cold imports are slow). */
	UPROPERTY(EditAnywhere, Config, Category = "Timeouts", meta = (ClampMin = "5.0"))
	float ProbeTimeoutSeconds = 60.0f;

	/**
	 * Seconds allowed for load_model. First load on a cold warp kernel cache
	 * compiles GPU kernels and can take minutes; warm-cache loads take
	 * seconds.
	 */
	UPROPERTY(EditAnywhere, Config, Category = "Timeouts", meta = (ClampMin = "10.0"))
	float LoadTimeoutSeconds = 600.0f;

	/**
	 * Seconds allowed for a single step exchange on the physics thread.
	 * On timeout the step handler falls back to local mj_step and the
	 * Newton bridge deactivates with a warning.
	 */
	UPROPERTY(EditAnywhere, Config, Category = "Timeouts", meta = (ClampMin = "0.05"))
	float StepTimeoutSeconds = 2.0f;

	/**
	 * Probe availability automatically the first time something asks for it
	 * (subsystem/component). Disable to only probe on explicit request.
	 */
	UPROPERTY(EditAnywhere, Config, Category = "Probe")
	bool bAutoProbe = true;

	/**
	 * After load_model, verify liveness: step the model and require state to
	 * be finite and time to advance. Catches silently-miscompiled kernel
	 * caches (observed in the wild) at the cost of a few worker steps.
	 */
	UPROPERTY(EditAnywhere, Config, Category = "Probe")
	bool bVerifyLivenessAfterLoad = true;
};
