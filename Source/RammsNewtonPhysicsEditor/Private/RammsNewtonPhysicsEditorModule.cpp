// Copyright RAMMP. All Rights Reserved.

// Editor actions for the Newton backend (plan §5.5 Milestone D, first cut):
//   Tools ▸ RAMMS Newton ▸
//     Probe Newton Availability   — async env probe, result as a toast
//     Validate Scene Under Newton — dry-run worker load of the compiled model
//     Export Compiled Scene       — MJCF + assets to Saved/NewtonExport/
// The latter two need a running session (PIE) with a compiled AAMjManager
// model; entries are disabled otherwise.

#include "Async/Async.h"
#include "Framework/Notifications/NotificationManager.h"
#include "HAL/PlatformProcess.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "RammsNewtonPhysicsSettings.h"
#include "RammsNewtonPhysicsTypes.h"
#include "RammsNewtonSolverComponent.h"
#include "RammsNewtonWorkerClient.h"
#include "ToolMenus.h"
#include "Widgets/Notifications/SNotificationList.h"

#define LOCTEXT_NAMESPACE "RammsNewtonPhysicsEditor"

namespace
{

	void ShowToast(const FString& Message, bool bSuccess, float Seconds = 8.0f)
	{
		FNotificationInfo Info(FText::AsCultureInvariant(Message));
		Info.ExpireDuration = Seconds;
		Info.bFireAndForget = true;
		TSharedPtr<SNotificationItem> Item = FSlateNotificationManager::Get().AddNotification(Info);
		if (Item.IsValid())
		{
			Item->SetCompletionState(
				bSuccess ? SNotificationItem::CS_Success : SNotificationItem::CS_Fail);
		}
	}

	/** Compiled-model engine of the current session, or null (gates menu entries). */
	UMjPhysicsEngine* GetCompiledEngine()
	{
		AAMjManager* Manager = AAMjManager::GetManager();
		if (Manager && Manager->PhysicsEngine && Manager->PhysicsEngine->GetModel())
		{
			return Manager->PhysicsEngine;
		}
		return nullptr;
	}

	void ProbeAvailability()
	{
		Async(EAsyncExecution::ThreadPool, []() {
			FRammsNewtonCapabilities Caps;
			FRammsNewtonWorkerClient::RunProbe(Caps);
			AsyncTask(ENamedThreads::GameThread, [Caps]() {
				if (Caps.bAvailable)
				{
					ShowToast(
						FString::Printf(
							TEXT("Newton available — newton %s, python %s, CUDA %s, solvers: %s"),
							*Caps.NewtonVersion, *Caps.PythonVersion,
							Caps.bCudaAvailable ? *Caps.CudaDeviceName : TEXT("no"),
							*FString::Join(Caps.Solvers, TEXT(", "))),
						true);
				}
				else
				{
					ShowToast(FString::Printf(TEXT("Newton unavailable: %s"), *Caps.Error), false, 12.0f);
				}
			});
		});
	}

	void ValidateScene()
	{
		UMjPhysicsEngine* Engine = GetCompiledEngine();
		if (!Engine)
		{
			ShowToast(TEXT("No compiled MuJoCo model — start a session (PIE) with an AMjManager first"), false);
			return;
		}

		FString						 Xml;
		TMap<FString, TArray<uint8>> Assets;
		FString						 Error;
		if (!URammsNewtonSolverComponent::SerializeCompiledModel(Engine, Xml, Assets, Error))
		{
			ShowToast(FString::Printf(TEXT("Scene serialization failed: %s"), *Error), false);
			return;
		}

		ShowToast(TEXT("Validating scene under Newton (first run may compile GPU kernels)..."), true, 4.0f);
		Async(EAsyncExecution::ThreadPool, [Xml = MoveTemp(Xml), Assets = MoveTemp(Assets)]() {
			FRammsNewtonWorkerClient Client;
			FString					 WorkerError;
			FString					 Verdict;
			bool					 bOk = false;
			do
			{
				if (!Client.Start(WorkerError))
				{
					Verdict = FString::Printf(TEXT("worker failed to start: %s"), *WorkerError);
					break;
				}
				FRammsNewtonCapabilities Caps;
				if (!Client.Hello(Caps, WorkerError))
				{
					Verdict = FString::Printf(TEXT("handshake failed: %s"), *WorkerError);
					break;
				}
				const URammsNewtonPhysicsSettings& Settings = URammsNewtonPhysicsSettings::Get();
				FString							   Solver = RammsNewton::SolverWireName(Settings.Solver);
				if (!Caps.SupportsSolver(Solver) && Settings.bFallBackToCpuSolver
					&& Caps.SupportsSolver(TEXT("mujoco_cpu")))
				{
					Solver = TEXT("mujoco_cpu");
				}
				FRammsNewtonModelInfo Info;
				if (!Client.LoadModel(Xml, Assets, Solver, Info, WorkerError))
				{
					Verdict = FString::Printf(TEXT("load failed: %s"), *WorkerError);
					break;
				}
				bOk = true;
				Verdict = FString::Printf(
					TEXT("Scene loads under Newton (%s): nq=%d nv=%d nu=%d nmocap=%d%s"),
					*Info.Solver, Info.Nq, Info.Nv, Info.Nu, Info.Nmocap,
					Info.Warnings.Num() > 0
						? *FString::Printf(TEXT(" — %d warning(s): %s"), Info.Warnings.Num(), *FString::Join(Info.Warnings, TEXT("; ")))
						: TEXT(""));
			}
			while (false);
			Client.Stop();
			AsyncTask(ENamedThreads::GameThread,
				[Verdict, bOk]() { ShowToast(Verdict, bOk, 12.0f); });
		});
	}

	void ExportCompiledScene()
	{
		UMjPhysicsEngine* Engine = GetCompiledEngine();
		if (!Engine)
		{
			ShowToast(TEXT("No compiled MuJoCo model — start a session (PIE) with an AMjManager first"), false);
			return;
		}

		FString						 Xml;
		TMap<FString, TArray<uint8>> Assets;
		FString						 Error;
		if (!URammsNewtonSolverComponent::SerializeCompiledModel(Engine, Xml, Assets, Error))
		{
			ShowToast(FString::Printf(TEXT("Scene serialization failed: %s"), *Error), false);
			return;
		}

		const FString ExportDir = FPaths::Combine(
			FPaths::ProjectSavedDir(), TEXT("NewtonExport"),
			FDateTime::Now().ToString(TEXT("%Y%m%d_%H%M%S")));
		bool bOk = FFileHelper::SaveStringToFile(
			Xml, *FPaths::Combine(ExportDir, TEXT("scene.xml")),
			FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
		for (const TPair<FString, TArray<uint8>>& Asset : Assets)
		{
			bOk &= FFileHelper::SaveArrayToFile(Asset.Value, *FPaths::Combine(ExportDir, Asset.Key));
		}

		if (bOk)
		{
			ShowToast(FString::Printf(TEXT("Exported scene.xml + %d asset(s) to %s"), Assets.Num(), *ExportDir), true, 12.0f);
			FPlatformProcess::ExploreFolder(*ExportDir);
		}
		else
		{
			ShowToast(FString::Printf(TEXT("Export to %s failed (disk?)"), *ExportDir), false);
		}
	}

	void RegisterMenus()
	{
		UToolMenu*		  Menu = UToolMenus::Get()->ExtendMenu("LevelEditor.MainMenu.Tools");
		FToolMenuSection& Section = Menu->AddSection(
			"RammsNewton", LOCTEXT("RammsNewtonSection", "RAMMS Newton"));

		Section.AddMenuEntry(
			"RammsNewtonProbe",
			LOCTEXT("ProbeLabel", "Probe Newton Availability"),
			LOCTEXT("ProbeTooltip", "Run the out-of-process Newton environment probe (python/newton/CUDA/solvers) and report the result."),
			FSlateIcon(),
			FUIAction(FExecuteAction::CreateStatic(&ProbeAvailability)));

		Section.AddMenuEntry(
			"RammsNewtonValidate",
			LOCTEXT("ValidateLabel", "Validate Scene Under Newton"),
			LOCTEXT("ValidateTooltip", "Dry-run load the current compiled MuJoCo scene in the Newton worker and report layout/warnings. Needs a running session with a compiled model."),
			FSlateIcon(),
			FUIAction(
				FExecuteAction::CreateStatic(&ValidateScene),
				FCanExecuteAction::CreateLambda([]() { return GetCompiledEngine() != nullptr; })));

		Section.AddMenuEntry(
			"RammsNewtonExport",
			LOCTEXT("ExportLabel", "Export Compiled Scene (MJCF + Assets)"),
			LOCTEXT("ExportTooltip", "Write the compiled scene XML and its mesh/texture assets to Saved/NewtonExport — the artifact newton/mujoco/MJX training scripts load. Needs a running session with a compiled model."),
			FSlateIcon(),
			FUIAction(
				FExecuteAction::CreateStatic(&ExportCompiledScene),
				FCanExecuteAction::CreateLambda([]() { return GetCompiledEngine() != nullptr; })));
	}

} // namespace

class FRammsNewtonPhysicsEditorModule : public IModuleInterface
{
public:
	virtual void StartupModule() override
	{
		UToolMenus::RegisterStartupCallback(
			FSimpleMulticastDelegate::FDelegate::CreateStatic(&RegisterMenus));
	}

	virtual void ShutdownModule() override
	{
		UToolMenus::UnregisterOwner(this);
	}
};

IMPLEMENT_MODULE(FRammsNewtonPhysicsEditorModule, RammsNewtonPhysicsEditor)

#undef LOCTEXT_NAMESPACE
