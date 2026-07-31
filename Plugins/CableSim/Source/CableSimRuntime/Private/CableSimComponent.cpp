#include "CableSimComponent.h"

#include "CableSimSolver.h"
#include "CableSimWorldCollisionProvider.h"
#include "DebugRenderSceneProxy.h"
#include "Engine/World.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformTime.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "SceneManagement.h"
#include "Serialization/CustomVersion.h"

namespace
{
	const FGuid CableSimObjectVersionGuid(0x47F49E33, 0x084E48C4, 0xA8F536A2, 0x53338B17);
	enum class ECableSimObjectVersion : int32
	{
		BeforeLinearDensity = 0,
		LinearDensity = 1,
		Latest = LinearDensity
	};
	FCustomVersionRegistration CableSimObjectVersionRegistration(
		CableSimObjectVersionGuid,
		static_cast<int32>(ECableSimObjectVersion::Latest),
		TEXT("CableSimObjectVersion"));

	FLinearColor CableSimRoleColor(const ECableSimEndpointState State, const FLinearColor& FreeColor)
	{
		switch (State)
		{
		case ECableSimEndpointState::Fixed: return FLinearColor::Red;
		case ECableSimEndpointState::Driven: return FLinearColor(1.0f, 0.5f, 0.0f);
		default: return FreeColor;
		}
	}

	class FCableSimDebugProxy final : public FDebugRenderSceneProxy
	{
	public:
		FCableSimDebugProxy(
			const UPrimitiveComponent* Component,
			const TConstArrayView<FVector3d> Polyline,
			const TConstArrayView<CableSim::FParticle> Particles,
			const ECableSimEndpointState StartState,
			const ECableSimEndpointState EndState,
			const TConstArrayView<CableSim::FContactConstraint> Contacts,
			const TConstArrayView<FVector3d> RejectedPoints,
			const FCableSimCollisionSnapshot& CollisionSnapshot,
			const FCableSimDebugSettings& Settings,
			const FCableSimStatus& Status)
			: FDebugRenderSceneProxy(Component)
		{
			DrawType = EDrawType::WireMesh;
			DrawAlpha = 255;
			const double MaximumTension = FMath::Max(Status.MaximumEstimatedTension, UE_DOUBLE_KINDA_SMALL_NUMBER);
			for (int32 Index = 0; Index + 1 < Polyline.Num(); ++Index)
			{
				FColor SegmentColor = Settings.CableColor.ToFColor(true);
				if (Settings.bColorByTension && Particles.IsValidIndex(Index) && Particles.IsValidIndex(Index + 1))
				{
					const double Tension = 0.5 * (Particles[Index].EstimatedTension + Particles[Index + 1].EstimatedTension);
					const double Alpha = FMath::Clamp(Tension / MaximumTension, 0.0, 1.0);
					SegmentColor = FLinearColor::LerpUsingHSV(
						FLinearColor(0.05f, 0.3f, 1.0f), FLinearColor(1.0f, 0.1f, 0.05f), Alpha).ToFColor(true);
				}
				Lines.Emplace(
					FVector(Polyline[Index]),
					FVector(Polyline[Index + 1]),
					SegmentColor,
					Settings.LineThickness);
			}
			if (Settings.bDrawParticles)
			{
				TArray<bool> bInContact;
				bInContact.Init(false, Particles.Num());
				for (const CableSim::FContactConstraint& Contact : Contacts)
				{
					if (bInContact.IsValidIndex(Contact.ParticleA)) bInContact[Contact.ParticleA] = true;
					if (bInContact.IsValidIndex(Contact.ParticleB)) bInContact[Contact.ParticleB] = true;
				}
				for (int32 Index = 0; Index < Polyline.Num(); ++Index)
				{
					FLinearColor Color = FLinearColor(0.2f, 0.9f, 0.3f);
					if (Index == 0)
					{
						Color = CableSimRoleColor(StartState, Color);
					}
					else if (Index + 1 == Polyline.Num())
					{
						Color = CableSimRoleColor(EndState, Color);
					}
					else if (bInContact.IsValidIndex(Index) && bInContact[Index])
					{
						Color = FLinearColor::Yellow;
					}
					Spheres.Emplace(1.25f, FVector(Polyline[Index]), Color);
				}
			}
			if (Settings.bDrawContacts)
			{
				for (const CableSim::FContactConstraint& Contact : Contacts)
				{
					if (!Particles.IsValidIndex(Contact.ParticleA)) continue;
					const FVector3d PositionA = Particles[Contact.ParticleA].Position;
					const FVector3d Point = Particles.IsValidIndex(Contact.ParticleB)
						? FMath::Lerp(PositionA, Particles[Contact.ParticleB].Position, Contact.SegmentAlpha)
						: PositionA;
					Spheres.Emplace(2.25f, FVector(Point), FLinearColor::Yellow);
					const double ArrowLength = FMath::Clamp(Contact.AccumulatedNormalCorrection, 0.0, 25.0) + 1.0;
					ArrowLines.Emplace(
						FVector(Point),
						FVector(Point + Contact.Normal * ArrowLength),
						FColor::Yellow,
						3.0f);
				}
			}
			if (Settings.bDrawFriction)
			{
				for (const CableSim::FContactConstraint& Contact : Contacts)
				{
					if (!Contact.bEnableFriction || Contact.AccumulatedNormalCorrection <= 0.0) continue;
					Spheres.Emplace(1.5f, FVector(Contact.FrictionAnchor), FLinearColor(0.6f, 0.2f, 0.9f));
					const FVector3d SlideDirection = Contact.SurfaceVelocity.GetSafeNormal();
					if (!SlideDirection.IsNearlyZero())
					{
						Lines.Emplace(
							FVector(Contact.FrictionAnchor),
							FVector(Contact.FrictionAnchor + SlideDirection * 5.0),
							FColor(150, 50, 230),
							1.0f);
					}
				}
			}
			if (Settings.bDrawRejectedContacts)
			{
				for (const FVector3d& Point : RejectedPoints)
				{
					Spheres.Emplace(2.0f, FVector(Point), FLinearColor::Red);
				}
			}
			if (Settings.bDrawCollisionGeometry)
			{
				for (const FCableSimNodeCollisionGeometry& Node : CollisionSnapshot.Nodes)
				{
					for (const CableSim::FCollisionTriangle& Triangle : Node.Triangles)
					{
						for (int32 Corner = 0; Corner < 3; ++Corner)
						{
							Lines.Emplace(
								FVector(Triangle.Vertices[Corner]),
								FVector(Triangle.Vertices[(Corner + 1) % 3]),
								FColor(80, 80, 80),
								0.5f);
						}
					}
					for (const CableSim::FCollisionEdge& Edge : Node.Edges)
					{
						if (Edge.Kind == CableSim::ECollisionEdgeKind::Convex)
						{
							Lines.Emplace(FVector(Edge.Start), FVector(Edge.End), FColor::Orange, 1.5f);
						}
					}
				}
			}
			if (Settings.bDrawPredictedBounds)
			{
				for (const FCableSimNodeCollisionGeometry& Node : CollisionSnapshot.Nodes)
				{
					const FVector Min = Node.PredictedBounds.Min;
					const FVector Max = Node.PredictedBounds.Max;
					FVector Corners[8];
					for (int32 Corner = 0; Corner < 8; ++Corner)
					{
						Corners[Corner] = FVector(
							(Corner & 1) ? Max.X : Min.X,
							(Corner & 2) ? Max.Y : Min.Y,
							(Corner & 4) ? Max.Z : Min.Z);
					}
					for (const int32 AxisBit : {1, 2, 4})
						for (int32 Corner = 0; Corner < 8; ++Corner)
							if ((Corner & AxisBit) == 0)
								Lines.Emplace(Corners[Corner], Corners[Corner | AxisBit], FColor(40, 180, 220), 0.25f);
				}
			}
			if (Settings.bDrawStatus && !Polyline.IsEmpty())
			{
				Texts.Emplace(
					FString::Printf(
						TEXT("CableSim  length=%.1f -> %.1f / %.1f  particles=%d  contacts=%d  strain=%.3f  accepted=%.2f  %.2f ms%s%s"),
						Status.ActiveLength,
						Status.TargetLength,
						Status.MaximumLength,
						Status.ParticleCount,
						Status.ContactCount,
						Status.MaximumSegmentStrain,
						Status.AcceptedMovementFraction,
						Status.SimulationMilliseconds,
						Status.bMotionClamped ? TEXT("  MOVEMENT LIMITED") : TEXT(""),
						Status.bBindingFailure ? TEXT("  BINDING INVALID") : TEXT("")),
					FVector(Polyline[0]) + FVector(0.0, 0.0, 15.0),
					Status.bMotionClamped ? FLinearColor::Yellow : FLinearColor::White);
				Texts.Emplace(
					FString::Printf(
						TEXT("penetration=%.2f  segment_error=%.2f  queries=%d  triangles=%d  edges=%d  rejected=%d%s%s%s"),
						Status.MaximumPenetration,
						Status.MaximumSegmentError,
						Status.CollisionQueryCount,
						Status.CollisionTriangleCount,
						Status.ConvexEdgeCount,
						Status.RejectedContactCount,
						Status.bCollisionDegraded ? TEXT("  COLLISION DEGRADED") : TEXT(""),
						Status.bLengthChanging ? TEXT("  LENGTH CHANGING") : TEXT(""),
						Status.bLengthClamped ? TEXT("  LENGTH CLAMPED") : TEXT("")),
					FVector(Polyline[0]) + FVector(0.0, 0.0, 9.0),
					Status.bCollisionDegraded ? FLinearColor::Red : FLinearColor(0.7f, 0.7f, 0.7f));
			}
		}

		virtual FPrimitiveViewRelevance GetViewRelevance(const FSceneView* View) const override
		{
			FPrimitiveViewRelevance Result;
			Result.bDrawRelevance = IsShown(View);
			Result.bDynamicRelevance = true;
			Result.bSeparateTranslucency = true;
			Result.bNormalTranslucency = true;
			return Result;
		}
	};

	CableSim::EEndpointState ToCoreState(const ECableSimEndpointState State)
	{
		switch (State)
		{
		case ECableSimEndpointState::Fixed:
			return CableSim::EEndpointState::Fixed;
		case ECableSimEndpointState::Driven:
			return CableSim::EEndpointState::Driven;
		default:
			return CableSim::EEndpointState::Free;
		}
	}

	CableSim::ELengthChangeOrigin ToCoreLengthOrigin(const ECableSimLengthChangeOrigin Origin)
	{
		switch (Origin)
		{
		case ECableSimLengthChangeOrigin::Start:
			return CableSim::ELengthChangeOrigin::Start;
		case ECableSimLengthChangeOrigin::Both:
			return CableSim::ELengthChangeOrigin::Both;
		default:
			return CableSim::ELengthChangeOrigin::End;
		}
	}

	ECableSimSimulationStatus ToRuntimeStatus(const CableSim::ESimulationStatus Status)
	{
		switch (Status)
		{
		case CableSim::ESimulationStatus::Ready:
			return ECableSimSimulationStatus::Ready;
		case CableSim::ESimulationStatus::Overextended:
			return ECableSimSimulationStatus::Overextended;
		case CableSim::ESimulationStatus::MovementLimited:
			return ECableSimSimulationStatus::MovementLimited;
		case CableSim::ESimulationStatus::ParticleBudgetExceeded:
			return ECableSimSimulationStatus::ParticleBudgetExceeded;
		case CableSim::ESimulationStatus::GeometryBudgetExceeded:
			return ECableSimSimulationStatus::CollisionBudgetExceeded;
		case CableSim::ESimulationStatus::InvalidInitialOverlap:
			return ECableSimSimulationStatus::InvalidInitialOverlap;
		case CableSim::ESimulationStatus::CollisionRecoveryFailed:
			return ECableSimSimulationStatus::CollisionRecoveryFailed;
		case CableSim::ESimulationStatus::InvalidConfiguration:
			return ECableSimSimulationStatus::InvalidConfiguration;
		case CableSim::ESimulationStatus::NumericalFailure:
			return ECableSimSimulationStatus::NumericalFailure;
		default:
			return ECableSimSimulationStatus::Uninitialized;
		}
	}

	ECableSimEndpointState ToRuntimeState(const CableSim::EEndpointState State)
	{
		switch (State)
		{
		case CableSim::EEndpointState::Fixed:
			return ECableSimEndpointState::Fixed;
		case CableSim::EEndpointState::Driven:
			return ECableSimEndpointState::Driven;
		default:
			return ECableSimEndpointState::Free;
		}
	}

	double EffectiveMaximumCableLength(const FCableSimSimulationSettings& Simulation)
	{
		const double ParticleBudgetLength = FMath::Max(Simulation.NodeSpacing, 2.0)
			* FMath::Max(FMath::Clamp(Simulation.MaximumParticles, 2, 4096) - 1, 1);
		return FMath::Max(FMath::Min(Simulation.MaximumLength, ParticleBudgetLength), 1.0);
	}

	CableSim::FSimulationConfig BuildCoreConfig(
		const FCableSimSimulationSettings& Simulation,
		const FCableSimFrictionSettings& Friction,
		const FCableSimCollisionSettings& Collision)
	{
		CableSim::FSimulationConfig Config;
		Config.Length = FMath::Clamp(
			Simulation.RestLength,
			1.0,
			EffectiveMaximumCableLength(Simulation));
		Config.SegmentLength = FMath::Max(Simulation.NodeSpacing, 2.0);
		Config.MaximumParticles = FMath::Clamp(Simulation.MaximumParticles, 2, 4096);
		Config.LinearDensity = FMath::Max(Simulation.LinearDensity / 100.0, 1.e-9);
		Config.Gravity = FVector3d(Simulation.Gravity);
		Config.VelocityDamping = FMath::Clamp(Simulation.VelocityDamping, 0.0, 1.0);
		Config.DistanceCompliance = FMath::Max(Simulation.DistanceCompliance, 0.0);
		Config.DistanceRelaxation = FMath::Clamp(Simulation.DistanceRelaxation, 1.0, 1.1);
		Config.BendStrength = FMath::Clamp(Simulation.BendStrength, 0.0, 1.0);
		Config.FreeBendDegreesPerMeter = FMath::Max(Simulation.FreeBendDegreesPerMeter, 0.0);
		Config.DrivenEndpointMaximumSpeed = FMath::Max(Simulation.DrivenEndpointMaximumSpeed, 1.0);
		Config.SolverIterations = FMath::Clamp(Simulation.SolverIterations, 1, 256);
		Config.MultigridIterations = FMath::Clamp(Simulation.MultigridIterations, 0, 8);
		Config.MultigridMinimumParticles = FMath::Clamp(Simulation.MultigridMinimumParticles, 8, 4096);
		Config.StaticFriction = Friction.bEnableFriction
			? FMath::Max(Friction.StaticFrictionCoefficient, 0.0)
			: 0.0;
		Config.DynamicFriction = Friction.bEnableFriction
			? FMath::Max(Friction.DynamicFrictionCoefficient, 0.0)
			: 0.0;
		Config.ConstantFrictionSpeedReduction = Friction.bEnableFriction
			? FMath::Max(Friction.ConstantVelocityReduction, 0.0)
			: 0.0;
		Config.StaticFrictionDeadZone = Friction.bEnableFriction ? FMath::Max(Friction.StaticDeadZone, 0.0) : 0.0;
		Config.FrictionFadeStartSpeed = FMath::Max(Friction.FadeStartSpeed, 0.0);
		Config.FrictionFadeEndSpeed = FMath::Max(Friction.FadeEndSpeed, Config.FrictionFadeStartSpeed + 1.0);
		Config.MaximumContactCorrection = FMath::Max(Collision.MaximumSafeCorrection, 0.1);
		return Config;
	}
}

struct FCableSimRuntimeState
{
	CableSim::FSolver Solver;
	CableSim::FSimulationConfig AppliedConfig;
	FCableSimChaosObjectTracker ObjectTracker;
	FCableSimCollisionSnapshot CollisionSnapshot;
	FCableSimStatus Status;
	FCableSimEndpointResult EndpointResults[2];
	FVector3d LastFrameTargets[2] = {FVector3d::ZeroVector, FVector3d::ZeroVector};
	FVector3d CurrentFrameTargets[2] = {FVector3d::ZeroVector, FVector3d::ZeroVector};
	FVector3d ManualVelocities[2] = {FVector3d::ZeroVector, FVector3d::ZeroVector};
	bool bHasManualVelocity[2] = {false, false};
	FVector3d LastValidEndpointTargets[2] = {FVector3d::ZeroVector, FVector3d::ZeroVector};
	bool bEndpointBindingValid[2] = {true, true};
	double TargetLength = 400.0;
	double LastObservedRestLength = 400.0;
	ECableSimLengthChangeOrigin LengthOrigin = ECableSimLengthChangeOrigin::End;
	bool bLengthClamped = false;
	TArray<FVector3d> PreviousPositions;
	TArray<FVector3d> CurrentPositions;
	TArray<CableSim::FContactConstraint> DebugContacts;
	TArray<FVector3d> DebugRejectedPoints;
	double AccumulatedTime = 0.0;
	double RenderAlpha = 1.0;
	bool bLegacyMigrated = false;
	bool bInitialized = false;
};

UCableSimComponent::UCableSimComponent()
	: RuntimeState(new FCableSimRuntimeState())
{
	bAutoActivate = true;
	bTickInEditor = true;
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
	PrimaryComponentTick.TickGroup = TG_PostPhysics;
	SetCollisionEnabled(ECollisionEnabled::NoCollision);
	StartEndpoint.SpawnLocalPosition = FVector(-150.0, 0.0, 0.0);
	StartEndpoint.LocalTarget = StartEndpoint.SpawnLocalPosition;
	EndEndpoint.SpawnLocalPosition = FVector(150.0, 0.0, 0.0);
	EndEndpoint.LocalTarget = EndEndpoint.SpawnLocalPosition;
}

void UCableSimComponent::PostLoad()
{
	Super::PostLoad();
	const int32 ObjectVersion = GetLinkerCustomVersion(CableSimObjectVersionGuid);
	if (ObjectVersion < static_cast<int32>(ECableSimObjectVersion::LinearDensity)
		|| MaterialSettingsVersion < 1)
	{
		SimulationSettings.LinearDensity = FMath::Max(
			SimulationSettings.ParticleMass * 100.0 / FMath::Max(SimulationSettings.NodeSpacing, 1.0),
			1.e-6);
		MaterialSettingsVersion = 1;
	}
}

void UCableSimComponent::Serialize(FArchive& Ar)
{
	Ar.UsingCustomVersion(CableSimObjectVersionGuid);
	Super::Serialize(Ar);
}

UCableSimComponent::~UCableSimComponent()
{
	delete RuntimeState;
	RuntimeState = nullptr;
}

void UCableSimComponent::OnRegister()
{
	Super::OnRegister();
	MigrateLegacyBindings();
	if (const UWorld* World = GetWorld(); World && !World->IsGameWorld())
	{
		RefreshEditorPreview();
		RefreshVisualization();
	}
}

void UCableSimComponent::BeginPlay()
{
	Super::BeginPlay();
	ReinitializeSimulation();
}

void UCableSimComponent::TickComponent(
	const float DeltaTime,
	const ELevelTick TickType,
	FActorComponentTickFunction* TickFunction)
{
	Super::TickComponent(DeltaTime, TickType, TickFunction);
	UWorld* World = GetWorld();
	if (!World || !World->IsGameWorld())
	{
		RefreshEditorPreview();
		RefreshVisualization();
		return;
	}
	if (!RuntimeState->bInitialized)
	{
		ReinitializeSimulation();
	}
	if (!FMath::IsNearlyEqual(SimulationSettings.RestLength, RuntimeState->LastObservedRestLength))
	{
		SetTargetCableLength(SimulationSettings.RestLength, SimulationSettings.LengthChangeOrigin);
		RuntimeState->LastObservedRestLength = SimulationSettings.RestLength;
	}
	CableSim::FSimulationConfig CurrentConfig = BuildCoreConfig(SimulationSettings, FrictionSettings, CollisionSettings);
	CurrentConfig.Length = RuntimeState->Solver.GetActiveLength();
	if (!CurrentConfig.Equals(RuntimeState->AppliedConfig))
	{
		if (RuntimeState->Solver.ApplyConfig(CurrentConfig))
		{
			RuntimeState->AppliedConfig = RuntimeState->Solver.GetConfig();
			RuntimeState->CollisionSnapshot.Reset();
			if (RuntimeState->CurrentPositions.Num() != RuntimeState->Solver.GetParticles().Num())
			{
				RuntimeState->PreviousPositions.Reset();
				RuntimeState->CurrentPositions.Reset();
				for (const CableSim::FParticle& Particle : RuntimeState->Solver.GetParticles())
				{
					RuntimeState->PreviousPositions.Add(Particle.Position);
					RuntimeState->CurrentPositions.Add(Particle.Position);
				}
			}
		}
		else
		{
			RuntimeState->Status.Status = ECableSimSimulationStatus::InvalidConfiguration;
		}
	}

	ResolveEndpointTarget(StartEndpoint, 0, RuntimeState->CurrentFrameTargets[0]);
	ResolveEndpointTarget(EndEndpoint, 1, RuntimeState->CurrentFrameTargets[1]);

	if (SimulationSettings.bSimulationEnabled)
	{
		const double FixedStep = FMath::Max(SimulationSettings.FixedTimeStep, 0.001);
		const int32 MaximumSteps = FMath::Clamp(SimulationSettings.MaximumSubsteps, 1, 8);
		RuntimeState->AccumulatedTime += FMath::Min(
			FMath::Max(static_cast<double>(DeltaTime), 0.0),
			FixedStep * MaximumSteps);
		const int32 StepCount = FMath::Min(
			FMath::FloorToInt(RuntimeState->AccumulatedTime / FixedStep),
			MaximumSteps);
		for (int32 Step = 0; Step < StepCount; ++Step)
		{
			PerformFixedStep(static_cast<double>(Step + 1) / static_cast<double>(StepCount));
			RuntimeState->AccumulatedTime -= FixedStep;
		}
		if (StepCount > 0)
		{
			RuntimeState->LastFrameTargets[0] = RuntimeState->CurrentFrameTargets[0];
			RuntimeState->LastFrameTargets[1] = RuntimeState->CurrentFrameTargets[1];
		}
		RuntimeState->RenderAlpha = FMath::Clamp(RuntimeState->AccumulatedTime / FixedStep, 0.0, 1.0);
	}
	RefreshVisualization();
}

void UCableSimComponent::ReinitializeSimulation()
{
	MigrateLegacyBindings();
	RuntimeState->AccumulatedTime = 0.0;
	RuntimeState->RenderAlpha = 1.0;
	RuntimeState->ObjectTracker.Reset();
	RuntimeState->CollisionSnapshot.Reset();
	RuntimeState->Status = FCableSimStatus{};
	RuntimeState->DebugContacts.Reset();
	RuntimeState->bEndpointBindingValid[0] = true;
	RuntimeState->bEndpointBindingValid[1] = true;

	auto InitialPosition = [this](const FCableSimEndpointBinding& Endpoint, const int32 EndpointIndex)
	{
		if (Endpoint.State == ECableSimEndpointState::Free)
		{
			return FVector3d(GetComponentTransform().TransformPosition(Endpoint.SpawnLocalPosition));
		}
		switch (Endpoint.TargetSpace)
		{
		case ECableSimTargetSpace::World:
			return FVector3d(Endpoint.WorldTarget);
		case ECableSimTargetSpace::Component:
			if (IsValid(Endpoint.TargetComponent))
			{
				const FTransform Transform = Endpoint.SocketName.IsNone()
					? Endpoint.TargetComponent->GetComponentTransform()
					: Endpoint.TargetComponent->GetSocketTransform(Endpoint.SocketName);
				RuntimeState->bEndpointBindingValid[EndpointIndex] = true;
				return FVector3d(Transform.TransformPosition(Endpoint.ComponentLocalOffset));
			}
			RuntimeState->bEndpointBindingValid[EndpointIndex] = false;
			return FVector3d(GetComponentTransform().TransformPosition(Endpoint.SpawnLocalPosition));
		default:
			return FVector3d(GetComponentTransform().TransformPosition(Endpoint.LocalTarget));
		}
	};

	const FVector3d Start = InitialPosition(StartEndpoint, 0);
	const FVector3d End = InitialPosition(EndEndpoint, 1);
	RuntimeState->AppliedConfig = BuildCoreConfig(SimulationSettings, FrictionSettings, CollisionSettings);
	RuntimeState->bInitialized = RuntimeState->Solver.Initialize(Start, End, RuntimeState->AppliedConfig);
	RuntimeState->TargetLength = RuntimeState->AppliedConfig.Length;
	RuntimeState->LastObservedRestLength = SimulationSettings.RestLength;
	RuntimeState->LengthOrigin = SimulationSettings.LengthChangeOrigin;
	RuntimeState->bLengthClamped = !FMath::IsNearlyEqual(RuntimeState->TargetLength, SimulationSettings.RestLength);
	RuntimeState->LastValidEndpointTargets[0] = Start;
	RuntimeState->LastValidEndpointTargets[1] = End;
	RuntimeState->LastFrameTargets[0] = Start;
	RuntimeState->LastFrameTargets[1] = End;
	RuntimeState->CurrentFrameTargets[0] = Start;
	RuntimeState->CurrentFrameTargets[1] = End;
	RuntimeState->PreviousPositions.Reset();
	RuntimeState->CurrentPositions.Reset();
	for (const CableSim::FParticle& Particle : RuntimeState->Solver.GetParticles())
	{
		RuntimeState->PreviousPositions.Add(Particle.Position);
		RuntimeState->CurrentPositions.Add(Particle.Position);
	}
	RuntimeState->Status.Status = RuntimeState->bInitialized
		? ECableSimSimulationStatus::Ready
		: ECableSimSimulationStatus::InvalidConfiguration;
	RuntimeState->Status.ParticleCount = RuntimeState->Solver.GetParticles().Num();
	RuntimeState->Status.ActiveLength = RuntimeState->Solver.GetActiveLength();
	RuntimeState->Status.TargetLength = RuntimeState->TargetLength;
	RuntimeState->Status.MaximumLength = EffectiveMaximumCableLength(SimulationSettings);
	RuntimeState->Status.LengthChangeOrigin = RuntimeState->LengthOrigin;
	RuntimeState->Status.bLengthClamped = RuntimeState->bLengthClamped;
	RuntimeState->Status.bBindingFailure = !RuntimeState->bEndpointBindingValid[0]
		|| !RuntimeState->bEndpointBindingValid[1];
	RefreshVisualization();
}

void UCableSimComponent::StepSimulation(const int32 StepCount)
{
	if (!RuntimeState->bInitialized)
	{
		ReinitializeSimulation();
	}
	for (int32 Step = 0; Step < FMath::Clamp(StepCount, 0, 1000); ++Step)
	{
		PerformFixedStep(1.0);
	}
	RuntimeState->RenderAlpha = 1.0;
	RefreshVisualization();
}

void UCableSimComponent::SetEndpointState(
	const ECableSimEndpoint Endpoint,
	const ECableSimEndpointState State)
{
	Binding(Endpoint).State = State;
	const int32 Index = Endpoint == ECableSimEndpoint::Start ? 0 : 1;
	RuntimeState->bHasManualVelocity[Index] = false;
	RuntimeState->CollisionSnapshot.Reset();
}

void UCableSimComponent::SetDrivenEndpointTarget(
	const ECableSimEndpoint Endpoint,
	const FVector WorldPosition,
	const FVector WorldVelocity)
{
	FCableSimEndpointBinding& EndpointBinding = Binding(Endpoint);
	EndpointBinding.State = ECableSimEndpointState::Driven;
	EndpointBinding.TargetSpace = ECableSimTargetSpace::World;
	EndpointBinding.WorldTarget = WorldPosition;
	const int32 Index = Endpoint == ECableSimEndpoint::Start ? 0 : 1;
	RuntimeState->LastValidEndpointTargets[Index] = FVector3d(WorldPosition);
	RuntimeState->bEndpointBindingValid[Index] = true;
	RuntimeState->ManualVelocities[Index] = FVector3d(WorldVelocity);
	RuntimeState->bHasManualVelocity[Index] = true;
}

void UCableSimComponent::AttachEndpointToComponent(
	const ECableSimEndpoint Endpoint,
	USceneComponent* Component,
	const FName SocketName,
	const FVector LocalOffset)
{
	FCableSimEndpointBinding& EndpointBinding = Binding(Endpoint);
	EndpointBinding.State = ECableSimEndpointState::Fixed;
	EndpointBinding.TargetSpace = ECableSimTargetSpace::Component;
	EndpointBinding.TargetComponent = Component;
	EndpointBinding.SocketName = SocketName;
	EndpointBinding.ComponentLocalOffset = LocalOffset;
	const int32 Index = Endpoint == ECableSimEndpoint::Start ? 0 : 1;
	RuntimeState->bHasManualVelocity[Index] = false;
	RuntimeState->CollisionSnapshot.Reset();
}

void UCableSimComponent::DriveEndpointFromComponent(
	const ECableSimEndpoint Endpoint,
	USceneComponent* Component,
	const FName SocketName,
	const FVector LocalOffset)
{
	FCableSimEndpointBinding& EndpointBinding = Binding(Endpoint);
	EndpointBinding.State = ECableSimEndpointState::Driven;
	EndpointBinding.TargetSpace = ECableSimTargetSpace::Component;
	EndpointBinding.TargetComponent = Component;
	EndpointBinding.SocketName = SocketName;
	EndpointBinding.ComponentLocalOffset = LocalOffset;
	const int32 Index = Endpoint == ECableSimEndpoint::Start ? 0 : 1;
	RuntimeState->bHasManualVelocity[Index] = false;
	RuntimeState->CollisionSnapshot.Reset();
}

void UCableSimComponent::ReleaseEndpoint(
	const ECableSimEndpoint Endpoint,
	const bool bPreserveVelocity)
{
	Binding(Endpoint).State = ECableSimEndpointState::Free;
	const int32 Index = Endpoint == ECableSimEndpoint::Start ? 0 : 1;
	RuntimeState->bHasManualVelocity[Index] = false;
	if (!bPreserveVelocity && RuntimeState->Solver.IsInitialized())
	{
		CableSim::FStateSnapshot Snapshot = RuntimeState->Solver.CaptureState();
		Snapshot.Particles[Endpoint == ECableSimEndpoint::Start ? 0 : Snapshot.Particles.Num() - 1].Velocity
			= FVector3d::ZeroVector;
		RuntimeState->Solver.RestoreState(Snapshot);
	}
	RuntimeState->CollisionSnapshot.Reset();
}

FCableSimEndpointResult UCableSimComponent::GetEndpointResult(const ECableSimEndpoint Endpoint) const
{
	return RuntimeState->EndpointResults[Endpoint == ECableSimEndpoint::Start ? 0 : 1];
}

TArray<FVector> UCableSimComponent::GetCablePolyline() const
{
	TArray<FVector> Result;
	const int32 Count = RuntimeState->CurrentPositions.Num();
	Result.Reserve(Count);
	for (int32 Index = 0; Index < Count; ++Index)
	{
		const FVector3d Previous = RuntimeState->PreviousPositions.IsValidIndex(Index)
			? RuntimeState->PreviousPositions[Index]
			: RuntimeState->CurrentPositions[Index];
		Result.Add(FVector(FMath::Lerp(Previous, RuntimeState->CurrentPositions[Index], RuntimeState->RenderAlpha)));
	}
	return Result;
}

FCableSimStatus UCableSimComponent::GetSimulationStatus() const
{
	return RuntimeState->Status;
}

double UCableSimComponent::SetTargetCableLength(
	const double NewLength,
	const ECableSimLengthChangeOrigin Origin)
{
	const double Maximum = EffectiveMaximumCableLength(SimulationSettings);
	RuntimeState->TargetLength = FMath::Clamp(NewLength, 1.0, Maximum);
	RuntimeState->LengthOrigin = Origin;
	RuntimeState->bLengthClamped = !FMath::IsNearlyEqual(NewLength, RuntimeState->TargetLength);
	SimulationSettings.RestLength = RuntimeState->TargetLength;
	RuntimeState->LastObservedRestLength = RuntimeState->TargetLength;
	return RuntimeState->TargetLength;
}

void UCableSimComponent::StopLengthChange()
{
	RuntimeState->TargetLength = GetActiveCableLength();
	SimulationSettings.RestLength = RuntimeState->TargetLength;
	RuntimeState->LastObservedRestLength = RuntimeState->TargetLength;
}

double UCableSimComponent::GetActiveCableLength() const
{
	return RuntimeState && RuntimeState->Solver.IsInitialized()
		? RuntimeState->Solver.GetActiveLength()
		: 0.0;
}

double UCableSimComponent::GetTargetCableLength() const
{
	return RuntimeState ? RuntimeState->TargetLength : 0.0;
}

bool UCableSimComponent::IsLengthChanging() const
{
	return RuntimeState && RuntimeState->Solver.IsInitialized()
		&& !FMath::IsNearlyEqual(RuntimeState->TargetLength, RuntimeState->Solver.GetActiveLength(), 1.e-4);
}

bool UCableSimComponent::ResolveEndpointTarget(
	const FCableSimEndpointBinding& Endpoint,
	const int32 EndpointIndex,
	FVector3d& OutTarget)
{
	if (Endpoint.State == ECableSimEndpointState::Free && RuntimeState->Solver.IsInitialized())
	{
		const TArray<CableSim::FParticle>& Particles = RuntimeState->Solver.GetParticles();
		OutTarget = EndpointIndex == 0 ? Particles[0].Position : Particles.Last().Position;
		RuntimeState->LastValidEndpointTargets[EndpointIndex] = OutTarget;
		RuntimeState->bEndpointBindingValid[EndpointIndex] = true;
		return true;
	}
	switch (Endpoint.TargetSpace)
	{
	case ECableSimTargetSpace::World:
		OutTarget = FVector3d(Endpoint.WorldTarget);
		break;
	case ECableSimTargetSpace::Component:
		if (!IsValid(Endpoint.TargetComponent))
		{
			OutTarget = RuntimeState->LastValidEndpointTargets[EndpointIndex];
			RuntimeState->bEndpointBindingValid[EndpointIndex] = false;
			return false;
		}
		{
			const FTransform Transform = Endpoint.SocketName.IsNone()
				? Endpoint.TargetComponent->GetComponentTransform()
				: Endpoint.TargetComponent->GetSocketTransform(Endpoint.SocketName);
			OutTarget = FVector3d(Transform.TransformPosition(Endpoint.ComponentLocalOffset));
		}
		break;
	default:
		OutTarget = FVector3d(GetComponentTransform().TransformPosition(Endpoint.LocalTarget));
		break;
	}
	RuntimeState->LastValidEndpointTargets[EndpointIndex] = OutTarget;
	RuntimeState->bEndpointBindingValid[EndpointIndex] = true;
	return true;
}

FString UCableSimComponent::WriteDebugFrameDump() const
{
	if (!RuntimeState || !RuntimeState->Solver.IsInitialized())
	{
		return FString();
	}

	const FString Directory = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("CableSimDumps"));
	IFileManager::Get().MakeDirectory(*Directory, true);
	const FString Path = FPaths::Combine(
		Directory,
		FString::Printf(TEXT("CableFrame_%lld.txt"), RuntimeState->Status.StepIndex));

	FString Text;
	Text.Reserve(65536);
	Text += FString::Printf(
		TEXT("step=%lld status=%d particles=%d contacts=%d active_length=%.9g target_length=%.9g maximum_length=%.9g length_origin=%d length_changing=%d length_clamped=%d binding_failure=%d strain=%.9g penetration=%.9g correction=%.9g tension=%.9g normal_load=%.9g accepted=%.9g degraded=%d\n"),
		RuntimeState->Status.StepIndex,
		static_cast<int32>(RuntimeState->Status.Status),
		RuntimeState->Status.ParticleCount,
		RuntimeState->Status.ContactCount,
		RuntimeState->Status.ActiveLength,
		RuntimeState->Status.TargetLength,
		RuntimeState->Status.MaximumLength,
		static_cast<int32>(RuntimeState->Status.LengthChangeOrigin),
		RuntimeState->Status.bLengthChanging ? 1 : 0,
		RuntimeState->Status.bLengthClamped ? 1 : 0,
		RuntimeState->Status.bBindingFailure ? 1 : 0,
		RuntimeState->Status.MaximumSegmentStrain,
		RuntimeState->Status.MaximumPenetration,
		RuntimeState->Status.MaximumContactCorrection,
		RuntimeState->Status.MaximumEstimatedTension,
		RuntimeState->Status.MaximumEstimatedNormalLoad,
		RuntimeState->Status.AcceptedMovementFraction,
		RuntimeState->Status.bCollisionDegraded ? 1 : 0);

	const TArray<CableSim::FParticle>& Particles = RuntimeState->Solver.GetParticles();
	Text += TEXT("\n[particles]\n");
	for (int32 Index = 0; Index < Particles.Num(); ++Index)
	{
		const CableSim::FParticle& Particle = Particles[Index];
		Text += FString::Printf(
			TEXT("%d p=(%.9g %.9g %.9g) prev=(%.9g %.9g %.9g) v=(%.9g %.9g %.9g) mass=%.9g inv_mass=%.9g material=%.9g tension=%.9g normal_load=%.9g\n"),
			Index,
			Particle.Position.X, Particle.Position.Y, Particle.Position.Z,
			Particle.PreviousPosition.X, Particle.PreviousPosition.Y, Particle.PreviousPosition.Z,
			Particle.Velocity.X, Particle.Velocity.Y, Particle.Velocity.Z,
			Particle.Mass, Particle.InverseMass, Particle.MaterialCoordinate,
			Particle.EstimatedTension, Particle.EstimatedNormalLoad);
	}

	Text += TEXT("\n[contacts]\n");
	for (int32 Index = 0; Index < RuntimeState->DebugContacts.Num(); ++Index)
	{
		const CableSim::FContactConstraint& Contact = RuntimeState->DebugContacts[Index];
		Text += FString::Printf(
			TEXT("%d node=(%d %d %.9g) feature=(%llu %d %d %d %d) n=(%.9g %.9g %.9g) min=%.9g surface_v=(%.9g %.9g %.9g) normal_correction=%.9g particle_correction=(%.9g %.9g) friction_correction=%.9g largest_projection=%.9g\n"),
			Index, Contact.ParticleA, Contact.ParticleB, Contact.SegmentAlpha,
			Contact.FeatureId.ObjectToken, Contact.FeatureId.ShapeIndex,
			static_cast<int32>(Contact.FeatureId.Type), Contact.FeatureId.Index0, Contact.FeatureId.Index1,
			Contact.Normal.X, Contact.Normal.Y, Contact.Normal.Z, Contact.MinimumNormalCoordinate,
			Contact.SurfaceVelocity.X, Contact.SurfaceVelocity.Y, Contact.SurfaceVelocity.Z,
			Contact.AccumulatedNormalCorrection,
			Contact.AccumulatedParticleCorrectionA,
			Contact.AccumulatedParticleCorrectionB,
			Contact.AccumulatedFrictionCorrection,
			Contact.LargestProjection);
	}

	Text += TEXT("\n[collision_nodes]\n");
	for (int32 NodeIndex = 0; NodeIndex < RuntimeState->CollisionSnapshot.Nodes.Num(); ++NodeIndex)
	{
		const FCableSimNodeCollisionGeometry& Node = RuntimeState->CollisionSnapshot.Nodes[NodeIndex];
		Text += FString::Printf(
			TEXT("node=%d bounds_min=(%.9g %.9g %.9g) bounds_max=(%.9g %.9g %.9g) triangles=%d edges=%d spheres=%d capsules=%d\n"),
			NodeIndex,
			Node.PredictedBounds.Min.X, Node.PredictedBounds.Min.Y, Node.PredictedBounds.Min.Z,
			Node.PredictedBounds.Max.X, Node.PredictedBounds.Max.Y, Node.PredictedBounds.Max.Z,
			Node.Triangles.Num(), Node.Edges.Num(), Node.Spheres.Num(), Node.Capsules.Num());
		for (const CableSim::FCollisionTriangle& Triangle : Node.Triangles)
		{
			Text += FString::Printf(
				TEXT("  tri feature=(%llu %d %d) a=(%.9g %.9g %.9g) b=(%.9g %.9g %.9g) c=(%.9g %.9g %.9g)\n"),
				Triangle.Id.ObjectToken, Triangle.Id.ShapeIndex, Triangle.Id.Index0,
				Triangle.Vertices[0].X, Triangle.Vertices[0].Y, Triangle.Vertices[0].Z,
				Triangle.Vertices[1].X, Triangle.Vertices[1].Y, Triangle.Vertices[1].Z,
				Triangle.Vertices[2].X, Triangle.Vertices[2].Y, Triangle.Vertices[2].Z);
		}
		for (const CableSim::FCollisionEdge& Edge : Node.Edges)
		{
			Text += FString::Printf(
				TEXT("  edge feature=(%llu %d %d %d) kind=%d a=(%.9g %.9g %.9g) b=(%.9g %.9g %.9g)\n"),
				Edge.Id.ObjectToken, Edge.Id.ShapeIndex, Edge.Id.Index0, Edge.Id.Index1,
				static_cast<int32>(Edge.Kind),
				Edge.Start.X, Edge.Start.Y, Edge.Start.Z, Edge.End.X, Edge.End.Y, Edge.End.Z);
		}
	}

	return FFileHelper::SaveStringToFile(Text, *Path) ? Path : FString();
}

FCableSimEndpointBinding& UCableSimComponent::Binding(const ECableSimEndpoint Endpoint)
{
	return Endpoint == ECableSimEndpoint::Start ? StartEndpoint : EndEndpoint;
}

const FCableSimEndpointBinding& UCableSimComponent::Binding(const ECableSimEndpoint Endpoint) const
{
	return Endpoint == ECableSimEndpoint::Start ? StartEndpoint : EndEndpoint;
}

void UCableSimComponent::MigrateLegacyBindings()
{
	if (RuntimeState->bLegacyMigrated)
	{
		return;
	}
	for (FCableSimEndpointBinding* Endpoint : {&StartEndpoint, &EndEndpoint})
	{
		if (Endpoint->Mode == ECableSimEndpointMode::Simulated)
		{
			Endpoint->State = ECableSimEndpointState::Free;
		}
		if (Endpoint->Mode == ECableSimEndpointMode::WorldKinematic)
		{
			Endpoint->TargetSpace = ECableSimTargetSpace::World;
		}
		else if (Endpoint->Mode == ECableSimEndpointMode::ComponentKinematic)
		{
			Endpoint->TargetSpace = ECableSimTargetSpace::Component;
		}
	}
	RuntimeState->bLegacyMigrated = true;
}

void UCableSimComponent::PerformFixedStep(const double FrameAlpha)
{
	if (!RuntimeState->Solver.IsInitialized())
	{
		return;
	}
	const double OuterDeltaTime = FMath::Max(SimulationSettings.FixedTimeStep, 0.001);
	const double ActiveLength = RuntimeState->Solver.GetActiveLength();
	const double LengthDelta = RuntimeState->TargetLength - ActiveLength;
	if (FMath::Abs(LengthDelta) > 1.e-4)
	{
		const double Rate = LengthDelta > 0.0
			? FMath::Max(SimulationSettings.PayoutSpeed, 0.0)
			: FMath::Max(SimulationSettings.ReelSpeed, 0.0);
		const double AppliedDelta = FMath::Clamp(LengthDelta, -Rate * OuterDeltaTime, Rate * OuterDeltaTime);
		if (FMath::Abs(AppliedDelta) > 1.e-6)
		{
			const bool bChanged = RuntimeState->Solver.SetActiveLength(
				ActiveLength + AppliedDelta,
				ToCoreLengthOrigin(RuntimeState->LengthOrigin));
			RuntimeState->bLengthClamped |= !bChanged;
			RuntimeState->AppliedConfig = RuntimeState->Solver.GetConfig();
		}
	}
	FVector3d RequestedTargets[2] = {
		FMath::Lerp(RuntimeState->LastFrameTargets[0], RuntimeState->CurrentFrameTargets[0], FrameAlpha),
		FMath::Lerp(RuntimeState->LastFrameTargets[1], RuntimeState->CurrentFrameTargets[1], FrameAlpha)};
	const TArray<CableSim::FParticle>& InitialParticles = RuntimeState->Solver.GetParticles();
	FVector3d CollisionTargets[2] = {RequestedTargets[0], RequestedTargets[1]};
	const double MaximumDrivenTravel = FMath::Max(SimulationSettings.DrivenEndpointMaximumSpeed, 1.0)
		* OuterDeltaTime;
	for (int32 EndpointIndex = 0; EndpointIndex < 2; ++EndpointIndex)
	{
		const FCableSimEndpointBinding& Endpoint = EndpointIndex == 0 ? StartEndpoint : EndEndpoint;
		if (Endpoint.State != ECableSimEndpointState::Driven) continue;
		const FVector3d Current = EndpointIndex == 0 ? InitialParticles[0].Position : InitialParticles.Last().Position;
		const FVector3d Delta = RequestedTargets[EndpointIndex] - Current;
		const double Distance = Delta.Length();
		if (Distance > MaximumDrivenTravel && Distance > UE_DOUBLE_SMALL_NUMBER)
			CollisionTargets[EndpointIndex] = Current + Delta * (MaximumDrivenTravel / Distance);
	}
	TArray<AActor*> IgnoredActors;
	if (IsValid(GetOwner()))
	{
		IgnoredActors.Add(GetOwner());
	}
	if (IsValid(StartEndpoint.TargetComponent))
	{
		IgnoredActors.AddUnique(StartEndpoint.TargetComponent->GetOwner());
	}
	if (IsValid(EndEndpoint.TargetComponent))
	{
		IgnoredActors.AddUnique(EndEndpoint.TargetComponent->GetOwner());
	}
	const double StartTime = FPlatformTime::Seconds();
	const bool bGeometryReady = FCableSimWorldCollisionProvider::GatherSnapshot(
		GetWorld(),
		GetOwner(),
		IgnoredActors,
		CollisionSettings,
		InitialParticles,
		CollisionTargets[0],
		CollisionTargets[1],
		OuterDeltaTime,
		RuntimeState->ObjectTracker,
		RuntimeState->CollisionSnapshot);

	FCableSimCollisionDiagnostics CollisionDiagnostics = RuntimeState->CollisionSnapshot.Diagnostics;
	TArray<CableSim::FContactConstraint> Contacts;
	TArray<FVector3d> RejectedPoints;
	CableSim::FStepResult CoreResult = RuntimeState->Solver.GetLastResult();
	CableSim::FStepInput Input;
	Input.DeltaTime = OuterDeltaTime;
	Input.StartEndpoint.State = ToCoreState(StartEndpoint.State);
	Input.EndEndpoint.State = ToCoreState(EndEndpoint.State);
	Input.StartEndpoint.TargetPosition = RequestedTargets[0];
	Input.EndEndpoint.TargetPosition = RequestedTargets[1];
	for (int32 EndpointIndex = 0; EndpointIndex < 2; ++EndpointIndex)
	{
		CableSim::FEndpointInput& EndpointInput = EndpointIndex == 0 ? Input.StartEndpoint : Input.EndEndpoint;
		const FCableSimEndpointBinding& EndpointBinding = EndpointIndex == 0 ? StartEndpoint : EndEndpoint;
		if (RuntimeState->bHasManualVelocity[EndpointIndex])
		{
			EndpointInput.TargetVelocity = RuntimeState->ManualVelocities[EndpointIndex];
		}
		else if (EndpointBinding.TargetSpace == ECableSimTargetSpace::Component
			&& IsValid(EndpointBinding.TargetComponent))
		{
			EndpointInput.TargetVelocity = FVector3d(EndpointBinding.TargetComponent->GetComponentVelocity());
		}
	}
	if (RuntimeState->Solver.BeginStep(Input))
	{
		FCableSimWorldCollisionProvider::CompileContacts(
			RuntimeState->CollisionSnapshot,
			CollisionSettings,
			FrictionSettings,
			RuntimeState->Solver,
			Input,
			Contacts,
			RejectedPoints,
			CollisionDiagnostics);
		RuntimeState->Solver.SolveBatch(Input, Contacts, RuntimeState->AppliedConfig.SolverIterations);
		TArray<CableSim::FContactConstraint> AdditionalContacts;
		TArray<FVector3d> AdditionalRejected;
		FCableSimWorldCollisionProvider::CompileContacts(
			RuntimeState->CollisionSnapshot,
			CollisionSettings,
			FrictionSettings,
			RuntimeState->Solver,
			Input,
			AdditionalContacts,
			AdditionalRejected,
			CollisionDiagnostics);
		for (CableSim::FContactConstraint& Additional : AdditionalContacts)
		{
			const bool bExists = Contacts.ContainsByPredicate([&](const CableSim::FContactConstraint& Existing)
			{
				return Existing.ParticleA == Additional.ParticleA
					&& Existing.ParticleB == Additional.ParticleB
					&& Existing.FeatureId == Additional.FeatureId;
			});
			if (!bExists) Contacts.Add(MoveTemp(Additional));
		}
		RejectedPoints.Append(AdditionalRejected);
		RuntimeState->Solver.ReconcileContacts(Input, Contacts, 4);
		CoreResult = RuntimeState->Solver.FinalizeStep(Input, Contacts);
	}

	RuntimeState->PreviousPositions = MoveTemp(RuntimeState->CurrentPositions);
	RuntimeState->CurrentPositions.Reset(RuntimeState->Solver.GetParticles().Num());
	for (const CableSim::FParticle& Particle : RuntimeState->Solver.GetParticles())
	{
		RuntimeState->CurrentPositions.Add(Particle.Position);
	}
	if (RuntimeState->PreviousPositions.Num() != RuntimeState->CurrentPositions.Num())
	{
		RuntimeState->PreviousPositions = RuntimeState->CurrentPositions;
	}
	RuntimeState->DebugContacts = Contacts;
	RuntimeState->DebugRejectedPoints = RejectedPoints;

	RuntimeState->Status = FCableSimStatus{};
	RuntimeState->Status.Status = bGeometryReady
		? ToRuntimeStatus(CoreResult.Status)
		: ECableSimSimulationStatus::CollisionBudgetExceeded;
	RuntimeState->Status.StepIndex = static_cast<int64>(RuntimeState->Solver.GetLastResult().StepIndex);
	RuntimeState->Status.ParticleCount = RuntimeState->Solver.GetParticles().Num();
	RuntimeState->Status.ContactCount = Contacts.Num();
	RuntimeState->Status.CollisionQueryCount = RuntimeState->CollisionSnapshot.Diagnostics.QueryCount;
	RuntimeState->Status.CollisionTriangleCount = RuntimeState->CollisionSnapshot.Diagnostics.TriangleCount;
	RuntimeState->Status.ConvexEdgeCount = RuntimeState->CollisionSnapshot.Diagnostics.ConvexEdgeCount;
	RuntimeState->Status.RejectedContactCount = CollisionDiagnostics.RejectedContactCount;
	RuntimeState->Status.MaximumSegmentError = CoreResult.MaximumSegmentError;
	RuntimeState->Status.MaximumPenetration = CoreResult.MaximumPenetration;
	RuntimeState->Status.MaximumSegmentStrain = CoreResult.MaximumSegmentStrain;
	RuntimeState->Status.MaximumContactCorrection = CoreResult.MaximumContactCorrection;
	RuntimeState->Status.MaximumEstimatedTension = CoreResult.MaximumEstimatedTension;
	RuntimeState->Status.MaximumEstimatedNormalLoad = CoreResult.MaximumEstimatedNormalLoad;
	RuntimeState->Status.AcceptedMovementFraction = 1.0;
	RuntimeState->Status.SimulationMilliseconds = (FPlatformTime::Seconds() - StartTime) * 1000.0;
	RuntimeState->Status.bMotionClamped = false;
	RuntimeState->Status.bCollisionDegraded = CollisionDiagnostics.bFeatureBudgetExceeded
		|| CollisionDiagnostics.UnsupportedShapeCount > 0;
	RuntimeState->Status.ActiveLength = RuntimeState->Solver.GetActiveLength();
	RuntimeState->Status.TargetLength = RuntimeState->TargetLength;
	RuntimeState->Status.MaximumLength = EffectiveMaximumCableLength(SimulationSettings);
	RuntimeState->Status.LengthChangeOrigin = RuntimeState->LengthOrigin;
	RuntimeState->Status.bLengthChanging = IsLengthChanging();
	RuntimeState->Status.bLengthClamped = RuntimeState->bLengthClamped;
	RuntimeState->Status.bBindingFailure = !RuntimeState->bEndpointBindingValid[0]
		|| !RuntimeState->bEndpointBindingValid[1];

	for (int32 EndpointIndex = 0; EndpointIndex < 2; ++EndpointIndex)
	{
		FCableSimEndpointResult& Result = RuntimeState->EndpointResults[EndpointIndex];
		const TArray<CableSim::FParticle>& Particles = RuntimeState->Solver.GetParticles();
		const CableSim::FParticle& Particle = EndpointIndex == 0 ? Particles[0] : Particles.Last();
		Result.State = EndpointIndex == 0 ? StartEndpoint.State : EndEndpoint.State;
		Result.RequestedWorldPosition = FVector(RequestedTargets[EndpointIndex]);
		Result.AcceptedWorldPosition = FVector(Particle.Position);
		Result.Velocity = FVector(Particle.Velocity);
		Result.CorrectionWorld = Result.AcceptedWorldPosition - Result.RequestedWorldPosition;
		Result.LimitError = Result.CorrectionWorld.Length();
		Result.bLimited = Result.State == ECableSimEndpointState::Driven && Result.LimitError > 0.1;
		Result.bBindingValid = RuntimeState->bEndpointBindingValid[EndpointIndex];
		if (Result.State == ECableSimEndpointState::Driven)
		{
			const FVector3d Initial = EndpointIndex == 0 ? InitialParticles[0].PreviousPosition : InitialParticles.Last().PreviousPosition;
			const FVector3d RequestedDelta = RequestedTargets[EndpointIndex] - Initial;
			const double RequestedDistanceSquared = RequestedDelta.SquaredLength();
			if (RequestedDistanceSquared > UE_DOUBLE_SMALL_NUMBER)
			{
				const double Progress = FVector3d::DotProduct(Particle.Position - Initial, RequestedDelta)
					/ RequestedDistanceSquared;
				RuntimeState->Status.AcceptedMovementFraction = FMath::Min(
					RuntimeState->Status.AcceptedMovementFraction,
					FMath::Clamp(Progress, 0.0, 1.0));
			}
			RuntimeState->Status.bMotionClamped |= Result.bLimited;
		}
		if (Result.bLimited && RuntimeState->Status.Status == ECableSimSimulationStatus::Ready)
		{
			RuntimeState->Status.Status = ECableSimSimulationStatus::MovementLimited;
		}
	}
}

void UCableSimComponent::RefreshEditorPreview()
{
	auto PreviewPosition = [this](const FCableSimEndpointBinding& Endpoint)
	{
		if (Endpoint.State == ECableSimEndpointState::Free)
			return FVector3d(GetComponentTransform().TransformPosition(Endpoint.SpawnLocalPosition));
		if (Endpoint.TargetSpace == ECableSimTargetSpace::World) return FVector3d(Endpoint.WorldTarget);
		if (Endpoint.TargetSpace == ECableSimTargetSpace::Component && IsValid(Endpoint.TargetComponent))
		{
			const FTransform Transform = Endpoint.SocketName.IsNone()
				? Endpoint.TargetComponent->GetComponentTransform()
				: Endpoint.TargetComponent->GetSocketTransform(Endpoint.SocketName);
			return FVector3d(Transform.TransformPosition(Endpoint.ComponentLocalOffset));
		}
		return FVector3d(GetComponentTransform().TransformPosition(Endpoint.LocalTarget));
	};
	const FVector3d Start = PreviewPosition(StartEndpoint);
	const FVector3d End = PreviewPosition(EndEndpoint);
	const double Length = FMath::Clamp(
		SimulationSettings.RestLength,
		1.0,
		EffectiveMaximumCableLength(SimulationSettings));
	const int32 Segments = FMath::Clamp(
		FMath::CeilToInt(Length / FMath::Max(SimulationSettings.NodeSpacing, 1.0)),
		1,
		FMath::Max(SimulationSettings.MaximumParticles - 1, 1));
	RuntimeState->PreviousPositions.Reset(Segments + 1);
	RuntimeState->CurrentPositions.Reset(Segments + 1);
	for (int32 Index = 0; Index <= Segments; ++Index)
	{
		const FVector3d Position = FMath::Lerp(Start, End, static_cast<double>(Index) / Segments);
		RuntimeState->PreviousPositions.Add(Position);
		RuntimeState->CurrentPositions.Add(Position);
	}
	RuntimeState->RenderAlpha = 1.0;
}

void UCableSimComponent::RefreshVisualization()
{
#if !UE_BUILD_SHIPPING
	MarkRenderStateDirty();
#endif
}

FBoxSphereBounds UCableSimComponent::CalcBounds(const FTransform& LocalToWorld) const
{
	FBox Box(ForceInit);
	for (const FVector3d& Point : RuntimeState->CurrentPositions)
	{
		Box += FVector(Point);
	}
	if (!Box.IsValid)
	{
		Box = FBox(GetComponentLocation(), GetComponentLocation());
	}
	return FBoxSphereBounds(Box.ExpandBy(FMath::Max(CollisionSettings.Radius, 10.0)));
}

#if WITH_EDITOR
void UCableSimComponent::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);
	RuntimeState->bLegacyMigrated = false;
	MigrateLegacyBindings();
	RefreshEditorPreview();
	RefreshVisualization();
}
#endif

FDebugRenderSceneProxy* UCableSimComponent::CreateDebugSceneProxy()
{
#if UE_BUILD_SHIPPING
	return nullptr;
#else
	if (!DebugSettings.bDraw)
	{
		return nullptr;
	}
	TArray<FVector3d> Polyline;
	const int32 Count = RuntimeState->CurrentPositions.Num();
	Polyline.Reserve(Count);
	for (int32 Index = 0; Index < Count; ++Index)
	{
		const FVector3d Previous = RuntimeState->PreviousPositions.IsValidIndex(Index)
			? RuntimeState->PreviousPositions[Index]
			: RuntimeState->CurrentPositions[Index];
		Polyline.Add(FMath::Lerp(Previous, RuntimeState->CurrentPositions[Index], RuntimeState->RenderAlpha));
	}
	return new FCableSimDebugProxy(
		this,
		Polyline,
		RuntimeState->Solver.GetParticles(),
		StartEndpoint.State,
		EndEndpoint.State,
		RuntimeState->DebugContacts,
		RuntimeState->DebugRejectedPoints,
		RuntimeState->CollisionSnapshot,
		DebugSettings,
		RuntimeState->Status);
#endif
}
