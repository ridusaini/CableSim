#include "CableSimComponent.h"

#include "CableSimContactManifold.h"
#include "CableSimEndpointResolver.h"
#include "Chaos/CableSimChaosCollisionAdapter.h"
#include "DebugRenderSceneProxy.h"
#include "Engine/World.h"
#include "HAL/PlatformTime.h"
#include "SceneManagement.h"

	namespace
	{
	class FCableSimDebugRenderSceneProxy final : public FDebugRenderSceneProxy
	{
	public:
		explicit FCableSimDebugRenderSceneProxy(const UPrimitiveComponent* Component)
			: FDebugRenderSceneProxy(Component)
		{
			DrawType = EDrawType::WireMesh;
			DrawAlpha = 255;
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

	struct FEndpointFrameSample
	{
		FVector3d ConsumedPosition = FVector3d::ZeroVector;
		FVector3d CurrentPosition = FVector3d::ZeroVector;
		FVector3d Velocity = FVector3d::ZeroVector;
	};

	CableSim::EParticleMode ToCoreMode(const ECableSimEndpointMode Mode)
	{
		return Mode == ECableSimEndpointMode::Simulated
			? CableSim::EParticleMode::Dynamic
			: CableSim::EParticleMode::Kinematic;
	}

	ECableSimSimulationStatus ToRuntimeStatus(const CableSim::ESimulationStatus Status)
	{
		switch (Status)
		{
		case CableSim::ESimulationStatus::Ready:
			return ECableSimSimulationStatus::Ready;
		case CableSim::ESimulationStatus::Overextended:
			return ECableSimSimulationStatus::Overextended;
		case CableSim::ESimulationStatus::NumericalFailure:
			return ECableSimSimulationStatus::NumericalFailure;
		case CableSim::ESimulationStatus::InvalidConfiguration:
			return ECableSimSimulationStatus::InvalidConfiguration;
		default:
			return ECableSimSimulationStatus::Uninitialized;
		}
	}

	ECableSimTautStatus ToRuntimeTautStatus(const CableSim::ETautStatus Status)
	{
		switch (Status)
		{
		case CableSim::ETautStatus::Ready:
			return ECableSimTautStatus::Ready;
		case CableSim::ETautStatus::NoRelevantGeometry:
			return ECableSimTautStatus::NoRelevantGeometry;
		case CableSim::ETautStatus::InvalidSeed:
			return ECableSimTautStatus::InvalidSeed;
		case CableSim::ETautStatus::FeatureUnavailable:
			return ECableSimTautStatus::FeatureUnavailable;
		case CableSim::ETautStatus::PathBlocked:
			return ECableSimTautStatus::PathBlocked;
		case CableSim::ETautStatus::NonManifoldTopology:
			return ECableSimTautStatus::NonManifoldTopology;
		case CableSim::ETautStatus::TopologyOverValence:
			return ECableSimTautStatus::TopologyOverValence;
		case CableSim::ETautStatus::CollisionBudgetExceeded:
			return ECableSimTautStatus::CollisionBudgetExceeded;
		case CableSim::ETautStatus::TopologyBudgetExceeded:
			return ECableSimTautStatus::TopologyBudgetExceeded;
		case CableSim::ETautStatus::InvalidConfiguration:
			return ECableSimTautStatus::InvalidConfiguration;
		case CableSim::ETautStatus::NumericalFailure:
			return ECableSimTautStatus::NumericalFailure;
		default:
			return ECableSimTautStatus::Uninitialized;
		}
	}

	FColor GetEdgeDebugColor(const CableSim::ECollisionEdgeKind Kind)
	{
		switch (Kind)
		{
		case CableSim::ECollisionEdgeKind::Convex:
			return FColor::Green;
		case CableSim::ECollisionEdgeKind::Concave:
			return FColor::Red;
		case CableSim::ECollisionEdgeKind::Coplanar:
			return FColor::Blue;
		case CableSim::ECollisionEdgeKind::Boundary:
			return FColor::Yellow;
		case CableSim::ECollisionEdgeKind::NonManifold:
			return FColor(255, 0, 255);
		default:
			return FColor::Silver;
		}
	}

	double CalculatePolylineLength(const TConstArrayView<CableSim::FTautPoint> Points)
	{
		double Length = 0.0;
		for (int32 Index = 0; Index + 1 < Points.Num(); ++Index)
		{
			Length += FVector3d::Distance(Points[Index].Position, Points[Index + 1].Position);
		}
		return Length;
	}

	FVector3d SamplePolyline(
		const TConstArrayView<CableSim::FTautPoint> Points,
		const double NormalizedCoordinate)
	{
		if (Points.IsEmpty())
		{
			return FVector3d::ZeroVector;
		}
		const double Length = CalculatePolylineLength(Points);
		if (Length <= 1.e-9)
		{
			return Points[0].Position;
		}
		const double TargetDistance = FMath::Clamp(NormalizedCoordinate, 0.0, 1.0) * Length;
		double AccumulatedDistance = 0.0;
		for (int32 Index = 0; Index + 1 < Points.Num(); ++Index)
		{
			const double SegmentLength = FVector3d::Distance(Points[Index].Position, Points[Index + 1].Position);
			if (AccumulatedDistance + SegmentLength >= TargetDistance && SegmentLength > 1.e-9)
			{
				return FMath::Lerp(
					Points[Index].Position,
					Points[Index + 1].Position,
					(TargetDistance - AccumulatedDistance) / SegmentLength);
			}
			AccumulatedDistance += SegmentLength;
		}
		return Points.Last().Position;
	}
}

struct FCableSimRuntimeState
{
	FCableSimEndpointResolver EndpointResolver;
	FEndpointFrameSample EndpointSamples[2];
	ECableSimEndpointBindingStatus EndpointStatuses[2] = {
		ECableSimEndpointBindingStatus::Ready,
		ECableSimEndpointBindingStatus::Ready};
	ECableSimEndpointMode AppliedEndpointModes[2] = {
		ECableSimEndpointMode::CableLocalKinematic,
		ECableSimEndpointMode::CableLocalKinematic};
	CableSim::FSimulationConfig AppliedConfig;
	CableSim::FSimulationConfig PendingConfig;
	CableSim::FTautPathSolver TautSolver;
	CableSim::FTautConfig AppliedTautConfig;
	FCableSimChaosObjectTracker ChaosObjectTracker;
	FCableSimChaosSnapshot ChaosSnapshot;
	bool bSnapshotValid = false;
	TArray<CableSim::FCollisionTriangle> SupportedTautTriangles;
	TArray<CableSim::FCollisionEdge> SupportedTautEdges;
	TArray<CableSim::FCollisionVertex> SupportedTautVertices;
	FCableSimTautDiagnostics TautDiagnostics;
	TArray<FVector3d> PreviousSolvedPositions;
	TArray<FVector3d> CurrentSolvedPositions;
	double AccumulatedTime = 0.0;
	double EndpointSampleElapsedTime = 0.0;
	double DroppedSimulationTime = 0.0;
	double RenderInterpolationAlpha = 1.0;
	FVector3d EditorPreviewEndpoints[2] = {FVector3d::ZeroVector, FVector3d::ZeroVector};
	bool bHasEditorPreview = false;
	bool bConfigPending = false;
	bool bTautWasActive = false;
	FVector3d LastAcceptedTautEndpoints[2] = {FVector3d::ZeroVector, FVector3d::ZeroVector};
	bool bHasLastAcceptedTautEndpoint[2] = {false, false};
	TArray<CableSim::FGuideConstraint> LastGuides;
};

UCableSimComponent::UCableSimComponent()
	: RuntimeState(new FCableSimRuntimeState())
{
	bAutoActivate = true;
	bTickInEditor = true;
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
	SetCollisionEnabled(ECollisionEnabled::NoCollision);
	StartEndpoint.SpawnLocalPosition = FVector(-150.0, 0.0, 0.0);
	StartEndpoint.LocalTarget = StartEndpoint.SpawnLocalPosition;
	EndEndpoint.SpawnLocalPosition = FVector(150.0, 0.0, 0.0);
	EndEndpoint.LocalTarget = EndEndpoint.SpawnLocalPosition;
}

void UCableSimComponent::OnRegister()
{
	Super::OnRegister();
	if (const UWorld* World = GetWorld(); World && !World->IsGameWorld())
	{
		RefreshEditorPreview();
		RefreshVisualization();
	}
}

UCableSimComponent::~UCableSimComponent()
{
	delete RuntimeState;
	RuntimeState = nullptr;
}

void UCableSimComponent::BeginPlay()
{
	Super::BeginPlay();
	ReinitializeSimulation();
	RefreshVisualization();
}

void UCableSimComponent::TickComponent(
	const float DeltaTime,
	const ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (const UWorld* World = GetWorld(); World && !World->IsGameWorld())
	{
		RefreshEditorPreview();
		RefreshVisualization();
		return;
	}
	SynchronizeConfiguration();
	SynchronizeEndpointModes();

	if (SimulationSettings.bSimulationEnabled && Solver.IsInitialized())
	{
		SampleEndpointTargets(DeltaTime);
		const double SafeFixedStep = FMath::Max(SimulationSettings.FixedTimeStep, 1.e-4);
		const int32 SafeMaximumSubsteps = FMath::Clamp(SimulationSettings.MaximumSubsteps, 1, 16);
		const double RawDeltaTime = FMath::Max(static_cast<double>(DeltaTime), 0.0);
		const double AcceptedDeltaTime = FMath::Min(RawDeltaTime, SafeFixedStep * SafeMaximumSubsteps);
		RuntimeState->DroppedSimulationTime += RawDeltaTime - AcceptedDeltaTime;
		RuntimeState->AccumulatedTime += AcceptedDeltaTime;

		const int32 StepsToPerform = FMath::Min(
			FMath::FloorToInt(RuntimeState->AccumulatedTime / SafeFixedStep),
			SafeMaximumSubsteps);
		for (int32 Step = 0; Step < StepsToPerform; ++Step)
		{
			PerformFixedStep(static_cast<double>(Step + 1) / static_cast<double>(StepsToPerform));
			RuntimeState->AccumulatedTime -= SafeFixedStep;
		}
		if (StepsToPerform > 0)
		{
			CommitEndpointSamples();
		}
		RuntimeState->RenderInterpolationAlpha = FMath::Clamp(
			RuntimeState->AccumulatedTime / SafeFixedStep,
			0.0,
			1.0);
	}
	else
	{
		RuntimeState->RenderInterpolationAlpha = 1.0;
	}

	RefreshVisualization();
}

void UCableSimComponent::ReinitializeSimulation()
{
	RuntimeState->AccumulatedTime = 0.0;
	RuntimeState->EndpointSampleElapsedTime = 0.0;
	RuntimeState->DroppedSimulationTime = 0.0;
	RuntimeState->RenderInterpolationAlpha = 1.0;
	RuntimeState->EndpointResolver.Reset();
	RuntimeState->TautSolver.Reset();
	RuntimeState->ChaosObjectTracker.Reset();
	RuntimeState->ChaosSnapshot.Reset();
	RuntimeState->SupportedTautTriangles.Reset();
	RuntimeState->SupportedTautEdges.Reset();
	RuntimeState->SupportedTautVertices.Reset();
	RuntimeState->TautDiagnostics = FCableSimTautDiagnostics{};
	RuntimeState->bTautWasActive = TautSettings.Mode != ECableSimTautMode::Disabled;
	RuntimeState->bHasLastAcceptedTautEndpoint[0] = false;
	RuntimeState->bHasLastAcceptedTautEndpoint[1] = false;

	const FVector3d StartPosition = ResolveInitialPosition(StartEndpoint, ECableSimEndpoint::Start);
	const FVector3d EndPosition = ResolveInitialPosition(EndEndpoint, ECableSimEndpoint::End);
	RuntimeState->AppliedConfig = BuildCoreConfig();
	RuntimeState->PendingConfig = RuntimeState->AppliedConfig;
	RuntimeState->bConfigPending = false;
	Solver.Initialize(StartPosition, EndPosition, RuntimeState->AppliedConfig);
	RuntimeState->PreviousSolvedPositions.Reset();
	RuntimeState->CurrentSolvedPositions.Reset();
	for (const CableSim::FParticle& Particle : Solver.GetParticles())
	{
		RuntimeState->PreviousSolvedPositions.Add(Particle.Position);
		RuntimeState->CurrentSolvedPositions.Add(Particle.Position);
	}
	RuntimeState->TautDiagnostics.Status = TautSettings.Mode == ECableSimTautMode::Disabled
		? ECableSimTautStatus::Disabled
		: ECableSimTautStatus::Uninitialized;

	const ECableSimEndpoint Endpoints[] = {ECableSimEndpoint::Start, ECableSimEndpoint::End};
	for (const ECableSimEndpoint Endpoint : Endpoints)
	{
		const int32 Index = Endpoint == ECableSimEndpoint::Start ? 0 : 1;
		FVector3d Position;
		ECableSimEndpointBindingStatus EndpointStatus;
		TryResolveTargetPosition(Endpoint, Position, EndpointStatus);
		RuntimeState->EndpointStatuses[Index] = EndpointStatus;
		RuntimeState->EndpointSamples[Index].ConsumedPosition = Position;
		RuntimeState->EndpointSamples[Index].CurrentPosition = Position;
		RuntimeState->EndpointSamples[Index].Velocity = FVector3d::ZeroVector;
		RuntimeState->AppliedEndpointModes[Index] = GetBinding(Endpoint).Mode;
	}
}

void UCableSimComponent::StepSimulation(const int32 StepCount)
{
	const int32 SafeStepCount = FMath::Clamp(StepCount, 0, 10000);
	if (SafeStepCount == 0)
	{
		return;
	}
	if (!Solver.IsInitialized())
	{
		ReinitializeSimulation();
	}
	SynchronizeConfiguration();
	SynchronizeEndpointModes();
	SampleEndpointTargets(FMath::Max(SimulationSettings.FixedTimeStep, 1.e-4));
	for (int32 Step = 0; Step < SafeStepCount; ++Step)
	{
		PerformFixedStep(1.0);
	}
	CommitEndpointSamples();
}

void UCableSimComponent::SetRestLength(const double NewRestLength)
{
	SimulationSettings.RestLength = FMath::Max(NewRestLength, 0.01);
	SynchronizeConfiguration();
}

void UCableSimComponent::SetEndpointMode(
	const ECableSimEndpoint Endpoint,
	const ECableSimEndpointMode Mode)
{
	const int32 EndpointIndex = Endpoint == ECableSimEndpoint::Start ? 0 : 1;
	FCableSimEndpointBinding& Binding = GetBinding(Endpoint);
	if (Solver.IsInitialized() && Mode != ECableSimEndpointMode::Simulated)
	{
		const int32 ParticleIndex = Endpoint == ECableSimEndpoint::Start ? 0 : Solver.GetParticles().Num() - 1;
		const FVector WorldPosition(Solver.GetParticles()[ParticleIndex].Position);
		if (Mode == ECableSimEndpointMode::CableLocalKinematic)
		{
			Binding.LocalTarget = GetComponentTransform().InverseTransformPosition(WorldPosition);
		}
		else if (Mode == ECableSimEndpointMode::WorldKinematic)
		{
			Binding.WorldTarget = WorldPosition;
		}
	}
	Binding.Mode = Mode;
	RuntimeState->AppliedEndpointModes[EndpointIndex] = Mode;
}

void UCableSimComponent::SetEndpointWorldTarget(
	const ECableSimEndpoint Endpoint,
	const FVector WorldPosition)
{
	FCableSimEndpointBinding& Binding = GetBinding(Endpoint);
	Binding.Mode = ECableSimEndpointMode::WorldKinematic;
	Binding.WorldTarget = WorldPosition;
	RuntimeState->AppliedEndpointModes[Endpoint == ECableSimEndpoint::Start ? 0 : 1] = Binding.Mode;
}

void UCableSimComponent::SetEndpointLocalTarget(
	const ECableSimEndpoint Endpoint,
	const FVector LocalPosition)
{
	FCableSimEndpointBinding& Binding = GetBinding(Endpoint);
	Binding.Mode = ECableSimEndpointMode::CableLocalKinematic;
	Binding.LocalTarget = LocalPosition;
	RuntimeState->AppliedEndpointModes[Endpoint == ECableSimEndpoint::Start ? 0 : 1] = Binding.Mode;
}

void UCableSimComponent::AttachEndpointToComponent(
	const ECableSimEndpoint Endpoint,
	USceneComponent* TargetComponent,
	const FName SocketName,
	const FVector LocalOffset)
{
	FCableSimEndpointBinding& Binding = GetBinding(Endpoint);
	Binding.Mode = ECableSimEndpointMode::ComponentKinematic;
	Binding.TargetComponent = TargetComponent;
	Binding.SocketName = SocketName;
	Binding.ComponentLocalOffset = LocalOffset;
	RuntimeState->AppliedEndpointModes[Endpoint == ECableSimEndpoint::Start ? 0 : 1] = Binding.Mode;
}

void UCableSimComponent::ReleaseEndpoint(
	const ECableSimEndpoint Endpoint,
	const bool bPreserveVelocity)
{
	if (Solver.IsInitialized() && !bPreserveVelocity)
	{
		CableSim::FStateSnapshot Snapshot = Solver.CaptureState();
		const int32 ParticleIndex = Endpoint == ECableSimEndpoint::Start ? 0 : Snapshot.Particles.Num() - 1;
		Snapshot.Particles[ParticleIndex].Velocity = FVector3d::ZeroVector;
		Snapshot.Particles[ParticleIndex].PreviousPosition = Snapshot.Particles[ParticleIndex].Position;
		Solver.RestoreState(Snapshot);
	}
	SetEndpointMode(Endpoint, ECableSimEndpointMode::Simulated);
}

void UCableSimComponent::TeleportEndpoint(
	const ECableSimEndpoint Endpoint,
	const FVector WorldPosition,
	const bool bResetVelocity)
{
	if (!Solver.IsInitialized())
	{
		ReinitializeSimulation();
	}
	CableSim::FStateSnapshot Snapshot = Solver.CaptureState();
	const int32 ParticleIndex = Endpoint == ECableSimEndpoint::Start ? 0 : Snapshot.Particles.Num() - 1;
	CableSim::FParticle& Particle = Snapshot.Particles[ParticleIndex];
	Particle.Position = FVector3d(WorldPosition);
	Particle.PreviousPosition = Particle.Position;
	Particle.KinematicTarget = Particle.Position;
	if (bResetVelocity)
	{
		Particle.Velocity = FVector3d::ZeroVector;
	}
	Solver.RestoreState(Snapshot);
	FCableSimEndpointBinding& Binding = GetBinding(Endpoint);
	if (Binding.Mode != ECableSimEndpointMode::Simulated)
	{
		Binding.Mode = ECableSimEndpointMode::WorldKinematic;
		Binding.WorldTarget = WorldPosition;
	}
	RuntimeState->EndpointResolver.Reset();
	RuntimeState->TautSolver.Reset();
	SampleEndpointTargets(0.0);
}

void UCableSimComponent::ResetSimulation()
{
	ReinitializeSimulation();
}

TArray<FVector> UCableSimComponent::GetSimulationPolyline() const
{
	TArray<FVector> Positions;
	Positions.Reserve(Solver.GetParticles().Num());
	for (const CableSim::FParticle& Particle : Solver.GetParticles())
	{
		Positions.Add(FVector(Particle.Position));
	}
	return Positions;
}

TArray<FVector> UCableSimComponent::GetRenderPolyline() const
{
	if (RuntimeState->PreviousSolvedPositions.Num() != RuntimeState->CurrentSolvedPositions.Num()
		|| RuntimeState->CurrentSolvedPositions.IsEmpty())
	{
		return GetSimulationPolyline();
	}
	TArray<FVector> Positions;
	Positions.Reserve(RuntimeState->CurrentSolvedPositions.Num());
	for (int32 Index = 0; Index < RuntimeState->CurrentSolvedPositions.Num(); ++Index)
	{
		Positions.Add(FVector(FMath::Lerp(
			RuntimeState->PreviousSolvedPositions[Index],
			RuntimeState->CurrentSolvedPositions[Index],
			RuntimeState->RenderInterpolationAlpha)));
	}
	return Positions;
}

FCableSimStatus UCableSimComponent::GetSimulationStatus() const
{
	const CableSim::FStepResult& Result = Solver.GetLastStepResult();
	FCableSimStatus Status;
	Status.Status = ToRuntimeStatus(Result.Status);
	Status.StepIndex = static_cast<int64>(Result.StepIndex);
	Status.ParticleCount = Result.ParticleCount;
	Status.ContactCount = Result.ContactCount;
	Status.PersistedContactCount = Result.ContactCount;
	Status.ContactAdditionCount = RuntimeState->ChaosSnapshot.Diagnostics.TriangleCount;
	Status.ContactRemovalCount = 0;
	Status.GuideConstraintCount = Result.GuideConstraintCount;
	Status.ProjectedContactCount = Result.ProjectedContactCount;
	Status.StaticFrictionAnchorCount = Result.StaticFrictionAnchorCount;
	Status.StartEndpointStatus = RuntimeState->EndpointStatuses[0];
	Status.EndEndpointStatus = RuntimeState->EndpointStatuses[1];
	Status.RestLength = Result.RestLength;
	Status.EndpointDistance = Result.EndpointDistance;
	Status.StrainRatio = Result.StrainRatio;
	Status.MaximumSegmentError = Result.MaximumSegmentError;
	Status.MaximumPenetration = Result.MaximumPenetration;
	Status.MaximumGuideError = Result.MaximumGuideError;
	Status.MaximumParticleSpeed = Result.MaximumParticleSpeed;
	Status.RmsParticleSpeed = Result.RmsParticleSpeed;
	Status.MaximumEstimatedTension = Result.MaximumEstimatedTension;
	Status.MaximumEstimatedNormalLoad = Result.MaximumEstimatedNormalLoad;
	Status.DroppedSimulationTime = RuntimeState->DroppedSimulationTime;
	return Status;
}

int64 UCableSimComponent::GetSimulationStepIndex() const
{
	return static_cast<int64>(Solver.GetStepIndex());
}

TArray<FVector> UCableSimComponent::GetTautPolyline() const
{
	TArray<FVector> Positions;
	Positions.Reserve(RuntimeState->TautSolver.GetPoints().Num());
	for (const CableSim::FTautPoint& Point : RuntimeState->TautSolver.GetPoints())
	{
		Positions.Add(FVector(Point.Position));
	}
	return Positions;
}

FCableSimTautDiagnostics UCableSimComponent::GetTautDiagnostics() const
{
	return RuntimeState->TautDiagnostics;
}

FCableSimEndpointBinding& UCableSimComponent::GetBinding(const ECableSimEndpoint Endpoint)
{
	return Endpoint == ECableSimEndpoint::Start ? StartEndpoint : EndEndpoint;
}

const FCableSimEndpointBinding& UCableSimComponent::GetBinding(const ECableSimEndpoint Endpoint) const
{
	return Endpoint == ECableSimEndpoint::Start ? StartEndpoint : EndEndpoint;
}

CableSim::FSimulationConfig UCableSimComponent::BuildCoreConfig() const
{
	CableSim::FSimulationConfig Config;
	Config.RestLength = FMath::Max(SimulationSettings.RestLength, 0.01);
	Config.NodeSpacing = FMath::Max(SimulationSettings.NodeSpacing, 0.01);
	Config.ParticleMass = FMath::Max(SimulationSettings.ParticleMass, 1.e-9);
	Config.Gravity = FVector3d(SimulationSettings.Gravity);
	Config.VelocityDamping = FMath::Clamp(SimulationSettings.VelocityDamping, 0.0, 1.0);
	Config.DistanceOverRelaxation = FMath::Clamp(SimulationSettings.DistanceOverRelaxation, 1.0, 1.05);
	Config.BendingStepStrength = FMath::Clamp(BendingSettings.StepStiffness, 0.0, 1.0);
	Config.FreeBendAngleRadiansPerMeter = FMath::Max(
		FMath::DegreesToRadians(BendingSettings.FreeAngleDegreesPerMeter),
		0.0);
	Config.ConstraintIterations = FMath::Clamp(SimulationSettings.ConstraintIterations, 0, 1024);
	Config.bEnableFriction = FrictionSettings.bEnableFriction;
	Config.StaticFrictionCoefficient = FMath::Max(FrictionSettings.StaticFrictionCoefficient, 0.0);
	Config.DynamicFrictionCoefficient = FMath::Max(FrictionSettings.DynamicFrictionCoefficient, 0.0);
	Config.StaticFrictionSpeedThreshold = FMath::Max(FrictionSettings.StaticSpeedThreshold, 0.0);
	Config.ContactActiveBand = FMath::Max(CollisionSettings.ContactReleaseDistance, CollisionSettings.SkinWidth);
	return Config;
}

FVector3d UCableSimComponent::ResolveInitialPosition(
	const FCableSimEndpointBinding& Binding,
	const ECableSimEndpoint Endpoint) const
{
	return FVector3d(GetComponentTransform().TransformPosition(Binding.SpawnLocalPosition));
}

FVector3d UCableSimComponent::ResolveTargetPosition(const ECableSimEndpoint Endpoint) const
{
	FVector3d Position;
	ECableSimEndpointBindingStatus Status;
	TryResolveTargetPosition(Endpoint, Position, Status);
	return Position;
}

bool UCableSimComponent::TryResolveTargetPosition(
	const ECableSimEndpoint Endpoint,
	FVector3d& OutPosition,
	ECableSimEndpointBindingStatus& OutStatus) const
{
	const FCableSimEndpointBinding& Binding = GetBinding(Endpoint);
	if (Binding.Mode == ECableSimEndpointMode::Simulated)
	{
		OutStatus = ECableSimEndpointBindingStatus::Simulated;
		if (Solver.IsInitialized())
		{
			const int32 ParticleIndex = Endpoint == ECableSimEndpoint::Start ? 0 : Solver.GetParticles().Num() - 1;
			OutPosition = Solver.GetParticles()[ParticleIndex].Position;
		}
		else
		{
			OutPosition = FVector3d(GetComponentTransform().TransformPosition(Binding.SpawnLocalPosition));
		}
		return true;
	}
	return RuntimeState->EndpointResolver.Resolve(*this, Binding, Endpoint, OutPosition, OutStatus);
}

void UCableSimComponent::SynchronizeConfiguration()
{
	if (!Solver.IsInitialized())
	{
		return;
	}
	const CableSim::FSimulationConfig CurrentConfig = BuildCoreConfig();
	if (!CurrentConfig.Equals(RuntimeState->AppliedConfig))
	{
		RuntimeState->PendingConfig = CurrentConfig;
		RuntimeState->bConfigPending = true;
	}
}

void UCableSimComponent::SynchronizeEndpointModes()
{
	if (!Solver.IsInitialized())
	{
		return;
	}
	const ECableSimEndpoint Endpoints[] = {ECableSimEndpoint::Start, ECableSimEndpoint::End};
	for (const ECableSimEndpoint Endpoint : Endpoints)
	{
		const int32 EndpointIndex = Endpoint == ECableSimEndpoint::Start ? 0 : 1;
		const ECableSimEndpointMode Mode = GetBinding(Endpoint).Mode;
		if (Mode != RuntimeState->AppliedEndpointModes[EndpointIndex])
		{
			RuntimeState->AppliedEndpointModes[EndpointIndex] = Mode;
			RuntimeState->EndpointResolver.Reset();
		}
	}
}

void UCableSimComponent::SampleEndpointTargets(const double DeltaTime)
{
	RuntimeState->EndpointSampleElapsedTime += FMath::Max(DeltaTime, 0.0);
	const double SampleTime = FMath::Max(RuntimeState->EndpointSampleElapsedTime, 1.e-6);
	const ECableSimEndpoint Endpoints[] = {ECableSimEndpoint::Start, ECableSimEndpoint::End};
	for (const ECableSimEndpoint Endpoint : Endpoints)
	{
		const int32 Index = Endpoint == ECableSimEndpoint::Start ? 0 : 1;
		FEndpointFrameSample& Sample = RuntimeState->EndpointSamples[Index];
		ECableSimEndpointBindingStatus EndpointStatus;
		TryResolveTargetPosition(Endpoint, Sample.CurrentPosition, EndpointStatus);
		RuntimeState->EndpointStatuses[Index] = EndpointStatus;
		Sample.Velocity = (Sample.CurrentPosition - Sample.ConsumedPosition) / SampleTime;
	}
}

CableSim::FStepInput UCableSimComponent::BuildStepInput(const double InterpolationAlpha) const
{
	CableSim::FStepInput Input;
	Input.DeltaTime = FMath::Max(SimulationSettings.FixedTimeStep, 1.e-4);
	const ECableSimEndpoint Endpoints[] = {ECableSimEndpoint::Start, ECableSimEndpoint::End};
	for (const ECableSimEndpoint Endpoint : Endpoints)
	{
		const int32 Index = Endpoint == ECableSimEndpoint::Start ? 0 : 1;
		const FEndpointFrameSample& Sample = RuntimeState->EndpointSamples[Index];
		CableSim::FEndpointStepInput& EndpointInput = Endpoint == ECableSimEndpoint::Start
			? Input.StartEndpoint
			: Input.EndEndpoint;
		EndpointInput.Mode = ToCoreMode(GetBinding(Endpoint).Mode);
		EndpointInput.TargetPosition = FMath::Lerp(
			Sample.ConsumedPosition,
			Sample.CurrentPosition,
			FMath::Clamp(InterpolationAlpha, 0.0, 1.0));
		EndpointInput.TargetVelocity = EndpointInput.Mode == CableSim::EParticleMode::Kinematic
			? Sample.Velocity
			: FVector3d::ZeroVector;
	}
	return Input;
}

void UCableSimComponent::CommitEndpointSamples()
{
	for (FEndpointFrameSample& Sample : RuntimeState->EndpointSamples)
	{
		Sample.ConsumedPosition = Sample.CurrentPosition;
	}
	RuntimeState->EndpointSampleElapsedTime = 0.0;
}

void UCableSimComponent::GatherCollisionSnapshot(
	const CableSim::FStepInput& Input,
	const TConstArrayView<AActor*> IgnoredActors)
{
	RuntimeState->bSnapshotValid = false;
	if (!CollisionSettings.bEnableWorldCollision && TautSettings.Mode == ECableSimTautMode::Disabled)
	{
		RuntimeState->ChaosSnapshot.Reset();
		return;
	}
	const double Tolerance = FMath::Max(TautSettings.TopologyTolerance, 0.001);
	TArray<FVector3d, TInlineAllocator<32>> TautPathPoints;
	if (TautSettings.Mode != ECableSimTautMode::Disabled)
	{
		for (const CableSim::FTautPoint& Point : RuntimeState->TautSolver.GetPoints())
		{
			TautPathPoints.Add(Point.Position);
		}
	}
	RuntimeState->bSnapshotValid = FCableSimChaosCollisionAdapter::GatherSnapshot(
		GetWorld(),
		GetOwner(),
		IgnoredActors,
		CollisionSettings,
		Tolerance,
		Solver.GetParticles(),
		Input.StartEndpoint.TargetPosition,
		Input.EndEndpoint.TargetPosition,
		TautPathPoints,
		RuntimeState->ChaosObjectTracker,
		RuntimeState->ChaosSnapshot);
}

CableSim::FManifoldConfig UCableSimComponent::BuildManifoldConfig() const
{
	CableSim::FManifoldConfig Config;
	Config.NodeRadius = FMath::Max(CollisionSettings.Radius, 0.0);
	Config.ActiveBand = FMath::Max(CollisionSettings.ContactReleaseDistance, CollisionSettings.SkinWidth);
	Config.MergeNormalCosine = FMath::Cos(FMath::DegreesToRadians(
		FMath::Clamp(CollisionSettings.ContactNormalToleranceDegrees, 0.0, 90.0)));
	Config.MergeOffsetTolerance = FMath::Max(CollisionSettings.SkinWidth, 0.1);
	Config.MaxPlanesPerNode = FMath::Clamp(CollisionSettings.MaximumContactsPerParticle, 1, 4);
	Config.Tolerance = FMath::Max(TautSettings.TopologyTolerance, 0.001);
	return Config;
}

void UCableSimComponent::PerformFixedStep(const double InterpolationAlpha)
{
	if (RuntimeState->bConfigPending)
	{
		if (Solver.ApplyConfig(RuntimeState->PendingConfig))
		{
			RuntimeState->AppliedConfig = Solver.GetConfig();
		}
		RuntimeState->bConfigPending = false;
	}

	TArray<AActor*> IgnoredActors;
	if (IsValid(StartEndpoint.TargetComponent))
	{
		IgnoredActors.AddUnique(StartEndpoint.TargetComponent->GetOwner());
	}
	if (IsValid(EndEndpoint.TargetComponent))
	{
		IgnoredActors.AddUnique(EndEndpoint.TargetComponent->GetOwner());
	}
	RuntimeState->EndpointResolver.AppendResolvedActors(IgnoredActors);

	CableSim::FStepInput Input = BuildStepInput(InterpolationAlpha);
	GatherCollisionSnapshot(Input, IgnoredActors);
	PerformTautStep(Input);
	RuntimeState->LastGuides = Input.GuideConstraints;

	const CableSim::FManifoldConfig ManifoldConfig = BuildManifoldConfig();
	const bool bCollide = CollisionSettings.bEnableWorldCollision && RuntimeState->bSnapshotValid;
	Solver.AdvanceStep(Input, [this, bCollide, ManifoldConfig](
		const TConstArrayView<CableSim::FParticle> Particles,
		TArray<CableSim::FContactConstraint>& OutContacts)
	{
		if (!bCollide)
		{
			return;
		}
		const TArray<CableSim::FCollisionTriangle>& Triangles = RuntimeState->ChaosSnapshot.Triangles;
		const TArray<CableSim::FCollisionEdge>& Edges = RuntimeState->ChaosSnapshot.Edges;
		for (int32 Index = 0; Index < Particles.Num(); ++Index)
		{
			if (Particles[Index].Mode != CableSim::EParticleMode::Dynamic)
			{
				continue;
			}
			CableSim::FContactManifoldCompiler::CompileNodeContacts(
				Index,
				Particles[Index].Position,
				Particles[Index].PreviousPosition,
				Triangles,
				Edges,
				ManifoldConfig,
				OutContacts);
		}
	});
	RuntimeState->PreviousSolvedPositions = MoveTemp(RuntimeState->CurrentSolvedPositions);
	RuntimeState->CurrentSolvedPositions.Reset(Solver.GetParticles().Num());
	for (const CableSim::FParticle& Particle : Solver.GetParticles())
	{
		RuntimeState->CurrentSolvedPositions.Add(Particle.Position);
	}
	if (RuntimeState->PreviousSolvedPositions.Num() != RuntimeState->CurrentSolvedPositions.Num())
	{
		RuntimeState->PreviousSolvedPositions = RuntimeState->CurrentSolvedPositions;
	}
}

void UCableSimComponent::PerformTautStep(CableSim::FStepInput& Input)
{
	if (TautSettings.Mode == ECableSimTautMode::Disabled)
	{
		if (RuntimeState->bTautWasActive)
		{
			RuntimeState->TautSolver.Reset();
			RuntimeState->ChaosObjectTracker.Reset();
			RuntimeState->ChaosSnapshot.Reset();
			RuntimeState->SupportedTautTriangles.Reset();
			RuntimeState->SupportedTautEdges.Reset();
			RuntimeState->SupportedTautVertices.Reset();
		}
		RuntimeState->bTautWasActive = false;
		RuntimeState->TautDiagnostics = FCableSimTautDiagnostics{};
		return;
	}
	const int32 ConstrainedEndpointIndex = TautSettings.ConstrainedEndpoint == ECableSimEndpoint::Start
		? 0 : 1;
	auto FreezeConstrainedEndpoint = [this, &Input, ConstrainedEndpointIndex]()
	{
		if (TautSettings.Mode != ECableSimTautMode::Enabled || Solver.GetParticles().Num() < 2)
		{
			return;
		}
		CableSim::FEndpointStepInput& EndpointInput = ConstrainedEndpointIndex == 0
			? Input.StartEndpoint : Input.EndEndpoint;
		EndpointInput.TargetPosition = RuntimeState->bHasLastAcceptedTautEndpoint[ConstrainedEndpointIndex]
			? RuntimeState->LastAcceptedTautEndpoints[ConstrainedEndpointIndex]
			: (ConstrainedEndpointIndex == 0
				? Solver.GetParticles()[0].Position : Solver.GetParticles().Last().Position);
		EndpointInput.TargetVelocity = FVector3d::ZeroVector;
	};

	FCableSimTautDiagnostics Diagnostics;
	const bool bSnapshotSucceeded = RuntimeState->bSnapshotValid;
	const FCableSimChaosSnapshotDiagnostics& SnapshotDiagnostics = RuntimeState->ChaosSnapshot.Diagnostics;
	Diagnostics.bSnapshotSucceeded = bSnapshotSucceeded;
	Diagnostics.bFeatureBudgetExceeded = SnapshotDiagnostics.bFeatureBudgetExceeded;
	Diagnostics.OverlapCount = SnapshotDiagnostics.OverlapCount;
	Diagnostics.PhysicsObjectCount = SnapshotDiagnostics.PhysicsObjectCount;
	Diagnostics.ShapeCount = SnapshotDiagnostics.ShapeCount;
	Diagnostics.TriangleCount = SnapshotDiagnostics.TriangleCount;
	Diagnostics.ExtractedEdgeCount = SnapshotDiagnostics.EdgeCount;
	Diagnostics.ExtractedVertexCount = SnapshotDiagnostics.VertexCount;
	Diagnostics.OverValenceVertexCount = SnapshotDiagnostics.OverValenceVertexCount;
	Diagnostics.SnapshotMilliseconds = SnapshotDiagnostics.SnapshotMilliseconds;
	Diagnostics.TopologyCompileMilliseconds = SnapshotDiagnostics.CompileMilliseconds;
	RuntimeState->SupportedTautTriangles.Reset();
	RuntimeState->SupportedTautEdges.Reset();
	RuntimeState->SupportedTautVertices.Reset();
	if (!bSnapshotSucceeded)
	{
		Diagnostics.Status = ECableSimTautStatus::SnapshotFailed;
		RuntimeState->TautDiagnostics = Diagnostics;
		FreezeConstrainedEndpoint();
		return;
	}

	auto IsSupported = [](const CableSim::ECollisionGeometryType GeometryType, const bool bStaticObject)
	{
		return bStaticObject
			&& (GeometryType == CableSim::ECollisionGeometryType::Box
				|| GeometryType == CableSim::ECollisionGeometryType::Convex);
	};
	for (const CableSim::FCollisionTriangle& Triangle : RuntimeState->ChaosSnapshot.Triangles)
	{
		if (IsSupported(Triangle.GeometryType, Triangle.bStaticObject))
		{
			RuntimeState->SupportedTautTriangles.Add(Triangle);
		}
	}
	for (const CableSim::FCollisionEdge& Edge : RuntimeState->ChaosSnapshot.Edges)
	{
		if (IsSupported(Edge.GeometryType, Edge.bStaticObject))
		{
			RuntimeState->SupportedTautEdges.Add(Edge);
		}
	}
	for (const CableSim::FCollisionVertex& Vertex : RuntimeState->ChaosSnapshot.Vertices)
	{
		if (IsSupported(Vertex.GeometryType, Vertex.bStaticObject))
		{
			RuntimeState->SupportedTautVertices.Add(Vertex);
		}
	}
	Diagnostics.SupportedEdgeCount = RuntimeState->SupportedTautEdges.Num();
	const CableSim::FTautCollisionScene Scene{
		RuntimeState->SupportedTautTriangles,
		RuntimeState->SupportedTautEdges,
		RuntimeState->SupportedTautVertices};

	CableSim::FTautConfig TautConfig;
	TautConfig.TopologyTolerance = FMath::Max(TautSettings.TopologyTolerance, 0.001);
	TautConfig.MovementConvergenceTolerance = FMath::Max(
		TautSettings.MovementConvergenceTolerance, 0.0001);
	TautConfig.MaximumMovementIterations = FMath::Clamp(
		TautSettings.MaximumMovementIterations, 1, 128);
	TautConfig.MaximumCollisionPhases = FMath::Clamp(
		TautSettings.MaximumCollisionPhases, 1, 128);
	const bool bConfigChanged = !FMath::IsNearlyEqual(
		TautConfig.TopologyTolerance,
		RuntimeState->AppliedTautConfig.TopologyTolerance,
		1.e-9)
		|| !FMath::IsNearlyEqual(
			TautConfig.MovementConvergenceTolerance,
			RuntimeState->AppliedTautConfig.MovementConvergenceTolerance,
			1.e-9)
		|| TautConfig.MaximumMovementIterations != RuntimeState->AppliedTautConfig.MaximumMovementIterations
		|| TautConfig.MaximumCollisionPhases != RuntimeState->AppliedTautConfig.MaximumCollisionPhases;
	if (!RuntimeState->bTautWasActive || !RuntimeState->TautSolver.IsInitialized() || bConfigChanged)
	{
		TArray<FVector3d, TInlineAllocator<128>> SeedPolyline;
		for (const CableSim::FParticle& Particle : Solver.GetParticles())
		{
			SeedPolyline.Add(Particle.Position);
		}
		if (!RuntimeState->TautSolver.Initialize(SeedPolyline, Scene, TautConfig))
		{
			const CableSim::FTautStepResult& Result = RuntimeState->TautSolver.GetLastResult();
			Diagnostics.Status = ToRuntimeTautStatus(Result.Status);
			Diagnostics.bPathCollisionFree = Result.bPathCollisionFree;
			RuntimeState->TautDiagnostics = Diagnostics;
			RuntimeState->bTautWasActive = true;
			FreezeConstrainedEndpoint();
			return;
		}
		RuntimeState->AppliedTautConfig = TautConfig;
	}
	RuntimeState->bTautWasActive = true;

	CableSim::FTautStepInput TautInput;
	TautInput.StartTarget = Input.StartEndpoint.TargetPosition;
	TautInput.EndTarget = Input.EndEndpoint.TargetPosition;
	const double SolverStart = FPlatformTime::Seconds();
	const CableSim::FTautStateSnapshot InitialState = RuntimeState->TautSolver.CaptureState();
	CableSim::FTautStepResult Result = RuntimeState->TautSolver.AdvanceStep(TautInput, Scene);
	const auto IsAccepted = [this](const CableSim::FTautStepResult& Candidate)
	{
		return (Candidate.Status == CableSim::ETautStatus::Ready
				|| Candidate.Status == CableSim::ETautStatus::NoRelevantGeometry)
			&& Candidate.bPathCollisionFree
			&& Candidate.PathLength <= SimulationSettings.RestLength
				+ FMath::Max(TautSettings.LengthTolerance, 0.0);
	};

	bool bAccepted = IsAccepted(Result);
	if (TautSettings.Mode == ECableSimTautMode::Enabled && !bAccepted)
	{
		// Solve every reach probe from the exact same state. This makes the
		// selected endpoint target deterministic and prevents failed probes from
		// contaminating later probes with contacts from a different path.
		const TArray<CableSim::FParticle>& Particles = Solver.GetParticles();
		const int32 EndpointIndex = TautSettings.ConstrainedEndpoint == ECableSimEndpoint::Start ? 0 : 1;
		const FVector3d CurrentTarget = EndpointIndex == 0
			? Particles[0].Position : Particles.Last().Position;
		const FVector3d RequestedTarget = EndpointIndex == 0
			? TautInput.StartTarget : TautInput.EndTarget;
		// Probe along the requested direction over a bounded span, not all the way to a
		// far overextended target. Capping the span at the rope length keeps the reach
		// boundary resolved finely (span/2^iterations), so an overextended endpoint tracks
		// the boundary continuously instead of landing coarsely and freezing. When the
		// request is within reach this span equals the requested distance -- identical to
		// a straight Lerp to the target.
		const FVector3d ToRequested = RequestedTarget - CurrentTarget;
		const double RequestedDistance = ToRequested.Length();
		const FVector3d ReachDirection = RequestedDistance > UE_KINDA_SMALL_NUMBER
			? ToRequested / RequestedDistance : FVector3d::ZeroVector;
		const double ProbeSpan = FMath::Min(
			RequestedDistance, FMath::Max(SimulationSettings.RestLength, 0.0));
		auto CandidateAt = [&](const double Alpha)
		{
			return CurrentTarget + ReachDirection * (Alpha * ProbeSpan);
		};
		double AcceptedAlpha = 0.0;
		CableSim::FTautStateSnapshot AcceptedState;
		bool bHasAcceptedProbe = false;

		auto Probe = [&](const double Alpha)
		{
			RuntimeState->TautSolver.RestoreState(InitialState);
			CableSim::FTautStepInput ProbeInput = TautInput;
			const FVector3d CandidateTarget = CandidateAt(Alpha);
			if (EndpointIndex == 0)
			{
				ProbeInput.StartTarget = CandidateTarget;
			}
			else
			{
				ProbeInput.EndTarget = CandidateTarget;
			}
			const CableSim::FTautStepResult ProbeResult = RuntimeState->TautSolver.AdvanceStep(ProbeInput, Scene);
			if (IsAccepted(ProbeResult))
			{
				AcceptedAlpha = Alpha;
				AcceptedState = RuntimeState->TautSolver.CaptureState();
				Result = ProbeResult;
				return true;
			}
			return false;
		};

		bHasAcceptedProbe = Probe(0.0);
		if (bHasAcceptedProbe)
		{
			double Lower = 0.0;
			double Upper = 1.0;
			for (int32 Iteration = 0;
				Iteration < FMath::Clamp(TautSettings.ReachBisectionIterations, 1, 24);
				++Iteration)
			{
				const double Middle = 0.5 * (Lower + Upper);
				if (Probe(Middle))
				{
					Lower = Middle;
				}
				else
				{
					Upper = Middle;
				}
			}
			RuntimeState->TautSolver.RestoreState(AcceptedState);
			CableSim::FEndpointStepInput& ConstrainedInput = EndpointIndex == 0
				? Input.StartEndpoint : Input.EndEndpoint;
			ConstrainedInput.TargetPosition = CandidateAt(AcceptedAlpha);
			ConstrainedInput.TargetVelocity *= AcceptedAlpha;
			bAccepted = true;
		}
		else
		{
			RuntimeState->TautSolver.RestoreState(InitialState);
			CableSim::FEndpointStepInput& ConstrainedInput = EndpointIndex == 0
				? Input.StartEndpoint : Input.EndEndpoint;
			ConstrainedInput.TargetPosition = RuntimeState->bHasLastAcceptedTautEndpoint[EndpointIndex]
				? RuntimeState->LastAcceptedTautEndpoints[EndpointIndex]
				: CurrentTarget;
			ConstrainedInput.TargetVelocity = FVector3d::ZeroVector;
		}
	}

	if (bAccepted)
	{
		const TArray<CableSim::FTautPoint>& Points = RuntimeState->TautSolver.GetPoints();
		RuntimeState->LastAcceptedTautEndpoints[0] = Points[0].Position;
		RuntimeState->LastAcceptedTautEndpoints[1] = Points.Last().Position;
		RuntimeState->bHasLastAcceptedTautEndpoint[0] = true;
		RuntimeState->bHasLastAcceptedTautEndpoint[1] = true;
		if (TautSettings.Mode == ECableSimTautMode::Enabled)
		{
			BuildTautGuideConstraints(Input);
		}
	}
	Diagnostics.SolverMilliseconds = (FPlatformTime::Seconds() - SolverStart) * 1000.0;
	Diagnostics.Status = ToRuntimeTautStatus(Result.Status);
	Diagnostics.StepIndex = static_cast<int64>(Result.StepIndex);
	Diagnostics.PointCount = Result.PointCount;
	Diagnostics.ContactCount = Result.ContactCount;
	Diagnostics.MovementIterationCount = Result.MovementIterationCount;
	Diagnostics.CollisionPhaseCount = Result.CollisionPhaseCount;
	Diagnostics.TopologyEventCount = Result.TopologyEventCount;
	Diagnostics.PathLength = Result.PathLength;
	Diagnostics.bPathCollisionFree = Result.bPathCollisionFree;
	RuntimeState->TautDiagnostics = Diagnostics;
}

void UCableSimComponent::BuildTautGuideConstraints(CableSim::FStepInput& Input) const
{
	Input.GuideConstraints.Reset();
	if (TautSettings.Mode != ECableSimTautMode::Enabled)
	{
		return;
	}
	const TArray<CableSim::FTautPoint>& Points = RuntimeState->TautSolver.GetPoints();
	const TArray<CableSim::FParticle>& Particles = Solver.GetParticles();
	if (Points.Num() < 2 || Particles.Num() < 3 || SimulationSettings.RestLength <= 0.0)
	{
		return;
	}
	const CableSim::ETautStatus Status = RuntimeState->TautSolver.GetLastResult().Status;
	if (Status != CableSim::ETautStatus::Ready && Status != CableSim::ETautStatus::NoRelevantGeometry)
	{
		return;
	}
	const double PathLength = CalculatePolylineLength(Points);
	const double PathRatio = PathLength / SimulationSettings.RestLength;
	const double ActivationRatio = FMath::Clamp(TautSettings.GuideActivationPathRatio, 0.0, 1.0);
	if (PathRatio < ActivationRatio)
	{
		return;
	}
	const double ActivationAlpha = ActivationRatio >= 1.0 - 1.e-9
		? 1.0
		: FMath::SmoothStep(
			0.0,
			1.0,
			FMath::Clamp((PathRatio - ActivationRatio) / (1.0 - ActivationRatio), 0.0, 1.0));
	if (ActivationAlpha <= 0.0)
	{
		return;
	}
	const double Slack = FMath::Max(SimulationSettings.RestLength - PathLength, 0.0);
	// Parabolic-catenary sag envelope (not linear in slack): a rope of span PathLength
	// with Slack excess length sags ~sqrt(3*span*slack/8) at its deepest point. Sizing
	// the corridor to that shape keeps the guide from fighting the rope's natural rest
	// shape as slack shrinks.
	const double BaseRadius = FMath::Min(
		FMath::Max(TautSettings.GuideSagScale, 0.0) * FMath::Sqrt(3.0 * PathLength * Slack / 8.0),
		FMath::Max(TautSettings.MaximumGuideRadius, 0.0));
	const double CollisionClearance = FMath::Max(CollisionSettings.Radius, 0.0)
		+ FMath::Max(CollisionSettings.SkinWidth, 0.0)
		+ FMath::Max(TautSettings.TopologyTolerance, 0.0);
	// The clearance floor only matters where the taut line rides geometry: there a node
	// must stay clear of the surface. In a free span the line hangs in open air, so the
	// corridor collapses to MinimumGuideRadius and a taut span hugs the line instead of
	// bowing by ~a cable radius. A contact is an interior taut point.
	const double ContactWindow = 3.0 * CollisionClearance;
	auto NearestContactDistance = [&Points](const FVector3d& Position) -> double
	{
		double Nearest = TNumericLimits<double>::Max();
		for (int32 Index = 1; Index + 1 < Points.Num(); ++Index)
		{
			Nearest = FMath::Min(Nearest, FVector3d::Distance(Position, Points[Index].Position));
		}
		return Nearest;
	};
	const double MinimumRadius = FMath::Max(TautSettings.MinimumGuideRadius, 0.0);
	// Previous frame's corridor width per node, so the corridor can close only gradually.
	// Collapsing it in one frame yanks a cable that was resting inside the wide corridor
	// onto the taut line (the snap); easing it in lets the cable follow smoothly, matching
	// the talk's "as the rope gets closer to taut, the distance gets shorter".
	TMap<int32, double> PreviousWidth;
	PreviousWidth.Reserve(RuntimeState->LastGuides.Num());
	for (const CableSim::FGuideConstraint& Previous : RuntimeState->LastGuides)
	{
		PreviousWidth.Add(Previous.ParticleIndex, Previous.MaximumDistance);
	}
	const double CloseFraction = FMath::Clamp(TautSettings.GuideCloseFraction, 0.0, 1.0);
	for (int32 ParticleIndex = 1; ParticleIndex + 1 < Particles.Num(); ++ParticleIndex)
	{
		if (Particles[ParticleIndex].Mode != CableSim::EParticleMode::Dynamic)
		{
			continue;
		}
		const double Alpha = FMath::Clamp(
			Particles[ParticleIndex].MaterialCoordinate / SimulationSettings.RestLength,
			0.0,
			1.0);
		CableSim::FGuideConstraint& Guide = Input.GuideConstraints.AddDefaulted_GetRef();
		Guide.ParticleIndex = ParticleIndex;
		Guide.TargetPosition = SamplePolyline(Points, Alpha);
		const double Floor = NearestContactDistance(Guide.TargetPosition) <= ContactWindow
			? FMath::Max(MinimumRadius, CollisionClearance)
			: MinimumRadius;
		const double TargetWidth = FMath::Max(BaseRadius * FMath::Sin(UE_PI * Alpha), Floor);
		if (const double* Prev = PreviousWidth.Find(ParticleIndex))
		{
			// Ease toward the target width; the max() lets it widen freely but close only
			// by CloseFraction per step.
			Guide.MaximumDistance = FMath::Max(TargetWidth, FMath::Lerp(*Prev, TargetWidth, CloseFraction));
		}
		else
		{
			// First engagement: open the corridor to wherever the node already sits so it
			// is not snapped, then let it close over the following steps.
			const double NodeOffset = FVector3d::Distance(
				Particles[ParticleIndex].Position, Guide.TargetPosition);
			Guide.MaximumDistance = FMath::Max(TargetWidth, NodeOffset);
		}
		Guide.StepStrength = FMath::Clamp(TautSettings.GuideStepStrength, 0.0, 1.0) * ActivationAlpha;
	}
}

void UCableSimComponent::RefreshEditorPreview()
{
	const UWorld* World = GetWorld();
	if (!World || World->IsGameWorld())
	{
		return;
	}

	auto ResolvePreviewEndpoint = [this](const FCableSimEndpointBinding& Binding, const ECableSimEndpoint Endpoint)
	{
		if (Binding.Mode == ECableSimEndpointMode::Simulated)
		{
			return FVector3d(GetComponentTransform().TransformPosition(Binding.SpawnLocalPosition));
		}
		FVector3d Position;
		ECableSimEndpointBindingStatus Status;
		if (RuntimeState->EndpointResolver.Resolve(*this, Binding, Endpoint, Position, Status))
		{
			return Position;
		}
		return FVector3d(GetComponentTransform().TransformPosition(Binding.SpawnLocalPosition));
	};

	const FVector3d Start = ResolvePreviewEndpoint(StartEndpoint, ECableSimEndpoint::Start);
	const FVector3d End = ResolvePreviewEndpoint(EndEndpoint, ECableSimEndpoint::End);
	const CableSim::FSimulationConfig Config = BuildCoreConfig();
	const bool bChanged = !RuntimeState->bHasEditorPreview
		|| !Start.Equals(RuntimeState->EditorPreviewEndpoints[0], 1.e-4)
		|| !End.Equals(RuntimeState->EditorPreviewEndpoints[1], 1.e-4)
		|| !Config.Equals(RuntimeState->AppliedConfig);
	if (!bChanged)
	{
		return;
	}

	RuntimeState->EndpointResolver.Reset();
	RuntimeState->EditorPreviewEndpoints[0] = Start;
	RuntimeState->EditorPreviewEndpoints[1] = End;
	RuntimeState->bHasEditorPreview = true;
	RuntimeState->AppliedConfig = Config;
	Solver.Initialize(Start, End, Config);
	RuntimeState->PreviousSolvedPositions.Reset();
	RuntimeState->CurrentSolvedPositions.Reset();
	for (const CableSim::FParticle& Particle : Solver.GetParticles())
	{
		RuntimeState->PreviousSolvedPositions.Add(Particle.Position);
		RuntimeState->CurrentSolvedPositions.Add(Particle.Position);
	}
	RuntimeState->RenderInterpolationAlpha = 1.0;
	RuntimeState->TautSolver.Reset();
	RuntimeState->ChaosSnapshot.Reset();
	RuntimeState->TautDiagnostics = FCableSimTautDiagnostics{};
	RuntimeState->bTautWasActive = false;
	if (TautSettings.Mode == ECableSimTautMode::Disabled)
	{
		return;
	}

	// Outside Play, TickComponent never reaches PerformFixedStep, so this is
	// the only place the taut path gets advanced.
	CableSim::FStepInput PreviewInput;
	PreviewInput.StartEndpoint.TargetPosition = Start;
	PreviewInput.EndEndpoint.TargetPosition = End;
	TArray<AActor*> IgnoredActors;
	if (IsValid(StartEndpoint.TargetComponent))
	{
		IgnoredActors.AddUnique(StartEndpoint.TargetComponent->GetOwner());
	}
	if (IsValid(EndEndpoint.TargetComponent))
	{
		IgnoredActors.AddUnique(EndEndpoint.TargetComponent->GetOwner());
	}
	GatherCollisionSnapshot(PreviewInput, IgnoredActors);
	PerformTautStep(PreviewInput);
}

void UCableSimComponent::RefreshVisualization()
{
	if (!IsRegistered())
	{
		return;
	}
	UpdateBounds();
	MarkRenderStateDirty();
}

FBoxSphereBounds UCableSimComponent::CalcBounds(const FTransform& LocalToWorld) const
{
	FBox Bounds(ForceInit);
	for (const FVector& Position : GetRenderPolyline())
	{
		Bounds += Position;
	}
	for (const CableSim::FTautPoint& Point : RuntimeState->TautSolver.GetPoints())
	{
		Bounds += FVector(Point.Position);
	}
	if (DebugSettings.bDraw && RuntimeState->ChaosSnapshot.QueryBounds.IsValid)
	{
		Bounds += RuntimeState->ChaosSnapshot.QueryBounds;
	}
	if (!Bounds.IsValid)
	{
		Bounds = FBox(LocalToWorld.GetLocation() - FVector(1.0), LocalToWorld.GetLocation() + FVector(1.0));
	}
	return FBoxSphereBounds(Bounds.ExpandBy(50.0));
}

#if WITH_EDITOR
void UCableSimComponent::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);
	RuntimeState->EndpointResolver.Reset();
	RefreshEditorPreview();
	RefreshVisualization();
}
#endif

FDebugRenderSceneProxy* UCableSimComponent::CreateDebugSceneProxy()
{
	const TArray<CableSim::FParticle>& Particles = Solver.GetParticles();
	const TArray<FVector> RenderPositions = GetRenderPolyline();
	if ((!PreviewSettings.bVisible && !DebugSettings.bDraw) || Particles.Num() < 2 || RenderPositions.Num() != Particles.Num())
	{
		return nullptr;
	}

	FCableSimDebugRenderSceneProxy* Proxy = new FCableSimDebugRenderSceneProxy(this);
	if (PreviewSettings.bVisible)
	{
		const FColor PreviewColor = PreviewSettings.Color.ToFColor(true);
		for (int32 Index = 0; Index + 1 < RenderPositions.Num(); ++Index)
		{
			Proxy->Lines.Emplace(
				RenderPositions[Index],
				RenderPositions[Index + 1],
				PreviewColor,
				PreviewSettings.LineThickness);
		}
	}

#if UE_BUILD_SHIPPING
	return Proxy;
#else
	if (!DebugSettings.bDraw)
	{
		return Proxy;
	}
	auto HasFlag = [this](const ECableSimDebugDraw Flag)
	{
		return (DebugSettings.DrawFlags & (1 << static_cast<uint8>(Flag))) != 0;
	};

	const CableSim::FStepResult& Result = Solver.GetLastStepResult();
	const float Thickness = DebugSettings.LineThickness;
	const FColor CableTeal(61, 214, 196);
	const FColor WarnRed(225, 45, 57);
	const FColor EndpointAmber(255, 176, 32);
	const FColor SurfaceOrange(255, 122, 69);
	const FColor FrictionHeld(76, 195, 138);
	const FColor FrictionSlip(245, 166, 35);
	const FColor TautViolet(179, 127, 235);
	const FColor CorridorBlue(91, 141, 239);
	const FColor NormalWhite(232, 236, 239);
	const FColor EdgeContactYellow(247, 201, 72);
	auto Ramp = [](double Ratio) -> FColor
	{
		const float T = static_cast<float>(FMath::Clamp(Ratio, 0.0, 1.0));
		const FLinearColor Low(0.24f, 0.84f, 0.77f);
		const FLinearColor Mid(0.96f, 0.65f, 0.14f);
		const FLinearColor High(0.88f, 0.18f, 0.22f);
		return (T < 0.5f ? FMath::Lerp(Low, Mid, T * 2.0f)
			: FMath::Lerp(Mid, High, (T - 0.5f) * 2.0f)).ToFColor(true);
	};
	const bool bOverextended = Result.StrainRatio > 1.e-4;
	const bool bTautActive = TautSettings.Mode != ECableSimTautMode::Disabled;

	// Cable, coloured by per-node tension (red when overextended); endpoints marked.
	if (HasFlag(ECableSimDebugDraw::Cable))
	{
		const double MaxTension = FMath::Max(Result.MaximumEstimatedTension, 1.e-6);
		for (int32 Index = 0; Index + 1 < RenderPositions.Num(); ++Index)
		{
			const FColor SegColor = bOverextended
				? WarnRed : Ramp(Particles[Index].EstimatedTension / MaxTension);
			Proxy->Lines.Emplace(RenderPositions[Index], RenderPositions[Index + 1], SegColor, Thickness + 1.0f);
		}
		for (int32 Index = 0; Index < RenderPositions.Num(); ++Index)
		{
			const bool bEndpoint = Index == 0 || Index == RenderPositions.Num() - 1;
			const FColor NodeColor = Particles[Index].Mode == CableSim::EParticleMode::Kinematic
				? EndpointAmber : CableTeal;
			Proxy->Spheres.Emplace(bEndpoint ? 5.0f : 2.5f, RenderPositions[Index],
				FLinearColor(NodeColor), FDebugRenderSceneProxy::EDrawType::SolidAndWireMeshes);
		}
	}

	// Contacts (normal + static/slipping state + surface velocity) and normal load.
	if (HasFlag(ECableSimDebugDraw::Contacts) || HasFlag(ECableSimDebugDraw::Load))
	{
		const double MaxLoad = FMath::Max(Result.MaximumEstimatedNormalLoad, 1.e-6);
		for (const CableSim::FContactDiagnostic& Contact : Solver.GetLastContactDiagnostics())
		{
			if (!Contact.bProjected || !Particles.IsValidIndex(Contact.ParticleIndex))
			{
				continue;
			}
			const FVector Position(Particles[Contact.ParticleIndex].Position);
			if (HasFlag(ECableSimDebugDraw::Contacts))
			{
				Proxy->ArrowLines.Emplace(Position, Position + FVector(Contact.Normal) * 16.0,
					NormalWhite, Thickness + 1.0f);
				Proxy->Spheres.Emplace(3.0f, Position,
					FLinearColor(Contact.bStaticAnchorHeld ? FrictionHeld : FrictionSlip),
					FDebugRenderSceneProxy::EDrawType::WireMesh);
				if (Contact.SurfaceVelocity.SizeSquared() > 1.e-6)
				{
					Proxy->ArrowLines.Emplace(Position, Position + FVector(Contact.SurfaceVelocity) * 0.1,
						SurfaceOrange, Thickness + 1.5f);
				}
			}
			if (HasFlag(ECableSimDebugDraw::Load))
			{
				const float LoadRatio = static_cast<float>(FMath::Clamp(Contact.EstimatedNormalLoad / MaxLoad, 0.0, 1.0));
				Proxy->Spheres.Emplace(3.0f + 5.0f * LoadRatio, Position,
					FLinearColor(Ramp(LoadRatio)), FDebugRenderSceneProxy::EDrawType::WireMesh);
			}
		}
	}

	// Guide corridor: the leash volume each dynamic node is held within.
	if (bTautActive && HasFlag(ECableSimDebugDraw::GuideCorridor))
	{
		for (const CableSim::FGuideConstraint& Guide : RuntimeState->LastGuides)
		{
			if (Particles.IsValidIndex(Guide.ParticleIndex) && Guide.MaximumDistance > 0.0)
			{
				Proxy->Spheres.Emplace(static_cast<float>(Guide.MaximumDistance),
					FVector(Guide.TargetPosition), FLinearColor(CorridorBlue),
					FDebugRenderSceneProxy::EDrawType::WireMesh);
			}
		}
	}

	if (bTautActive && HasFlag(ECableSimDebugDraw::Snapshot) && RuntimeState->ChaosSnapshot.QueryBounds.IsValid)
	{
		Proxy->Boxes.Emplace(RuntimeState->ChaosSnapshot.QueryBounds, FColor(142, 124, 195),
			FDebugRenderSceneProxy::EDrawType::WireMesh, Thickness);
	}

	// Gathered collision edges, coloured by kind; moving surfaces tinted orange.
	if (bTautActive && HasFlag(ECableSimDebugDraw::Topology))
	{
		for (const CableSim::FCollisionEdge& Edge : RuntimeState->ChaosSnapshot.Edges)
		{
			const FColor EdgeColor = Edge.SurfaceVelocity.SizeSquared() > 1.e-6
				? SurfaceOrange : GetEdgeDebugColor(Edge.Kind);
			Proxy->Lines.Emplace(FVector(Edge.Start), FVector(Edge.End), EdgeColor, Thickness);
		}
	}

	const TArray<CableSim::FTautPoint>& TautPoints = RuntimeState->TautSolver.GetPoints();
	if (bTautActive && HasFlag(ECableSimDebugDraw::TautPath))
	{
		const bool bPathValid = RuntimeState->TautDiagnostics.Status == ECableSimTautStatus::Ready
			|| RuntimeState->TautDiagnostics.Status == ECableSimTautStatus::NoRelevantGeometry;
		const FColor PathColor = bPathValid ? TautViolet : WarnRed;
		for (int32 Index = 0; Index + 1 < TautPoints.Num(); ++Index)
		{
			Proxy->Lines.Emplace(FVector(TautPoints[Index].Position),
				FVector(TautPoints[Index + 1].Position), PathColor, Thickness + 2.0f);
		}
		for (const CableSim::FTautPoint& Point : TautPoints)
		{
			if (Point.Type != CableSim::ETautPointType::Endpoint)
			{
				Proxy->Spheres.Emplace(4.0f, FVector(Point.Position),
					FLinearColor(Point.Type == CableSim::ETautPointType::VertexContact
						? WarnRed : EdgeContactYellow),
					FDebugRenderSceneProxy::EDrawType::WireMesh);
			}
		}
	}

	// Colour-coded HUD stacked above the start endpoint.
	if (HasFlag(ECableSimDebugDraw::Status))
	{
		const FCableSimStatus Status = GetSimulationStatus();
		const FVector Base(Particles[0].Position);
		const FColor Ok(76, 195, 138);
		const FColor Warn(245, 166, 35);
		const FColor Bad(225, 45, 57);
		auto Line = [&Proxy, &Base](const int32 Row, const FString& Text, const FColor& Color)
		{
			Proxy->Texts.Emplace(Text, Base + FVector(0.0, 0.0, 28.0 - Row * 7.0), FLinearColor(Color));
		};
		const FColor SimColor = Status.Status == ECableSimSimulationStatus::Ready ? Ok
			: (Status.Status == ECableSimSimulationStatus::Overextended ? Warn : Bad);
		Line(0, FString::Printf(TEXT("Sim %s  step %lld  nodes %d  iters %d"),
			*StaticEnum<ECableSimSimulationStatus>()->GetNameStringByValue(static_cast<int64>(Status.Status)),
			Status.StepIndex, Status.ParticleCount, Result.ConstraintIterations), SimColor);
		Line(1, FString::Printf(TEXT("seg %.2f  pen %.2f  spd %.0f"),
			Status.MaximumSegmentError, Status.MaximumPenetration, Status.MaximumParticleSpeed),
			Status.MaximumPenetration > 1.0 ? Bad : (Status.MaximumSegmentError > 1.0 ? Warn : NormalWhite));
		Line(2, FString::Printf(TEXT("tension %.1f  load %.1f"),
			Status.MaximumEstimatedTension, Status.MaximumEstimatedNormalLoad), NormalWhite);
		if (bTautActive)
		{
			Line(3, FString::Printf(TEXT("reach: strain %.0f%%  path %.0f / %.0f"),
				Status.StrainRatio * 100.0, RuntimeState->TautDiagnostics.PathLength, Status.RestLength),
				bOverextended ? Bad : Ok);
			const double TautMilliseconds = RuntimeState->TautDiagnostics.SnapshotMilliseconds
				+ RuntimeState->TautDiagnostics.TopologyCompileMilliseconds
				+ RuntimeState->TautDiagnostics.SolverMilliseconds;
			Line(4, FString::Printf(TEXT("taut %s  contacts %d  %.2f ms"),
				*StaticEnum<ECableSimTautStatus>()->GetNameStringByValue(
					static_cast<int64>(RuntimeState->TautDiagnostics.Status)),
				RuntimeState->TautDiagnostics.ContactCount, TautMilliseconds),
				RuntimeState->TautDiagnostics.Status == ECableSimTautStatus::Ready ? Ok : Warn);
		}
	}
	return Proxy;
#endif
}
