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
		case CableSim::ETautStatus::NonManifoldTopology:
			return ECableSimTautStatus::NonManifoldTopology;
		case CableSim::ETautStatus::TopologyOverValence:
			return ECableSimTautStatus::TopologyOverValence;
		case CableSim::ETautStatus::FeatureInvalidated:
			return ECableSimTautStatus::FeatureInvalidated;
		case CableSim::ETautStatus::IterationBudgetExceeded:
			return ECableSimTautStatus::IterationBudgetExceeded;
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

	uint32 GetCollisionSignature(const FCableSimCollisionSettings& Settings)
	{
		uint32 Hash = GetTypeHash(Settings.bEnableWorldCollision);
		Hash = HashCombineFast(Hash, GetTypeHash(Settings.Radius));
		Hash = HashCombineFast(Hash, GetTypeHash(Settings.SkinWidth));
		Hash = HashCombineFast(Hash, GetTypeHash(Settings.MaximumContactsPerParticle));
		Hash = HashCombineFast(Hash, GetTypeHash(Settings.ContactReleaseDistance));
		Hash = HashCombineFast(Hash, GetTypeHash(Settings.ContactNormalToleranceDegrees));
		Hash = HashCombineFast(Hash, GetTypeHash(Settings.ContactPersistenceSteps));
		return HashCombineFast(Hash, GetTypeHash(static_cast<uint8>(Settings.Channel.GetValue())));
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
	TArray<CableSim::FCollisionEdge> SupportedTautEdges;
	FCableSimTautDiagnostics TautDiagnostics;
	FCableSimEndpointConstraint EndpointConstraints[2];
	TArray<FVector3d> PreviousSolvedPositions;
	TArray<FVector3d> CurrentSolvedPositions;
	double AccumulatedTime = 0.0;
	double EndpointSampleElapsedTime = 0.0;
	double DroppedSimulationTime = 0.0;
	double RenderInterpolationAlpha = 1.0;
	FVector3d EditorPreviewEndpoints[2] = {FVector3d::ZeroVector, FVector3d::ZeroVector};
	bool bHasEditorPreview = false;
	FVector3d LastAcceptedReachablePositions[2] = {FVector3d::ZeroVector, FVector3d::ZeroVector};
	bool bHasAcceptedReachablePosition[2] = {false, false};
	bool bConfigPending = false;
	bool bTautWasEnabled = false;
	uint32 AppliedCollisionSignature = 0;
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
	RuntimeState->SupportedTautEdges.Reset();
	RuntimeState->TautDiagnostics = FCableSimTautDiagnostics{};
	RuntimeState->bTautWasEnabled = TautSettings.bEnableTautSolver;
	RuntimeState->AppliedCollisionSignature = GetCollisionSignature(CollisionSettings);

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
	for (int32 EndpointIndex = 0; EndpointIndex < 2; ++EndpointIndex)
	{
		FCableSimEndpointConstraint& Constraint = RuntimeState->EndpointConstraints[EndpointIndex];
		Constraint = FCableSimEndpointConstraint{};
		Constraint.Endpoint = EndpointIndex == 0 ? ECableSimEndpoint::Start : ECableSimEndpoint::End;
		Constraint.Status = TautSettings.bEnableTautSolver
			? ECableSimEndpointConstraintStatus::UnsupportedEndpointConfiguration
			: ECableSimEndpointConstraintStatus::Disabled;
	}
	StartEndpointConstraint = RuntimeState->EndpointConstraints[0];
	EndEndpointConstraint = RuntimeState->EndpointConstraints[1];
	if (TautSettings.bEnableTautSolver)
	{
		RuntimeState->AppliedTautConfig.TopologyTolerance = FMath::Max(TautSettings.TopologyTolerance, 0.001);
		RuntimeState->AppliedTautConfig.MaximumCollisionPasses = FMath::Clamp(TautSettings.MaximumCollisionPasses, 2, 64);
		RuntimeState->TautSolver.Initialize(StartPosition, EndPosition, RuntimeState->AppliedTautConfig);
		RuntimeState->TautDiagnostics.Status = ECableSimTautStatus::Uninitialized;
	}

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
		RuntimeState->LastAcceptedReachablePositions[Index] = Position;
		RuntimeState->bHasAcceptedReachablePosition[Index] = true;
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
	Status.EffectiveSolveLength = Result.EffectiveSolveLength;
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

FCableSimEndpointConstraint UCableSimComponent::GetEndpointConstraint(
	const ECableSimEndpoint Endpoint) const
{
	return Endpoint == ECableSimEndpoint::Start ? StartEndpointConstraint : EndEndpointConstraint;
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
	const uint32 CollisionSignature = GetCollisionSignature(CollisionSettings);
	if (CollisionSignature != RuntimeState->AppliedCollisionSignature)
	{
		RuntimeState->AppliedCollisionSignature = CollisionSignature;
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
	if (!CollisionSettings.bEnableWorldCollision && !TautSettings.bEnableTautSolver)
	{
		RuntimeState->ChaosSnapshot.Reset();
		return;
	}
	const double Tolerance = FMath::Max(TautSettings.TopologyTolerance, 0.001);
	RuntimeState->bSnapshotValid = FCableSimChaosCollisionAdapter::GatherSnapshot(
		GetWorld(),
		GetOwner(),
		IgnoredActors,
		CollisionSettings,
		Tolerance,
		Solver.GetParticles(),
		Input.StartEndpoint.TargetPosition,
		Input.EndEndpoint.TargetPosition,
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
	StartEndpointConstraint = RuntimeState->EndpointConstraints[0];
	EndEndpointConstraint = RuntimeState->EndpointConstraints[1];

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
	if (!TautSettings.bEnableTautSolver)
	{
		if (RuntimeState->bTautWasEnabled)
		{
			RuntimeState->TautSolver.Reset();
			RuntimeState->ChaosObjectTracker.Reset();
			RuntimeState->ChaosSnapshot.Reset();
			RuntimeState->SupportedTautEdges.Reset();
		}
		RuntimeState->bTautWasEnabled = false;
		RuntimeState->TautDiagnostics = FCableSimTautDiagnostics{};
		for (FCableSimEndpointConstraint& Constraint : RuntimeState->EndpointConstraints)
		{
			Constraint = FCableSimEndpointConstraint{};
			Constraint.Status = ECableSimEndpointConstraintStatus::Disabled;
		}
		return;
	}

	const int32 ConstrainedIndex = TautSettings.ConstrainedEndpoint == ECableSimEndpoint::Start ? 0 : 1;
	const int32 AnchorIndex = 1 - ConstrainedIndex;
	const FCableSimEndpointBinding& ConstrainedBinding = ConstrainedIndex == 0 ? StartEndpoint : EndEndpoint;
	const FCableSimEndpointBinding& AnchorBinding = AnchorIndex == 0 ? StartEndpoint : EndEndpoint;
	const bool bSupportedEndpointConfiguration =
		ConstrainedBinding.Mode != ECableSimEndpointMode::Simulated
		&& AnchorBinding.Mode != ECableSimEndpointMode::Simulated;

	for (int32 EndpointIndex = 0; EndpointIndex < 2; ++EndpointIndex)
	{
		FCableSimEndpointConstraint& Constraint = RuntimeState->EndpointConstraints[EndpointIndex];
		Constraint = FCableSimEndpointConstraint{};
		Constraint.Endpoint = EndpointIndex == 0 ? ECableSimEndpoint::Start : ECableSimEndpoint::End;
		Constraint.Status = ECableSimEndpointConstraintStatus::UnsupportedEndpointConfiguration;
		Constraint.RequestedWorldPosition = FVector(EndpointIndex == 0
			? Input.StartEndpoint.TargetPosition
			: Input.EndEndpoint.TargetPosition);
		Constraint.ReachableWorldPosition = Constraint.RequestedWorldPosition;
		Constraint.RestLength = SimulationSettings.RestLength;
	}
	if (bSupportedEndpointConfiguration)
	{
		RuntimeState->EndpointConstraints[AnchorIndex].Status = ECableSimEndpointConstraintStatus::Disabled;
		if (!TautSettings.bEnableReachConstraint)
		{
			RuntimeState->EndpointConstraints[ConstrainedIndex].Status = ECableSimEndpointConstraintStatus::Disabled;
		}
	}

	auto ApplyConstrainedPosition = [this, &Input, ConstrainedIndex](const FVector3d& Position)
	{
		CableSim::FEndpointStepInput& EndpointInput = ConstrainedIndex == 0
			? Input.StartEndpoint
			: Input.EndEndpoint;
		const int32 ParticleIndex = ConstrainedIndex == 0 ? 0 : Solver.GetParticles().Num() - 1;
		EndpointInput.TargetPosition = Position;
		EndpointInput.TargetVelocity = (Position - Solver.GetParticles()[ParticleIndex].Position) / Input.DeltaTime;
	};
	auto FreezeAtLastAcceptedPosition = [this, &ApplyConstrainedPosition, ConstrainedIndex]()
	{
		FCableSimEndpointConstraint& Constraint = RuntimeState->EndpointConstraints[ConstrainedIndex];
		Constraint.Status = ECableSimEndpointConstraintStatus::TautPathUnavailable;
		if (!RuntimeState->bHasAcceptedReachablePosition[ConstrainedIndex])
		{
			return;
		}
		const FVector3d FrozenPosition = RuntimeState->LastAcceptedReachablePositions[ConstrainedIndex];
		Constraint.ReachableWorldPosition = FVector(FrozenPosition);
		Constraint.CorrectionWorld = Constraint.ReachableWorldPosition - Constraint.RequestedWorldPosition;
		Constraint.ConstraintDirection = Constraint.CorrectionWorld.GetSafeNormal();
		Constraint.bLimited = !Constraint.CorrectionWorld.IsNearlyZero();
		ApplyConstrainedPosition(FrozenPosition);
	};

	CableSim::FTautConfig TautConfig;
	TautConfig.TopologyTolerance = FMath::Max(TautSettings.TopologyTolerance, 0.001);
	TautConfig.MaximumCollisionPasses = FMath::Clamp(TautSettings.MaximumCollisionPasses, 2, 64);
	const bool bConfigChanged = !FMath::IsNearlyEqual(
		TautConfig.TopologyTolerance,
		RuntimeState->AppliedTautConfig.TopologyTolerance,
		1.e-9)
		|| TautConfig.MaximumCollisionPasses != RuntimeState->AppliedTautConfig.MaximumCollisionPasses;
	if (!RuntimeState->bTautWasEnabled || !RuntimeState->TautSolver.IsInitialized() || bConfigChanged)
	{
		const TArray<CableSim::FParticle>& Particles = Solver.GetParticles();
		if (Particles.Num() < 2 || !RuntimeState->TautSolver.Initialize(
			Particles[0].Position,
			Particles.Last().Position,
			TautConfig))
		{
			RuntimeState->TautDiagnostics = FCableSimTautDiagnostics{};
			RuntimeState->TautDiagnostics.Status = ECableSimTautStatus::InvalidConfiguration;
			if (bSupportedEndpointConfiguration && TautSettings.bEnableReachConstraint)
			{
				FreezeAtLastAcceptedPosition();
			}
			return;
		}
		RuntimeState->AppliedTautConfig = TautConfig;
	}
	RuntimeState->bTautWasEnabled = true;

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
	RuntimeState->SupportedTautEdges.Reset();
	if (!bSnapshotSucceeded)
	{
		Diagnostics.Status = ECableSimTautStatus::SnapshotFailed;
		RuntimeState->TautDiagnostics = Diagnostics;
		if (bSupportedEndpointConfiguration && TautSettings.bEnableReachConstraint)
		{
			FreezeAtLastAcceptedPosition();
		}
		return;
	}

	for (const CableSim::FCollisionEdge& Edge : RuntimeState->ChaosSnapshot.Edges)
	{
		const bool bSupportedGeometry = Edge.GeometryType == CableSim::ECollisionGeometryType::Box
			|| Edge.GeometryType == CableSim::ECollisionGeometryType::Convex;
		if (Edge.bStaticObject && bSupportedGeometry)
		{
			RuntimeState->SupportedTautEdges.Add(Edge);
		}
	}
	Diagnostics.SupportedEdgeCount = RuntimeState->SupportedTautEdges.Num();
	CableSim::FTautStepInput TautInput;
	TautInput.StartTarget = Input.StartEndpoint.TargetPosition;
	TautInput.EndTarget = Input.EndEndpoint.TargetPosition;
	const double SolverStart = FPlatformTime::Seconds();
	CableSim::FTautStepResult Result = RuntimeState->TautSolver.AdvanceStep(
		TautInput,
		RuntimeState->SupportedTautEdges);

	for (FCableSimEndpointConstraint& Constraint : RuntimeState->EndpointConstraints)
	{
		Constraint.PathLength = Result.PathLength;
		Constraint.ExcessDistance = FMath::Max(Result.PathLength - SimulationSettings.RestLength, 0.0);
	}

	const bool bSupportedResult = Result.Status == CableSim::ETautStatus::Ready
		|| Result.Status == CableSim::ETautStatus::NoRelevantGeometry;
	if (bSupportedEndpointConfiguration && TautSettings.bEnableReachConstraint)
	{
		FCableSimEndpointConstraint& Constraint = RuntimeState->EndpointConstraints[ConstrainedIndex];
		if (!bSupportedResult)
		{
			FreezeAtLastAcceptedPosition();
		}
		else if (Result.PathLength <= SimulationSettings.RestLength + TautSettings.LengthTolerance)
		{
			Constraint.Status = ECableSimEndpointConstraintStatus::ValidSlack;
			RuntimeState->LastAcceptedReachablePositions[ConstrainedIndex] = ConstrainedIndex == 0
				? TautInput.StartTarget
				: TautInput.EndTarget;
			RuntimeState->bHasAcceptedReachablePosition[ConstrainedIndex] = true;
		}
		else
		{
			FVector3d ReachablePoint;
			double ExcessDistance = 0.0;
			if (!RuntimeState->TautSolver.CalculateReachableEndpoint(
				ConstrainedIndex == 0,
				SimulationSettings.RestLength,
				ReachablePoint,
				ExcessDistance))
			{
				FreezeAtLastAcceptedPosition();
			}
			else
			{
				const FVector3d RequestedPoint = ConstrainedIndex == 0
					? TautInput.StartTarget
					: TautInput.EndTarget;
				Constraint.Status = ECableSimEndpointConstraintStatus::Limited;
				Constraint.bLimited = true;
				Constraint.ExcessDistance = ExcessDistance;
				Constraint.ReachableWorldPosition = FVector(ReachablePoint);
				Constraint.CorrectionWorld = FVector(ReachablePoint - RequestedPoint);
				Constraint.ConstraintDirection = Constraint.CorrectionWorld.GetSafeNormal();
				ApplyConstrainedPosition(ReachablePoint);
				(ConstrainedIndex == 0 ? TautInput.StartTarget : TautInput.EndTarget) = ReachablePoint;
				RuntimeState->LastAcceptedReachablePositions[ConstrainedIndex] = ReachablePoint;
				RuntimeState->bHasAcceptedReachablePosition[ConstrainedIndex] = true;
				Result = RuntimeState->TautSolver.AdvanceStep(TautInput, RuntimeState->SupportedTautEdges);
				Constraint.PathLength = Result.PathLength;
			}
		}
	}

	BuildTautGuideConstraints(Input);
	Diagnostics.SolverMilliseconds = (FPlatformTime::Seconds() - SolverStart) * 1000.0;
	Diagnostics.Status = ToRuntimeTautStatus(Result.Status);
	Diagnostics.StepIndex = static_cast<int64>(Result.StepIndex);
	Diagnostics.PointCount = Result.PointCount;
	Diagnostics.ContactCount = Result.ContactCount;
	Diagnostics.PathLength = Result.PathLength;
	RuntimeState->TautDiagnostics = Diagnostics;
}

void UCableSimComponent::BuildTautGuideConstraints(CableSim::FStepInput& Input) const
{
	Input.GuideConstraints.Reset();
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
	const double BaseRadius = FMath::Min(
		Slack * FMath::Max(TautSettings.GuideSlackScale, 0.0),
		FMath::Max(TautSettings.MaximumGuideRadius, 0.0));
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
		Guide.MaximumDistance = FMath::Max(
			BaseRadius * FMath::Sin(UE_PI * Alpha),
			FMath::Max(TautSettings.MinimumGuideRadius, 0.0));
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
	if (HasFlag(ECableSimDebugDraw::Particles))
	{
		for (int32 Index = 0; Index < RenderPositions.Num(); ++Index)
		{
			const FLinearColor Color = Particles[Index].Mode == CableSim::EParticleMode::Kinematic
				? FLinearColor(FColor::Orange)
				: FLinearColor::White;
			Proxy->Spheres.Emplace(
				3.0f,
				RenderPositions[Index],
				Color,
				FDebugRenderSceneProxy::EDrawType::SolidAndWireMeshes);
		}
	}
	if (HasFlag(ECableSimDebugDraw::ActiveContacts) || HasFlag(ECableSimDebugDraw::Friction))
	{
		const double MaximumLoad = FMath::Max(Solver.GetLastStepResult().MaximumEstimatedNormalLoad, 1.e-6);
		for (const CableSim::FContactDiagnostic& Diagnostic : Solver.GetLastContactDiagnostics())
		{
			if (!Diagnostic.bProjected || !Particles.IsValidIndex(Diagnostic.ParticleIndex))
			{
				continue;
			}
			const FVector Position(Particles[Diagnostic.ParticleIndex].Position);
			if (HasFlag(ECableSimDebugDraw::ActiveContacts))
			{
				Proxy->ArrowLines.Emplace(Position, Position + FVector(Diagnostic.Normal) * 18.0, FColor::Yellow, 5.0f);
			}
			if (HasFlag(ECableSimDebugDraw::Friction))
			{
				const float LoadRatio = static_cast<float>(FMath::Clamp(Diagnostic.EstimatedNormalLoad / MaximumLoad, 0.0, 1.0));
				const FColor HeatColor = FColor::MakeRedToGreenColorFromScalar(1.0f - LoadRatio);
				Proxy->Spheres.Emplace(3.0f + 4.0f * LoadRatio, Position, FLinearColor(HeatColor), FDebugRenderSceneProxy::EDrawType::WireMesh);
			}
		}
	}

	if (!TautSettings.bEnableTautSolver)
	{
		return Proxy;
	}
	if (HasFlag(ECableSimDebugDraw::SnapshotBounds) && RuntimeState->ChaosSnapshot.QueryBounds.IsValid)
	{
		Proxy->Boxes.Emplace(
			RuntimeState->ChaosSnapshot.QueryBounds,
			FColor(180, 60, 255),
			FDebugRenderSceneProxy::EDrawType::WireMesh,
			DebugSettings.LineThickness);
	}
	if (HasFlag(ECableSimDebugDraw::CandidateTopology))
	{
		for (const CableSim::FCollisionEdge& Edge : RuntimeState->ChaosSnapshot.Edges)
		{
			Proxy->Lines.Emplace(FVector(Edge.Start), FVector(Edge.End), GetEdgeDebugColor(Edge.Kind), DebugSettings.LineThickness);
		}
	}
	const TArray<CableSim::FTautPoint>& TautPoints = RuntimeState->TautSolver.GetPoints();
	if (HasFlag(ECableSimDebugDraw::TautPathAndGuide))
	{
		const bool bHasContact = TautPoints.ContainsByPredicate([](const CableSim::FTautPoint& Point)
		{
			return Point.Type == CableSim::ETautPointType::EdgeContact;
		});
		const bool bPathValid = RuntimeState->TautDiagnostics.Status == ECableSimTautStatus::Ready
			|| RuntimeState->TautDiagnostics.Status == ECableSimTautStatus::NoRelevantGeometry;
		const FColor PathColor = !bPathValid
			? FColor(255, 96, 32)
			: (bHasContact ? FColor(255, 0, 255) : FColor(80, 200, 120));
		for (int32 Index = 0; Index + 1 < TautPoints.Num(); ++Index)
		{
			Proxy->Lines.Emplace(
				FVector(TautPoints[Index].Position),
				FVector(TautPoints[Index + 1].Position),
				PathColor,
				DebugSettings.LineThickness + 1.0f);
		}
		for (const CableSim::FTautPoint& Point : TautPoints)
		{
			if (Point.Type == CableSim::ETautPointType::EdgeContact)
			{
				Proxy->Spheres.Emplace(4.0f, FVector(Point.Position), FLinearColor::Yellow, FDebugRenderSceneProxy::EDrawType::WireMesh);
			}
		}
	}
	if (HasFlag(ECableSimDebugDraw::StatusAndReach))
	{
		const FString StatusName = StaticEnum<ECableSimTautStatus>()->GetNameStringByValue(static_cast<int64>(RuntimeState->TautDiagnostics.Status));
		Proxy->Texts.Emplace(
			FString::Printf(
				TEXT("Taut: %s | tris %d | verts %d | edges %d/%d | contacts %d | %.2f ms"),
				*StatusName,
				RuntimeState->TautDiagnostics.TriangleCount,
				RuntimeState->TautDiagnostics.ExtractedVertexCount,
				RuntimeState->TautDiagnostics.SupportedEdgeCount,
				RuntimeState->TautDiagnostics.ExtractedEdgeCount,
				RuntimeState->TautDiagnostics.ContactCount,
				RuntimeState->TautDiagnostics.SnapshotMilliseconds
					+ RuntimeState->TautDiagnostics.TopologyCompileMilliseconds
					+ RuntimeState->TautDiagnostics.SolverMilliseconds),
			FVector(Particles[0].Position) + FVector(0.0, 0.0, 20.0),
			FLinearColor::White);
		const int32 ReachIndex = TautSettings.ConstrainedEndpoint == ECableSimEndpoint::Start ? 0 : 1;
		const FCableSimEndpointConstraint& ReachConstraint = RuntimeState->EndpointConstraints[ReachIndex];
		const FString ReachStatusName = StaticEnum<ECableSimEndpointConstraintStatus>()->GetNameStringByValue(static_cast<int64>(ReachConstraint.Status));
		Proxy->Texts.Emplace(
			FString::Printf(
				TEXT("Reach: %s | path %.1f / %.1f cm | excess %.1f cm"),
				*ReachStatusName,
				ReachConstraint.PathLength,
				ReachConstraint.RestLength,
				ReachConstraint.ExcessDistance),
			FVector(Particles[0].Position) + FVector(0.0, 0.0, 36.0),
			ReachConstraint.bLimited ? FLinearColor::Yellow : FLinearColor(FColor::Silver));
		for (const FCableSimEndpointConstraint& Constraint : RuntimeState->EndpointConstraints)
		{
			if (Constraint.bLimited)
			{
				Proxy->Lines.Emplace(
					Constraint.RequestedWorldPosition,
					Constraint.ReachableWorldPosition,
					FColor::Yellow,
					DebugSettings.LineThickness + 1.0f);
			}
		}
	}
	return Proxy;
#endif
}
