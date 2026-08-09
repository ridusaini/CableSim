#pragma once

#include "CableSimContactManifold.h"
#include "CableSimSolver.h"
#include "CableSimTautSolver.h"
#include "Debug/DebugDrawComponent.h"
#include "CableSimComponent.generated.h"

UENUM(BlueprintType)
enum class ECableSimEndpointMode : uint8
{
	Simulated,
	CableLocalKinematic,
	WorldKinematic,
	ComponentKinematic
};

UENUM(BlueprintType)
enum class ECableSimEndpointBindingStatus : uint8
{
	Ready,
	Simulated,
	MissingComponent,
	MissingSocket
};

UENUM(BlueprintType)
enum class ECableSimEndpoint : uint8
{
	Start,
	End
};

UENUM(BlueprintType)
enum class ECableSimSimulationStatus : uint8
{
	Uninitialized,
	Ready,
	Overextended,
	NumericalFailure,
	InvalidConfiguration
};

UENUM(BlueprintType)
enum class ECableSimTautMode : uint8
{
	Disabled,
	Shadow,
	Enabled
};

UENUM(BlueprintType)
enum class ECableSimTautStatus : uint8
{
	Disabled,
	Inactive,
	Uninitialized,
	Ready,
	NoRelevantGeometry,
	InvalidSeed,
	FeatureUnavailable,
	PathBlocked,
	UnsupportedTopology,
	NonManifoldTopology,
	TopologyOverValence,
	TJunctionTopology,
	OverlappingTopology,
	CollisionBudgetExceeded,
	TopologyBudgetExceeded,
	SnapshotFailed,
	InvalidConfiguration,
	NumericalFailure
};

UENUM(BlueprintType)
enum class ECableSimTautEndpointRole : uint8
{
	// This endpoint target is authoritative. Reach projection never moves it.
	Fixed,
	// This endpoint follows its requested target while feasible and participates
	// in weighted reach projection when the rope is fully extended.
	Driven,
	// The taut solve follows the dynamic endpoint particle and never drives it.
	Free
};

USTRUCT(BlueprintType)
struct CABLESIMRUNTIME_API FCableSimTautEndpointPolicy
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	ECableSimTautEndpointRole Role = ECableSimTautEndpointRole::Driven;

	// Higher weight preserves more of this endpoint's requested movement when
	// both endpoints are driven and the full request is infeasible.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.001", EditCondition = "Role == ECableSimTautEndpointRole::Driven"))
	double ReachWeight = 1.0;
};

USTRUCT(BlueprintType)
struct CABLESIMRUNTIME_API FCableSimEndpointBinding
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	ECableSimEndpointMode Mode = ECableSimEndpointMode::CableLocalKinematic;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	FVector SpawnLocalPosition = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (EditCondition = "Mode == ECableSimEndpointMode::CableLocalKinematic"))
	FVector LocalTarget = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (EditCondition = "Mode == ECableSimEndpointMode::WorldKinematic"))
	FVector WorldTarget = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (EditCondition = "Mode == ECableSimEndpointMode::ComponentKinematic"))
	TObjectPtr<USceneComponent> TargetComponent = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (EditCondition = "Mode == ECableSimEndpointMode::ComponentKinematic"))
	FName SocketName = NAME_None;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (EditCondition = "Mode == ECableSimEndpointMode::ComponentKinematic"))
	FVector ComponentLocalOffset = FVector::ZeroVector;
};

USTRUCT(BlueprintType)
struct CABLESIMRUNTIME_API FCableSimSimulationSettings
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.01", Units = "cm"))
	double RestLength = 400.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.01", Units = "cm"))
	double NodeSpacing = 10.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.000001", Units = "kg"))
	double ParticleMass = 0.05;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	FVector Gravity = FVector(0.0, 0.0, -980.665);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	double VelocityDamping = 0.01;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "1.0", ClampMax = "1.05"))
	double DistanceOverRelaxation = 1.015;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0", UIMax = "256"))
	int32 ConstraintIterations = 64;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0001", Units = "s"))
	double FixedTimeStep = 1.0 / 60.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "1", ClampMax = "16"))
	int32 MaximumSubsteps = 4;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	bool bSimulationEnabled = true;
};

USTRUCT(BlueprintType)
struct CABLESIMRUNTIME_API FCableSimBendingSettings
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	double StepStiffness = 0.20;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0", Units = "deg"))
	double FreeAngleDegreesPerMeter = 90.0;
};

USTRUCT(BlueprintType)
struct CABLESIMRUNTIME_API FCableSimCollisionSettings
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	bool bEnableWorldCollision = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.1", Units = "cm"))
	double Radius = 5.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0", Units = "cm"))
	double SkinWidth = 0.5;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "1", ClampMax = "4"))
	int32 MaximumContactsPerParticle = 4;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0", Units = "cm"))
	double ContactReleaseDistance = 1.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0", ClampMax = "90.0", Units = "deg"))
	double ContactNormalToleranceDegrees = 15.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	TEnumAsByte<ECollisionChannel> Channel = ECC_WorldStatic;
};

USTRUCT(BlueprintType)
struct CABLESIMRUNTIME_API FCableSimFrictionSettings
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	bool bEnableFriction = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0"))
	double StaticFrictionCoefficient = 0.35;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0"))
	double DynamicFrictionCoefficient = 0.25;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0", Units = "cm/s"))
	double StaticSpeedThreshold = 2.0;
};

USTRUCT(BlueprintType)
struct CABLESIMRUNTIME_API FCableSimTautSettings
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	ECableSimTautMode Mode = ECableSimTautMode::Disabled;

	// Convenience ownership lifecycle: a controlled Driven endpoint engages taut;
	// releasing the final Driven endpoint disengages it. Fixed anchors do not count.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (EditCondition = "Mode != ECableSimTautMode::Disabled"))
	bool bAutoManageOwnershipFromEndpoints = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (EditCondition = "Mode != ECableSimTautMode::Disabled"))
	FCableSimTautEndpointPolicy StartEndpoint;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (EditCondition = "Mode != ECableSimTautMode::Disabled"))
	FCableSimTautEndpointPolicy EndEndpoint;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.001", Units = "cm"))
	double TopologyTolerance = 0.1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0001", Units = "cm"))
	double MovementConvergenceTolerance = 0.01;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "1", ClampMax = "128"))
	int32 MaximumMovementIterations = 32;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "1", ClampMax = "128"))
	int32 MaximumCollisionPhases = 32;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0", Units = "cm"))
	double LengthTolerance = 0.1;

	// Ordered continuation finds the first infeasible interval instead of assuming
	// reach acceptance is globally monotone across taut topology changes.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "2", ClampMax = "32", EditCondition = "Mode == ECableSimTautMode::Enabled"))
	int32 ReachContinuationSteps = 8;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "1", ClampMax = "16", EditCondition = "Mode == ECableSimTautMode::Enabled"))
	int32 ReachRefinementIterations = 8;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	double GuideActivationPathRatio = 0.80;

	// Scales the parabolic-catenary sag envelope BaseRadius = GuideSagScale *
	// sqrt(3 * PathLength * Slack / 8): how far the corridor lets a dynamic node
	// stray from the taut path, sized to match a real rope's natural sag rather
	// than a flat fraction of slack.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0"))
	double GuideSagScale = 1.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0", Units = "cm"))
	double MaximumGuideRadius = 100.0;

	// A resting cable must be free to follow the surface, so the corridor never
	// collapses below this radius: below it the leash exerts no pull, which keeps
	// a near-taut cable from being pinned to a jittering geodesic sample.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0", Units = "cm"))
	double MinimumGuideRadius = 5.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	double GuideStepStrength = 0.85;

	// How much of the way the guide corridor closes toward its target width each step
	// (1 = instant, which snaps a resting cable onto the taut line; lower eases it in).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0", ClampMax = "1.0", EditCondition = "Mode == ECableSimTautMode::Enabled"))
	double GuideCloseFraction = 0.15;
};

UENUM(BlueprintType, meta = (Bitflags))
enum class ECableSimDebugDraw : uint8
{
	Cable = 0 UMETA(DisplayName = "Cable (tension)"),
	Contacts = 1 UMETA(DisplayName = "Contacts / friction"),
	Load = 2 UMETA(DisplayName = "Normal load"),
	TautPath = 3 UMETA(DisplayName = "Taut path / reach"),
	GuideCorridor = 4 UMETA(DisplayName = "Guide corridor"),
	Snapshot = 5 UMETA(DisplayName = "Snapshot bounds"),
	Topology = 6 UMETA(DisplayName = "Collision topology"),
	Status = 7 UMETA(DisplayName = "Status HUD")
};

USTRUCT(BlueprintType)
struct CABLESIMRUNTIME_API FCableSimPreviewSettings
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	bool bVisible = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	FLinearColor Color = FLinearColor(0.0f, 1.0f, 1.0f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0"))
	float LineThickness = 1.5f;
};

USTRUCT(BlueprintType)
struct CABLESIMRUNTIME_API FCableSimDebugSettings
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	bool bDraw = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (Bitmask, BitmaskEnum = "/Script/CableSimRuntime.ECableSimDebugDraw"))
	int32 DrawFlags = (1 << static_cast<uint8>(ECableSimDebugDraw::Cable))
		| (1 << static_cast<uint8>(ECableSimDebugDraw::Contacts))
		| (1 << static_cast<uint8>(ECableSimDebugDraw::TautPath))
		| (1 << static_cast<uint8>(ECableSimDebugDraw::Status));

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0"))
	float LineThickness = 1.5f;
};

USTRUCT(BlueprintType)
struct CABLESIMRUNTIME_API FCableSimTautDiagnostics
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	ECableSimTautStatus Status = ECableSimTautStatus::Disabled;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	int64 StepIndex = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	int32 PointCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	int32 ContactCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	int32 OverlapCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	int32 PhysicsObjectCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	int32 ShapeCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	int32 TriangleCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	int32 ExtractedEdgeCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	int32 ExtractedVertexCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	int32 OverValenceVertexCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	int32 SupportedEdgeCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	int64 TopologyRevision = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	int32 MovementIterationCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	int32 CollisionPhaseCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	int32 TopologyEventCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	int32 PairDecisionCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	int32 VertexDecisionCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	int32 StableVertexHoldCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	int32 SingularVertexHoldCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	int32 ConflictVertexHoldCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	int32 VertexMoveAlongCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	int32 VertexOuterSplitCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	int32 VertexReleaseCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	int32 OneSidedPairCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	bool bReachLimited = false;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	double AcceptedReachProgress = 1.0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut", meta = (Units = "cm"))
	double PathLength = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut", meta = (Units = "ms"))
	double SnapshotMilliseconds = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut", meta = (Units = "ms"))
	double TopologyCompileMilliseconds = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut", meta = (Units = "ms"))
	double SolverMilliseconds = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	bool bSnapshotSucceeded = false;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	bool bFeatureBudgetExceeded = false;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	bool bPathCollisionFree = false;
};

USTRUCT(BlueprintType)
struct CABLESIMRUNTIME_API FCableSimStatus
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "Cable")
	ECableSimSimulationStatus Status = ECableSimSimulationStatus::Uninitialized;

	UPROPERTY(BlueprintReadOnly, Category = "Cable")
	int64 StepIndex = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable")
	int32 ParticleCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable")
	int32 ContactCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable")
	int32 PersistedContactCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable")
	int32 ContactAdditionCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable")
	int32 ContactRemovalCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable")
	int32 GuideConstraintCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable")
	int32 ProjectedContactCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable")
	int32 StaticFrictionAnchorCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable")
	ECableSimEndpointBindingStatus StartEndpointStatus = ECableSimEndpointBindingStatus::Ready;

	UPROPERTY(BlueprintReadOnly, Category = "Cable")
	ECableSimEndpointBindingStatus EndEndpointStatus = ECableSimEndpointBindingStatus::Ready;

	UPROPERTY(BlueprintReadOnly, Category = "Cable", meta = (Units = "cm"))
	double RestLength = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable", meta = (Units = "cm"))
	double EndpointDistance = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable")
	double StrainRatio = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable", meta = (Units = "cm"))
	double MaximumSegmentError = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable", meta = (Units = "cm"))
	double MaximumPenetration = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable", meta = (Units = "cm"))
	double MaximumGuideError = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable", meta = (Units = "cm/s"))
	double MaximumParticleSpeed = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable", meta = (Units = "cm/s"))
	double RmsParticleSpeed = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable", meta = (Units = "N"))
	double MaximumEstimatedTension = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable", meta = (Units = "N"))
	double MaximumEstimatedNormalLoad = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable", meta = (Units = "s"))
	double DroppedSimulationTime = 0.0;
};

struct FCableSimRuntimeState;

UCLASS(ClassGroup = (Physics), meta = (BlueprintSpawnableComponent))
class CABLESIMRUNTIME_API UCableSimComponent : public UDebugDrawComponent
{
	GENERATED_BODY()

public:
	UCableSimComponent();
	virtual ~UCableSimComponent() override;

	virtual void OnRegister() override;
	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	virtual FBoxSphereBounds CalcBounds(const FTransform& LocalToWorld) const override;

#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable|Endpoints")
	FCableSimEndpointBinding StartEndpoint;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable|Endpoints")
	FCableSimEndpointBinding EndEndpoint;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable|Settings")
	FCableSimSimulationSettings SimulationSettings;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable|Settings")
	FCableSimBendingSettings BendingSettings;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable|Settings")
	FCableSimCollisionSettings CollisionSettings;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable|Settings")
	FCableSimFrictionSettings FrictionSettings;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable|Settings")
	FCableSimTautSettings TautSettings;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable|Settings")
	FCableSimPreviewSettings PreviewSettings;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable|Settings")
	FCableSimDebugSettings DebugSettings;

	UFUNCTION(BlueprintCallable, Category = "Cable|Simulation")
	void ReinitializeSimulation();

	UFUNCTION(BlueprintCallable, Category = "Cable|Simulation")
	void StepSimulation(int32 StepCount = 1);

	UFUNCTION(BlueprintCallable, Category = "Cable|Simulation")
	void SetRestLength(double NewRestLength);

	UFUNCTION(BlueprintCallable, Category = "Cable|Endpoints")
	void SetEndpointMode(ECableSimEndpoint Endpoint, ECableSimEndpointMode Mode);

	UFUNCTION(BlueprintCallable, Category = "Cable|Endpoints")
	void SetEndpointWorldTarget(ECableSimEndpoint Endpoint, FVector WorldPosition);

	UFUNCTION(BlueprintCallable, Category = "Cable|Endpoints")
	void SetEndpointLocalTarget(ECableSimEndpoint Endpoint, FVector LocalPosition);

	UFUNCTION(BlueprintCallable, Category = "Cable|Endpoints")
	void AttachEndpointToComponent(
		ECableSimEndpoint Endpoint,
		USceneComponent* TargetComponent,
		FName SocketName = NAME_None,
		FVector LocalOffset = FVector::ZeroVector);

	UFUNCTION(BlueprintCallable, Category = "Cable|Endpoints")
	void ReleaseEndpoint(ECableSimEndpoint Endpoint, bool bPreserveVelocity = true);

	UFUNCTION(BlueprintCallable, Category = "Cable|Endpoints")
	void TeleportEndpoint(ECableSimEndpoint Endpoint, FVector WorldPosition, bool bResetVelocity = true);

	UFUNCTION(BlueprintCallable, Category = "Cable|Simulation")
	void ResetSimulation();

	// Begin GDC-style taut ownership. The current dynamic cable polyline is consumed
	// as the seed on the next fixed step, once collision topology is available.
	UFUNCTION(BlueprintCallable, Category = "Cable|Taut")
	void EngageTautFromCable();

	// Return ownership to the dynamic cable without changing particles or velocities.
	UFUNCTION(BlueprintCallable, Category = "Cable|Taut")
	void DisengageTaut();

	UFUNCTION(BlueprintPure, Category = "Cable|Taut")
	bool IsTautEngaged() const;

	UFUNCTION(BlueprintPure, Category = "Cable|Simulation")
	TArray<FVector> GetSimulationPolyline() const;

	UFUNCTION(BlueprintPure, Category = "Cable|Simulation")
	TArray<FVector> GetRenderPolyline() const;

	UFUNCTION(BlueprintPure, Category = "Cable|Simulation")
	FCableSimStatus GetSimulationStatus() const;

	UFUNCTION(BlueprintPure, Category = "Cable|Simulation")
	int64 GetSimulationStepIndex() const;

	UFUNCTION(BlueprintPure, Category = "Cable|Taut")
	TArray<FVector> GetTautPolyline() const;

	UFUNCTION(BlueprintPure, Category = "Cable|Taut")
	FCableSimTautDiagnostics GetTautDiagnostics() const;

private:
	virtual FDebugRenderSceneProxy* CreateDebugSceneProxy() override;
	FCableSimEndpointBinding& GetBinding(ECableSimEndpoint Endpoint);
	const FCableSimEndpointBinding& GetBinding(ECableSimEndpoint Endpoint) const;
	CableSim::FSimulationConfig BuildCoreConfig() const;
	FVector3d ResolveInitialPosition(const FCableSimEndpointBinding& Binding, ECableSimEndpoint Endpoint) const;
	FVector3d ResolveTargetPosition(ECableSimEndpoint Endpoint) const;
	bool TryResolveTargetPosition(
		ECableSimEndpoint Endpoint,
		FVector3d& OutPosition,
		ECableSimEndpointBindingStatus& OutStatus) const;
	void SynchronizeConfiguration();
	void SynchronizeEndpointModes();
	bool HasControlledDrivenEndpoint() const;
	void ApplyAutomaticTautOwnershipTransition(bool bWasControlled, bool bIsControlled);
	void SampleEndpointTargets(double DeltaTime);
	CableSim::FStepInput BuildStepInput(double InterpolationAlpha) const;
	void CommitEndpointSamples();
	void PerformFixedStep(double InterpolationAlpha);
	void GatherCollisionSnapshot(const CableSim::FStepInput& Input, TConstArrayView<AActor*> IgnoredActors);
	CableSim::FManifoldConfig BuildManifoldConfig() const;
	void PerformTautStep(CableSim::FStepInput& Input);
	void BuildTautGuideConstraints(CableSim::FStepInput& Input) const;
	void RefreshEditorPreview();
	void RefreshVisualization();

	CableSim::FSolver Solver;
	FCableSimRuntimeState* RuntimeState = nullptr;
};
