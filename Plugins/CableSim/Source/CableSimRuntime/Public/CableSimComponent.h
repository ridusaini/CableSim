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
enum class ECableSimTautStatus : uint8
{
	Disabled,
	Uninitialized,
	Ready,
	NoRelevantGeometry,
	NonManifoldTopology,
	TopologyOverValence,
	FeatureInvalidated,
	IterationBudgetExceeded,
	SnapshotFailed,
	InvalidConfiguration,
	NumericalFailure
};

UENUM(BlueprintType)
enum class ECableSimEndpointConstraintStatus : uint8
{
	Disabled,
	ValidSlack,
	Limited,
	UnsupportedEndpointConfiguration,
	TautPathUnavailable
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

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0", ClampMax = "8"))
	int32 ContactPersistenceSteps = 2;

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
	bool bEnableTautSolver = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	bool bEnableReachConstraint = true;

	/** Endpoint limited by the taut path. The opposite kinematic endpoint is the anchor. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (EditCondition = "bEnableReachConstraint"))
	ECableSimEndpoint ConstrainedEndpoint = ECableSimEndpoint::End;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.001", Units = "cm"))
	double TopologyTolerance = 0.1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "2", ClampMax = "64"))
	int32 MaximumCollisionPasses = 16;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	double GuideActivationPathRatio = 0.80;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0"))
	double GuideSlackScale = 0.50;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0", Units = "cm"))
	double MaximumGuideRadius = 100.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	double GuideStepStrength = 0.85;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0", Units = "cm"))
	double LengthTolerance = 0.1;
};

UENUM(BlueprintType, meta = (Bitflags))
enum class ECableSimDebugDraw : uint8
{
	Particles = 0 UMETA(DisplayName = "Particles"),
	SnapshotBounds = 1 UMETA(DisplayName = "Snapshot bounds"),
	CandidateTopology = 2 UMETA(DisplayName = "Candidate topology"),
	ActiveContacts = 3 UMETA(DisplayName = "Active contacts"),
	Friction = 4 UMETA(DisplayName = "Friction / load"),
	TautPathAndGuide = 5 UMETA(DisplayName = "Taut path / guide"),
	StatusAndReach = 6 UMETA(DisplayName = "Status / reach")
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
	int32 DrawFlags = (1 << static_cast<uint8>(ECableSimDebugDraw::Particles))
		| (1 << static_cast<uint8>(ECableSimDebugDraw::ActiveContacts))
		| (1 << static_cast<uint8>(ECableSimDebugDraw::TautPathAndGuide))
		| (1 << static_cast<uint8>(ECableSimDebugDraw::StatusAndReach));

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
	double EffectiveSolveLength = 0.0;

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

USTRUCT(BlueprintType)
struct CABLESIMRUNTIME_API FCableSimEndpointConstraint
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	ECableSimEndpointConstraintStatus Status = ECableSimEndpointConstraintStatus::Disabled;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	ECableSimEndpoint Endpoint = ECableSimEndpoint::End;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	FVector RequestedWorldPosition = FVector::ZeroVector;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	FVector ReachableWorldPosition = FVector::ZeroVector;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	FVector CorrectionWorld = FVector::ZeroVector;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	FVector ConstraintDirection = FVector::ZeroVector;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut", meta = (Units = "cm"))
	double PathLength = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut", meta = (Units = "cm"))
	double RestLength = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut", meta = (Units = "cm"))
	double ExcessDistance = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable|Taut")
	bool bLimited = false;
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

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Cable|Taut")
	FCableSimEndpointConstraint StartEndpointConstraint;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Cable|Taut")
	FCableSimEndpointConstraint EndEndpointConstraint;

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

	UFUNCTION(BlueprintPure, Category = "Cable|Taut")
	FCableSimEndpointConstraint GetEndpointConstraint(ECableSimEndpoint Endpoint) const;

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
