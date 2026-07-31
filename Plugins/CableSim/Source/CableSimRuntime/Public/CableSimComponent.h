#pragma once

#include "Debug/DebugDrawComponent.h"
#include "CableSimComponent.generated.h"

UENUM(BlueprintType)
enum class ECableSimEndpoint : uint8
{
	Start,
	End
};

UENUM(BlueprintType)
enum class ECableSimEndpointState : uint8
{
	Free,
	Fixed,
	Driven
};

UENUM(BlueprintType)
enum class ECableSimLengthChangeOrigin : uint8
{
	Start,
	End,
	Both
};

UENUM(BlueprintType)
enum class ECableSimTargetSpace : uint8
{
	CableLocal,
	World,
	Component
};

// Serialization bridge for the existing playground. New code uses
// ECableSimEndpointState and ECableSimTargetSpace independently.
UENUM(meta = (Deprecated, DeprecationMessage = "Use endpoint State and TargetSpace"))
enum class ECableSimEndpointMode : uint8
{
	Simulated,
	CableLocalKinematic,
	WorldKinematic,
	ComponentKinematic
};

UENUM(BlueprintType)
enum class ECableSimSimulationStatus : uint8
{
	Uninitialized,
	Ready,
	Overextended,
	MotionClamped,
	MovementLimited,
	ParticleBudgetExceeded,
	CollisionBudgetExceeded,
	InvalidInitialOverlap,
	CollisionRecoveryFailed,
	InvalidConfiguration,
	NumericalFailure
};

USTRUCT(BlueprintType)
struct CABLESIMRUNTIME_API FCableSimEndpointBinding
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	ECableSimEndpointState State = ECableSimEndpointState::Fixed;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	ECableSimTargetSpace TargetSpace = ECableSimTargetSpace::CableLocal;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	FVector SpawnLocalPosition = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (EditCondition = "TargetSpace == ECableSimTargetSpace::CableLocal"))
	FVector LocalTarget = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (EditCondition = "TargetSpace == ECableSimTargetSpace::World"))
	FVector WorldTarget = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (EditCondition = "TargetSpace == ECableSimTargetSpace::Component"))
	TObjectPtr<USceneComponent> TargetComponent = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (EditCondition = "TargetSpace == ECableSimTargetSpace::Component"))
	FName SocketName = NAME_None;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (EditCondition = "TargetSpace == ECableSimTargetSpace::Component"))
	FVector ComponentLocalOffset = FVector::ZeroVector;

	UPROPERTY()
	ECableSimEndpointMode Mode = ECableSimEndpointMode::CableLocalKinematic;
};

USTRUCT(BlueprintType)
struct CABLESIMRUNTIME_API FCableSimSimulationSettings
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "1.0", Units = "cm"))
	double RestLength = 400.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "2.0", Units = "cm"))
	double NodeSpacing = 10.0;

	UPROPERTY(meta = (DeprecatedProperty, DeprecationMessage = "Uniform particles replace contact-time refinement"))
	double ContactRefinementLength = 5.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "2", ClampMax = "4096"))
	int32 MaximumParticles = 512;

	UPROPERTY(meta = (DeprecatedProperty, DeprecationMessage = "Use LinearDensity"))
	double ParticleMass = 0.05;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable|Material", meta = (ClampMin = "0.000001"))
	double LinearDensity = 0.5;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable|Length", meta = (ClampMin = "1.0", Units = "cm"))
	double MaximumLength = 3000.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable|Length", meta = (ClampMin = "0.0", Units = "cm/s"))
	double PayoutSpeed = 100.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable|Length", meta = (ClampMin = "0.0", Units = "cm/s"))
	double ReelSpeed = 100.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable|Length")
	ECableSimLengthChangeOrigin LengthChangeOrigin = ECableSimLengthChangeOrigin::End;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	FVector Gravity = FVector(0.0, 0.0, -980.665);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	double VelocityDamping = 0.01;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0"))
	double DistanceCompliance = 0.0;

	UPROPERTY(meta = (DeprecatedProperty, DeprecationMessage = "Use BendStrength and FreeBendDegreesPerMeter"))
	double BendCompliance = 0.02;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "1.0", ClampMax = "1.1"))
	double DistanceRelaxation = 1.015;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	double BendStrength = 0.20;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0"))
	double FreeBendDegreesPerMeter = 90.0;

	/** Maximum speed at which a driven end may chase its requested target. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "1.0", Units = "cm/s"))
	double DrivenEndpointMaximumSpeed = 300.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "1", ClampMax = "256"))
	int32 SolverIterations = 64;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0", ClampMax = "8"))
	int32 MultigridIterations = 2;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "8", ClampMax = "4096"))
	int32 MultigridMinimumParticles = 64;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.001", Units = "s"))
	double FixedTimeStep = 1.0 / 60.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "1", ClampMax = "8"))
	int32 MaximumSubsteps = 4;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	bool bSimulationEnabled = true;
};

USTRUCT(BlueprintType)
struct CABLESIMRUNTIME_API FCableSimCollisionSettings
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	bool bEnableWorldCollision = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.1", Units = "cm"))
	double Radius = 2.5;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0", Units = "cm"))
	double SkinWidth = 0.25;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "1", ClampMax = "8"))
	int32 MaximumContactsPerElement = 4;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "1", ClampMax = "3"))
	int32 MaximumEdgesPerElement = 3;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0", Units = "cm"))
	double ContactReleaseDistance = 1.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.01", Units = "cm"))
	double TopologyTolerance = 0.1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.1", Units = "cm"))
	double MaximumSafeCorrection = 5.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	bool bCollideWorldStatic = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	bool bCollideWorldDynamic = true;

	/** Dedicated trace channel used by normal and rope-only collision. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	TEnumAsByte<ECollisionChannel> TraceChannel = ECC_Visibility;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	bool bTraceComplex = false;

	/** Broadphase allowance for moving collision that can cross the cable in one step. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0", Units = "cm/s"))
	double MaximumDynamicColliderSpeed = 2000.0;
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

	/** Constant tangential speed removed after each contacting solve step. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0", Units = "cm/s"))
	double ConstantVelocityReduction = 5.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0", Units = "cm"))
	double StaticDeadZone = 0.05;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0", Units = "cm/s"))
	double FadeStartSpeed = 50.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0", Units = "cm/s"))
	double FadeEndSpeed = 150.0;
};

USTRUCT(BlueprintType)
struct CABLESIMRUNTIME_API FCableSimDebugSettings
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	bool bDraw = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	bool bDrawParticles = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	bool bDrawContacts = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	bool bDrawCollisionGeometry = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	bool bDrawPredictedBounds = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	bool bDrawRejectedContacts = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	bool bDrawStatus = true;

	/** Colors the cable line by each segment's estimated tension (cold = slack, hot = near this frame's peak) instead of a flat CableColor. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	bool bColorByTension = true;

	/** Draws a small marker at each active friction anchor plus a line toward the contact's surface velocity. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	bool bDrawFriction = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable")
	FLinearColor CableColor = FLinearColor(0.0f, 1.0f, 1.0f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable", meta = (ClampMin = "0.0"))
	float LineThickness = 1.5f;
};

USTRUCT(BlueprintType)
struct CABLESIMRUNTIME_API FCableSimEndpointResult
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "Cable")
	ECableSimEndpointState State = ECableSimEndpointState::Free;

	UPROPERTY(BlueprintReadOnly, Category = "Cable")
	FVector RequestedWorldPosition = FVector::ZeroVector;

	UPROPERTY(BlueprintReadOnly, Category = "Cable")
	FVector AcceptedWorldPosition = FVector::ZeroVector;

	UPROPERTY(BlueprintReadOnly, Category = "Cable")
	FVector CorrectionWorld = FVector::ZeroVector;

	UPROPERTY(BlueprintReadOnly, Category = "Cable")
	FVector Velocity = FVector::ZeroVector;

	UPROPERTY(BlueprintReadOnly, Category = "Cable", meta = (Units = "cm"))
	double LimitError = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable")
	bool bLimited = false;

	UPROPERTY(BlueprintReadOnly, Category = "Cable")
	bool bBindingValid = true;
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
	int32 CollisionQueryCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable")
	int32 CollisionTriangleCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable")
	int32 ConvexEdgeCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable")
	int32 RejectedContactCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable", meta = (Units = "cm"))
	double MaximumSegmentError = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable", meta = (Units = "cm"))
	double MaximumPenetration = 0.0;

	/** Maximum positive (tensile) segment strain. */
	UPROPERTY(BlueprintReadOnly, Category = "Cable")
	double MaximumSegmentStrain = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable", meta = (Units = "cm"))
	double MaximumContactCorrection = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable")
	double MaximumEstimatedTension = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable")
	double MaximumEstimatedNormalLoad = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	double AcceptedMovementFraction = 1.0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable", meta = (Units = "ms"))
	double SimulationMilliseconds = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable")
	bool bMotionClamped = false;

	UPROPERTY(BlueprintReadOnly, Category = "Cable")
	bool bCollisionDegraded = false;

	UPROPERTY(BlueprintReadOnly, Category = "Cable", meta = (Units = "cm"))
	double ActiveLength = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable", meta = (Units = "cm"))
	double TargetLength = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable", meta = (Units = "cm"))
	double MaximumLength = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Cable")
	ECableSimLengthChangeOrigin LengthChangeOrigin = ECableSimLengthChangeOrigin::End;

	UPROPERTY(BlueprintReadOnly, Category = "Cable")
	bool bLengthChanging = false;

	UPROPERTY(BlueprintReadOnly, Category = "Cable")
	bool bLengthClamped = false;

	UPROPERTY(BlueprintReadOnly, Category = "Cable")
	bool bBindingFailure = false;
};

struct FCableSimRuntimeState;

UCLASS(ClassGroup = (Physics), meta = (BlueprintSpawnableComponent))
class CABLESIMRUNTIME_API UCableSimComponent : public UDebugDrawComponent
{
	GENERATED_BODY()

public:
	UCableSimComponent();
	virtual ~UCableSimComponent() override;
	virtual void Serialize(FArchive& Ar) override;
	virtual void PostLoad() override;
	virtual void OnRegister() override;
	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* TickFunction) override;
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
	FCableSimCollisionSettings CollisionSettings;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable|Settings")
	FCableSimFrictionSettings FrictionSettings;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cable|Settings")
	FCableSimDebugSettings DebugSettings;

	UFUNCTION(BlueprintCallable, Category = "Cable|Simulation")
	void ReinitializeSimulation();

	UFUNCTION(BlueprintCallable, Category = "Cable|Simulation")
	void StepSimulation(int32 StepCount = 1);

	UFUNCTION(BlueprintCallable, Category = "Cable|Endpoints")
	void SetEndpointState(ECableSimEndpoint Endpoint, ECableSimEndpointState State);

	UFUNCTION(BlueprintCallable, Category = "Cable|Endpoints")
	void SetDrivenEndpointTarget(ECableSimEndpoint Endpoint, FVector WorldPosition, FVector WorldVelocity = FVector::ZeroVector);

	UFUNCTION(BlueprintCallable, Category = "Cable|Endpoints")
	void AttachEndpointToComponent(ECableSimEndpoint Endpoint, USceneComponent* Component, FName SocketName = NAME_None, FVector LocalOffset = FVector::ZeroVector);

	UFUNCTION(BlueprintCallable, Category = "Cable|Endpoints")
	void DriveEndpointFromComponent(ECableSimEndpoint Endpoint, USceneComponent* Component, FName SocketName = NAME_None, FVector LocalOffset = FVector::ZeroVector);

	UFUNCTION(BlueprintCallable, Category = "Cable|Endpoints")
	void ReleaseEndpoint(ECableSimEndpoint Endpoint, bool bPreserveVelocity = true);

	UFUNCTION(BlueprintPure, Category = "Cable|Endpoints")
	FCableSimEndpointResult GetEndpointResult(ECableSimEndpoint Endpoint) const;

	UFUNCTION(BlueprintPure, Category = "Cable|Simulation")
	TArray<FVector> GetCablePolyline() const;

	UFUNCTION(BlueprintPure, Category = "Cable|Simulation")
	FCableSimStatus GetSimulationStatus() const;

	UFUNCTION(BlueprintCallable, Category = "Cable|Length")
	double SetTargetCableLength(double NewLength, ECableSimLengthChangeOrigin Origin);

	UFUNCTION(BlueprintCallable, Category = "Cable|Length")
	void StopLengthChange();

	UFUNCTION(BlueprintPure, Category = "Cable|Length")
	double GetActiveCableLength() const;

	UFUNCTION(BlueprintPure, Category = "Cable|Length")
	double GetTargetCableLength() const;

	UFUNCTION(BlueprintPure, Category = "Cable|Length")
	bool IsLengthChanging() const;

	/** Writes the complete current solver/collision frame to Saved/CableSimDumps. */
	UFUNCTION(BlueprintCallable, Category = "Cable|Debug")
	FString WriteDebugFrameDump() const;

private:
	virtual FDebugRenderSceneProxy* CreateDebugSceneProxy() override;
	FCableSimEndpointBinding& Binding(ECableSimEndpoint Endpoint);
	const FCableSimEndpointBinding& Binding(ECableSimEndpoint Endpoint) const;
	void MigrateLegacyBindings();
	void RefreshEditorPreview();
	void RefreshVisualization();
	void PerformFixedStep(double FrameAlpha);
	bool ResolveEndpointTarget(const FCableSimEndpointBinding& Endpoint, int32 EndpointIndex, FVector3d& OutTarget);

	FCableSimRuntimeState* RuntimeState = nullptr;

	UPROPERTY()
	int32 MaterialSettingsVersion = 0;
};
