#pragma once

#include "CoreMinimal.h"

namespace CableSim
{
	enum class EEndpoint : uint8
	{
		Start,
		End
	};

	enum class EEndpointState : uint8
	{
		Free,
		Fixed,
		Driven
	};

	enum class ELengthChangeOrigin : uint8
	{
		Start,
		End,
		Both
	};

	enum class ESimulationStatus : uint8
	{
		Uninitialized,
		Ready,
		Overextended,
		MovementLimited,
		ParticleBudgetExceeded,
		GeometryBudgetExceeded,
		InvalidInitialOverlap,
		CollisionRecoveryFailed,
		InvalidConfiguration,
		NumericalFailure
	};

	enum class ECollisionFeatureType : uint8
	{
		None,
		Face,
		Edge,
		Vertex
	};

	struct CABLESIMCORE_API FCollisionFeatureId
	{
		uint64 ObjectToken = 0;
		int32 ShapeIndex = INDEX_NONE;
		ECollisionFeatureType Type = ECollisionFeatureType::None;
		int32 Index0 = INDEX_NONE;
		int32 Index1 = INDEX_NONE;

		bool operator==(const FCollisionFeatureId& Other) const = default;
		bool IsValid() const;
		static bool Less(const FCollisionFeatureId& A, const FCollisionFeatureId& B);
	};

	CABLESIMCORE_API uint32 GetTypeHash(const FCollisionFeatureId& Id);

	struct CABLESIMCORE_API FParticle
	{
		FVector3d Position = FVector3d::ZeroVector;
		FVector3d PreviousPosition = FVector3d::ZeroVector;
		FVector3d Velocity = FVector3d::ZeroVector;
		double InverseMass = 1.0;
		double Mass = 1.0;
		/** Absolute distance along the active cable material, in centimetres. */
		double MaterialCoordinate = 0.0;
		double EstimatedTension = 0.0;
		double EstimatedNormalLoad = 0.0;
	};

	struct CABLESIMCORE_API FSimulationConfig
	{
		double Length = 400.0;
		double SegmentLength = 10.0;
		int32 MaximumParticles = 512;
		/** Physical cable density in kilograms per centimetre (0.005 = 0.5 kg/m). */
		double LinearDensity = 0.005;
		FVector3d Gravity = FVector3d(0.0, 0.0, -980.665);
		double VelocityDamping = 0.01;
		double DistanceCompliance = 0.0;
		double DistanceRelaxation = 1.015;
		double BendStrength = 0.20;
		double FreeBendDegreesPerMeter = 90.0;
		double DrivenEndpointMaximumSpeed = 300.0;
		int32 SolverIterations = 64;
		int32 MultigridIterations = 2;
		int32 MultigridMinimumParticles = 64;
		double StaticFriction = 0.35;
		double DynamicFriction = 0.25;
		double ConstantFrictionSpeedReduction = 5.0;
		double StaticFrictionDeadZone = 0.05;
		double FrictionFadeStartSpeed = 50.0;
		double FrictionFadeEndSpeed = 150.0;
		double MaximumContactCorrection = 5.0;

		bool Equals(const FSimulationConfig& Other, double Tolerance = 1.e-9) const;
	};

	struct CABLESIMCORE_API FEndpointInput
	{
		EEndpointState State = EEndpointState::Free;
		FVector3d TargetPosition = FVector3d::ZeroVector;
		FVector3d TargetVelocity = FVector3d::ZeroVector;
	};

	struct CABLESIMCORE_API FContactConstraint
	{
		FCollisionFeatureId FeatureId;
		int32 SampleIndex = INDEX_NONE;
		int32 ParticleA = INDEX_NONE;
		int32 ParticleB = INDEX_NONE;
		double SegmentAlpha = 0.0;
		FVector3d Normal = FVector3d::UnitZ();
		double MinimumNormalCoordinate = 0.0;
		FVector3d SurfaceVelocity = FVector3d::ZeroVector;
		FVector3d FrictionAnchor = FVector3d::ZeroVector;
		bool bEnableFriction = true;
		double AccumulatedNormalCorrection = 0.0;
		double AccumulatedParticleCorrectionA = 0.0;
		double AccumulatedParticleCorrectionB = 0.0;
		double AccumulatedFrictionCorrection = 0.0;
		double LargestProjection = 0.0;
	};

	struct CABLESIMCORE_API FStepInput
	{
		double DeltaTime = 1.0 / 60.0;
		FEndpointInput StartEndpoint;
		FEndpointInput EndEndpoint;
	};

	struct CABLESIMCORE_API FEndpointResult
	{
		EEndpointState State = EEndpointState::Free;
		FVector3d RequestedPosition = FVector3d::ZeroVector;
		FVector3d AcceptedPosition = FVector3d::ZeroVector;
		FVector3d Velocity = FVector3d::ZeroVector;
		FVector3d Correction = FVector3d::ZeroVector;
		double LimitError = 0.0;
		bool bLimited = false;
	};

	struct CABLESIMCORE_API FStepResult
	{
		ESimulationStatus Status = ESimulationStatus::Uninitialized;
		uint64 StepIndex = 0;
		int32 ParticleCount = 0;
		int32 ContactCount = 0;
		double MaximumSegmentError = 0.0;
		/** Maximum positive (tensile) segment strain; compression remains visible through MaximumSegmentError. */
		double MaximumSegmentStrain = 0.0;
		double MaximumPenetration = 0.0;
		double MaximumContactCorrection = 0.0;
		double MaximumParticleTravel = 0.0;
		double MaximumSpeed = 0.0;
		double MaximumEstimatedTension = 0.0;
		double MaximumEstimatedNormalLoad = 0.0;
		double ActiveLength = 0.0;
		FEndpointResult StartEndpoint;
		FEndpointResult EndEndpoint;
	};

	struct CABLESIMCORE_API FStateSnapshot
	{
		TArray<FParticle> Particles;
		FSimulationConfig Config;
		FStepResult LastResult;
		uint64 StepIndex = 0;
	};

	class CABLESIMCORE_API FSolver
	{
	public:
		bool Initialize(const FVector3d& Start, const FVector3d& End, const FSimulationConfig& Config);
		void Reset();
		bool ApplyConfig(const FSimulationConfig& Config);
		bool SetActiveLength(double NewLength, ELengthChangeOrigin Origin);
		bool RemeshPreservingState(double NewSegmentLength, int32 NewMaximumParticles);

		bool BeginStep(const FStepInput& Input);
		void SolveBatch(const FStepInput& Input, TArrayView<FContactConstraint> Contacts, int32 Iterations);
		void ReconcileContacts(const FStepInput& Input, TArrayView<FContactConstraint> Contacts, int32 Iterations = 4);
		FStepResult FinalizeStep(const FStepInput& Input, TArrayView<FContactConstraint> Contacts);
		FStepResult AdvanceStep(const FStepInput& Input, TArrayView<FContactConstraint> Contacts = {});

		int32 FindSegmentAtMaterialCoordinate(double Coordinate, double& OutAlpha) const;
		FVector3d SamplePosition(double MaterialCoordinate) const;
		FVector3d SamplePreviousPosition(double MaterialCoordinate) const;

		bool IsInitialized() const { return Particles.Num() >= 2; }
		const TArray<FParticle>& GetParticles() const { return Particles; }
		const FSimulationConfig& GetConfig() const { return Config; }
		double GetActiveLength() const;
		const FStepResult& GetLastResult() const { return LastResult; }
		FStateSnapshot CaptureState() const;
		bool RestoreState(const FStateSnapshot& Snapshot);

	private:
		static bool IsFinite(const FVector3d& Value);
		static bool IsValidConfig(const FSimulationConfig& Value);
		static FParticle InterpolateParticle(const FParticle& A, const FParticle& B, double Alpha);
		int32 GetEndpointIndex(EEndpoint Endpoint) const;
		double EffectiveInverseMass(int32 ParticleIndex, const FStepInput& Input) const;
		double SegmentRestLength(int32 SegmentIndex) const;
		void AddLengthAtEndpoint(double Amount, EEndpoint Endpoint);
		void RemoveLengthAtEndpoint(double Amount, EEndpoint Endpoint);
		void NormalizeEndpointResolution(EEndpoint Endpoint);
		void RecalculateParticleMasses();
		void ProjectEndpointDrive(EEndpoint Endpoint, const FEndpointInput& EndpointInput, double DeltaTime);
		void ProjectDistance(int32 SegmentIndex, const FStepInput& Input);
		void ProjectDistanceRange(int32 FirstIndex, int32 SecondIndex, const FStepInput& Input);
		void SolveMultigrid(const FStepInput& Input, TArrayView<FContactConstraint> Contacts);
		void ProjectBend(int32 FirstIndex, const FStepInput& Input, double PerIterationStrength);
		double ProjectContact(FContactConstraint& Contact, const FStepInput& Input);
		void ProjectParticleFriction(int32 ParticleIndex, TConstArrayView<FContactConstraint> Contacts, const FStepInput& Input);
		void ApplyContactVelocityResponse(
			TConstArrayView<FContactConstraint> Contacts,
			const FStepInput& Input,
			TConstArrayView<FVector3d> PreSolveVelocities);
		void UpdateContactLoads(TConstArrayView<FContactConstraint> Contacts, const FStepInput& Input);
		bool ValidateState() const;
		double CalculateMaximumSegmentError() const;
		double CalculateMaximumPenetration(TConstArrayView<FContactConstraint> Contacts) const;
		void FillEndpointResult(EEndpoint Endpoint, const FEndpointInput& Input, FEndpointResult& Result) const;

		TArray<FParticle> Particles;
		FSimulationConfig Config;
		FStepResult LastResult;
		FStateSnapshot LastValidState;
		TArray<double> DistanceLambdas;
		TArray<double> ParticleFrictionCorrections;
		uint64 StepIndex = 0;
		bool bStepActive = false;
	};
}
