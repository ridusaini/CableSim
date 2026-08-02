#pragma once

#include "CoreMinimal.h"

namespace CableSim
{
	enum class EParticleMode : uint8
	{
		Dynamic,
		Kinematic
	};

	enum class EEndpoint : uint8
	{
		Start,
		End
	};

	enum class ESimulationStatus : uint8
	{
		Uninitialized,
		Ready,
		Overextended,
		NumericalFailure,
		InvalidConfiguration
	};

	struct CABLESIMCORE_API FParticle
	{
		FVector3d Position = FVector3d::ZeroVector;
		FVector3d PreviousPosition = FVector3d::ZeroVector;
		FVector3d Velocity = FVector3d::ZeroVector;
		FVector3d KinematicTarget = FVector3d::ZeroVector;
		double EstimatedTension = 0.0;
		double EstimatedNormalLoad = 0.0;
		double InverseMass = 1.0;
		double MaterialCoordinate = 0.0;
		EParticleMode Mode = EParticleMode::Dynamic;
	};

	struct CABLESIMCORE_API FSimulationConfig
	{
		double RestLength = 400.0;
		double NodeSpacing = 10.0;
		double ParticleMass = 0.05;
		FVector3d Gravity = FVector3d(0.0, 0.0, -980.665);
		double VelocityDamping = 0.01;
		// Strength for the complete simulation step. The solver derives the
		// per-iteration strength so this material setting is iteration-independent.
		double BendingStepStrength = 0.20;
		double FreeBendAngleRadiansPerMeter = 0.0;
		double DistanceOverRelaxation = 1.015;
		int32 ConstraintIterations = 64;
		bool bEnableFriction = true;
		double StaticFrictionCoefficient = 0.35;
		double DynamicFrictionCoefficient = 0.25;
		double StaticFrictionSpeedThreshold = 2.0;
		double ContactActiveBand = 1.0;
		// Coarse-to-fine passes for near-taut convergence; a flat Gauss-Seidel sweep
		// alone leaves large residual sag on near-taut cables at typical resolutions.
		// 0 disables multigrid. The gate is low because ordinary-length cables need it;
		// SolveMultigrid is a no-op below ~4 particles regardless.
		int32 MultigridIterations = 4;
		int32 MultigridMinimumParticles = 8;

		bool Equals(const FSimulationConfig& Other, double Tolerance = 1.e-9) const;
	};

	struct CABLESIMCORE_API FEndpointStepInput
	{
		EParticleMode Mode = EParticleMode::Dynamic;
		FVector3d TargetPosition = FVector3d::ZeroVector;
		FVector3d TargetVelocity = FVector3d::ZeroVector;
	};

	struct CABLESIMCORE_API FContactConstraint
	{
		uint64 FeatureId = 0;
		int32 ParticleIndex = INDEX_NONE;
		FVector3d Normal = FVector3d::UnitZ();
		double MinimumNormalCoordinate = 0.0;
		FVector3d SurfaceVelocity = FVector3d::ZeroVector;
		FVector3d FrictionAnchorPosition = FVector3d::ZeroVector;
		bool bHasFrictionAnchor = false;
		bool bConvexEdge = false;
		FVector3d EdgeStart = FVector3d::ZeroVector;
		FVector3d EdgeEnd = FVector3d::ZeroVector;
		FVector3d SecondNormal = FVector3d::UnitX();
		double EdgeRadius = 0.0;
	};

	struct CABLESIMCORE_API FGuideConstraint
	{
		int32 ParticleIndex = INDEX_NONE;
		FVector3d TargetPosition = FVector3d::ZeroVector;
		double MaximumDistance = 0.0;
		// Strength for the complete simulation step. The solver derives an
		// iteration strength so changing iteration count does not change the guide.
		double StepStrength = 0.85;
	};

	struct CABLESIMCORE_API FContactDiagnostic
	{
		uint64 FeatureId = 0;
		int32 ParticleIndex = INDEX_NONE;
		FVector3d Normal = FVector3d::UnitZ();
		FVector3d SurfaceVelocity = FVector3d::ZeroVector;
		double NormalCorrection = 0.0;
		double StaticFrictionCorrection = 0.0;
		double DynamicFrictionVelocityChange = 0.0;
		double EstimatedTension = 0.0;
		double EstimatedNormalLoad = 0.0;
		bool bProjected = false;
		bool bStaticAnchorHeld = false;
	};

	struct CABLESIMCORE_API FStepInput
	{
		double DeltaTime = 1.0 / 60.0;
		FEndpointStepInput StartEndpoint;
		FEndpointStepInput EndEndpoint;
		TArray<FGuideConstraint> GuideConstraints;
	};

	struct CABLESIMCORE_API FStepResult
	{
		ESimulationStatus Status = ESimulationStatus::Uninitialized;
		uint64 StepIndex = 0;
		int32 ParticleCount = 0;
		int32 ConstraintIterations = 0;
		int32 ContactCount = 0;
		int32 GuideConstraintCount = 0;
		double RestLength = 0.0;
		double EndpointDistance = 0.0;
		double StrainRatio = 0.0;
		double MaximumSegmentError = 0.0;
		double MaximumPenetration = 0.0;
		double MaximumGuideError = 0.0;
		double MaximumParticleSpeed = 0.0;
		double RmsParticleSpeed = 0.0;
		double MaximumEstimatedTension = 0.0;
		double MaximumEstimatedNormalLoad = 0.0;
		int32 ProjectedContactCount = 0;
		int32 StaticFrictionAnchorCount = 0;
	};

	struct CABLESIMCORE_API FStateSnapshot
	{
		TArray<FParticle> Particles;
		FSimulationConfig Config;
		FStepResult LastStepResult;
		double RestSegmentLength = 0.0;
		uint64 StepIndex = 0;
	};

	struct CABLESIMCORE_API FReplayFrame
	{
		FStepInput Input;
		TArray<FContactConstraint> Contacts;
	};

	using FContactGenerator = TFunction<void(TConstArrayView<FParticle>, TArray<FContactConstraint>&)>;

	class CABLESIMCORE_API FSolver
	{
	public:
		bool Initialize(
			const FVector3d& StartPosition,
			const FVector3d& EndPosition,
			const FSimulationConfig& InConfig);

		void Reset();
		bool ApplyConfig(const FSimulationConfig& InConfig);
		FStepResult AdvanceStep(const FStepInput& Input, const FContactGenerator& ContactGenerator = {});

		bool IsInitialized() const { return Particles.Num() >= 2; }
		bool IsSuspended() const { return bSuspendedAfterFailure; }
		const TArray<FParticle>& GetParticles() const { return Particles; }
		const FSimulationConfig& GetConfig() const { return Config; }
		double GetRestLength() const { return Config.RestLength; }
		double GetRestSegmentLength() const { return RestSegmentLength; }
		uint64 GetStepIndex() const { return StepIndex; }
		const FStepResult& GetLastStepResult() const { return LastStepResult; }
		const FReplayFrame& GetLastReplayFrame() const { return LastReplayFrame; }
		const TArray<FContactDiagnostic>& GetLastContactDiagnostics() const { return LastContactDiagnostics; }

		FStateSnapshot CaptureState() const;
		bool RestoreState(const FStateSnapshot& Snapshot);
		static double CalculateIterationStrength(double StepStrength, int32 IterationCount);

	private:
		static int32 GetEndpointIndex(EEndpoint Endpoint, int32 ParticleCount);
		static bool IsFinite(const FVector3d& Value);
		static bool IsValidConfig(const FSimulationConfig& InConfig);
		static FSimulationConfig SanitizeConfig(const FSimulationConfig& InConfig);
		static FParticle SampleParticle(TConstArrayView<FParticle> Source, double NormalizedCoordinate);
		bool RemeshPreservingState(const FSimulationConfig& NewConfig);
		bool ValidateState() const;
		void ApplyEndpointInput(EEndpoint Endpoint, const FEndpointStepInput& EndpointInput);
		double GetEffectiveInverseMass(const FParticle& Particle) const;
		void ProjectDistanceConstraint(int32 FirstIndex, int32 SecondIndex, bool bPullOnly = false);
		void SolveMultigrid(TConstArrayView<FContactConstraint> Contacts, TConstArrayView<FGuideConstraint> Guides);
		void ProjectBendingConstraint(
			int32 FirstIndex,
			int32 MiddleIndex,
			int32 LastIndex,
			double IterationStrength);
		double ProjectContactConstraint(const FContactConstraint& Contact, FVector3d& OutEffectiveNormal, bool& bOutActive);
		static FVector3d ClosestPointOnSegment(const FVector3d& Point, const FVector3d& Start, const FVector3d& End);
		void ProjectParticleFriction(
			int32 ParticleIndex,
			TConstArrayView<FContactConstraint> Contacts,
			TArray<double>& AccumulatedStaticCorrections,
			double DeltaTime);
		void ProjectGuideConstraint(const FGuideConstraint& Guide, double IterationStrength);
		void ApplyContactVelocityResponse(const TArray<FContactConstraint>& Contacts, double DeltaTime);
		void UpdateContactLoads(const TArray<FContactConstraint>& Contacts);
		static void SanitizeContacts(TConstArrayView<FParticle> Particles, TArray<FContactConstraint>& Contacts);
		double CalculateMaximumSegmentError() const;
		double CalculateMaximumPenetration(const TArray<FContactConstraint>& Contacts) const;
		double CalculateMaximumGuideError(const TArray<FGuideConstraint>& Guides) const;
		void UpdateStepResult(
			ESimulationStatus Status,
			double EndpointDistance,
			const TArray<FContactConstraint>& Contacts,
			const TArray<FGuideConstraint>& Guides);

		TArray<FParticle> Particles;
		FSimulationConfig Config;
		FStepResult LastStepResult;
		FReplayFrame LastReplayFrame;
		TArray<FContactDiagnostic> LastContactDiagnostics;
		TArray<bool> ProjectedContacts;
		TArray<bool> ActiveContacts;
		TArray<FVector3d> ContactEffectiveNormals;
		TArray<double> ContactNormalCorrections;
		TArray<double> ParticleStaticFrictionCorrections;
		TArray<double> ParticleDynamicFrictionVelocityChanges;
		double RestSegmentLength = 0.0;
		uint64 StepIndex = 0;
		bool bSuspendedAfterFailure = false;
	};
}
