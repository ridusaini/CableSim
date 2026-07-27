#include "CableSimSolver.h"

namespace CableSim
{
	namespace
	{
		constexpr double MinimumLength = 0.01;
		constexpr double MinimumMass = 1.e-9;
		constexpr double FeasibilityTolerance = 0.1;
		constexpr int32 MaximumSegmentCount = 4095;

		FVector3d RemoveNormalComponents(
			const FVector3d& Value,
			TConstArrayView<FVector3d> OrthonormalNormals)
		{
			FVector3d Result = Value;
			for (const FVector3d& Normal : OrthonormalNormals)
			{
				Result -= Normal * FVector3d::DotProduct(Result, Normal);
			}
			return Result;
		}

		void AddOrthonormalNormal(const FVector3d& Candidate, TArray<FVector3d, TInlineAllocator<3>>& Normals)
		{
			FVector3d Orthogonal = RemoveNormalComponents(Candidate, Normals);
			if (Orthogonal.Normalize())
			{
				Normals.Add(Orthogonal);
			}
		}
	}

	bool FSimulationConfig::Equals(const FSimulationConfig& Other, const double Tolerance) const
	{
		return FMath::IsNearlyEqual(RestLength, Other.RestLength, Tolerance)
			&& FMath::IsNearlyEqual(NodeSpacing, Other.NodeSpacing, Tolerance)
			&& FMath::IsNearlyEqual(ParticleMass, Other.ParticleMass, Tolerance)
			&& Gravity.Equals(Other.Gravity, Tolerance)
			&& FMath::IsNearlyEqual(VelocityDamping, Other.VelocityDamping, Tolerance)
			&& FMath::IsNearlyEqual(BendingStepStrength, Other.BendingStepStrength, Tolerance)
			&& FMath::IsNearlyEqual(FreeBendAngleRadiansPerMeter, Other.FreeBendAngleRadiansPerMeter, Tolerance)
			&& FMath::IsNearlyEqual(DistanceOverRelaxation, Other.DistanceOverRelaxation, Tolerance)
			&& ConstraintIterations == Other.ConstraintIterations
			&& bEnableFriction == Other.bEnableFriction
			&& FMath::IsNearlyEqual(StaticFrictionCoefficient, Other.StaticFrictionCoefficient, Tolerance)
			&& FMath::IsNearlyEqual(DynamicFrictionCoefficient, Other.DynamicFrictionCoefficient, Tolerance)
			&& FMath::IsNearlyEqual(StaticFrictionSpeedThreshold, Other.StaticFrictionSpeedThreshold, Tolerance)
			&& FMath::IsNearlyEqual(ContactActiveBand, Other.ContactActiveBand, Tolerance);
	}

	bool FSolver::Initialize(
		const FVector3d& StartPosition,
		const FVector3d& EndPosition,
		const FSimulationConfig& InConfig)
	{
		Reset();
		if (!IsFinite(StartPosition) || !IsFinite(EndPosition) || !IsValidConfig(InConfig))
		{
			LastStepResult.Status = ESimulationStatus::InvalidConfiguration;
			return false;
		}

		Config = SanitizeConfig(InConfig);
		const int32 SegmentCount = FMath::Clamp(
			FMath::CeilToInt(Config.RestLength / Config.NodeSpacing),
			1,
			MaximumSegmentCount);
		RestSegmentLength = Config.RestLength / static_cast<double>(SegmentCount);
		EffectiveSolveLength = Config.RestLength;
		SolveSegmentLength = RestSegmentLength;
		Particles.SetNum(SegmentCount + 1);

		for (int32 Index = 0; Index <= SegmentCount; ++Index)
		{
			const double Alpha = static_cast<double>(Index) / static_cast<double>(SegmentCount);
			FParticle& Particle = Particles[Index];
			Particle.Position = FMath::Lerp(StartPosition, EndPosition, Alpha);
			Particle.PreviousPosition = Particle.Position;
			Particle.KinematicTarget = Particle.Position;
			Particle.Velocity = FVector3d::ZeroVector;
			Particle.InverseMass = 1.0 / Config.ParticleMass;
			Particle.MaterialCoordinate = Alpha * Config.RestLength;
			Particle.Mode = EParticleMode::Dynamic;
		}

		LastStepResult.Status = ESimulationStatus::Ready;
		LastStepResult.ParticleCount = Particles.Num();
		LastStepResult.RestLength = Config.RestLength;
		LastStepResult.EffectiveSolveLength = EffectiveSolveLength;
		bSuspendedAfterFailure = false;
		return true;
	}

	void FSolver::Reset()
	{
		Particles.Reset();
		Config = FSimulationConfig{};
		LastStepResult = FStepResult{};
		LastReplayFrame = FReplayFrame{};
		LastContactDiagnostics.Reset();
		ProjectedContacts.Reset();
		ActiveContacts.Reset();
		ContactNormalCorrections.Reset();
		ParticleStaticFrictionCorrections.Reset();
		ParticleDynamicFrictionVelocityChanges.Reset();
		RestSegmentLength = 0.0;
		EffectiveSolveLength = 0.0;
		SolveSegmentLength = 0.0;
		StepIndex = 0;
		bSuspendedAfterFailure = false;
	}

	bool FSolver::ApplyConfig(const FSimulationConfig& InConfig)
	{
		if (!IsInitialized() || !IsValidConfig(InConfig))
		{
			LastStepResult.Status = ESimulationStatus::InvalidConfiguration;
			return false;
		}

		const FSimulationConfig NewConfig = SanitizeConfig(InConfig);
		const bool bRequiresRemesh =
			!FMath::IsNearlyEqual(NewConfig.RestLength, Config.RestLength)
			|| !FMath::IsNearlyEqual(NewConfig.NodeSpacing, Config.NodeSpacing);
		if (bRequiresRemesh && !RemeshPreservingState(NewConfig))
		{
			LastStepResult.Status = ESimulationStatus::InvalidConfiguration;
			return false;
		}

		Config = NewConfig;
		const double InverseMass = 1.0 / Config.ParticleMass;
		for (FParticle& Particle : Particles)
		{
			Particle.InverseMass = InverseMass;
		}
		RestSegmentLength = Config.RestLength / static_cast<double>(Particles.Num() - 1);
		EffectiveSolveLength = FMath::Max(EffectiveSolveLength, Config.RestLength);
		SolveSegmentLength = EffectiveSolveLength / static_cast<double>(Particles.Num() - 1);
		bSuspendedAfterFailure = false;
		return true;
	}

	FStepResult FSolver::AdvanceStep(const FStepInput& Input, const FContactGenerator& ContactGenerator)
	{
		if (!IsInitialized())
		{
			LastStepResult.Status = ESimulationStatus::Uninitialized;
			return LastStepResult;
		}
		if (bSuspendedAfterFailure)
		{
			LastStepResult.Status = ESimulationStatus::NumericalFailure;
			return LastStepResult;
		}
		if (!FMath::IsFinite(Input.DeltaTime) || Input.DeltaTime <= 0.0
			|| !IsFinite(Input.StartEndpoint.TargetPosition)
			|| !IsFinite(Input.EndEndpoint.TargetPosition)
			|| !IsFinite(Input.StartEndpoint.TargetVelocity)
			|| !IsFinite(Input.EndEndpoint.TargetVelocity))
		{
			LastStepResult.Status = ESimulationStatus::InvalidConfiguration;
			return LastStepResult;
		}

		const FStateSnapshot LastValidState = CaptureState();
		ApplyEndpointInput(EEndpoint::Start, Input.StartEndpoint);
		ApplyEndpointInput(EEndpoint::End, Input.EndEndpoint);

		const bool bBothEndpointsKinematic =
			Input.StartEndpoint.Mode == EParticleMode::Kinematic
			&& Input.EndEndpoint.Mode == EParticleMode::Kinematic;
		const double EndpointDistance = FVector3d::Distance(
			Input.StartEndpoint.TargetPosition,
			Input.EndEndpoint.TargetPosition);
		EffectiveSolveLength = bBothEndpointsKinematic
			? FMath::Max(Config.RestLength, EndpointDistance)
			: Config.RestLength;
		SolveSegmentLength = EffectiveSolveLength / static_cast<double>(Particles.Num() - 1);

		const double DampingMultiplier = FMath::Clamp(1.0 - Config.VelocityDamping, 0.0, 1.0);
		for (FParticle& Particle : Particles)
		{
			Particle.PreviousPosition = Particle.Position;
			if (Particle.Mode == EParticleMode::Kinematic)
			{
				Particle.Position = Particle.KinematicTarget;
				continue;
			}
			Particle.Velocity *= DampingMultiplier;
			Particle.Velocity += Config.Gravity * Input.DeltaTime;
			Particle.Position += Particle.Velocity * Input.DeltaTime;
		}

		TArray<FContactConstraint> Contacts;
		if (ContactGenerator)
		{
			ContactGenerator(Particles, Contacts);
		}
		SanitizeContacts(Particles, Contacts);
		TArray<FGuideConstraint> Guides = Input.GuideConstraints;
		for (int32 Index = Guides.Num() - 1; Index >= 0; --Index)
		{
			FGuideConstraint& Guide = Guides[Index];
			if (!Particles.IsValidIndex(Guide.ParticleIndex)
				|| Particles[Guide.ParticleIndex].Mode != EParticleMode::Dynamic
				|| !IsFinite(Guide.TargetPosition)
				|| !FMath::IsFinite(Guide.MaximumDistance)
				|| !FMath::IsFinite(Guide.StepStrength))
			{
				Guides.RemoveAt(Index, 1, EAllowShrinking::No);
				continue;
			}
			Guide.MaximumDistance = FMath::Max(Guide.MaximumDistance, 0.0);
			Guide.StepStrength = FMath::Clamp(Guide.StepStrength, 0.0, 1.0);
		}

		ProjectedContacts.Init(false, Contacts.Num());
		ActiveContacts.Init(false, Contacts.Num());
		ContactNormalCorrections.Init(0.0, Contacts.Num());
		TArray<double> AccumulatedStaticCorrections;
		AccumulatedStaticCorrections.Init(0.0, Particles.Num());
		ParticleStaticFrictionCorrections.Init(0.0, Particles.Num());
		ParticleDynamicFrictionVelocityChanges.Init(0.0, Particles.Num());
		int32 RefreshedContactCount = 0;
		for (int32 Iteration = 0; Iteration < Config.ConstraintIterations; ++Iteration)
		{
			if ((Iteration & 1) == 0)
			{
				for (int32 SegmentIndex = 0; SegmentIndex + 1 < Particles.Num(); ++SegmentIndex)
				{
					ProjectDistanceConstraint(SegmentIndex, SegmentIndex + 1);
				}
			}
			else
			{
				for (int32 SegmentIndex = Particles.Num() - 2; SegmentIndex >= 0; --SegmentIndex)
				{
					ProjectDistanceConstraint(SegmentIndex, SegmentIndex + 1);
				}
			}
			if (Config.BendingStepStrength > 0.0)
			{
				const double IterationStrength = CalculateIterationStrength(
					Config.BendingStepStrength,
					Config.ConstraintIterations);
				for (int32 MiddleIndex = 1; MiddleIndex + 1 < Particles.Num(); ++MiddleIndex)
				{
					ProjectBendingConstraint(
						MiddleIndex - 1,
						MiddleIndex,
						MiddleIndex + 1,
						IterationStrength);
				}
			}
			for (int32 ContactIndex = 0; ContactIndex < Contacts.Num(); ++ContactIndex)
			{
				const FContactConstraint& Contact = Contacts[ContactIndex];
				const double Correction = ProjectContactConstraint(Contact);
				if (Correction > 0.0)
				{
					ProjectedContacts[ContactIndex] = true;
					ContactNormalCorrections[ContactIndex] += Correction;
				}
				const double Separation = FVector3d::DotProduct(
					Particles[Contact.ParticleIndex].Position, Contact.Normal)
					- Contact.MinimumNormalCoordinate;
				if (Separation <= Config.ContactActiveBand)
				{
					ActiveContacts[ContactIndex] = true;
				}
			}
			for (int32 ParticleIndex = 0; ParticleIndex < Particles.Num(); ++ParticleIndex)
			{
				ProjectParticleFriction(
					ParticleIndex,
					Contacts,
					AccumulatedStaticCorrections,
					Input.DeltaTime);
			}
			for (const FGuideConstraint& Guide : Guides)
			{
				const double IterationStrength = CalculateIterationStrength(
					Guide.StepStrength,
					Config.ConstraintIterations);
				ProjectGuideConstraint(Guide, IterationStrength);
			}
			if (ContactGenerator && Config.ConstraintIterations > 1
				&& Iteration + 1 == Config.ConstraintIterations / 2)
			{
				TArray<FContactConstraint> RefreshedContacts;
				ContactGenerator(Particles, RefreshedContacts);
				SanitizeContacts(Particles, RefreshedContacts);
				RefreshedContactCount = RefreshedContacts.Num();
				Contacts = MoveTemp(RefreshedContacts);
				ProjectedContacts.Init(false, Contacts.Num());
				ActiveContacts.Init(false, Contacts.Num());
				ContactNormalCorrections.Init(0.0, Contacts.Num());
			}
		}

		for (FParticle& Particle : Particles)
		{
			if (Particle.Mode == EParticleMode::Dynamic)
			{
				Particle.Velocity = (Particle.Position - Particle.PreviousPosition) / Input.DeltaTime;
			}
		}
		if (Input.StartEndpoint.Mode == EParticleMode::Kinematic)
		{
			Particles[GetEndpointIndex(EEndpoint::Start, Particles.Num())].Velocity = Input.StartEndpoint.TargetVelocity;
		}
		if (Input.EndEndpoint.Mode == EParticleMode::Kinematic)
		{
			Particles[GetEndpointIndex(EEndpoint::End, Particles.Num())].Velocity = Input.EndEndpoint.TargetVelocity;
		}
		ApplyContactVelocityResponse(Contacts, Input.DeltaTime);
		UpdateContactLoads(Contacts);

		if (!ValidateState())
		{
			RestoreState(LastValidState);
			bSuspendedAfterFailure = true;
			LastStepResult.Status = ESimulationStatus::NumericalFailure;
			return LastStepResult;
		}

		++StepIndex;
		const ESimulationStatus Status = bBothEndpointsKinematic
			&& EndpointDistance > Config.RestLength + FeasibilityTolerance
			? ESimulationStatus::Overextended
			: ESimulationStatus::Ready;
		UpdateStepResult(Status, EndpointDistance, Contacts, RefreshedContactCount, Guides);
		LastReplayFrame.Input = Input;
		LastReplayFrame.Contacts = Contacts;
		return LastStepResult;
	}

	FStateSnapshot FSolver::CaptureState() const
	{
		FStateSnapshot Snapshot;
		Snapshot.Particles = Particles;
		Snapshot.Config = Config;
		Snapshot.LastStepResult = LastStepResult;
		Snapshot.RestSegmentLength = RestSegmentLength;
		Snapshot.EffectiveSolveLength = EffectiveSolveLength;
		Snapshot.StepIndex = StepIndex;
		return Snapshot;
	}

	bool FSolver::RestoreState(const FStateSnapshot& Snapshot)
	{
		if (Snapshot.Particles.Num() < 2 || !IsValidConfig(Snapshot.Config)
			|| !FMath::IsFinite(Snapshot.RestSegmentLength) || Snapshot.RestSegmentLength <= 0.0
			|| !FMath::IsFinite(Snapshot.EffectiveSolveLength)
			|| Snapshot.EffectiveSolveLength < Snapshot.Config.RestLength)
		{
			return false;
		}
		for (const FParticle& Particle : Snapshot.Particles)
		{
			if (!IsFinite(Particle.Position) || !IsFinite(Particle.PreviousPosition)
				|| !IsFinite(Particle.Velocity) || !IsFinite(Particle.KinematicTarget)
				|| !FMath::IsFinite(Particle.InverseMass)
				|| !FMath::IsFinite(Particle.MaterialCoordinate)
				|| !FMath::IsFinite(Particle.EstimatedTension)
				|| !FMath::IsFinite(Particle.EstimatedNormalLoad))
			{
				return false;
			}
		}
		Particles = Snapshot.Particles;
		Config = SanitizeConfig(Snapshot.Config);
		LastStepResult = Snapshot.LastStepResult;
		RestSegmentLength = Snapshot.RestSegmentLength;
		EffectiveSolveLength = Snapshot.EffectiveSolveLength;
		SolveSegmentLength = EffectiveSolveLength / static_cast<double>(Particles.Num() - 1);
		StepIndex = Snapshot.StepIndex;
		bSuspendedAfterFailure = false;
		return true;
	}

	double FSolver::CalculateIterationStrength(const double StepStrength, const int32 IterationCount)
	{
		const double SafeStepStrength = FMath::Clamp(StepStrength, 0.0, 1.0);
		return IterationCount > 0
			? 1.0 - FMath::Pow(1.0 - SafeStepStrength, 1.0 / static_cast<double>(IterationCount))
			: SafeStepStrength;
	}

	int32 FSolver::GetEndpointIndex(const EEndpoint Endpoint, const int32 ParticleCount)
	{
		return Endpoint == EEndpoint::Start ? 0 : ParticleCount - 1;
	}

	bool FSolver::IsFinite(const FVector3d& Value)
	{
		return FMath::IsFinite(Value.X) && FMath::IsFinite(Value.Y) && FMath::IsFinite(Value.Z);
	}

	bool FSolver::IsValidConfig(const FSimulationConfig& InConfig)
	{
		return FMath::IsFinite(InConfig.RestLength) && InConfig.RestLength > 0.0
			&& FMath::IsFinite(InConfig.NodeSpacing) && InConfig.NodeSpacing > 0.0
			&& FMath::IsFinite(InConfig.ParticleMass) && InConfig.ParticleMass > 0.0
			&& IsFinite(InConfig.Gravity)
			&& FMath::IsFinite(InConfig.VelocityDamping)
			&& FMath::IsFinite(InConfig.BendingStepStrength)
			&& FMath::IsFinite(InConfig.FreeBendAngleRadiansPerMeter)
			&& FMath::IsFinite(InConfig.DistanceOverRelaxation)
			&& FMath::IsFinite(InConfig.StaticFrictionCoefficient)
			&& FMath::IsFinite(InConfig.DynamicFrictionCoefficient)
			&& FMath::IsFinite(InConfig.StaticFrictionSpeedThreshold)
			&& FMath::IsFinite(InConfig.ContactActiveBand);
	}

	FSimulationConfig FSolver::SanitizeConfig(const FSimulationConfig& InConfig)
	{
		FSimulationConfig Result = InConfig;
		Result.RestLength = FMath::Max(Result.RestLength, MinimumLength);
		Result.NodeSpacing = FMath::Max(Result.NodeSpacing, MinimumLength);
		Result.ParticleMass = FMath::Max(Result.ParticleMass, MinimumMass);
		Result.VelocityDamping = FMath::Clamp(Result.VelocityDamping, 0.0, 1.0);
		Result.BendingStepStrength = FMath::Clamp(Result.BendingStepStrength, 0.0, 1.0);
		Result.FreeBendAngleRadiansPerMeter = FMath::Max(Result.FreeBendAngleRadiansPerMeter, 0.0);
		Result.DistanceOverRelaxation = FMath::Clamp(Result.DistanceOverRelaxation, 1.0, 1.05);
		Result.ConstraintIterations = FMath::Clamp(Result.ConstraintIterations, 0, 1024);
		Result.StaticFrictionCoefficient = FMath::Max(Result.StaticFrictionCoefficient, 0.0);
		Result.DynamicFrictionCoefficient = FMath::Max(Result.DynamicFrictionCoefficient, 0.0);
		Result.StaticFrictionSpeedThreshold = FMath::Max(Result.StaticFrictionSpeedThreshold, 0.0);
		Result.ContactActiveBand = FMath::Max(Result.ContactActiveBand, 0.0);
		return Result;
	}

	FParticle FSolver::SampleParticle(const TConstArrayView<FParticle> Source, const double NormalizedCoordinate)
	{
		const double ScaledIndex = FMath::Clamp(NormalizedCoordinate, 0.0, 1.0) * (Source.Num() - 1);
		const int32 FirstIndex = FMath::Clamp(FMath::FloorToInt(ScaledIndex), 0, Source.Num() - 1);
		const int32 SecondIndex = FMath::Min(FirstIndex + 1, Source.Num() - 1);
		const double Alpha = ScaledIndex - static_cast<double>(FirstIndex);
		FParticle Result;
		Result.Position = FMath::Lerp(Source[FirstIndex].Position, Source[SecondIndex].Position, Alpha);
		Result.PreviousPosition = FMath::Lerp(Source[FirstIndex].PreviousPosition, Source[SecondIndex].PreviousPosition, Alpha);
		Result.Velocity = FMath::Lerp(Source[FirstIndex].Velocity, Source[SecondIndex].Velocity, Alpha);
		Result.KinematicTarget = FMath::Lerp(Source[FirstIndex].KinematicTarget, Source[SecondIndex].KinematicTarget, Alpha);
		Result.EstimatedTension = FMath::Lerp(Source[FirstIndex].EstimatedTension, Source[SecondIndex].EstimatedTension, Alpha);
		Result.EstimatedNormalLoad = FMath::Lerp(Source[FirstIndex].EstimatedNormalLoad, Source[SecondIndex].EstimatedNormalLoad, Alpha);
		Result.Mode = EParticleMode::Dynamic;
		return Result;
	}

	bool FSolver::RemeshPreservingState(const FSimulationConfig& NewConfig)
	{
		if (!IsInitialized())
		{
			return false;
		}
		const int32 NewSegmentCount = FMath::Clamp(
			FMath::CeilToInt(NewConfig.RestLength / NewConfig.NodeSpacing),
			1,
			MaximumSegmentCount);
		const TArray<FParticle> OldParticles = Particles;
		Particles.SetNum(NewSegmentCount + 1);
		for (int32 Index = 0; Index <= NewSegmentCount; ++Index)
		{
			const double Alpha = static_cast<double>(Index) / static_cast<double>(NewSegmentCount);
			Particles[Index] = SampleParticle(OldParticles, Alpha);
			Particles[Index].MaterialCoordinate = Alpha * NewConfig.RestLength;
			Particles[Index].InverseMass = 1.0 / NewConfig.ParticleMass;
		}
		Particles[0].Mode = OldParticles[0].Mode;
		Particles[0].KinematicTarget = OldParticles[0].KinematicTarget;
		Particles.Last().Mode = OldParticles.Last().Mode;
		Particles.Last().KinematicTarget = OldParticles.Last().KinematicTarget;
		return ValidateState();
	}

	bool FSolver::ValidateState() const
	{
		if (Particles.Num() < 2)
		{
			return false;
		}
		for (const FParticle& Particle : Particles)
		{
			if (!IsFinite(Particle.Position) || !IsFinite(Particle.PreviousPosition)
				|| !IsFinite(Particle.Velocity) || !IsFinite(Particle.KinematicTarget)
				|| !FMath::IsFinite(Particle.InverseMass)
				|| !FMath::IsFinite(Particle.MaterialCoordinate)
				|| !FMath::IsFinite(Particle.EstimatedTension)
				|| !FMath::IsFinite(Particle.EstimatedNormalLoad))
			{
				return false;
			}
		}
		return true;
	}

	void FSolver::ApplyEndpointInput(const EEndpoint Endpoint, const FEndpointStepInput& EndpointInput)
	{
		FParticle& Particle = Particles[GetEndpointIndex(Endpoint, Particles.Num())];
		Particle.Mode = EndpointInput.Mode;
		Particle.KinematicTarget = EndpointInput.TargetPosition;
	}

	double FSolver::GetEffectiveInverseMass(const FParticle& Particle) const
	{
		return Particle.Mode == EParticleMode::Kinematic ? 0.0 : Particle.InverseMass;
	}

	void FSolver::ProjectDistanceConstraint(const int32 FirstIndex, const int32 SecondIndex)
	{
		FParticle& First = Particles[FirstIndex];
		FParticle& Second = Particles[SecondIndex];
		const double FirstWeight = GetEffectiveInverseMass(First);
		const double SecondWeight = GetEffectiveInverseMass(Second);
		const double WeightSum = FirstWeight + SecondWeight;
		if (WeightSum <= 0.0)
		{
			return;
		}

		const FVector3d Difference = Second.Position - First.Position;
		const double Distance = Difference.Length();
		const FVector3d Direction = Distance > 1.e-12
			? Difference / Distance
			: ((FirstIndex & 1) == 0 ? FVector3d::UnitX() : -FVector3d::UnitX());
		// Macklin-style over-relaxation used by the talk: shorten the target by
		// omega instead of multiplying the entire correction by omega.
		const FVector3d Correction = Direction
			* (Distance - SolveSegmentLength / Config.DistanceOverRelaxation);
		First.Position += Correction * (FirstWeight / WeightSum);
		Second.Position -= Correction * (SecondWeight / WeightSum);
	}

	void FSolver::ProjectBendingConstraint(
		const int32 FirstIndex,
		const int32 MiddleIndex,
		const int32 LastIndex,
		const double IterationStrength)
	{
		FParticle& First = Particles[FirstIndex];
		FParticle& Middle = Particles[MiddleIndex];
		FParticle& Last = Particles[LastIndex];
		const FVector3d Baseline = Last.Position - First.Position;
		const double Span = Baseline.Length();
		if (Span <= 1.e-12)
		{
			return;
		}

		const FVector3d BaselineDirection = Baseline / Span;
		const double FirstSpan = FVector3d::DotProduct(Middle.Position - First.Position, BaselineDirection);
		const double LastSpan = Span - FirstSpan;
		const FVector3d BaselinePoint = First.Position + BaselineDirection * FirstSpan;
		const FVector3d Offset = Middle.Position - BaselinePoint;
		const double OffsetLength = Offset.Length();
		if (OffsetLength <= 1.e-12)
		{
			return;
		}

		const double FreeAngle = FMath::Clamp(
			Config.FreeBendAngleRadiansPerMeter * (SolveSegmentLength / 100.0),
			0.0,
			UE_PI);
		const double FreeOffset = SolveSegmentLength * FMath::Cos((UE_PI - FreeAngle) * 0.5);
		const double Error = FMath::Max(OffsetLength - FreeOffset, 0.0);
		if (Error <= 0.0)
		{
			return;
		}

		const double FirstWeight = GetEffectiveInverseMass(First);
		const double MiddleWeight = GetEffectiveInverseMass(Middle);
		const double LastWeight = GetEffectiveInverseMass(Last);
		const double Denominator =
			LastSpan * LastSpan * FirstWeight
			+ Span * Span * MiddleWeight
			+ FirstSpan * FirstSpan * LastWeight;
		if (Denominator <= 1.e-12)
		{
			return;
		}

		const FVector3d CorrectionDirection = Offset / OffsetLength;
		const double Magnitude = FMath::Clamp(IterationStrength, 0.0, 1.0) * Error * Span / Denominator;
		First.Position += CorrectionDirection * (LastSpan * FirstWeight * Magnitude);
		Middle.Position -= CorrectionDirection * (Span * MiddleWeight * Magnitude);
		Last.Position += CorrectionDirection * (FirstSpan * LastWeight * Magnitude);
	}

	double FSolver::ProjectContactConstraint(const FContactConstraint& Contact)
	{
		FParticle& Particle = Particles[Contact.ParticleIndex];
		const double Penetration = Contact.MinimumNormalCoordinate
			- FVector3d::DotProduct(Particle.Position, Contact.Normal);
		if (Penetration > 0.0)
		{
			Particle.Position += Contact.Normal * Penetration;
			return Penetration;
		}
		return 0.0;
	}

	void FSolver::ProjectParticleFriction(
		const int32 ParticleIndex,
		const TConstArrayView<FContactConstraint> Contacts,
		TArray<double>& AccumulatedStaticCorrections,
		const double DeltaTime)
	{
		if (!Config.bEnableFriction || !Particles.IsValidIndex(ParticleIndex)
			|| Particles[ParticleIndex].Mode != EParticleMode::Dynamic)
		{
			return;
		}

		TArray<FVector3d, TInlineAllocator<3>> Normals;
		FVector3d AverageAnchor = FVector3d::ZeroVector;
		int32 AnchorCount = 0;
		for (int32 ContactIndex = 0; ContactIndex < Contacts.Num(); ++ContactIndex)
		{
			const FContactConstraint& Contact = Contacts[ContactIndex];
			if (Contact.ParticleIndex != ParticleIndex
				|| !ActiveContacts.IsValidIndex(ContactIndex)
				|| !ActiveContacts[ContactIndex])
			{
				continue;
			}
			AddOrthonormalNormal(Contact.Normal, Normals);
			if (Contact.bHasFrictionAnchor)
			{
				AverageAnchor += Contact.FrictionAnchorPosition;
				++AnchorCount;
			}
		}
		if (Normals.IsEmpty() || AnchorCount == 0)
		{
			return;
		}

		// Static friction is positional (pull toward the step-start anchor, capped by
		// the Coulomb budget) and must not be gated by instantaneous velocity: the
		// per-step gravity impulse alone exceeds any reasonable speed threshold on a
		// non-flat contact, which would defeat static hold. The budget cap yields the
		// Static/Sliding split — a correction the budget can fully cover holds; beyond
		// it the node slides and dynamic velocity friction takes over.
		FParticle& Particle = Particles[ParticleIndex];
		AverageAnchor /= static_cast<double>(AnchorCount);
		const FVector3d TangentialOffset = RemoveNormalComponents(Particle.Position - AverageAnchor, Normals);
		const double Distance = TangentialOffset.Length();
		if (Distance <= 1.e-12)
		{
			return;
		}

		const double Mass = Particle.InverseMass > 0.0 ? 1.0 / Particle.InverseMass : 0.0;
		if (Mass <= 0.0)
		{
			return;
		}
		const double StepBudget = Config.StaticFrictionCoefficient
			* Particle.EstimatedNormalLoad / Mass * 100.0 * DeltaTime * DeltaTime;
		const double RemainingBudget = FMath::Max(
			StepBudget - AccumulatedStaticCorrections[ParticleIndex],
			0.0);
		const double CorrectionDistance = FMath::Min(Distance, RemainingBudget);
		if (CorrectionDistance <= 0.0)
		{
			return;
		}
		Particle.Position -= TangentialOffset * (CorrectionDistance / Distance);
		AccumulatedStaticCorrections[ParticleIndex] += CorrectionDistance;
		ParticleStaticFrictionCorrections[ParticleIndex] += CorrectionDistance;
	}

	void FSolver::ProjectGuideConstraint(const FGuideConstraint& Guide, const double IterationStrength)
	{
		FParticle& Particle = Particles[Guide.ParticleIndex];
		const FVector3d Offset = Particle.Position - Guide.TargetPosition;
		const double Distance = Offset.Length();
		if (Distance <= Guide.MaximumDistance || Distance <= 1.e-12)
		{
			return;
		}
		Particle.Position -= Offset * (((Distance - Guide.MaximumDistance) / Distance) * IterationStrength);
	}

	void FSolver::ApplyContactVelocityResponse(
		const TArray<FContactConstraint>& Contacts,
		const double DeltaTime)
	{
		for (int32 ParticleIndex = 0; ParticleIndex < Particles.Num(); ++ParticleIndex)
		{
			FParticle& Particle = Particles[ParticleIndex];
			if (Particle.Mode != EParticleMode::Dynamic)
			{
				continue;
			}
			TArray<FVector3d, TInlineAllocator<3>> Normals;
			FVector3d SurfaceVelocity = FVector3d::ZeroVector;
			int32 ActiveCount = 0;
			for (int32 ContactIndex = 0; ContactIndex < Contacts.Num(); ++ContactIndex)
			{
				if (Contacts[ContactIndex].ParticleIndex != ParticleIndex
					|| !ActiveContacts.IsValidIndex(ContactIndex)
					|| !ActiveContacts[ContactIndex])
				{
					continue;
				}
				AddOrthonormalNormal(Contacts[ContactIndex].Normal, Normals);
				SurfaceVelocity += Contacts[ContactIndex].SurfaceVelocity;
				++ActiveCount;
			}
			if (ActiveCount == 0 || Normals.IsEmpty())
			{
				continue;
			}
			SurfaceVelocity /= static_cast<double>(ActiveCount);
			FVector3d RelativeVelocity = Particle.Velocity - SurfaceVelocity;
			for (const FVector3d& Normal : Normals)
			{
				const double NormalSpeed = FVector3d::DotProduct(RelativeVelocity, Normal);
				if (NormalSpeed < 0.0)
				{
					RelativeVelocity -= Normal * NormalSpeed;
				}
			}
			FVector3d TangentialVelocity = RemoveNormalComponents(RelativeVelocity, Normals);
			const double TangentialSpeed = TangentialVelocity.Length();
			if (Config.bEnableFriction && TangentialSpeed > Config.StaticFrictionSpeedThreshold)
			{
				const double Mass = Particle.InverseMass > 0.0 ? 1.0 / Particle.InverseMass : 0.0;
				const double SpeedChange = Mass > 0.0
					? Config.DynamicFrictionCoefficient * Particle.EstimatedNormalLoad / Mass * 100.0 * DeltaTime
					: 0.0;
				const FVector3d ReducedTangent = TangentialVelocity
					* (FMath::Max(TangentialSpeed - SpeedChange, 0.0) / TangentialSpeed);
				ParticleDynamicFrictionVelocityChanges[ParticleIndex] = FMath::Min(
					TangentialSpeed,
					SpeedChange);
				RelativeVelocity += ReducedTangent - TangentialVelocity;
			}
			Particle.Velocity = SurfaceVelocity + RelativeVelocity;
		}
	}

	void FSolver::UpdateContactLoads(const TArray<FContactConstraint>& Contacts)
	{
		const int32 ParticleCount = Particles.Num();
		TArray<bool> HasActiveContact;
		HasActiveContact.Init(false, ParticleCount);
		TArray<FVector3d> AverageNormals;
		AverageNormals.Init(FVector3d::ZeroVector, ParticleCount);
		for (int32 ContactIndex = 0; ContactIndex < Contacts.Num(); ++ContactIndex)
		{
			if (ActiveContacts.IsValidIndex(ContactIndex) && ActiveContacts[ContactIndex])
			{
				const int32 ParticleIndex = Contacts[ContactIndex].ParticleIndex;
				HasActiveContact[ParticleIndex] = true;
				AverageNormals[ParticleIndex] += Contacts[ContactIndex].Normal;
			}
		}

		const double ParticleWeight = Config.ParticleMass * Config.Gravity.Length() / 100.0;
		const double UnsupportedBendThreshold = FMath::DegreesToRadians(30.0);
		TArray<double> FromStart;
		TArray<double> FromEnd;
		FromStart.Init(0.0, ParticleCount);
		FromEnd.Init(0.0, ParticleCount);
		for (int32 Index = 1; Index < ParticleCount; ++Index)
		{
			FromStart[Index] = FromStart[Index - 1] + ParticleWeight;
			if (Index + 1 < ParticleCount && !HasActiveContact[Index])
			{
				const FVector3d A = (Particles[Index - 1].Position - Particles[Index].Position).GetSafeNormal();
				const FVector3d B = (Particles[Index + 1].Position - Particles[Index].Position).GetSafeNormal();
				const double Bend = FMath::Acos(FMath::Clamp(FVector3d::DotProduct(A, B), -1.0, 1.0));
				if (UE_PI - Bend > UnsupportedBendThreshold)
				{
					FromStart[Index] = 0.0;
				}
			}
		}
		for (int32 Index = ParticleCount - 2; Index >= 0; --Index)
		{
			FromEnd[Index] = FromEnd[Index + 1] + ParticleWeight;
			if (Index > 0 && !HasActiveContact[Index])
			{
				const FVector3d A = (Particles[Index - 1].Position - Particles[Index].Position).GetSafeNormal();
				const FVector3d B = (Particles[Index + 1].Position - Particles[Index].Position).GetSafeNormal();
				const double Bend = FMath::Acos(FMath::Clamp(FVector3d::DotProduct(A, B), -1.0, 1.0));
				if (UE_PI - Bend > UnsupportedBendThreshold)
				{
					FromEnd[Index] = 0.0;
				}
			}
		}

		const bool bStartKinematic = Particles[0].Mode == EParticleMode::Kinematic;
		const bool bEndKinematic = Particles.Last().Mode == EParticleMode::Kinematic;
		for (int32 Index = 0; Index < ParticleCount; ++Index)
		{
			double Tension = 0.0;
			if (bStartKinematic && !bEndKinematic)
			{
				Tension = FromEnd[Index];
			}
			else if (!bStartKinematic && bEndKinematic)
			{
				Tension = FromStart[Index];
			}
			else
			{
				Tension = FMath::Min(FromStart[Index], FromEnd[Index]);
			}
			Particles[Index].EstimatedTension = Tension;
		}
		for (int32 Index = 0; Index < ParticleCount; ++Index)
		{
			const double Tension = Particles[Index].EstimatedTension;
			Particles[Index].EstimatedNormalLoad = 0.0;
			if (!HasActiveContact[Index])
			{
				continue;
			}
			const FVector3d Normal = AverageNormals[Index].GetSafeNormal();
			const double GravityLoad = Config.ParticleMass
				* FMath::Max(-FVector3d::DotProduct(Config.Gravity / 100.0, Normal), 0.0);
			FVector3d BendForce = FVector3d::ZeroVector;
			if (Index > 0)
			{
				BendForce += (Particles[Index - 1].Position - Particles[Index].Position).GetSafeNormal()
					* 0.5 * (Tension + Particles[Index - 1].EstimatedTension);
			}
			if (Index + 1 < ParticleCount)
			{
				BendForce += (Particles[Index + 1].Position - Particles[Index].Position).GetSafeNormal()
					* 0.5 * (Tension + Particles[Index + 1].EstimatedTension);
			}
			Particles[Index].EstimatedNormalLoad = GravityLoad
				+ FMath::Max(-FVector3d::DotProduct(BendForce, Normal), 0.0);
		}

		LastContactDiagnostics.Reset(Contacts.Num());
		for (int32 ContactIndex = 0; ContactIndex < Contacts.Num(); ++ContactIndex)
		{
			const FContactConstraint& Contact = Contacts[ContactIndex];
			FContactDiagnostic& Diagnostic = LastContactDiagnostics.AddDefaulted_GetRef();
			Diagnostic.FeatureId = Contact.FeatureId;
			Diagnostic.ParticleIndex = Contact.ParticleIndex;
			Diagnostic.Normal = Contact.Normal;
			Diagnostic.bProjected = ProjectedContacts.IsValidIndex(ContactIndex) && ProjectedContacts[ContactIndex];
			Diagnostic.NormalCorrection = ContactNormalCorrections.IsValidIndex(ContactIndex)
				? ContactNormalCorrections[ContactIndex] : 0.0;
			Diagnostic.StaticFrictionCorrection = ParticleStaticFrictionCorrections.IsValidIndex(Contact.ParticleIndex)
				? ParticleStaticFrictionCorrections[Contact.ParticleIndex] : 0.0;
			Diagnostic.DynamicFrictionVelocityChange = ParticleDynamicFrictionVelocityChanges.IsValidIndex(Contact.ParticleIndex)
				? ParticleDynamicFrictionVelocityChanges[Contact.ParticleIndex] : 0.0;
			Diagnostic.EstimatedTension = Particles[Contact.ParticleIndex].EstimatedTension;
			Diagnostic.EstimatedNormalLoad = Particles[Contact.ParticleIndex].EstimatedNormalLoad;
			const bool bActive = ActiveContacts.IsValidIndex(ContactIndex) && ActiveContacts[ContactIndex];
			Diagnostic.bStaticAnchorHeld = bActive && Contact.bHasFrictionAnchor;
		}
	}

	void FSolver::SanitizeContacts(
		const TConstArrayView<FParticle> InParticles,
		TArray<FContactConstraint>& Contacts)
	{
		for (int32 Index = Contacts.Num() - 1; Index >= 0; --Index)
		{
			FContactConstraint& Contact = Contacts[Index];
			if (!InParticles.IsValidIndex(Contact.ParticleIndex)
				|| InParticles[Contact.ParticleIndex].Mode != EParticleMode::Dynamic
				|| !IsFinite(Contact.Normal)
				|| !FMath::IsFinite(Contact.MinimumNormalCoordinate)
				|| !IsFinite(Contact.SurfaceVelocity)
				|| (Contact.bHasFrictionAnchor && !IsFinite(Contact.FrictionAnchorPosition)))
			{
				Contacts.RemoveAt(Index, 1, EAllowShrinking::No);
				continue;
			}
			Contact.Normal = Contact.Normal.GetSafeNormal();
			if (Contact.Normal.IsNearlyZero())
			{
				Contacts.RemoveAt(Index, 1, EAllowShrinking::No);
				continue;
			}
			bool bHasKinematicReference = false;
			bool bPlaneIsReachable = false;
			for (const int32 EndpointIndex : {0, InParticles.Num() - 1})
			{
				const FParticle& EndpointParticle = InParticles[EndpointIndex];
				if (EndpointParticle.Mode != EParticleMode::Kinematic)
				{
					continue;
				}
				bHasKinematicReference = true;
				const double AvailableArcLength = FMath::Abs(
					InParticles[Contact.ParticleIndex].MaterialCoordinate
						- EndpointParticle.MaterialCoordinate);
				const double RequiredNormalDistance = FMath::Max(
					Contact.MinimumNormalCoordinate
						- FVector3d::DotProduct(EndpointParticle.Position, Contact.Normal),
					0.0);
				if (RequiredNormalDistance <= AvailableArcLength + FeasibilityTolerance)
				{
					bPlaneIsReachable = true;
					break;
				}
			}
			if (bHasKinematicReference && !bPlaneIsReachable)
			{
				Contacts.RemoveAt(Index, 1, EAllowShrinking::No);
				continue;
			}
		}
		Contacts.StableSort([](const FContactConstraint& First, const FContactConstraint& Second)
		{
			if (First.ParticleIndex != Second.ParticleIndex)
			{
				return First.ParticleIndex < Second.ParticleIndex;
			}
			return First.FeatureId < Second.FeatureId;
		});
	}

	double FSolver::CalculateMaximumSegmentError() const
	{
		double MaximumError = 0.0;
		for (int32 SegmentIndex = 0; SegmentIndex + 1 < Particles.Num(); ++SegmentIndex)
		{
			MaximumError = FMath::Max(
				MaximumError,
				FMath::Abs(FVector3d::Distance(
					Particles[SegmentIndex].Position,
					Particles[SegmentIndex + 1].Position) - SolveSegmentLength));
		}
		return MaximumError;
	}

	double FSolver::CalculateMaximumPenetration(const TArray<FContactConstraint>& Contacts) const
	{
		double MaximumPenetration = 0.0;
		for (const FContactConstraint& Contact : Contacts)
		{
			MaximumPenetration = FMath::Max(
				MaximumPenetration,
				Contact.MinimumNormalCoordinate
					- FVector3d::DotProduct(Particles[Contact.ParticleIndex].Position, Contact.Normal));
		}
		return FMath::Max(MaximumPenetration, 0.0);
	}

	double FSolver::CalculateMaximumGuideError(const TArray<FGuideConstraint>& Guides) const
	{
		double MaximumError = 0.0;
		for (const FGuideConstraint& Guide : Guides)
		{
			MaximumError = FMath::Max(
				MaximumError,
				FVector3d::Distance(Particles[Guide.ParticleIndex].Position, Guide.TargetPosition)
					- Guide.MaximumDistance);
		}
		return FMath::Max(MaximumError, 0.0);
	}

	void FSolver::UpdateStepResult(
		const ESimulationStatus Status,
		const double EndpointDistance,
		const TArray<FContactConstraint>& Contacts,
		const int32 RefreshedContactCount,
		const TArray<FGuideConstraint>& Guides)
	{
		LastStepResult.Status = Status;
		LastStepResult.StepIndex = StepIndex;
		LastStepResult.ParticleCount = Particles.Num();
		LastStepResult.ConstraintIterations = Config.ConstraintIterations;
		LastStepResult.ContactCount = Contacts.Num();
		LastStepResult.RefreshedContactCount = RefreshedContactCount;
		LastStepResult.GuideConstraintCount = Guides.Num();
		LastStepResult.RestLength = Config.RestLength;
		LastStepResult.EffectiveSolveLength = EffectiveSolveLength;
		LastStepResult.EndpointDistance = EndpointDistance;
		LastStepResult.StrainRatio = Config.RestLength > 0.0
			? FMath::Max(EndpointDistance / Config.RestLength - 1.0, 0.0)
			: 0.0;
		LastStepResult.MaximumSegmentError = CalculateMaximumSegmentError();
		LastStepResult.MaximumPenetration = CalculateMaximumPenetration(Contacts);
		LastStepResult.MaximumGuideError = CalculateMaximumGuideError(Guides);
		double SpeedSquaredSum = 0.0;
		LastStepResult.MaximumParticleSpeed = 0.0;
		LastStepResult.MaximumEstimatedTension = 0.0;
		LastStepResult.MaximumEstimatedNormalLoad = 0.0;
		for (const FParticle& Particle : Particles)
		{
			const double SpeedSquared = Particle.Velocity.SquaredLength();
			SpeedSquaredSum += SpeedSquared;
			LastStepResult.MaximumParticleSpeed = FMath::Max(
				LastStepResult.MaximumParticleSpeed,
				FMath::Sqrt(SpeedSquared));
			LastStepResult.MaximumEstimatedTension = FMath::Max(
				LastStepResult.MaximumEstimatedTension,
				Particle.EstimatedTension);
			LastStepResult.MaximumEstimatedNormalLoad = FMath::Max(
				LastStepResult.MaximumEstimatedNormalLoad,
				Particle.EstimatedNormalLoad);
		}
		LastStepResult.ProjectedContactCount = 0;
		LastStepResult.StaticFrictionAnchorCount = 0;
		for (const FContactDiagnostic& Diagnostic : LastContactDiagnostics)
		{
			LastStepResult.ProjectedContactCount += Diagnostic.bProjected ? 1 : 0;
			LastStepResult.StaticFrictionAnchorCount += Diagnostic.bStaticAnchorHeld ? 1 : 0;
		}
		LastStepResult.RmsParticleSpeed = Particles.IsEmpty()
			? 0.0
			: FMath::Sqrt(SpeedSquaredSum / static_cast<double>(Particles.Num()));
	}
}
