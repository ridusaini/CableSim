#include "CableSimSolver.h"

namespace CableSim
{
	namespace
	{
		constexpr double SmallNumber = 1.e-9;

		bool NearlyEqual(const double A, const double B, const double Tolerance)
		{
			return FMath::Abs(A - B) <= Tolerance;
		}

		FVector3d RemoveNormalComponents(
			const FVector3d& Value,
			const TConstArrayView<FVector3d> OrthonormalNormals)
		{
			FVector3d Result = Value;
			for (const FVector3d& Normal : OrthonormalNormals)
			{
				Result -= Normal * FVector3d::DotProduct(Result, Normal);
			}
			return Result;
		}

		void AddOrthonormalNormal(
			const FVector3d& Candidate,
			TArray<FVector3d, TInlineAllocator<3>>& Normals)
		{
			FVector3d Orthogonal = RemoveNormalComponents(Candidate.GetSafeNormal(), Normals);
			if (Orthogonal.Normalize())
			{
				Normals.Add(Orthogonal);
			}
		}

		FVector3d SamplePoint(const TArray<FParticle>& Particles, const FContactConstraint& Contact, const bool bPrevious)
		{
			const double Alpha = Particles.IsValidIndex(Contact.ParticleB)
				? FMath::Clamp(Contact.SegmentAlpha, 0.0, 1.0)
				: 0.0;
			const FVector3d A = bPrevious
				? Particles[Contact.ParticleA].PreviousPosition
				: Particles[Contact.ParticleA].Position;
			if (!Particles.IsValidIndex(Contact.ParticleB))
			{
				return A;
			}
			const FVector3d B = bPrevious
				? Particles[Contact.ParticleB].PreviousPosition
				: Particles[Contact.ParticleB].Position;
			return FMath::Lerp(A, B, Alpha);
		}
	}

	bool FCollisionFeatureId::IsValid() const
	{
		return ObjectToken != 0 && ShapeIndex != INDEX_NONE
			&& Type != ECollisionFeatureType::None && Index0 != INDEX_NONE;
	}

	bool FCollisionFeatureId::Less(const FCollisionFeatureId& A, const FCollisionFeatureId& B)
	{
		if (A.ObjectToken != B.ObjectToken) return A.ObjectToken < B.ObjectToken;
		if (A.ShapeIndex != B.ShapeIndex) return A.ShapeIndex < B.ShapeIndex;
		if (A.Type != B.Type) return static_cast<uint8>(A.Type) < static_cast<uint8>(B.Type);
		return A.Index0 != B.Index0 ? A.Index0 < B.Index0 : A.Index1 < B.Index1;
	}

	uint32 GetTypeHash(const FCollisionFeatureId& Id)
	{
		uint32 Hash = ::GetTypeHash(Id.ObjectToken);
		Hash = HashCombineFast(Hash, ::GetTypeHash(Id.ShapeIndex));
		Hash = HashCombineFast(Hash, ::GetTypeHash(static_cast<uint8>(Id.Type)));
		Hash = HashCombineFast(Hash, ::GetTypeHash(Id.Index0));
		return HashCombineFast(Hash, ::GetTypeHash(Id.Index1));
	}

	bool FSimulationConfig::Equals(const FSimulationConfig& Other, const double Tolerance) const
	{
		return NearlyEqual(Length, Other.Length, Tolerance)
			&& NearlyEqual(SegmentLength, Other.SegmentLength, Tolerance)
			&& MaximumParticles == Other.MaximumParticles
			&& NearlyEqual(LinearDensity, Other.LinearDensity, Tolerance)
			&& Gravity.Equals(Other.Gravity, Tolerance)
			&& NearlyEqual(VelocityDamping, Other.VelocityDamping, Tolerance)
			&& NearlyEqual(DistanceCompliance, Other.DistanceCompliance, Tolerance)
			&& NearlyEqual(DistanceRelaxation, Other.DistanceRelaxation, Tolerance)
			&& NearlyEqual(BendStrength, Other.BendStrength, Tolerance)
			&& NearlyEqual(FreeBendDegreesPerMeter, Other.FreeBendDegreesPerMeter, Tolerance)
			&& NearlyEqual(DrivenEndpointMaximumSpeed, Other.DrivenEndpointMaximumSpeed, Tolerance)
			&& SolverIterations == Other.SolverIterations
			&& MultigridIterations == Other.MultigridIterations
			&& MultigridMinimumParticles == Other.MultigridMinimumParticles
			&& NearlyEqual(StaticFriction, Other.StaticFriction, Tolerance)
			&& NearlyEqual(DynamicFriction, Other.DynamicFriction, Tolerance)
			&& NearlyEqual(ConstantFrictionSpeedReduction, Other.ConstantFrictionSpeedReduction, Tolerance)
			&& NearlyEqual(StaticFrictionDeadZone, Other.StaticFrictionDeadZone, Tolerance)
			&& NearlyEqual(FrictionFadeStartSpeed, Other.FrictionFadeStartSpeed, Tolerance)
			&& NearlyEqual(FrictionFadeEndSpeed, Other.FrictionFadeEndSpeed, Tolerance)
			&& NearlyEqual(MaximumContactCorrection, Other.MaximumContactCorrection, Tolerance);
	}

	bool FSolver::Initialize(const FVector3d& Start, const FVector3d& End, const FSimulationConfig& InConfig)
	{
		Reset();
		if (!IsFinite(Start) || !IsFinite(End) || !IsValidConfig(InConfig))
		{
			LastResult.Status = ESimulationStatus::InvalidConfiguration;
			return false;
		}
		Config = InConfig;
		const int32 SegmentCount = FMath::Max(FMath::CeilToInt(Config.Length / Config.SegmentLength), 1);
		if (SegmentCount + 1 > Config.MaximumParticles)
		{
			LastResult.Status = ESimulationStatus::ParticleBudgetExceeded;
			return false;
		}
		Particles.SetNum(SegmentCount + 1);
		for (int32 Index = 0; Index <= SegmentCount; ++Index)
		{
			const double Alpha = static_cast<double>(Index) / SegmentCount;
			FParticle& Particle = Particles[Index];
			// Match the original production-facing behavior: all material exists
			// immediately, but its initial world shape is the endpoint chord.
			Particle.Position = FMath::Lerp(Start, End, Alpha);
			Particle.PreviousPosition = Particle.Position;
			Particle.Velocity = FVector3d::ZeroVector;
			Particle.MaterialCoordinate = Alpha * Config.Length;
			Particle.EstimatedTension = 0.0;
			Particle.EstimatedNormalLoad = 0.0;
		}
		RecalculateParticleMasses();
		DistanceLambdas.Init(0.0, SegmentCount);
		LastResult.Status = ESimulationStatus::Ready;
		LastResult.ParticleCount = Particles.Num();
		LastValidState = CaptureState();
		return true;
	}

	void FSolver::Reset()
	{
		Particles.Reset();
		Config = FSimulationConfig{};
		LastResult = FStepResult{};
		LastValidState = FStateSnapshot{};
		DistanceLambdas.Reset();
		ParticleFrictionCorrections.Reset();
		StepIndex = 0;
		bStepActive = false;
	}

	bool FSolver::ApplyConfig(const FSimulationConfig& InConfig)
	{
		if (!IsValidConfig(InConfig)) return false;
		if (IsInitialized() && InConfig.MaximumParticles < Particles.Num())
		{
			return false;
		}
		if (IsInitialized() && !NearlyEqual(Config.SegmentLength, InConfig.SegmentLength, 1.e-9)
			&& !RemeshPreservingState(InConfig.SegmentLength, InConfig.MaximumParticles))
		{
			return false;
		}
		const double ActiveLength = GetActiveLength();
		Config = InConfig;
		if (IsInitialized()) Config.Length = ActiveLength;
		RecalculateParticleMasses();
		return true;
	}

	double FSolver::GetActiveLength() const
	{
		return Particles.Num() >= 2
			? Particles.Last().MaterialCoordinate - Particles[0].MaterialCoordinate
			: 0.0;
	}

	FParticle FSolver::InterpolateParticle(const FParticle& A, const FParticle& B, const double Alpha)
	{
		FParticle Result;
		Result.Position = FMath::Lerp(A.Position, B.Position, Alpha);
		Result.PreviousPosition = FMath::Lerp(A.PreviousPosition, B.PreviousPosition, Alpha);
		Result.Velocity = FMath::Lerp(A.Velocity, B.Velocity, Alpha);
		Result.MaterialCoordinate = FMath::Lerp(A.MaterialCoordinate, B.MaterialCoordinate, Alpha);
		Result.EstimatedTension = FMath::Lerp(A.EstimatedTension, B.EstimatedTension, Alpha);
		Result.EstimatedNormalLoad = FMath::Lerp(A.EstimatedNormalLoad, B.EstimatedNormalLoad, Alpha);
		return Result;
	}

	bool FSolver::SetActiveLength(const double NewLength, const ELengthChangeOrigin Origin)
	{
		if (!IsInitialized() || bStepActive || !FMath::IsFinite(NewLength) || NewLength < 1.0)
		{
			return false;
		}
		const double CurrentLength = GetActiveLength();
		const double Delta = NewLength - CurrentLength;
		if (FMath::Abs(Delta) <= SmallNumber) return true;
		const double StartShare = Origin == ELengthChangeOrigin::Start ? 1.0
			: (Origin == ELengthChangeOrigin::Both ? 0.5 : 0.0);
		const double EndShare = 1.0 - StartShare;
		if (Delta > 0.0)
		{
			if (FMath::CeilToInt(NewLength / Config.SegmentLength) + 1 > Config.MaximumParticles) return false;
			AddLengthAtEndpoint(Delta * StartShare, EEndpoint::Start);
			AddLengthAtEndpoint(Delta * EndShare, EEndpoint::End);
		}
		else
		{
			RemoveLengthAtEndpoint(-Delta * StartShare, EEndpoint::Start);
			RemoveLengthAtEndpoint(-Delta * EndShare, EEndpoint::End);
		}
		Config.Length = GetActiveLength();
		DistanceLambdas.Init(0.0, Particles.Num() - 1);
		ParticleFrictionCorrections.Init(0.0, Particles.Num());
		RecalculateParticleMasses();
		LastValidState = CaptureState();
		return NearlyEqual(Config.Length, NewLength, 1.e-6);
	}

	void FSolver::AddLengthAtEndpoint(const double Amount, const EEndpoint Endpoint)
	{
		if (Amount <= SmallNumber) return;
		if (Endpoint == EEndpoint::Start)
		{
			for (int32 Index = 1; Index < Particles.Num(); ++Index) Particles[Index].MaterialCoordinate += Amount;
		}
		else
		{
			Particles.Last().MaterialCoordinate += Amount;
		}
		NormalizeEndpointResolution(Endpoint);
	}

	void FSolver::RemoveLengthAtEndpoint(double Amount, const EEndpoint Endpoint)
	{
		Amount = FMath::Min(Amount, GetActiveLength() - 1.0);
		while (Amount > SmallNumber && Particles.Num() >= 2)
		{
			if (Endpoint == EEndpoint::Start)
			{
				const double EdgeLength = Particles[1].MaterialCoordinate;
				if (Particles.Num() > 2 && Amount >= EdgeLength - SmallNumber)
				{
					Amount -= EdgeLength;
					Particles.RemoveAt(1, 1, EAllowShrinking::No);
					for (int32 Index = 1; Index < Particles.Num(); ++Index) Particles[Index].MaterialCoordinate -= EdgeLength;
					continue;
				}
				const double Applied = FMath::Min(Amount, EdgeLength - (Particles.Num() == 2 ? 1.0 : 1.e-4));
				for (int32 Index = 1; Index < Particles.Num(); ++Index) Particles[Index].MaterialCoordinate -= Applied;
				Amount -= Applied;
			}
			else
			{
				const int32 Last = Particles.Num() - 1;
				const double EdgeLength = Particles[Last].MaterialCoordinate - Particles[Last - 1].MaterialCoordinate;
				if (Particles.Num() > 2 && Amount >= EdgeLength - SmallNumber)
				{
					Amount -= EdgeLength;
					Particles.RemoveAt(Last - 1, 1, EAllowShrinking::No);
					Particles.Last().MaterialCoordinate -= EdgeLength;
					continue;
				}
				const double Applied = FMath::Min(Amount, EdgeLength - (Particles.Num() == 2 ? 1.0 : 1.e-4));
				Particles.Last().MaterialCoordinate -= Applied;
				Amount -= Applied;
			}
		}
		NormalizeEndpointResolution(Endpoint);
	}

	void FSolver::NormalizeEndpointResolution(const EEndpoint Endpoint)
	{
		const double Spacing = FMath::Max(Config.SegmentLength, 1.0);
		while (Particles.Num() < Config.MaximumParticles)
		{
			const int32 A = Endpoint == EEndpoint::Start ? 0 : Particles.Num() - 2;
			const int32 B = A + 1;
			const double Rest = Particles[B].MaterialCoordinate - Particles[A].MaterialCoordinate;
			if (Rest <= 1.5 * Spacing) break;
			const double Alpha = Endpoint == EEndpoint::Start ? Spacing / Rest : 1.0 - Spacing / Rest;
			FParticle Inserted = InterpolateParticle(Particles[A], Particles[B], Alpha);
			Particles.Insert(Inserted, B);
		}
		if (Particles.Num() <= 2) return;
		const int32 A = Endpoint == EEndpoint::Start ? 0 : Particles.Num() - 2;
		const int32 B = A + 1;
		const double Rest = Particles[B].MaterialCoordinate - Particles[A].MaterialCoordinate;
		if (Rest >= 0.5 * Spacing) return;
		if (Endpoint == EEndpoint::Start) Particles.RemoveAt(1, 1, EAllowShrinking::No);
		else Particles.RemoveAt(Particles.Num() - 2, 1, EAllowShrinking::No);
	}

	bool FSolver::RemeshPreservingState(const double NewSegmentLength, const int32 NewMaximumParticles)
	{
		if (!IsInitialized() || bStepActive || !FMath::IsFinite(NewSegmentLength) || NewSegmentLength <= 0.0)
			return false;
		const double ActiveLength = GetActiveLength();
		const int32 SegmentCount = FMath::Max(FMath::CeilToInt(ActiveLength / NewSegmentLength), 1);
		if (SegmentCount + 1 > NewMaximumParticles) return false;
		const TArray<FParticle> Old = Particles;
		Particles.SetNum(SegmentCount + 1);
		int32 OldSegment = 0;
		for (int32 Index = 0; Index <= SegmentCount; ++Index)
		{
			const double Coordinate = ActiveLength * static_cast<double>(Index) / SegmentCount;
			while (OldSegment + 2 < Old.Num() && Old[OldSegment + 1].MaterialCoordinate < Coordinate) ++OldSegment;
			const double Span = Old[OldSegment + 1].MaterialCoordinate - Old[OldSegment].MaterialCoordinate;
			const double Alpha = Span > SmallNumber ? (Coordinate - Old[OldSegment].MaterialCoordinate) / Span : 0.0;
			Particles[Index] = InterpolateParticle(Old[OldSegment], Old[OldSegment + 1], FMath::Clamp(Alpha, 0.0, 1.0));
			Particles[Index].MaterialCoordinate = Coordinate;
		}
		Config.SegmentLength = NewSegmentLength;
		Config.MaximumParticles = NewMaximumParticles;
		DistanceLambdas.Init(0.0, SegmentCount);
		ParticleFrictionCorrections.Init(0.0, Particles.Num());
		RecalculateParticleMasses();
		LastValidState = CaptureState();
		return ValidateState();
	}

	void FSolver::RecalculateParticleMasses()
	{
		if (Particles.Num() < 2) return;
		const double Density = FMath::Max(Config.LinearDensity, 1.e-9);
		for (int32 Index = 0; Index < Particles.Num(); ++Index)
		{
			double SupportedLength = 0.0;
			if (Index > 0) SupportedLength += 0.5 * SegmentRestLength(Index - 1);
			if (Index + 1 < Particles.Num()) SupportedLength += 0.5 * SegmentRestLength(Index);
			FParticle& Particle = Particles[Index];
			Particle.Mass = FMath::Max(Density * SupportedLength, 1.e-9);
			Particle.InverseMass = 1.0 / Particle.Mass;
		}
	}

	bool FSolver::BeginStep(const FStepInput& Input)
	{
		if (!IsInitialized()) { LastResult.Status = ESimulationStatus::Uninitialized; return false; }
		if (bStepActive || !FMath::IsFinite(Input.DeltaTime) || Input.DeltaTime <= 0.0
			|| !IsFinite(Input.StartEndpoint.TargetPosition) || !IsFinite(Input.EndEndpoint.TargetPosition)
			|| !IsFinite(Input.StartEndpoint.TargetVelocity) || !IsFinite(Input.EndEndpoint.TargetVelocity))
		{
			LastResult.Status = ESimulationStatus::InvalidConfiguration;
			return false;
		}
		LastValidState = CaptureState();
		const double Damping = FMath::Clamp(1.0 - Config.VelocityDamping, 0.0, 1.0);
		for (int32 Index = 0; Index < Particles.Num(); ++Index)
		{
			FParticle& Particle = Particles[Index];
			Particle.PreviousPosition = Particle.Position;
			const FEndpointInput* EndpointInput = Index == 0 ? &Input.StartEndpoint
				: (Index == Particles.Num() - 1 ? &Input.EndEndpoint : nullptr);
			if (EndpointInput && EndpointInput->State != EEndpointState::Free)
			{
				continue;
			}
			Particle.Velocity = Particle.Velocity * Damping + Config.Gravity * Input.DeltaTime;
			Particle.Position += Particle.Velocity * Input.DeltaTime;
		}
		auto ApplyFixedEndpoint = [this](const EEndpoint Endpoint, const FEndpointInput& EndpointInput)
		{
			if (EndpointInput.State != EEndpointState::Fixed) return;
			FParticle& Particle = Particles[GetEndpointIndex(Endpoint)];
			Particle.Position = EndpointInput.TargetPosition;
			Particle.Velocity = EndpointInput.TargetVelocity;
		};
		ApplyFixedEndpoint(EEndpoint::Start, Input.StartEndpoint);
		ApplyFixedEndpoint(EEndpoint::End, Input.EndEndpoint);
		ProjectEndpointDrive(EEndpoint::Start, Input.StartEndpoint, Input.DeltaTime);
		ProjectEndpointDrive(EEndpoint::End, Input.EndEndpoint, Input.DeltaTime);
		DistanceLambdas.Init(0.0, Particles.Num() - 1);
		ParticleFrictionCorrections.Init(0.0, Particles.Num());
		bStepActive = true;
		return true;
	}

	void FSolver::SolveBatch(const FStepInput& Input, TArrayView<FContactConstraint> Contacts, const int32 Iterations)
	{
		if (!bStepActive) return;
		const int32 SafeIterations = FMath::Clamp(Iterations, 0, 256);
		SolveMultigrid(Input, Contacts);
		const double PerIterationBend = SafeIterations > 0
			? 1.0 - FMath::Pow(1.0 - FMath::Clamp(Config.BendStrength, 0.0, 1.0), 1.0 / SafeIterations)
			: 0.0;
		for (int32 Iteration = 0; Iteration < SafeIterations; ++Iteration)
		{
			if ((Iteration & 1) == 0)
			{
				for (int32 Segment = 0; Segment + 1 < Particles.Num(); ++Segment) ProjectDistance(Segment, Input);
			}
			else
			{
				for (int32 Segment = Particles.Num() - 2; Segment >= 0; --Segment) ProjectDistance(Segment, Input);
			}
			for (FContactConstraint& Contact : Contacts) ProjectContact(Contact, Input);
			for (int32 First = 0; First + 2 < Particles.Num(); ++First) ProjectBend(First, Input, PerIterationBend);
			for (int32 ParticleIndex = 0; ParticleIndex < Particles.Num(); ++ParticleIndex)
				ProjectParticleFriction(ParticleIndex, Contacts, Input);
		}
		// A local endpoint/contact disturbance needs O(N) Gauss-Seidel sweeps to
		// travel through a long slack chain; a coarse endpoint chord cannot see it.
		// Keep the authored iteration count as the normal cost, then add a bounded
		// distance/contact-only convergence tail only while an inextensible cable
		// is still visibly strained.
		if (Config.DistanceCompliance <= SmallNumber)
		{
			const int32 MaximumTailIterations = FMath::Min(Particles.Num() * 4, 512);
			for (int32 Iteration = 0; Iteration < MaximumTailIterations; ++Iteration)
			{
				double MaximumStrain = 0.0;
				for (int32 Segment = 0; Segment + 1 < Particles.Num(); ++Segment)
				{
					const double Rest = SegmentRestLength(Segment);
					MaximumStrain = FMath::Max(MaximumStrain,
						(FVector3d::Distance(Particles[Segment].Position, Particles[Segment + 1].Position) - Rest)
						/ FMath::Max(Rest, SmallNumber));
				}
				if (MaximumStrain <= 0.0025) break;
				if ((Iteration & 1) == 0)
				{
					for (int32 Segment = 0; Segment + 1 < Particles.Num(); ++Segment) ProjectDistance(Segment, Input);
				}
				else
				{
					for (int32 Segment = Particles.Num() - 2; Segment >= 0; --Segment) ProjectDistance(Segment, Input);
				}
				for (FContactConstraint& Contact : Contacts) ProjectContact(Contact, Input);
			}
		}
	}

	void FSolver::ReconcileContacts(
		const FStepInput& Input,
		TArrayView<FContactConstraint> Contacts,
		const int32 Iterations)
	{
		if (!bStepActive) return;
		for (int32 Iteration = 0; Iteration < FMath::Clamp(Iterations, 0, 16); ++Iteration)
		{
			if ((Iteration & 1) == 0)
			{
				for (int32 Segment = 0; Segment + 1 < Particles.Num(); ++Segment) ProjectDistance(Segment, Input);
			}
			else
			{
				for (int32 Segment = Particles.Num() - 2; Segment >= 0; --Segment) ProjectDistance(Segment, Input);
			}
			for (FContactConstraint& Contact : Contacts) ProjectContact(Contact, Input);
		}
	}

	FStepResult FSolver::FinalizeStep(const FStepInput& Input, TArrayView<FContactConstraint> Contacts)
	{
		if (!bStepActive) return LastResult;
		bStepActive = false;
		TArray<FVector3d, TInlineAllocator<128>> PreSolveVelocities;
		PreSolveVelocities.Reserve(Particles.Num());
		for (const FParticle& Particle : Particles) PreSolveVelocities.Add(Particle.Velocity);
		for (FParticle& Particle : Particles)
		{
			Particle.Velocity = (Particle.Position - Particle.PreviousPosition) / Input.DeltaTime;
		}
		ApplyContactVelocityResponse(Contacts, Input, PreSolveVelocities);
		UpdateContactLoads(Contacts, Input);
		if (Input.StartEndpoint.State == EEndpointState::Fixed) Particles[0].Velocity = Input.StartEndpoint.TargetVelocity;
		if (Input.EndEndpoint.State == EEndpointState::Fixed) Particles.Last().Velocity = Input.EndEndpoint.TargetVelocity;

		if (!ValidateState())
		{
			RestoreState(LastValidState);
			LastResult.Status = ESimulationStatus::NumericalFailure;
			return LastResult;
		}
		++StepIndex;
		LastResult = FStepResult{};
		LastResult.Status = ESimulationStatus::Ready;
		if (Input.StartEndpoint.State == EEndpointState::Fixed && Input.EndEndpoint.State == EEndpointState::Fixed
			&& FVector3d::Distance(Input.StartEndpoint.TargetPosition, Input.EndEndpoint.TargetPosition) > GetActiveLength() + 0.1)
			LastResult.Status = ESimulationStatus::Overextended;
		LastResult.StepIndex = StepIndex;
		LastResult.ParticleCount = Particles.Num();
		LastResult.ContactCount = Contacts.Num();
		LastResult.MaximumSegmentError = CalculateMaximumSegmentError();
		LastResult.MaximumSegmentStrain = 0.0;
		for (int32 Segment = 0; Segment + 1 < Particles.Num(); ++Segment)
		{
			const double Rest = SegmentRestLength(Segment);
			LastResult.MaximumSegmentStrain = FMath::Max(
				LastResult.MaximumSegmentStrain,
				(FVector3d::Distance(Particles[Segment].Position, Particles[Segment + 1].Position) - Rest)
					/ FMath::Max(Rest, SmallNumber));
		}
		LastResult.ActiveLength = GetActiveLength();
		LastResult.MaximumPenetration = CalculateMaximumPenetration(Contacts);
		for (const FContactConstraint& Contact : Contacts)
			LastResult.MaximumContactCorrection = FMath::Max(LastResult.MaximumContactCorrection, Contact.LargestProjection);
		for (const FParticle& Particle : Particles)
		{
			LastResult.MaximumSpeed = FMath::Max(LastResult.MaximumSpeed, Particle.Velocity.Length());
			LastResult.MaximumEstimatedTension = FMath::Max(LastResult.MaximumEstimatedTension, Particle.EstimatedTension);
			LastResult.MaximumEstimatedNormalLoad = FMath::Max(LastResult.MaximumEstimatedNormalLoad, Particle.EstimatedNormalLoad);
			LastResult.MaximumParticleTravel = FMath::Max(LastResult.MaximumParticleTravel,
				FVector3d::Distance(Particle.Position, Particle.PreviousPosition));
		}
		FillEndpointResult(EEndpoint::Start, Input.StartEndpoint, LastResult.StartEndpoint);
		FillEndpointResult(EEndpoint::End, Input.EndEndpoint, LastResult.EndEndpoint);
		LastValidState = CaptureState();
		return LastResult;
	}

	FStepResult FSolver::AdvanceStep(const FStepInput& Input, TArrayView<FContactConstraint> Contacts)
	{
		if (!BeginStep(Input)) return LastResult;
		SolveBatch(Input, Contacts, Config.SolverIterations);
		return FinalizeStep(Input, Contacts);
	}

	int32 FSolver::FindSegmentAtMaterialCoordinate(const double Coordinate, double& OutAlpha) const
	{
		OutAlpha = 0.0;
		if (Particles.Num() < 2) return INDEX_NONE;
		const double Clamped = FMath::Clamp(Coordinate, 0.0, GetActiveLength());
		int32 Low = 0;
		int32 High = Particles.Num() - 1;
		while (Low + 1 < High)
		{
			const int32 Mid = (Low + High) / 2;
			if (Particles[Mid].MaterialCoordinate <= Clamped) Low = Mid;
			else High = Mid;
		}
		const int32 Segment = FMath::Min(Low, Particles.Num() - 2);
		const double Span = SegmentRestLength(Segment);
		OutAlpha = Span > SmallNumber
			? FMath::Clamp((Clamped - Particles[Segment].MaterialCoordinate) / Span, 0.0, 1.0)
			: 0.0;
		return Segment;
	}

	FVector3d FSolver::SamplePosition(const double Coordinate) const
	{
		double Alpha = 0.0;
		const int32 Segment = FindSegmentAtMaterialCoordinate(Coordinate, Alpha);
		return Particles.IsValidIndex(Segment + 1) ? FMath::Lerp(Particles[Segment].Position, Particles[Segment + 1].Position, Alpha) : FVector3d::ZeroVector;
	}

	FVector3d FSolver::SamplePreviousPosition(const double Coordinate) const
	{
		double Alpha = 0.0;
		const int32 Segment = FindSegmentAtMaterialCoordinate(Coordinate, Alpha);
		return Particles.IsValidIndex(Segment + 1) ? FMath::Lerp(Particles[Segment].PreviousPosition, Particles[Segment + 1].PreviousPosition, Alpha) : FVector3d::ZeroVector;
	}

	FStateSnapshot FSolver::CaptureState() const
	{
		return {Particles, Config, LastResult, StepIndex};
	}

	bool FSolver::RestoreState(const FStateSnapshot& Snapshot)
	{
		if (Snapshot.Particles.Num() < 2 || !IsValidConfig(Snapshot.Config)) return false;
		Particles = Snapshot.Particles;
		Config = Snapshot.Config;
		LastResult = Snapshot.LastResult;
		StepIndex = Snapshot.StepIndex;
		DistanceLambdas.Init(0.0, Particles.Num() - 1);
		ParticleFrictionCorrections.Init(0.0, Particles.Num());
		bStepActive = false;
		return ValidateState();
	}

	bool FSolver::IsFinite(const FVector3d& Value)
	{
		return FMath::IsFinite(Value.X) && FMath::IsFinite(Value.Y) && FMath::IsFinite(Value.Z);
	}

	bool FSolver::IsValidConfig(const FSimulationConfig& Value)
	{
		return FMath::IsFinite(Value.Length) && Value.Length > 0.0
			&& FMath::IsFinite(Value.SegmentLength) && Value.SegmentLength > 0.0
			&& Value.MaximumParticles >= 2 && Value.MaximumParticles <= 4096
			&& FMath::IsFinite(Value.LinearDensity) && Value.LinearDensity > 0.0
			&& IsFinite(Value.Gravity)
			&& FMath::IsFinite(Value.VelocityDamping) && Value.VelocityDamping >= 0.0 && Value.VelocityDamping <= 1.0
			&& FMath::IsFinite(Value.DistanceCompliance) && Value.DistanceCompliance >= 0.0
			&& FMath::IsFinite(Value.DistanceRelaxation) && Value.DistanceRelaxation >= 1.0 && Value.DistanceRelaxation <= 1.1
			&& FMath::IsFinite(Value.BendStrength) && Value.BendStrength >= 0.0 && Value.BendStrength <= 1.0
			&& FMath::IsFinite(Value.FreeBendDegreesPerMeter) && Value.FreeBendDegreesPerMeter >= 0.0
			&& FMath::IsFinite(Value.DrivenEndpointMaximumSpeed) && Value.DrivenEndpointMaximumSpeed > 0.0
			&& Value.SolverIterations >= 1 && Value.SolverIterations <= 256
			&& Value.MultigridIterations >= 0 && Value.MultigridIterations <= 8
			&& Value.MultigridMinimumParticles >= 8 && Value.MultigridMinimumParticles <= 4096
			&& FMath::IsFinite(Value.StaticFriction) && Value.StaticFriction >= 0.0
			&& FMath::IsFinite(Value.DynamicFriction) && Value.DynamicFriction >= 0.0
			&& FMath::IsFinite(Value.ConstantFrictionSpeedReduction) && Value.ConstantFrictionSpeedReduction >= 0.0
			&& FMath::IsFinite(Value.MaximumContactCorrection) && Value.MaximumContactCorrection > 0.0;
	}

	int32 FSolver::GetEndpointIndex(const EEndpoint Endpoint) const { return Endpoint == EEndpoint::Start ? 0 : Particles.Num() - 1; }

	double FSolver::EffectiveInverseMass(const int32 Index, const FStepInput& Input) const
	{
		if (!Particles.IsValidIndex(Index)) return 0.0;
		const FEndpointInput* Endpoint = Index == 0 ? &Input.StartEndpoint : (Index == Particles.Num() - 1 ? &Input.EndEndpoint : nullptr);
		if (!Endpoint || Endpoint->State != EEndpointState::Fixed) return Particles[Index].InverseMass;
		return 0.0;
	}

	double FSolver::SegmentRestLength(const int32 SegmentIndex) const
	{
		return Particles[SegmentIndex + 1].MaterialCoordinate - Particles[SegmentIndex].MaterialCoordinate;
	}

	void FSolver::ProjectEndpointDrive(
		const EEndpoint Endpoint,
		const FEndpointInput& EndpointInput,
		const double DeltaTime)
	{
		if (EndpointInput.State != EEndpointState::Driven)
		{
			return;
		}
		FParticle& Particle = Particles[GetEndpointIndex(Endpoint)];
		// The target is intentionally projected once, before structural and
		// collision constraints. A speed limit turns discontinuous input into the
		// stable "chasing" behaviour used by gameplay.
		const FVector3d ToTarget = EndpointInput.TargetPosition - Particle.Position;
		const double Distance = ToTarget.Length();
		const double MaximumDistance = Config.DrivenEndpointMaximumSpeed * DeltaTime;
		Particle.Position += Distance > MaximumDistance && Distance > SmallNumber
			? ToTarget * (MaximumDistance / Distance)
			: ToTarget;
	}

	void FSolver::ProjectDistance(const int32 Segment, const FStepInput& Input)
	{
		FParticle& A = Particles[Segment];
		FParticle& B = Particles[Segment + 1];
		const FVector3d Delta = B.Position - A.Position;
		const double Length = Delta.Length();
		if (Length <= SmallNumber) return;
		const double InvA = EffectiveInverseMass(Segment, Input);
		const double InvB = EffectiveInverseMass(Segment + 1, Input);
		const double Compliance = Config.DistanceCompliance / FMath::Square(Input.DeltaTime);
		if (InvA + InvB + Compliance <= SmallNumber) return;
		const double Constraint = Length - SegmentRestLength(Segment);
		double& Lambda = DistanceLambdas[Segment];
		const double DeltaLambda = Config.DistanceRelaxation
			* (-Constraint - Compliance * Lambda) / (InvA + InvB + Compliance);
		Lambda += DeltaLambda;
		const FVector3d Direction = Delta / Length;
		A.Position -= Direction * (InvA * DeltaLambda);
		B.Position += Direction * (InvB * DeltaLambda);
	}

	void FSolver::ProjectDistanceRange(
		const int32 FirstIndex,
		const int32 SecondIndex,
		const FStepInput& Input)
	{
		if (!Particles.IsValidIndex(FirstIndex) || !Particles.IsValidIndex(SecondIndex)
			|| FirstIndex >= SecondIndex)
		{
			return;
		}
		FParticle& A = Particles[FirstIndex];
		FParticle& B = Particles[SecondIndex];
		const FVector3d Delta = B.Position - A.Position;
		const double Length = Delta.Length();
		if (Length <= SmallNumber) return;
		const double InvA = EffectiveInverseMass(FirstIndex, Input);
		const double InvB = EffectiveInverseMass(SecondIndex, Input);
		if (InvA + InvB <= SmallNumber) return;
		const double RestLength = B.MaterialCoordinate - A.MaterialCoordinate;
		// A coarse chord may legitimately be shorter than the material arc when
		// the rope is slack. Multigrid exists to propagate extension; expanding
		// that chord would incorrectly straighten the rope and create local strain.
		if (Length <= RestLength) return;
		const double Correction = Config.DistanceRelaxation * (Length - RestLength) / (InvA + InvB);
		const FVector3d Direction = Delta / Length;
		A.Position += Direction * (InvA * Correction);
		B.Position -= Direction * (InvB * Correction);
	}

	void FSolver::SolveMultigrid(
		const FStepInput& Input,
		TArrayView<FContactConstraint> Contacts)
	{
		if (Config.MultigridIterations <= 0
			|| Particles.Num() < Config.MultigridMinimumParticles
			|| Config.DistanceCompliance > SmallNumber)
		{
			return;
		}
		TSet<int32> RequiredNodes;
		RequiredNodes.Add(0);
		RequiredNodes.Add(Particles.Num() - 1);
		for (const FContactConstraint& Contact : Contacts)
		{
			if (Particles.IsValidIndex(Contact.ParticleA)) RequiredNodes.Add(Contact.ParticleA);
			if (Particles.IsValidIndex(Contact.ParticleB)) RequiredNodes.Add(Contact.ParticleB);
		}
		int32 LargestStride = 1;
		while (LargestStride * 2 < Particles.Num() - 1) LargestStride *= 2;
		for (int32 Stride = LargestStride; Stride >= 2; Stride /= 2)
		{
			TArray<int32> Level;
			for (int32 Index = 0; Index < Particles.Num(); Index += Stride) Level.Add(Index);
			if (Level.IsEmpty() || Level.Last() != Particles.Num() - 1) Level.Add(Particles.Num() - 1);
			for (const int32 Required : RequiredNodes) Level.AddUnique(Required);
			Level.Sort();
			TArray<FVector3d> Before;
			Before.Reserve(Level.Num());
			for (const int32 Index : Level) Before.Add(Particles[Index].Position);
			for (int32 Iteration = 0; Iteration < Config.MultigridIterations; ++Iteration)
			{
				if ((Iteration & 1) == 0)
				{
					for (int32 Pair = 0; Pair + 1 < Level.Num(); ++Pair)
						ProjectDistanceRange(Level[Pair], Level[Pair + 1], Input);
				}
				else
				{
					for (int32 Pair = Level.Num() - 2; Pair >= 0; --Pair)
						ProjectDistanceRange(Level[Pair], Level[Pair + 1], Input);
				}
				for (FContactConstraint& Contact : Contacts) ProjectContact(Contact, Input);
			}
			for (int32 Pair = 0; Pair + 1 < Level.Num(); ++Pair)
			{
				const int32 First = Level[Pair];
				const int32 Last = Level[Pair + 1];
				const FVector3d FirstDelta = Particles[First].Position - Before[Pair];
				const FVector3d LastDelta = Particles[Last].Position - Before[Pair + 1];
				for (int32 Index = First + 1; Index < Last; ++Index)
				{
					if (RequiredNodes.Contains(Index) || EffectiveInverseMass(Index, Input) <= SmallNumber) continue;
					const double Alpha = static_cast<double>(Index - First) / static_cast<double>(Last - First);
					Particles[Index].Position += FMath::Lerp(FirstDelta, LastDelta, Alpha);
				}
			}
			for (FContactConstraint& Contact : Contacts) ProjectContact(Contact, Input);
		}
	}

	void FSolver::ProjectBend(const int32 First, const FStepInput& Input, const double Strength)
	{
		if (Strength <= 0.0) return;
		FParticle& P1 = Particles[First];
		FParticle& P2 = Particles[First + 1];
		FParticle& P3 = Particles[First + 2];
		const FVector3d Chord = P3.Position - P1.Position;
		const double S = Chord.Length();
		if (S <= SmallNumber) return;
		const FVector3d Direction = Chord / S;
		const double S1 = FMath::Clamp(FVector3d::DotProduct(P2.Position - P1.Position, Direction), 0.0, S);
		const double S2 = S - S1;
		const FVector3d Offset = P2.Position - (P1.Position + Direction * S1);
		const double E = Offset.Length();
		if (E <= SmallNumber) return;
		const double L0 = 0.5 * (SegmentRestLength(First) + SegmentRestLength(First + 1));
		const double FreeRadiansPerCm = FMath::DegreesToRadians(Config.FreeBendDegreesPerMeter) / 100.0;
		const double FreeHeight = L0 * FMath::Cos((UE_DOUBLE_PI - FreeRadiansPerCm * L0) * 0.5);
		const double Excess = E - FMath::Max(FreeHeight, 0.0);
		if (Excess <= 0.0) return;
		const double W1 = EffectiveInverseMass(First, Input);
		const double W2 = EffectiveInverseMass(First + 1, Input);
		const double W3 = EffectiveInverseMass(First + 2, Input);
		const double Denominator = S2 * S2 * W1 + S * S * W2 + S1 * S1 * W3;
		if (Denominator <= SmallNumber) return;
		const double H = Strength * Excess * S / Denominator;
		const FVector3d Normal = Offset / E;
		P1.Position += Normal * (S2 * W1 * H);
		P2.Position -= Normal * (S * W2 * H);
		P3.Position += Normal * (S1 * W3 * H);
	}

	double FSolver::ProjectContact(FContactConstraint& Contact, const FStepInput& Input)
	{
		if (!Particles.IsValidIndex(Contact.ParticleA) || !IsFinite(Contact.Normal)) return 0.0;
		const FVector3d Normal = Contact.Normal.GetSafeNormal();
		if (Normal.IsNearlyZero()) return 0.0;
		const double Alpha = Particles.IsValidIndex(Contact.ParticleB) ? FMath::Clamp(Contact.SegmentAlpha, 0.0, 1.0) : 0.0;
		const double ShapeA = 1.0 - Alpha;
		const double ShapeB = Alpha;
		const FVector3d Point = SamplePoint(Particles, Contact, false);
		const double Constraint = FVector3d::DotProduct(Point, Normal) - Contact.MinimumNormalCoordinate;
		if (Constraint >= 0.0) return 0.0;
		const double InvA = EffectiveInverseMass(Contact.ParticleA, Input);
		const double InvB = Particles.IsValidIndex(Contact.ParticleB) ? EffectiveInverseMass(Contact.ParticleB, Input) : 0.0;
		const double Denominator = InvA * ShapeA * ShapeA + InvB * ShapeB * ShapeB;
		if (Denominator <= SmallNumber) return 0.0;
		const double PointCorrection = FMath::Min(-Constraint, Config.MaximumContactCorrection);
		const double Lambda = PointCorrection / Denominator;
		const double CorrectionA = InvA * ShapeA * Lambda;
		const double CorrectionB = InvB * ShapeB * Lambda;
		Particles[Contact.ParticleA].Position += Normal * CorrectionA;
		if (Particles.IsValidIndex(Contact.ParticleB)) Particles[Contact.ParticleB].Position += Normal * CorrectionB;
		Contact.AccumulatedNormalCorrection += PointCorrection;
		Contact.AccumulatedParticleCorrectionA += CorrectionA;
		Contact.AccumulatedParticleCorrectionB += CorrectionB;
		Contact.LargestProjection = FMath::Max(Contact.LargestProjection, -Constraint);
		return PointCorrection;
	}

	void FSolver::ProjectParticleFriction(
		const int32 ParticleIndex,
		const TConstArrayView<FContactConstraint> Contacts,
		const FStepInput& Input)
	{
		if (!Particles.IsValidIndex(ParticleIndex)
			|| EffectiveInverseMass(ParticleIndex, Input) <= SmallNumber)
		{
			return;
		}
		TArray<FVector3d, TInlineAllocator<3>> Normals;
		FVector3d AverageAnchor = FVector3d::ZeroVector;
		int32 ActiveCount = 0;
		for (const FContactConstraint& Contact : Contacts)
		{
			if (Contact.ParticleA != ParticleIndex || !Contact.bEnableFriction
				|| Contact.AccumulatedNormalCorrection <= 0.0)
			{
				continue;
			}
			AddOrthonormalNormal(Contact.Normal, Normals);
			AverageAnchor += Contact.FrictionAnchor;
			++ActiveCount;
		}
		if (ActiveCount == 0 || Normals.IsEmpty()) return;
		AverageAnchor /= static_cast<double>(ActiveCount);
		FParticle& Particle = Particles[ParticleIndex];
		const FVector3d Tangent = RemoveNormalComponents(Particle.Position - AverageAnchor, Normals);
		const double TangentLength = Tangent.Length();
		if (TangentLength <= SmallNumber) return;
		const double PointSpeed = RemoveNormalComponents(Particle.Velocity, Normals).Length();
		const double Fade = Config.FrictionFadeEndSpeed > Config.FrictionFadeStartSpeed
			? 1.0 - FMath::Clamp((PointSpeed - Config.FrictionFadeStartSpeed)
				/ (Config.FrictionFadeEndSpeed - Config.FrictionFadeStartSpeed), 0.0, 1.0)
			: 1.0;
		const double Mass = Particle.Mass;
		if (Mass <= SmallNumber) return;
		const double LoadDisplacement = Particle.EstimatedNormalLoad / Mass * 100.0
			* Input.DeltaTime * Input.DeltaTime;
		const double TotalAllowance = Fade
			* (Config.StaticFrictionDeadZone + Config.StaticFriction * LoadDisplacement);
		const double Remaining = FMath::Max(TotalAllowance - ParticleFrictionCorrections[ParticleIndex], 0.0);
		const double CorrectionLength = FMath::Min(TangentLength, Remaining);
		if (CorrectionLength <= 0.0) return;
		Particle.Position -= Tangent * (CorrectionLength / TangentLength);
		ParticleFrictionCorrections[ParticleIndex] += CorrectionLength;
	}

	void FSolver::ApplyContactVelocityResponse(
		const TConstArrayView<FContactConstraint> Contacts,
		const FStepInput& Input,
		const TConstArrayView<FVector3d> PreSolveVelocities)
	{
		for (int32 ParticleIndex = 0; ParticleIndex < Particles.Num(); ++ParticleIndex)
		{
			if (EffectiveInverseMass(ParticleIndex, Input) <= SmallNumber) continue;
			TArray<FVector3d, TInlineAllocator<3>> Normals;
			FVector3d SurfaceVelocity = FVector3d::ZeroVector;
			int32 ActiveCount = 0;
			for (const FContactConstraint& Contact : Contacts)
			{
				const double ParticleCorrection = Contact.ParticleA == ParticleIndex
					? Contact.AccumulatedParticleCorrectionA
					: (Contact.ParticleB == ParticleIndex ? Contact.AccumulatedParticleCorrectionB : 0.0);
				if (ParticleCorrection <= 0.0) continue;
				AddOrthonormalNormal(Contact.Normal, Normals);
				SurfaceVelocity += Contact.SurfaceVelocity;
				++ActiveCount;
			}
			if (ActiveCount == 0 || Normals.IsEmpty()) continue;
			SurfaceVelocity /= static_cast<double>(ActiveCount);
			FParticle& Particle = Particles[ParticleIndex];
			FVector3d Relative = Particle.Velocity - SurfaceVelocity;
			const FVector3d PreSolveRelative = PreSolveVelocities.IsValidIndex(ParticleIndex)
				? PreSolveVelocities[ParticleIndex] - SurfaceVelocity
				: FVector3d::ZeroVector;
			for (const FVector3d& Normal : Normals)
			{
				const double CurrentNormalSpeed = FVector3d::DotProduct(Relative, Normal);
				const double GenuineSeparationSpeed = FMath::Max(
					FVector3d::DotProduct(PreSolveRelative, Normal),
					0.0);
				// Projection is positional repair, not physical restitution. Replace
				// its apparent speed with only the separation already present before
				// constraint solving; closing speed becomes zero.
				Relative += Normal * (GenuineSeparationSpeed - CurrentNormalSpeed);
			}
			const FVector3d Tangent = RemoveNormalComponents(Relative, Normals);
			const double TangentSpeed = Tangent.Length();
			if (TangentSpeed > SmallNumber)
			{
				const double Fade = Config.FrictionFadeEndSpeed > Config.FrictionFadeStartSpeed
					? 1.0 - FMath::Clamp((TangentSpeed - Config.FrictionFadeStartSpeed)
						/ (Config.FrictionFadeEndSpeed - Config.FrictionFadeStartSpeed), 0.0, 1.0)
					: 1.0;
				const double ProportionalSpeed = TangentSpeed
					* (1.0 - FMath::Clamp(Config.DynamicFriction * Fade, 0.0, 1.0));
				const double ReducedSpeed = FMath::Max(
					ProportionalSpeed - Config.ConstantFrictionSpeedReduction * Fade,
					0.0);
				Relative += Tangent * (ReducedSpeed / TangentSpeed - 1.0);
			}
			Particle.Velocity = SurfaceVelocity + Relative;
		}
	}

	void FSolver::UpdateContactLoads(
		const TConstArrayView<FContactConstraint> Contacts,
		const FStepInput& Input)
	{
		TArray<bool> HasContact;
		TArray<bool> HasEdgeContact;
		TArray<FVector3d> AverageNormals;
		HasContact.Init(false, Particles.Num());
		HasEdgeContact.Init(false, Particles.Num());
		AverageNormals.Init(FVector3d::ZeroVector, Particles.Num());
		for (const FContactConstraint& Contact : Contacts)
		{
			if (!Particles.IsValidIndex(Contact.ParticleA) || Contact.AccumulatedNormalCorrection <= 0.0) continue;
			HasContact[Contact.ParticleA] = true;
			HasEdgeContact[Contact.ParticleA] |= Contact.FeatureId.Type == ECollisionFeatureType::Edge;
			AverageNormals[Contact.ParticleA] += Contact.Normal;
			if (Particles.IsValidIndex(Contact.ParticleB))
			{
				HasContact[Contact.ParticleB] = true;
				HasEdgeContact[Contact.ParticleB] |= Contact.FeatureId.Type == ECollisionFeatureType::Edge;
				AverageNormals[Contact.ParticleB] += Contact.Normal;
			}
		}
		const double UnsupportedBendThreshold = FMath::DegreesToRadians(30.0);
		TArray<double> FromStart;
		TArray<double> FromEnd;
		FromStart.Init(0.0, Particles.Num());
		FromEnd.Init(0.0, Particles.Num());
		for (int32 Index = 1; Index < Particles.Num(); ++Index)
		{
			FromStart[Index] = FromStart[Index - 1] + Particles[Index].Mass * Config.Gravity.Length() / 100.0;
			if (Index + 1 < Particles.Num() && !HasEdgeContact[Index])
			{
				const FVector3d A = (Particles[Index - 1].Position - Particles[Index].Position).GetSafeNormal();
				const FVector3d B = (Particles[Index + 1].Position - Particles[Index].Position).GetSafeNormal();
				const double Bend = FMath::Acos(FMath::Clamp(FVector3d::DotProduct(A, B), -1.0, 1.0));
				if (UE_DOUBLE_PI - Bend > UnsupportedBendThreshold) FromStart[Index] = 0.0;
			}
		}
		for (int32 Index = Particles.Num() - 2; Index >= 0; --Index)
		{
			FromEnd[Index] = FromEnd[Index + 1] + Particles[Index].Mass * Config.Gravity.Length() / 100.0;
			if (Index > 0 && !HasEdgeContact[Index])
			{
				const FVector3d A = (Particles[Index - 1].Position - Particles[Index].Position).GetSafeNormal();
				const FVector3d B = (Particles[Index + 1].Position - Particles[Index].Position).GetSafeNormal();
				const double Bend = FMath::Acos(FMath::Clamp(FVector3d::DotProduct(A, B), -1.0, 1.0));
				if (UE_DOUBLE_PI - Bend > UnsupportedBendThreshold) FromEnd[Index] = 0.0;
			}
		}
		const bool bStartSupported = Input.StartEndpoint.State != EEndpointState::Free;
		const bool bEndSupported = Input.EndEndpoint.State != EEndpointState::Free;
		for (int32 Index = 0; Index < Particles.Num(); ++Index)
		{
			Particles[Index].EstimatedTension = bStartSupported && !bEndSupported
				? FromEnd[Index]
				: (!bStartSupported && bEndSupported
					? FromStart[Index]
					: FMath::Min(FromStart[Index], FromEnd[Index]));
		}
		for (int32 Index = 0; Index < Particles.Num(); ++Index)
		{
			Particles[Index].EstimatedNormalLoad = 0.0;
			if (!HasContact[Index]) continue;
			const FVector3d Normal = AverageNormals[Index].GetSafeNormal();
			const double GravityLoad = Particles[Index].Mass
				* FMath::Max(-FVector3d::DotProduct(Config.Gravity / 100.0, Normal), 0.0);
			FVector3d BendForce = FVector3d::ZeroVector;
			if (Index > 0)
				BendForce += (Particles[Index - 1].Position - Particles[Index].Position).GetSafeNormal()
					* 0.5 * (Particles[Index].EstimatedTension + Particles[Index - 1].EstimatedTension);
			if (Index + 1 < Particles.Num())
				BendForce += (Particles[Index + 1].Position - Particles[Index].Position).GetSafeNormal()
					* 0.5 * (Particles[Index].EstimatedTension + Particles[Index + 1].EstimatedTension);
			Particles[Index].EstimatedNormalLoad = GravityLoad
				+ FMath::Max(-FVector3d::DotProduct(BendForce, Normal), 0.0);
		}
	}

	bool FSolver::ValidateState() const
	{
		if (Particles.Num() < 2 || Particles.Num() > Config.MaximumParticles) return false;
		double Previous = -1.0;
		for (const FParticle& Particle : Particles)
		{
			if (!IsFinite(Particle.Position) || !IsFinite(Particle.PreviousPosition) || !IsFinite(Particle.Velocity)
				|| !FMath::IsFinite(Particle.Mass) || Particle.Mass <= 0.0
				|| !FMath::IsFinite(Particle.InverseMass) || Particle.InverseMass <= 0.0
				|| !FMath::IsFinite(Particle.EstimatedTension) || !FMath::IsFinite(Particle.EstimatedNormalLoad)
				|| !FMath::IsFinite(Particle.MaterialCoordinate) || Particle.MaterialCoordinate <= Previous) return false;
			Previous = Particle.MaterialCoordinate;
		}
		return NearlyEqual(Particles[0].MaterialCoordinate, 0.0, 1.e-9)
			&& NearlyEqual(Particles.Last().MaterialCoordinate, Config.Length, 1.e-6);
	}

	double FSolver::CalculateMaximumSegmentError() const
	{
		double Maximum = 0.0;
		for (int32 Segment = 0; Segment + 1 < Particles.Num(); ++Segment)
			Maximum = FMath::Max(Maximum, FMath::Abs(FVector3d::Distance(Particles[Segment].Position, Particles[Segment + 1].Position) - SegmentRestLength(Segment)));
		return Maximum;
	}

	double FSolver::CalculateMaximumPenetration(const TConstArrayView<FContactConstraint> Contacts) const
	{
		double Maximum = 0.0;
		for (const FContactConstraint& Contact : Contacts)
		{
			if (!Particles.IsValidIndex(Contact.ParticleA)) continue;
			Maximum = FMath::Max(Maximum, Contact.MinimumNormalCoordinate
				- FVector3d::DotProduct(SamplePoint(Particles, Contact, false), Contact.Normal));
		}
		return FMath::Max(Maximum, 0.0);
	}

	void FSolver::FillEndpointResult(const EEndpoint Endpoint, const FEndpointInput& Input, FEndpointResult& Result) const
	{
		const FParticle& Particle = Particles[GetEndpointIndex(Endpoint)];
		Result.State = Input.State;
		Result.RequestedPosition = Input.State == EEndpointState::Free ? Particle.Position : Input.TargetPosition;
		Result.AcceptedPosition = Particle.Position;
		Result.Velocity = Particle.Velocity;
		Result.Correction = Particle.Position - Result.RequestedPosition;
		Result.LimitError = Result.Correction.Length();
		Result.bLimited = Input.State == EEndpointState::Driven && Result.LimitError > 0.1;
	}
}
