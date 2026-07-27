#if WITH_DEV_AUTOMATION_TESTS

#include "CableSimSolver.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"

#include <limits>

namespace CableSimTests
{
	CableSim::FSimulationConfig MakeConfig(
		const double RestLength = 100.0,
		const double NodeSpacing = 25.0)
	{
		CableSim::FSimulationConfig Config;
		Config.RestLength = RestLength;
		Config.NodeSpacing = NodeSpacing;
		Config.ParticleMass = 1.0;
		Config.Gravity = FVector3d::ZeroVector;
		Config.VelocityDamping = 0.0;
		Config.ConstraintIterations = 16;
		return Config;
	}

	CableSim::FStepInput MakeFreeInput(const double DeltaTime = 1.0 / 60.0)
	{
		CableSim::FStepInput Input;
		Input.DeltaTime = DeltaTime;
		return Input;
	}

	CableSim::FStepInput MakeFixedInput(
		const FVector3d& Start,
		const FVector3d& End,
		const double DeltaTime = 1.0 / 60.0)
	{
		CableSim::FStepInput Input;
		Input.DeltaTime = DeltaTime;
		Input.StartEndpoint.Mode = CableSim::EParticleMode::Kinematic;
		Input.StartEndpoint.TargetPosition = Start;
		Input.EndEndpoint.Mode = CableSim::EParticleMode::Kinematic;
		Input.EndEndpoint.TargetPosition = End;
		return Input;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimInitializationTest,
	"CableSim.Core.Initialization",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimInitializationTest::RunTest(const FString& Parameters)
{
	CableSim::FSolver Solver;
	TestTrue(TEXT("Initialization succeeds"), Solver.Initialize(
		FVector3d::ZeroVector,
		FVector3d(100.0, 0.0, 0.0),
		CableSimTests::MakeConfig()));
	TestEqual(TEXT("Particle count"), Solver.GetParticles().Num(), 5);
	TestEqual(TEXT("Rest length"), Solver.GetRestLength(), 100.0);
	TestEqual(TEXT("Rest segment length"), Solver.GetRestSegmentLength(), 25.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimFixedEndpointsTest,
	"CableSim.Core.FixedEndpoints",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimFixedEndpointsTest::RunTest(const FString& Parameters)
{
	const FVector3d Start(0.0, 0.0, 100.0);
	const FVector3d End(100.0, 0.0, 100.0);
	CableSim::FSolver Solver;
	Solver.Initialize(Start, End, CableSimTests::MakeConfig(150.0, 10.0));
	const CableSim::FStepInput Input = CableSimTests::MakeFixedInput(Start, End);
	for (int32 Step = 0; Step < 30; ++Step)
	{
		Solver.AdvanceStep(Input);
	}
	TestTrue(TEXT("Start endpoint remains fixed"), Solver.GetParticles()[0].Position.Equals(Start, 1.e-9));
	TestTrue(TEXT("End endpoint remains fixed"), Solver.GetParticles().Last().Position.Equals(End, 1.e-9));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimFreeFallTest,
	"CableSim.Core.FreeFall",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimFreeFallTest::RunTest(const FString& Parameters)
{
	CableSim::FSimulationConfig Config = CableSimTests::MakeConfig();
	Config.Gravity = FVector3d(0.0, 0.0, -100.0);
	Config.ConstraintIterations = 4;
	CableSim::FSolver Solver;
	Solver.Initialize(FVector3d::ZeroVector, FVector3d(100.0, 0.0, 0.0), Config);
	Solver.AdvanceStep(CableSimTests::MakeFreeInput(0.1));
	for (const CableSim::FParticle& Particle : Solver.GetParticles())
	{
		TestTrue(TEXT("Uniform free-fall displacement"), FMath::IsNearlyEqual(Particle.Position.Z, -1.0, 1.e-9));
		TestTrue(TEXT("Uniform free-fall velocity"), FMath::IsNearlyEqual(Particle.Velocity.Z, -10.0, 1.e-9));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimReplayTest,
	"CableSim.Core.StateReplay",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimReplayTest::RunTest(const FString& Parameters)
{
	const FVector3d Start(0.0, 0.0, 100.0);
	const FVector3d End(100.0, 0.0, 100.0);
	CableSim::FSolver Solver;
	Solver.Initialize(Start, End, CableSimTests::MakeConfig(150.0, 10.0));
	CableSim::FStepInput Input = CableSimTests::MakeFixedInput(Start, End);
	Input.EndEndpoint.Mode = CableSim::EParticleMode::Dynamic;
	for (int32 Step = 0; Step < 10; ++Step)
	{
		Solver.AdvanceStep(Input);
	}
	const CableSim::FStateSnapshot Snapshot = Solver.CaptureState();
	for (int32 Step = 0; Step < 10; ++Step)
	{
		Solver.AdvanceStep(Input);
	}
	const CableSim::FStateSnapshot FirstResult = Solver.CaptureState();
	TestTrue(TEXT("Snapshot restores"), Solver.RestoreState(Snapshot));
	for (int32 Step = 0; Step < 10; ++Step)
	{
		Solver.AdvanceStep(Input);
	}
	const CableSim::FStateSnapshot SecondResult = Solver.CaptureState();
	TestEqual(TEXT("Replay step index"), SecondResult.StepIndex, FirstResult.StepIndex);
	TestEqual(TEXT("Replay particle count"), SecondResult.Particles.Num(), FirstResult.Particles.Num());
	for (int32 Index = 0; Index < FirstResult.Particles.Num(); ++Index)
	{
		TestTrue(
			FString::Printf(TEXT("Replay position %d"), Index),
			SecondResult.Particles[Index].Position.Equals(FirstResult.Particles[Index].Position, 1.e-12));
		TestTrue(
			FString::Printf(TEXT("Replay velocity %d"), Index),
			SecondResult.Particles[Index].Velocity.Equals(FirstResult.Particles[Index].Velocity, 1.e-12));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimBendingMomentumTest,
	"CableSim.Core.Bending.MomentumConservation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimBendingMomentumTest::RunTest(const FString& Parameters)
{
	constexpr double HalfSpan = 100.0;
	constexpr double Height = 100.0;
	const double SegmentLength = FMath::Sqrt(HalfSpan * HalfSpan + Height * Height);
	CableSim::FSimulationConfig Config = CableSimTests::MakeConfig(SegmentLength * 2.0, SegmentLength + 1.0);
	Config.BendingStepStrength = 1.0;
	Config.ConstraintIterations = 1;
	Config.DistanceOverRelaxation = 1.0;
	CableSim::FSolver Solver;
	Solver.Initialize(FVector3d(-HalfSpan, 0.0, 0.0), FVector3d(HalfSpan, 0.0, 0.0), Config);
	CableSim::FStateSnapshot InitialState = Solver.CaptureState();
	InitialState.Particles[0].Position = FVector3d(-HalfSpan, 0.0, 0.0);
	InitialState.Particles[1].Position = FVector3d(0.0, 0.0, Height);
	InitialState.Particles[2].Position = FVector3d(HalfSpan, 0.0, 0.0);
	for (CableSim::FParticle& Particle : InitialState.Particles)
	{
		Particle.PreviousPosition = Particle.Position;
	}
	InitialState.Particles[0].InverseMass = 1.0;
	InitialState.Particles[1].InverseMass = 0.5;
	InitialState.Particles[2].InverseMass = 1.0 / 3.0;
	Solver.RestoreState(InitialState);
	Solver.AdvanceStep(CableSimTests::MakeFreeInput(1.0));

	const TArray<CableSim::FParticle>& Result = Solver.GetParticles();
	FVector3d LinearImpulse = FVector3d::ZeroVector;
	FVector3d AngularImpulse = FVector3d::ZeroVector;
	for (int32 Index = 0; Index < Result.Num(); ++Index)
	{
		const double Mass = 1.0 / InitialState.Particles[Index].InverseMass;
		const FVector3d Displacement = Result[Index].Position - InitialState.Particles[Index].Position;
		const FVector3d MomentumChange = Displacement * Mass;
		LinearImpulse += MomentumChange;
		AngularImpulse += FVector3d::CrossProduct(InitialState.Particles[Index].Position, MomentumChange);
	}
	TestTrue(TEXT("Bend correction changes middle node"), Result[1].Position.Z < Height);
	TestTrue(TEXT("Bend correction conserves linear momentum"), LinearImpulse.IsNearlyZero(1.e-8));
	TestTrue(TEXT("Bend correction conserves angular momentum"), AngularImpulse.IsNearlyZero(1.e-8));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimFreeBendThresholdTest,
	"CableSim.Core.Bending.FreeAngleThreshold",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimFreeBendThresholdTest::RunTest(const FString& Parameters)
{
	constexpr double HalfSpan = 100.0;
	constexpr double Height = 5.0;
	const double SegmentLength = FMath::Sqrt(HalfSpan * HalfSpan + Height * Height);
	CableSim::FSimulationConfig Config = CableSimTests::MakeConfig(SegmentLength * 2.0, SegmentLength + 1.0);
	Config.BendingStepStrength = 1.0;
	Config.FreeBendAngleRadiansPerMeter = FMath::DegreesToRadians(30.0);
	Config.ConstraintIterations = 1;
	Config.DistanceOverRelaxation = 1.0;
	CableSim::FSolver Solver;
	Solver.Initialize(FVector3d(-HalfSpan, 0.0, 0.0), FVector3d(HalfSpan, 0.0, 0.0), Config);
	CableSim::FStateSnapshot InitialState = Solver.CaptureState();
	InitialState.Particles[1].Position = FVector3d(0.0, 0.0, Height);
	InitialState.Particles[1].PreviousPosition = InitialState.Particles[1].Position;
	Solver.RestoreState(InitialState);
	Solver.AdvanceStep(CableSimTests::MakeFreeInput(1.0));
	TestTrue(
		TEXT("Bend inside free-angle threshold is unchanged"),
		Solver.GetParticles()[1].Position.Equals(InitialState.Particles[1].Position, 1.e-9));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimRemeshTest,
	"CableSim.Core.Configuration.StatePreservingRemesh",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimRemeshTest::RunTest(const FString& Parameters)
{
	CableSim::FSolver Solver;
	CableSim::FSimulationConfig Config = CableSimTests::MakeConfig(100.0, 25.0);
	Solver.Initialize(FVector3d::ZeroVector, FVector3d(100.0, 0.0, 0.0), Config);
	CableSim::FStateSnapshot State = Solver.CaptureState();
	for (int32 Index = 0; Index < State.Particles.Num(); ++Index)
	{
		State.Particles[Index].Position.Z = Index * Index;
		State.Particles[Index].PreviousPosition = State.Particles[Index].Position - FVector3d(0.0, 0.0, 1.0);
		State.Particles[Index].Velocity = FVector3d(0.0, 0.0, Index);
	}
	Solver.RestoreState(State);
	const uint64 StepIndex = Solver.GetStepIndex();
	Config.RestLength = 200.0;
	TestTrue(TEXT("Remesh succeeds"), Solver.ApplyConfig(Config));
	TestEqual(TEXT("Particle count follows new rest length"), Solver.GetParticles().Num(), 9);
	TestEqual(TEXT("Remesh does not reset step index"), Solver.GetStepIndex(), StepIndex);
	TestTrue(TEXT("Quarter position preserved"), Solver.GetParticles()[2].Position.Equals(State.Particles[1].Position, 1.e-12));
	TestTrue(TEXT("Quarter velocity preserved"), Solver.GetParticles()[2].Velocity.Equals(State.Particles[1].Velocity, 1.e-12));
	TestEqual(TEXT("New material coordinate"), Solver.GetParticles()[2].MaterialCoordinate, 50.0);

	const TArray<CableSim::FParticle> BeforeMassChange = Solver.GetParticles();
	Config.ParticleMass = 2.0;
	TestTrue(TEXT("Mass update succeeds"), Solver.ApplyConfig(Config));
	TestEqual(TEXT("Mass update preserves particle count"), Solver.GetParticles().Num(), BeforeMassChange.Num());
	TestTrue(TEXT("Mass update preserves position"), Solver.GetParticles()[2].Position.Equals(BeforeMassChange[2].Position, 1.e-12));
	TestEqual(TEXT("Mass update changes inverse mass"), Solver.GetParticles()[2].InverseMass, 0.5);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimOverextensionTest,
	"CableSim.Core.Endpoints.ControlledOverextension",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimOverextensionTest::RunTest(const FString& Parameters)
{
	const FVector3d Start = FVector3d::ZeroVector;
	const FVector3d FarEnd(1000.0, 0.0, 0.0);
	CableSim::FSolver Solver;
	CableSim::FSimulationConfig Config = CableSimTests::MakeConfig(100.0, 10.0);
	Config.ConstraintIterations = 64;
	Solver.Initialize(Start, FVector3d(100.0, 0.0, 0.0), Config);
	const CableSim::FStepInput FarInput = CableSimTests::MakeFixedInput(Start, FarEnd);
	CableSim::FStepResult Result;
	for (int32 Step = 0; Step < 120; ++Step)
	{
		Result = Solver.AdvanceStep(FarInput);
	}
	TestEqual(TEXT("Overextension is reported"), Result.Status, CableSim::ESimulationStatus::Overextended);
	TestTrue(TEXT("Start stays attached"), Solver.GetParticles()[0].Position.Equals(Start, 0.1));
	TestTrue(TEXT("End stays attached"), Solver.GetParticles().Last().Position.Equals(FarEnd, 0.1));
	TestEqual(TEXT("Physical rest length is unchanged"), Result.RestLength, 100.0);
	TestEqual(TEXT("Effective length follows separation"), Result.EffectiveSolveLength, 1000.0);
	TestTrue(TEXT("Strain is reported"), FMath::IsNearlyEqual(Result.StrainRatio, 9.0, 1.e-9));
	for (const CableSim::FParticle& Particle : Solver.GetParticles())
	{
		TestFalse(TEXT("Position remains finite"), Particle.Position.ContainsNaN());
	}

	Result = Solver.AdvanceStep(CableSimTests::MakeFixedInput(Start, FVector3d(100.0, 0.0, 0.0)));
	TestEqual(TEXT("Feasible state recovers"), Result.Status, CableSim::ESimulationStatus::Ready);
	TestEqual(TEXT("Effective length returns to rest length"), Result.EffectiveSolveLength, 100.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimContactPlanesTest,
	"CableSim.Core.Collision.MultipleContactPlanes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimContactPlanesTest::RunTest(const FString& Parameters)
{
	CableSim::FSolver Solver;
	CableSim::FSimulationConfig Config = CableSimTests::MakeConfig(20.0, 10.0);
	Config.ConstraintIterations = 4;
	Solver.Initialize(FVector3d(-10.0), FVector3d(10.0), Config);
	CableSim::FStateSnapshot State = Solver.CaptureState();
	State.Particles[1].Position = FVector3d(-5.0, -6.0, -7.0);
	State.Particles[1].PreviousPosition = State.Particles[1].Position;
	Solver.RestoreState(State);
	const CableSim::FStepResult Result = Solver.AdvanceStep(
		CableSimTests::MakeFreeInput(),
		[](const TConstArrayView<CableSim::FParticle>, TArray<CableSim::FContactConstraint>& Contacts)
		{
			for (const FVector3d Normal : {FVector3d::UnitX(), FVector3d::UnitY(), FVector3d::UnitZ()})
			{
				CableSim::FContactConstraint& Contact = Contacts.AddDefaulted_GetRef();
				Contact.ParticleIndex = 1;
				Contact.Normal = Normal;
				Contact.MinimumNormalCoordinate = 0.0;
			}
		});
	TestEqual(TEXT("All contact planes retained"), Result.ContactCount, 3);
	TestTrue(TEXT("X plane projected"), Solver.GetParticles()[1].Position.X >= -1.e-9);
	TestTrue(TEXT("Y plane projected"), Solver.GetParticles()[1].Position.Y >= -1.e-9);
	TestTrue(TEXT("Z plane projected"), Solver.GetParticles()[1].Position.Z >= -1.e-9);
	TestTrue(TEXT("Penetration residual is small"), Result.MaximumPenetration <= 1.e-9);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimRecordedContactReplayTest,
	"CableSim.Core.Replay.RecordedContacts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimRecordedContactReplayTest::RunTest(const FString& Parameters)
{
	CableSim::FSolver Solver;
	Solver.Initialize(FVector3d::ZeroVector, FVector3d(100.0, 0.0, 0.0), CableSimTests::MakeConfig());
	const CableSim::FStateSnapshot InitialState = Solver.CaptureState();
	const CableSim::FStepInput Input = CableSimTests::MakeFreeInput();
	Solver.AdvanceStep(Input, [](const TConstArrayView<CableSim::FParticle>, TArray<CableSim::FContactConstraint>& Contacts)
	{
		CableSim::FContactConstraint& Contact = Contacts.AddDefaulted_GetRef();
		Contact.ParticleIndex = 2;
		Contact.Normal = FVector3d::UnitZ();
		Contact.MinimumNormalCoordinate = 5.0;
	});
	const CableSim::FStateSnapshot FirstResult = Solver.CaptureState();
	const CableSim::FReplayFrame Frame = Solver.GetLastReplayFrame();
	Solver.RestoreState(InitialState);
	Solver.AdvanceStep(Frame.Input, [&Frame](
		const TConstArrayView<CableSim::FParticle>,
		TArray<CableSim::FContactConstraint>& Contacts)
	{
		Contacts = Frame.Contacts;
	});
	for (int32 Index = 0; Index < FirstResult.Particles.Num(); ++Index)
	{
		TestTrue(
			FString::Printf(TEXT("Recorded contact replay position %d"), Index),
			Solver.GetParticles()[Index].Position.Equals(FirstResult.Particles[Index].Position, 1.e-12));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimValidationTest,
	"CableSim.Core.Validation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimValidationTest::RunTest(const FString& Parameters)
{
	CableSim::FSolver Solver;
	CableSim::FSimulationConfig InvalidConfig = CableSimTests::MakeConfig();
	InvalidConfig.RestLength = -1.0;
	TestFalse(TEXT("Invalid initialization is rejected"), Solver.Initialize(
		FVector3d::ZeroVector,
		FVector3d(100.0, 0.0, 0.0),
		InvalidConfig));

	Solver.Initialize(FVector3d::ZeroVector, FVector3d(100.0, 0.0, 0.0), CableSimTests::MakeConfig());
	const uint64 InitialStep = Solver.GetStepIndex();
	CableSim::FStepInput InvalidInput = CableSimTests::MakeFreeInput();
	InvalidInput.DeltaTime = 0.0;
	const CableSim::FStepResult Result = Solver.AdvanceStep(InvalidInput);
	TestEqual(TEXT("Invalid timestep status"), Result.Status, CableSim::ESimulationStatus::InvalidConfiguration);
	TestEqual(TEXT("Invalid timestep does not advance"), Solver.GetStepIndex(), InitialStep);

	CableSim::FStateSnapshot InvalidSnapshot = Solver.CaptureState();
	InvalidSnapshot.Particles[1].Position.X = std::numeric_limits<double>::quiet_NaN();
	TestFalse(TEXT("NaN snapshot is rejected"), Solver.RestoreState(InvalidSnapshot));
	TestTrue(TEXT("Rejected snapshot preserves valid solver"), Solver.IsInitialized());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimGuideConstraintTest,
	"CableSim.Core.Guides.CorridorProjection",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimGuideConstraintTest::RunTest(const FString& Parameters)
{
	CableSim::FSimulationConfig Config = CableSimTests::MakeConfig(20.0, 10.0);
	Config.ConstraintIterations = 4;
	Config.DistanceOverRelaxation = 1.0;
	CableSim::FSolver Solver;
	Solver.Initialize(FVector3d(-10.0, 0.0, 0.0), FVector3d(10.0, 0.0, 0.0), Config);
	CableSim::FStateSnapshot State = Solver.CaptureState();
	State.Particles[1].Position.Y = 10.0;
	State.Particles[1].PreviousPosition = State.Particles[1].Position;
	Solver.RestoreState(State);
	CableSim::FStepInput Input = CableSimTests::MakeFreeInput();
	CableSim::FGuideConstraint& Guide = Input.GuideConstraints.AddDefaulted_GetRef();
	Guide.ParticleIndex = 1;
	Guide.TargetPosition = FVector3d::ZeroVector;
	Guide.MaximumDistance = 2.0;
	Guide.StepStrength = 1.0;
	const CableSim::FStepResult Result = Solver.AdvanceStep(Input);
	TestEqual(TEXT("One guide is reported"), Result.GuideConstraintCount, 1);
	TestTrue(TEXT("Guided particle is inside the corridor"),
		Solver.GetParticles()[1].Position.Length() <= 2.0 + 1.e-9);
	TestTrue(TEXT("Guide residual is solved"), Result.MaximumGuideError <= 1.e-9);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimInSolverFrictionTest,
	"CableSim.Core.Friction.InSolverAnchor",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimInSolverFrictionTest::RunTest(const FString& Parameters)
{
		auto Simulate = [](const bool bEnableFriction)
		{
		CableSim::FSimulationConfig Config = CableSimTests::MakeConfig(20.0, 10.0);
		Config.ConstraintIterations = 8;
			Config.DistanceOverRelaxation = 1.0;
			Config.bEnableFriction = bEnableFriction;
			Config.StaticFrictionCoefficient = 10.0;
			Config.StaticFrictionSpeedThreshold = 100.0;
		CableSim::FSolver Solver;
		Solver.Initialize(FVector3d(-10.0, 0.0, 0.0), FVector3d(10.0, 0.0, 0.0), Config);
		CableSim::FStateSnapshot State = Solver.CaptureState();
			State.Particles[1].Velocity = FVector3d(0.0, 60.0, 0.0);
			State.Particles[1].EstimatedNormalLoad = 10.0;
			State.Particles[1].Position.Z = -0.1;
			State.Particles[1].PreviousPosition = State.Particles[1].Position;
		Solver.RestoreState(State);
		Solver.AdvanceStep(
			CableSimTests::MakeFreeInput(),
			[bEnableFriction](const TConstArrayView<CableSim::FParticle> Particles,
				TArray<CableSim::FContactConstraint>& Contacts)
			{
				CableSim::FContactConstraint& Contact = Contacts.AddDefaulted_GetRef();
				Contact.FeatureId = 1;
				Contact.ParticleIndex = 1;
				Contact.Normal = FVector3d::UnitZ();
				Contact.MinimumNormalCoordinate = 0.0;
				Contact.FrictionAnchorPosition = Particles[1].PreviousPosition;
					Contact.bHasFrictionAnchor = bEnableFriction;
			});
		double CenterOfMassY = 0.0;
		for (const CableSim::FParticle& Particle : Solver.GetParticles())
		{
			CenterOfMassY += Particle.Position.Y;
		}
		return CenterOfMassY / Solver.GetParticles().Num();
	};

	const double WithoutFriction = FMath::Abs(Simulate(false));
	const double WithFriction = FMath::Abs(Simulate(true));
	TestTrue(TEXT("In-solver friction removes tangential drift"),
		WithFriction < WithoutFriction * 0.5);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimContactRefreshReplacementTest,
	"CableSim.Core.Collision.RefreshReplacesActiveSet",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimContactRefreshReplacementTest::RunTest(const FString& Parameters)
{
	CableSim::FSimulationConfig Config = CableSimTests::MakeConfig(20.0, 10.0);
	Config.ConstraintIterations = 4;
	CableSim::FSolver Solver;
	Solver.Initialize(FVector3d(-10.0, 0.0, 0.0), FVector3d(10.0, 0.0, 0.0), Config);
	int32 GeneratorCallCount = 0;
	const CableSim::FStepResult Result = Solver.AdvanceStep(
		CableSimTests::MakeFreeInput(),
		[&GeneratorCallCount](const TConstArrayView<CableSim::FParticle>, TArray<CableSim::FContactConstraint>& Contacts)
		{
			if (GeneratorCallCount++ == 0)
			{
				CableSim::FContactConstraint& Contact = Contacts.AddDefaulted_GetRef();
				Contact.FeatureId = 7;
				Contact.ParticleIndex = 1;
				Contact.Normal = FVector3d::UnitZ();
				Contact.MinimumNormalCoordinate = 1.0;
			}
		});
	TestEqual(TEXT("Generator runs at integration and midpoint"), GeneratorCallCount, 2);
	TestEqual(TEXT("Midpoint refresh removes stale active contacts"), Result.ContactCount, 0);
	TestEqual(TEXT("Refresh count reports the replacement set"), Result.RefreshedContactCount, 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimFrictionIterationIndependenceTest,
	"CableSim.Core.Friction.IterationIndependentDynamicBudget",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimFrictionIterationIndependenceTest::RunTest(const FString& Parameters)
{
	auto Simulate = [](const int32 Iterations)
	{
		CableSim::FSimulationConfig Config = CableSimTests::MakeConfig(20.0, 10.0);
		Config.ConstraintIterations = Iterations;
		Config.DistanceOverRelaxation = 1.0;
		Config.bEnableFriction = true;
		Config.DynamicFrictionCoefficient = 0.5;
		Config.StaticFrictionSpeedThreshold = 2.0;
		CableSim::FSolver Solver;
		Solver.Initialize(FVector3d(-10.0, 0.0, 0.0), FVector3d(10.0, 0.0, 0.0), Config);
		CableSim::FStateSnapshot State = Solver.CaptureState();
		State.Particles[1].Velocity = FVector3d(0.0, 100.0, 0.0);
		State.Particles[1].EstimatedNormalLoad = 10.0;
		State.Particles[1].Position.Z = -0.1;
		State.Particles[1].PreviousPosition = State.Particles[1].Position;
		Solver.RestoreState(State);
		Solver.AdvanceStep(CableSimTests::MakeFreeInput(), [](
			const TConstArrayView<CableSim::FParticle> Particles,
			TArray<CableSim::FContactConstraint>& Contacts)
		{
			CableSim::FContactConstraint& Contact = Contacts.AddDefaulted_GetRef();
			Contact.FeatureId = 9;
			Contact.ParticleIndex = 1;
			Contact.Normal = FVector3d::UnitZ();
			Contact.MinimumNormalCoordinate = 0.0;
			Contact.FrictionAnchorPosition = Particles[1].PreviousPosition;
			Contact.bHasFrictionAnchor = true;
		});
		return Solver.GetParticles()[1].Velocity.Y;
	};

	const double FourIterations = Simulate(4);
	const double SixtyFourIterations = Simulate(64);
	TestTrue(TEXT("Dynamic Coulomb budget does not compound per solver iteration"),
		FMath::IsNearlyEqual(FourIterations, SixtyFourIterations, 0.5));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimImpossibleContactTest,
	"CableSim.Core.Collision.ImpossiblePlaneRejected",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimImpossibleContactTest::RunTest(const FString& Parameters)
{
	CableSim::FSimulationConfig Config = CableSimTests::MakeConfig(20.0, 10.0);
	Config.ConstraintIterations = 4;
	CableSim::FSolver Solver;
	Solver.Initialize(FVector3d::ZeroVector, FVector3d(20.0, 0.0, 0.0), Config);
	CableSim::FStepInput Input = CableSimTests::MakeFreeInput();
	Input.StartEndpoint.Mode = CableSim::EParticleMode::Kinematic;
	Input.StartEndpoint.TargetPosition = FVector3d::ZeroVector;
	const CableSim::FStepResult Result = Solver.AdvanceStep(
		Input,
		[](const TConstArrayView<CableSim::FParticle>, TArray<CableSim::FContactConstraint>& Contacts)
		{
			CableSim::FContactConstraint& Contact = Contacts.AddDefaulted_GetRef();
			Contact.FeatureId = 2;
			Contact.ParticleIndex = 1;
			Contact.Normal = FVector3d::UnitX();
			Contact.MinimumNormalCoordinate = 100.0;
		});
	TestEqual(TEXT("Unreachable plane is discarded"), Result.ContactCount, 0);
	TestTrue(TEXT("Solver remains finite"), !Solver.IsSuspended());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimGuideIterationStrengthTest,
	"CableSim.Core.Guides.IterationIndependentStrength",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimGuideIterationStrengthTest::RunTest(const FString& Parameters)
{
	constexpr double StepStrength = 0.85;
	for (const int32 Iterations : {1, 4, 16, 64, 256})
	{
		const double IterationStrength = CableSim::FSolver::CalculateIterationStrength(StepStrength, Iterations);
		const double ComposedStrength = 1.0 - FMath::Pow(1.0 - IterationStrength, Iterations);
		TestTrue(
			FString::Printf(TEXT("Guide strength composes to the requested step strength at %d iterations"), Iterations),
			FMath::IsNearlyEqual(ComposedStrength, StepStrength, 1.e-12));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimSmoothSlackSettlingTest,
	"CableSim.Core.Bending.SmoothSlackSettling",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimSmoothSlackSettlingTest::RunTest(const FString& Parameters)
{
	CableSim::FSimulationConfig Config = CableSimTests::MakeConfig(400.0, 10.0);
	Config.Gravity = FVector3d(0.0, 0.0, -980.665);
	Config.VelocityDamping = 0.01;
	Config.BendingStepStrength = 0.20;
	Config.FreeBendAngleRadiansPerMeter = FMath::DegreesToRadians(90.0);
	Config.DistanceOverRelaxation = 1.015;
	Config.ConstraintIterations = 64;

	const FVector3d Start(-175.0, 0.0, 0.0);
	const FVector3d End(175.0, 0.0, 0.0);
	CableSim::FSolver Solver;
	TestTrue(TEXT("Slack settling solver initializes"), Solver.Initialize(Start, End, Config));
	const CableSim::FStepInput Input = CableSimTests::MakeFixedInput(Start, End);
	CableSim::FStepResult Result;
	for (int32 Step = 0; Step < 600; ++Step)
	{
		Result = Solver.AdvanceStep(Input);
	}

	TestEqual(TEXT("Slack cable remains ready"), Result.Status, CableSim::ESimulationStatus::Ready);
	TestFalse(TEXT("Slack cable does not suspend"), Solver.IsSuspended());
	const TArray<CableSim::FParticle>& Particles = Solver.GetParticles();
	double PolylineLength = 0.0;
	double MaximumTurnAngle = 0.0;
	for (int32 Index = 0; Index + 1 < Particles.Num(); ++Index)
	{
		PolylineLength += FVector3d::Distance(Particles[Index].Position, Particles[Index + 1].Position);
	}
	for (int32 Index = 1; Index + 1 < Particles.Num(); ++Index)
	{
		const FVector3d Incoming = (Particles[Index].Position - Particles[Index - 1].Position).GetSafeNormal();
		const FVector3d Outgoing = (Particles[Index + 1].Position - Particles[Index].Position).GetSafeNormal();
		MaximumTurnAngle = FMath::Max(
			MaximumTurnAngle,
			FMath::Acos(FMath::Clamp(FVector3d::DotProduct(Incoming, Outgoing), -1.0, 1.0)));
	}
	TestTrue(TEXT("Settled polyline retains cable length"), FMath::Abs(PolylineLength - Config.RestLength) < 8.0);
	TestTrue(TEXT("Settled cable has no particle-scale kink"), MaximumTurnAngle < FMath::DegreesToRadians(45.0));
	TestTrue(TEXT("Settled cable velocity is low"), Result.RmsParticleSpeed < 2.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimNodeCountBenchmarkTest,
	"CableSim.Core.Performance.NodeCountScaling",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimNodeCountBenchmarkTest::RunTest(const FString& Parameters)
{
	constexpr double RestLength = 1024.0;
	for (const int32 NodeCount : {33, 129, 513, 1025})
	{
		CableSim::FSimulationConfig Config = CableSimTests::MakeConfig(
			RestLength,
			RestLength / static_cast<double>(NodeCount - 1));
		Config.ConstraintIterations = 8;
		Config.DistanceOverRelaxation = 1.0;
		CableSim::FSolver Solver;
		TestTrue(TEXT("Benchmark solver initializes"), Solver.Initialize(
			FVector3d::ZeroVector,
			FVector3d(RestLength, 0.0, 0.0),
			Config));
		const CableSim::FStepInput Input = CableSimTests::MakeFixedInput(
			FVector3d::ZeroVector,
			FVector3d(RestLength, 0.0, 0.0));
		const double StartTime = FPlatformTime::Seconds();
		for (int32 Step = 0; Step < 8; ++Step)
		{
			Solver.AdvanceStep(Input);
		}
		const double MicrosecondsPerStep = (FPlatformTime::Seconds() - StartTime) * 1.e6 / 8.0;
		AddInfo(FString::Printf(
			TEXT("Node benchmark: %d nodes, 8 iterations, %.1f us/step"),
			NodeCount,
			MicrosecondsPerStep));
		TestEqual(TEXT("Benchmark particle count"), Solver.GetParticles().Num(), NodeCount);
		TestFalse(TEXT("Benchmark remains numerically valid"), Solver.IsSuspended());
	}
	return true;
}

#endif
