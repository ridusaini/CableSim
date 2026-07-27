#if WITH_DEV_AUTOMATION_TESTS

#include "CableSimSolver.h"
#include "CableSimContactManifold.h"
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
	FCableSimOneManifoldPerStepTest,
	"CableSim.Core.Collision.OneManifoldPerStep",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimOneManifoldPerStepTest::RunTest(const FString& Parameters)
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
			++GeneratorCallCount;
			CableSim::FContactConstraint& Contact = Contacts.AddDefaulted_GetRef();
			Contact.FeatureId = 7;
			Contact.ParticleIndex = 1;
			Contact.Normal = FVector3d::UnitZ();
			Contact.MinimumNormalCoordinate = 1.0;
		});
	TestEqual(TEXT("Contacts are generated once per step"), GeneratorCallCount, 1);
	TestEqual(TEXT("The single contact set is kept for the whole step"), Result.ContactCount, 1);
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

// Resting-regression harness. Drives the solver with stable analytical contacts
// to isolate core resting behavior from runtime sweep-adapter churn. Thresholds
// are loose baselines to tighten as Phase 1/2 land.

namespace CableSimTests
{
	CableSim::FSimulationConfig MakeSettleConfig(
		const double RestLength = 400.0,
		const double NodeSpacing = 10.0)
	{
		CableSim::FSimulationConfig Config;
		Config.RestLength = RestLength;
		Config.NodeSpacing = NodeSpacing;
		Config.ParticleMass = 0.05;
		Config.Gravity = FVector3d(0.0, 0.0, -980.665);
		Config.VelocityDamping = 0.01;
		Config.BendingStepStrength = 0.20;
		Config.FreeBendAngleRadiansPerMeter = FMath::DegreesToRadians(90.0);
		Config.DistanceOverRelaxation = 1.015;
		Config.ConstraintIterations = 64;
		Config.bEnableFriction = true;
		Config.StaticFrictionCoefficient = 0.35;
		Config.DynamicFrictionCoefficient = 0.25;
		Config.StaticFrictionSpeedThreshold = 2.0;
		return Config;
	}

	struct FSettleMetrics
	{
		double WindowAverageRmsSpeed = 0.0;
		double WindowMaxSpeed = 0.0;
	};

	FSettleMetrics RunSettle(
		CableSim::FSolver& Solver,
		const CableSim::FStepInput& Input,
		const int32 StepCount,
		const int32 WindowSteps,
		const CableSim::FContactGenerator& ContactGenerator)
	{
		double RmsSum = 0.0;
		double MaxSpeed = 0.0;
		const int32 WindowStart = FMath::Max(StepCount - WindowSteps, 0);
		for (int32 Step = 0; Step < StepCount; ++Step)
		{
			const CableSim::FStepResult Result = Solver.AdvanceStep(Input, ContactGenerator);
			if (Step >= WindowStart)
			{
				RmsSum += Result.RmsParticleSpeed;
				MaxSpeed = FMath::Max(MaxSpeed, Result.MaximumParticleSpeed);
			}
		}
		FSettleMetrics Metrics;
		Metrics.WindowAverageRmsSpeed = RmsSum / static_cast<double>(FMath::Max(StepCount - WindowStart, 1));
		Metrics.WindowMaxSpeed = MaxSpeed;
		return Metrics;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimRestingHangingTest,
	"CableSim.Core.Resting.HangingNoCollision",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimRestingHangingTest::RunTest(const FString& Parameters)
{
	// No contacts: isolates the base solver from collision.
	const FVector3d Start(-100.0, 0.0, 0.0);
	const FVector3d End(100.0, 0.0, 0.0);
	CableSim::FSolver Solver;
	TestTrue(TEXT("Hanging solver initializes"),
		Solver.Initialize(Start, End, CableSimTests::MakeSettleConfig()));
	const CableSim::FStepInput Input = CableSimTests::MakeFixedInput(Start, End);
	const CableSimTests::FSettleMetrics Metrics =
		CableSimTests::RunSettle(Solver, Input, 900, 120, CableSim::FContactGenerator{});
	AddInfo(FString::Printf(
		TEXT("R1 hanging: window-avg RMS %.4f cm/s, window-max %.4f cm/s"),
		Metrics.WindowAverageRmsSpeed, Metrics.WindowMaxSpeed));
	TestFalse(TEXT("Hanging cable does not suspend"), Solver.IsSuspended());
	TestTrue(TEXT("Hanging cable settles (RMS < 1 cm/s)"),
		Metrics.WindowAverageRmsSpeed < 1.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimRestingOnPlaneTest,
	"CableSim.Core.Resting.OnPlane",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimRestingOnPlaneTest::RunTest(const FString& Parameters)
{
	// R2: a slack cable draped so its middle rests on a horizontal ground plane.
	const FVector3d Start(-150.0, 0.0, 20.0);
	const FVector3d End(150.0, 0.0, 20.0);
	CableSim::FSolver Solver;
	TestTrue(TEXT("On-plane solver initializes"),
		Solver.Initialize(Start, End, CableSimTests::MakeSettleConfig()));
	const CableSim::FStepInput Input = CableSimTests::MakeFixedInput(Start, End);
	const CableSim::FContactGenerator GroundPlane =
		[](const TConstArrayView<CableSim::FParticle> Particles,
			TArray<CableSim::FContactConstraint>& Contacts)
	{
		for (int32 Index = 0; Index < Particles.Num(); ++Index)
		{
			if (Particles[Index].Mode != CableSim::EParticleMode::Dynamic
				|| Particles[Index].Position.Z > 0.5)
			{
				continue;
			}
			CableSim::FContactConstraint& Contact = Contacts.AddDefaulted_GetRef();
			Contact.FeatureId = static_cast<uint64>(Index);
			Contact.ParticleIndex = Index;
			Contact.Normal = FVector3d::UnitZ();
			Contact.MinimumNormalCoordinate = 0.0;
			Contact.FrictionAnchorPosition = Particles[Index].PreviousPosition;
			Contact.bHasFrictionAnchor = true;
		}
	};
	const CableSimTests::FSettleMetrics Metrics =
		CableSimTests::RunSettle(Solver, Input, 900, 120, GroundPlane);
	double LowestParticleZ = TNumericLimits<double>::Max();
	for (const CableSim::FParticle& Particle : Solver.GetParticles())
	{
		LowestParticleZ = FMath::Min(LowestParticleZ, Particle.Position.Z);
	}
	AddInfo(FString::Printf(
		TEXT("R2 on-plane: window-avg RMS %.4f cm/s, window-max %.4f cm/s, lowest Z %.4f cm"),
		Metrics.WindowAverageRmsSpeed, Metrics.WindowMaxSpeed, LowestParticleZ));
	TestFalse(TEXT("On-plane cable does not suspend"), Solver.IsSuspended());
	TestTrue(TEXT("On-plane cable does not sink through the plane"), LowestParticleZ > -0.5);
	TestTrue(TEXT("On-plane cable settles (RMS < 2 cm/s)"),
		Metrics.WindowAverageRmsSpeed < 2.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimStaticFrictionSlopeTest,
	"CableSim.Core.Friction.StaticHoldsOnSlope",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimStaticFrictionSlopeTest::RunTest(const FString& Parameters)
{
	// The dynamic node rests on a horizontal plane (+Z) and hangs from a kinematic
	// anchor placed straight up the normal, so the segment gives negligible restoring
	// for small tangential motion. A horizontal gravity component pulls the node
	// along the plane; with tan(angle) below the friction coefficient it must be held
	// statically. Anchor-along-normal is what gives the node a genuine tangential DOF
	// (unlike a taut span, where distance constraints pin it regardless of friction).
	auto MeasureDrift = [](const double StaticFrictionCoefficient)
	{
		CableSim::FSimulationConfig Config = CableSimTests::MakeSettleConfig(20.0, 20.0);
		Config.ConstraintIterations = 16;
		Config.DistanceOverRelaxation = 1.0;
		Config.Gravity = FVector3d(300.0, 0.0, -980.665);
		Config.StaticFrictionCoefficient = StaticFrictionCoefficient;
		const FVector3d Anchor(0.0, 0.0, 20.0);
		const FVector3d NodeStart(0.0, 0.0, 0.0);
		CableSim::FSolver Solver;
		Solver.Initialize(Anchor, NodeStart, Config);
		CableSim::FStepInput Input = CableSimTests::MakeFreeInput();
		Input.StartEndpoint.Mode = CableSim::EParticleMode::Kinematic;
		Input.StartEndpoint.TargetPosition = Anchor;
		Input.EndEndpoint.Mode = CableSim::EParticleMode::Dynamic;
		const CableSim::FContactGenerator Ground =
			[](const TConstArrayView<CableSim::FParticle> Particles,
				TArray<CableSim::FContactConstraint>& Contacts)
		{
			const int32 Index = Particles.Num() - 1;
			if (Particles[Index].Mode != CableSim::EParticleMode::Dynamic)
			{
				return;
			}
			CableSim::FContactConstraint& Contact = Contacts.AddDefaulted_GetRef();
			Contact.FeatureId = 1;
			Contact.ParticleIndex = Index;
			Contact.Normal = FVector3d::UnitZ();
			Contact.MinimumNormalCoordinate = 0.0;
			Contact.FrictionAnchorPosition = Particles[Index].PreviousPosition;
			Contact.bHasFrictionAnchor = true;
		};
		for (int32 Step = 0; Step < 300; ++Step)
		{
			Solver.AdvanceStep(Input, Ground);
		}
		return FMath::Abs(Solver.GetParticles().Last().Position.X);
	};

	const double DriftWithoutFriction = MeasureDrift(0.0);
	const double DriftWithFriction = MeasureDrift(1.0);
	AddInfo(FString::Printf(
		TEXT("Slope friction: tangential drift %.4f cm (mu=0) vs %.4f cm (mu=1)"),
		DriftWithoutFriction, DriftWithFriction));
	TestTrue(TEXT("Static friction meaningfully reduces tangential creep"),
		DriftWithFriction < DriftWithoutFriction * 0.5);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimEdgeFrictionHoldsTest,
	"CableSim.Core.Manifold.EdgeContactFrictionHolds",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimEdgeFrictionHoldsTest::RunTest(const FString& Parameters)
{
	// A node resting in a convex edge's wedge, contact generated by the manifold
	// compiler, must be held against the tangential gravity component by static
	// friction (else the cable slides across the edge and off the object).
	CableSim::FCollisionEdge Edge;
	Edge.Id = {1, 0, CableSim::ECollisionFeatureType::Edge, 0, INDEX_NONE};
	Edge.GeometryType = CableSim::ECollisionGeometryType::Box;
	Edge.bStaticObject = true;
	Edge.Start = FVector3d(0.0, -50.0, 0.0);
	Edge.End = FVector3d(0.0, 50.0, 0.0);
	Edge.FaceNormal0 = FVector3d::UnitZ();
	Edge.FaceNormal1 = FVector3d::UnitX();
	Edge.Kind = CableSim::ECollisionEdgeKind::Convex;
	const TArray<CableSim::FCollisionEdge> Edges = {Edge};

	auto MeasureDrift = [&Edges, this](const double StaticFrictionCoefficient)
	{
		CableSim::FSimulationConfig Config = CableSimTests::MakeSettleConfig(20.0, 20.0);
		Config.ConstraintIterations = 16;
		Config.DistanceOverRelaxation = 1.0;
		Config.StaticFrictionCoefficient = StaticFrictionCoefficient;
		const FVector3d RadialDirection = FVector3d(1.0, 0.0, 1.0).GetSafeNormal();
		const FVector3d NodeStart = RadialDirection * 5.0;
		const FVector3d Anchor = NodeStart + RadialDirection * 20.0;
		CableSim::FSolver Solver;
		Solver.Initialize(Anchor, NodeStart, Config);
		CableSim::FStepInput Input = CableSimTests::MakeFreeInput();
		Input.StartEndpoint.Mode = CableSim::EParticleMode::Kinematic;
		Input.StartEndpoint.TargetPosition = Anchor;
		Input.EndEndpoint.Mode = CableSim::EParticleMode::Dynamic;
		CableSim::FManifoldConfig ManifoldConfig;
		ManifoldConfig.NodeRadius = 5.0;
		const CableSim::FContactGenerator Generator =
			[&Edges, ManifoldConfig](const TConstArrayView<CableSim::FParticle> Particles,
				TArray<CableSim::FContactConstraint>& Contacts)
		{
			const int32 Index = Particles.Num() - 1;
			CableSim::FContactManifoldCompiler::CompileNodeContacts(
				Index, Particles[Index].Position, Particles[Index].PreviousPosition,
				{}, Edges, ManifoldConfig, Contacts);
		};
		const FVector3d Initial = Solver.GetParticles().Last().Position;
		for (int32 Step = 0; Step < 300; ++Step)
		{
			Solver.AdvanceStep(Input, Generator);
			if (Step == 5)
			{
				const CableSim::FStepResult& R = Solver.GetLastStepResult();
				AddInfo(FString::Printf(
					TEXT("mu=%.0f step5: contacts=%d projected=%d anchors=%d maxLoad=%.4f nodePos=(%.2f,%.2f,%.2f)"),
					StaticFrictionCoefficient, R.ContactCount, R.ProjectedContactCount,
					R.StaticFrictionAnchorCount, R.MaximumEstimatedNormalLoad,
					Solver.GetParticles().Last().Position.X, Solver.GetParticles().Last().Position.Y,
					Solver.GetParticles().Last().Position.Z));
			}
		}
		return FVector3d::Distance(Solver.GetParticles().Last().Position, Initial);
	};

	const double DriftWithoutFriction = MeasureDrift(0.0);
	const double DriftWithFriction = MeasureDrift(2.0);
	AddInfo(FString::Printf(
		TEXT("Edge friction: drift %.4f cm (mu=0) vs %.4f cm (mu=2)"),
		DriftWithoutFriction, DriftWithFriction));
	TestTrue(TEXT("Static friction holds the node on the convex edge"),
		DriftWithFriction < DriftWithoutFriction * 0.5);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimDrapeOverEdgeTest,
	"CableSim.Core.Manifold.DrapeOverEdgeHolds",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimDrapeOverEdgeTest::RunTest(const FString& Parameters)
{
	// A symmetric cable draped over a real convex ridge (edge compiled by the
	// topology compiler, contacts by the manifold compiler) must settle centered on
	// the edge without sliding off or bouncing.
	auto MakeTri = [](const int32 Index, const int32 I0, const int32 I1, const int32 I2,
		const FVector3d& V0, const FVector3d& V1, const FVector3d& V2)
	{
		CableSim::FCollisionTriangle Triangle;
		Triangle.Id = {1, 0, CableSim::ECollisionFeatureType::Triangle, Index, INDEX_NONE};
		Triangle.GeometryType = CableSim::ECollisionGeometryType::TriangleMesh;
		Triangle.bStaticObject = true;
		Triangle.Vertices[0] = V0;
		Triangle.Vertices[1] = V1;
		Triangle.Vertices[2] = V2;
		Triangle.VertexIndices[0] = I0;
		Triangle.VertexIndices[1] = I1;
		Triangle.VertexIndices[2] = I2;
		return Triangle;
	};
	const FVector3d V0(0.0, -60.0, 0.0);
	const FVector3d V1(0.0, 60.0, 0.0);
	const FVector3d V2(-60.0, 0.0, -60.0);
	const FVector3d V3(60.0, 0.0, -60.0);
	TArray<CableSim::FCollisionTriangle> Triangles;
	Triangles.Add(MakeTri(0, 0, 1, 2, V0, V1, V2));
	Triangles.Add(MakeTri(1, 1, 0, 3, V1, V0, V3));
	TArray<CableSim::FCollisionEdge> Edges;
	TArray<CableSim::FCollisionVertex> Vertices;
	const CableSim::FTopologyCompileDiagnostics TopologyDiagnostics =
		CableSim::FCollisionTopologyCompiler::CompileTopology(Triangles, 0.1, 8, Edges, Vertices);
	TestTrue(TEXT("Ridge compiles a convex edge"), TopologyDiagnostics.ConvexEdgeCount >= 1);

	CableSim::FSimulationConfig Config = CableSimTests::MakeSettleConfig(200.0, 10.0);
	const FVector3d Start(-80.0, 0.0, -30.0);
	const FVector3d End(80.0, 0.0, -30.0);
	CableSim::FSolver Solver;
	TestTrue(TEXT("Drape solver initializes"), Solver.Initialize(Start, End, Config));
	const CableSim::FStepInput Input = CableSimTests::MakeFixedInput(Start, End);
	CableSim::FManifoldConfig ManifoldConfig;
	const CableSim::FContactGenerator Generator =
		[&Triangles, &Edges, ManifoldConfig](const TConstArrayView<CableSim::FParticle> Particles,
			TArray<CableSim::FContactConstraint>& Contacts)
	{
		for (int32 Index = 0; Index < Particles.Num(); ++Index)
		{
			if (Particles[Index].Mode != CableSim::EParticleMode::Dynamic)
			{
				continue;
			}
			CableSim::FContactManifoldCompiler::CompileNodeContacts(
				Index, Particles[Index].Position, Particles[Index].PreviousPosition,
				Triangles, Edges, ManifoldConfig, Contacts);
		}
	};

	double WindowRms = 0.0;
	CableSim::FStepResult Result;
	for (int32 Step = 0; Step < 600; ++Step)
	{
		Result = Solver.AdvanceStep(Input, Generator);
		if (Step >= 480)
		{
			WindowRms += Result.RmsParticleSpeed;
		}
	}
	WindowRms /= 120.0;

	const TArray<CableSim::FParticle>& Particles = Solver.GetParticles();
	double CenterOfMassX = 0.0;
	for (const CableSim::FParticle& Particle : Particles)
	{
		CenterOfMassX += Particle.Position.X;
	}
	CenterOfMassX /= static_cast<double>(Particles.Num());
	const FVector3d MiddleNode = Particles[Particles.Num() / 2].Position;
	AddInfo(FString::Printf(
		TEXT("Drape: window-avg RMS %.4f cm/s, CoM.X %.3f cm, middle node (%.2f,%.2f,%.2f)"),
		WindowRms, CenterOfMassX, MiddleNode.X, MiddleNode.Y, MiddleNode.Z));

	TestFalse(TEXT("Drape does not suspend"), Solver.IsSuspended());
	TestTrue(TEXT("Drape stays centered on the ridge (no slide-off)"), FMath::Abs(CenterOfMassX) < 15.0);
	TestTrue(TEXT("Middle node rests on the ridge, not down a slope"),
		FMath::Abs(MiddleNode.X) < 15.0 && MiddleNode.Z > -15.0);
	TestTrue(TEXT("Drape settles without bouncing (RMS < 3 cm/s)"), WindowRms < 3.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimBackFacePruneTest,
	"CableSim.Core.Manifold.BackFacePruned",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimBackFacePruneTest::RunTest(const FString& Parameters)
{
	auto MakeTri = [](const int32 Index, const FVector3d& V0, const FVector3d& V1, const FVector3d& V2)
	{
		CableSim::FCollisionTriangle Triangle;
		Triangle.Id = {1, 0, CableSim::ECollisionFeatureType::Triangle, Index, INDEX_NONE};
		Triangle.GeometryType = CableSim::ECollisionGeometryType::TriangleMesh;
		Triangle.bStaticObject = true;
		Triangle.Vertices[0] = V0;
		Triangle.Vertices[1] = V1;
		Triangle.Vertices[2] = V2;
		return Triangle;
	};
	TArray<CableSim::FCollisionTriangle> Triangles;
	Triangles.Add(MakeTri(0, FVector3d(-10, -10, 0), FVector3d(10, -10, 0), FVector3d(0, 10, 0)));
	Triangles.Add(MakeTri(1, FVector3d(-10, -10, -2.5), FVector3d(0, 10, -2.5), FVector3d(10, -10, -2.5)));

	const FVector3d NodePosition(0.0, 0.0, 3.0);
	CableSim::FManifoldConfig ManifoldConfig;
	TArray<CableSim::FContactConstraint> Contacts;
	CableSim::FContactManifoldCompiler::CompileNodeContacts(
		0, NodePosition, NodePosition, Triangles, {}, ManifoldConfig, Contacts);

	TestEqual(TEXT("Only the front-facing plane survives"), Contacts.Num(), 1);
	if (Contacts.Num() == 1)
	{
		TestTrue(TEXT("Surviving plane faces the node (+Z)"),
			FVector3d::DotProduct(Contacts[0].Normal, FVector3d::UnitZ()) > 0.99);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimGuideCorridorTest,
	"CableSim.Core.Coupling.GuideCorridorAbsorbsJitter",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimGuideCorridorTest::RunTest(const FString& Parameters)
{
	const double RestHeight = 5.0;
	const CableSim::FContactGenerator PlaneGenerator =
		[RestHeight](const TConstArrayView<CableSim::FParticle> Particles,
			TArray<CableSim::FContactConstraint>& Contacts)
	{
		for (int32 Index = 0; Index < Particles.Num(); ++Index)
		{
			if (Particles[Index].Mode != CableSim::EParticleMode::Dynamic)
			{
				continue;
			}
			CableSim::FContactConstraint& Contact = Contacts.AddDefaulted_GetRef();
			Contact.FeatureId = 1;
			Contact.ParticleIndex = Index;
			Contact.Normal = FVector3d::UnitZ();
			Contact.MinimumNormalCoordinate = RestHeight;
			Contact.FrictionAnchorPosition = Particles[Index].PreviousPosition;
			Contact.bHasFrictionAnchor = true;
		}
	};

	auto MeasureJitterResponse = [&PlaneGenerator, RestHeight](const double Corridor)
	{
		CableSim::FSimulationConfig Config = CableSimTests::MakeSettleConfig(300.0, 10.0);
		const FVector3d Start(-150.0, 0.0, RestHeight);
		const FVector3d End(150.0, 0.0, RestHeight);
		CableSim::FSolver Solver;
		Solver.Initialize(Start, End, Config);
		CableSim::FStepInput Input = CableSimTests::MakeFixedInput(Start, End);
		for (int32 Step = 0; Step < 200; ++Step)
		{
			Solver.AdvanceStep(Input, PlaneGenerator);
		}
		const int32 Middle = Solver.GetParticles().Num() / 2;
		double RmsSum = 0.0;
		for (int32 Step = 0; Step < 200; ++Step)
		{
			Input.GuideConstraints.Reset();
			CableSim::FGuideConstraint& Guide = Input.GuideConstraints.AddDefaulted_GetRef();
			Guide.ParticleIndex = Middle;
			Guide.TargetPosition = FVector3d(
				Solver.GetParticles()[Middle].Position.X,
				(Step % 2 == 0) ? 0.5 : -0.5,
				RestHeight);
			Guide.MaximumDistance = Corridor;
			Guide.StepStrength = 0.85;
			const CableSim::FStepResult Result = Solver.AdvanceStep(Input, PlaneGenerator);
			if (Step >= 100)
			{
				RmsSum += Result.RmsParticleSpeed;
			}
		}
		return RmsSum / 100.0;
	};

	const double FlooredRms = MeasureJitterResponse(5.0);
	const double PinnedRms = MeasureJitterResponse(0.0);
	AddInfo(FString::Printf(
		TEXT("Guide jitter response: floored corridor %.3f cm/s vs zero corridor %.3f cm/s"),
		FlooredRms, PinnedRms));
	TestTrue(TEXT("A floored corridor absorbs sample jitter (stays at rest)"), FlooredRms < 2.0);
	TestTrue(TEXT("A zero corridor pins to the jittering sample and buzzes"), PinnedRms > 10.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimTautRestBuzzTest,
	"CableSim.Core.Resting.NearlyTautOnPlane",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimTautRestBuzzTest::RunTest(const FString& Parameters)
{
	auto MakeTri = [](const int32 Index, const int32 I0, const int32 I1, const int32 I2,
		const FVector3d& A, const FVector3d& B, const FVector3d& C)
	{
		CableSim::FCollisionTriangle Triangle;
		Triangle.Id = {1, 0, CableSim::ECollisionFeatureType::Triangle, Index, INDEX_NONE};
		Triangle.GeometryType = CableSim::ECollisionGeometryType::TriangleMesh;
		Triangle.bStaticObject = true;
		Triangle.Vertices[0] = A;
		Triangle.Vertices[1] = B;
		Triangle.Vertices[2] = C;
		Triangle.VertexIndices[0] = I0;
		Triangle.VertexIndices[1] = I1;
		Triangle.VertexIndices[2] = I2;
		return Triangle;
	};
	// Flat top (+Z) meeting a down-ramp (normal ~0.26,0,0.97) at a convex bend along Y,
	// matching the live scene's contact normals.
	const FVector3d A(-120, -60, 0), B(-120, 60, 0), C(0, -60, 0), D(0, 60, 0), E(120, -60, -32), F(120, 60, -32);
	TArray<CableSim::FCollisionTriangle> Triangles;
	Triangles.Add(MakeTri(0, 0, 3, 1, A, D, B));
	Triangles.Add(MakeTri(1, 0, 2, 3, A, C, D));
	Triangles.Add(MakeTri(2, 2, 5, 3, C, F, D));
	Triangles.Add(MakeTri(3, 2, 4, 5, C, E, F));
	TArray<CableSim::FCollisionEdge> Edges;
	TArray<CableSim::FCollisionVertex> Verts;
	CableSim::FCollisionTopologyCompiler::CompileTopology(Triangles, 0.1, 8, Edges, Verts);

	const FVector3d Start(-120.0, 0.0, 5.0);
	const FVector3d End(120.0, 0.0, -27.0);
	CableSim::FSolver Solver;
	TestTrue(TEXT("Solver initializes"),
		Solver.Initialize(Start, End, CableSimTests::MakeSettleConfig(248.0, 10.0)));
	const CableSim::FStepInput Input = CableSimTests::MakeFixedInput(Start, End);
	CableSim::FManifoldConfig ManifoldConfig;
	// The live Chaos snapshot concatenates triangles from several objects in an order
	// that varies frame to frame; reproduce that by shuffling the input order each step.
	int32 StepSeed = 0;
	const CableSim::FContactGenerator Generator =
		[&Triangles, &Edges, ManifoldConfig, &StepSeed](const TConstArrayView<CableSim::FParticle> Particles,
			TArray<CableSim::FContactConstraint>& Contacts)
	{
		TArray<CableSim::FCollisionTriangle> Shuffled = Triangles;
		FRandomStream Random(StepSeed);
		for (int32 I = Shuffled.Num() - 1; I > 0; --I)
		{
			Shuffled.Swap(I, Random.RandRange(0, I));
		}
		for (int32 Index = 0; Index < Particles.Num(); ++Index)
		{
			if (Particles[Index].Mode == CableSim::EParticleMode::Dynamic)
			{
				CableSim::FContactManifoldCompiler::CompileNodeContacts(
					Index, Particles[Index].Position, Particles[Index].PreviousPosition,
					Shuffled, Edges, ManifoldConfig, Contacts);
			}
		}
	};
	double WindowRms = 0.0;
	double WindowMax = 0.0;
	for (int32 Step = 0; Step < 900; ++Step)
	{
		StepSeed = Step;
		const CableSim::FStepResult Result = Solver.AdvanceStep(Input, Generator);
		if (Step >= 780)
		{
			WindowRms += Result.RmsParticleSpeed;
			WindowMax = FMath::Max(WindowMax, Result.MaximumParticleSpeed);
		}
	}
	WindowRms /= 120.0;
	AddInfo(FString::Printf(
		TEXT("Nearly-taut over bend (shuffled snapshot): window-avg RMS %.4f cm/s, window-max %.4f cm/s"),
		WindowRms, WindowMax));
	TestTrue(TEXT("Nearly-taut cable over a convex bend settles (RMS < 2 cm/s)"), WindowRms < 2.0);
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
