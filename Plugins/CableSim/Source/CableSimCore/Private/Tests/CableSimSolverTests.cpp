#if WITH_DEV_AUTOMATION_TESTS

#include "CableSimSolver.h"
#include "Misc/AutomationTest.h"

#include <limits>

namespace CableSimTests
{
	CableSim::FSimulationConfig MakeConfig()
	{
		CableSim::FSimulationConfig Config;
		Config.Length = 100.0;
		Config.SegmentLength = 10.0;
		Config.MaximumParticles = 128;
		Config.LinearDensity = 0.01;
		Config.Gravity = FVector3d::ZeroVector;
		Config.VelocityDamping = 0.0;
		Config.DistanceCompliance = 0.0;
		Config.DistanceRelaxation = 1.015;
		Config.BendStrength = 0.0;
		Config.FreeBendDegreesPerMeter = 0.0;
		Config.SolverIterations = 48;
		Config.StaticFriction = 0.0;
		Config.DynamicFriction = 0.0;
		return Config;
	}

	CableSim::FStepInput FreeInput()
	{
		CableSim::FStepInput Input;
		Input.DeltaTime = 1.0 / 60.0;
		return Input;
	}

	FVector3d CentreOfMass(const TConstArrayView<CableSim::FParticle> Particles)
	{
		FVector3d Result = FVector3d::ZeroVector;
		double TotalMass = 0.0;
		for (const CableSim::FParticle& Particle : Particles)
		{
			Result += Particle.Position * Particle.Mass;
			TotalMass += Particle.Mass;
		}
		return TotalMass > UE_DOUBLE_SMALL_NUMBER ? Result / TotalMass : FVector3d::ZeroVector;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimUniformInitializationTest,
	"CableSim.Core.Dynamic.UniformInitializationAndLength",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimUniformInitializationTest::RunTest(const FString& Parameters)
{
	CableSim::FSolver Solver;
	TestTrue(TEXT("Initialize"), Solver.Initialize(
		FVector3d::ZeroVector,
		FVector3d(100.0, 0.0, 0.0),
		CableSimTests::MakeConfig()));
	TestEqual(TEXT("Uniform particle count"), Solver.GetParticles().Num(), 11);
	for (int32 Index = 0; Index + 1 < Solver.GetParticles().Num(); ++Index)
	{
		TestTrue(TEXT("Uniform material spacing"), FMath::IsNearlyEqual(
			Solver.GetParticles()[Index + 1].MaterialCoordinate - Solver.GetParticles()[Index].MaterialCoordinate,
			10.0,
			1.e-9));
		TestTrue(TEXT("Initial shape is the exact endpoint chord"),
			Solver.GetParticles()[Index].Position.Equals(FVector3d(Index * 10.0, 0.0, 0.0), 1.e-9));
	}
	double TotalMass = 0.0;
	for (const CableSim::FParticle& Particle : Solver.GetParticles()) TotalMass += Particle.Mass;
	TestTrue(TEXT("Lumped masses integrate the configured linear density"),
		FMath::IsNearlyEqual(TotalMass, 1.0, 1.e-9));
	const CableSim::FStepResult Result = Solver.AdvanceStep(CableSimTests::FreeInput());
	TestEqual(TEXT("Ready"), Result.Status, CableSim::ESimulationStatus::Ready);
	TestTrue(TEXT("Length error is bounded"), Result.MaximumSegmentError < 1.e-6);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimEndpointLengthFlowTest,
	"CableSim.Core.Dynamic.EndpointLengthFlowPreservesState",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimEndpointLengthFlowTest::RunTest(const FString& Parameters)
{
	CableSim::FSolver Solver;
	CableSim::FSimulationConfig Config = CableSimTests::MakeConfig();
	TestTrue(TEXT("Initialize"), Solver.Initialize(
		FVector3d::ZeroVector,
		FVector3d(80.0, 0.0, 0.0),
		Config));
	CableSim::FStateSnapshot Shaped = Solver.CaptureState();
	Shaped.Particles[5].Position.Y = 20.0;
	Shaped.Particles[5].PreviousPosition.Y = 18.0;
	Shaped.Particles[5].Velocity.Y = 120.0;
	TestTrue(TEXT("Restore shaped state"), Solver.RestoreState(Shaped));

	const FVector3d StartBefore = Solver.GetParticles()[0].Position;
	const FVector3d EndBefore = Solver.GetParticles().Last().Position;
	TestTrue(TEXT("Pay out at the end"), Solver.SetActiveLength(125.0, CableSim::ELengthChangeOrigin::End));
	TestTrue(TEXT("Absolute active length grows"), FMath::IsNearlyEqual(Solver.GetActiveLength(), 125.0, 1.e-6));
	TestTrue(TEXT("Start endpoint state is preserved"), Solver.GetParticles()[0].Position.Equals(StartBefore, 1.e-9));
	TestTrue(TEXT("End endpoint state is preserved"), Solver.GetParticles().Last().Position.Equals(EndBefore, 1.e-9));
	TestTrue(TEXT("Local payout inserts only the required samples"), Solver.GetParticles().Num() > Shaped.Particles.Num());
	TestTrue(TEXT("The shaped material survives local payout"), Solver.SamplePosition(50.0).Y > 19.9);

	TestTrue(TEXT("Reel from the start"), Solver.SetActiveLength(75.0, CableSim::ELengthChangeOrigin::Start));
	TestTrue(TEXT("Absolute active length shrinks"), FMath::IsNearlyEqual(Solver.GetActiveLength(), 75.0, 1.e-6));
	TestTrue(TEXT("Both world endpoints remain continuous while reeling"),
		Solver.GetParticles()[0].Position.Equals(StartBefore, 1.e-9)
		&& Solver.GetParticles().Last().Position.Equals(EndBefore, 1.e-9));
	double TotalMass = 0.0;
	for (const CableSim::FParticle& Particle : Solver.GetParticles()) TotalMass += Particle.Mass;
	TestTrue(TEXT("Mass follows the current material length"),
		FMath::IsNearlyEqual(TotalMass, Config.LinearDensity * 75.0, 1.e-6));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimMomentumConservingBendTest,
	"CableSim.Core.Dynamic.MomentumConservingBend",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimMomentumConservingBendTest::RunTest(const FString& Parameters)
{
	CableSim::FSimulationConfig Config = CableSimTests::MakeConfig();
	Config.Length = 20.0;
	Config.SegmentLength = 10.0;
	Config.BendStrength = 1.0;
	CableSim::FSolver Solver;
	Solver.Initialize(FVector3d(-10.0, 0.0, 0.0), FVector3d(10.0, 0.0, 0.0), Config);
	CableSim::FStateSnapshot Bent = Solver.CaptureState();
	Bent.Particles[1].Position = FVector3d(0.0, 5.0, 0.0);
	Bent.Particles[1].PreviousPosition = Bent.Particles[1].Position;
	Solver.RestoreState(Bent);
	const FVector3d BeforeCentre = CableSimTests::CentreOfMass(Solver.GetParticles());
	const CableSim::FStepResult Result = Solver.AdvanceStep(CableSimTests::FreeInput());
	const FVector3d AfterCentre = CableSimTests::CentreOfMass(Solver.GetParticles());
	TestEqual(TEXT("Ready"), Result.Status, CableSim::ESimulationStatus::Ready);
	TestTrue(TEXT("Bend reduces the middle-point deviation"), FMath::Abs(Solver.GetParticles()[1].Position.Y) < 5.0);
	TestTrue(TEXT("Internal bend and length projections preserve centre of mass"), BeforeCentre.Equals(AfterCentre, 1.e-8));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimBarycentricContactTest,
	"CableSim.Core.Dynamic.BarycentricSegmentContact",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimBarycentricContactTest::RunTest(const FString& Parameters)
{
	CableSim::FSimulationConfig Config = CableSimTests::MakeConfig();
	Config.SegmentLength = 100.0;
	CableSim::FSolver Solver;
	Solver.Initialize(FVector3d(0.0, 0.0, 0.0), FVector3d(100.0, 0.0, 0.0), Config);
	CableSim::FContactConstraint Contact;
	Contact.ParticleA = 0;
	Contact.ParticleB = 1;
	Contact.SegmentAlpha = 0.5;
	Contact.Normal = FVector3d::UnitZ();
	Contact.MinimumNormalCoordinate = 10.0;
	const CableSim::FStepResult Result = Solver.AdvanceStep(
		CableSimTests::FreeInput(),
		MakeArrayView(&Contact, 1));
	const FVector3d Midpoint = 0.5 * (Solver.GetParticles()[0].Position + Solver.GetParticles()[1].Position);
	TestTrue(TEXT("Segment centre is projected out"), Midpoint.Z >= 9.99);
	TestEqual(TEXT("One contact is reported"), Result.ContactCount, 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimRestingPlaneTest,
	"CableSim.Core.Dynamic.RestingPlaneSettles",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimRestingPlaneTest::RunTest(const FString& Parameters)
{
	CableSim::FSimulationConfig Config = CableSimTests::MakeConfig();
	Config.Length = 10.0;
	Config.SegmentLength = 10.0;
	Config.Gravity = FVector3d(0.0, 0.0, -980.665);
	Config.VelocityDamping = 0.01;
	CableSim::FSolver Solver;
	Solver.Initialize(FVector3d(0.0, 0.0, 1.0), FVector3d(10.0, 0.0, 1.0), Config);
	CableSim::FStepResult Result;
	for (int32 Step = 0; Step < 60; ++Step)
	{
		CableSim::FContactConstraint Contacts[2];
		for (int32 Index = 0; Index < 2; ++Index)
		{
			Contacts[Index].ParticleA = Index;
			Contacts[Index].Normal = FVector3d::UnitZ();
			Contacts[Index].MinimumNormalCoordinate = 1.0;
			Contacts[Index].FrictionAnchor = Solver.GetParticles()[Index].Position;
		}
		Result = Solver.AdvanceStep(CableSimTests::FreeInput(), MakeArrayView(Contacts));
	}
	TestEqual(TEXT("Ready"), Result.Status, CableSim::ESimulationStatus::Ready);
	TestTrue(TEXT("Cable remains above the plane"), Solver.GetParticles()[0].Position.Z >= 0.999);
	TestTrue(TEXT("Closing normal velocity is removed"), FMath::Abs(Solver.GetParticles()[0].Velocity.Z) < 1.e-6);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimInvalidInputRollbackTest,
	"CableSim.Core.Dynamic.InvalidInputPreservesState",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimInvalidInputRollbackTest::RunTest(const FString& Parameters)
{
	CableSim::FSolver Solver;
	Solver.Initialize(FVector3d::ZeroVector, FVector3d(100.0, 0.0, 0.0), CableSimTests::MakeConfig());
	const CableSim::FStateSnapshot Before = Solver.CaptureState();
	CableSim::FStepInput Input = CableSimTests::FreeInput();
	Input.StartEndpoint.TargetPosition.X = std::numeric_limits<double>::quiet_NaN();
	const CableSim::FStepResult Result = Solver.AdvanceStep(Input);
	TestEqual(TEXT("Invalid input is visible"), Result.Status, CableSim::ESimulationStatus::InvalidConfiguration);
	const CableSim::FStateSnapshot After = Solver.CaptureState();
	TestEqual(TEXT("Step index unchanged"), After.StepIndex, Before.StepIndex);
	for (int32 Index = 0; Index < Before.Particles.Num(); ++Index)
	{
		TestTrue(TEXT("Position unchanged"), After.Particles[Index].Position.Equals(Before.Particles[Index].Position, 0.0));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimDrivenEndpointLimitTest,
	"CableSim.Core.Dynamic.DrivenEndpointChasesWithinLength",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimDrivenEndpointLimitTest::RunTest(const FString& Parameters)
{
	CableSim::FSimulationConfig Config = CableSimTests::MakeConfig();
	CableSim::FSolver Solver;
	TestTrue(TEXT("Initialize slack cable"), Solver.Initialize(
		FVector3d::ZeroVector,
		FVector3d(50.0, 0.0, 0.0),
		Config));
	CableSim::FStepInput Input = CableSimTests::FreeInput();
	Input.StartEndpoint.State = CableSim::EEndpointState::Fixed;
	Input.StartEndpoint.TargetPosition = FVector3d::ZeroVector;
	Input.EndEndpoint.State = CableSim::EEndpointState::Driven;
	Input.EndEndpoint.TargetPosition = FVector3d(120.0, 0.0, 0.0);
	const CableSim::FStepResult Result = Solver.AdvanceStep(Input);
	TestEqual(TEXT("Ready"), Result.Status, CableSim::ESimulationStatus::Ready);
	TestTrue(TEXT("Driven endpoint moves toward the request"), Result.EndEndpoint.AcceptedPosition.X > 50.0);
	TestTrue(TEXT("Driven endpoint remains inside cable reach"),
		FVector3d::Distance(Result.StartEndpoint.AcceptedPosition, Result.EndEndpoint.AcceptedPosition) <= Config.Length + 0.5);
	TestTrue(TEXT("Limit is reported to gameplay"), Result.EndEndpoint.bLimited);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimLongCableMultigridTest,
	"CableSim.Core.Dynamic.LongCableMultigridConverges",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimLongCableMultigridTest::RunTest(const FString& Parameters)
{
	CableSim::FSimulationConfig Config = CableSimTests::MakeConfig();
	Config.Length = 3000.0;
	Config.SegmentLength = 10.0;
	Config.MaximumParticles = 512;
	Config.MultigridIterations = 2;
	Config.MultigridMinimumParticles = 64;
	Config.SolverIterations = 64;
	CableSim::FSolver Solver;
	TestTrue(TEXT("Initialize 30 m cable"), Solver.Initialize(
		FVector3d::ZeroVector,
		FVector3d(2500.0, 0.0, 0.0),
		Config));
	CableSim::FStepInput Input = CableSimTests::FreeInput();
	Input.StartEndpoint.State = CableSim::EEndpointState::Fixed;
	Input.StartEndpoint.TargetPosition = FVector3d::ZeroVector;
	Input.EndEndpoint.State = CableSim::EEndpointState::Driven;
	Input.EndEndpoint.TargetPosition = FVector3d(3200.0, 0.0, 0.0);
	CableSim::FStepResult Result;
	double MaximumObservedStrain = 0.0;
	int32 MaximumStrainStep = INDEX_NONE;
	for (int32 Step = 0; Step < 180; ++Step)
	{
		Result = Solver.AdvanceStep(Input);
		if (Result.MaximumSegmentStrain > MaximumObservedStrain)
		{
			MaximumObservedStrain = Result.MaximumSegmentStrain;
			MaximumStrainStep = Step;
		}
	}
	AddInfo(FString::Printf(
		TEXT("30m maxStrain=%.6f maxStep=%d finalStrain=%.6f segmentError=%.6f endpointX=%.6f limit=%.6f"),
		MaximumObservedStrain,
		MaximumStrainStep,
		Result.MaximumSegmentStrain,
		Result.MaximumSegmentError,
		Result.EndEndpoint.AcceptedPosition.X,
		Result.EndEndpoint.LimitError));
	TestEqual(TEXT("Long solve remains valid"), Result.Status, CableSim::ESimulationStatus::Ready);
	TestTrue(TEXT("Straight-chord unfolding stays below two percent tensile strain"), MaximumObservedStrain < 0.02);
	TestTrue(TEXT("Settled long-cable tensile strain stays below one percent"),
		Result.MaximumSegmentStrain < 0.01);
	TestTrue(TEXT("Driven endpoint remains within rest-length reach"),
		FVector3d::Distance(Result.StartEndpoint.AcceptedPosition, Result.EndEndpoint.AcceptedPosition)
			<= Config.Length * 1.005);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimCapstanFrictionDepletesTensionTest,
	"CableSim.Core.Dynamic.CapstanFrictionDepletesTensionAcrossWrap",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimCapstanFrictionDepletesTensionTest::RunTest(const FString& Parameters)
{
	// A rope fixed at particle 0, hanging free at particle 4, folded roughly 90
	// degrees at particle 2. Particle 2 is tagged as an edge contact so
	// UpdateContactLoads treats it as a wrap rather than an unsupported bend.
	auto BuildFoldedSolver = [](double StaticFriction, CableSim::FStepResult& OutResult) -> CableSim::FSolver
	{
		CableSim::FSimulationConfig Config = CableSimTests::MakeConfig();
		Config.Length = 40.0;
		Config.SegmentLength = 10.0;
		Config.Gravity = FVector3d(0.0, 0.0, -980.665);
		Config.StaticFriction = StaticFriction;
		CableSim::FSolver Solver;
		Solver.Initialize(FVector3d::ZeroVector, FVector3d(40.0, 0.0, 0.0), Config);
		CableSim::FStateSnapshot Folded = Solver.CaptureState();
		Folded.Particles[3].Position = FVector3d(20.0, 0.0, -10.0);
		Folded.Particles[3].PreviousPosition = Folded.Particles[3].Position;
		Folded.Particles[4].Position = FVector3d(20.0, 0.0, -20.0);
		Folded.Particles[4].PreviousPosition = Folded.Particles[4].Position;
		Solver.RestoreState(Folded);

		CableSim::FStepInput Input = CableSimTests::FreeInput();
		Input.StartEndpoint.State = CableSim::EEndpointState::Fixed;
		Input.StartEndpoint.TargetPosition = FVector3d::ZeroVector;
		CableSim::FContactConstraint Contact;
		Contact.ParticleA = 2;
		Contact.FeatureId.Type = CableSim::ECollisionFeatureType::Edge;
		Contact.Normal = FVector3d::UnitX();
		Contact.MinimumNormalCoordinate = Solver.GetParticles()[2].Position.X + 1.0;
		OutResult = Solver.AdvanceStep(Input, MakeArrayView(&Contact, 1));
		return Solver;
	};

	CableSim::FStepResult BaselineResult;
	CableSim::FSolver Baseline = BuildFoldedSolver(0.0, BaselineResult);
	CableSim::FStepResult DepletedResult;
	CableSim::FSolver Depleted = BuildFoldedSolver(0.5, DepletedResult);

	TestEqual(TEXT("Baseline step is valid"), BaselineResult.Status, CableSim::ESimulationStatus::Ready);
	TestEqual(TEXT("Depleted step is valid"), DepletedResult.Status, CableSim::ESimulationStatus::Ready);

	const double BaselineWrapTension = Baseline.GetParticles()[2].EstimatedTension;
	const double DepletedWrapTension = Depleted.GetParticles()[2].EstimatedTension;
	AddInfo(FString::Printf(
		TEXT("wrap tension baseline=%.6f depleted=%.6f anchor baseline=%.6f depleted=%.6f"),
		BaselineWrapTension, DepletedWrapTension,
		Baseline.GetParticles()[0].EstimatedTension, Depleted.GetParticles()[0].EstimatedTension));

	TestTrue(TEXT("A wrap with friction bleeds tension relative to the frictionless baseline"),
		DepletedWrapTension < BaselineWrapTension - 1.e-6);
	TestTrue(TEXT("A supported wrap still carries some tension, unlike an unsupported bend"),
		DepletedWrapTension > 1.e-6);
	TestTrue(TEXT("Tension loss at the wrap propagates further upstream toward the anchor"),
		Depleted.GetParticles()[0].EstimatedTension < Baseline.GetParticles()[0].EstimatedTension - 1.e-6);
	TestTrue(TEXT("Tension on the free side of the wrap is unaffected by depletion at the wrap"),
		FMath::IsNearlyEqual(
			Baseline.GetParticles()[4].EstimatedTension, Depleted.GetParticles()[4].EstimatedTension, 1.e-9));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimSuddenEndpointJumpTest,
	"CableSim.Core.Dynamic.SuddenEndpointJumpSettles",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimSuddenEndpointJumpTest::RunTest(const FString& Parameters)
{
	CableSim::FSimulationConfig Config = CableSimTests::MakeConfig();
	Config.Length = 3000.0;
	Config.SegmentLength = 10.0;
	Config.MaximumParticles = 512;
	Config.MultigridIterations = 2;
	Config.MultigridMinimumParticles = 64;
	Config.SolverIterations = 64;
	CableSim::FSolver Solver;
	TestTrue(TEXT("Initialize 30 m cable"), Solver.Initialize(
		FVector3d::ZeroVector,
		FVector3d(2500.0, 0.0, 0.0),
		Config));
	CableSim::FStepInput Input = CableSimTests::FreeInput();
	Input.StartEndpoint.State = CableSim::EEndpointState::Fixed;
	Input.StartEndpoint.TargetPosition = FVector3d::ZeroVector;
	Input.EndEndpoint.State = CableSim::EEndpointState::Fixed;
	Input.EndEndpoint.TargetPosition = FVector3d(2500.0, 0.0, 0.0);
	for (int32 Step = 0; Step < 60; ++Step)
	{
		Solver.AdvanceStep(Input);
	}
	// A hard, instantaneous yank: the Fixed endpoint jumps a large distance in
	// a single step with no per-step speed limit, the way a teleported or
	// hard-dragged attachment would. Without a motion pre-seed, only the
	// iterative distance/bend/multigrid solve is responsible for resolving it.
	Input.EndEndpoint.TargetPosition = FVector3d(2500.0, 300.0, 200.0);
	const CableSim::FStepResult JumpResult = Solver.AdvanceStep(Input);
	TestEqual(TEXT("Jump step remains valid"), JumpResult.Status, CableSim::ESimulationStatus::Ready);
	CableSim::FStepResult Result;
	for (int32 Step = 0; Step < 120; ++Step)
	{
		Result = Solver.AdvanceStep(Input);
	}
	AddInfo(FString::Printf(
		TEXT("post-jump settle strain=%.6f penetration=%.6f endpoint=(%.3f %.3f %.3f)"),
		Result.MaximumSegmentStrain,
		Result.MaximumPenetration,
		Result.EndEndpoint.AcceptedPosition.X,
		Result.EndEndpoint.AcceptedPosition.Y,
		Result.EndEndpoint.AcceptedPosition.Z));
	TestEqual(TEXT("Settled after jump remains valid"), Result.Status, CableSim::ESimulationStatus::Ready);
	TestTrue(TEXT("Cable settles to low strain after a hard jump"), Result.MaximumSegmentStrain < 0.01);
	TestTrue(TEXT("Fixed endpoint reaches the jumped target exactly"),
		FVector3d::Distance(Result.EndEndpoint.AcceptedPosition, Input.EndEndpoint.TargetPosition) < 0.01);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimSustainedDragTest,
	"CableSim.Core.Dynamic.SustainedDragStaysSmooth",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimSustainedDragTest::RunTest(const FString& Parameters)
{
	CableSim::FSimulationConfig Config = CableSimTests::MakeConfig();
	Config.Length = 400.0;
	Config.SegmentLength = 10.0;
	Config.MaximumParticles = 64;
	CableSim::FSolver Solver;
	const double Radius = 350.0;
	TestTrue(TEXT("Initialize 4 m cable with 50cm slack"), Solver.Initialize(
		FVector3d::ZeroVector,
		FVector3d(Radius, 0.0, 0.0),
		Config));
	CableSim::FStepInput Input = CableSimTests::FreeInput();
	Input.StartEndpoint.State = CableSim::EEndpointState::Fixed;
	Input.StartEndpoint.TargetPosition = FVector3d::ZeroVector;
	Input.EndEndpoint.State = CableSim::EEndpointState::Fixed;
	// A realistic continuous drag: swing the far end around the anchor at a
	// constant 350cm radius (within the 400cm rest length, so the cable never
	// has to stretch), with ~700 cm/s (sprint speed) tangential velocity.
	const double AngularSpeed = 700.0 / Radius;
	double MaximumObservedStrain = 0.0;
	CableSim::FStepResult Result;
	for (int32 Step = 0; Step < 60; ++Step)
	{
		const double Angle = AngularSpeed * (Step + 1) * Input.DeltaTime;
		Input.EndEndpoint.TargetPosition = FVector3d(Radius * FMath::Cos(Angle), Radius * FMath::Sin(Angle), 0.0);
		Result = Solver.AdvanceStep(Input);
		MaximumObservedStrain = FMath::Max(MaximumObservedStrain, Result.MaximumSegmentStrain);
	}
	AddInfo(FString::Printf(
		TEXT("sustained drag maxStrain=%.6f finalStrain=%.6f endpoint=(%.3f %.3f %.3f)"),
		MaximumObservedStrain,
		Result.MaximumSegmentStrain,
		Result.EndEndpoint.AcceptedPosition.X,
		Result.EndEndpoint.AcceptedPosition.Y,
		Result.EndEndpoint.AcceptedPosition.Z));
	TestEqual(TEXT("Sustained drag remains valid throughout"), Result.Status, CableSim::ESimulationStatus::Ready);
	TestTrue(TEXT("Realistic continuous drag never approaches a large single-step strain spike"),
		MaximumObservedStrain < 0.05);
	TestTrue(TEXT("Dragged endpoint tracks its target exactly"),
		FVector3d::Distance(Result.EndEndpoint.AcceptedPosition, Input.EndEndpoint.TargetPosition) < 0.01);
	return true;
}

#endif
