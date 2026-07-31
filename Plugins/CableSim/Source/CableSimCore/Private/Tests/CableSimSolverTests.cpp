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
		Config.SegmentCount = 10;
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
	FCableSimLengthChangeTest,
	"CableSim.Core.Dynamic.LengthChangeIsExactAndInstant",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimLengthChangeTest::RunTest(const FString& Parameters)
{
	CableSim::FSolver Solver;
	// SegmentCount=10 from MakeConfig(): 11 particles, uniform 10cm material spacing.
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

	const int32 ParticleCountBefore = Solver.GetParticles().Num();
	const FVector3d ShapedPositionBefore = Solver.GetParticles()[5].Position;
	const FVector3d ShapedVelocityBefore = Solver.GetParticles()[5].Velocity;

	// Particle 5 of 11 (SegmentCount=10) starts at material coordinate 50.0.
	TestTrue(TEXT("Set a longer length"), Solver.SetActiveLength(125.0));
	TestTrue(TEXT("Active length applies exactly, with no rate limit"),
		FMath::IsNearlyEqual(Solver.GetActiveLength(), 125.0, 1.e-6));
	TestEqual(TEXT("Segment count is fixed: length changes never insert or remove particles"),
		Solver.GetParticles().Num(), ParticleCountBefore);
	TestTrue(TEXT("A length change only rescales material coordinates, not world shape"),
		Solver.GetParticles()[5].Position.Equals(ShapedPositionBefore, 1.e-9)
		&& Solver.GetParticles()[5].Velocity.Equals(ShapedVelocityBefore, 1.e-9));
	TestTrue(TEXT("Material coordinates rescale proportionally (50.0 * 125/100 = 62.5)"),
		FMath::IsNearlyEqual(Solver.GetParticles()[5].MaterialCoordinate, 62.5, 1.e-6));

	TestTrue(TEXT("Set a shorter length"), Solver.SetActiveLength(75.0));
	TestTrue(TEXT("Active length shrinks exactly"), FMath::IsNearlyEqual(Solver.GetActiveLength(), 75.0, 1.e-6));
	TestEqual(TEXT("Segment count remains fixed while reeling in"),
		Solver.GetParticles().Num(), ParticleCountBefore);
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
	Config.SegmentCount = 2;
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
	Config.SegmentCount = 1;
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
	Config.SegmentCount = 1;
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
	Config.SegmentCount = 300;
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
		Config.SegmentCount = 4;
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
	Config.SegmentCount = 300;
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
	Config.SegmentCount = 40;
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

// A long, nearly-inextensible cable pinned to both ends and draped over a
// sphere is the steady-state case that used to make SolveBatch's tail
// convergence loop pay its full worst case (Particles.Num()*4, capped at 512)
// every single frame once settled: distance and contact corrections never
// stop mildly disagreeing at the sphere, so the pre-multigrid tail needed
// ~410-420 flat Gauss-Seidel sweeps/step to hold strain under its 0.25%
// threshold. Re-running multigrid each tail iteration cut that to ~17-40
// while reaching the same strain. This guards the correctness side of that
// fix; there is no automation-friendly way to assert the iteration count
// itself without flaky wall-clock timing, so this pins convergence quality
// instead, which the old flat-sweep-only tail could not sustain within budget
// for a case this size.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimSustainedContactConflictTest,
	"CableSim.Core.Dynamic.SustainedContactConflictConverges",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimSustainedContactConflictTest::RunTest(const FString& Parameters)
{
	CableSim::FSimulationConfig Config = CableSimTests::MakeConfig();
	Config.Length = 1050.0; // generous slack; the cable can comfortably drape over a small sphere
	Config.SegmentCount = 105;
	Config.Gravity = FVector3d(0.0, 0.0, -980.665);
	Config.VelocityDamping = 0.01;
	Config.SolverIterations = 64;
	Config.MultigridIterations = 2;
	Config.MultigridMinimumParticles = 64;
	CableSim::FSolver Solver;
	TestTrue(TEXT("Initialize slack 10m cable"), Solver.Initialize(
		FVector3d::ZeroVector,
		FVector3d(1000.0, 0.0, 0.0),
		Config));
	CableSim::FStepInput Input = CableSimTests::FreeInput();
	Input.StartEndpoint.State = CableSim::EEndpointState::Fixed;
	Input.StartEndpoint.TargetPosition = FVector3d::ZeroVector;
	Input.EndEndpoint.State = CableSim::EEndpointState::Fixed;
	Input.EndEndpoint.TargetPosition = FVector3d(1000.0, 0.0, 0.0);
	const FVector3d SphereCentre(500.0, 0.0, -20.0);
	const double CombinedRadius = 22.5;
	CableSim::FStepResult Result;
	for (int32 Step = 0; Step < 150; ++Step)
	{
		TArray<CableSim::FContactConstraint> Contacts;
		for (int32 Index = 0; Index < Solver.GetParticles().Num(); ++Index)
		{
			const FVector3d Position = Solver.GetParticles()[Index].Position;
			const FVector3d Radial = Position - SphereCentre;
			if (Radial.Length() < CombinedRadius)
			{
				CableSim::FContactConstraint& Contact = Contacts.AddDefaulted_GetRef();
				Contact.ParticleA = Index;
				Contact.Normal = Radial.GetSafeNormal(1.0, FVector3d::UnitZ());
				Contact.MinimumNormalCoordinate = FVector3d::DotProduct(SphereCentre, Contact.Normal) + CombinedRadius;
				Contact.FrictionAnchor = Position;
			}
		}
		Result = Solver.AdvanceStep(Input, Contacts);
		TestEqual(TEXT("Every step stays numerically valid under sustained conflict"),
			Result.Status, CableSim::ESimulationStatus::Ready);
	}
	AddInfo(FString::Printf(
		TEXT("tail-conflict settle strain=%.6f penetration=%.6f"),
		Result.MaximumSegmentStrain, Result.MaximumPenetration));
	TestTrue(TEXT("Draped-over-contact strain settles near the tail loop's own convergence threshold"),
		Result.MaximumSegmentStrain < 0.01);
	TestTrue(TEXT("The sphere contact is fully resolved, not just tolerated"),
		Result.MaximumPenetration < 0.01);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimSegmentFrictionResistsSlideTest,
	"CableSim.Core.Dynamic.SegmentFrictionResistsSlide",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimSegmentFrictionResistsSlideTest::RunTest(const FString& Parameters)
{
	// Segment (barycentric) contacts used to hard-disable friction entirely, so a
	// cable resting on something narrower than the node spacing could slide
	// across it frictionlessly. ProjectSegmentFriction fixes that by splitting
	// the correction across both segment endpoints by barycentric weight. This
	// compares a dragged, resting segment with and without friction: the
	// contact point (segment 1-2's midpoint, held at alpha=0.5 so no particle
	// sits on it directly) must lag the frictionless case's lateral slide.
	auto BuildAndDrag = [](const double StaticFriction, const double DynamicFriction) -> CableSim::FSolver
	{
		CableSim::FSimulationConfig Config = CableSimTests::MakeConfig();
		Config.Length = 55.0; // slack relative to the 40cm anchor chord, so the drag doesn't just go taut
		Config.SegmentCount = 4; // 5 particles, same indexing as the taut case
		Config.Gravity = FVector3d(0.0, 0.0, -980.665);
		Config.VelocityDamping = 0.01;
		Config.StaticFriction = StaticFriction;
		Config.DynamicFriction = DynamicFriction;
		Config.StaticFrictionDeadZone = 0.05;
		CableSim::FSolver Solver;
		Solver.Initialize(FVector3d::ZeroVector, FVector3d(40.0, 0.0, 0.0), Config);
		CableSim::FStepInput Input = CableSimTests::FreeInput();
		Input.StartEndpoint.State = CableSim::EEndpointState::Fixed;
		Input.StartEndpoint.TargetPosition = FVector3d::ZeroVector;
		Input.EndEndpoint.State = CableSim::EEndpointState::Fixed;
		const double LateralSpeed = 20.0; // cm/s: slow, steady drag, not a yank
		for (int32 Step = 0; Step < 90; ++Step)
		{
			Input.EndEndpoint.TargetPosition = FVector3d(40.0, LateralSpeed * (Step + 1) * Input.DeltaTime, 0.0);
			CableSim::FContactConstraint Contact;
			Contact.ParticleA = 1;
			Contact.ParticleB = 2;
			Contact.SegmentAlpha = 0.5;
			Contact.Normal = FVector3d::UnitZ();
			Contact.MinimumNormalCoordinate = -5.0;
			Contact.bEnableFriction = true;
			// Matches production: the anchor is this step's starting contact point
			// (WorldCollisionProvider sets it from the surface's previous-step
			// position), so friction resists slip within the step, not cumulative
			// drift across the whole run.
			Contact.FrictionAnchor = 0.5 * (Solver.GetParticles()[1].Position + Solver.GetParticles()[2].Position);
			Solver.AdvanceStep(Input, MakeArrayView(&Contact, 1));
		}
		return Solver;
	};

	CableSim::FSolver Frictionless = BuildAndDrag(0.0, 0.0);
	CableSim::FSolver Frictional = BuildAndDrag(1.5, 1.0);

	const double FrictionlessSlideY = 0.5
		* (Frictionless.GetParticles()[1].Position.Y + Frictionless.GetParticles()[2].Position.Y);
	const double FrictionalSlideY = 0.5
		* (Frictional.GetParticles()[1].Position.Y + Frictional.GetParticles()[2].Position.Y);
	AddInfo(FString::Printf(
		TEXT("segment friction slide frictionless=%.4f frictional=%.4f status=%d/%d"),
		FrictionlessSlideY, FrictionalSlideY,
		static_cast<int32>(Frictionless.GetLastResult().Status),
		static_cast<int32>(Frictional.GetLastResult().Status)));
	TestEqual(TEXT("Frictionless run stays numerically valid"),
		Frictionless.GetLastResult().Status, CableSim::ESimulationStatus::Ready);
	TestEqual(TEXT("Frictional run stays numerically valid"),
		Frictional.GetLastResult().Status, CableSim::ESimulationStatus::Ready);
	TestTrue(TEXT("A resting contact still slides when undragged force overcomes it (sanity: the frictionless case actually moves)"),
		FrictionlessSlideY > 1.0);
	TestTrue(TEXT("Friction measurably resists the dragged segment contact's lateral slide"),
		FrictionalSlideY < FrictionlessSlideY * 0.9);
	return true;
}

#endif
