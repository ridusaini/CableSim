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

#endif
