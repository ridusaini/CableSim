#if WITH_DEV_AUTOMATION_TESTS

#include "CableSimComponent.h"
#include "CableSimWorldCollisionProvider.h"
#include "Misc/AutomationTest.h"

namespace
{
	void AddTestBox(TArray<CableSim::FCollisionTriangle>& OutTriangles)
	{
		const FVector3d V[8] = {
			{-50.0, -50.0, -50.0}, {50.0, -50.0, -50.0}, {50.0, 50.0, -50.0}, {-50.0, 50.0, -50.0},
			{-50.0, -50.0, 50.0}, {50.0, -50.0, 50.0}, {50.0, 50.0, 50.0}, {-50.0, 50.0, 50.0}};
		const int32 Faces[12][3] = {
			{0,3,2}, {0,2,1}, {4,5,6}, {4,6,7}, {0,1,5}, {0,5,4},
			{1,2,6}, {1,6,5}, {2,3,7}, {2,7,6}, {3,0,4}, {3,4,7}};
		for (int32 Face = 0; Face < 12; ++Face)
		{
			CableSim::FCollisionTriangle& Triangle = OutTriangles.AddDefaulted_GetRef();
			Triangle.Id = {1, 0, CableSim::ECollisionFeatureType::Face, Face, INDEX_NONE};
			Triangle.GeometryType = CableSim::ECollisionGeometryType::Box;
			for (int32 Corner = 0; Corner < 3; ++Corner)
			{
				Triangle.VertexIndices[Corner] = Faces[Face][Corner];
				Triangle.Vertices[Corner] = V[Faces[Face][Corner]];
			}
		}
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimRuntimeSlackFallsTest,
	"CableSim.Runtime.Dynamic.SlackCableFalls",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimRuntimeSlackFallsTest::RunTest(const FString& Parameters)
{
	UCableSimComponent* Component = NewObject<UCableSimComponent>();
	Component->SimulationSettings.RestLength = 400.0;
	Component->SimulationSettings.SegmentCount = 40;
	Component->StartEndpoint.State = ECableSimEndpointState::Fixed;
	Component->EndEndpoint.State = ECableSimEndpointState::Fixed;
	Component->StartEndpoint.LocalTarget = FVector(-150.0, 0.0, 0.0);
	Component->EndEndpoint.LocalTarget = FVector(150.0, 0.0, 0.0);
	Component->ReinitializeSimulation();
	const TArray<FVector> Before = Component->GetCablePolyline();
	Component->StepSimulation(30);
	const TArray<FVector> After = Component->GetCablePolyline();
	double LowestZ = TNumericLimits<double>::Max();
	for (const FVector& Point : After) LowestZ = FMath::Min(LowestZ, Point.Z);
	const FCableSimStatus Status = Component->GetSimulationStatus();
	AddInfo(FString::Printf(
		TEXT("midpoint before=%.6f after=%.6f lowest=%.6f status=%d step=%lld strain=%.6f"),
		Before.IsValidIndex(Before.Num() / 2) ? Before[Before.Num() / 2].Z : 0.0,
		After.IsValidIndex(After.Num() / 2) ? After[After.Num() / 2].Z : 0.0,
		LowestZ,
		static_cast<int32>(Status.Status),
		Status.StepIndex,
		Status.MaximumSegmentStrain));
	TestEqual(TEXT("Particle count is stable"), After.Num(), Before.Num());
	TestTrue(TEXT("A dynamic node in the slack cable falls under gravity"), LowestZ < -1.e-4);
	TestTrue(TEXT("The runtime step was accepted"),
		Status.Status == ECableSimSimulationStatus::Ready
		|| Status.Status == ECableSimSimulationStatus::MovementLimited);
	TestTrue(TEXT("All requested dynamic steps advanced"), Status.StepIndex >= 30);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimRuntimeLengthControllerTest,
	"CableSim.Runtime.Dynamic.LengthAndSegmentCountApplyExactly",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimRuntimeLengthControllerTest::RunTest(const FString& Parameters)
{
	UCableSimComponent* Component = NewObject<UCableSimComponent>();
	Component->CollisionSettings.bEnableWorldCollision = false;
	Component->SimulationSettings.RestLength = 100.0;
	Component->SimulationSettings.SegmentCount = 10;
	Component->SimulationSettings.MaximumLength = 250.0;
	Component->StartEndpoint.State = ECableSimEndpointState::Fixed;
	Component->EndEndpoint.State = ECableSimEndpointState::Fixed;
	Component->StartEndpoint.LocalTarget = FVector(-40.0, 0.0, 0.0);
	Component->EndEndpoint.LocalTarget = FVector(40.0, 0.0, 0.0);
	Component->ReinitializeSimulation();
	TestTrue(TEXT("Initial active length is configured"),
		FMath::IsNearlyEqual(Component->GetCableLength(), 100.0, 1.e-6));
	TestEqual(TEXT("Initial segment count is configured"), Component->GetCableSegmentCount(), 10);

	const double AcceptedLength = Component->SetCableLength(500.0);
	TestTrue(TEXT("Length is clamped to the authored maximum"),
		FMath::IsNearlyEqual(AcceptedLength, 250.0, 1.e-6));
	Component->StepSimulation(1);
	TestTrue(TEXT("Length applies in full on the very next step: no gradual feed"),
		FMath::IsNearlyEqual(Component->GetCableLength(), 250.0, 1.e-4));
	TestEqual(TEXT("A length change never touches segment count"), Component->GetCableSegmentCount(), 10);
	TestTrue(TEXT("Clamping is visible to gameplay"), Component->GetSimulationStatus().bLengthClamped);

	const int32 AcceptedCount = Component->SetCableSegmentCount(20);
	TestEqual(TEXT("Segment count applies exactly"), AcceptedCount, 20);
	Component->StepSimulation(1);
	TestEqual(TEXT("Particle count reflects the new segment count immediately"),
		Component->GetCablePolyline().Num(), 21);
	TestTrue(TEXT("A segment count change never touches length"),
		FMath::IsNearlyEqual(Component->GetCableLength(), 250.0, 1.e-4));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimClosedBoxRecoveryTest,
	"CableSim.Runtime.Dynamic.ClosedBoxUsesNearestExit",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimClosedBoxRecoveryTest::RunTest(const FString& Parameters)
{
	CableSim::FSimulationConfig Config;
	Config.Length = 10.0;
	Config.SegmentCount = 1;
	Config.Gravity = FVector3d::ZeroVector;
	Config.BendStrength = 0.0;
	Config.StaticFriction = 0.0;
	Config.DynamicFriction = 0.0;
	CableSim::FSolver Solver;
	TestTrue(TEXT("Initialize inside box"), Solver.Initialize(
		FVector3d(40.0, 0.0, 0.0),
		FVector3d(30.0, 0.0, 0.0),
		Config));
	FCableSimCollisionSnapshot Snapshot;
	Snapshot.Nodes.SetNum(2);
	for (FCableSimNodeCollisionGeometry& Node : Snapshot.Nodes)
	{
		AddTestBox(Node.Triangles);
		CableSim::FCollisionTopologyCompiler::CompileEdges(Node.Triangles, 0.1, Node.Edges);
	}
	CableSim::FStepInput Input;
	Input.DeltaTime = 1.0 / 60.0;
	TestTrue(TEXT("Begin prediction"), Solver.BeginStep(Input));
	FCableSimCollisionSettings Collision;
	Collision.Radius = 2.5;
	Collision.SkinWidth = 0.0;
	Collision.MaximumSafeCorrection = 5.0;
	FCableSimFrictionSettings Friction;
	Friction.bEnableFriction = false;
	TArray<CableSim::FContactConstraint> Contacts;
	TArray<FVector3d> Rejected;
	FCableSimCollisionDiagnostics Diagnostics;
	FCableSimWorldCollisionProvider::CompileContacts(
		Snapshot,
		Collision,
		Friction,
		Solver,
		Input,
		Contacts,
		Rejected,
		Diagnostics);
	int32 StartContacts = 0;
	for (const CableSim::FContactConstraint& Contact : Contacts)
	{
		if (Contact.ParticleA == 0)
		{
			++StartContacts;
			TestTrue(TEXT("Nearest exit points through +X face"), Contact.Normal.X > 0.99);
		}
	}
	TestEqual(TEXT("One recovery plane is selected for the embedded node"), StartContacts, 1);
	Solver.SolveBatch(Input, Contacts, Config.SolverIterations);
	const CableSim::FStepResult Result = Solver.FinalizeStep(Input, Contacts);
	TestEqual(TEXT("Recovery remains numerically valid"), Result.Status, CableSim::ESimulationStatus::Ready);
	TestTrue(TEXT("Embedded node reaches the outside of the box"), Solver.GetParticles()[0].Position.X >= 52.4);
	TestTrue(TEXT("Recovery velocity does not launch away from the surface"),
		FMath::Abs(Solver.GetParticles()[0].Velocity.X) < 1.e-3);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimBoxEdgeContactTest,
	"CableSim.Runtime.Dynamic.BoxEdgeUsesConvexContact",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimBoxEdgeContactTest::RunTest(const FString& Parameters)
{
	CableSim::FSimulationConfig Config;
	Config.Length = 10.0;
	Config.SegmentCount = 1;
	Config.Gravity = FVector3d::ZeroVector;
	Config.BendStrength = 0.0;
	Config.StaticFriction = 0.0;
	Config.DynamicFriction = 0.0;
	CableSim::FSolver Solver;
	TestTrue(TEXT("Initialize beside convex edge"), Solver.Initialize(
		FVector3d(51.0, 51.0, -5.0),
		FVector3d(51.0, 51.0, 5.0),
		Config));
	FCableSimCollisionSnapshot Snapshot;
	Snapshot.Nodes.SetNum(2);
	for (FCableSimNodeCollisionGeometry& Node : Snapshot.Nodes)
	{
		AddTestBox(Node.Triangles);
		CableSim::FCollisionTopologyCompiler::CompileEdges(Node.Triangles, 0.1, Node.Edges);
	}
	CableSim::FStepInput Input;
	Input.DeltaTime = 1.0 / 60.0;
	Solver.BeginStep(Input);
	FCableSimCollisionSettings Collision;
	Collision.Radius = 2.5;
	Collision.SkinWidth = 0.0;
	FCableSimFrictionSettings Friction;
	Friction.bEnableFriction = false;
	TArray<CableSim::FContactConstraint> Contacts;
	TArray<FVector3d> Rejected;
	FCableSimCollisionDiagnostics Diagnostics;
	FCableSimWorldCollisionProvider::CompileContacts(
		Snapshot, Collision, Friction, Solver, Input, Contacts, Rejected, Diagnostics);
	int32 EdgeContacts = 0;
	for (const CableSim::FContactConstraint& Contact : Contacts)
	{
		if (Contact.ParticleA == 0 && Contact.ParticleB == INDEX_NONE
			&& Contact.FeatureId.Type == CableSim::ECollisionFeatureType::Edge)
		{
			++EdgeContacts;
			TestTrue(TEXT("Edge normal points out of the box corner"), Contact.Normal.X > 0.6 && Contact.Normal.Y > 0.6);
		}
	}
	TestEqual(TEXT("The node receives one convex-edge contact"), EdgeContacts, 1);
	Solver.SolveBatch(Input, Contacts, Config.SolverIterations);
	Solver.FinalizeStep(Input, Contacts);
	const FVector3d Point = Solver.GetParticles()[0].Position;
	const double EdgeDistance = FVector2d(Point.X - 50.0, Point.Y - 50.0).Length();
	TestTrue(TEXT("The finite-radius node is outside the rounded edge"), EdgeDistance >= 2.49);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimSegmentPrimitiveContactTest,
	"CableSim.Runtime.Dynamic.SegmentContactClosesParticleGap",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimSegmentPrimitiveContactTest::RunTest(const FString& Parameters)
{
	CableSim::FSimulationConfig Config;
	Config.Length = 100.0;
	Config.SegmentCount = 1;
	Config.Gravity = FVector3d::ZeroVector;
	Config.BendStrength = 0.0;
	Config.StaticFriction = 0.0;
	Config.DynamicFriction = 0.0;
	CableSim::FSolver Solver;
	TestTrue(TEXT("Initialize one long segment"), Solver.Initialize(
		FVector3d(-50.0, 0.0, 0.0),
		FVector3d(50.0, 0.0, 0.0),
		Config));
	FCableSimCollisionSnapshot Snapshot;
	Snapshot.Nodes.SetNum(2);
	FCableSimCollisionSphere& Sphere = Snapshot.Nodes[0].Spheres.AddDefaulted_GetRef();
	Sphere.Id = {1, 0, CableSim::ECollisionFeatureType::Face, 0, INDEX_NONE};
	Sphere.Centre = FVector3d::ZeroVector;
	Sphere.Radius = 5.0;
	CableSim::FStepInput Input;
	Input.DeltaTime = 1.0 / 60.0;
	TestTrue(TEXT("Begin prediction"), Solver.BeginStep(Input));
	FCableSimCollisionSettings Collision;
	Collision.Radius = 2.5;
	Collision.SkinWidth = 0.0;
	FCableSimFrictionSettings Friction;
	Friction.bEnableFriction = false;
	TArray<CableSim::FContactConstraint> Contacts;
	TArray<FVector3d> Rejected;
	FCableSimCollisionDiagnostics Diagnostics;
	FCableSimWorldCollisionProvider::CompileContacts(
		Snapshot, Collision, Friction, Solver, Input, Contacts, Rejected, Diagnostics);
	const CableSim::FContactConstraint* SegmentContact = Contacts.FindByPredicate([](const CableSim::FContactConstraint& Contact)
	{
		return Contact.ParticleA == 0 && Contact.ParticleB == 1;
	});
	TestNotNull(TEXT("A barycentric contact covers the unsampled segment interior"), SegmentContact);
	if (SegmentContact)
	{
		TestTrue(TEXT("The contact lies at the segment interior"),
			SegmentContact->SegmentAlpha > 0.49 && SegmentContact->SegmentAlpha < 0.51);
	}
	Solver.SolveBatch(Input, Contacts, Config.SolverIterations);
	const CableSim::FStepResult Result = Solver.FinalizeStep(Input, Contacts);
	TestEqual(TEXT("Segment solve remains valid"), Result.Status, CableSim::ESimulationStatus::Ready);
	const FVector3d Midpoint = 0.5 * (Solver.GetParticles()[0].Position + Solver.GetParticles()[1].Position);
	TestTrue(TEXT("The cable centre is separated from the primitive"), Midpoint.Z >= 7.49);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimDrivenEndpointUnreachablePlaneTest,
	"CableSim.Runtime.Dynamic.DrivenEndpointDiscardsUnreachablePlane",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimDrivenEndpointUnreachablePlaneTest::RunTest(const FString& Parameters)
{
	CableSim::FSimulationConfig Config;
	Config.Length = 10.0;
	Config.SegmentCount = 1;
	Config.Gravity = FVector3d::ZeroVector;
	Config.BendStrength = 0.0;
	Config.StaticFriction = 0.0;
	Config.DynamicFriction = 0.0;
	CableSim::FSolver Solver;
	TestTrue(TEXT("Initialize beside convex edge"), Solver.Initialize(
		FVector3d(51.0, 51.0, -5.0),
		FVector3d(51.0, 51.0, 5.0),
		Config));
	FCableSimCollisionSnapshot Snapshot;
	Snapshot.Nodes.SetNum(2);
	for (FCableSimNodeCollisionGeometry& Node : Snapshot.Nodes)
	{
		AddTestBox(Node.Triangles);
		CableSim::FCollisionTopologyCompiler::CompileEdges(Node.Triangles, 0.1, Node.Edges);
	}
	CableSim::FStepInput Input;
	Input.DeltaTime = 1.0 / 60.0;
	Input.EndEndpoint.State = CableSim::EEndpointState::Driven;
	Input.EndEndpoint.TargetPosition = FVector3d(5051.0, 5051.0, 5.0);
	Solver.BeginStep(Input);
	FCableSimCollisionSettings Collision;
	Collision.Radius = 2.5;
	Collision.SkinWidth = 0.0;
	FCableSimFrictionSettings Friction;
	Friction.bEnableFriction = false;
	TArray<CableSim::FContactConstraint> Contacts;
	TArray<FVector3d> Rejected;
	FCableSimCollisionDiagnostics Diagnostics;
	FCableSimWorldCollisionProvider::CompileContacts(
		Snapshot, Collision, Friction, Solver, Input, Contacts, Rejected, Diagnostics);
	TestEqual(TEXT("The box corner is unreachable from the far Driven target and is discarded"), Contacts.Num(), 0);
	TestTrue(TEXT("The unreachable plane is recorded as rejected, not silently dropped"), Rejected.Num() > 0);
	return true;
}

#endif
