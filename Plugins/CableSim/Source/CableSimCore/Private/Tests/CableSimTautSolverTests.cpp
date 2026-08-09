#if WITH_DEV_AUTOMATION_TESTS

#include "CableSimCollisionGeometry.h"
#include "CableSimTautSolver.h"
#include "Algo/Reverse.h"
#include "Misc/AutomationTest.h"

namespace CableSimTautTests
{
	CableSim::FCollisionTriangle MakeTriangle(
		const int32 TriangleIndex,
		const int32 Vertex0,
		const int32 Vertex1,
		const int32 Vertex2,
		const FVector3d& Position0,
		const FVector3d& Position1,
		const FVector3d& Position2)
	{
		CableSim::FCollisionTriangle Triangle;
		Triangle.Id = {1, 0, CableSim::ECollisionFeatureType::Triangle, TriangleIndex, INDEX_NONE};
		Triangle.GeometryType = CableSim::ECollisionGeometryType::Box;
		Triangle.bStaticObject = true;
		Triangle.VertexIndices[0] = Vertex0;
		Triangle.VertexIndices[1] = Vertex1;
		Triangle.VertexIndices[2] = Vertex2;
		Triangle.Vertices[0] = Position0;
		Triangle.Vertices[1] = Position1;
		Triangle.Vertices[2] = Position2;
		return Triangle;
	}

	TArray<CableSim::FCollisionTriangle> MakeBoxTriangles()
	{
		const FVector3d Vertices[8] = {
			{-1.0, -1.0, -1.0}, {1.0, -1.0, -1.0},
			{1.0, 1.0, -1.0}, {-1.0, 1.0, -1.0},
			{-1.0, -1.0, 1.0}, {1.0, -1.0, 1.0},
			{1.0, 1.0, 1.0}, {-1.0, 1.0, 1.0}};
		const int32 Indices[12][3] = {
			{0, 3, 2}, {0, 2, 1}, {4, 5, 6}, {4, 6, 7},
			{0, 1, 5}, {0, 5, 4}, {1, 2, 6}, {1, 6, 5},
			{2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7}};
		TArray<CableSim::FCollisionTriangle> Triangles;
		for (int32 Index = 0; Index < 12; ++Index)
		{
			Triangles.Add(MakeTriangle(
				Index,
				Indices[Index][0], Indices[Index][1], Indices[Index][2],
				Vertices[Indices[Index][0]],
				Vertices[Indices[Index][1]],
				Vertices[Indices[Index][2]]));
		}
		return Triangles;
	}

	struct FOwnedScene
	{
		TArray<CableSim::FCollisionTriangle> Triangles;
		TArray<CableSim::FCollisionEdge> Edges;
		TArray<CableSim::FCollisionVertex> Vertices;

		CableSim::FTautCollisionScene View() const
		{
			return {Triangles, Edges, Vertices};
		}
	};

	FOwnedScene MakeBoxScene()
	{
		FOwnedScene Scene;
		Scene.Triangles = MakeBoxTriangles();
		CableSim::FCollisionTopologyCompiler::CompileTopology(
			Scene.Triangles, 1.e-6, 8, Scene.Edges, Scene.Vertices);
		return Scene;
	}

	CableSim::FCollisionEdge MakeVerticalEdge()
	{
		CableSim::FCollisionEdge Edge;
		Edge.Id = {7, 2, CableSim::ECollisionFeatureType::Edge, 10, 11};
		Edge.GeometryType = CableSim::ECollisionGeometryType::Box;
		Edge.bStaticObject = true;
		Edge.Start = FVector3d(0.0, 0.0, -100.0);
		Edge.End = FVector3d(0.0, 0.0, 100.0);
		Edge.FaceNormal0 = FVector3d::UnitX();
		Edge.FaceNormal1 = FVector3d::UnitY();
		Edge.Kind = CableSim::ECollisionEdgeKind::Convex;
		return Edge;
	}

	FOwnedScene MakeVerticalEdgeScene()
	{
		FOwnedScene Scene;
		Scene.Edges.Add(MakeVerticalEdge());
		for (const bool bStart : {true, false})
		{
			CableSim::FCollisionVertex Vertex;
			Vertex.Id = {
				Scene.Edges[0].Id.ObjectToken,
				Scene.Edges[0].Id.ShapeIndex,
				CableSim::ECollisionFeatureType::Vertex,
				bStart ? Scene.Edges[0].Id.Index0 : Scene.Edges[0].Id.Index1,
				INDEX_NONE};
			Vertex.GeometryType = CableSim::ECollisionGeometryType::Box;
			Vertex.bStaticObject = true;
			Vertex.Position = bStart ? Scene.Edges[0].Start : Scene.Edges[0].End;
			Vertex.IncidentEdges.Add(Scene.Edges[0].Id);
			Scene.Vertices.Add(Vertex);
		}
		return Scene;
	}

	CableSim::FTautStepInput MakeInput(const FVector3d& Start, const FVector3d& End)
	{
		return {Start, End};
	}

	bool PointsEqual(
		const TConstArrayView<CableSim::FTautPoint> First,
		const TConstArrayView<CableSim::FTautPoint> Second)
	{
		if (First.Num() != Second.Num())
		{
			return false;
		}
		for (int32 Index = 0; Index < First.Num(); ++Index)
		{
			if (First[Index].Type != Second[Index].Type
				|| First[Index].FeatureId != Second[Index].FeatureId
				|| !First[Index].Position.Equals(Second[Index].Position, 1.e-12)
				|| First[Index].bHasPendingTarget != Second[Index].bHasPendingTarget
				|| !First[Index].PendingTarget.Equals(Second[Index].PendingTarget, 1.e-12)
				|| First[Index].SingularOrientation != Second[Index].SingularOrientation
				|| !FMath::IsNearlyEqual(
					First[Index].EdgeParameter, Second[Index].EdgeParameter, 1.e-12))
			{
				return false;
			}
		}
		return true;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimTopologyBoxTest,
	"CableSim.Core.Topology.Box",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimTopologyBoxTest::RunTest(const FString& Parameters)
{
	const CableSimTautTests::FOwnedScene Scene = CableSimTautTests::MakeBoxScene();
	int32 ConvexEdges = 0;
	int32 CoplanarEdges = 0;
	for (const CableSim::FCollisionEdge& Edge : Scene.Edges)
	{
		ConvexEdges += Edge.Kind == CableSim::ECollisionEdgeKind::Convex ? 1 : 0;
		CoplanarEdges += Edge.Kind == CableSim::ECollisionEdgeKind::Coplanar ? 1 : 0;
		TestEqual(TEXT("Box geometry provenance reaches every edge"),
			Edge.GeometryType, CableSim::ECollisionGeometryType::Box);
		TestTrue(TEXT("Box static provenance reaches every edge"), Edge.bStaticObject);
	}
	TestEqual(TEXT("A triangulated engine cube has twelve convex edges"), ConvexEdges, 12);
	TestEqual(TEXT("Its six face diagonals remain non-wrapping topology"), CoplanarEdges, 6);
	TestEqual(TEXT("All full-shape edges are retained"), Scene.Edges.Num(), 18);
	TestEqual(TEXT("All eight shared cube vertices are retained"), Scene.Vertices.Num(), 8);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimTopologyTouchingShapesClusterTest,
	"CableSim.Core.Topology.TouchingShapesCluster",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimTopologyTouchingShapesClusterTest::RunTest(const FString& Parameters)
{
	TArray<CableSim::FCollisionTriangle> Triangles = CableSimTautTests::MakeBoxTriangles();
	TArray<CableSim::FCollisionTriangle> SecondBox = CableSimTautTests::MakeBoxTriangles();
	for (CableSim::FCollisionTriangle& Triangle : SecondBox)
	{
		Triangle.Id.ObjectToken = 2;
		for (FVector3d& Position : Triangle.Vertices)
		{
			Position.X += 2.0;
		}
	}
	Triangles.Append(SecondBox);
	TArray<CableSim::FCollisionEdge> Edges;
	TArray<CableSim::FCollisionVertex> Vertices;
	const CableSim::FTopologyCompileDiagnostics Diagnostics =
		CableSim::FCollisionTopologyCompiler::CompileTopology(
			Triangles, 1.e-6, 8, Edges, Vertices);
	TestEqual(TEXT("Four coincident vertex pairs become shared topology"), Vertices.Num(), 12);
	TestEqual(TEXT("Touching ordinary boxes remain within the valence budget"),
		Diagnostics.OverValenceVertexCount, 0);
	int32 MaximumValence = 0;
	for (const CableSim::FCollisionVertex& Vertex : Vertices)
	{
		MaximumValence = FMath::Max(MaximumValence, Vertex.IncidentEdges.Num());
	}
	TestEqual(TEXT("A shared cube corner carries the six wrapping edges"), MaximumValence, 6);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimTopologyDeterminismTest,
	"CableSim.Core.Topology.Determinism",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimTopologyDeterminismTest::RunTest(const FString& Parameters)
{
	TArray<CableSim::FCollisionTriangle> Forward = CableSimTautTests::MakeBoxTriangles();
	TArray<CableSim::FCollisionTriangle> Reverse = Forward;
	Algo::Reverse(Reverse);
	TArray<CableSim::FCollisionEdge> FirstEdges;
	TArray<CableSim::FCollisionEdge> SecondEdges;
	CableSim::FCollisionTopologyCompiler::CompileEdges(Forward, 1.e-6, FirstEdges);
	CableSim::FCollisionTopologyCompiler::CompileEdges(Reverse, 1.e-6, SecondEdges);
	TestEqual(TEXT("Triangle order produces the same edge count"), FirstEdges.Num(), SecondEdges.Num());
	for (int32 Index = 0; Index < FirstEdges.Num() && Index < SecondEdges.Num(); ++Index)
	{
		TestTrue(TEXT("Edge identifiers are stable"), FirstEdges[Index].Id == SecondEdges[Index].Id);
		TestEqual(TEXT("Edge classification is stable"), FirstEdges[Index].Kind, SecondEdges[Index].Kind);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimStaticTautTopologyContractTest,
	"CableSim.Core.Topology.StaticTautContract",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimStaticTautTopologyContractTest::RunTest(const FString& Parameters)
{
	TArray<CableSim::FCollisionTriangle> Forward = CableSimTautTests::MakeBoxTriangles();
	TArray<CableSim::FCollisionTriangle> Reverse = Forward;
	Algo::Reverse(Reverse);
	CableSim::FTautStaticTopology First;
	CableSim::FTautStaticTopology Second;
	TestTrue(TEXT("A complete static convex box is admitted"), First.Build(Forward, 1.e-6, 8));
	TestTrue(TEXT("Authored triangle order does not affect admission"), Second.Build(Reverse, 1.e-6, 8));
	TestEqual(TEXT("The persistent topology revision is order-independent"),
		First.GetRevision(), Second.GetRevision());
	TestEqual(TEXT("The provider publishes all convex box edges"),
		First.GetScene().Edges.Num(), 18);

	TArray<CableSim::FCollisionTriangle> TJunction = Forward;
	TArray<CableSim::FCollisionTriangle> SmallBox = CableSimTautTests::MakeBoxTriangles();
	for (CableSim::FCollisionTriangle& Triangle : SmallBox)
	{
		Triangle.Id.ObjectToken = 2;
		for (FVector3d& Position : Triangle.Vertices)
		{
			Position = 0.5 * Position + FVector3d(1.5, 1.5, 0.5);
		}
	}
	TJunction.Append(SmallBox);
	CableSim::FTautStaticTopology RejectedTJunction;
	TestFalse(TEXT("A shape vertex terminating in another shape's edge is rejected"),
		RejectedTJunction.Build(TJunction, 1.e-6, 8));
	TestEqual(TEXT("The T-junction failure is explicit"),
		RejectedTJunction.GetDiagnostics().Issue, CableSim::ETautTopologyIssue::TJunction);

	TArray<CableSim::FCollisionTriangle> Overlap = Forward;
	TArray<CableSim::FCollisionTriangle> CoincidentBox = CableSimTautTests::MakeBoxTriangles();
	for (CableSim::FCollisionTriangle& Triangle : CoincidentBox)
	{
		Triangle.Id.ObjectToken = 3;
	}
	Overlap.Append(CoincidentBox);
	CableSim::FTautStaticTopology RejectedOverlap;
	TestFalse(TEXT("Penetrating static convex shapes are rejected"),
		RejectedOverlap.Build(Overlap, 1.e-6, 8));
	TestEqual(TEXT("The overlap failure is explicit"),
		RejectedOverlap.GetDiagnostics().Issue, CableSim::ETautTopologyIssue::OverlappingShapes);

	TArray<CableSim::FCollisionTriangle> DisjointWithOverlappingBounds = Forward;
	TArray<CableSim::FCollisionTriangle> RotatedBox = CableSimTautTests::MakeBoxTriangles();
	const double C = FMath::Sqrt(0.5);
	for (CableSim::FCollisionTriangle& Triangle : RotatedBox)
	{
		Triangle.Id.ObjectToken = 4;
		for (FVector3d& Position : Triangle.Vertices)
		{
			const FVector3d Original = Position;
			Position.X = C * Original.X - C * Original.Y + 2.2;
			Position.Y = C * Original.X + C * Original.Y + 2.2;
		}
	}
	DisjointWithOverlappingBounds.Append(RotatedBox);
	CableSim::FTautStaticTopology ExactOverlapTest;
	TestTrue(TEXT("Overlapping AABBs do not reject separated rotated convexes"),
		ExactOverlapTest.Build(DisjointWithOverlappingBounds, 1.e-6, 8));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimTautDefaultCubeSeedTest,
	"CableSim.Core.Taut.DefaultCubeSeed",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimTautDefaultCubeSeedTest::RunTest(const FString& Parameters)
{
	const CableSimTautTests::FOwnedScene Scene = CableSimTautTests::MakeBoxScene();
	const FVector3d DirectSeed[] = {{-3.0, 0.0, 0.0}, {3.0, 0.0, 0.0}};
	CableSim::FTautPathSolver DirectSolver;
	TestTrue(TEXT("A blocked seed through opposing cube faces is repaired"),
		DirectSolver.Initialize(MakeArrayView(DirectSeed), Scene.View()));
	TestTrue(TEXT("The repaired opposing-face route is collision free"),
		DirectSolver.GetLastResult().bPathCollisionFree);
	TestTrue(TEXT("The repaired route goes around the cube"),
		DirectSolver.GetPoints().Num() > 2);

	const FVector3d RoutedSeed[] = {
		{-3.0, 0.0, 0.0}, {-2.0, 2.0, 0.0},
		{2.0, 2.0, 0.0}, {3.0, 0.0, 0.0}};
	CableSim::FTautPathSolver RoutedSolver;
	TestTrue(TEXT("A collision-free dynamic-rope seed around the cube initializes"),
		RoutedSolver.Initialize(MakeArrayView(RoutedSeed), Scene.View()));
	TestTrue(TEXT("The initialized route is collision free"),
		RoutedSolver.GetLastResult().bPathCollisionFree);
	TestTrue(TEXT("The route preserves contacts instead of collapsing through the cube"),
		RoutedSolver.GetPoints().Num() > 2);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimTautWrapLifecycleTest,
	"CableSim.Core.Taut.WrapLifecycle",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimTautWrapLifecycleTest::RunTest(const FString& Parameters)
{
	const CableSimTautTests::FOwnedScene Scene = CableSimTautTests::MakeVerticalEdgeScene();
	const FVector3d InitialStart(10.0, 10.0, 0.0);
	const FVector3d End(-10.0, 10.0, 0.0);
	const FVector3d Seed[] = {InitialStart, End};
	CableSim::FTautPathSolver Solver;
	TestTrue(TEXT("Taut solver initializes from a polyline"),
		Solver.Initialize(MakeArrayView(Seed), Scene.View()));
	const CableSim::FTautStepResult WrapResult = Solver.AdvanceStep(
		CableSimTautTests::MakeInput(FVector3d(10.0, -10.0, 0.0), End), Scene.View());
	TestEqual(TEXT("Continuous endpoint motion inserts an edge contact"), WrapResult.ContactCount, 1);
	TestTrue(TEXT("The wrapped path remains valid"), WrapResult.bPathCollisionFree);

	const CableSim::FTautStepResult SameFaceResult = Solver.AdvanceStep(
		CableSimTautTests::MakeInput(InitialStart, End), Scene.View());
	TestEqual(TEXT("Crossing back continuously forgets the unwrapped edge contact"),
		SameFaceResult.ContactCount, 0);
	TestTrue(TEXT("The straightened path remains collision free"),
		SameFaceResult.bPathCollisionFree);

	const CableSim::FTautStepResult RewrapResult = Solver.AdvanceStep(
		CableSimTautTests::MakeInput(FVector3d(10.0, -10.0, 0.0), End), Scene.View());
	TestEqual(TEXT("Crossing the edge again creates a fresh collision contact"),
		RewrapResult.ContactCount, 1);
	TestTrue(TEXT("The rewrapped contact is the live edge feature"),
		Solver.GetPoints().ContainsByPredicate([&Scene](const CableSim::FTautPoint& Point)
		{
			return Point.Type == CableSim::ETautPointType::EdgeContact
				&& Point.FeatureId == Scene.Edges[0].Id;
		}));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimTautReplayAndInvalidationTest,
	"CableSim.Core.Taut.ReplayAndConservativeInvalidation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimTautReplayAndInvalidationTest::RunTest(const FString& Parameters)
{
	const CableSimTautTests::FOwnedScene Scene = CableSimTautTests::MakeVerticalEdgeScene();
	const FVector3d Seed[] = {{10.0, 10.0, 0.0}, {-10.0, 10.0, 0.0}};
	CableSim::FTautPathSolver Solver;
	Solver.Initialize(MakeArrayView(Seed), Scene.View());
	Solver.AdvanceStep(
		CableSimTautTests::MakeInput({10.0, -10.0, 20.0}, {-10.0, 10.0, -10.0}),
		Scene.View());
	const CableSim::FTautStateSnapshot WrappedState = Solver.CaptureState();
	const CableSim::FTautReplayFrame Replay = Solver.GetLastReplayFrame();
	TestTrue(TEXT("Wrapped state restores"), Solver.RestoreState(WrappedState));
	const CableSim::FTautCollisionScene ReplayScene{
		Replay.Triangles, Replay.Edges, Replay.Vertices};
	Solver.AdvanceStep(Replay.Input, ReplayScene);
	TestTrue(TEXT("A captured solve replays deterministically"),
		CableSimTautTests::PointsEqual(WrappedState.Points, Solver.GetPoints()));

	const TArray<CableSim::FTautPoint> BeforeInvalidation = Solver.GetPoints();
	const CableSim::FTautStepResult Invalidated = Solver.AdvanceStep(Replay.Input, {});
	TestEqual(TEXT("A missing persistent feature is an explicit conservative failure"),
		Invalidated.Status, CableSim::ETautStatus::FeatureUnavailable);
	TestTrue(TEXT("Feature loss preserves the last valid path"),
		CableSimTautTests::PointsEqual(BeforeInvalidation, Solver.GetPoints()));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimTautBudgetDegradesTest,
	"CableSim.Core.Taut.BudgetExhaustionDegradesGracefully",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimTautBudgetDegradesTest::RunTest(const FString& Parameters)
{
	const CableSimTautTests::FOwnedScene Scene = CableSimTautTests::MakeVerticalEdgeScene();
	CableSim::FTautConfig Config;
	Config.MaximumCollisionPhases = 1;
	const FVector3d Seed[] = {{10.0, 10.0, 0.0}, {-10.0, 10.0, 0.0}};

	// Case 1: one endpoint crosses the edge. A single collision phase is enough to
	// find the complete wrap (endpoints at their targets, collision free), so
	// graceful degradation emits that best-effort path instead of freezing at the
	// seed the way the old rollback-on-budget behaviour did.
	{
		CableSim::FTautPathSolver Solver;
		Solver.Initialize(MakeArrayView(Seed), Scene.View(), Config);
		const CableSim::FTautStepResult Result = Solver.AdvanceStep(
			CableSimTautTests::MakeInput({10.0, -10.0, 0.0}, {-10.0, 10.0, 0.0}),
			Scene.View());
		TestTrue(TEXT("A complete wrap is emitted despite the tight budget"),
			Result.Status == CableSim::ETautStatus::Ready
			|| Result.Status == CableSim::ETautStatus::NoRelevantGeometry);
		TestTrue(TEXT("Best-effort path is collision free"), Result.bPathCollisionFree);
		TestEqual(TEXT("The wrap contact was found within the budget"), Result.ContactCount, 1);
		TestTrue(TEXT("Endpoints reach their requested targets"),
			Solver.GetPoints()[0].Position.Equals(FVector3d(10.0, -10.0, 0.0), 0.1)
			&& Solver.GetPoints().Last().Position.Equals(FVector3d(-10.0, 10.0, 0.0), 0.1));
	}

	// Case 2: both endpoints must cross the edge, so one phase cannot advance them
	// both to their targets. Emitting the half-advanced path would misplace an
	// endpoint, so the solver conservatively holds the last good state instead --
	// the endpoint-at-target guard on best-effort emission.
	{
		CableSim::FTautPathSolver Solver;
		Solver.Initialize(MakeArrayView(Seed), Scene.View(), Config);
		const CableSim::FTautStateSnapshot InitialState = Solver.CaptureState();
		const CableSim::FTautStepResult Result = Solver.AdvanceStep(
			CableSimTautTests::MakeInput({10.0, -10.0, 0.0}, {-10.0, -10.0, 0.0}),
			Scene.View());
		TestEqual(TEXT("An incomplete advance holds the last valid path"),
			Result.Status, CableSim::ETautStatus::CollisionBudgetExceeded);
		TestTrue(TEXT("The last valid state is preserved intact"),
			CableSimTautTests::PointsEqual(InitialState.Points, Solver.GetPoints()));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimTautParallelUnrollTest,
	"CableSim.Core.Taut.ParallelEdgesUnrollToTautLine",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimTautParallelUnrollTest::RunTest(const FString& Parameters)
{
	// A rope wrapping three PARALLEL ridge edges. The movement phase must reach the
	// exact taut configuration, which for parallel edges is a straight line once the
	// edges are unrolled into a plane. Unrolling is isometric, so the true 3D path
	// length equals the closed-form unrolled straight-line length -- this checks the
	// 2D-unroll acceleration produces the optimal path, not a Gauss-Seidel near-miss.
	const FVector3d Direction(0.0, 1.0, 0.0);
	CableSimTautTests::FOwnedScene Scene;
	const double Xs[3] = {0.0, 10.0, 20.0};
	for (int32 Index = 0; Index < 3; ++Index)
	{
		CableSim::FCollisionEdge Edge;
		Edge.Id = {1, 0, CableSim::ECollisionFeatureType::Edge, Index, Index + 10};
		Edge.GeometryType = CableSim::ECollisionGeometryType::Box;
		Edge.bStaticObject = true;
		Edge.Start = FVector3d(Xs[Index], -50.0, 10.0);
		Edge.End = FVector3d(Xs[Index], 50.0, 10.0);
		Edge.FaceNormal0 = FVector3d(-1.0, 0.0, 1.0).GetSafeNormal();
		Edge.FaceNormal1 = FVector3d(1.0, 0.0, 1.0).GetSafeNormal();
		Edge.Kind = CableSim::ECollisionEdgeKind::Convex;
		Scene.Edges.Add(Edge);
	}

	const FVector3d A(-15.0, -20.0, 5.0);
	const FVector3d B(35.0, 20.0, 5.0);

	CableSim::FTautStateSnapshot Snapshot;
	Snapshot.Config = CableSim::FTautConfig{};
	CableSim::FTautPoint StartPoint;
	StartPoint.Type = CableSim::ETautPointType::Endpoint;
	StartPoint.Position = A;
	Snapshot.Points.Add(StartPoint);
	for (int32 Index = 0; Index < 3; ++Index)
	{
		CableSim::FTautPoint Contact;
		Contact.Type = CableSim::ETautPointType::EdgeContact;
		Contact.FeatureId = Scene.Edges[Index].Id;
		Contact.Position = FVector3d(Xs[Index], 0.0, 10.0);
		Contact.EdgeParameter = 0.5;
		Snapshot.Points.Add(Contact);
	}
	CableSim::FTautPoint EndPoint;
	EndPoint.Type = CableSim::ETautPointType::Endpoint;
	EndPoint.Position = B;
	Snapshot.Points.Add(EndPoint);

	CableSim::FTautPathSolver Solver;
	TestTrue(TEXT("Injected parallel-edge state restores"), Solver.RestoreState(Snapshot));
	CableSim::FTautStepResult Result;
	for (int32 Step = 0; Step < 3; ++Step)
	{
		Result = Solver.AdvanceStep(CableSimTautTests::MakeInput(A, B), Scene.View());
	}

	// Closed-form unrolled length: cumulative perpendicular (to Direction) distance
	// across the edges, and the along-Direction span, as the two legs of a right
	// triangle.
	auto Perp = [&Direction](const FVector3d& P, const FVector3d& Q)
	{
		const FVector3d R = P - Q;
		return (R - Direction * FVector3d::DotProduct(R, Direction)).Length();
	};
	const double TotalHorizontal = Perp(A, Scene.Edges[0].Start)
		+ Perp(Scene.Edges[1].Start, Scene.Edges[0].Start)
		+ Perp(Scene.Edges[2].Start, Scene.Edges[1].Start)
		+ Perp(B, Scene.Edges[2].Start);
	const double AlongSpan = FVector3d::DotProduct(B - A, Direction);
	const double ExpectedLength = FMath::Sqrt(TotalHorizontal * TotalHorizontal + AlongSpan * AlongSpan);

	AddInfo(FString::Printf(TEXT("parallel unroll: status=%d contacts=%d pathLen=%.3f expected=%.3f"),
		static_cast<int32>(Result.Status), Result.ContactCount, Result.PathLength, ExpectedLength));
	TestTrue(TEXT("The parallel-edge solve is valid"),
		Result.Status == CableSim::ETautStatus::Ready
		|| Result.Status == CableSim::ETautStatus::NoRelevantGeometry);
	TestTrue(TEXT("The path is collision free"), Result.bPathCollisionFree);
	TestEqual(TEXT("All three parallel ridges are still wrapped"), Result.ContactCount, 3);
	TestTrue(TEXT("The taut path length matches the closed-form unrolled straight line"),
		FMath::IsNearlyEqual(Result.PathLength, ExpectedLength, 0.5));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimTautCoplanarUnrollTest,
	"CableSim.Core.Taut.CoplanarNonParallelEdgesUnrollToLine",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimTautCoplanarUnrollTest::RunTest(const FString& Parameters)
{
	CableSimTautTests::FOwnedScene Scene;
	CableSim::FCollisionEdge First;
	First.Id = {11, 0, CableSim::ECollisionFeatureType::Edge, 0, 1};
	First.GeometryType = CableSim::ECollisionGeometryType::Convex;
	First.bStaticObject = true;
	First.Start = {3.0, -5.0, 0.0};
	First.End = {3.0, 5.0, 0.0};
	First.FaceNormal0 = {-1.0, 0.0, 0.0};
	First.FaceNormal1 = {1.0, 0.0, 0.0};
	First.Kind = CableSim::ECollisionEdgeKind::Convex;
	CableSim::FCollisionEdge Second;
	Second.Id = {11, 0, CableSim::ECollisionFeatureType::Edge, 2, 3};
	Second.GeometryType = CableSim::ECollisionGeometryType::Convex;
	Second.bStaticObject = true;
	Second.Start = {5.0, -5.0, 0.0};
	Second.End = {9.0, 5.0, 0.0};
	Second.FaceNormal0 = FVector3d(-10.0, 4.0, 0.0).GetSafeNormal();
	Second.FaceNormal1 = -Second.FaceNormal0;
	Second.Kind = CableSim::ECollisionEdgeKind::Convex;
	Scene.Edges = {First, Second};

	CableSim::FTautStateSnapshot Snapshot;
	Snapshot.Config = CableSim::FTautConfig{};
	Snapshot.Config.MovementConvergenceTolerance = 1.e-9;
	CableSim::FTautPoint A;
	A.Type = CableSim::ETautPointType::Endpoint;
	A.Position = {0.0, 0.0, 0.0};
	CableSim::FTautPoint Contact0;
	Contact0.Type = CableSim::ETautPointType::EdgeContact;
	Contact0.FeatureId = First.Id;
	Contact0.Position = {3.0, 2.0, 0.0};
	Contact0.EdgeParameter = 0.7;
	CableSim::FTautPoint Contact1;
	Contact1.Type = CableSim::ETautPointType::EdgeContact;
	Contact1.FeatureId = Second.Id;
	Contact1.Position = {6.2, -2.0, 0.0};
	Contact1.EdgeParameter = 0.3;
	CableSim::FTautPoint B;
	B.Type = CableSim::ETautPointType::Endpoint;
	B.Position = {10.0, 0.0, 0.0};
	Snapshot.Points = {A, Contact0, Contact1, B};

	CableSim::FTautPathSolver Solver;
	TestTrue(TEXT("The non-parallel coplanar seed restores"), Solver.RestoreState(Snapshot));
	const CableSim::FTautStepResult Result = Solver.AdvanceStep(
		CableSimTautTests::MakeInput(A.Position, B.Position), Scene.View());
	TestTrue(TEXT("The coplanar solve remains valid and collision free"),
		(Result.Status == CableSim::ETautStatus::Ready
			|| Result.Status == CableSim::ETautStatus::NoRelevantGeometry)
		&& Result.bPathCollisionFree);
	TestEqual(TEXT("The two edge contacts remain in the route"), Result.ContactCount, 2);
	TestTrue(TEXT("The unfolded route is the exact straight chord"),
		FMath::IsNearlyEqual(Result.PathLength, 10.0, 1.e-6));
	const TArray<CableSim::FTautPoint>& Points = Solver.GetPoints();
	if (Points.Num() == 4)
	{
		TestTrue(TEXT("The first non-parallel edge is intersected exactly"),
			Points[1].Position.Equals(FVector3d(3.0, 0.0, 0.0), 1.e-6));
		TestTrue(TEXT("The second non-parallel edge is intersected exactly"),
			Points[2].Position.Equals(FVector3d(7.0, 0.0, 0.0), 1.e-6));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimTautCornerResolutionTest,
	"CableSim.Core.Taut.CornerResolutionIsOptimal",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimTautCornerResolutionTest::RunTest(const FString& Parameters)
{
	// Put a contact exactly on a real compiled cube corner and resolve it against
	// a spread of neighbour positions. The talk's oriented classifier may retain
	// a stable inner contact, slide out along one edge, or split an outer contact
	// into an ordered pair. An outer route can be longer than the chord through the
	// vertex because that chord crosses the forbidden solid sector; continuity and
	// collision freedom, not a homotopy-blind vertex shortcut, are the invariants.
	const double S = 50.0;
	CableSimTautTests::FOwnedScene Scene;
	Scene.Triangles = CableSimTautTests::MakeBoxTriangles();
	for (CableSim::FCollisionTriangle& Triangle : Scene.Triangles)
	{
		for (FVector3d& Vertex : Triangle.Vertices)
		{
			Vertex *= S;
		}
	}
	CableSim::FCollisionTopologyCompiler::CompileTopology(
		Scene.Triangles, 1.e-4, 8, Scene.Edges, Scene.Vertices);

	const FVector3d CornerTarget(S, S, S);
	const CableSim::FCollisionVertex* Corner = nullptr;
	double BestDistance = TNumericLimits<double>::Max();
	for (const CableSim::FCollisionVertex& Vertex : Scene.Vertices)
	{
		const double Distance = FVector3d::Distance(Vertex.Position, CornerTarget);
		if (Distance < BestDistance)
		{
			BestDistance = Distance;
			Corner = &Vertex;
		}
	}
	TestTrue(TEXT("A cube corner vertex was compiled"), Corner != nullptr && BestDistance < 1.0);
	if (!Corner)
	{
		return false;
	}
	TestTrue(TEXT("The corner has multiple incident edges"), Corner->IncidentEdges.Num() >= 2);

	// Neighbour positions spread across the three faces meeting at the corner, some
	// requiring a wrap and some not.
	const FVector3d Samples[] = {
		{S + 30.0, S - 20.0, S - 35.0}, {S - 35.0, S + 30.0, S - 20.0},
		{S - 20.0, S - 35.0, S + 30.0}, {S + 40.0, S + 40.0, S - 30.0},
		{S - 30.0, S + 40.0, S + 40.0}, {S + 40.0, S - 30.0, S + 40.0},
		{S + 25.0, S + 25.0, S + 25.0}, {S - 45.0, S - 10.0, S + 35.0}};

	int32 SplitCount = 0;
	int32 EvaluatedCount = 0;
	int32 OneSidedLimitCount = 0;
	int32 ActionCounts[9] = {};
	for (const FVector3d& Previous : Samples)
	{
		for (const FVector3d& Next : Samples)
		{
			if (Previous.Equals(Next))
			{
				continue;
			}
			CableSim::FTautStateSnapshot Snapshot;
			Snapshot.Config = CableSim::FTautConfig{};
			CableSim::FTautPoint StartPoint;
			StartPoint.Type = CableSim::ETautPointType::Endpoint;
			StartPoint.Position = Previous;
			CableSim::FTautPoint VertexPoint;
			VertexPoint.Type = CableSim::ETautPointType::VertexContact;
			VertexPoint.Position = Corner->Position;
			VertexPoint.FeatureId = Corner->Id;
			CableSim::FTautPoint EndPoint;
			EndPoint.Type = CableSim::ETautPointType::Endpoint;
			EndPoint.Position = Next;
			Snapshot.Points = {StartPoint, VertexPoint, EndPoint};

			CableSim::FTautPathSolver Solver;
			if (!Solver.RestoreState(Snapshot))
			{
				continue;
			}
			const CableSim::FTautStepResult Result = Solver.AdvanceStep(
				CableSimTautTests::MakeInput(Previous, Next), Scene.View());
			if (Result.Status != CableSim::ETautStatus::Ready
				&& Result.Status != CableSim::ETautStatus::NoRelevantGeometry)
			{
				continue;
			}
			++EvaluatedCount;
			for (const CableSim::FTautPairDecision& Decision : Solver.GetLastPairDecisions())
			{
				++ActionCounts[static_cast<uint8>(Decision.Action)];
				OneSidedLimitCount += Decision.bUsedOneSidedLimit ? 1 : 0;
			}
			SplitCount += Result.OuterSplitCount;
			const double StayLength = FVector3d::Distance(Previous, Corner->Position)
				+ FVector3d::Distance(Corner->Position, Next);
			TestTrue(TEXT("Resolved corner path is collision free"), Result.bPathCollisionFree);
			if (Result.ContactCount < 2)
			{
				TestTrue(TEXT("A stable/single-edge resolution is no longer than staying on the vertex"),
					Result.PathLength <= StayLength + 0.5);
			}
			if (Result.OuterSplitCount > 0)
			{
				CableSim::FTautPathSolver ReplaySolver;
				TestTrue(TEXT("The corner seed can be replayed"), ReplaySolver.RestoreState(Snapshot));
				const CableSim::FTautStepResult ReplayResult = ReplaySolver.AdvanceStep(
					CableSimTautTests::MakeInput(Previous, Next), Scene.View());
				TestEqual(TEXT("Outer-corner event count is deterministic"),
					ReplayResult.OuterSplitCount, Result.OuterSplitCount);
				TestTrue(TEXT("Outer-corner ordering is deterministic"),
					CableSimTautTests::PointsEqual(Solver.GetPoints(), ReplaySolver.GetPoints()));
			}
		}
	}
	TestTrue(TEXT("At least some corner cases were evaluated"), EvaluatedCount > 0);
	TestTrue(TEXT("Unresolved exact singular predicates remain explicit"), ActionCounts[0] > 0);
	TestTrue(TEXT("Exact corner predicates exercise continuous one-sided limits"),
		OneSidedLimitCount > 0);
	TestTrue(TEXT("Stable inner-corner classification is exercised"), ActionCounts[1] > 0);
	TestTrue(TEXT("Move-Along A/B classification is exercised"),
		ActionCounts[2] + ActionCounts[3] > 0);
	TestTrue(TEXT("Unstable choose-A-or-B classification is exercised"), ActionCounts[4] > 0);
	TestTrue(TEXT("Side-edge ignore classification is exercised"),
		ActionCounts[5] + ActionCounts[6] > 0);
	TestTrue(TEXT("Both ordered outer-corner classifications are exercised before later release events"),
		ActionCounts[7] > 0 && ActionCounts[8] > 0);
	AddInfo(FString::Printf(
		TEXT("Corner resolution: evaluated=%d splitCases=%d actions=[%d,%d,%d,%d,%d,%d,%d,%d,%d]"),
		EvaluatedCount, SplitCount,
		ActionCounts[0], ActionCounts[1], ActionCounts[2], ActionCounts[3], ActionCounts[4],
		ActionCounts[5], ActionCounts[6], ActionCounts[7], ActionCounts[8]));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimTautReverseCornerMergeTest,
	"CableSim.Core.Taut.ReverseOuterContactsResolveAtSharedVertex",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimTautReverseCornerMergeTest::RunTest(const FString& Parameters)
{
	CableSimTautTests::FOwnedScene Scene = CableSimTautTests::MakeBoxScene();
	const CableSim::FCollisionVertex& Corner = Scene.Vertices.Last();
	if (!TestTrue(TEXT("The test corner has two incident edges"), Corner.IncidentEdges.Num() >= 2))
	{
		return false;
	}
	auto FindEdge = [&Scene](const CableSim::FCollisionFeatureId& Id)
	{
		return Scene.Edges.FindByPredicate([&Id](const CableSim::FCollisionEdge& Edge)
		{
			return Edge.Id == Id;
		});
	};
	const CableSim::FCollisionEdge* FirstEdge = FindEdge(Corner.IncidentEdges[0]);
	const CableSim::FCollisionEdge* SecondEdge = FindEdge(Corner.IncidentEdges[1]);
	if (!TestNotNull(TEXT("The incident edges are published"), FirstEdge)
		|| !TestNotNull(TEXT("The second incident edge is published"), SecondEdge))
	{
		return false;
	}
	const FVector3d Samples[] = {
		Corner.Position + FVector3d(3.0, -2.0, -4.0),
		Corner.Position + FVector3d(-4.0, 3.0, -2.0),
		Corner.Position + FVector3d(-2.0, -4.0, 3.0),
		Corner.Position + FVector3d(4.0, 4.0, 4.0)};
	int32 SharedVertexResolutionCount = 0;
	for (const FVector3d& Previous : Samples)
	{
		for (const FVector3d& Next : Samples)
		{
			if (Previous.Equals(Next))
			{
				continue;
			}
			CableSim::FTautStateSnapshot Snapshot;
			Snapshot.Config = CableSim::FTautConfig{};
			CableSim::FTautPoint Start;
			Start.Type = CableSim::ETautPointType::Endpoint;
			Start.Position = Previous;
			CableSim::FTautPoint FirstContact;
			FirstContact.Type = CableSim::ETautPointType::EdgeContact;
			FirstContact.FeatureId = FirstEdge->Id;
			FirstContact.Position = Corner.Position;
			FirstContact.EdgeParameter = FVector3d::Distance(FirstEdge->Start, Corner.Position) < 1.e-6
				? 0.0 : 1.0;
			CableSim::FTautPoint SecondContact;
			SecondContact.Type = CableSim::ETautPointType::EdgeContact;
			SecondContact.FeatureId = SecondEdge->Id;
			SecondContact.Position = Corner.Position;
			SecondContact.EdgeParameter = FVector3d::Distance(SecondEdge->Start, Corner.Position) < 1.e-6
				? 0.0 : 1.0;
			CableSim::FTautPoint End;
			End.Type = CableSim::ETautPointType::Endpoint;
			End.Position = Next;
			Snapshot.Points = {Start, FirstContact, SecondContact, End};
			CableSim::FTautPathSolver Solver;
			if (!Solver.RestoreState(Snapshot))
			{
				continue;
			}
			const CableSim::FTautStepResult Result = Solver.AdvanceStep(
				CableSimTautTests::MakeInput(Previous, Next), Scene.View());
			const TArray<CableSim::FTautPoint>& Points = Solver.GetPoints();
			if ((Result.Status == CableSim::ETautStatus::Ready
					|| Result.Status == CableSim::ETautStatus::NoRelevantGeometry)
				&& Points.Num() == 3
				&& Points[1].Type == CableSim::ETautPointType::VertexContact
				&& Points[1].FeatureId == Corner.Id)
			{
				++SharedVertexResolutionCount;
				TestTrue(TEXT("Reverse merge retains the shared geometric position"),
					Points[1].Position.Equals(Corner.Position, 1.e-9));
			}
			else if ((Result.Status == CableSim::ETautStatus::Ready
					|| Result.Status == CableSim::ETautStatus::NoRelevantGeometry)
				&& Solver.GetLastVertexDecisions().ContainsByPredicate([](
					const CableSim::FTautVertexDecision& Decision)
				{
					return Decision.Resolution == CableSim::ETautVertexResolution::Release;
				}))
			{
				++SharedVertexResolutionCount;
				TestTrue(TEXT("A clear reverse merge releases continuously"),
					Result.bPathCollisionFree);
			}
		}
	}
	TestTrue(TEXT("At least one reverse route resolves through the shared vertex"),
		SharedVertexResolutionCount > 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimTautEdgeEndpointPromotesToVertexTest,
	"CableSim.Core.Taut.EdgeEndpointPromotesToVertex",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimTautEdgeEndpointPromotesToVertexTest::RunTest(const FString& Parameters)
{
	const CableSimTautTests::FOwnedScene Scene = CableSimTautTests::MakeVerticalEdgeScene();
	const CableSim::FCollisionEdge& Edge = Scene.Edges[0];
	const CableSim::FCollisionVertex& EndVertex = Scene.Vertices[1];
	const FVector3d Previous(10.0, 0.0, Edge.End.Z);
	const FVector3d Next(-10.0, 0.0, Edge.End.Z);

	CableSim::FTautStateSnapshot Snapshot;
	Snapshot.Config = CableSim::FTautConfig{};
	CableSim::FTautPoint Start;
	Start.Position = Previous;
	CableSim::FTautPoint Contact;
	Contact.Type = CableSim::ETautPointType::EdgeContact;
	Contact.FeatureId = Edge.Id;
	Contact.Position = Edge.End;
	Contact.EdgeParameter = 1.0;
	CableSim::FTautPoint End;
	End.Position = Next;
	Snapshot.Points = {Start, Contact, End};

	CableSim::FTautPathSolver Solver;
	TestTrue(TEXT("Endpoint edge state restores"), Solver.RestoreState(Snapshot));
	const CableSim::FTautStepResult Result = Solver.AdvanceStep(
		CableSimTautTests::MakeInput(Previous, Next), Scene.View());
	const TArray<CableSim::FTautPoint>& Points = Solver.GetPoints();
	TestTrue(TEXT("Endpoint promotion remains a valid continuous path"),
		(Result.Status == CableSim::ETautStatus::Ready
			|| Result.Status == CableSim::ETautStatus::NoRelevantGeometry)
		&& Result.bPathCollisionFree);
	TestTrue(TEXT("An edge point at endpoint tolerance enters vertex classification"),
		Solver.GetLastVertexDecisions().ContainsByPredicate([&EndVertex](
			const CableSim::FTautVertexDecision& Decision)
		{
			return Decision.VertexId == EndVertex.Id;
		}));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimTautLiftOffUsesContinuousSweepTest,
	"CableSim.Core.Taut.InteriorEdgeReleasesFromLiveSupport",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimTautLiftOffUsesContinuousSweepTest::RunTest(const FString& Parameters)
{
	CableSimTautTests::FOwnedScene BlockedScene = CableSimTautTests::MakeVerticalEdgeScene();
	const CableSim::FCollisionEdge& OwnEdge = BlockedScene.Edges[0];
	CableSim::FCollisionEdge Blocker = OwnEdge;
	Blocker.Id = {8, 2, CableSim::ECollisionFeatureType::Edge, 20, 21};
	Blocker.Start = {0.0, 5.0, -10.0};
	Blocker.End = {0.0, 5.0, 10.0};
	BlockedScene.Edges.Add(Blocker);

	const FVector3d Previous(10.0, 10.0, 0.0);
	const FVector3d Next(-10.0, 10.0, 0.0);
	CableSim::FTautStateSnapshot Snapshot;
	Snapshot.Config = CableSim::FTautConfig{};
	CableSim::FTautPoint Start;
	Start.Position = Previous;
	CableSim::FTautPoint Contact;
	Contact.Type = CableSim::ETautPointType::EdgeContact;
	Contact.FeatureId = OwnEdge.Id;
	Contact.Position = {0.0, 0.0, 0.0};
	Contact.EdgeParameter = 0.5;
	CableSim::FTautPoint End;
	End.Position = Next;
	Snapshot.Points = {Start, Contact, End};

	CableSim::FTautPathSolver Solver;
	TestTrue(TEXT("Bent edge state restores"), Solver.RestoreState(Snapshot));
	const CableSim::FTautStepResult Released = Solver.AdvanceStep(
		CableSimTautTests::MakeInput(Previous, Next), BlockedScene.View());
	TestEqual(TEXT("Same-side endpoints release the unsupported yellow contact"),
		Released.ContactCount, 0);
	TestTrue(TEXT("An unrelated edge in the old bend sweep does not preserve topology"),
		Released.bPathCollisionFree);
	TestFalse(TEXT("The released feature is absent from the live path"),
		Solver.GetPoints().ContainsByPredicate([&OwnEdge](const CableSim::FTautPoint& Point)
		{
			return Point.FeatureId == OwnEdge.Id;
		}));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimTautVertexLifecycleTest,
	"CableSim.Core.Taut.VertexReclassifiesAndReleases",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimTautVertexLifecycleTest::RunTest(const FString& Parameters)
{
	constexpr double Scale = 50.0;
	CableSimTautTests::FOwnedScene Scene;
	Scene.Triangles = CableSimTautTests::MakeBoxTriangles();
	for (CableSim::FCollisionTriangle& Triangle : Scene.Triangles)
	{
		for (FVector3d& Position : Triangle.Vertices)
		{
			Position *= Scale;
		}
	}
	CableSim::FCollisionTopologyCompiler::CompileTopology(
		Scene.Triangles, 1.e-4, 8, Scene.Edges, Scene.Vertices);
	const CableSim::FCollisionVertex* Corner = Scene.Vertices.FindByPredicate([](
		const CableSim::FCollisionVertex& Vertex)
	{
		return Vertex.Position.Equals(FVector3d(Scale, Scale, Scale), 1.0);
	});
	if (!TestNotNull(TEXT("The stateful fixture has a cube corner"), Corner))
	{
		return false;
	}

	const FVector3d StablePrevious(90.0, 90.0, 20.0);
	const FVector3d StableNext(80.0, 30.0, 15.0);
	CableSim::FTautStateSnapshot Seed;
	Seed.Config = CableSim::FTautConfig{};
	CableSim::FTautPoint Start;
	Start.Position = StablePrevious;
	CableSim::FTautPoint Contact;
	Contact.Type = CableSim::ETautPointType::VertexContact;
	Contact.Position = Corner->Position;
	Contact.FeatureId = Corner->Id;
	CableSim::FTautPoint End;
	End.Position = StableNext;
	Seed.Points = {Start, Contact, End};

	CableSim::FTautPathSolver Solver;
	TestTrue(TEXT("The obsolete vertex fixture restores"), Solver.RestoreState(Seed));
	const CableSim::FTautStepResult ReleasedResult = Solver.AdvanceStep(
		CableSimTautTests::MakeInput(StablePrevious, StableNext), Scene.View());
	const bool bReleased = Solver.GetLastVertexDecisions().ContainsByPredicate([](
		const CableSim::FTautVertexDecision& Decision)
	{
		return Decision.Resolution == CableSim::ETautVertexResolution::Release;
	});
	const bool bOriginalRedRemains = Solver.GetPoints().ContainsByPredicate([Corner](
		const CableSim::FTautPoint& Point)
	{
		return Point.Type == CableSim::ETautPointType::VertexContact
			&& Point.FeatureId == Corner->Id;
	});
	TestTrue(TEXT("The live matrix records a vertex release"), bReleased);
	TestTrue(TEXT("The obsolete red contact is forgotten"),
		(ReleasedResult.Status == CableSim::ETautStatus::Ready
			|| ReleasedResult.Status == CableSim::ETautStatus::NoRelevantGeometry)
		&& ReleasedResult.bPathCollisionFree && !bOriginalRedRemains);
	return true;
}

#endif
