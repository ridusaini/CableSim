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

	const CableSim::FTautStepResult UnwrapResult = Solver.AdvanceStep(
		CableSimTautTests::MakeInput(InitialStart, End), Scene.View());
	TestEqual(TEXT("Returning to the same incident face releases the contact"),
		UnwrapResult.ContactCount, 0);
	TestEqual(TEXT("The solver is not sticky after the obstruction clears"),
		UnwrapResult.PointCount, 2);
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
	FCableSimTautCornerResolutionTest,
	"CableSim.Core.Taut.CornerResolutionIsOptimal",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimTautCornerResolutionTest::RunTest(const FString& Parameters)
{
	// Put a contact exactly on a real compiled cube corner (via a restored state)
	// and resolve it against a spread of neighbour positions. The vertex resolver
	// must, for every case, choose a collision-free path no longer than simply
	// staying on the corner -- the shortest-path invariant behind the stay / single
	// / split (outer-corner) decision. This directly exercises the split branch:
	// whenever routing across two of the corner's edges is shorter than one edge or
	// the vertex, the resolver takes it, and that path is still valid.
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
			const double StayLength = FVector3d::Distance(Previous, Corner->Position)
				+ FVector3d::Distance(Corner->Position, Next);
			TestTrue(TEXT("Resolved corner path is collision free"), Result.bPathCollisionFree);
			TestTrue(TEXT("Resolved corner path is never longer than staying on the vertex"),
				Result.PathLength <= StayLength + 0.5);
			if (Result.ContactCount >= 2)
			{
				++SplitCount;
			}
		}
	}
	TestTrue(TEXT("At least some corner cases were evaluated"), EvaluatedCount > 0);
	AddInfo(FString::Printf(
		TEXT("Corner resolution: evaluated=%d splitCases=%d"), EvaluatedCount, SplitCount));
	return true;
}

#endif
