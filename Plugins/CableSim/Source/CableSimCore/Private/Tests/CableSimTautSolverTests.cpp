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
	FCableSimTautCollisionBudgetTest,
	"CableSim.Core.Taut.CollisionBudget",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimTautCollisionBudgetTest::RunTest(const FString& Parameters)
{
	const CableSimTautTests::FOwnedScene Scene = CableSimTautTests::MakeVerticalEdgeScene();
	CableSim::FTautConfig Config;
	Config.MaximumCollisionPhases = 1;
	const FVector3d Seed[] = {{10.0, 10.0, 0.0}, {-10.0, 10.0, 0.0}};
	CableSim::FTautPathSolver Solver;
	Solver.Initialize(MakeArrayView(Seed), Scene.View(), Config);
	const CableSim::FTautStateSnapshot InitialState = Solver.CaptureState();
	const CableSim::FTautStepResult Result = Solver.AdvanceStep(
		CableSimTautTests::MakeInput({10.0, -10.0, 0.0}, {-10.0, 10.0, 0.0}),
		Scene.View());
	TestEqual(TEXT("Collision-phase exhaustion has its own status"),
		Result.Status, CableSim::ETautStatus::CollisionBudgetExceeded);
	TestTrue(TEXT("Budget exhaustion preserves the last valid state"),
		CableSimTautTests::PointsEqual(InitialState.Points, Solver.GetPoints()));
	return true;
}

#endif
