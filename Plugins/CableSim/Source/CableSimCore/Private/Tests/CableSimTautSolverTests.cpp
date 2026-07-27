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
				Indices[Index][0],
				Indices[Index][1],
				Indices[Index][2],
				Vertices[Indices[Index][0]],
				Vertices[Indices[Index][1]],
				Vertices[Indices[Index][2]]));
		}
		return Triangles;
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

	CableSim::FCollisionEdge MakeSecondVerticalEdge()
	{
		CableSim::FCollisionEdge Edge = MakeVerticalEdge();
		Edge.Id.Index0 = 20;
		Edge.Id.Index1 = 21;
		Edge.Start.X = -5.0;
		Edge.End.X = -5.0;
		Edge.Start.Y = 5.0;
		Edge.End.Y = 5.0;
		return Edge;
	}

	CableSim::FTautStepInput MakeInput(const FVector3d& Start, const FVector3d& End)
	{
		CableSim::FTautStepInput Input;
		Input.StartTarget = Start;
		Input.EndTarget = End;
		return Input;
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
				|| !FMath::IsNearlyEqual(First[Index].EdgeParameter, Second[Index].EdgeParameter, 1.e-12))
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
	const TArray<CableSim::FCollisionTriangle> Triangles = CableSimTautTests::MakeBoxTriangles();
	TArray<CableSim::FCollisionEdge> Edges;
	TArray<CableSim::FCollisionVertex> Vertices;
	const CableSim::FTopologyCompileDiagnostics Diagnostics =
		CableSim::FCollisionTopologyCompiler::CompileTopology(Triangles, 1.e-6, 8, Edges, Vertices);
	TestEqual(TEXT("A triangulated box has twelve convex boundary edges"), Diagnostics.ConvexEdgeCount, 12);
	TestEqual(TEXT("A triangulated box has six coplanar face diagonals"), Diagnostics.CoplanarEdgeCount, 6);
	TestEqual(TEXT("A closed box has no boundary edges"), Diagnostics.BoundaryEdgeCount, 0);
	TestEqual(TEXT("All unique box edges are emitted"), Edges.Num(), 18);
	TestEqual(TEXT("The topology snapshot retains eight shared vertices"), Vertices.Num(), 8);
	TestEqual(TEXT("Box topology stays within the incident-edge budget"), Diagnostics.OverValenceVertexCount, 0);
	for (const CableSim::FCollisionEdge& Edge : Edges)
	{
		TestEqual(TEXT("Geometry provenance reaches compiled edges"), Edge.GeometryType, CableSim::ECollisionGeometryType::Box);
		TestTrue(TEXT("Static provenance reaches compiled edges"), Edge.bStaticObject);
	}
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
	TestEqual(TEXT("Both orders produce the same number of edges"), FirstEdges.Num(), SecondEdges.Num());
	for (int32 Index = 0; Index < FirstEdges.Num() && Index < SecondEdges.Num(); ++Index)
	{
		TestTrue(TEXT("Stable edge identifiers"), FirstEdges[Index].Id == SecondEdges[Index].Id);
		TestEqual(TEXT("Stable edge classification"), FirstEdges[Index].Kind, SecondEdges[Index].Kind);
		TestTrue(TEXT("Stable edge start"), FirstEdges[Index].Start.Equals(SecondEdges[Index].Start, 1.e-12));
		TestTrue(TEXT("Stable edge end"), FirstEdges[Index].End.Equals(SecondEdges[Index].End, 1.e-12));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimTautWrapLifecycleTest,
	"CableSim.Core.Taut.WrapLifecycle",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimTautWrapLifecycleTest::RunTest(const FString& Parameters)
{
	const FVector3d InitialStart(10.0, 10.0, 0.0);
	const FVector3d End(-10.0, 10.0, 0.0);
	const CableSim::FCollisionEdge Edge = CableSimTautTests::MakeVerticalEdge();
	CableSim::FTautPathSolver Solver;
	TestTrue(TEXT("Taut solver initializes"), Solver.Initialize(InitialStart, End));
	const CableSim::FTautStepResult WrapResult = Solver.AdvanceStep(
		CableSimTautTests::MakeInput(FVector3d(10.0, -10.0, 0.0), End),
		MakeArrayView(&Edge, 1));
	TestEqual(TEXT("Crossing inserts one edge contact"), WrapResult.ContactCount, 1);
	TestEqual(TEXT("One-edge path contains three points"), WrapResult.PointCount, 3);
	TestEqual(TEXT("Supported wrap remains ready"), WrapResult.Status, CableSim::ETautStatus::Ready);

	const CableSim::FTautStepResult UnwrapResult = Solver.AdvanceStep(
		CableSimTautTests::MakeInput(InitialStart, End),
		MakeArrayView(&Edge, 1));
	TestEqual(TEXT("Returning to the same incident face unwraps"), UnwrapResult.ContactCount, 0);
	TestEqual(TEXT("Unwrapped path contains endpoints only"), UnwrapResult.PointCount, 2);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimTautSlideAndVertexBoundaryTest,
	"CableSim.Core.Taut.SlideAndVertexBoundary",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimTautSlideAndVertexBoundaryTest::RunTest(const FString& Parameters)
{
	const CableSim::FCollisionEdge Edge = CableSimTautTests::MakeVerticalEdge();
	CableSim::FTautPathSolver Solver;
	Solver.Initialize(FVector3d(10.0, 10.0, 0.0), FVector3d(-10.0, 10.0, 0.0));
	Solver.AdvanceStep(
		CableSimTautTests::MakeInput(FVector3d(10.0, -10.0, 0.0), FVector3d(-10.0, 10.0, 0.0)),
		MakeArrayView(&Edge, 1));
	const CableSim::FTautStepResult SlideResult = Solver.AdvanceStep(
		CableSimTautTests::MakeInput(FVector3d(10.0, -10.0, 20.0), FVector3d(-10.0, 10.0, -10.0)),
		MakeArrayView(&Edge, 1));
	TestEqual(TEXT("Sliding remains a supported one-edge path"), SlideResult.Status, CableSim::ETautStatus::Ready);
	TestTrue(TEXT("The contact slides to the unfolded axial minimum"), Solver.GetPoints()[1].Position.Equals(FVector3d(0.0, 0.0, 5.0), 1.e-9));

	const CableSim::FTautStepResult VertexResult = Solver.AdvanceStep(
		CableSimTautTests::MakeInput(FVector3d(10.0, -10.0, 300.0), FVector3d(-10.0, 10.0, 300.0)),
		MakeArrayView(&Edge, 1));
	TestEqual(TEXT("A represented vertex remains a valid topology point"), VertexResult.Status, CableSim::ETautStatus::Ready);
	TestTrue(TEXT("The contact clamps to the represented edge vertex"),
		Solver.GetPoints()[1].Position.Equals(Edge.End, 1.e-9));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimTautReplayAndInvalidationTest,
	"CableSim.Core.Taut.ReplayAndInvalidation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimTautReplayAndInvalidationTest::RunTest(const FString& Parameters)
{
	const CableSim::FCollisionEdge Edge = CableSimTautTests::MakeVerticalEdge();
	CableSim::FTautPathSolver Solver;
	Solver.Initialize(FVector3d(10.0, 10.0, 0.0), FVector3d(-10.0, 10.0, 0.0));
	const CableSim::FTautStateSnapshot InitialState = Solver.CaptureState();
	Solver.AdvanceStep(
		CableSimTautTests::MakeInput(FVector3d(10.0, -10.0, 20.0), FVector3d(-10.0, 10.0, -10.0)),
		MakeArrayView(&Edge, 1));
	const CableSim::FTautStateSnapshot FirstResult = Solver.CaptureState();
	const CableSim::FTautReplayFrame ReplayFrame = Solver.GetLastReplayFrame();
	TestTrue(TEXT("Initial state restores"), Solver.RestoreState(InitialState));
	Solver.AdvanceStep(ReplayFrame.Input, ReplayFrame.Edges);
	TestTrue(TEXT("Recorded topology and input replay deterministically"),
		CableSimTautTests::PointsEqual(FirstResult.Points, Solver.GetPoints()));

	const CableSim::FTautStateSnapshot BeforeInvalidation = Solver.CaptureState();
	const CableSim::FTautStepResult Invalidated = Solver.AdvanceStep(ReplayFrame.Input, {});
	TestEqual(TEXT("A disappeared persistent feature is reported"), Invalidated.Status, CableSim::ETautStatus::FeatureInvalidated);
	TestTrue(TEXT("Feature invalidation preserves the last valid path"),
		CableSimTautTests::PointsEqual(BeforeInvalidation.Points, Solver.GetPoints()));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimTautMultipleContactBoundaryTest,
	"CableSim.Core.Taut.MultipleContactBoundary",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimTautMultipleContactBoundaryTest::RunTest(const FString& Parameters)
{
	CableSim::FTautPathSolver Solver;
	Solver.Initialize(FVector3d(10.0, 10.0, 0.0), FVector3d(-10.0, -10.0, 0.0));
	const CableSim::FCollisionEdge Edges[2] = {
		CableSimTautTests::MakeVerticalEdge(),
		CableSimTautTests::MakeSecondVerticalEdge()};
	const CableSim::FTautStepResult Result = Solver.AdvanceStep(
		CableSimTautTests::MakeInput(FVector3d(10.0, -10.0, 0.0), FVector3d(-10.0, 10.0, 0.0)),
		MakeArrayView(Edges));
	TestEqual(TEXT("Separate simultaneous wraps remain supported"), Result.Status, CableSim::ETautStatus::Ready);
	TestEqual(TEXT("The ordered path retains both edge contacts"), Result.ContactCount, 2);
	TestEqual(TEXT("Two contacts plus two endpoints form four path points"), Result.PointCount, 4);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimTautBudgetTest,
	"CableSim.Core.Taut.IterationBudget",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimTautBudgetTest::RunTest(const FString& Parameters)
{
	CableSim::FTautConfig Config;
	Config.MaximumCollisionPasses = 1;
	CableSim::FTautPathSolver Solver;
	Solver.Initialize(FVector3d(10.0, 10.0, 0.0), FVector3d(-10.0, 10.0, 0.0), Config);
	const CableSim::FTautStateSnapshot InitialState = Solver.CaptureState();
	const CableSim::FCollisionEdge Edge = CableSimTautTests::MakeVerticalEdge();
	const CableSim::FTautStepResult Result = Solver.AdvanceStep(
		CableSimTautTests::MakeInput(FVector3d(10.0, -10.0, 0.0), FVector3d(-10.0, 10.0, 0.0)),
		MakeArrayView(&Edge, 1));
	TestEqual(TEXT("The collision-pass budget is explicit"), Result.Status, CableSim::ETautStatus::IterationBudgetExceeded);
	TestTrue(TEXT("Budget exhaustion preserves the last valid path"),
		CableSimTautTests::PointsEqual(InitialState.Points, Solver.GetPoints()));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimTautReachLimitTest,
	"CableSim.Core.Taut.ReachLimit",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimTautReachLimitTest::RunTest(const FString& Parameters)
{
	CableSim::FTautPathSolver StraightSolver;
	StraightSolver.Initialize(FVector3d::ZeroVector, FVector3d(20.0, 0.0, 0.0));
	FVector3d Reachable;
	double Excess = 0.0;
	TestTrue(TEXT("Straight reach limit is solvable"),
		StraightSolver.CalculateReachableEndpoint(false, 10.0, Reachable, Excess));
	TestTrue(TEXT("Straight endpoint is clamped along its final segment"),
		Reachable.Equals(FVector3d(10.0, 0.0, 0.0), 1.e-9));
	TestTrue(TEXT("Straight excess is reported"), FMath::IsNearlyEqual(Excess, 10.0, 1.e-9));

	const CableSim::FCollisionEdge Edge = CableSimTautTests::MakeVerticalEdge();
	CableSim::FTautPathSolver WrappedSolver;
	WrappedSolver.Initialize(FVector3d(10.0, 10.0, 0.0), FVector3d(-10.0, 10.0, 0.0));
	WrappedSolver.AdvanceStep(
		CableSimTautTests::MakeInput(FVector3d(10.0, -10.0, 0.0), FVector3d(-10.0, 10.0, 0.0)),
		MakeArrayView(&Edge, 1));
	TestTrue(TEXT("Wrapped reach limit is solvable"),
		WrappedSolver.CalculateReachableEndpoint(false, 20.0, Reachable, Excess));
	const TArray<CableSim::FTautPoint>& Points = WrappedSolver.GetPoints();
	const double LimitedLength = FVector3d::Distance(Points[0].Position, Points[1].Position)
		+ FVector3d::Distance(Points[1].Position, Reachable);
	TestTrue(TEXT("Wrapped reachable path respects length"), FMath::IsNearlyEqual(LimitedLength, 20.0, 1.e-9));
	TestFalse(TEXT("Path prefix longer than cable is rejected"),
		WrappedSolver.CalculateReachableEndpoint(false, 5.0, Reachable, Excess));
	return true;
}

#endif
