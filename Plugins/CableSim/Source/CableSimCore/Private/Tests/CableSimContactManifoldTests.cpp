#if WITH_DEV_AUTOMATION_TESTS

#include "CableSimContactManifold.h"
#include "Misc/AutomationTest.h"

namespace CableSimTests
{
	CableSim::FCollisionTriangle MakeTriangle(
		const uint64 ObjectToken,
		const int32 TriangleIndex,
		const FVector3d& V0,
		const FVector3d& V1,
		const FVector3d& V2,
		const bool bStatic = true)
	{
		CableSim::FCollisionTriangle Triangle;
		Triangle.Id = {ObjectToken, 0, CableSim::ECollisionFeatureType::Triangle, TriangleIndex, INDEX_NONE};
		Triangle.GeometryType = CableSim::ECollisionGeometryType::TriangleMesh;
		Triangle.bStaticObject = bStatic;
		Triangle.Vertices[0] = V0;
		Triangle.Vertices[1] = V1;
		Triangle.Vertices[2] = V2;
		Triangle.VertexIndices[0] = 0;
		Triangle.VertexIndices[1] = 1;
		Triangle.VertexIndices[2] = 2;
		return Triangle;
	}

	CableSim::FCollisionEdge MakeConvexEdge(
		const uint64 ObjectToken,
		const int32 EdgeIndex,
		const FVector3d& Start,
		const FVector3d& End,
		const FVector3d& FaceNormal0,
		const FVector3d& FaceNormal1)
	{
		CableSim::FCollisionEdge Edge;
		Edge.Id = {ObjectToken, 0, CableSim::ECollisionFeatureType::Edge, EdgeIndex, INDEX_NONE};
		Edge.GeometryType = CableSim::ECollisionGeometryType::Box;
		Edge.bStaticObject = true;
		Edge.Start = Start;
		Edge.End = End;
		Edge.FaceNormal0 = FaceNormal0;
		Edge.FaceNormal1 = FaceNormal1;
		Edge.Kind = CableSim::ECollisionEdgeKind::Convex;
		return Edge;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimManifoldSinglePlaneTest,
	"CableSim.Core.Manifold.SinglePlane",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimManifoldSinglePlaneTest::RunTest(const FString& Parameters)
{
	TArray<CableSim::FCollisionTriangle> Triangles;
	Triangles.Add(CableSimTests::MakeTriangle(1, 0,
		FVector3d(10.0, -10.0, 0.0), FVector3d(10.0, 10.0, 0.0), FVector3d(-10.0, 0.0, 0.0)));
	CableSim::FManifoldConfig Config;
	TArray<CableSim::FContactConstraint> Contacts;
	CableSim::FContactManifoldCompiler::CompileNodeContacts(
		1, FVector3d(0.0, 0.0, 3.0), FVector3d(0.0, 0.0, 3.0), Triangles, {}, Config, Contacts);
	TestEqual(TEXT("One plane emitted"), Contacts.Num(), 1);
	if (Contacts.Num() == 1)
	{
		TestTrue(TEXT("Upward normal"), Contacts[0].Normal.Equals(FVector3d::UnitZ(), 1.e-9));
		TestTrue(TEXT("Plane offset includes node radius"),
			FMath::IsNearlyEqual(Contacts[0].MinimumNormalCoordinate, Config.NodeRadius, 1.e-9));
		TestTrue(TEXT("Friction anchor set"), Contacts[0].bHasFrictionAnchor);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimManifoldCoplanarMergeTest,
	"CableSim.Core.Manifold.CoplanarMerge",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimManifoldCoplanarMergeTest::RunTest(const FString& Parameters)
{
	TArray<CableSim::FCollisionTriangle> Triangles;
	Triangles.Add(CableSimTests::MakeTriangle(1, 0,
		FVector3d(-10.0, -10.0, 0.0), FVector3d(10.0, -10.0, 0.0), FVector3d(10.0, 10.0, 0.0)));
	Triangles.Add(CableSimTests::MakeTriangle(1, 1,
		FVector3d(-10.0, -10.0, 0.0), FVector3d(10.0, 10.0, 0.0), FVector3d(-10.0, 10.0, 0.0)));
	CableSim::FManifoldConfig Config;
	TArray<CableSim::FContactConstraint> Contacts;
	CableSim::FContactManifoldCompiler::CompileNodeContacts(
		2, FVector3d(0.0, 0.0, 2.0), FVector3d(0.0, 0.0, 2.0), Triangles, {}, Config, Contacts);
	TestEqual(TEXT("Coplanar quad merges to one plane"), Contacts.Num(), 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimManifoldConvexCornerTest,
	"CableSim.Core.Manifold.ConvexCornerKeepsBothPlanes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimManifoldConvexCornerTest::RunTest(const FString& Parameters)
{
	TArray<CableSim::FCollisionTriangle> Triangles;
	Triangles.Add(CableSimTests::MakeTriangle(1, 0,
		FVector3d(10.0, -10.0, 0.0), FVector3d(10.0, 10.0, 0.0), FVector3d(-10.0, 0.0, 0.0)));
	Triangles.Add(CableSimTests::MakeTriangle(1, 1,
		FVector3d(0.0, -10.0, -10.0), FVector3d(0.0, 10.0, -10.0), FVector3d(0.0, 0.0, 10.0)));
	CableSim::FManifoldConfig Config;
	TArray<CableSim::FContactConstraint> Contacts;
	CableSim::FContactManifoldCompiler::CompileNodeContacts(
		0, FVector3d(2.0, 0.0, 2.0), FVector3d(2.0, 0.0, 2.0), Triangles, {}, Config, Contacts);
	TestEqual(TEXT("Two non-parallel planes retained"), Contacts.Num(), 2);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimManifoldFarNoContactTest,
	"CableSim.Core.Manifold.FarNodeHasNoContact",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimManifoldFarNoContactTest::RunTest(const FString& Parameters)
{
	TArray<CableSim::FCollisionTriangle> Triangles;
	Triangles.Add(CableSimTests::MakeTriangle(1, 0,
		FVector3d(10.0, -10.0, 0.0), FVector3d(10.0, 10.0, 0.0), FVector3d(-10.0, 0.0, 0.0)));
	CableSim::FManifoldConfig Config;
	TArray<CableSim::FContactConstraint> Contacts;
	CableSim::FContactManifoldCompiler::CompileNodeContacts(
		0, FVector3d(0.0, 0.0, 100.0), FVector3d(0.0, 0.0, 100.0), Triangles, {}, Config, Contacts);
	TestEqual(TEXT("Distant node gathers no contact"), Contacts.Num(), 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimManifoldStableFeatureIdTest,
	"CableSim.Core.Manifold.StableFeatureIdAcrossSteps",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimManifoldStableFeatureIdTest::RunTest(const FString& Parameters)
{
	TArray<CableSim::FCollisionTriangle> Triangles;
	Triangles.Add(CableSimTests::MakeTriangle(7, 3,
		FVector3d(10.0, -10.0, 0.0), FVector3d(10.0, 10.0, 0.0), FVector3d(-10.0, 0.0, 0.0)));
	CableSim::FManifoldConfig Config;
	TArray<CableSim::FContactConstraint> First;
	TArray<CableSim::FContactConstraint> Second;
	CableSim::FContactManifoldCompiler::CompileNodeContacts(
		0, FVector3d(0.0, 0.0, 3.0), FVector3d(0.0, 0.0, 3.1), Triangles, {}, Config, First);
	CableSim::FContactManifoldCompiler::CompileNodeContacts(
		0, FVector3d(0.2, 0.1, 2.9), FVector3d(0.0, 0.0, 3.0), Triangles, {}, Config, Second);
	TestEqual(TEXT("Both steps produce a contact"), First.Num(), Second.Num());
	if (First.Num() == 1 && Second.Num() == 1)
	{
		TestEqual(TEXT("Feature id is stable across steps"), First[0].FeatureId, Second[0].FeatureId);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimManifoldPlaneCapTest,
	"CableSim.Core.Manifold.PlaneCountIsCapped",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimManifoldPlaneCapTest::RunTest(const FString& Parameters)
{
	TArray<CableSim::FCollisionTriangle> Triangles;
	Triangles.Add(CableSimTests::MakeTriangle(1, 0,
		FVector3d(10.0, -10.0, 0.0), FVector3d(10.0, 10.0, 0.0), FVector3d(-10.0, 0.0, 0.0)));
	Triangles.Add(CableSimTests::MakeTriangle(1, 1,
		FVector3d(0.0, -10.0, -10.0), FVector3d(0.0, 10.0, -10.0), FVector3d(0.0, 0.0, 10.0)));
	Triangles.Add(CableSimTests::MakeTriangle(1, 2,
		FVector3d(-10.0, 0.0, -10.0), FVector3d(10.0, 0.0, -10.0), FVector3d(0.0, 0.0, 10.0)));
	CableSim::FManifoldConfig Config;
	Config.MaxPlanesPerNode = 2;
	TArray<CableSim::FContactConstraint> Contacts;
	CableSim::FContactManifoldCompiler::CompileNodeContacts(
		0, FVector3d(1.0, 1.0, 1.0), FVector3d(1.0, 1.0, 1.0), Triangles, {}, Config, Contacts);
	TestTrue(TEXT("Plane count respects the cap"), Contacts.Num() <= 2);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimManifoldConvexEdgeRadialTest,
	"CableSim.Core.Manifold.ConvexEdgeRadialContact",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimManifoldConvexEdgeRadialTest::RunTest(const FString& Parameters)
{
	// Box top edge: top face (+Z), side face (+X), sharing an edge along Y at origin.
	// A node in the exterior wedge must get ONE radial edge contact, both faces suppressed.
	TArray<CableSim::FCollisionTriangle> Triangles;
	Triangles.Add(CableSimTests::MakeTriangle(1, 0,
		FVector3d(0.0, -10.0, 0.0), FVector3d(0.0, 10.0, 0.0), FVector3d(-10.0, 0.0, 0.0)));
	Triangles.Add(CableSimTests::MakeTriangle(1, 1,
		FVector3d(0.0, 10.0, 0.0), FVector3d(0.0, -10.0, 0.0), FVector3d(0.0, 0.0, -10.0)));
	TArray<CableSim::FCollisionEdge> Edges;
	Edges.Add(CableSimTests::MakeConvexEdge(1, 0,
		FVector3d(0.0, -10.0, 0.0), FVector3d(0.0, 10.0, 0.0), FVector3d::UnitZ(), FVector3d::UnitX()));
	CableSim::FManifoldConfig Config;
	TArray<CableSim::FContactConstraint> Contacts;
	CableSim::FContactManifoldCompiler::CompileNodeContacts(
		0, FVector3d(2.0, 0.0, 2.0), FVector3d(2.0, 0.0, 2.0), Triangles, Edges, Config, Contacts);
	TestEqual(TEXT("Edge yields a single contact, faces suppressed"), Contacts.Num(), 1);
	if (Contacts.Num() == 1)
	{
		TestTrue(TEXT("Contact is a convex-edge contact"), Contacts[0].bConvexEdge);
		TestTrue(TEXT("Edge radius is the node radius"),
			FMath::IsNearlyEqual(Contacts[0].EdgeRadius, Config.NodeRadius, 1.e-9));
		TestTrue(TEXT("Edge geometry carried through"),
			Contacts[0].EdgeStart.Equals(FVector3d(0.0, -10.0, 0.0), 1.e-6)
				&& Contacts[0].EdgeEnd.Equals(FVector3d(0.0, 10.0, 0.0), 1.e-6));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimManifoldOverFaceUsesPlaneTest,
	"CableSim.Core.Manifold.OverFaceUsesPlaneNotEdge",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimManifoldOverFaceUsesPlaneTest::RunTest(const FString& Parameters)
{
	// A node well over the top face (not in the edge wedge) uses the face plane.
	TArray<CableSim::FCollisionTriangle> Triangles;
	Triangles.Add(CableSimTests::MakeTriangle(1, 0,
		FVector3d(0.0, -10.0, 0.0), FVector3d(0.0, 10.0, 0.0), FVector3d(-10.0, 0.0, 0.0)));
	TArray<CableSim::FCollisionEdge> Edges;
	Edges.Add(CableSimTests::MakeConvexEdge(1, 0,
		FVector3d(0.0, -10.0, 0.0), FVector3d(0.0, 10.0, 0.0), FVector3d::UnitZ(), FVector3d::UnitX()));
	CableSim::FManifoldConfig Config;
	TArray<CableSim::FContactConstraint> Contacts;
	CableSim::FContactManifoldCompiler::CompileNodeContacts(
		0, FVector3d(-8.0, 0.0, 2.0), FVector3d(-8.0, 0.0, 2.0), Triangles, Edges, Config, Contacts);
	TestEqual(TEXT("Over-face node gets one plane"), Contacts.Num(), 1);
	if (Contacts.Num() == 1)
	{
		TestTrue(TEXT("Plane normal is the face normal"), Contacts[0].Normal.Equals(FVector3d::UnitZ(), 1.e-6));
	}
	return true;
}

#endif
