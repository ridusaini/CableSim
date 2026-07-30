#pragma once

#include "CableSimSolver.h"

namespace CableSim
{
	enum class ECollisionGeometryType : uint8
	{
		Unknown,
		Box,
		Convex,
		Sphere,
		Capsule,
		TriangleMesh,
		HeightField
	};

	enum class ECollisionEdgeKind : uint8
	{
		Convex,
		Concave,
		Coplanar,
		Boundary,
		Degenerate,
		NonManifold
	};

	struct CABLESIMCORE_API FCollisionTriangle
	{
		FCollisionFeatureId Id;
		ECollisionGeometryType GeometryType = ECollisionGeometryType::Unknown;
		FVector3d Vertices[3] = {FVector3d::ZeroVector, FVector3d::ZeroVector, FVector3d::ZeroVector};
		int32 VertexIndices[3] = {INDEX_NONE, INDEX_NONE, INDEX_NONE};

		FVector3d CalculateNormal() const;
		bool IsFinite() const;
	};

	struct CABLESIMCORE_API FCollisionEdge
	{
		FCollisionFeatureId Id;
		ECollisionGeometryType GeometryType = ECollisionGeometryType::Unknown;
		FVector3d Start = FVector3d::ZeroVector;
		FVector3d End = FVector3d::ZeroVector;
		FVector3d FaceNormal0 = FVector3d::ZeroVector;
		FVector3d FaceNormal1 = FVector3d::ZeroVector;
		ECollisionEdgeKind Kind = ECollisionEdgeKind::Degenerate;
	};

	struct CABLESIMCORE_API FTopologyDiagnostics
	{
		int32 ConvexEdges = 0;
		int32 ConcaveEdges = 0;
		int32 CoplanarEdges = 0;
		int32 BoundaryEdges = 0;
		int32 DegenerateEdges = 0;
		int32 NonManifoldEdges = 0;
	};

	class CABLESIMCORE_API FCollisionTopologyCompiler
	{
	public:
		static FTopologyDiagnostics CompileEdges(
			TConstArrayView<FCollisionTriangle> Triangles,
			double Tolerance,
			TArray<FCollisionEdge>& OutEdges);
	};
}
