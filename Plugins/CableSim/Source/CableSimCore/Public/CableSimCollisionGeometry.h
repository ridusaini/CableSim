#pragma once

#include "CoreMinimal.h"

namespace CableSim
{
	enum class ECollisionFeatureType : uint8
	{
		None,
		Vertex,
		Triangle,
		Edge
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

	enum class ECollisionGeometryType : uint8
	{
		Unknown,
		Box,
		Convex,
		TriangleMesh,
		HeightField
	};

	struct CABLESIMCORE_API FCollisionFeatureId
	{
		uint64 ObjectToken = 0;
		int32 ShapeIndex = INDEX_NONE;
		ECollisionFeatureType Type = ECollisionFeatureType::None;
		int32 Index0 = INDEX_NONE;
		int32 Index1 = INDEX_NONE;

		bool operator==(const FCollisionFeatureId& Other) const = default;
		bool IsValid() const;
		static bool Less(const FCollisionFeatureId& First, const FCollisionFeatureId& Second);
	};

	CABLESIMCORE_API uint32 GetTypeHash(const FCollisionFeatureId& FeatureId);

	struct CABLESIMCORE_API FCollisionTriangle
	{
		FCollisionFeatureId Id;
		ECollisionGeometryType GeometryType = ECollisionGeometryType::Unknown;
		bool bStaticObject = false;
		// World velocity of the owning surface at this triangle (zero for static
		// objects); consumed by friction so contacts move with a moving surface.
		FVector3d SurfaceVelocity = FVector3d::ZeroVector;
		FVector3d Vertices[3] = {
			FVector3d::ZeroVector,
			FVector3d::ZeroVector,
			FVector3d::ZeroVector};
		int32 VertexIndices[3] = {INDEX_NONE, INDEX_NONE, INDEX_NONE};

		FVector3d CalculateNormal() const;
		bool IsFinite() const;
	};

	struct CABLESIMCORE_API FCollisionEdge
	{
		FCollisionFeatureId Id;
		ECollisionGeometryType GeometryType = ECollisionGeometryType::Unknown;
		bool bStaticObject = false;
		FVector3d Start = FVector3d::ZeroVector;
		FVector3d End = FVector3d::ZeroVector;
		FVector3d FaceNormal0 = FVector3d::ZeroVector;
		FVector3d FaceNormal1 = FVector3d::ZeroVector;
		// Inherited from the edge's first incident triangle; see FCollisionTriangle.
		FVector3d SurfaceVelocity = FVector3d::ZeroVector;
		ECollisionEdgeKind Kind = ECollisionEdgeKind::Degenerate;

		bool IsFinite() const;
	};

	struct CABLESIMCORE_API FCollisionVertex
	{
		FCollisionFeatureId Id;
		ECollisionGeometryType GeometryType = ECollisionGeometryType::Unknown;
		bool bStaticObject = false;
		FVector3d Position = FVector3d::ZeroVector;
		TArray<FCollisionFeatureId> IncidentEdges;
		bool bNonManifold = false;
		bool bOverValence = false;

		bool IsFinite() const;
	};

	struct CABLESIMCORE_API FTopologyCompileDiagnostics
	{
		int32 InputTriangleCount = 0;
		int32 ConvexEdgeCount = 0;
		int32 ConcaveEdgeCount = 0;
		int32 CoplanarEdgeCount = 0;
		int32 BoundaryEdgeCount = 0;
		int32 DegenerateEdgeCount = 0;
		int32 NonManifoldEdgeCount = 0;
		int32 VertexCount = 0;
		int32 OverValenceVertexCount = 0;
	};

	class CABLESIMCORE_API FCollisionTopologyCompiler
	{
	public:
		static FTopologyCompileDiagnostics CompileEdges(
			TConstArrayView<FCollisionTriangle> Triangles,
			double Tolerance,
			TArray<FCollisionEdge>& OutEdges);
		static FTopologyCompileDiagnostics CompileTopology(
			TConstArrayView<FCollisionTriangle> Triangles,
			double Tolerance,
			int32 MaximumIncidentEdges,
			TArray<FCollisionEdge>& OutEdges,
			TArray<FCollisionVertex>& OutVertices);
	};
}
