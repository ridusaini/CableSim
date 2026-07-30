#include "CableSimCollisionGeometry.h"

namespace CableSim
{
	namespace
	{
		bool IsFiniteVector(const FVector3d& Value)
		{
			return FMath::IsFinite(Value.X) && FMath::IsFinite(Value.Y) && FMath::IsFinite(Value.Z);
		}

		struct FEdgeKey
		{
			uint64 ObjectToken = 0;
			int32 ShapeIndex = INDEX_NONE;
			int32 Vertex0 = INDEX_NONE;
			int32 Vertex1 = INDEX_NONE;
			bool operator==(const FEdgeKey& Other) const = default;
		};

		uint32 GetTypeHash(const FEdgeKey& Key)
		{
			uint32 Hash = ::GetTypeHash(Key.ObjectToken);
			Hash = HashCombineFast(Hash, ::GetTypeHash(Key.ShapeIndex));
			Hash = HashCombineFast(Hash, ::GetTypeHash(Key.Vertex0));
			return HashCombineFast(Hash, ::GetTypeHash(Key.Vertex1));
		}

		struct FIncidentFace
		{
			int32 Triangle = INDEX_NONE;
			int32 OppositeCorner = INDEX_NONE;
		};

		struct FPendingEdge
		{
			FEdgeKey Key;
			TArray<FIncidentFace, TInlineAllocator<3>> Faces;
		};
	}

	FVector3d FCollisionTriangle::CalculateNormal() const
	{
		return FVector3d::CrossProduct(Vertices[1] - Vertices[0], Vertices[2] - Vertices[0]).GetSafeNormal();
	}

	bool FCollisionTriangle::IsFinite() const
	{
		return Id.IsValid() && IsFiniteVector(Vertices[0]) && IsFiniteVector(Vertices[1]) && IsFiniteVector(Vertices[2]);
	}

	FTopologyDiagnostics FCollisionTopologyCompiler::CompileEdges(
		const TConstArrayView<FCollisionTriangle> Triangles,
		const double Tolerance,
		TArray<FCollisionEdge>& OutEdges)
	{
		OutEdges.Reset();
		FTopologyDiagnostics Diagnostics;
		const double SafeTolerance = FMath::Max(Tolerance, 1.e-9);
		TMap<FEdgeKey, FPendingEdge> Pending;
		for (int32 TriangleIndex = 0; TriangleIndex < Triangles.Num(); ++TriangleIndex)
		{
			const FCollisionTriangle& Triangle = Triangles[TriangleIndex];
			if (!Triangle.IsFinite() || Triangle.CalculateNormal().IsNearlyZero())
			{
				Diagnostics.DegenerateEdges += 3;
				continue;
			}
			for (int32 Side = 0; Side < 3; ++Side)
			{
				const int32 First = Side;
				const int32 Second = (Side + 1) % 3;
				const int32 Opposite = (Side + 2) % 3;
				const int32 V0 = Triangle.VertexIndices[First];
				const int32 V1 = Triangle.VertexIndices[Second];
				if (V0 == INDEX_NONE || V1 == INDEX_NONE || V0 == V1
					|| FVector3d::Distance(Triangle.Vertices[First], Triangle.Vertices[Second]) <= SafeTolerance)
				{
					++Diagnostics.DegenerateEdges;
					continue;
				}
				FEdgeKey Key{Triangle.Id.ObjectToken, Triangle.Id.ShapeIndex, FMath::Min(V0, V1), FMath::Max(V0, V1)};
				FPendingEdge& Edge = Pending.FindOrAdd(Key);
				Edge.Key = Key;
				Edge.Faces.Add({TriangleIndex, Opposite});
			}
		}

		TArray<FPendingEdge> Ordered;
		Pending.GenerateValueArray(Ordered);
		Ordered.Sort([](const FPendingEdge& A, const FPendingEdge& B)
		{
			return FCollisionFeatureId::Less(
				{A.Key.ObjectToken, A.Key.ShapeIndex, ECollisionFeatureType::Edge, A.Key.Vertex0, A.Key.Vertex1},
				{B.Key.ObjectToken, B.Key.ShapeIndex, ECollisionFeatureType::Edge, B.Key.Vertex0, B.Key.Vertex1});
		});

		for (const FPendingEdge& PendingEdge : Ordered)
		{
			const FCollisionTriangle& FirstTriangle = Triangles[PendingEdge.Faces[0].Triangle];
			int32 Corner0 = INDEX_NONE;
			int32 Corner1 = INDEX_NONE;
			for (int32 Corner = 0; Corner < 3; ++Corner)
			{
				if (FirstTriangle.VertexIndices[Corner] == PendingEdge.Key.Vertex0) Corner0 = Corner;
				if (FirstTriangle.VertexIndices[Corner] == PendingEdge.Key.Vertex1) Corner1 = Corner;
			}
			if (Corner0 == INDEX_NONE || Corner1 == INDEX_NONE) { ++Diagnostics.DegenerateEdges; continue; }
			FCollisionEdge Edge;
			Edge.Id = {PendingEdge.Key.ObjectToken, PendingEdge.Key.ShapeIndex, ECollisionFeatureType::Edge,
				PendingEdge.Key.Vertex0, PendingEdge.Key.Vertex1};
			Edge.GeometryType = FirstTriangle.GeometryType;
			Edge.Start = FirstTriangle.Vertices[Corner0];
			Edge.End = FirstTriangle.Vertices[Corner1];
			Edge.FaceNormal0 = FirstTriangle.CalculateNormal();
			if (PendingEdge.Faces.Num() == 1)
			{
				Edge.Kind = ECollisionEdgeKind::Boundary;
				++Diagnostics.BoundaryEdges;
			}
			else if (PendingEdge.Faces.Num() > 2)
			{
				Edge.Kind = ECollisionEdgeKind::NonManifold;
				++Diagnostics.NonManifoldEdges;
			}
			else
			{
				const FIncidentFace& SecondFace = PendingEdge.Faces[1];
				const FCollisionTriangle& SecondTriangle = Triangles[SecondFace.Triangle];
				Edge.FaceNormal1 = SecondTriangle.CalculateNormal();
				const double SecondBehindFirst = FVector3d::DotProduct(
					Edge.FaceNormal0, SecondTriangle.Vertices[SecondFace.OppositeCorner] - Edge.Start);
				const FIncidentFace& FirstFace = PendingEdge.Faces[0];
				const double FirstBehindSecond = FVector3d::DotProduct(
					Edge.FaceNormal1, FirstTriangle.Vertices[FirstFace.OppositeCorner] - Edge.Start);
				if (FMath::Abs(SecondBehindFirst) <= SafeTolerance && FMath::Abs(FirstBehindSecond) <= SafeTolerance)
				{
					Edge.Kind = ECollisionEdgeKind::Coplanar;
					++Diagnostics.CoplanarEdges;
				}
				else if (SecondBehindFirst < -SafeTolerance && FirstBehindSecond < -SafeTolerance)
				{
					Edge.Kind = ECollisionEdgeKind::Convex;
					++Diagnostics.ConvexEdges;
				}
				else
				{
					Edge.Kind = ECollisionEdgeKind::Concave;
					++Diagnostics.ConcaveEdges;
				}
			}
			OutEdges.Add(MoveTemp(Edge));
		}
		return Diagnostics;
	}
}
