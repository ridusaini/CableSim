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
			int32 TriangleArrayIndex = INDEX_NONE;
			int32 OppositeVertex = INDEX_NONE;
		};

		struct FPendingEdge
		{
			FEdgeKey Key;
			TArray<FIncidentFace, TInlineAllocator<3>> Faces;
		};
	}

	bool FCollisionFeatureId::IsValid() const
	{
		return ObjectToken != 0 && ShapeIndex != INDEX_NONE && Type != ECollisionFeatureType::None
			&& Index0 != INDEX_NONE;
	}

	uint32 GetTypeHash(const FCollisionFeatureId& FeatureId)
	{
		uint32 Hash = ::GetTypeHash(FeatureId.ObjectToken);
		Hash = HashCombineFast(Hash, ::GetTypeHash(FeatureId.ShapeIndex));
		Hash = HashCombineFast(Hash, ::GetTypeHash(static_cast<uint8>(FeatureId.Type)));
		Hash = HashCombineFast(Hash, ::GetTypeHash(FeatureId.Index0));
		return HashCombineFast(Hash, ::GetTypeHash(FeatureId.Index1));
	}

	bool FCollisionFeatureId::Less(
		const FCollisionFeatureId& First,
		const FCollisionFeatureId& Second)
	{
		if (First.ObjectToken != Second.ObjectToken)
		{
			return First.ObjectToken < Second.ObjectToken;
		}
		if (First.ShapeIndex != Second.ShapeIndex)
		{
			return First.ShapeIndex < Second.ShapeIndex;
		}
		if (First.Type != Second.Type)
		{
			return static_cast<uint8>(First.Type) < static_cast<uint8>(Second.Type);
		}
		return First.Index0 != Second.Index0
			? First.Index0 < Second.Index0
			: First.Index1 < Second.Index1;
	}

	FVector3d FCollisionTriangle::CalculateNormal() const
	{
		return FVector3d::CrossProduct(Vertices[1] - Vertices[0], Vertices[2] - Vertices[0]).GetSafeNormal();
	}

	bool FCollisionTriangle::IsFinite() const
	{
		return IsFiniteVector(Vertices[0]) && IsFiniteVector(Vertices[1]) && IsFiniteVector(Vertices[2]);
	}

	bool FCollisionEdge::IsFinite() const
	{
		return Id.IsValid() && IsFiniteVector(Start) && IsFiniteVector(End)
			&& IsFiniteVector(FaceNormal0) && IsFiniteVector(FaceNormal1);
	}

	bool FCollisionVertex::IsFinite() const
	{
		return Id.IsValid() && IsFiniteVector(Position);
	}

	FTopologyCompileDiagnostics FCollisionTopologyCompiler::CompileEdges(
		const TConstArrayView<FCollisionTriangle> Triangles,
		const double Tolerance,
		TArray<FCollisionEdge>& OutEdges)
	{
		OutEdges.Reset();
		FTopologyCompileDiagnostics Diagnostics;
		Diagnostics.InputTriangleCount = Triangles.Num();
		const double SafeTolerance = FMath::Max(Tolerance, 1.e-9);

		TMap<FEdgeKey, FPendingEdge> PendingEdges;
		PendingEdges.Reserve(Triangles.Num() * 3);
		for (int32 TriangleArrayIndex = 0; TriangleArrayIndex < Triangles.Num(); ++TriangleArrayIndex)
		{
			const FCollisionTriangle& Triangle = Triangles[TriangleArrayIndex];
			if (!Triangle.IsFinite() || Triangle.CalculateNormal().IsNearlyZero())
			{
				Diagnostics.DegenerateEdgeCount += 3;
				continue;
			}
			for (int32 Side = 0; Side < 3; ++Side)
			{
				const int32 FirstCorner = Side;
				const int32 SecondCorner = (Side + 1) % 3;
				const int32 OppositeCorner = (Side + 2) % 3;
				const int32 FirstVertex = Triangle.VertexIndices[FirstCorner];
				const int32 SecondVertex = Triangle.VertexIndices[SecondCorner];
				if (FirstVertex == INDEX_NONE || SecondVertex == INDEX_NONE || FirstVertex == SecondVertex
					|| FVector3d::Distance(Triangle.Vertices[FirstCorner], Triangle.Vertices[SecondCorner]) <= SafeTolerance)
				{
					++Diagnostics.DegenerateEdgeCount;
					continue;
				}

				FEdgeKey Key;
				Key.ObjectToken = Triangle.Id.ObjectToken;
				Key.ShapeIndex = Triangle.Id.ShapeIndex;
				Key.Vertex0 = FMath::Min(FirstVertex, SecondVertex);
				Key.Vertex1 = FMath::Max(FirstVertex, SecondVertex);
				FPendingEdge& Pending = PendingEdges.FindOrAdd(Key);
				Pending.Key = Key;
				Pending.Faces.Add({TriangleArrayIndex, OppositeCorner});
			}
		}

		TArray<FPendingEdge> OrderedEdges;
		PendingEdges.GenerateValueArray(OrderedEdges);
		OrderedEdges.Sort([](const FPendingEdge& First, const FPendingEdge& Second)
		{
			const FCollisionFeatureId FirstId{
				First.Key.ObjectToken,
				First.Key.ShapeIndex,
				ECollisionFeatureType::Edge,
				First.Key.Vertex0,
				First.Key.Vertex1};
			const FCollisionFeatureId SecondId{
				Second.Key.ObjectToken,
				Second.Key.ShapeIndex,
				ECollisionFeatureType::Edge,
				Second.Key.Vertex0,
				Second.Key.Vertex1};
			return FCollisionFeatureId::Less(FirstId, SecondId);
		});

		for (FPendingEdge& Pending : OrderedEdges)
		{
			Pending.Faces.Sort([&Triangles](const FIncidentFace& First, const FIncidentFace& Second)
			{
				return FCollisionFeatureId::Less(
					Triangles[First.TriangleArrayIndex].Id,
					Triangles[Second.TriangleArrayIndex].Id);
			});
			const FIncidentFace& FirstFace = Pending.Faces[0];
			const FCollisionTriangle& FirstTriangle = Triangles[FirstFace.TriangleArrayIndex];
			int32 FirstCorner = INDEX_NONE;
			int32 SecondCorner = INDEX_NONE;
			for (int32 Corner = 0; Corner < 3; ++Corner)
			{
				if (FirstTriangle.VertexIndices[Corner] == Pending.Key.Vertex0)
				{
					FirstCorner = Corner;
				}
				else if (FirstTriangle.VertexIndices[Corner] == Pending.Key.Vertex1)
				{
					SecondCorner = Corner;
				}
			}
			if (FirstCorner == INDEX_NONE || SecondCorner == INDEX_NONE)
			{
				++Diagnostics.DegenerateEdgeCount;
				continue;
			}

			FCollisionEdge Edge;
			Edge.Id = {
				Pending.Key.ObjectToken,
				Pending.Key.ShapeIndex,
				ECollisionFeatureType::Edge,
				Pending.Key.Vertex0,
				Pending.Key.Vertex1};
			Edge.Start = FirstTriangle.Vertices[FirstCorner];
			Edge.End = FirstTriangle.Vertices[SecondCorner];
			Edge.GeometryType = FirstTriangle.GeometryType;
			Edge.bStaticObject = FirstTriangle.bStaticObject;
			Edge.FaceNormal0 = FirstTriangle.CalculateNormal();

			if (Pending.Faces.Num() == 1)
			{
				Edge.Kind = ECollisionEdgeKind::Boundary;
				++Diagnostics.BoundaryEdgeCount;
			}
			else if (Pending.Faces.Num() > 2)
			{
				Edge.Kind = ECollisionEdgeKind::NonManifold;
				++Diagnostics.NonManifoldEdgeCount;
			}
			else
			{
				const FIncidentFace& SecondFace = Pending.Faces[1];
				const FCollisionTriangle& SecondTriangle = Triangles[SecondFace.TriangleArrayIndex];
				Edge.FaceNormal1 = SecondTriangle.CalculateNormal();
				const double SecondBehindFirst = FVector3d::DotProduct(
					Edge.FaceNormal0,
					SecondTriangle.Vertices[SecondFace.OppositeVertex] - Edge.Start);
				const double FirstBehindSecond = FVector3d::DotProduct(
					Edge.FaceNormal1,
					FirstTriangle.Vertices[FirstFace.OppositeVertex] - Edge.Start);
				if (FMath::Abs(SecondBehindFirst) <= SafeTolerance
					&& FMath::Abs(FirstBehindSecond) <= SafeTolerance)
				{
					Edge.Kind = ECollisionEdgeKind::Coplanar;
					++Diagnostics.CoplanarEdgeCount;
				}
				else if (SecondBehindFirst < -SafeTolerance && FirstBehindSecond < -SafeTolerance)
				{
					Edge.Kind = ECollisionEdgeKind::Convex;
					++Diagnostics.ConvexEdgeCount;
				}
				else
				{
					Edge.Kind = ECollisionEdgeKind::Concave;
					++Diagnostics.ConcaveEdgeCount;
				}
			}
			OutEdges.Add(MoveTemp(Edge));
		}
		return Diagnostics;
	}

	FTopologyCompileDiagnostics FCollisionTopologyCompiler::CompileTopology(
		const TConstArrayView<FCollisionTriangle> Triangles,
		const double Tolerance,
		const int32 MaximumIncidentEdges,
		TArray<FCollisionEdge>& OutEdges,
		TArray<FCollisionVertex>& OutVertices)
	{
		FTopologyCompileDiagnostics Diagnostics = CompileEdges(Triangles, Tolerance, OutEdges);
		OutVertices.Reset();
		const int32 SafeMaximumIncidentEdges = FMath::Clamp(MaximumIncidentEdges, 1, 8);
		TMap<FCollisionFeatureId, FCollisionVertex> Vertices;
		for (const FCollisionEdge& Edge : OutEdges)
		{
			for (const bool bStart : {true, false})
			{
				FCollisionFeatureId VertexId{
					Edge.Id.ObjectToken,
					Edge.Id.ShapeIndex,
					ECollisionFeatureType::Vertex,
					bStart ? Edge.Id.Index0 : Edge.Id.Index1,
					INDEX_NONE};
				FCollisionVertex& Vertex = Vertices.FindOrAdd(VertexId);
				Vertex.Id = VertexId;
				Vertex.GeometryType = Edge.GeometryType;
				Vertex.bStaticObject = Edge.bStaticObject;
				Vertex.Position = bStart ? Edge.Start : Edge.End;
				Vertex.IncidentEdges.Add(Edge.Id);
				Vertex.bNonManifold |= Edge.Kind == ECollisionEdgeKind::NonManifold;
			}
		}
		Vertices.GenerateValueArray(OutVertices);
		OutVertices.Sort([](const FCollisionVertex& First, const FCollisionVertex& Second)
		{
			return FCollisionFeatureId::Less(First.Id, Second.Id);
		});
		for (FCollisionVertex& Vertex : OutVertices)
		{
			Vertex.IncidentEdges.Sort([](const FCollisionFeatureId& First, const FCollisionFeatureId& Second)
			{
				return FCollisionFeatureId::Less(First, Second);
			});
			Vertex.bOverValence = Vertex.IncidentEdges.Num() > SafeMaximumIncidentEdges;
			Diagnostics.OverValenceVertexCount += Vertex.bOverValence ? 1 : 0;
		}
		Diagnostics.VertexCount = OutVertices.Num();
		return Diagnostics;
	}
}
