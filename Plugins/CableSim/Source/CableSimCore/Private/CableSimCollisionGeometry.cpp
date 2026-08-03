#include "CableSimCollisionGeometry.h"
#include "CableSimTautSolver.h"
#include "Algo/Unique.h"

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
			Edge.SurfaceVelocity = FirstTriangle.SurfaceVelocity;
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
			// Only wrapping topology belongs at a rope vertex. Face diagonals and
			// concave/boundary edges would inflate valence and make an ordinary
			// triangulated cube look unsupported at a corner.
			if (Edge.Kind != ECollisionEdgeKind::Convex
				&& Edge.Kind != ECollisionEdgeKind::NonManifold)
			{
				continue;
			}
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
		TArray<FCollisionVertex> UnclusteredVertices;
		Vertices.GenerateValueArray(UnclusteredVertices);
		UnclusteredVertices.Sort([](const FCollisionVertex& First, const FCollisionVertex& Second)
		{
			return FCollisionFeatureId::Less(First.Id, Second.Id);
		});

		// Shapes are compiled independently for edge adjacency, but their
		// coincident endpoints must form one topological intersection. This is
		// what prevents a taut line slipping through a numerical crack between
		// two boxes placed flush in the level editor.
		TArray<int32> Parents;
		Parents.SetNumUninitialized(UnclusteredVertices.Num());
		for (int32 Index = 0; Index < Parents.Num(); ++Index)
		{
			Parents[Index] = Index;
		}
		auto FindRoot = [&Parents](int32 Index)
		{
			while (Parents[Index] != Index)
			{
				Parents[Index] = Parents[Parents[Index]];
				Index = Parents[Index];
			}
			return Index;
		};
		for (int32 First = 0; First < UnclusteredVertices.Num(); ++First)
		{
			for (int32 Second = First + 1; Second < UnclusteredVertices.Num(); ++Second)
			{
				if (FVector3d::Distance(
					UnclusteredVertices[First].Position,
					UnclusteredVertices[Second].Position) <= FMath::Max(Tolerance, 1.e-9))
				{
					const int32 FirstRoot = FindRoot(First);
					const int32 SecondRoot = FindRoot(Second);
					if (FirstRoot != SecondRoot)
					{
						Parents[FMath::Max(FirstRoot, SecondRoot)] = FMath::Min(FirstRoot, SecondRoot);
					}
				}
			}
		}

		struct FClusteredVertex
		{
			FCollisionVertex Vertex;
			int32 Count = 0;
		};
		TMap<int32, FClusteredVertex> Clusters;
		for (int32 Index = 0; Index < UnclusteredVertices.Num(); ++Index)
		{
			const int32 Root = FindRoot(Index);
			FClusteredVertex& Cluster = Clusters.FindOrAdd(Root);
			const FCollisionVertex& Source = UnclusteredVertices[Index];
			if (Cluster.Count == 0)
			{
				Cluster.Vertex = Source;
				Cluster.Vertex.Position = FVector3d::ZeroVector;
				Cluster.Vertex.IncidentEdges.Reset();
			}
			Cluster.Vertex.Position += Source.Position;
			Cluster.Vertex.IncidentEdges.Append(Source.IncidentEdges);
			Cluster.Vertex.bNonManifold |= Source.bNonManifold;
			++Cluster.Count;
		}
		OutVertices.Reset(Clusters.Num());
		for (TPair<int32, FClusteredVertex>& Pair : Clusters)
		{
			Pair.Value.Vertex.Position /= static_cast<double>(Pair.Value.Count);
			OutVertices.Add(MoveTemp(Pair.Value.Vertex));
		}
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
			Vertex.IncidentEdges.SetNum(Algo::Unique(Vertex.IncidentEdges));
			Vertex.bOverValence = Vertex.IncidentEdges.Num() > SafeMaximumIncidentEdges;
			Diagnostics.OverValenceVertexCount += Vertex.bOverValence ? 1 : 0;
		}
		Diagnostics.VertexCount = OutVertices.Num();
		return Diagnostics;
	}

	namespace
	{
		struct FShapeKey
		{
			uint64 ObjectToken = 0;
			int32 ShapeIndex = INDEX_NONE;

			bool operator==(const FShapeKey& Other) const = default;
		};

		uint32 GetTypeHash(const FShapeKey& Key)
		{
			return HashCombineFast(::GetTypeHash(Key.ObjectToken), ::GetTypeHash(Key.ShapeIndex));
		}

		struct FConvexShapeData
		{
			FBox3d Bounds = FBox3d(ForceInit);
			TArray<FVector3d> Positions;
			TArray<FVector3d> FaceNormals;
			TArray<FVector3d> EdgeDirections;
			FCollisionFeatureId Feature;
		};

		void AddUniquePosition(TArray<FVector3d>& Positions, const FVector3d& Candidate, const double Tolerance)
		{
			if (!Positions.ContainsByPredicate([&Candidate, Tolerance](const FVector3d& Existing)
			{
				return Existing.Equals(Candidate, Tolerance);
			}))
			{
				Positions.Add(Candidate);
			}
		}

		void AddUniqueDirection(TArray<FVector3d>& Directions, const FVector3d& Candidate)
		{
			const FVector3d Normalized = Candidate.GetSafeNormal();
			if (Normalized.IsNearlyZero()
				|| Directions.ContainsByPredicate([&Normalized](const FVector3d& Existing)
				{
					return FMath::Abs(FVector3d::DotProduct(Existing, Normalized)) >= 1.0 - 1.e-6;
				}))
			{
				return;
			}
			Directions.Add(Normalized);
		}

		bool HasSeparatingAxis(
			const FConvexShapeData& First,
			const FConvexShapeData& Second,
			const FVector3d& Axis,
			const double Tolerance)
		{
			if (Axis.IsNearlyZero())
			{
				return false;
			}
			double FirstMin = TNumericLimits<double>::Max();
			double FirstMax = TNumericLimits<double>::Lowest();
			double SecondMin = TNumericLimits<double>::Max();
			double SecondMax = TNumericLimits<double>::Lowest();
			for (const FVector3d& Position : First.Positions)
			{
				const double Projection = FVector3d::DotProduct(Position, Axis);
				FirstMin = FMath::Min(FirstMin, Projection);
				FirstMax = FMath::Max(FirstMax, Projection);
			}
			for (const FVector3d& Position : Second.Positions)
			{
				const double Projection = FVector3d::DotProduct(Position, Axis);
				SecondMin = FMath::Min(SecondMin, Projection);
				SecondMax = FMath::Max(SecondMax, Projection);
			}
			const double Overlap = FMath::Min(FirstMax, SecondMax) - FMath::Max(FirstMin, SecondMin);
			return Overlap <= Tolerance;
		}

		bool ConvexShapesPenetrate(
			const FConvexShapeData& First,
			const FConvexShapeData& Second,
			const double Tolerance)
		{
			const FVector3d BoundsPenetration = First.Bounds.Max.ComponentMin(Second.Bounds.Max)
				- First.Bounds.Min.ComponentMax(Second.Bounds.Min);
			if (BoundsPenetration.X <= Tolerance || BoundsPenetration.Y <= Tolerance
				|| BoundsPenetration.Z <= Tolerance)
			{
				return false;
			}
			for (const FVector3d& Axis : First.FaceNormals)
			{
				if (HasSeparatingAxis(First, Second, Axis, Tolerance))
				{
					return false;
				}
			}
			for (const FVector3d& Axis : Second.FaceNormals)
			{
				if (HasSeparatingAxis(First, Second, Axis, Tolerance))
				{
					return false;
				}
			}
			for (const FVector3d& FirstEdge : First.EdgeDirections)
			{
				for (const FVector3d& SecondEdge : Second.EdgeDirections)
				{
					const FVector3d Axis = FVector3d::CrossProduct(FirstEdge, SecondEdge).GetSafeNormal();
					if (!Axis.IsNearlyZero() && HasSeparatingAxis(First, Second, Axis, Tolerance))
					{
						return false;
					}
				}
			}
			return true;
		}

		double PointSegmentDistance(
			const FVector3d& Point,
			const FVector3d& Start,
			const FVector3d& End,
			double& OutParameter)
		{
			const FVector3d Segment = End - Start;
			const double LengthSquared = Segment.SquaredLength();
			OutParameter = LengthSquared > UE_DOUBLE_SMALL_NUMBER
				? FMath::Clamp(FVector3d::DotProduct(Point - Start, Segment) / LengthSquared, 0.0, 1.0)
				: 0.0;
			return FVector3d::Distance(Point, Start + OutParameter * Segment);
		}

		uint64 CalculateTopologyRevision(
			const TConstArrayView<FCollisionTriangle> Triangles,
			const TConstArrayView<FCollisionEdge> Edges,
			const TConstArrayView<FCollisionVertex> Vertices)
		{
			uint32 Hash = HashCombineFast(::GetTypeHash(Triangles.Num()), ::GetTypeHash(Edges.Num()));
			Hash = HashCombineFast(Hash, ::GetTypeHash(Vertices.Num()));
			for (const FCollisionTriangle& Triangle : Triangles)
			{
				Hash = HashCombineFast(Hash, GetTypeHash(Triangle.Id));
				for (const FVector3d& Position : Triangle.Vertices)
				{
					Hash = HashCombineFast(Hash, ::GetTypeHash(Position.X));
					Hash = HashCombineFast(Hash, ::GetTypeHash(Position.Y));
					Hash = HashCombineFast(Hash, ::GetTypeHash(Position.Z));
				}
			}
			// Zero is reserved for an unpublished topology.
			return static_cast<uint64>(Hash) + 1;
		}
	}

	bool FTautStaticTopology::Build(
		const TConstArrayView<FCollisionTriangle> InTriangles,
		const double Tolerance,
		const int32 MaximumIncidentEdges)
	{
		const double SafeTolerance = FMath::Max(Tolerance, 1.e-9);
		TArray<FCollisionTriangle> CandidateTriangles(InTriangles);
		CandidateTriangles.Sort([](const FCollisionTriangle& First, const FCollisionTriangle& Second)
		{
			return FCollisionFeatureId::Less(First.Id, Second.Id);
		});

		FTautTopologyDiagnostics CandidateDiagnostics;
		for (int32 Index = 0; Index < CandidateTriangles.Num(); ++Index)
		{
			const FCollisionTriangle& Triangle = CandidateTriangles[Index];
			if (!Triangle.Id.IsValid() || Triangle.Id.Type != ECollisionFeatureType::Triangle
				|| !Triangle.IsFinite() || Triangle.CalculateNormal().IsNearlyZero())
			{
				CandidateDiagnostics.Issue = ETautTopologyIssue::InvalidFeature;
				CandidateDiagnostics.Feature = Triangle.Id;
				break;
			}
			if (!Triangle.bStaticObject
				|| (Triangle.GeometryType != ECollisionGeometryType::Box
					&& Triangle.GeometryType != ECollisionGeometryType::Convex))
			{
				CandidateDiagnostics.Issue = ETautTopologyIssue::UnsupportedGeometry;
				CandidateDiagnostics.Feature = Triangle.Id;
				break;
			}
			if (Index > 0 && Triangle.Id == CandidateTriangles[Index - 1].Id)
			{
				CandidateDiagnostics.Issue = ETautTopologyIssue::InvalidFeature;
				CandidateDiagnostics.Feature = Triangle.Id;
				break;
			}
		}

		TArray<FCollisionEdge> CandidateEdges;
		TArray<FCollisionVertex> CandidateVertices;
		if (CandidateDiagnostics.IsValid())
		{
			CandidateDiagnostics.Compile = FCollisionTopologyCompiler::CompileTopology(
				CandidateTriangles,
				SafeTolerance,
				FMath::Clamp(MaximumIncidentEdges, 1, 8),
				CandidateEdges,
				CandidateVertices);
			for (const FCollisionEdge& Edge : CandidateEdges)
			{
				if (Edge.Kind == ECollisionEdgeKind::Boundary)
				{
					CandidateDiagnostics.Issue = ETautTopologyIssue::OpenBoundary;
					CandidateDiagnostics.Feature = Edge.Id;
					break;
				}
				if (Edge.Kind == ECollisionEdgeKind::NonManifold)
				{
					CandidateDiagnostics.Issue = ETautTopologyIssue::NonManifold;
					CandidateDiagnostics.Feature = Edge.Id;
					break;
				}
			}
		}
		if (CandidateDiagnostics.IsValid())
		{
			for (const FCollisionVertex& Vertex : CandidateVertices)
			{
				if (Vertex.bNonManifold)
				{
					CandidateDiagnostics.Issue = ETautTopologyIssue::NonManifold;
					CandidateDiagnostics.Feature = Vertex.Id;
					break;
				}
				if (Vertex.bOverValence)
				{
					CandidateDiagnostics.Issue = ETautTopologyIssue::OverValence;
					CandidateDiagnostics.Feature = Vertex.Id;
					break;
				}
			}
		}

		// A vertex landing in the strict interior of an unrelated convex edge is a
		// T-junction. It has no unique incident-edge ordering, so the oriented
		// corner predicates must reject it rather than guess.
		if (CandidateDiagnostics.IsValid())
		{
			for (const FCollisionVertex& Vertex : CandidateVertices)
			{
				for (const FCollisionEdge& Edge : CandidateEdges)
				{
					if (Edge.Kind != ECollisionEdgeKind::Convex || Vertex.IncidentEdges.Contains(Edge.Id))
					{
						continue;
					}
					double Parameter = 0.0;
					if (PointSegmentDistance(Vertex.Position, Edge.Start, Edge.End, Parameter) <= SafeTolerance
						&& Parameter > SafeTolerance / FVector3d::Distance(Edge.Start, Edge.End)
						&& Parameter < 1.0 - SafeTolerance / FVector3d::Distance(Edge.Start, Edge.End))
					{
						CandidateDiagnostics.Issue = ETautTopologyIssue::TJunction;
						CandidateDiagnostics.Feature = Vertex.Id;
						CandidateDiagnostics.OtherFeature = Edge.Id;
						break;
					}
				}
				if (!CandidateDiagnostics.IsValid())
				{
					break;
				}
			}
		}

		// Complete convex shapes may touch at faces/edges/points, but their interiors
		// may not overlap. Face normals plus cross-products of unique edge directions
		// form the full 3D convex separating-axis set, avoiding AABB false positives.
		if (CandidateDiagnostics.IsValid())
		{
			TMap<FShapeKey, FConvexShapeData> Shapes;
			for (const FCollisionTriangle& Triangle : CandidateTriangles)
			{
				const FShapeKey Key{Triangle.Id.ObjectToken, Triangle.Id.ShapeIndex};
				FConvexShapeData& Shape = Shapes.FindOrAdd(Key);
				Shape.Feature = Triangle.Id;
				AddUniqueDirection(Shape.FaceNormals, Triangle.CalculateNormal());
				for (int32 Corner = 0; Corner < 3; ++Corner)
				{
					Shape.Bounds += Triangle.Vertices[Corner];
					AddUniquePosition(Shape.Positions, Triangle.Vertices[Corner], SafeTolerance);
					AddUniqueDirection(
						Shape.EdgeDirections,
						Triangle.Vertices[(Corner + 1) % 3] - Triangle.Vertices[Corner]);
				}
			}
			TArray<FShapeKey> Keys;
			Shapes.GetKeys(Keys);
			for (int32 A = 0; A < Keys.Num() && CandidateDiagnostics.IsValid(); ++A)
			{
				for (int32 B = A + 1; B < Keys.Num(); ++B)
				{
					if (ConvexShapesPenetrate(Shapes[Keys[A]], Shapes[Keys[B]], SafeTolerance))
					{
						CandidateDiagnostics.Issue = ETautTopologyIssue::OverlappingShapes;
						CandidateDiagnostics.Feature = Shapes[Keys[A]].Feature;
						CandidateDiagnostics.OtherFeature = Shapes[Keys[B]].Feature;
						break;
					}
				}
			}
		}

		Diagnostics = CandidateDiagnostics;
		if (!Diagnostics.IsValid())
		{
			return false;
		}
		Triangles = MoveTemp(CandidateTriangles);
		Edges = MoveTemp(CandidateEdges);
		Vertices = MoveTemp(CandidateVertices);
		Revision = CalculateTopologyRevision(Triangles, Edges, Vertices);
		return true;
	}

	void FTautStaticTopology::Reset()
	{
		Triangles.Reset();
		Edges.Reset();
		Vertices.Reset();
		Diagnostics = {};
		Revision = 0;
	}

	FTautCollisionScene FTautStaticTopology::GetScene() const
	{
		return {Triangles, Edges, Vertices};
	}
}
