#include "Chaos/CableSimChaosCollisionAdapter.h"

#include "Chaos/Box.h"
#include "Chaos/CollisionFilterData.h"
#include "Chaos/Convex.h"
#include "Chaos/HeightField.h"
#include "Chaos/ImplicitObject.h"
#include "Chaos/ImplicitObjectScaled.h"
#include "Chaos/ImplicitObjectTransformed.h"
#include "Chaos/PhysicsObjectInterface.h"
#include "Chaos/ShapeInstance.h"
#include "Chaos/TriangleMeshImplicitObject.h"
#include "CollisionQueryParams.h"
#include "CollisionShape.h"
#include "Engine/OverlapResult.h"
#include "Engine/World.h"
#include "HAL/PlatformTime.h"
#include "PhysicsEngine/PhysicsObjectExternalInterface.h"

namespace
{
	constexpr int32 MaximumPhysicsObjects = 256;
	constexpr int32 MaximumShapes = 512;
	constexpr int32 MaximumTriangles = 4096;

	void BuildSegmentBounds(
		const TConstArrayView<CableSim::FParticle> Particles,
		const FVector3d& StartTarget,
		const FVector3d& EndTarget,
		const TConstArrayView<FVector3d> TautPathPoints,
		const double Padding,
		TArray<FBox>& OutSegmentBounds)
	{
		const double SafePadding = FMath::Max(Padding, 0.1);
		const int32 LastParticle = Particles.Num() - 1;
		OutSegmentBounds.Reset(FMath::Max(LastParticle, 0) + FMath::Max(TautPathPoints.Num() - 1, 0));
		for (int32 Index = 0; Index < LastParticle; ++Index)
		{
			FBox Segment(ForceInit);
			Segment += FVector(Particles[Index].PreviousPosition);
			Segment += FVector(Particles[Index].Position);
			Segment += FVector(Particles[Index + 1].PreviousPosition);
			Segment += FVector(Particles[Index + 1].Position);
			if (Index == 0)
			{
				Segment += FVector(StartTarget);
			}
			if (Index + 1 == LastParticle)
			{
				Segment += FVector(EndTarget);
			}
			OutSegmentBounds.Add(Segment.ExpandBy(SafePadding));
		}
		// The dynamic rope can settle away from the taut path with nothing
		// pulling it back until a wrap is (re)found; sweep the taut path's own
		// segments too so its geometry stays visible regardless of where the
		// rope currently sits.
		for (int32 Index = 0; Index + 1 < TautPathPoints.Num(); ++Index)
		{
			FBox Segment(ForceInit);
			Segment += FVector(TautPathPoints[Index]);
			Segment += FVector(TautPathPoints[Index + 1]);
			OutSegmentBounds.Add(Segment.ExpandBy(SafePadding));
		}
	}

	FBox UnionBounds(TConstArrayView<FBox> SegmentBounds)
	{
		FBox Union(ForceInit);
		for (const FBox& Segment : SegmentBounds)
		{
			Union += Segment;
		}
		return Union;
	}

	bool IntersectsAny(const Chaos::FAABB3& Bounds, TConstArrayView<FBox> SegmentBounds)
	{
		const FBox ShapeBounds(FVector(Bounds.Min()), FVector(Bounds.Max()));
		for (const FBox& Segment : SegmentBounds)
		{
			if (ShapeBounds.Intersect(Segment))
			{
				return true;
			}
		}
		return false;
	}

	bool TriangleNearSegments(
		const FVector3d& Position0,
		const FVector3d& Position1,
		const FVector3d& Position2,
		TConstArrayView<FBox> SegmentBounds)
	{
		FBox TriangleBounds(ForceInit);
		TriangleBounds += FVector(Position0);
		TriangleBounds += FVector(Position1);
		TriangleBounds += FVector(Position2);
		for (const FBox& Segment : SegmentBounds)
		{
			if (Segment.Intersect(TriangleBounds))
			{
				return true;
			}
		}
		return false;
	}

	Chaos::FAABB3 TransformBoundsToLocal(const FBox& WorldBounds, const FTransform& LocalToWorld)
	{
		FBox LocalBounds(ForceInit);
		for (int32 Corner = 0; Corner < 8; ++Corner)
		{
			const FVector WorldPoint(
				(Corner & 1) ? WorldBounds.Max.X : WorldBounds.Min.X,
				(Corner & 2) ? WorldBounds.Max.Y : WorldBounds.Min.Y,
				(Corner & 4) ? WorldBounds.Max.Z : WorldBounds.Min.Z);
			LocalBounds += LocalToWorld.InverseTransformPosition(WorldPoint);
		}
		return Chaos::FAABB3(FVector3d(LocalBounds.Min), FVector3d(LocalBounds.Max));
	}

	void AddTriangle(
		const uint64 ObjectToken,
		const int32 ShapeIndex,
		const int32 TriangleIndex,
		const int32 Vertex0,
		const int32 Vertex1,
		const int32 Vertex2,
		const FVector3d& Position0,
		const FVector3d& Position1,
		const FVector3d& Position2,
		const CableSim::ECollisionGeometryType GeometryType,
		const bool bStaticObject,
		const TConstArrayView<FBox> SegmentBounds,
		TArray<CableSim::FCollisionTriangle>& OutTriangles,
		const bool bCullByProximity = true)
	{
		if (bCullByProximity && !TriangleNearSegments(Position0, Position1, Position2, SegmentBounds))
		{
			return;
		}
		CableSim::FCollisionTriangle& Triangle = OutTriangles.AddDefaulted_GetRef();
		Triangle.Id = {
			ObjectToken,
			ShapeIndex,
			CableSim::ECollisionFeatureType::Triangle,
			TriangleIndex,
			INDEX_NONE};
		Triangle.GeometryType = GeometryType;
		Triangle.bStaticObject = bStaticObject;
		Triangle.Vertices[0] = Position0;
		Triangle.Vertices[1] = Position1;
		Triangle.Vertices[2] = Position2;
		Triangle.VertexIndices[0] = Vertex0;
		Triangle.VertexIndices[1] = Vertex1;
		Triangle.VertexIndices[2] = Vertex2;
	}

	void AddBoxTriangles(
		const Chaos::FImplicitBox3& Box,
		const FTransform& LocalToWorld,
		const uint64 ObjectToken,
		const int32 ShapeIndex,
		const bool bStaticObject,
		const TConstArrayView<FBox> SegmentBounds,
		TArray<CableSim::FCollisionTriangle>& OutTriangles)
	{
		const FVector3d Min(Box.Min());
		const FVector3d Max(Box.Max());
		FVector3d Vertices[8] = {
			{Min.X, Min.Y, Min.Z}, {Max.X, Min.Y, Min.Z},
			{Max.X, Max.Y, Min.Z}, {Min.X, Max.Y, Min.Z},
			{Min.X, Min.Y, Max.Z}, {Max.X, Min.Y, Max.Z},
			{Max.X, Max.Y, Max.Z}, {Min.X, Max.Y, Max.Z}};
		for (FVector3d& Vertex : Vertices)
		{
			Vertex = FVector3d(LocalToWorld.TransformPosition(FVector(Vertex)));
		}
		int32 Indices[12][3] = {
			{0, 3, 2}, {0, 2, 1}, {4, 5, 6}, {4, 6, 7},
			{0, 1, 5}, {0, 5, 4}, {1, 2, 6}, {1, 6, 5},
			{2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7}};
		const bool bReverseWinding = LocalToWorld.ToMatrixWithScale().Determinant() < 0.0;
		for (int32 TriangleIndex = 0; TriangleIndex < 12; ++TriangleIndex)
		{
			if (bReverseWinding)
			{
				Swap(Indices[TriangleIndex][1], Indices[TriangleIndex][2]);
			}
			// Shape-level bounds already gated by the caller; per-triangle
			// culling here would fragment the box's topology (see CompileTopology).
			AddTriangle(
				ObjectToken,
				ShapeIndex,
				TriangleIndex,
				Indices[TriangleIndex][0],
				Indices[TriangleIndex][1],
				Indices[TriangleIndex][2],
				Vertices[Indices[TriangleIndex][0]],
				Vertices[Indices[TriangleIndex][1]],
				Vertices[Indices[TriangleIndex][2]],
				CableSim::ECollisionGeometryType::Box,
				bStaticObject,
				SegmentBounds,
				OutTriangles,
				false);
		}
	}

	void AddConvexTriangles(
		const Chaos::FConvex& Convex,
		const FVector3d& Scale,
		const FTransform& LocalToWorld,
		const uint64 ObjectToken,
		const int32 ShapeIndex,
		const bool bStaticObject,
		const TConstArrayView<FBox> SegmentBounds,
		TArray<CableSim::FCollisionTriangle>& OutTriangles)
	{
		int32 TriangleIndex = 0;
		const bool bReverseWinding = Scale.X * Scale.Y * Scale.Z
			* LocalToWorld.ToMatrixWithScale().Determinant() < 0.0;
		for (int32 FaceIndex = 0; FaceIndex < Convex.NumPlanes(); ++FaceIndex)
		{
			const int32 VertexCount = Convex.NumPlaneVertices(FaceIndex);
			for (int32 FaceVertex = 1; FaceVertex + 1 < VertexCount; ++FaceVertex)
			{
				int32 VertexIndices[3] = {
					Convex.GetPlaneVertex(FaceIndex, 0),
					Convex.GetPlaneVertex(FaceIndex, FaceVertex),
					Convex.GetPlaneVertex(FaceIndex, FaceVertex + 1)};
				if (bReverseWinding)
				{
					Swap(VertexIndices[1], VertexIndices[2]);
				}
				FVector3d Positions[3];
				for (int32 Corner = 0; Corner < 3; ++Corner)
				{
					const FVector3d Local = FVector3d(Convex.GetVertex(VertexIndices[Corner])) * Scale;
					Positions[Corner] = FVector3d(LocalToWorld.TransformPosition(FVector(Local)));
				}
				// Same reasoning as AddBoxTriangles above.
				AddTriangle(
					ObjectToken,
					ShapeIndex,
					TriangleIndex++,
					VertexIndices[0],
					VertexIndices[1],
					VertexIndices[2],
					Positions[0],
					Positions[1],
					Positions[2],
					CableSim::ECollisionGeometryType::Convex,
					bStaticObject,
					SegmentBounds,
					OutTriangles,
					false);
			}
		}
	}

	const Chaos::FImplicitObject* UnwrapGeometry(
		const Chaos::FImplicitObject* Geometry,
		FTransform& InOutLocalToWorld,
		FVector3d& OutScale)
	{
		OutScale = FVector3d::OneVector;
		while (Geometry)
		{
			const Chaos::EImplicitObjectType Type = Geometry->GetType();
			if (Type == Chaos::ImplicitObjectType::Transformed)
			{
				const auto& Transformed = Geometry->GetObjectChecked<Chaos::TImplicitObjectTransformed<Chaos::FReal, 3>>();
				InOutLocalToWorld = FTransform(Transformed.GetTransform()) * InOutLocalToWorld;
				Geometry = Transformed.GetTransformedObject();
				continue;
			}
			if (Chaos::IsScaled(Type))
			{
				const auto* Scaled = static_cast<const Chaos::FImplicitObjectScaled*>(Geometry);
				OutScale *= FVector3d(Scaled->GetScale());
				Geometry = Scaled->GetInnerObject().Get();
				continue;
			}
			if (Chaos::IsInstanced(Type))
			{
				const auto* Instanced = static_cast<const Chaos::FImplicitObjectInstanced*>(Geometry);
				Geometry = Instanced->GetInnerObject().Get();
				continue;
			}
			break;
		}
		return Geometry;
	}

	bool AddGeometryTriangles(
		const Chaos::FImplicitObject* Geometry,
		const FTransform& ObjectTransform,
		const FBox& QueryBounds,
		const uint64 ObjectToken,
		const int32 ShapeIndex,
		const bool bStaticObject,
		const TConstArrayView<FBox> SegmentBounds,
		TArray<CableSim::FCollisionTriangle>& OutTriangles)
	{
		FTransform LocalToWorld = ObjectTransform;
		FVector3d Scale;
		Geometry = UnwrapGeometry(Geometry, LocalToWorld, Scale);
		if (!Geometry)
		{
			return false;
		}
		LocalToWorld.SetScale3D(FVector(Scale));
		const Chaos::EImplicitObjectType Type = Chaos::GetInnerType(Geometry->GetType());
		if (Type == Chaos::ImplicitObjectType::Box)
		{
			AddBoxTriangles(
				Geometry->GetObjectChecked<Chaos::FImplicitBox3>(),
				LocalToWorld,
				ObjectToken,
				ShapeIndex,
				bStaticObject,
				SegmentBounds,
				OutTriangles);
			return true;
		}
		if (Type == Chaos::ImplicitObjectType::Convex)
		{
			FTransform RigidLocalToWorld = LocalToWorld;
			RigidLocalToWorld.SetScale3D(FVector::OneVector);
			AddConvexTriangles(
				Geometry->GetObjectChecked<Chaos::FConvex>(),
				Scale,
				RigidLocalToWorld,
				ObjectToken,
				ShapeIndex,
				bStaticObject,
				SegmentBounds,
				OutTriangles);
			return true;
		}

		const Chaos::FAABB3 LocalBounds = TransformBoundsToLocal(QueryBounds, LocalToWorld);
		if (Type == Chaos::ImplicitObjectType::TriangleMesh)
		{
			const auto& Mesh = Geometry->GetObjectChecked<Chaos::FTriangleMeshImplicitObject>();
			Mesh.VisitTriangles(LocalBounds, Chaos::FRigidTransform3(LocalToWorld),
				[&OutTriangles, ObjectToken, ShapeIndex, bStaticObject, SegmentBounds](
					const Chaos::FTriangle& Triangle,
					const int32 TriangleIndex,
					const int32 Vertex0,
					const int32 Vertex1,
					const int32 Vertex2)
				{
					if (OutTriangles.Num() < MaximumTriangles)
					{
						AddTriangle(
							ObjectToken, ShapeIndex, TriangleIndex,
							Vertex0, Vertex1, Vertex2,
							FVector3d(Triangle[0]), FVector3d(Triangle[1]), FVector3d(Triangle[2]),
							CableSim::ECollisionGeometryType::TriangleMesh,
							bStaticObject,
							SegmentBounds,
							OutTriangles);
					}
				});
			return true;
		}
		if (Type == Chaos::ImplicitObjectType::HeightField)
		{
			const auto& HeightField = Geometry->GetObjectChecked<Chaos::FHeightField>();
			HeightField.VisitTriangles(LocalBounds, Chaos::FRigidTransform3(LocalToWorld),
				[&OutTriangles, ObjectToken, ShapeIndex, bStaticObject, SegmentBounds](
					const Chaos::FTriangle& Triangle,
					const int32 TriangleIndex,
					const int32 Vertex0,
					const int32 Vertex1,
					const int32 Vertex2)
				{
					if (OutTriangles.Num() < MaximumTriangles)
					{
						AddTriangle(
							ObjectToken, ShapeIndex, TriangleIndex,
							Vertex0, Vertex1, Vertex2,
							FVector3d(Triangle[0]), FVector3d(Triangle[1]), FVector3d(Triangle[2]),
							CableSim::ECollisionGeometryType::HeightField,
							bStaticObject,
							SegmentBounds,
							OutTriangles);
					}
				});
			return true;
		}
		return false;
	}
}

void FCableSimChaosSnapshot::Reset()
{
	QueryBounds = FBox(ForceInit);
	Triangles.Reset();
	Edges.Reset();
	Vertices.Reset();
	Diagnostics = FCableSimChaosSnapshotDiagnostics{};
}

void FCableSimChaosObjectTracker::Reset()
{
	Records.Reset();
	NextToken = 1;
	SnapshotSerial = 0;
}

void FCableSimChaosObjectTracker::BeginSnapshot()
{
	++SnapshotSerial;
}

void FCableSimChaosObjectTracker::EndSnapshot()
{
	constexpr uint64 RetainedSnapshotCount = 600;
	for (auto Iterator = Records.CreateIterator(); Iterator; ++Iterator)
	{
		const FRecord& Record = Iterator.Value();
		const bool bOwnerExpired = !Record.Owner.IsValid() && !Record.Owner.IsExplicitlyNull();
		const bool bStale = SnapshotSerial > Record.LastSeenSnapshot
			&& SnapshotSerial - Record.LastSeenSnapshot > RetainedSnapshotCount;
		if (bOwnerExpired || bStale)
		{
			Iterator.RemoveCurrent();
		}
	}
}

uint64 FCableSimChaosObjectTracker::ResolveToken(
	Chaos::FConstPhysicsObjectHandle Handle,
	const TWeakObjectPtr<UObject>& Owner)
{
	if (!Handle)
	{
		return 0;
	}
	const void* Key = Handle;
	FRecord* Existing = Records.Find(Key);
	if (Existing && Existing->Owner == Owner && (Owner.IsValid() || Existing->Owner.IsExplicitlyNull()))
	{
		Existing->LastSeenSnapshot = SnapshotSerial;
		return Existing->Token;
	}
	FRecord& Record = Records.FindOrAdd(Key);
	Record.Owner = Owner;
	Record.Token = NextToken++;
	Record.LastSeenSnapshot = SnapshotSerial;
	return Record.Token;
}

bool FCableSimChaosCollisionAdapter::GatherSnapshot(
	UWorld* World,
	AActor* Owner,
	const TConstArrayView<AActor*> IgnoredActors,
	const FCableSimCollisionSettings& CollisionSettings,
	const double TopologyTolerance,
	const TConstArrayView<CableSim::FParticle> Particles,
	const FVector3d& StartTarget,
	const FVector3d& EndTarget,
	const TConstArrayView<FVector3d> TautPathPoints,
	FCableSimChaosObjectTracker& ObjectTracker,
	FCableSimChaosSnapshot& OutSnapshot)
{
	OutSnapshot.Reset();
	if (!World || Particles.Num() < 2)
	{
		return false;
	}
	ObjectTracker.BeginSnapshot();
	const double StartTime = FPlatformTime::Seconds();
	TArray<FBox> SegmentBounds;
	BuildSegmentBounds(
		Particles,
		StartTarget,
		EndTarget,
		TautPathPoints,
		CollisionSettings.Radius + TopologyTolerance,
		SegmentBounds);
	OutSnapshot.QueryBounds = UnionBounds(SegmentBounds);

	FCollisionQueryParams QueryParams(SCENE_QUERY_STAT(CableSimChaosTopology), false, Owner);
	for (AActor* IgnoredActor : IgnoredActors)
	{
		if (IsValid(IgnoredActor))
		{
			QueryParams.AddIgnoredActor(IgnoredActor);
		}
	}
	const FVector Center = OutSnapshot.QueryBounds.GetCenter();
	const FVector Extent = OutSnapshot.QueryBounds.GetExtent().ComponentMax(FVector(0.1));
	TArray<FOverlapResult> Overlaps;
	for (const bool bTraceComplex : {false, true})
	{
		QueryParams.bTraceComplex = bTraceComplex;
		TArray<FOverlapResult> PassOverlaps;
		World->OverlapMultiByChannel(
			PassOverlaps,
			Center,
			FQuat::Identity,
			CollisionSettings.Channel.GetValue(),
			FCollisionShape::MakeBox(Extent),
			QueryParams);
		Overlaps.Append(PassOverlaps);
	}
	OutSnapshot.Diagnostics.OverlapCount = Overlaps.Num();

	TArray<Chaos::FConstPhysicsObjectHandle> Handles;
	TMap<Chaos::FConstPhysicsObjectHandle, TWeakObjectPtr<UObject>> HandleOwners;
	for (const FOverlapResult& Overlap : Overlaps)
	{
		if (!Overlap.PhysicsObject || Handles.Contains(Overlap.PhysicsObject))
		{
			continue;
		}
		if (Handles.Num() >= MaximumPhysicsObjects)
		{
			OutSnapshot.Diagnostics.bFeatureBudgetExceeded = true;
			break;
		}
		Handles.Add(Overlap.PhysicsObject);
		HandleOwners.Add(Overlap.PhysicsObject, Overlap.PhysicsObjectOwner);
	}
	OutSnapshot.Diagnostics.PhysicsObjectCount = Handles.Num();
	if (Handles.IsEmpty())
	{
		ObjectTracker.EndSnapshot();
		OutSnapshot.Diagnostics.SnapshotMilliseconds = (FPlatformTime::Seconds() - StartTime) * 1000.0;
		return true;
	}

	auto Locked = FPhysicsObjectExternalInterface::LockRead(Handles);
	auto& Interface = Locked.GetInterface();
	for (const Chaos::FConstPhysicsObjectHandle Handle : Handles)
	{
		if (OutSnapshot.Diagnostics.ShapeCount >= MaximumShapes)
		{
			OutSnapshot.Diagnostics.bFeatureBudgetExceeded = true;
			break;
		}
		const Chaos::FConstPhysicsObjectHandle SingleHandle = Handle;
		if (!Interface.AreAllValid(MakeArrayView(&SingleHandle, 1)))
		{
			continue;
		}
		const TWeakObjectPtr<UObject> PhysicsOwner = HandleOwners.FindRef(Handle);
		const uint64 ObjectToken = ObjectTracker.ResolveToken(Handle, PhysicsOwner);
		const FTransform ObjectTransform = Interface.GetTransform(Handle);
		const bool bStaticObject = !Interface.AreAllKinematic(MakeArrayView(&SingleHandle, 1))
			&& !Interface.AreAllDynamicOrSleeping(MakeArrayView(&SingleHandle, 1));
		const int32 ObjectTriangleStart = OutSnapshot.Triangles.Num();
		Interface.VisitEveryShape(MakeArrayView(&SingleHandle, 1),
			[&](const Chaos::FConstPhysicsObjectHandle, Chaos::TThreadShapeInstance<Chaos::EThreadContext::External>* Shape)
			{
				if (OutSnapshot.Diagnostics.ShapeCount >= MaximumShapes)
				{
					OutSnapshot.Diagnostics.bFeatureBudgetExceeded = true;
					return true;
				}
				if (!Shape || !Shape->GetQueryEnabled() || !IntersectsAny(Shape->GetWorldSpaceShapeBounds(), SegmentBounds))
				{
					return false;
				}
				const Chaos::Filter::FCombinedShapeFilterData CombinedFilter =
					Shape->GetCombinedShapeFilterData();
				const Chaos::Filter::FShapeFilterData& ShapeFilter = CombinedFilter.GetShapeFilterData();
				const uint64 ChannelBit = 1ull << static_cast<uint8>(CollisionSettings.Channel.GetValue());
				if ((ShapeFilter.GetBlockChannels() & ChannelBit) == 0)
				{
					return false;
				}
				++OutSnapshot.Diagnostics.ShapeCount;
				if (!AddGeometryTriangles(
					Shape->GetGeometry(),
					ObjectTransform,
					OutSnapshot.QueryBounds,
					ObjectToken,
					Shape->GetShapeIndex(),
					bStaticObject,
					SegmentBounds,
					OutSnapshot.Triangles))
				{
					++OutSnapshot.Diagnostics.UnsupportedShapeCount;
				}
				if (OutSnapshot.Triangles.Num() >= MaximumTriangles)
				{
					OutSnapshot.Diagnostics.bFeatureBudgetExceeded = true;
					return true;
				}
				return OutSnapshot.Diagnostics.ShapeCount >= MaximumShapes;
			});
		// Stamp the surface's world velocity onto the triangles this object just added
		// so friction can carry a resting cable along with a moving platform. Static
		// objects keep zero. GetVAtPoint includes rotation; a per-triangle centroid is
		// exact for translation and a close approximation for rotation across one face.
		if (!bStaticObject)
		{
			for (int32 TriangleIndex = ObjectTriangleStart;
				TriangleIndex < OutSnapshot.Triangles.Num();
				++TriangleIndex)
			{
				CableSim::FCollisionTriangle& Triangle = OutSnapshot.Triangles[TriangleIndex];
				const FVector Centroid(
					(Triangle.Vertices[0] + Triangle.Vertices[1] + Triangle.Vertices[2]) / 3.0);
				Triangle.SurfaceVelocity = FVector3d(Interface.GetVAtPoint(Handle, Centroid));
			}
		}
		if (OutSnapshot.Diagnostics.bFeatureBudgetExceeded)
		{
			break;
		}
	}
	Locked.Release();
	ObjectTracker.EndSnapshot();
	OutSnapshot.Diagnostics.SnapshotMilliseconds = (FPlatformTime::Seconds() - StartTime) * 1000.0;
	OutSnapshot.Diagnostics.TriangleCount = OutSnapshot.Triangles.Num();
	if (OutSnapshot.Diagnostics.bFeatureBudgetExceeded)
	{
		OutSnapshot.Triangles.Reset();
		return false;
	}

	const double CompileStart = FPlatformTime::Seconds();
	const CableSim::FTopologyCompileDiagnostics TopologyDiagnostics =
		CableSim::FCollisionTopologyCompiler::CompileTopology(
			OutSnapshot.Triangles,
			TopologyTolerance,
			8,
			OutSnapshot.Edges,
			OutSnapshot.Vertices);
	OutSnapshot.Diagnostics.CompileMilliseconds = (FPlatformTime::Seconds() - CompileStart) * 1000.0;
	OutSnapshot.Diagnostics.EdgeCount = OutSnapshot.Edges.Num();
	OutSnapshot.Diagnostics.VertexCount = TopologyDiagnostics.VertexCount;
	OutSnapshot.Diagnostics.OverValenceVertexCount = TopologyDiagnostics.OverValenceVertexCount;
	OutSnapshot.Diagnostics.DiscardedFeatureCount =
		TopologyDiagnostics.ConcaveEdgeCount
		+ TopologyDiagnostics.CoplanarEdgeCount
		+ TopologyDiagnostics.DegenerateEdgeCount
		+ TopologyDiagnostics.NonManifoldEdgeCount;
	return true;
}
