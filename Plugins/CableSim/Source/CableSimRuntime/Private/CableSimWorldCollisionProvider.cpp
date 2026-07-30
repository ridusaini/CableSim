#include "CableSimWorldCollisionProvider.h"

#include "Chaos/Box.h"
#include "Chaos/Capsule.h"
#include "Chaos/Convex.h"
#include "Chaos/HeightField.h"
#include "Chaos/ImplicitObject.h"
#include "Chaos/ImplicitObjectScaled.h"
#include "Chaos/ImplicitObjectTransformed.h"
#include "Chaos/PhysicsObjectInterface.h"
#include "Chaos/ShapeInstance.h"
#include "Chaos/Sphere.h"
#include "Chaos/TriangleMeshImplicitObject.h"
#include "CollisionQueryParams.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/OverlapResult.h"
#include "Engine/World.h"
#include "HAL/PlatformTime.h"
#include "PhysicsEngine/PhysicsObjectExternalInterface.h"

namespace
{
	constexpr int32 MaximumPhysicsObjects = 256;
	constexpr int32 MaximumShapes = 512;
	constexpr int32 MaximumTrianglesPerNode = 256;
	constexpr double NormalMergeCosine = 0.996194698; // 5 degrees

	FBox BuildPredictedNodeBounds(
		const TConstArrayView<CableSim::FParticle> Particles,
		const FVector3d& StartTarget,
		const FVector3d& EndTarget,
		const double DeltaTime,
		const double Padding,
		TArray<FCableSimNodeCollisionGeometry>& OutNodes)
	{
		OutNodes.SetNum(Particles.Num());
		FBox CableBounds(ForceInit);
		const FVector3d StartDelta = StartTarget - Particles[0].Position;
		const FVector3d EndDelta = EndTarget - Particles.Last().Position;
		const double ActiveLength = FMath::Max(
			Particles.Last().MaterialCoordinate - Particles[0].MaterialCoordinate,
			1.e-9);
		for (int32 Index = 0; Index < Particles.Num(); ++Index)
		{
			const CableSim::FParticle& Particle = Particles[Index];
			FBox Bounds(ForceInit);
			Bounds += FVector(Particle.PreviousPosition);
			Bounds += FVector(Particle.Position);
			Bounds += FVector(Particle.Position + Particle.Velocity * DeltaTime);
			const double MaterialAlpha = FMath::Clamp(
				(Particle.MaterialCoordinate - Particles[0].MaterialCoordinate) / ActiveLength,
				0.0,
				1.0);
			const FVector3d KeyframeMotion = FMath::Lerp(StartDelta, EndDelta, MaterialAlpha);
			Bounds += FVector(Particle.Position + KeyframeMotion);
			if (Index > 0)
			{
				Bounds += FVector(Particles[Index - 1].Position);
				Bounds += FVector(Particles[Index - 1].Position + Particles[Index - 1].Velocity * DeltaTime);
			}
			if (Index + 1 < Particles.Num())
			{
				Bounds += FVector(Particles[Index + 1].Position);
				Bounds += FVector(Particles[Index + 1].Position + Particles[Index + 1].Velocity * DeltaTime);
			}
			OutNodes[Index].PredictedBounds = Bounds.ExpandBy(FMath::Max(Padding, 0.1));
			CableBounds += OutNodes[Index].PredictedBounds;
		}
		return CableBounds;
	}

	bool Intersects(const Chaos::FAABB3& Bounds, const FBox& QueryBounds)
	{
		return FBox(FVector(Bounds.Min()), FVector(Bounds.Max())).Intersect(QueryBounds);
	}

	Chaos::FAABB3 TransformBoundsToLocal(const FBox& WorldBounds, const FTransform& LocalToWorld)
	{
		FBox LocalBounds(ForceInit);
		for (int32 Corner = 0; Corner < 8; ++Corner)
		{
			const FVector Point(
				(Corner & 1) ? WorldBounds.Max.X : WorldBounds.Min.X,
				(Corner & 2) ? WorldBounds.Max.Y : WorldBounds.Min.Y,
				(Corner & 4) ? WorldBounds.Max.Z : WorldBounds.Min.Z);
			LocalBounds += LocalToWorld.InverseTransformPosition(Point);
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
		const FVector3d& P0,
		const FVector3d& P1,
		const FVector3d& P2,
		const CableSim::ECollisionGeometryType Type,
		TArray<CableSim::FCollisionTriangle>& OutTriangles)
	{
		CableSim::FCollisionTriangle& Triangle = OutTriangles.AddDefaulted_GetRef();
		Triangle.Id = {ObjectToken, ShapeIndex, CableSim::ECollisionFeatureType::Face, TriangleIndex, INDEX_NONE};
		Triangle.GeometryType = Type;
		Triangle.Vertices[0] = P0;
		Triangle.Vertices[1] = P1;
		Triangle.Vertices[2] = P2;
		Triangle.VertexIndices[0] = Vertex0;
		Triangle.VertexIndices[1] = Vertex1;
		Triangle.VertexIndices[2] = Vertex2;
	}

	void AddBoxTriangles(
		const Chaos::FImplicitBox3& Box,
		const FTransform& LocalToWorld,
		const uint64 ObjectToken,
		const int32 ShapeIndex,
		TArray<CableSim::FCollisionTriangle>& OutTriangles)
	{
		const FVector3d Min(Box.Min());
		const FVector3d Max(Box.Max());
		FVector3d Vertices[8] = {
			{Min.X, Min.Y, Min.Z}, {Max.X, Min.Y, Min.Z}, {Max.X, Max.Y, Min.Z}, {Min.X, Max.Y, Min.Z},
			{Min.X, Min.Y, Max.Z}, {Max.X, Min.Y, Max.Z}, {Max.X, Max.Y, Max.Z}, {Min.X, Max.Y, Max.Z}};
		for (FVector3d& Vertex : Vertices) Vertex = FVector3d(LocalToWorld.TransformPosition(FVector(Vertex)));
		int32 Indices[12][3] = {
			{0,3,2}, {0,2,1}, {4,5,6}, {4,6,7}, {0,1,5}, {0,5,4},
			{1,2,6}, {1,6,5}, {2,3,7}, {2,7,6}, {3,0,4}, {3,4,7}};
		const bool bReverse = LocalToWorld.ToMatrixWithScale().Determinant() < 0.0;
		for (int32 Index = 0; Index < 12; ++Index)
		{
			if (bReverse) Swap(Indices[Index][1], Indices[Index][2]);
			AddTriangle(ObjectToken, ShapeIndex, Index,
				Indices[Index][0], Indices[Index][1], Indices[Index][2],
				Vertices[Indices[Index][0]], Vertices[Indices[Index][1]], Vertices[Indices[Index][2]],
				CableSim::ECollisionGeometryType::Box, OutTriangles);
		}
	}

	void AddConvexTriangles(
		const Chaos::FConvex& Convex,
		const FVector3d& Scale,
		const FTransform& LocalToWorld,
		const uint64 ObjectToken,
		const int32 ShapeIndex,
		TArray<CableSim::FCollisionTriangle>& OutTriangles)
	{
		int32 TriangleIndex = 0;
		const bool bReverse = Scale.X * Scale.Y * Scale.Z * LocalToWorld.ToMatrixWithScale().Determinant() < 0.0;
		for (int32 Face = 0; Face < Convex.NumPlanes(); ++Face)
		{
			for (int32 FaceVertex = 1; FaceVertex + 1 < Convex.NumPlaneVertices(Face); ++FaceVertex)
			{
				int32 Indices[3] = {Convex.GetPlaneVertex(Face, 0), Convex.GetPlaneVertex(Face, FaceVertex), Convex.GetPlaneVertex(Face, FaceVertex + 1)};
				if (bReverse) Swap(Indices[1], Indices[2]);
				FVector3d Positions[3];
				for (int32 Corner = 0; Corner < 3; ++Corner)
				{
					const FVector3d Local = FVector3d(Convex.GetVertex(Indices[Corner])) * Scale;
					Positions[Corner] = FVector3d(LocalToWorld.TransformPosition(FVector(Local)));
				}
				AddTriangle(ObjectToken, ShapeIndex, TriangleIndex++, Indices[0], Indices[1], Indices[2],
					Positions[0], Positions[1], Positions[2], CableSim::ECollisionGeometryType::Convex, OutTriangles);
			}
		}
	}

	void AppendNearestTriangles(
		TArray<CableSim::FCollisionTriangle>& Candidates,
		const FBox& QueryBounds,
		TArray<CableSim::FCollisionTriangle>& OutTriangles)
	{
		const FVector3d Centre(QueryBounds.GetCenter());
		Candidates.Sort([&](const CableSim::FCollisionTriangle& A, const CableSim::FCollisionTriangle& B)
		{
			const FVector3d CentreA = (A.Vertices[0] + A.Vertices[1] + A.Vertices[2]) / 3.0;
			const FVector3d CentreB = (B.Vertices[0] + B.Vertices[1] + B.Vertices[2]) / 3.0;
			const double DistanceA = (CentreA - Centre).SquaredLength();
			const double DistanceB = (CentreB - Centre).SquaredLength();
			return DistanceA != DistanceB
				? DistanceA < DistanceB
				: CableSim::FCollisionFeatureId::Less(A.Id, B.Id);
		});
		const int32 Available = FMath::Max(MaximumTrianglesPerNode - OutTriangles.Num(), 0);
		for (int32 Index = 0; Index < FMath::Min(Available, Candidates.Num()); ++Index)
		{
			OutTriangles.Add(MoveTemp(Candidates[Index]));
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

	bool AddGeometry(
		const Chaos::FImplicitObject* Geometry,
		const FTransform& ObjectTransform,
		const FBox& QueryBounds,
		const uint64 ObjectToken,
		const int32 ShapeIndex,
		FCableSimNodeCollisionGeometry& OutGeometry)
	{
		FTransform LocalToWorld = ObjectTransform;
		FVector3d Scale;
		Geometry = UnwrapGeometry(Geometry, LocalToWorld, Scale);
		if (!Geometry) return false;
		LocalToWorld.SetScale3D(FVector(Scale));
		const Chaos::EImplicitObjectType Type = Chaos::GetInnerType(Geometry->GetType());
		if (Type == Chaos::ImplicitObjectType::Box)
		{
			AddBoxTriangles(Geometry->GetObjectChecked<Chaos::FImplicitBox3>(), LocalToWorld, ObjectToken, ShapeIndex, OutGeometry.Triangles);
			return true;
		}
		if (Type == Chaos::ImplicitObjectType::Convex)
		{
			FTransform RigidTransform = LocalToWorld;
			RigidTransform.SetScale3D(FVector::OneVector);
			AddConvexTriangles(Geometry->GetObjectChecked<Chaos::FConvex>(), Scale, RigidTransform, ObjectToken, ShapeIndex, OutGeometry.Triangles);
			return true;
		}
		if (Type == Chaos::ImplicitObjectType::Sphere)
		{
			const auto& Sphere = Geometry->GetObjectChecked<Chaos::FSphere>();
			FCableSimCollisionSphere& Primitive = OutGeometry.Spheres.AddDefaulted_GetRef();
			Primitive.Id = {ObjectToken, ShapeIndex, CableSim::ECollisionFeatureType::Face, 0, INDEX_NONE};
			Primitive.Centre = FVector3d(LocalToWorld.TransformPosition(FVector(Sphere.GetCenterf())));
			Primitive.Radius = Sphere.GetRadiusf() * LocalToWorld.GetScale3D().GetAbsMax();
			return true;
		}
		if (Type == Chaos::ImplicitObjectType::Capsule)
		{
			const auto& Capsule = Geometry->GetObjectChecked<Chaos::FCapsule>();
			FCableSimCollisionCapsule& Primitive = OutGeometry.Capsules.AddDefaulted_GetRef();
			Primitive.Id = {ObjectToken, ShapeIndex, CableSim::ECollisionFeatureType::Face, 0, INDEX_NONE};
			Primitive.Start = FVector3d(LocalToWorld.TransformPosition(FVector(Capsule.GetX1f())));
			Primitive.End = FVector3d(LocalToWorld.TransformPosition(FVector(Capsule.GetX2f())));
			Primitive.Radius = Capsule.GetRadiusf() * LocalToWorld.GetScale3D().GetAbsMax();
			return true;
		}
		const Chaos::FAABB3 LocalBounds = TransformBoundsToLocal(QueryBounds, LocalToWorld);
		if (Type == Chaos::ImplicitObjectType::TriangleMesh)
		{
			const auto& Mesh = Geometry->GetObjectChecked<Chaos::FTriangleMeshImplicitObject>();
			TArray<CableSim::FCollisionTriangle> Candidates;
			Candidates.Reserve(MaximumTrianglesPerNode * 4);
			Mesh.VisitTriangles(LocalBounds, Chaos::FRigidTransform3::Identity,
				[&](const Chaos::FTriangle& Triangle, const int32 TriangleIndex, const int32 V0, const int32 V1, const int32 V2)
				{
					if (Candidates.Num() < MaximumTrianglesPerNode * 4)
						AddTriangle(ObjectToken, ShapeIndex, TriangleIndex, V0, V1, V2,
							FVector3d(LocalToWorld.TransformPosition(FVector(Triangle[0]))),
							FVector3d(LocalToWorld.TransformPosition(FVector(Triangle[1]))),
							FVector3d(LocalToWorld.TransformPosition(FVector(Triangle[2]))),
							CableSim::ECollisionGeometryType::TriangleMesh, Candidates);
				});
			AppendNearestTriangles(Candidates, QueryBounds, OutGeometry.Triangles);
			return true;
		}
		if (Type == Chaos::ImplicitObjectType::HeightField)
		{
			const auto& HeightField = Geometry->GetObjectChecked<Chaos::FHeightField>();
			TArray<CableSim::FCollisionTriangle> Candidates;
			Candidates.Reserve(MaximumTrianglesPerNode * 4);
			HeightField.VisitTriangles(LocalBounds, Chaos::FRigidTransform3::Identity,
				[&](const Chaos::FTriangle& Triangle, const int32 TriangleIndex, const int32 V0, const int32 V1, const int32 V2)
				{
					if (Candidates.Num() < MaximumTrianglesPerNode * 4)
						AddTriangle(ObjectToken, ShapeIndex, TriangleIndex, V0, V1, V2,
							FVector3d(LocalToWorld.TransformPosition(FVector(Triangle[0]))),
							FVector3d(LocalToWorld.TransformPosition(FVector(Triangle[1]))),
							FVector3d(LocalToWorld.TransformPosition(FVector(Triangle[2]))),
							CableSim::ECollisionGeometryType::HeightField, Candidates);
				});
			AppendNearestTriangles(Candidates, QueryBounds, OutGeometry.Triangles);
			return true;
		}
		return false;
	}

	FVector3d ClosestPointOnTriangle(const FVector3d& Point, const CableSim::FCollisionTriangle& Triangle)
	{
		return FVector3d(FMath::ClosestPointOnTriangleToPoint(
			FVector(Point), FVector(Triangle.Vertices[0]), FVector(Triangle.Vertices[1]), FVector(Triangle.Vertices[2])));
	}

	FVector3d ClosestPointOnSegment(const FVector3d& Point, const FVector3d& Start, const FVector3d& End)
	{
		const FVector3d Delta = End - Start;
		const double LengthSquared = Delta.SquaredLength();
		if (LengthSquared <= 1.e-12) return Start;
		return Start + Delta * FMath::Clamp(FVector3d::DotProduct(Point - Start, Delta) / LengthSquared, 0.0, 1.0);
	}

	double SegmentAlphaAtPoint(const FVector3d& Start, const FVector3d& End, const FVector3d& Point)
	{
		const FVector3d Delta = End - Start;
		const double LengthSquared = Delta.SquaredLength();
		return LengthSquared > 1.e-12
			? FMath::Clamp(FVector3d::DotProduct(Point - Start, Delta) / LengthSquared, 0.0, 1.0)
			: 0.0;
	}

	bool ProjectionInsideTriangle(const FVector3d& Projection, const CableSim::FCollisionTriangle& Triangle, const double Tolerance)
	{
		return FVector3d::Distance(Projection, ClosestPointOnTriangle(Projection, Triangle)) <= Tolerance;
	}

	bool CrossesTrianglePlane(
		const FVector3d& Previous,
		const FVector3d& Current,
		const CableSim::FCollisionTriangle& Triangle,
		const FVector3d& Normal,
		const double Radius,
		const double Tolerance)
	{
		const double Plane = FVector3d::DotProduct(Triangle.Vertices[0], Normal) + Radius;
		const double PreviousDistance = FVector3d::DotProduct(Previous, Normal) - Plane;
		const double CurrentDistance = FVector3d::DotProduct(Current, Normal) - Plane;
		if (PreviousDistance <= 0.0 || CurrentDistance >= 0.0 || PreviousDistance - CurrentDistance <= 1.e-9) return false;
		const double Alpha = PreviousDistance / (PreviousDistance - CurrentDistance);
		return ProjectionInsideTriangle(FMath::Lerp(Previous, Current, Alpha) - Normal * Radius, Triangle, Tolerance);
	}

	bool IsInsideClosedConvexShape(
		const FVector3d& Point,
		const CableSim::FCollisionTriangle& Reference,
		const TConstArrayView<CableSim::FCollisionTriangle> Triangles,
		const double Tolerance)
	{
		if (Reference.GeometryType != CableSim::ECollisionGeometryType::Box
			&& Reference.GeometryType != CableSim::ECollisionGeometryType::Convex)
		{
			return false;
		}
		bool bFoundFace = false;
		for (const CableSim::FCollisionTriangle& Triangle : Triangles)
		{
			if (Triangle.Id.ObjectToken != Reference.Id.ObjectToken
				|| Triangle.Id.ShapeIndex != Reference.Id.ShapeIndex)
			{
				continue;
			}
			const FVector3d Normal = Triangle.CalculateNormal();
			if (Normal.IsNearlyZero()) continue;
			bFoundFace = true;
			if (FVector3d::DotProduct(Point - Triangle.Vertices[0], Normal) > Tolerance)
			{
				return false;
			}
		}
		return bFoundFace;
	}

	struct FCandidateContact
	{
		CableSim::FContactConstraint Contact;
		double Penetration = 0.0;
		bool bEdge = false;
		bool bMovable = false;
		bool bInsideClosedConvex = false;
	};
}

void FCableSimCollisionSnapshot::Reset()
{
	QueryBounds = FBox(ForceInit);
	Nodes.Reset();
	ObjectMotions.Reset();
	Diagnostics = FCableSimCollisionDiagnostics{};
}

FVector3d FCableSimCollisionSnapshot::VelocityAtPoint(
	const uint64 ObjectToken,
	const FVector3d& Point) const
{
	const FCableSimCollisionObjectMotion* Motion = ObjectMotions.Find(ObjectToken);
	return Motion ? Motion->VelocityAtPoint(Point) : FVector3d::ZeroVector;
}

bool FCableSimCollisionSnapshot::IsMovable(const uint64 ObjectToken) const
{
	const FCableSimCollisionObjectMotion* Motion = ObjectMotions.Find(ObjectToken);
	return Motion && Motion->bMovable;
}

void FCableSimChaosObjectTracker::Reset()
{
	Records.Reset();
	NextToken = 1;
	SnapshotSerial = 0;
}

void FCableSimChaosObjectTracker::BeginSnapshot() { ++SnapshotSerial; }

void FCableSimChaosObjectTracker::EndSnapshot()
{
	for (auto Iterator = Records.CreateIterator(); Iterator; ++Iterator)
	{
		const FRecord& Record = Iterator.Value();
		if ((!Record.Owner.IsValid() && !Record.Owner.IsExplicitlyNull())
			|| (SnapshotSerial > Record.LastSeenSnapshot && SnapshotSerial - Record.LastSeenSnapshot > 600))
			Iterator.RemoveCurrent();
	}
}

uint64 FCableSimChaosObjectTracker::ResolveToken(
	Chaos::FConstPhysicsObjectHandle Handle,
	const TWeakObjectPtr<UObject>& Owner)
{
	if (!Handle) return 0;
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

bool FCableSimWorldCollisionProvider::GatherSnapshot(
	UWorld* World,
	AActor* Owner,
	const TConstArrayView<AActor*> IgnoredActors,
	const FCableSimCollisionSettings& Settings,
	const TConstArrayView<CableSim::FParticle> Particles,
	const FVector3d& StartTarget,
	const FVector3d& EndTarget,
	const double DeltaTime,
	FCableSimChaosObjectTracker& ObjectTracker,
	FCableSimCollisionSnapshot& OutSnapshot)
{
	OutSnapshot.Reset();
	if (!World || !Settings.bEnableWorldCollision
		|| (!Settings.bCollideWorldStatic && !Settings.bCollideWorldDynamic)
		|| Particles.Num() < 2)
	{
		return true;
	}
	const double StartTime = FPlatformTime::Seconds();
	ObjectTracker.BeginSnapshot();
	OutSnapshot.QueryBounds = BuildPredictedNodeBounds(
		Particles,
		StartTarget,
		EndTarget,
		FMath::Max(DeltaTime, 0.0),
		Settings.Radius + Settings.SkinWidth + Settings.ContactReleaseDistance,
		OutSnapshot.Nodes);
	// The overlap is performed before Chaos geometry is locked. Keep enough room
	// for a dynamic/kinematic collider to enter the cable bounds during this step.
	OutSnapshot.QueryBounds = OutSnapshot.QueryBounds.ExpandBy(
		FMath::Max(Settings.MaximumDynamicColliderSpeed, 0.0) * FMath::Max(DeltaTime, 0.0));
	FCollisionQueryParams QueryParams(SCENE_QUERY_STAT(CableSimStableGeometry), false, Owner);
	for (AActor* Ignored : IgnoredActors) if (IsValid(Ignored)) QueryParams.AddIgnoredActor(Ignored);
	TArray<FOverlapResult> Overlaps;
	const FVector Center = OutSnapshot.QueryBounds.GetCenter();
	const FVector Extent = OutSnapshot.QueryBounds.GetExtent().ComponentMax(FVector(0.1));
	QueryParams.bTraceComplex = Settings.bTraceComplex;
	++OutSnapshot.Diagnostics.QueryCount;
	World->OverlapMultiByChannel(
		Overlaps,
		Center,
		FQuat::Identity,
		Settings.TraceChannel,
		FCollisionShape::MakeBox(Extent),
		QueryParams);
	OutSnapshot.Diagnostics.OverlapCount = Overlaps.Num();
	Overlaps.StableSort([&](const FOverlapResult& A, const FOverlapResult& B)
	{
		const UPrimitiveComponent* ComponentA = A.Component.Get();
		const UPrimitiveComponent* ComponentB = B.Component.Get();
		const double DistanceA = ComponentA
			? ComponentA->Bounds.GetBox().ComputeSquaredDistanceToPoint(Center)
			: TNumericLimits<double>::Max();
		const double DistanceB = ComponentB
			? ComponentB->Bounds.GetBox().ComputeSquaredDistanceToPoint(Center)
			: TNumericLimits<double>::Max();
		return DistanceA < DistanceB;
	});

	TArray<Chaos::FConstPhysicsObjectHandle> Handles;
	TMap<Chaos::FConstPhysicsObjectHandle, TWeakObjectPtr<UObject>> Owners;
	for (const FOverlapResult& Overlap : Overlaps)
	{
		if (!Overlap.PhysicsObject || Handles.Contains(Overlap.PhysicsObject)) continue;
		if (Handles.Num() >= MaximumPhysicsObjects) { OutSnapshot.Diagnostics.bFeatureBudgetExceeded = true; break; }
		Handles.Add(Overlap.PhysicsObject);
		Owners.Add(Overlap.PhysicsObject, Overlap.PhysicsObjectOwner);
	}
	OutSnapshot.Diagnostics.PhysicsObjectCount = Handles.Num();
	if (!Handles.IsEmpty())
	{
		auto Locked = FPhysicsObjectExternalInterface::LockRead(Handles);
		auto& Interface = Locked.GetInterface();
		for (const Chaos::FConstPhysicsObjectHandle Handle : Handles)
		{
			const Chaos::FConstPhysicsObjectHandle Single = Handle;
			if (!Interface.AreAllValid(MakeArrayView(&Single, 1))) continue;
			const bool bKinematic = Interface.AreAllKinematic(MakeArrayView(&Single, 1));
			const bool bDynamic = Interface.AreAllDynamicOrSleeping(MakeArrayView(&Single, 1));
			const bool bMovable = bKinematic || bDynamic;
			if ((bMovable && !Settings.bCollideWorldDynamic)
				|| (!bMovable && !Settings.bCollideWorldStatic))
			{
				continue;
			}
			const uint64 Token = ObjectTracker.ResolveToken(Handle, Owners.FindRef(Handle));
			const FTransform ObjectTransform = Interface.GetTransform(Handle);
			FCableSimCollisionObjectMotion& Motion = OutSnapshot.ObjectMotions.FindOrAdd(Token);
			Motion.bMovable = bMovable;
			if (bMovable)
			{
				Motion.CentreOfMass = FVector3d(Interface.GetWorldCoM(Handle));
				Motion.LinearVelocity = FVector3d(Interface.GetV(Handle));
				Motion.AngularVelocity = FVector3d(Interface.GetW(Handle));
			}
			Interface.VisitEveryShape(MakeArrayView(&Single, 1),
				[&](const Chaos::FConstPhysicsObjectHandle, Chaos::TThreadShapeInstance<Chaos::EThreadContext::External>* Shape)
				{
					if (!Shape || !Shape->GetQueryEnabled() || !Intersects(Shape->GetWorldSpaceShapeBounds(), OutSnapshot.QueryBounds)) return false;
					if (++OutSnapshot.Diagnostics.ShapeCount > MaximumShapes) { OutSnapshot.Diagnostics.bFeatureBudgetExceeded = true; return true; }
					bool bIntersectedNode = false;
					bool bSupported = false;
					for (FCableSimNodeCollisionGeometry& Node : OutSnapshot.Nodes)
					{
						if (!Intersects(Shape->GetWorldSpaceShapeBounds(), Node.PredictedBounds))
						{
							continue;
						}
						bIntersectedNode = true;
						bSupported |= AddGeometry(
							Shape->GetGeometry(),
							ObjectTransform,
							Node.PredictedBounds,
							Token,
							Shape->GetShapeIndex(),
							Node);
						if (Node.Triangles.Num() >= MaximumTrianglesPerNode)
						{
							OutSnapshot.Diagnostics.bFeatureBudgetExceeded = true;
						}
					}
					if (bIntersectedNode && !bSupported) ++OutSnapshot.Diagnostics.UnsupportedShapeCount;
					return false;
				});
			if (OutSnapshot.Diagnostics.ShapeCount > MaximumShapes) break;
		}
		Locked.Release();
	}
	ObjectTracker.EndSnapshot();
	OutSnapshot.Diagnostics.SnapshotMilliseconds = (FPlatformTime::Seconds() - StartTime) * 1000.0;
	const double CompileStart = FPlatformTime::Seconds();
	for (FCableSimNodeCollisionGeometry& Node : OutSnapshot.Nodes)
	{
		OutSnapshot.Diagnostics.TriangleCount += Node.Triangles.Num();
		const CableSim::FTopologyDiagnostics Topology = CableSim::FCollisionTopologyCompiler::CompileEdges(
			Node.Triangles, Settings.TopologyTolerance, Node.Edges);
		OutSnapshot.Diagnostics.ConvexEdgeCount += Topology.ConvexEdges;
	}
	OutSnapshot.Diagnostics.CompileMilliseconds = (FPlatformTime::Seconds() - CompileStart) * 1000.0;
	return true;
}

void FCableSimWorldCollisionProvider::CompileContacts(
	const FCableSimCollisionSnapshot& Snapshot,
	const FCableSimCollisionSettings& CollisionSettings,
	const FCableSimFrictionSettings& FrictionSettings,
	const CableSim::FSolver& Solver,
	const CableSim::FStepInput& Input,
	TArray<CableSim::FContactConstraint>& OutContacts,
	TArray<FVector3d>& OutRejectedPoints,
	FCableSimCollisionDiagnostics& InOutDiagnostics)
{
	const double StartTime = FPlatformTime::Seconds();
	OutContacts.Reset();
	OutRejectedPoints.Reset();
	if (!CollisionSettings.bEnableWorldCollision || !Solver.IsInitialized()) return;
	const double Radius = FMath::Max(CollisionSettings.Radius + CollisionSettings.SkinWidth, 0.1);
	const double Release = FMath::Max(CollisionSettings.ContactReleaseDistance, 0.0);
		const double Tolerance = FMath::Max(CollisionSettings.TopologyTolerance, 0.01);
		const TArray<CableSim::FParticle>& Particles = Solver.GetParticles();
		for (int32 SampleIndex = 0; SampleIndex < Particles.Num(); ++SampleIndex)
		{
			// A fixed attachment is deliberately allowed to sit on/in its support.
			// Adjacent segment contacts still make the rest of the cable collide;
			// ignoring the attachment's whole actor would incorrectly remove that
			// actor from collision everywhere along the cable.
			if ((SampleIndex == 0 && Input.StartEndpoint.State == CableSim::EEndpointState::Fixed)
				|| (SampleIndex + 1 == Particles.Num()
					&& Input.EndEndpoint.State == CableSim::EEndpointState::Fixed))
			{
				continue;
			}
			if (!Snapshot.Nodes.IsValidIndex(SampleIndex)) continue;
			const FCableSimNodeCollisionGeometry& NodeGeometry = Snapshot.Nodes[SampleIndex];
		const double Coordinate = Particles[SampleIndex].MaterialCoordinate;
		const FVector3d Current = Particles[SampleIndex].Position;
		const FVector3d Previous = Particles[SampleIndex].PreviousPosition;
		const FVector3d PredictedContactPosition = Current;
		TArray<FCandidateContact, TInlineAllocator<16>> Candidates;

		for (const CableSim::FCollisionTriangle& Triangle : NodeGeometry.Triangles)
		{
			const FVector3d Normal = Triangle.CalculateNormal();
			if (Normal.IsNearlyZero()) continue;
			const double SurfacePlane = FVector3d::DotProduct(Triangle.Vertices[0], Normal);
			const double Signed = FVector3d::DotProduct(PredictedContactPosition, Normal) - SurfacePlane;
			const FVector3d Projection = PredictedContactPosition - Normal * Signed;
			const bool bInsideSupport = ProjectionInsideTriangle(Projection, Triangle, Tolerance);
			const FVector3d SurfaceVelocity = Snapshot.VelocityAtPoint(Triangle.Id.ObjectToken, Projection);
			const FVector3d RelativePrevious = Previous + SurfaceVelocity * Input.DeltaTime;
			const bool bCrossed = CrossesTrianglePlane(RelativePrevious, PredictedContactPosition, Triangle, Normal, Radius, Tolerance);
			if (!bInsideSupport && !bCrossed) continue;
			const bool bInsideClosedConvex = IsInsideClosedConvexShape(
				PredictedContactPosition,
				Triangle,
				NodeGeometry.Triangles,
				Tolerance);
			if (!bCrossed && Signed < -CollisionSettings.MaximumSafeCorrection && !bInsideClosedConvex)
			{
				continue;
			}
			if (!bCrossed && Signed > Radius + Release) continue;
			FCandidateContact Candidate;
			Candidate.bMovable = Snapshot.IsMovable(Triangle.Id.ObjectToken);
			Candidate.bInsideClosedConvex = bInsideClosedConvex;
			Candidate.Contact.FeatureId = Triangle.Id;
			Candidate.Contact.SampleIndex = SampleIndex;
			Candidate.Contact.ParticleA = SampleIndex;
			Candidate.Contact.ParticleB = INDEX_NONE;
			Candidate.Contact.SegmentAlpha = 0.0;
			Candidate.Contact.Normal = Normal;
			Candidate.Contact.MinimumNormalCoordinate = SurfacePlane + Radius;
			Candidate.Contact.SurfaceVelocity = SurfaceVelocity;
			Candidate.Contact.FrictionAnchor = Previous + Candidate.Contact.SurfaceVelocity * Input.DeltaTime;
			Candidate.Contact.bEnableFriction = FrictionSettings.bEnableFriction;
			Candidate.Penetration = Candidate.Contact.MinimumNormalCoordinate - FVector3d::DotProduct(PredictedContactPosition, Normal);
			Candidates.Add(Candidate);
		}

		for (const CableSim::FCollisionEdge& Edge : NodeGeometry.Edges)
		{
			if (Edge.Kind != CableSim::ECollisionEdgeKind::Convex) continue;
			FVector3d ClosestMotion;
			FVector3d Closest;
			const FVector3d EdgeVelocity = Snapshot.VelocityAtPoint(
				Edge.Id.ObjectToken,
				0.5 * (Edge.Start + Edge.End));
			FMath::SegmentDistToSegmentSafe(
				Previous + EdgeVelocity * Input.DeltaTime,
				PredictedContactPosition,
				Edge.Start,
				Edge.End,
				ClosestMotion,
				Closest);
			const FVector3d Radial = ClosestMotion - Closest;
			const double Distance = Radial.Length();
			if (Distance > Radius + Release) continue;
			const double Side0 = FVector3d::DotProduct(ClosestMotion - Edge.Start, Edge.FaceNormal0);
			const double Side1 = FVector3d::DotProduct(ClosestMotion - Edge.Start, Edge.FaceNormal1);
			if (Side0 > Radius + Release || Side1 > Radius + Release) continue;
			FCandidateContact Candidate;
			Candidate.bEdge = true;
			Candidate.bMovable = Snapshot.IsMovable(Edge.Id.ObjectToken);
			Candidate.Contact.FeatureId = Edge.Id;
			Candidate.Contact.SampleIndex = SampleIndex;
			Candidate.Contact.ParticleA = SampleIndex;
			Candidate.Contact.ParticleB = INDEX_NONE;
			Candidate.Contact.SegmentAlpha = 0.0;
			Candidate.Contact.Normal = Distance > 1.e-6
				? Radial / Distance
				: (Edge.FaceNormal0 + Edge.FaceNormal1).GetSafeNormal();
			if (Candidate.Contact.Normal.IsNearlyZero()) continue;
			Candidate.Contact.MinimumNormalCoordinate = FVector3d::DotProduct(Closest, Candidate.Contact.Normal) + Radius;
			Candidate.Contact.SurfaceVelocity = Snapshot.VelocityAtPoint(Edge.Id.ObjectToken, Closest);
			Candidate.Contact.FrictionAnchor = Previous + Candidate.Contact.SurfaceVelocity * Input.DeltaTime;
			Candidate.Contact.bEnableFriction = FrictionSettings.bEnableFriction;
			Candidate.Penetration = Radius - Distance;
			Candidates.Add(Candidate);
		}

		for (const FCableSimCollisionSphere& Sphere : NodeGeometry.Spheres)
		{
			const double CombinedRadius = Radius + Sphere.Radius;
			const FVector3d SphereVelocity = Snapshot.VelocityAtPoint(Sphere.Id.ObjectToken, Sphere.Centre);
			const FVector3d ClosestPath = ClosestPointOnSegment(
				Sphere.Centre,
				Previous + SphereVelocity * Input.DeltaTime,
				PredictedContactPosition);
			const bool bSwept = FVector3d::Distance(ClosestPath, Sphere.Centre) <= CombinedRadius;
			FVector3d Radial = PredictedContactPosition - Sphere.Centre;
			const double Distance = Radial.Length();
			if (!bSwept && Distance > CombinedRadius + Release) continue;
			if (bSwept && Distance > CombinedRadius + Release)
			{
				Radial = Previous + SphereVelocity * Input.DeltaTime - Sphere.Centre;
			}
			FVector3d Normal = Radial.GetSafeNormal();
			if (Normal.IsNearlyZero()) Normal = FVector3d::UnitZ();
			FCandidateContact Candidate;
			Candidate.bMovable = Snapshot.IsMovable(Sphere.Id.ObjectToken);
			Candidate.Contact.FeatureId = Sphere.Id;
			Candidate.Contact.SampleIndex = SampleIndex;
			Candidate.Contact.ParticleA = SampleIndex;
			Candidate.Contact.Normal = Normal;
			Candidate.Contact.MinimumNormalCoordinate = FVector3d::DotProduct(Sphere.Centre, Normal) + CombinedRadius;
			const FVector3d SurfacePoint = Sphere.Centre + Normal * Sphere.Radius;
			Candidate.Contact.SurfaceVelocity = Snapshot.VelocityAtPoint(Sphere.Id.ObjectToken, SurfacePoint);
			Candidate.Contact.FrictionAnchor = Previous + Candidate.Contact.SurfaceVelocity * Input.DeltaTime;
			Candidate.Contact.bEnableFriction = FrictionSettings.bEnableFriction;
			Candidate.Penetration = Candidate.Contact.MinimumNormalCoordinate - FVector3d::DotProduct(PredictedContactPosition, Normal);
			Candidates.Add(Candidate);
		}

		for (const FCableSimCollisionCapsule& Capsule : NodeGeometry.Capsules)
		{
			FVector3d ClosestMotion;
			FVector3d AxisPoint;
			FMath::SegmentDistToSegmentSafe(
				Previous + Snapshot.VelocityAtPoint(Capsule.Id.ObjectToken, 0.5 * (Capsule.Start + Capsule.End)) * Input.DeltaTime,
				PredictedContactPosition,
				Capsule.Start,
				Capsule.End,
				ClosestMotion,
				AxisPoint);
			const FVector3d Radial = ClosestMotion - AxisPoint;
			const double Distance = Radial.Length();
			const double CombinedRadius = Radius + Capsule.Radius;
			if (Distance > CombinedRadius + Release) continue;
			FCandidateContact Candidate;
			Candidate.bMovable = Snapshot.IsMovable(Capsule.Id.ObjectToken);
			Candidate.Contact.FeatureId = Capsule.Id;
			Candidate.Contact.SampleIndex = SampleIndex;
			Candidate.Contact.ParticleA = SampleIndex;
			Candidate.Contact.Normal = Radial.GetSafeNormal();
			if (Candidate.Contact.Normal.IsNearlyZero())
			{
				const FVector3d Axis = (Capsule.End - Capsule.Start).GetSafeNormal();
				Candidate.Contact.Normal = FVector3d::CrossProduct(
					Axis,
					FMath::Abs(Axis.Z) < 0.9 ? FVector3d::UnitZ() : FVector3d::UnitY()).GetSafeNormal();
			}
			Candidate.Contact.MinimumNormalCoordinate = FVector3d::DotProduct(AxisPoint, Candidate.Contact.Normal) + CombinedRadius;
			const FVector3d SurfacePoint = AxisPoint + Candidate.Contact.Normal * Capsule.Radius;
			Candidate.Contact.SurfaceVelocity = Snapshot.VelocityAtPoint(Capsule.Id.ObjectToken, SurfacePoint);
			Candidate.Contact.FrictionAnchor = Previous + Candidate.Contact.SurfaceVelocity * Input.DeltaTime;
			Candidate.Contact.bEnableFriction = FrictionSettings.bEnableFriction;
			Candidate.Penetration = CombinedRadius - Distance;
			Candidates.Add(Candidate);
		}

		Candidates.Sort([](const FCandidateContact& A, const FCandidateContact& B)
		{
			// Pinching prevention from the GDC talk: static world geometry is
			// authoritative when animated collision produces a conflicting plane.
			if (A.bMovable != B.bMovable) return !A.bMovable;
			if (A.bInsideClosedConvex && B.bInsideClosedConvex && A.Penetration != B.Penetration)
				return A.Penetration < B.Penetration;
			if (A.Penetration != B.Penetration) return A.Penetration > B.Penetration;
			return CableSim::FCollisionFeatureId::Less(A.Contact.FeatureId, B.Contact.FeatureId);
		});
		TArray<FCandidateContact, TInlineAllocator<7>> Selected;
		int32 FaceCount = 0;
		int32 EdgeCount = 0;
		for (const FCandidateContact& Candidate : Candidates)
		{
			bool bReject = false;
			for (const FCandidateContact& Existing : Selected)
			{
				if (Candidate.bInsideClosedConvex && Existing.bInsideClosedConvex
					&& Candidate.Contact.FeatureId.ObjectToken == Existing.Contact.FeatureId.ObjectToken
					&& Candidate.Contact.FeatureId.ShapeIndex == Existing.Contact.FeatureId.ShapeIndex)
				{
					bReject = true;
					break;
				}
				const double NormalDot = FVector3d::DotProduct(Candidate.Contact.Normal, Existing.Contact.Normal);
				if (!Candidate.bEdge && !Existing.bEdge && NormalDot >= NormalMergeCosine
					&& FMath::Abs(Candidate.Contact.MinimumNormalCoordinate - Existing.Contact.MinimumNormalCoordinate) <= Tolerance)
				{
					bReject = true;
					break;
				}
				if (NormalDot < -0.5 && Candidate.Penetration > 0.0 && Existing.Penetration > 0.0)
				{
					bReject = true;
					break;
				}
			}
			// Reject any plane that a hard keyframe cannot reach through the
			// intervening material arc. This applies to static and moving shapes.
			{
				double RelevantArcLength = TNumericLimits<double>::Max();
				FVector3d RelevantKeyframe = FVector3d::ZeroVector;
				if (Input.StartEndpoint.State == CableSim::EEndpointState::Fixed)
				{
					RelevantArcLength = Coordinate;
					RelevantKeyframe = Input.StartEndpoint.TargetPosition;
				}
				const double ArcFromEnd = Solver.GetActiveLength() - Coordinate;
				if (Input.EndEndpoint.State == CableSim::EEndpointState::Fixed && ArcFromEnd < RelevantArcLength)
				{
					RelevantArcLength = ArcFromEnd;
					RelevantKeyframe = Input.EndEndpoint.TargetPosition;
				}
				if (RelevantArcLength < TNumericLimits<double>::Max())
				{
					const double PlaneDistance = FMath::Abs(
						Candidate.Contact.MinimumNormalCoordinate
						- FVector3d::DotProduct(RelevantKeyframe, Candidate.Contact.Normal));
					bReject |= PlaneDistance > RelevantArcLength + Tolerance;
				}
			}
			if ((!Candidate.bEdge && FaceCount >= FMath::Clamp(CollisionSettings.MaximumContactsPerElement, 1, 4))
				|| (Candidate.bEdge && EdgeCount >= FMath::Clamp(CollisionSettings.MaximumEdgesPerElement, 1, 3))) bReject = true;
			if (bReject)
			{
				OutRejectedPoints.Add(Current);
				++InOutDiagnostics.RejectedContactCount;
				continue;
			}
			Selected.Add(Candidate);
			Candidate.bEdge ? ++EdgeCount : ++FaceCount;
		}
		for (const FCandidateContact& Candidate : Selected) OutContacts.Add(Candidate.Contact);
	}

	// Particle-only contacts leave a blind span between samples. Compile a
	// second, barycentric contact manifold for each cable segment so a thin box,
	// edge, sphere, or capsule cannot sit between two otherwise valid nodes.
	for (int32 SegmentIndex = 0; SegmentIndex + 1 < Particles.Num(); ++SegmentIndex)
	{
		if (!Snapshot.Nodes.IsValidIndex(SegmentIndex)) continue;
		const FCableSimNodeCollisionGeometry& Geometry = Snapshot.Nodes[SegmentIndex];
		const FVector3d Start = Particles[SegmentIndex].Position;
		const FVector3d End = Particles[SegmentIndex + 1].Position;
		const FVector3d PreviousStart = Particles[SegmentIndex].PreviousPosition;
		const FVector3d PreviousEnd = Particles[SegmentIndex + 1].PreviousPosition;
		if ((End - Start).SquaredLength() <= 1.e-12) continue;

		TArray<FCandidateContact, TInlineAllocator<16>> Candidates;
		TArray<CableSim::FCollisionFeatureId, TInlineAllocator<32>> SeenFeatures;
		auto BeginCandidate = [&](const CableSim::FCollisionFeatureId& Feature, const double Alpha)
			-> FCandidateContact*
		{
			if (SeenFeatures.Contains(Feature)) return nullptr;
			SeenFeatures.Add(Feature);
			FCandidateContact& Candidate = Candidates.AddDefaulted_GetRef();
			Candidate.bMovable = Snapshot.IsMovable(Feature.ObjectToken);
			Candidate.Contact.FeatureId = Feature;
			Candidate.Contact.SampleIndex = SegmentIndex;
			Candidate.Contact.ParticleA = SegmentIndex;
			Candidate.Contact.ParticleB = SegmentIndex + 1;
			Candidate.Contact.SegmentAlpha = FMath::Clamp(Alpha, 0.0, 1.0);
			// Segment friction needs a material-point anchor, rather than either
			// endpoint anchor. Leave it to the adjacent node contacts for now.
			Candidate.Contact.bEnableFriction = false;
			return &Candidate;
		};

		for (const CableSim::FCollisionTriangle& Triangle : Geometry.Triangles)
		{
			FVector Intersection;
			FVector IgnoredNormal;
			if (!FMath::SegmentTriangleIntersection(
				FVector(Start), FVector(End),
				FVector(Triangle.Vertices[0]), FVector(Triangle.Vertices[1]), FVector(Triangle.Vertices[2]),
				Intersection, IgnoredNormal))
			{
				continue;
			}
			const FVector3d Normal = Triangle.CalculateNormal();
			if (Normal.IsNearlyZero()) continue;
			const FVector3d Hit(Intersection);
			const double Alpha = SegmentAlphaAtPoint(Start, End, Hit);
			const FVector3d PreviousPoint = FMath::Lerp(PreviousStart, PreviousEnd, Alpha);
			const FVector3d CurrentPoint = FMath::Lerp(Start, End, Alpha);
			const double IntoSurfaceSpeed = FVector3d::DotProduct(CurrentPoint - PreviousPoint, Normal);
			const double PreviousSide = FVector3d::DotProduct(PreviousPoint - Triangle.Vertices[0], Normal);
			// A static chord piercing a closed body has no well-defined route. Do
			// not manufacture two opposing escape planes; continuous contacts are
			// activated by a known previous outside side.
			if (PreviousSide < -Tolerance || IntoSurfaceSpeed >= -1.e-9) continue;
			FCandidateContact* Candidate = BeginCandidate(Triangle.Id, Alpha);
			if (!Candidate) continue;
			Candidate->Contact.Normal = Normal;
			Candidate->Contact.MinimumNormalCoordinate =
				FVector3d::DotProduct(Triangle.Vertices[0], Normal) + Radius;
			Candidate->Contact.SurfaceVelocity = Snapshot.VelocityAtPoint(Triangle.Id.ObjectToken, Hit);
			Candidate->Contact.FrictionAnchor = PreviousPoint + Candidate->Contact.SurfaceVelocity * Input.DeltaTime;
			Candidate->Penetration = Radius;
		}

		for (const CableSim::FCollisionEdge& Edge : Geometry.Edges)
		{
			if (Edge.Kind != CableSim::ECollisionEdgeKind::Convex) continue;
			FVector3d CablePoint;
			FVector3d EdgePoint;
			FMath::SegmentDistToSegmentSafe(Start, End, Edge.Start, Edge.End, CablePoint, EdgePoint);
			FVector3d Radial = CablePoint - EdgePoint;
			const double Distance = Radial.Length();
			if (Distance > Radius + Release) continue;
			const double Alpha = SegmentAlphaAtPoint(Start, End, CablePoint);
			if (Distance <= 1.e-6)
			{
				const FVector3d PreviousPoint = FMath::Lerp(PreviousStart, PreviousEnd, Alpha);
				Radial = PreviousPoint - EdgePoint;
			}
			FVector3d Normal = Radial.GetSafeNormal();
			if (Normal.IsNearlyZero()) Normal = (Edge.FaceNormal0 + Edge.FaceNormal1).GetSafeNormal();
			if (Normal.IsNearlyZero()) continue;
			const double Side0 = FVector3d::DotProduct(CablePoint - Edge.Start, Edge.FaceNormal0);
			const double Side1 = FVector3d::DotProduct(CablePoint - Edge.Start, Edge.FaceNormal1);
			if (Side0 > Radius + Release || Side1 > Radius + Release) continue;
			FCandidateContact* Candidate = BeginCandidate(Edge.Id, Alpha);
			if (!Candidate) continue;
			Candidate->bEdge = true;
			Candidate->Contact.Normal = Normal;
			Candidate->Contact.MinimumNormalCoordinate = FVector3d::DotProduct(EdgePoint, Normal) + Radius;
			Candidate->Contact.SurfaceVelocity = Snapshot.VelocityAtPoint(Edge.Id.ObjectToken, EdgePoint);
			Candidate->Contact.FrictionAnchor = FMath::Lerp(PreviousStart, PreviousEnd, Alpha)
				+ Candidate->Contact.SurfaceVelocity * Input.DeltaTime;
			Candidate->Penetration = Radius - Distance;
		}

		for (const FCableSimCollisionSphere& Sphere : Geometry.Spheres)
		{
			const FVector3d CablePoint = ClosestPointOnSegment(Sphere.Centre, Start, End);
			const double Alpha = SegmentAlphaAtPoint(Start, End, CablePoint);
			FVector3d Radial = CablePoint - Sphere.Centre;
			const double Distance = Radial.Length();
			const double CombinedRadius = Radius + Sphere.Radius;
			if (Distance > CombinedRadius + Release) continue;
			if (Distance <= 1.e-6) Radial = FMath::Lerp(PreviousStart, PreviousEnd, Alpha) - Sphere.Centre;
			FVector3d Normal = Radial.GetSafeNormal();
			if (Normal.IsNearlyZero()) Normal = FVector3d::UnitZ();
			FCandidateContact* Candidate = BeginCandidate(Sphere.Id, Alpha);
			if (!Candidate) continue;
			Candidate->Contact.Normal = Normal;
			Candidate->Contact.MinimumNormalCoordinate = FVector3d::DotProduct(Sphere.Centre, Normal) + CombinedRadius;
			const FVector3d SurfacePoint = Sphere.Centre + Normal * Sphere.Radius;
			Candidate->Contact.SurfaceVelocity = Snapshot.VelocityAtPoint(Sphere.Id.ObjectToken, SurfacePoint);
			Candidate->Contact.FrictionAnchor = FMath::Lerp(PreviousStart, PreviousEnd, Alpha)
				+ Candidate->Contact.SurfaceVelocity * Input.DeltaTime;
			Candidate->Penetration = CombinedRadius - Distance;
		}

		for (const FCableSimCollisionCapsule& Capsule : Geometry.Capsules)
		{
			FVector3d CablePoint;
			FVector3d AxisPoint;
			FMath::SegmentDistToSegmentSafe(Start, End, Capsule.Start, Capsule.End, CablePoint, AxisPoint);
			const double Alpha = SegmentAlphaAtPoint(Start, End, CablePoint);
			FVector3d Radial = CablePoint - AxisPoint;
			const double Distance = Radial.Length();
			const double CombinedRadius = Radius + Capsule.Radius;
			if (Distance > CombinedRadius + Release) continue;
			if (Distance <= 1.e-6) Radial = FMath::Lerp(PreviousStart, PreviousEnd, Alpha) - AxisPoint;
			FVector3d Normal = Radial.GetSafeNormal();
			if (Normal.IsNearlyZero())
			{
				const FVector3d Axis = (Capsule.End - Capsule.Start).GetSafeNormal();
				Normal = FVector3d::CrossProduct(
					Axis,
					FMath::Abs(Axis.Z) < 0.9 ? FVector3d::UnitZ() : FVector3d::UnitY()).GetSafeNormal();
			}
			if (Normal.IsNearlyZero()) continue;
			FCandidateContact* Candidate = BeginCandidate(Capsule.Id, Alpha);
			if (!Candidate) continue;
			Candidate->Contact.Normal = Normal;
			Candidate->Contact.MinimumNormalCoordinate = FVector3d::DotProduct(AxisPoint, Normal) + CombinedRadius;
			const FVector3d SurfacePoint = AxisPoint + Normal * Capsule.Radius;
			Candidate->Contact.SurfaceVelocity = Snapshot.VelocityAtPoint(Capsule.Id.ObjectToken, SurfacePoint);
			Candidate->Contact.FrictionAnchor = FMath::Lerp(PreviousStart, PreviousEnd, Alpha)
				+ Candidate->Contact.SurfaceVelocity * Input.DeltaTime;
			Candidate->Penetration = CombinedRadius - Distance;
		}

		Candidates.Sort([](const FCandidateContact& A, const FCandidateContact& B)
		{
			if (A.bMovable != B.bMovable) return !A.bMovable;
			if (A.Penetration != B.Penetration) return A.Penetration > B.Penetration;
			return CableSim::FCollisionFeatureId::Less(A.Contact.FeatureId, B.Contact.FeatureId);
		});
		TArray<FCandidateContact, TInlineAllocator<7>> Selected;
		int32 FaceCount = 0;
		int32 EdgeCount = 0;
		for (const FCandidateContact& Candidate : Candidates)
		{
			bool bReject = false;
			for (const FCandidateContact& Existing : Selected)
			{
				const double NormalDot = FVector3d::DotProduct(Candidate.Contact.Normal, Existing.Contact.Normal);
				if (NormalDot >= NormalMergeCosine
					&& FMath::Abs(Candidate.Contact.MinimumNormalCoordinate - Existing.Contact.MinimumNormalCoordinate) <= Tolerance)
				{
					bReject = true;
					break;
				}
				if (NormalDot < -0.5 && Candidate.Penetration > 0.0 && Existing.Penetration > 0.0)
				{
					bReject = true;
					break;
				}
			}
			const double Coordinate = FMath::Lerp(
				Particles[SegmentIndex].MaterialCoordinate,
				Particles[SegmentIndex + 1].MaterialCoordinate,
				Candidate.Contact.SegmentAlpha);
			double RelevantArcLength = TNumericLimits<double>::Max();
			FVector3d RelevantKeyframe = FVector3d::ZeroVector;
			if (Input.StartEndpoint.State == CableSim::EEndpointState::Fixed)
			{
				RelevantArcLength = Coordinate;
				RelevantKeyframe = Input.StartEndpoint.TargetPosition;
			}
			const double ArcFromEnd = Solver.GetActiveLength() - Coordinate;
			if (Input.EndEndpoint.State == CableSim::EEndpointState::Fixed && ArcFromEnd < RelevantArcLength)
			{
				RelevantArcLength = ArcFromEnd;
				RelevantKeyframe = Input.EndEndpoint.TargetPosition;
			}
			if (RelevantArcLength < TNumericLimits<double>::Max())
			{
				const double PlaneDistance = FMath::Abs(
					Candidate.Contact.MinimumNormalCoordinate
					- FVector3d::DotProduct(RelevantKeyframe, Candidate.Contact.Normal));
				bReject |= PlaneDistance > RelevantArcLength + Tolerance;
			}
			if ((!Candidate.bEdge && FaceCount >= FMath::Clamp(CollisionSettings.MaximumContactsPerElement, 1, 4))
				|| (Candidate.bEdge && EdgeCount >= FMath::Clamp(CollisionSettings.MaximumEdgesPerElement, 1, 3)))
			{
				bReject = true;
			}
			if (bReject)
			{
				OutRejectedPoints.Add(FMath::Lerp(Start, End, Candidate.Contact.SegmentAlpha));
				++InOutDiagnostics.RejectedContactCount;
				continue;
			}
			Selected.Add(Candidate);
			Candidate.bEdge ? ++EdgeCount : ++FaceCount;
		}
		for (const FCandidateContact& Candidate : Selected) OutContacts.Add(Candidate.Contact);
	}
	InOutDiagnostics.ContactCount = OutContacts.Num();
	InOutDiagnostics.CompileMilliseconds += (FPlatformTime::Seconds() - StartTime) * 1000.0;
}
