#pragma once

#include "CableSimCollisionGeometry.h"
#include "CableSimComponent.h"
#include "CableSimSolver.h"
#include "Chaos/PhysicsObject.h"

struct FCableSimCollisionDiagnostics
{
	int32 QueryCount = 0;
	int32 OverlapCount = 0;
	int32 PhysicsObjectCount = 0;
	int32 ShapeCount = 0;
	int32 TriangleCount = 0;
	int32 ConvexEdgeCount = 0;
	int32 ContactCount = 0;
	int32 RejectedContactCount = 0;
	int32 UnsupportedShapeCount = 0;
	bool bFeatureBudgetExceeded = false;
	double SnapshotMilliseconds = 0.0;
	double CompileMilliseconds = 0.0;
};

struct FCableSimCollisionObjectMotion
{
	FVector3d CentreOfMass = FVector3d::ZeroVector;
	FVector3d LinearVelocity = FVector3d::ZeroVector;
	FVector3d AngularVelocity = FVector3d::ZeroVector;
	bool bMovable = false;

	FVector3d VelocityAtPoint(const FVector3d& Point) const
	{
		return bMovable
			? LinearVelocity + FVector3d::CrossProduct(AngularVelocity, Point - CentreOfMass)
			: FVector3d::ZeroVector;
	}
};

struct FCableSimCollisionSphere
{
	CableSim::FCollisionFeatureId Id;
	FVector3d Centre = FVector3d::ZeroVector;
	double Radius = 0.0;
};

struct FCableSimCollisionCapsule
{
	CableSim::FCollisionFeatureId Id;
	FVector3d Start = FVector3d::ZeroVector;
	FVector3d End = FVector3d::ZeroVector;
	double Radius = 0.0;
};

struct FCableSimNodeCollisionGeometry
{
	FBox PredictedBounds = FBox(ForceInit);
	TArray<CableSim::FCollisionTriangle> Triangles;
	TArray<CableSim::FCollisionEdge> Edges;
	TArray<FCableSimCollisionSphere> Spheres;
	TArray<FCableSimCollisionCapsule> Capsules;
};

struct FCableSimCollisionSnapshot
{
	FBox QueryBounds = FBox(ForceInit);
	TArray<FCableSimNodeCollisionGeometry> Nodes;
	TMap<uint64, FCableSimCollisionObjectMotion> ObjectMotions;
	FCableSimCollisionDiagnostics Diagnostics;

	void Reset();
	FVector3d VelocityAtPoint(uint64 ObjectToken, const FVector3d& Point) const;
	bool IsMovable(uint64 ObjectToken) const;
};

class FCableSimChaosObjectTracker
{
public:
	void Reset();
	void BeginSnapshot();
	void EndSnapshot();
	uint64 ResolveToken(Chaos::FConstPhysicsObjectHandle Handle, const TWeakObjectPtr<UObject>& Owner);

private:
	struct FRecord
	{
		TWeakObjectPtr<UObject> Owner;
		uint64 Token = 0;
		uint64 LastSeenSnapshot = 0;
	};
	TMap<const void*, FRecord> Records;
	uint64 NextToken = 1;
	uint64 SnapshotSerial = 0;
};

class FCableSimWorldCollisionProvider
{
public:
	static bool GatherSnapshot(
		UWorld* World,
		AActor* Owner,
		TConstArrayView<AActor*> IgnoredActors,
		const FCableSimCollisionSettings& Settings,
		TConstArrayView<CableSim::FParticle> Particles,
		const FVector3d& StartTarget,
		const FVector3d& EndTarget,
		double DeltaTime,
		FCableSimChaosObjectTracker& ObjectTracker,
		FCableSimCollisionSnapshot& OutSnapshot);

	static void CompileContacts(
		const FCableSimCollisionSnapshot& Snapshot,
		const FCableSimCollisionSettings& CollisionSettings,
		const FCableSimFrictionSettings& FrictionSettings,
		const CableSim::FSolver& Solver,
		const CableSim::FStepInput& Input,
		TArray<CableSim::FContactConstraint>& OutContacts,
		TArray<FVector3d>& OutRejectedPoints,
		FCableSimCollisionDiagnostics& InOutDiagnostics);
};
