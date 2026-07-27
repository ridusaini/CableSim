#pragma once

#include "Chaos/PhysicsObject.h"
#include "CableSimCollisionGeometry.h"
#include "CableSimComponent.h"

struct FCableSimChaosSnapshotDiagnostics
{
	int32 OverlapCount = 0;
	int32 PhysicsObjectCount = 0;
	int32 ShapeCount = 0;
	int32 TriangleCount = 0;
	int32 EdgeCount = 0;
	int32 VertexCount = 0;
	int32 OverValenceVertexCount = 0;
	int32 UnsupportedShapeCount = 0;
	int32 DiscardedFeatureCount = 0;
	double SnapshotMilliseconds = 0.0;
	double CompileMilliseconds = 0.0;
	bool bFeatureBudgetExceeded = false;
};

struct FCableSimChaosSnapshot
{
	FBox QueryBounds = FBox(ForceInit);
	TArray<CableSim::FCollisionTriangle> Triangles;
	TArray<CableSim::FCollisionEdge> Edges;
	TArray<CableSim::FCollisionVertex> Vertices;
	FCableSimChaosSnapshotDiagnostics Diagnostics;

	void Reset();
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

class FCableSimChaosCollisionAdapter
{
public:
	static bool GatherSnapshot(
		UWorld* World,
		AActor* Owner,
		TConstArrayView<AActor*> IgnoredActors,
		const FCableSimCollisionSettings& CollisionSettings,
		double TopologyTolerance,
		TConstArrayView<CableSim::FParticle> Particles,
		const FVector3d& StartTarget,
		const FVector3d& EndTarget,
		FCableSimChaosObjectTracker& ObjectTracker,
		FCableSimChaosSnapshot& OutSnapshot);
};
