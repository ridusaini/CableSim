#pragma once

#include "CableSimComponent.h"

struct FCableSimCachedContact
{
	uint64 FeatureId = 0;
	int32 ParticleIndex = INDEX_NONE;
	TWeakObjectPtr<UPrimitiveComponent> Collider;
	int32 FaceIndex = INDEX_NONE;
	int32 Item = INDEX_NONE;
	FVector3d Normal = FVector3d::UnitZ();
	double MinimumNormalCoordinate = 0.0;
	FVector3d ColliderLocalPlanePoint = FVector3d::ZeroVector;
	FVector3d ColliderLocalNormal = FVector3d::UnitZ();
	FVector3d SurfaceVelocity = FVector3d::ZeroVector;
	FVector3d FrictionAnchorPosition = FVector3d::ZeroVector;
	FVector3d ColliderLocalFrictionAnchor = FVector3d::ZeroVector;
	int32 MissedStepCount = 0;
	bool bSeenThisStep = false;
	bool bSeenThisPass = false;
};

struct FCableSimContactCache
{
	TArray<FCableSimCachedContact> Contacts;
	int32 AdditionCount = 0;
	int32 RemovalCount = 0;

	void Reset();
	void BeginStep();
	void BeginPass();
	void EndStep(int32 PersistenceSteps);
};

class FCableSimWorldCollisionAdapter
{
public:
	static void GatherContacts(
		UWorld* World,
		AActor* Owner,
		TConstArrayView<AActor*> IgnoredActors,
		const FCableSimCollisionSettings& Settings,
		const FCableSimFrictionSettings& FrictionSettings,
		TConstArrayView<CableSim::FParticle> Particles,
		FCableSimContactCache& ContactCache,
		TArray<CableSim::FContactConstraint>& OutContacts);
};
