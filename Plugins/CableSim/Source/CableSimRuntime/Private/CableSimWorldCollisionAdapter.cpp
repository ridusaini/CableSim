#include "CableSimWorldCollisionAdapter.h"

#include "CollisionQueryParams.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/World.h"

namespace
{
	uint64 MakeContactFeatureId(const FHitResult& Hit, const FVector3d& Normal)
	{
		const uint32 ObjectHash = PointerHash(Hit.Component.Get());
		uint32 FeatureHash = HashCombineFast(GetTypeHash(Hit.FaceIndex), GetTypeHash(Hit.Item));
		if (Hit.FaceIndex == INDEX_NONE)
		{
			const FIntVector QuantizedNormal(
				FMath::RoundToInt(Normal.X * 32.0),
				FMath::RoundToInt(Normal.Y * 32.0),
				FMath::RoundToInt(Normal.Z * 32.0));
			FeatureHash = HashCombineFast(FeatureHash, GetTypeHash(QuantizedNormal));
		}
		return (static_cast<uint64>(ObjectHash) << 32) | FeatureHash;
	}

	void FillCoreContact(
		const FCableSimCachedContact& Cached,
		const CableSim::FParticle& Particle,
		const FCableSimFrictionSettings& FrictionSettings,
		CableSim::FContactConstraint& OutContact)
	{
		OutContact.FeatureId = Cached.FeatureId;
		OutContact.ParticleIndex = Cached.ParticleIndex;
		OutContact.Normal = Cached.Normal;
		OutContact.MinimumNormalCoordinate = Cached.MinimumNormalCoordinate;
		OutContact.SurfaceVelocity = Cached.SurfaceVelocity;
		OutContact.FrictionAnchorPosition = Cached.Collider.IsValid()
			? FVector3d(Cached.Collider->GetComponentTransform().TransformPosition(FVector(Cached.ColliderLocalFrictionAnchor)))
			: Cached.FrictionAnchorPosition;
		OutContact.bHasFrictionAnchor = FrictionSettings.bEnableFriction;
	}
}

void FCableSimContactCache::Reset()
{
	Contacts.Reset();
	AdditionCount = 0;
	RemovalCount = 0;
}

void FCableSimContactCache::BeginStep()
{
	AdditionCount = 0;
	RemovalCount = 0;
	for (FCableSimCachedContact& Contact : Contacts)
	{
		Contact.bSeenThisStep = false;
		Contact.bSeenThisPass = false;
	}
}

void FCableSimContactCache::BeginPass()
{
	for (FCableSimCachedContact& Contact : Contacts)
	{
		Contact.bSeenThisPass = false;
	}
}

void FCableSimContactCache::EndStep(const int32 PersistenceSteps)
{
	const int32 SafePersistence = FMath::Clamp(PersistenceSteps, 0, 8);
	for (int32 Index = Contacts.Num() - 1; Index >= 0; --Index)
	{
		FCableSimCachedContact& Contact = Contacts[Index];
		Contact.MissedStepCount = Contact.bSeenThisStep ? 0 : Contact.MissedStepCount + 1;
		if (Contact.MissedStepCount > SafePersistence)
		{
			Contacts.RemoveAt(Index, 1, EAllowShrinking::No);
			++RemovalCount;
		}
	}
}

void FCableSimWorldCollisionAdapter::GatherContacts(
	UWorld* World,
	AActor* Owner,
	const TConstArrayView<AActor*> IgnoredActors,
	const FCableSimCollisionSettings& Settings,
	const FCableSimFrictionSettings& FrictionSettings,
	const TConstArrayView<CableSim::FParticle> Particles,
	FCableSimContactCache& ContactCache,
	TArray<CableSim::FContactConstraint>& OutContacts)
{
	if (!World || !Settings.bEnableWorldCollision || Settings.Radius <= 0.0)
	{
		return;
	}
	ContactCache.BeginPass();
	for (FCableSimCachedContact& Cached : ContactCache.Contacts)
	{
		if (!Cached.Collider.IsValid())
		{
			continue;
		}
		const FTransform& ColliderTransform = Cached.Collider->GetComponentTransform();
		Cached.Normal = FVector3d(ColliderTransform.TransformVectorNoScale(FVector(Cached.ColliderLocalNormal))).GetSafeNormal();
		const FVector3d PlanePoint(ColliderTransform.TransformPosition(FVector(Cached.ColliderLocalPlanePoint)));
		Cached.MinimumNormalCoordinate = FVector3d::DotProduct(PlanePoint, Cached.Normal);
		Cached.SurfaceVelocity = FVector3d(Cached.Collider->GetComponentVelocity());
	}

	FCollisionQueryParams QueryParams(SCENE_QUERY_STAT(CableSimWorldCollision), false, Owner);
	for (AActor* IgnoredActor : IgnoredActors)
	{
		if (IsValid(IgnoredActor))
		{
			QueryParams.AddIgnoredActor(IgnoredActor);
		}
	}

	const double SafeRadius = FMath::Max(Settings.Radius, 0.1);
	const double SafeSkinWidth = FMath::Max(Settings.SkinWidth, 0.0);
	const int32 MaximumContacts = FMath::Clamp(Settings.MaximumContactsPerParticle, 1, 4);
	const double ParallelThreshold = FMath::Cos(FMath::DegreesToRadians(
		FMath::Clamp(Settings.ContactNormalToleranceDegrees, 0.0, 90.0)));
	const double ReleaseDistance = FMath::Max(Settings.ContactReleaseDistance, 0.0);
	const FCollisionShape ParticleShape = FCollisionShape::MakeSphere(SafeRadius);

	for (int32 ParticleIndex = 0; ParticleIndex < Particles.Num(); ++ParticleIndex)
	{
		const CableSim::FParticle& Particle = Particles[ParticleIndex];
		if (Particle.Mode != CableSim::EParticleMode::Dynamic)
		{
			continue;
		}

		TArray<FHitResult> Hits;
		World->SweepMultiByChannel(
			Hits,
			FVector(Particle.PreviousPosition),
			FVector(Particle.Position),
			FQuat::Identity,
			Settings.Channel.GetValue(),
			ParticleShape,
			QueryParams);
		Hits.StableSort([](const FHitResult& First, const FHitResult& Second)
		{
			if (!FMath::IsNearlyEqual(First.Time, Second.Time))
			{
				return First.Time < Second.Time;
			}
			return First.FaceIndex < Second.FaceIndex;
		});

		TArray<FVector3d, TInlineAllocator<4>> AcceptedNormals;
		for (const FHitResult& Hit : Hits)
		{
			const FVector3d Normal = FVector3d(Hit.Normal).GetSafeNormal();
			if (Normal.IsNearlyZero())
			{
				continue;
			}

			bool bDuplicate = false;
			for (const FVector3d& AcceptedNormal : AcceptedNormals)
			{
				if (FVector3d::DotProduct(AcceptedNormal, Normal) >= ParallelThreshold)
				{
					bDuplicate = true;
					break;
				}
			}
			if (bDuplicate)
			{
				continue;
			}

			const FVector3d End = Particle.Position;
			const FVector3d CorrectedCenter = Hit.bStartPenetrating
				? End + Normal * (Hit.PenetrationDepth + SafeSkinWidth)
				: FVector3d(Hit.Location) + Normal * SafeSkinWidth;
			const double MinimumNormalCoordinate = FVector3d::DotProduct(CorrectedCenter, Normal);
			const uint64 CandidateFeatureId = MakeContactFeatureId(Hit, Normal);
			UPrimitiveComponent* HitComponent = Hit.Component.Get();
			int32 CachedIndex = ContactCache.Contacts.IndexOfByPredicate(
				[ParticleIndex, HitComponent, &Hit](const FCableSimCachedContact& Cached)
				{
					return Cached.ParticleIndex == ParticleIndex
						&& Cached.Collider.Get() == HitComponent
						&& Cached.FaceIndex == Hit.FaceIndex
						&& Cached.Item == Hit.Item;
				});
			if (CachedIndex == INDEX_NONE)
			{
				CachedIndex = ContactCache.Contacts.IndexOfByPredicate(
					[ParticleIndex, Normal, MinimumNormalCoordinate, ParallelThreshold, ReleaseDistance](
						const FCableSimCachedContact& Cached)
					{
						return Cached.ParticleIndex == ParticleIndex
							&& FVector3d::DotProduct(Cached.Normal, Normal) >= ParallelThreshold
							&& FMath::Abs(Cached.MinimumNormalCoordinate - MinimumNormalCoordinate)
								<= ReleaseDistance;
					});
			}

			if (CachedIndex == INDEX_NONE)
			{
				FCableSimCachedContact& Cached = ContactCache.Contacts.AddDefaulted_GetRef();
				Cached.FeatureId = CandidateFeatureId;
				Cached.ParticleIndex = ParticleIndex;
				Cached.Collider = HitComponent;
				Cached.FaceIndex = Hit.FaceIndex;
				Cached.Item = Hit.Item;
				Cached.Normal = Normal;
				Cached.MinimumNormalCoordinate = MinimumNormalCoordinate;
				Cached.ColliderLocalPlanePoint = IsValid(HitComponent)
					? FVector3d(HitComponent->GetComponentTransform().InverseTransformPosition(FVector(CorrectedCenter)))
					: CorrectedCenter;
				Cached.ColliderLocalNormal = IsValid(HitComponent)
					? FVector3d(HitComponent->GetComponentTransform().InverseTransformVectorNoScale(FVector(Normal))).GetSafeNormal()
					: Normal;
				Cached.SurfaceVelocity = IsValid(HitComponent)
					? FVector3d(HitComponent->GetComponentVelocity())
					: FVector3d::ZeroVector;
				Cached.FrictionAnchorPosition = Particle.PreviousPosition;
				Cached.ColliderLocalFrictionAnchor = IsValid(HitComponent)
					? FVector3d(HitComponent->GetComponentTransform().InverseTransformPosition(FVector(Particle.PreviousPosition)))
					: Particle.PreviousPosition;
				Cached.bSeenThisStep = true;
				Cached.bSeenThisPass = true;
				CachedIndex = ContactCache.Contacts.Num() - 1;
				++ContactCache.AdditionCount;
			}
			else
			{
				FCableSimCachedContact& Cached = ContactCache.Contacts[CachedIndex];
				Cached.Normal = Normal;
				Cached.MinimumNormalCoordinate = MinimumNormalCoordinate;
				Cached.ColliderLocalPlanePoint = IsValid(HitComponent)
					? FVector3d(HitComponent->GetComponentTransform().InverseTransformPosition(FVector(CorrectedCenter)))
					: CorrectedCenter;
				Cached.ColliderLocalNormal = IsValid(HitComponent)
					? FVector3d(HitComponent->GetComponentTransform().InverseTransformVectorNoScale(FVector(Normal))).GetSafeNormal()
					: Normal;
				Cached.Collider = HitComponent;
				Cached.FaceIndex = Hit.FaceIndex;
				Cached.Item = Hit.Item;
				Cached.SurfaceVelocity = IsValid(HitComponent)
					? FVector3d(HitComponent->GetComponentVelocity())
					: FVector3d::ZeroVector;
				Cached.bSeenThisStep = true;
				Cached.bSeenThisPass = true;
			}

			CableSim::FContactConstraint& Contact = OutContacts.AddDefaulted_GetRef();
			FillCoreContact(ContactCache.Contacts[CachedIndex], Particle, FrictionSettings, Contact);
			AcceptedNormals.Add(Normal);
			if (AcceptedNormals.Num() >= MaximumContacts)
			{
				break;
			}
		}
	}

	for (const FCableSimCachedContact& Cached : ContactCache.Contacts)
	{
		if (Cached.bSeenThisPass || !Particles.IsValidIndex(Cached.ParticleIndex))
		{
			continue;
		}
		const CableSim::FParticle& Particle = Particles[Cached.ParticleIndex];
		const double Separation = FVector3d::DotProduct(Particle.Position, Cached.Normal)
			- Cached.MinimumNormalCoordinate;
		if (Particle.Mode != CableSim::EParticleMode::Dynamic || Separation > ReleaseDistance)
		{
			continue;
		}
		const bool bAlreadyAdded = OutContacts.ContainsByPredicate([&Cached](const CableSim::FContactConstraint& Contact)
		{
			return Contact.ParticleIndex == Cached.ParticleIndex && Contact.FeatureId == Cached.FeatureId;
		});
		if (!bAlreadyAdded)
		{
			CableSim::FContactConstraint& Contact = OutContacts.AddDefaulted_GetRef();
			FillCoreContact(Cached, Particle, FrictionSettings, Contact);
		}
	}
}
