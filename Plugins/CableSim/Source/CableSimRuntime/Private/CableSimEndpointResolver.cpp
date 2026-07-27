#include "CableSimEndpointResolver.h"

void FCableSimEndpointResolver::Reset()
{
	for (int32 Index = 0; Index < 2; ++Index)
	{
		LastValidPositions[Index] = FVector3d::ZeroVector;
		bHasLastValidPosition[Index] = false;
		ResolvedComponents[Index].Reset();
	}
}

bool FCableSimEndpointResolver::Resolve(
	const USceneComponent& CableComponent,
	const FCableSimEndpointBinding& Binding,
	const ECableSimEndpoint Endpoint,
	FVector3d& OutPosition,
	ECableSimEndpointBindingStatus& OutStatus) const
{
	const int32 EndpointIndex = Endpoint == ECableSimEndpoint::Start ? 0 : 1;
	auto CommitValid = [this, EndpointIndex, &OutPosition, &OutStatus](const FVector3d& Position)
	{
		OutPosition = Position;
		OutStatus = ECableSimEndpointBindingStatus::Ready;
		LastValidPositions[EndpointIndex] = Position;
		bHasLastValidPosition[EndpointIndex] = true;
	};

	switch (Binding.Mode)
	{
	case ECableSimEndpointMode::Simulated:
		OutStatus = ECableSimEndpointBindingStatus::Simulated;
		return false;
	case ECableSimEndpointMode::CableLocalKinematic:
		CommitValid(FVector3d(CableComponent.GetComponentTransform().TransformPosition(Binding.LocalTarget)));
		return true;
	case ECableSimEndpointMode::WorldKinematic:
		CommitValid(FVector3d(Binding.WorldTarget));
		return true;
	case ECableSimEndpointMode::ComponentKinematic:
		if (!IsValid(Binding.TargetComponent))
		{
			OutStatus = ECableSimEndpointBindingStatus::MissingComponent;
			OutPosition = bHasLastValidPosition[EndpointIndex]
				? LastValidPositions[EndpointIndex]
				: FVector3d(CableComponent.GetComponentTransform().TransformPosition(Binding.SpawnLocalPosition));
			ResolvedComponents[EndpointIndex].Reset();
			return false;
		}
		ResolvedComponents[EndpointIndex] = Binding.TargetComponent;
		if (!Binding.SocketName.IsNone() && !Binding.TargetComponent->DoesSocketExist(Binding.SocketName))
		{
			OutStatus = ECableSimEndpointBindingStatus::MissingSocket;
			OutPosition = bHasLastValidPosition[EndpointIndex]
				? LastValidPositions[EndpointIndex]
				: FVector3d(CableComponent.GetComponentTransform().TransformPosition(Binding.SpawnLocalPosition));
			return false;
		}
		{
			const FTransform TargetTransform = Binding.SocketName.IsNone()
				? Binding.TargetComponent->GetComponentTransform()
				: Binding.TargetComponent->GetSocketTransform(Binding.SocketName, RTS_World);
			CommitValid(FVector3d(TargetTransform.TransformPosition(Binding.ComponentLocalOffset)));
			return true;
		}
	default:
		break;
	}
	OutStatus = ECableSimEndpointBindingStatus::MissingComponent;
	OutPosition = FVector3d(CableComponent.GetComponentTransform().TransformPosition(Binding.SpawnLocalPosition));
	return false;
}

void FCableSimEndpointResolver::AppendResolvedActors(TArray<AActor*>& OutActors) const
{
	for (const TWeakObjectPtr<USceneComponent>& Component : ResolvedComponents)
	{
		if (Component.IsValid() && IsValid(Component->GetOwner()))
		{
			OutActors.AddUnique(Component->GetOwner());
		}
	}
}
