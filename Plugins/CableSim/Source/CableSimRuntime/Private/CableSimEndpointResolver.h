#pragma once

#include "CableSimComponent.h"

class FCableSimEndpointResolver
{
public:
	void Reset();
	bool Resolve(
		const USceneComponent& CableComponent,
		const FCableSimEndpointBinding& Binding,
		ECableSimEndpoint Endpoint,
		FVector3d& OutPosition,
		ECableSimEndpointBindingStatus& OutStatus) const;
	void AppendResolvedActors(TArray<AActor*>& OutActors) const;

private:
	mutable FVector3d LastValidPositions[2] = {FVector3d::ZeroVector, FVector3d::ZeroVector};
	mutable bool bHasLastValidPosition[2] = {false, false};
	mutable TWeakObjectPtr<USceneComponent> ResolvedComponents[2];
};
