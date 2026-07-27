#pragma once

#include "CableSimCollisionGeometry.h"
#include "CableSimSolver.h"

namespace CableSim
{
	struct CABLESIMCORE_API FManifoldConfig
	{
		double NodeRadius = 5.0;
		double ActiveBand = 1.0;
		double MergeNormalCosine = 0.985;
		double MergeOffsetTolerance = 1.0;
		int32 MaxPlanesPerNode = 4;
		int32 MaxEdgesPerNode = 3;
		double Tolerance = 0.1;
	};

	CABLESIMCORE_API uint64 PackContactFeatureId(const FCollisionFeatureId& FeatureId);

	class CABLESIMCORE_API FContactManifoldCompiler
	{
	public:
		static void CompileNodeContacts(
			int32 ParticleIndex,
			const FVector3d& NodePosition,
			const FVector3d& NodePreviousPosition,
			TConstArrayView<FCollisionTriangle> Triangles,
			TConstArrayView<FCollisionEdge> Edges,
			const FManifoldConfig& Config,
			TArray<FContactConstraint>& OutContacts);

		static FVector3d ClosestPointOnTriangle(
			const FVector3d& Point,
			const FVector3d& A,
			const FVector3d& B,
			const FVector3d& C);

		static FVector3d ClosestPointOnSegment(
			const FVector3d& Point,
			const FVector3d& Start,
			const FVector3d& End);
	};
}
