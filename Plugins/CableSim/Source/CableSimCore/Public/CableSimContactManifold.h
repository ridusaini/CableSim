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
		double Tolerance = 0.1;
	};

	// Packs a stable collision feature id into the solver's uint64 contact id so a
	// contact keeps its identity (and friction anchor) across steps.
	CABLESIMCORE_API uint64 PackContactFeatureId(const FCollisionFeatureId& FeatureId);

	class CABLESIMCORE_API FContactManifoldCompiler
	{
	public:
		// Builds up to MaxPlanesPerNode merged plane contacts for one node from the
		// nearby snapshot triangles. Coplanar faces (e.g. a box top) collapse to one
		// plane; the nearest planes win when capped. Emitted contacts carry the source
		// triangle's stable feature id and the node's previous position as the friction
		// anchor.
		static void CompileNodeContacts(
			int32 ParticleIndex,
			const FVector3d& NodePosition,
			const FVector3d& NodePreviousPosition,
			TConstArrayView<FCollisionTriangle> Triangles,
			const FManifoldConfig& Config,
			TArray<FContactConstraint>& OutContacts);

		static FVector3d ClosestPointOnTriangle(
			const FVector3d& Point,
			const FVector3d& A,
			const FVector3d& B,
			const FVector3d& C);
	};
}
