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

	// Packs a stable collision feature id into the solver's uint64 contact id so a
	// contact keeps its identity (and friction anchor) across steps.
	CABLESIMCORE_API uint64 PackContactFeatureId(const FCollisionFeatureId& FeatureId);

	class CABLESIMCORE_API FContactManifoldCompiler
	{
	public:
		// Builds a node's local contact manifold from the nearby snapshot geometry:
		// up to MaxEdgesPerNode radial convex-edge contacts (when the node sits in an
		// edge's exterior wedge) plus up to MaxPlanesPerNode merged face planes.
		// Coplanar faces collapse to one plane; face planes coincident with an active
		// edge's faces are suppressed so the edge and its faces never fight. Every
		// contact carries a stable feature id and the node's previous position as the
		// friction anchor.
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
