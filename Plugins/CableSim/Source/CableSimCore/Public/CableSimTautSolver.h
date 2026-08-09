#pragma once

#include "CableSimCollisionGeometry.h"

namespace CableSim
{
	enum class ETautStatus : uint8
	{
		Uninitialized,
		Ready,
		NoRelevantGeometry,
		InvalidSeed,
		FeatureUnavailable,
		PathBlocked,
		NonManifoldTopology,
		TopologyOverValence,
		CollisionBudgetExceeded,
		TopologyBudgetExceeded,
		InvalidConfiguration,
		NumericalFailure
	};

	enum class ETautPointType : uint8
	{
		Endpoint,
		EdgeContact,
		VertexContact
	};

	enum class ETautPairCase : uint8
	{
		Indeterminate,
		Inner,
		Unstable,
		Side,
		Outer
	};

	enum class ETautPairAction : uint8
	{
		Singular,
		StableInner,
		MoveAlongA,
		MoveAlongB,
		ChooseAOrB,
		IgnoreA,
		IgnoreB,
		OuterAThenB,
		OuterBThenA
	};

	enum class ETautVertexResolution : uint8
	{
		StableHold,
		SingularHold,
		ConflictHold,
		MoveAlong,
		OuterSplit,
		Release
	};

	/**
	 * Non-owning, immutable topology supplied for one solve. Runtime owns the
	 * backing snapshot. A complete admitted Box/Convex shape must be present;
	 * fragmenting a convex shape invalidates the edge topology predicates.
	 */
	struct CABLESIMCORE_API FTautCollisionScene
	{
		TConstArrayView<FCollisionTriangle> Triangles;
		TConstArrayView<FCollisionEdge> Edges;
		TConstArrayView<FCollisionVertex> Vertices;
	};

	struct CABLESIMCORE_API FTautConfig
	{
		double TopologyTolerance = 0.1;
		double MovementConvergenceTolerance = 0.01;
		double ParametricTolerance = 1.e-8;
		int32 MaximumMovementIterations = 32;
		int32 MaximumCollisionPhases = 32;
		int32 MaximumPathPoints = 32;
		int32 MaximumTopologyEvents = 32;
		int32 MaximumIncidentEdges = 8;
	};

	struct CABLESIMCORE_API FTautPoint
	{
		ETautPointType Type = ETautPointType::Endpoint;
		FVector3d Position = FVector3d::ZeroVector;
		FCollisionFeatureId FeatureId;
		double EdgeParameter = 0.0;

		// Topology transitions are created at their old geometric position and
		// moved by the normal swept collision phase. A pending target is therefore
		// state, not an immediate position assignment.
		FVector3d PendingTarget = FVector3d::ZeroVector;
		bool bHasPendingTarget = false;
		// Exact face-parallel predicates have two valid one-sided limits. Preserve
		// only the selected side while geometry remains singular; never preserve a
		// semantic Stable/Move/Ignore matrix action across frames.
		int8 SingularOrientation = 0;
	};

	struct CABLESIMCORE_API FTautStepInput
	{
		FVector3d StartTarget = FVector3d::ZeroVector;
		FVector3d EndTarget = FVector3d::ZeroVector;
	};

	struct CABLESIMCORE_API FTautStepResult
	{
		ETautStatus Status = ETautStatus::Uninitialized;
		FCollisionFeatureId FailureFeature;
		uint64 StepIndex = 0;
		int32 PointCount = 0;
		int32 ContactCount = 0;
		int32 MovementIterationCount = 0;
		int32 CollisionPhaseCount = 0;
		int32 TopologyEventCount = 0;
		// Outer-corner splits may subsequently merge or release within the same
		// step, so the final contact count alone cannot describe this transition.
		int32 OuterSplitCount = 0;
		double PathLength = 0.0;
		bool bPathCollisionFree = false;
	};

	/** One-frame corner-classification record for tests and debugger overlays. */
	struct CABLESIMCORE_API FTautPairDecision
	{
		FCollisionFeatureId VertexId;
		FCollisionFeatureId EdgeA;
		FCollisionFeatureId EdgeB;
		ETautPairCase PairCase = ETautPairCase::Indeterminate;
		ETautPairAction Action = ETautPairAction::Singular;
		bool bUsedOneSidedLimit = false;
		int8 SingularOrientation = 0;
	};

	struct CABLESIMCORE_API FTautVertexDecision
	{
		FCollisionFeatureId VertexId;
		ETautVertexResolution Resolution = ETautVertexResolution::ConflictHold;
		int32 ActiveEdgeCount = 0;
		int32 IndeterminatePairCount = 0;
		int32 OneSidedPairCount = 0;
	};

	struct CABLESIMCORE_API FTautStateSnapshot
	{
		TArray<FTautPoint> Points;
		FTautConfig Config;
		FTautStepResult LastResult;
		uint64 StepIndex = 0;
	};

	struct CABLESIMCORE_API FTautReplayFrame
	{
		FTautStepInput Input;
		TArray<FCollisionTriangle> Triangles;
		TArray<FCollisionEdge> Edges;
		TArray<FCollisionVertex> Vertices;
	};

	class CABLESIMCORE_API FTautPathSolver
	{
	public:
		bool Initialize(
			TConstArrayView<FVector3d> SeedPolyline,
			const FTautCollisionScene& Scene,
			const FTautConfig& InConfig = {});
		void Reset();
		FTautStepResult AdvanceStep(const FTautStepInput& Input, const FTautCollisionScene& Scene);

		bool IsInitialized() const { return Points.Num() >= 2; }
		const TArray<FTautPoint>& GetPoints() const { return Points; }
		const FTautStepResult& GetLastResult() const { return LastResult; }
		const FTautReplayFrame& GetLastReplayFrame() const { return LastReplayFrame; }
		const TArray<FTautPairDecision>& GetLastPairDecisions() const { return LastPairDecisions; }
		const TArray<FTautVertexDecision>& GetLastVertexDecisions() const { return LastVertexDecisions; }
		FTautStateSnapshot CaptureState() const;
		bool RestoreState(const FTautStateSnapshot& Snapshot);

	private:
		enum class EEdgeMotionKind : uint8
		{
			Interior,
			ReachStart,
			ReachEnd,
			Release,
			Singular
		};

		struct FEdgeMotion
		{
			EEdgeMotionKind Kind = EEdgeMotionKind::Singular;
			FVector3d Position = FVector3d::ZeroVector;
			double Parameter = 0.0;
			double UnclampedAxis = 0.0;
		};

		struct FSweepHit
		{
			TArray<int32, TInlineAllocator<8>> EdgeArrayIndices;
			double MovementTime = 1.0;
			FVector3d Position = FVector3d::ZeroVector;
		};

		struct FPairClassification
		{
			const FCollisionEdge* EdgeA = nullptr;
			const FCollisionEdge* EdgeB = nullptr;
			ETautPairCase PairCase = ETautPairCase::Indeterminate;
			ETautPairAction Action = ETautPairAction::Singular;
			bool bUsedOneSidedLimit = false;
			int8 SingularOrientation = 0;
		};

		static bool IsFinite(const FVector3d& Value);
		static bool IsValidConfig(const FTautConfig& InConfig);
		static bool IsUsableEdge(const FCollisionEdge& Edge, double Tolerance);
		static bool HasUsableEdge(const FTautCollisionScene& Scene, double Tolerance);
		static const FCollisionEdge* FindEdge(
			const FCollisionFeatureId& FeatureId,
			const FTautCollisionScene& Scene);
		static const FCollisionVertex* FindVertex(
			const FCollisionFeatureId& FeatureId,
			const FTautCollisionScene& Scene);
		static const FCollisionVertex* FindSharedVertex(
			TConstArrayView<int32> EdgeArrayIndices,
			const FVector3d& Position,
			const FTautCollisionScene& Scene,
			double Tolerance);
		static bool SegmentIntersectsTriangle(
			const FVector3d& SegmentStart,
			const FVector3d& SegmentEnd,
			const FVector3d& Triangle0,
			const FVector3d& Triangle1,
			const FVector3d& Triangle2,
			double ParametricTolerance,
			double& OutSegmentParameter,
			FVector3d& OutBarycentric);
		static bool SegmentIsCollisionFree(
			const FVector3d& Start,
			const FVector3d& End,
			const FTautCollisionScene& Scene,
			double DistanceTolerance,
			double ParametricTolerance);
		static bool FindFirstSweepHit(
			const FVector3d& FixedPoint,
			const FVector3d& MovingStart,
			const FVector3d& MovingTarget,
			const FTautCollisionScene& Scene,
			const FCollisionFeatureId& IgnoredFeature,
			double DistanceTolerance,
			double ParametricTolerance,
			FSweepHit& OutHit);
		static void MergeSweepHits(
			const FSweepHit& Candidate,
			double DistanceTolerance,
			double ParametricTolerance,
			FSweepHit& InOutBest,
			bool& bInOutHasHit);
		static bool BuildCollisionFreeSeed(
			TConstArrayView<FVector3d> PreferredPolyline,
			const FTautCollisionScene& Scene,
			const FTautConfig& InConfig,
			TArray<FTautPoint>& OutPoints);
		static FEdgeMotion EvaluateEdgeMotion(
			const FVector3d& RopeOrigin,
			const FVector3d& Start,
			const FVector3d& End,
			const FCollisionEdge& Edge,
			double Tolerance);
		static FPairClassification ClassifyEdgePair(
			const FVector3d& RopeOrigin,
			const FVector3d& Previous,
			const FVector3d& Next,
			const FCollisionVertex& Vertex,
			const FCollisionEdge& EdgeA,
			const FCollisionEdge& EdgeB,
			double Tolerance);
		static double CalculatePathLength(TConstArrayView<FTautPoint> InPoints);
		bool ValidateState(const FTautCollisionScene* Scene = nullptr) const;
		void UpdateResult(
			ETautStatus Status,
			int32 MovementIterations,
			int32 CollisionPhases,
			int32 TopologyEvents,
			const FCollisionFeatureId& FailureFeature = {});

		TArray<FTautPoint> Points;
		FTautConfig Config;
		FTautStepResult LastResult;
		FTautReplayFrame LastReplayFrame;
		TArray<FTautPairDecision> LastPairDecisions;
		TArray<FTautVertexDecision> LastVertexDecisions;
		uint64 StepIndex = 0;
		int32 CurrentOuterSplitCount = 0;
	};
}
