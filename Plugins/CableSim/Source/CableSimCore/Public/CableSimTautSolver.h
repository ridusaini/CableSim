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
		double PathLength = 0.0;
		bool bPathCollisionFree = false;
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
		FTautStateSnapshot CaptureState() const;
		bool RestoreState(const FTautStateSnapshot& Snapshot);

	private:
		struct FSweepHit
		{
			int32 EdgeArrayIndex = INDEX_NONE;
			double MovementTime = 1.0;
			FVector3d Position = FVector3d::ZeroVector;
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
		static bool BuildCollisionFreeSeed(
			TConstArrayView<FVector3d> PreferredPolyline,
			const FTautCollisionScene& Scene,
			const FTautConfig& InConfig,
			TArray<FTautPoint>& OutPoints);
		static bool RequiresWrap(
			const FVector3d& Start,
			const FVector3d& End,
			const FCollisionEdge& Edge,
			double Tolerance);
		static bool CalculateContactPosition(
			const FVector3d& Start,
			const FVector3d& End,
			const FCollisionEdge& Edge,
			double Tolerance,
			FVector3d& OutPosition,
			double& OutParameter);
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
		uint64 StepIndex = 0;
	};
}
