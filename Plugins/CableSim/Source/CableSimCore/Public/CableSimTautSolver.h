#pragma once

#include "CableSimCollisionGeometry.h"

namespace CableSim
{
	enum class ETautStatus : uint8
	{
		Uninitialized,
		Ready,
		NoRelevantGeometry,
		NonManifoldTopology,
		TopologyOverValence,
		FeatureInvalidated,
		IterationBudgetExceeded,
		InvalidConfiguration,
		NumericalFailure
	};

	enum class ETautPointType : uint8
	{
		Endpoint,
		EdgeContact
	};

	struct CABLESIMCORE_API FTautConfig
	{
		double TopologyTolerance = 0.1;
		int32 MaximumCollisionPasses = 16;
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
		uint64 StepIndex = 0;
		int32 PointCount = 0;
		int32 ContactCount = 0;
		int32 CollisionPassCount = 0;
		double PathLength = 0.0;
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
		TArray<FCollisionEdge> Edges;
	};

	class CABLESIMCORE_API FTautPathSolver
	{
	public:
		bool Initialize(const FVector3d& Start, const FVector3d& End, const FTautConfig& InConfig = {});
		void Reset();
		FTautStepResult AdvanceStep(const FTautStepInput& Input, TConstArrayView<FCollisionEdge> Edges);

		bool IsInitialized() const { return Points.Num() >= 2; }
		const TArray<FTautPoint>& GetPoints() const { return Points; }
		const FTautStepResult& GetLastResult() const { return LastResult; }
		const FTautReplayFrame& GetLastReplayFrame() const { return LastReplayFrame; }
		FTautStateSnapshot CaptureState() const;
		bool RestoreState(const FTautStateSnapshot& Snapshot);
		bool CalculateReachableEndpoint(
			bool bStartEndpoint,
			double MaximumPathLength,
			FVector3d& OutReachablePosition,
			double& OutExcessDistance) const;

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
		static bool HasUsableEdge(TConstArrayView<FCollisionEdge> Edges, double Tolerance);
		static bool SegmentIntersectsTriangle(
			const FVector3d& SegmentStart,
			const FVector3d& SegmentEnd,
			const FVector3d& Triangle0,
			const FVector3d& Triangle1,
			const FVector3d& Triangle2,
			double Tolerance,
			double& OutSegmentParameter,
			FVector3d& OutBarycentric);
		static bool FindFirstSweepHit(
			const FVector3d& FixedPoint,
			const FVector3d& MovingStart,
			const FVector3d& MovingTarget,
			TConstArrayView<FCollisionEdge> Edges,
			double Tolerance,
			FSweepHit& OutHit);
		static const FCollisionEdge* FindEdge(
			const FCollisionFeatureId& FeatureId,
			TConstArrayView<FCollisionEdge> Edges);
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
		bool ValidateState() const;
		void UpdateResult(ETautStatus Status, int32 CollisionPassCount);

		TArray<FTautPoint> Points;
		FTautConfig Config;
		FTautStepResult LastResult;
		FTautReplayFrame LastReplayFrame;
		uint64 StepIndex = 0;
	};
}
