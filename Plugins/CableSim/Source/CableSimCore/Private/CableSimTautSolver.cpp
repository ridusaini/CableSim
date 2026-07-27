#include "CableSimTautSolver.h"

namespace CableSim
{
	bool FTautPathSolver::Initialize(
		const FVector3d& Start,
		const FVector3d& End,
		const FTautConfig& InConfig)
	{
		Reset();
		if (!IsFinite(Start) || !IsFinite(End) || !IsValidConfig(InConfig))
		{
			LastResult.Status = ETautStatus::InvalidConfiguration;
			return false;
		}
		Config = InConfig;
		Config.TopologyTolerance = FMath::Max(Config.TopologyTolerance, 1.e-6);
		Config.MaximumCollisionPasses = FMath::Clamp(Config.MaximumCollisionPasses, 1, 64);
		Config.MaximumPathPoints = FMath::Clamp(Config.MaximumPathPoints, 2, 32);
		Config.MaximumTopologyEvents = FMath::Clamp(Config.MaximumTopologyEvents, 1, 32);
		Config.MaximumIncidentEdges = FMath::Clamp(Config.MaximumIncidentEdges, 1, 8);
		Points.SetNum(2);
		Points[0].Position = Start;
		Points[1].Position = End;
		UpdateResult(ETautStatus::Ready, 0);
		return true;
	}

	void FTautPathSolver::Reset()
	{
		Points.Reset();
		Config = FTautConfig{};
		LastResult = FTautStepResult{};
		LastReplayFrame = FTautReplayFrame{};
		StepIndex = 0;
	}

	FTautStepResult FTautPathSolver::AdvanceStep(
		const FTautStepInput& Input,
		const TConstArrayView<FCollisionEdge> Edges)
	{
		if (!IsInitialized())
		{
			LastResult.Status = ETautStatus::Uninitialized;
			return LastResult;
		}
		if (!IsFinite(Input.StartTarget) || !IsFinite(Input.EndTarget))
		{
			LastResult.Status = ETautStatus::InvalidConfiguration;
			return LastResult;
		}

		const FTautStateSnapshot LastValidState = CaptureState();
		LastReplayFrame.Input = Input;
		LastReplayFrame.Edges = Edges;
		int32 CollisionPasses = 0;
		int32 TopologyEvents = 0;
		const ETautStatus ReadyStatus = HasUsableEdge(Edges, Config.TopologyTolerance)
			? ETautStatus::Ready
			: ETautStatus::NoRelevantGeometry;

		auto Fail = [this, &LastValidState, &CollisionPasses](const ETautStatus FailureStatus)
		{
			RestoreState(LastValidState);
			UpdateResult(FailureStatus, CollisionPasses);
			return LastResult;
		};
		auto ConsumePass = [this, &CollisionPasses]()
		{
			return ++CollisionPasses < Config.MaximumCollisionPasses;
		};
		auto IsFeatureInPath = [this](const FCollisionFeatureId& FeatureId)
		{
			return Points.ContainsByPredicate([&FeatureId](const FTautPoint& Point)
			{
				return Point.Type == ETautPointType::EdgeContact && Point.FeatureId == FeatureId;
			});
		};
		auto MoveEndpoint = [this, Edges, &TopologyEvents, &ConsumePass, &IsFeatureInPath](
			const bool bStartEndpoint,
			const FVector3d& Target)
		{
			const FVector3d MovingStart = bStartEndpoint ? Points[0].Position : Points.Last().Position;
			for (;;)
			{
				if (!ConsumePass())
				{
					return ETautStatus::IterationBudgetExceeded;
				}
				TArray<FCollisionEdge, TInlineAllocator<32>> EligibleEdges;
				for (const FCollisionEdge& Edge : Edges)
				{
					if (!IsFeatureInPath(Edge.Id))
					{
						EligibleEdges.Add(Edge);
					}
				}
				const FVector3d FixedPoint = bStartEndpoint ? Points[1].Position : Points[Points.Num() - 2].Position;
				FSweepHit Hit;
				if (!FindFirstSweepHit(
					FixedPoint,
					MovingStart,
					Target,
					EligibleEdges,
					Config.TopologyTolerance,
					Hit))
				{
					break;
				}
				if (Points.Num() >= Config.MaximumPathPoints
					|| ++TopologyEvents > Config.MaximumTopologyEvents)
				{
					return ETautStatus::IterationBudgetExceeded;
				}
				FTautPoint Contact;
				Contact.Type = ETautPointType::EdgeContact;
				Contact.FeatureId = EligibleEdges[Hit.EdgeArrayIndex].Id;
				Contact.Position = Hit.Position;
				const FCollisionEdge& HitEdge = EligibleEdges[Hit.EdgeArrayIndex];
				Contact.EdgeParameter = FVector3d::Distance(HitEdge.Start, HitEdge.End) > Config.TopologyTolerance
					? FVector3d::Distance(HitEdge.Start, Hit.Position) / FVector3d::Distance(HitEdge.Start, HitEdge.End)
					: 0.0;
				Points.Insert(Contact, bStartEndpoint ? 1 : Points.Num() - 1);
			}
			(bStartEndpoint ? Points[0] : Points.Last()).Position = Target;
			return ETautStatus::Ready;
		};

		ETautStatus MovementStatus = MoveEndpoint(true, Input.StartTarget);
		if (MovementStatus != ETautStatus::Ready)
		{
			return Fail(MovementStatus);
		}
		MovementStatus = MoveEndpoint(false, Input.EndTarget);
		if (MovementStatus != ETautStatus::Ready)
		{
			return Fail(MovementStatus);
		}

		bool bTopologyChanged = true;
		while (bTopologyChanged)
		{
			bTopologyChanged = false;
			if (!ConsumePass())
			{
				return Fail(ETautStatus::IterationBudgetExceeded);
			}
			for (int32 PointIndex = Points.Num() - 2; PointIndex >= 1; --PointIndex)
			{
				FTautPoint& Point = Points[PointIndex];
				const FCollisionEdge* ContactEdge = FindEdge(Point.FeatureId, Edges);
				if (!ContactEdge)
				{
					return Fail(ETautStatus::FeatureInvalidated);
				}
				const FVector3d& Previous = Points[PointIndex - 1].Position;
				const FVector3d& Next = Points[PointIndex + 1].Position;
				if (!RequiresWrap(Previous, Next, *ContactEdge, Config.TopologyTolerance))
				{
					Points.RemoveAt(PointIndex);
					bTopologyChanged = true;
					++TopologyEvents;
					continue;
				}

				FVector3d ContactPosition;
				double EdgeParameter = 0.0;
				if (CalculateContactPosition(
					Previous,
					Next,
					*ContactEdge,
					Config.TopologyTolerance,
					ContactPosition,
					EdgeParameter))
				{
					Point.Position = ContactPosition;
					Point.EdgeParameter = EdgeParameter;
					continue;
				}

				const FVector3d EdgeVector = ContactEdge->End - ContactEdge->Start;
				const double EdgeLength = EdgeVector.Length();
				const FVector3d EdgeDirection = EdgeVector / EdgeLength;
				const double PreviousAxis = FVector3d::DotProduct(Previous - ContactEdge->Start, EdgeDirection);
				const double NextAxis = FVector3d::DotProduct(Next - ContactEdge->Start, EdgeDirection);
				const double PreviousRadius = ((Previous - ContactEdge->Start) - EdgeDirection * PreviousAxis).Length();
				const double NextRadius = ((Next - ContactEdge->Start) - EdgeDirection * NextAxis).Length();
				const double RadiusSum = PreviousRadius + NextRadius;
				const double AxisPosition = RadiusSum > Config.TopologyTolerance
					? (NextRadius * PreviousAxis + PreviousRadius * NextAxis) / RadiusSum
					: 0.0;
				const bool bAtStartVertex = AxisPosition <= 0.0;
				const int32 VertexIndex = bAtStartVertex ? ContactEdge->Id.Index0 : ContactEdge->Id.Index1;
				const FVector3d VertexPosition = bAtStartVertex ? ContactEdge->Start : ContactEdge->End;
				TArray<const FCollisionEdge*, TInlineAllocator<8>> IncidentCandidates;
				int32 IncidentCount = 0;
				for (const FCollisionEdge& Edge : Edges)
				{
					if (Edge.Id.ObjectToken != ContactEdge->Id.ObjectToken
						|| Edge.Id.ShapeIndex != ContactEdge->Id.ShapeIndex
						|| (Edge.Id.Index0 != VertexIndex && Edge.Id.Index1 != VertexIndex))
					{
						continue;
					}
					++IncidentCount;
					if (Edge.Kind == ECollisionEdgeKind::NonManifold)
					{
						return Fail(ETautStatus::NonManifoldTopology);
					}
					if (Edge.Id != ContactEdge->Id && IsUsableEdge(Edge, Config.TopologyTolerance)
						&& !IsFeatureInPath(Edge.Id)
						&& RequiresWrap(Previous, Next, Edge, Config.TopologyTolerance))
					{
						IncidentCandidates.Add(&Edge);
					}
				}
				if (IncidentCount > Config.MaximumIncidentEdges)
				{
					return Fail(ETautStatus::TopologyOverValence);
				}
				IncidentCandidates.Sort([](const FCollisionEdge& First, const FCollisionEdge& Second)
				{
					return FCollisionFeatureId::Less(First.Id, Second.Id);
				});
				Point.Position = VertexPosition;
				Point.EdgeParameter = bAtStartVertex ? 0.0 : 1.0;
				if (!IncidentCandidates.IsEmpty())
				{
					const FCollisionEdge& NextEdge = *IncidentCandidates[0];
					Point.FeatureId = NextEdge.Id;
					Point.EdgeParameter = NextEdge.Id.Index0 == VertexIndex ? 0.0 : 1.0;
					bTopologyChanged = true;
					if (++TopologyEvents > Config.MaximumTopologyEvents)
					{
						return Fail(ETautStatus::IterationBudgetExceeded);
					}
				}
			}
		}

		if (!ValidateState())
		{
			RestoreState(LastValidState);
			UpdateResult(ETautStatus::NumericalFailure, CollisionPasses);
			return LastResult;
		}

		++StepIndex;
		UpdateResult(ReadyStatus, CollisionPasses);
		return LastResult;
	}

	FTautStateSnapshot FTautPathSolver::CaptureState() const
	{
		FTautStateSnapshot Snapshot;
		Snapshot.Points = Points;
		Snapshot.Config = Config;
		Snapshot.LastResult = LastResult;
		Snapshot.StepIndex = StepIndex;
		return Snapshot;
	}

	bool FTautPathSolver::RestoreState(const FTautStateSnapshot& Snapshot)
	{
		if (Snapshot.Points.Num() < 2
			|| Snapshot.Points.Num() > Snapshot.Config.MaximumPathPoints
			|| !IsValidConfig(Snapshot.Config))
		{
			return false;
		}
		for (const FTautPoint& Point : Snapshot.Points)
		{
			if (!IsFinite(Point.Position) || !FMath::IsFinite(Point.EdgeParameter))
			{
				return false;
			}
		}
		Points = Snapshot.Points;
		Config = Snapshot.Config;
		LastResult = Snapshot.LastResult;
		StepIndex = Snapshot.StepIndex;
		return ValidateState();
	}

	bool FTautPathSolver::CalculateReachableEndpoint(
		const bool bStartEndpoint,
		const double MaximumPathLength,
		FVector3d& OutReachablePosition,
		double& OutExcessDistance) const
	{
		OutReachablePosition = FVector3d::ZeroVector;
		OutExcessDistance = 0.0;
		if (!IsInitialized() || !FMath::IsFinite(MaximumPathLength) || MaximumPathLength <= 0.0)
		{
			return false;
		}
		const double PathLength = CalculatePathLength(Points);
		OutExcessDistance = FMath::Max(PathLength - MaximumPathLength, 0.0);
		const int32 EndpointIndex = bStartEndpoint ? 0 : Points.Num() - 1;
		if (OutExcessDistance <= 0.0)
		{
			OutReachablePosition = Points[EndpointIndex].Position;
			return true;
		}

		double FixedPathLength = 0.0;
		FVector3d AdjacentPoint;
		if (bStartEndpoint)
		{
			AdjacentPoint = Points[1].Position;
			for (int32 Index = 1; Index + 1 < Points.Num(); ++Index)
			{
				FixedPathLength += FVector3d::Distance(Points[Index].Position, Points[Index + 1].Position);
			}
		}
		else
		{
			AdjacentPoint = Points[Points.Num() - 2].Position;
			for (int32 Index = 0; Index + 2 < Points.Num(); ++Index)
			{
				FixedPathLength += FVector3d::Distance(Points[Index].Position, Points[Index + 1].Position);
			}
		}
		const double RemainingLength = MaximumPathLength - FixedPathLength;
		const FVector3d Direction = (Points[EndpointIndex].Position - AdjacentPoint).GetSafeNormal();
		if (RemainingLength < 0.0 || Direction.IsNearlyZero())
		{
			return false;
		}
		OutReachablePosition = AdjacentPoint + Direction * RemainingLength;
		return IsFinite(OutReachablePosition);
	}

	bool FTautPathSolver::IsFinite(const FVector3d& Value)
	{
		return FMath::IsFinite(Value.X) && FMath::IsFinite(Value.Y) && FMath::IsFinite(Value.Z);
	}

	bool FTautPathSolver::IsValidConfig(const FTautConfig& InConfig)
	{
		return FMath::IsFinite(InConfig.TopologyTolerance) && InConfig.TopologyTolerance > 0.0
			&& InConfig.MaximumCollisionPasses > 0
			&& InConfig.MaximumPathPoints >= 2 && InConfig.MaximumPathPoints <= 32
			&& InConfig.MaximumTopologyEvents > 0 && InConfig.MaximumTopologyEvents <= 32
			&& InConfig.MaximumIncidentEdges > 0 && InConfig.MaximumIncidentEdges <= 8;
	}

	bool FTautPathSolver::IsUsableEdge(const FCollisionEdge& Edge, const double Tolerance)
	{
		return Edge.Kind == ECollisionEdgeKind::Convex && Edge.IsFinite()
			&& FVector3d::Distance(Edge.Start, Edge.End) > Tolerance
			&& !Edge.FaceNormal0.IsNearlyZero() && !Edge.FaceNormal1.IsNearlyZero();
	}

	bool FTautPathSolver::HasUsableEdge(
		const TConstArrayView<FCollisionEdge> Edges,
		const double Tolerance)
	{
		for (const FCollisionEdge& Edge : Edges)
		{
			if (IsUsableEdge(Edge, Tolerance))
			{
				return true;
			}
		}
		return false;
	}

	bool FTautPathSolver::SegmentIntersectsTriangle(
		const FVector3d& SegmentStart,
		const FVector3d& SegmentEnd,
		const FVector3d& Triangle0,
		const FVector3d& Triangle1,
		const FVector3d& Triangle2,
		const double Tolerance,
		double& OutSegmentParameter,
		FVector3d& OutBarycentric)
	{
		const FVector3d Direction = SegmentEnd - SegmentStart;
		const FVector3d Edge1 = Triangle1 - Triangle0;
		const FVector3d Edge2 = Triangle2 - Triangle0;
		const FVector3d P = FVector3d::CrossProduct(Direction, Edge2);
		const double Determinant = FVector3d::DotProduct(Edge1, P);
		if (FMath::Abs(Determinant) <= Tolerance * 1.e-3)
		{
			return false;
		}
		const double InverseDeterminant = 1.0 / Determinant;
		const double BarycentricTolerance = 1.e-8;
		const FVector3d T = SegmentStart - Triangle0;
		const double U = FVector3d::DotProduct(T, P) * InverseDeterminant;
		if (U < -BarycentricTolerance || U > 1.0 + BarycentricTolerance)
		{
			return false;
		}
		const FVector3d Q = FVector3d::CrossProduct(T, Edge1);
		const double V = FVector3d::DotProduct(Direction, Q) * InverseDeterminant;
		if (V < -BarycentricTolerance || U + V > 1.0 + BarycentricTolerance)
		{
			return false;
		}
		const double SegmentParameter = FVector3d::DotProduct(Edge2, Q) * InverseDeterminant;
		if (SegmentParameter < -BarycentricTolerance || SegmentParameter > 1.0 + BarycentricTolerance)
		{
			return false;
		}
		OutSegmentParameter = FMath::Clamp(SegmentParameter, 0.0, 1.0);
		OutBarycentric = FVector3d(1.0 - U - V, U, V);
		return true;
	}

	bool FTautPathSolver::FindFirstSweepHit(
		const FVector3d& FixedPoint,
		const FVector3d& MovingStart,
		const FVector3d& MovingTarget,
		const TConstArrayView<FCollisionEdge> Edges,
		const double Tolerance,
		FSweepHit& OutHit)
	{
		bool bFound = false;
		for (int32 EdgeIndex = 0; EdgeIndex < Edges.Num(); ++EdgeIndex)
		{
			const FCollisionEdge& Edge = Edges[EdgeIndex];
			if (!IsUsableEdge(Edge, Tolerance))
			{
				continue;
			}
			double EdgeParameter = 0.0;
			FVector3d Barycentric;
			if (!SegmentIntersectsTriangle(
				Edge.Start,
				Edge.End,
				FixedPoint,
				MovingStart,
				MovingTarget,
				Tolerance,
				EdgeParameter,
				Barycentric))
			{
				continue;
			}
			const double MovingWeight = Barycentric.Y + Barycentric.Z;
			const double MovementTime = MovingWeight > 1.e-12
				? FMath::Clamp(Barycentric.Z / MovingWeight, 0.0, 1.0)
				: 0.0;
			if (!bFound || MovementTime < OutHit.MovementTime - Tolerance
				|| (FMath::IsNearlyEqual(MovementTime, OutHit.MovementTime, Tolerance)
					&& FCollisionFeatureId::Less(Edge.Id, Edges[OutHit.EdgeArrayIndex].Id)))
			{
				bFound = true;
				OutHit.EdgeArrayIndex = EdgeIndex;
				OutHit.MovementTime = MovementTime;
				OutHit.Position = FMath::Lerp(Edge.Start, Edge.End, EdgeParameter);
			}
		}
		return bFound;
	}

	const FCollisionEdge* FTautPathSolver::FindEdge(
		const FCollisionFeatureId& FeatureId,
		const TConstArrayView<FCollisionEdge> Edges)
	{
		for (const FCollisionEdge& Edge : Edges)
		{
			if (Edge.Id == FeatureId)
			{
				return &Edge;
			}
		}
		return nullptr;
	}

	bool FTautPathSolver::RequiresWrap(
		const FVector3d& Start,
		const FVector3d& End,
		const FCollisionEdge& Edge,
		const double Tolerance)
	{
		const double Start0 = FVector3d::DotProduct(Edge.FaceNormal0, Start - Edge.Start);
		const double Start1 = FVector3d::DotProduct(Edge.FaceNormal1, Start - Edge.Start);
		const double End0 = FVector3d::DotProduct(Edge.FaceNormal0, End - Edge.Start);
		const double End1 = FVector3d::DotProduct(Edge.FaceNormal1, End - Edge.Start);
		const bool bStartOnFace0 = Start0 > Tolerance && Start1 <= Tolerance;
		const bool bStartOnFace1 = Start1 > Tolerance && Start0 <= Tolerance;
		const bool bEndOnFace0 = End0 > Tolerance && End1 <= Tolerance;
		const bool bEndOnFace1 = End1 > Tolerance && End0 <= Tolerance;
		return (bStartOnFace0 && bEndOnFace1) || (bStartOnFace1 && bEndOnFace0);
	}

	bool FTautPathSolver::CalculateContactPosition(
		const FVector3d& Start,
		const FVector3d& End,
		const FCollisionEdge& Edge,
		const double Tolerance,
		FVector3d& OutPosition,
		double& OutParameter)
	{
		const FVector3d EdgeVector = Edge.End - Edge.Start;
		const double EdgeLength = EdgeVector.Length();
		if (EdgeLength <= Tolerance)
		{
			return false;
		}
		const FVector3d Direction = EdgeVector / EdgeLength;
		const double StartAxis = FVector3d::DotProduct(Start - Edge.Start, Direction);
		const double EndAxis = FVector3d::DotProduct(End - Edge.Start, Direction);
		const double StartRadius = ((Start - Edge.Start) - Direction * StartAxis).Length();
		const double EndRadius = ((End - Edge.Start) - Direction * EndAxis).Length();
		const double RadiusSum = StartRadius + EndRadius;
		if (RadiusSum <= Tolerance)
		{
			return false;
		}
		const double AxisPosition = (EndRadius * StartAxis + StartRadius * EndAxis) / RadiusSum;
		if (AxisPosition < -Tolerance || AxisPosition > EdgeLength + Tolerance)
		{
			return false;
		}
		OutParameter = FMath::Clamp(AxisPosition / EdgeLength, 0.0, 1.0);
		OutPosition = FMath::Lerp(Edge.Start, Edge.End, OutParameter);
		return true;
	}

	double FTautPathSolver::CalculatePathLength(const TConstArrayView<FTautPoint> InPoints)
	{
		double Length = 0.0;
		for (int32 Index = 0; Index + 1 < InPoints.Num(); ++Index)
		{
			Length += FVector3d::Distance(InPoints[Index].Position, InPoints[Index + 1].Position);
		}
		return Length;
	}

	bool FTautPathSolver::ValidateState() const
	{
		if (Points.Num() < 2 || Points.Num() > Config.MaximumPathPoints)
		{
			return false;
		}
		for (int32 Index = 0; Index < Points.Num(); ++Index)
		{
			const FTautPoint& Point = Points[Index];
			if (!IsFinite(Point.Position) || !FMath::IsFinite(Point.EdgeParameter))
			{
				return false;
			}
			if ((Index == 0 || Index == Points.Num() - 1) && Point.Type != ETautPointType::Endpoint)
			{
				return false;
			}
			if (Index > 0 && Index + 1 < Points.Num()
				&& (Point.Type != ETautPointType::EdgeContact || !Point.FeatureId.IsValid()))
			{
				return false;
			}
		}
		return true;
	}

	void FTautPathSolver::UpdateResult(const ETautStatus Status, const int32 CollisionPassCount)
	{
		LastResult.Status = Status;
		LastResult.StepIndex = StepIndex;
		LastResult.PointCount = Points.Num();
		LastResult.ContactCount = FMath::Max(Points.Num() - 2, 0);
		LastResult.CollisionPassCount = CollisionPassCount;
		LastResult.PathLength = CalculatePathLength(Points);
	}
}
