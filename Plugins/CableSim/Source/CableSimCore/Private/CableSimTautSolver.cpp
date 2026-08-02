#include "CableSimTautSolver.h"

namespace CableSim
{
	namespace
	{
		double CalculateUnclampedAxisPosition(
			const FVector3d& Start,
			const FVector3d& End,
			const FCollisionEdge& Edge,
			const double Tolerance)
		{
			const FVector3d EdgeVector = Edge.End - Edge.Start;
			const double EdgeLength = EdgeVector.Length();
			if (EdgeLength <= Tolerance)
			{
				return 0.0;
			}
			const FVector3d Direction = EdgeVector / EdgeLength;
			const double StartAxis = FVector3d::DotProduct(Start - Edge.Start, Direction);
			const double EndAxis = FVector3d::DotProduct(End - Edge.Start, Direction);
			const double StartRadius = ((Start - Edge.Start) - Direction * StartAxis).Length();
			const double EndRadius = ((End - Edge.Start) - Direction * EndAxis).Length();
			const double RadiusSum = StartRadius + EndRadius;
			return RadiusSum > Tolerance
				? (EndRadius * StartAxis + StartRadius * EndAxis) / RadiusSum
				: 0.5 * (StartAxis + EndAxis);
		}

		FCollisionFeatureId MakeVertexId(const FCollisionEdge& Edge, const bool bStart)
		{
			return {
				Edge.Id.ObjectToken,
				Edge.Id.ShapeIndex,
				ECollisionFeatureType::Vertex,
				bStart ? Edge.Id.Index0 : Edge.Id.Index1,
				INDEX_NONE};
		}
	}

	bool FTautPathSolver::Initialize(
		const TConstArrayView<FVector3d> SeedPolyline,
		const FTautCollisionScene& Scene,
		const FTautConfig& InConfig)
	{
		Reset();
		if (!IsValidConfig(InConfig) || SeedPolyline.Num() < 2)
		{
			LastResult.Status = ETautStatus::InvalidConfiguration;
			return false;
		}

		Config = InConfig;
		Config.TopologyTolerance = FMath::Max(Config.TopologyTolerance, 1.e-6);
		Config.MovementConvergenceTolerance = FMath::Clamp(
			Config.MovementConvergenceTolerance, 1.e-8, Config.TopologyTolerance);
		Config.ParametricTolerance = FMath::Clamp(Config.ParametricTolerance, 1.e-12, 1.e-3);
		Config.MaximumMovementIterations = FMath::Clamp(Config.MaximumMovementIterations, 1, 128);
		Config.MaximumCollisionPhases = FMath::Clamp(Config.MaximumCollisionPhases, 1, 128);
		Config.MaximumPathPoints = FMath::Clamp(Config.MaximumPathPoints, 2, 32);
		Config.MaximumTopologyEvents = FMath::Clamp(Config.MaximumTopologyEvents, 1, 32);
		Config.MaximumIncidentEdges = FMath::Clamp(Config.MaximumIncidentEdges, 1, 8);

		TArray<FVector3d, TInlineAllocator<128>> Seed;
		for (const FVector3d& Position : SeedPolyline)
		{
			if (!IsFinite(Position))
			{
				Reset();
				LastResult.Status = ETautStatus::InvalidSeed;
				return false;
			}
			if (Seed.IsEmpty()
				|| FVector3d::Distance(Seed.Last(), Position) > Config.MovementConvergenceTolerance)
			{
				Seed.Add(Position);
			}
		}
		if (Seed.Num() < 2)
		{
			Reset();
			LastResult.Status = ETautStatus::InvalidSeed;
			return false;
		}
		bool bSeedNeedsRepair = false;
		for (int32 Index = 0; Index + 1 < Seed.Num(); ++Index)
		{
			if (!SegmentIsCollisionFree(
				Seed[Index], Seed[Index + 1], Scene,
				Config.TopologyTolerance, Config.ParametricTolerance))
			{
				bSeedNeedsRepair = true;
				break;
			}
		}
		if (bSeedNeedsRepair)
		{
			TArray<FTautPoint> RepairedPoints;
			if (!BuildCollisionFreeSeed(Seed, Scene, Config, RepairedPoints))
			{
				Reset();
				LastResult.Status = ETautStatus::InvalidSeed;
				return false;
			}
			Points = MoveTemp(RepairedPoints);
			UpdateResult(ETautStatus::Ready, 0, 0, 0);
			FTautStepInput RepairStep;
			RepairStep.StartTarget = Points[0].Position;
			RepairStep.EndTarget = Points.Last().Position;
			const FTautStepResult RepairResult = AdvanceStep(RepairStep, Scene);
			if (RepairResult.Status != ETautStatus::Ready
				&& RepairResult.Status != ETautStatus::NoRelevantGeometry)
			{
				Reset();
				LastResult.Status = ETautStatus::InvalidSeed;
				return false;
			}
			StepIndex = 0;
			LastReplayFrame = FTautReplayFrame{};
			UpdateResult(RepairResult.Status, 0, 0, 0);
			LastResult.bPathCollisionFree = ValidateState(&Scene);
			return LastResult.bPathCollisionFree;
		}

		Points.SetNum(2);
		Points[0].Type = ETautPointType::Endpoint;
		Points[0].Position = Seed[0];
		Points[1].Type = ETautPointType::Endpoint;
		Points[1].Position = Seed[1];
		UpdateResult(ETautStatus::Ready, 0, 0, 0);

		// Thread the endpoint through the collision-free seed. This preserves the
		// visible rope's homotopy while the same continuous collision logic used by
		// normal steps inserts the minimal persistent edge contacts.
		for (int32 Index = 2; Index < Seed.Num(); ++Index)
		{
			FTautStepInput SeedStep;
			SeedStep.StartTarget = Seed[0];
			SeedStep.EndTarget = Seed[Index];
			const FTautStepResult SeedResult = AdvanceStep(SeedStep, Scene);
			if (SeedResult.Status != ETautStatus::Ready
				&& SeedResult.Status != ETautStatus::NoRelevantGeometry)
			{
				Reset();
				LastResult.Status = ETautStatus::InvalidSeed;
				return false;
			}
		}

		StepIndex = 0;
		LastReplayFrame = FTautReplayFrame{};
		UpdateResult(
			HasUsableEdge(Scene, Config.TopologyTolerance)
				? ETautStatus::Ready : ETautStatus::NoRelevantGeometry,
			0, 0, 0);
		LastResult.bPathCollisionFree = ValidateState(&Scene);
		return LastResult.bPathCollisionFree;
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
		const FTautCollisionScene& Scene)
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
		LastReplayFrame.Triangles = Scene.Triangles;
		LastReplayFrame.Edges = Scene.Edges;
		LastReplayFrame.Vertices = Scene.Vertices;
		int32 MovementIterations = 0;
		int32 CollisionPhases = 0;
		int32 TopologyEvents = 0;

		auto Fail = [this, &LastValidState, &MovementIterations, &CollisionPhases, &TopologyEvents](
			const ETautStatus Status,
			const FCollisionFeatureId& Feature = FCollisionFeatureId{})
		{
			RestoreState(LastValidState);
			UpdateResult(Status, MovementIterations, CollisionPhases, TopologyEvents, Feature);
			return LastResult;
		};

		// On budget exhaustion, emit the current path if it is already collision-free
		// rather than freezing; only a path that actually clips falls back to the last
		// good state (FallbackStatus).
		auto EmitBestEffort = [this, &Input, &Scene, &MovementIterations, &CollisionPhases, &TopologyEvents, &Fail](
			const ETautStatus FallbackStatus)
		{
			// Only ship a best-effort path whose endpoints reached their targets and
			// that is collision-free. If an endpoint is still mid-advance around an
			// obstacle (budget ran out before it arrived), the path is incomplete --
			// hold the last good state instead.
			const bool bEndpointsAtTargets =
				Points[0].Position.Equals(Input.StartTarget, Config.TopologyTolerance)
				&& Points.Last().Position.Equals(Input.EndTarget, Config.TopologyTolerance);
			if (!bEndpointsAtTargets || !ValidateState(&Scene))
			{
				return Fail(FallbackStatus);
			}
			++StepIndex;
			UpdateResult(
				HasUsableEdge(Scene, Config.TopologyTolerance)
					? ETautStatus::Ready : ETautStatus::NoRelevantGeometry,
				MovementIterations, CollisionPhases + 1, TopologyEvents);
			LastResult.bPathCollisionFree = true;
			return LastResult;
		};

		for (const FTautPoint& Point : Points)
		{
			if (Point.Type == ETautPointType::EdgeContact && !FindEdge(Point.FeatureId, Scene))
			{
				return Fail(ETautStatus::FeatureUnavailable, Point.FeatureId);
			}
			if (Point.Type == ETautPointType::VertexContact && !FindVertex(Point.FeatureId, Scene))
			{
				return Fail(ETautStatus::FeatureUnavailable, Point.FeatureId);
			}
		}

		for (; CollisionPhases < Config.MaximumCollisionPhases; ++CollisionPhases)
		{
			TArray<FVector3d, TInlineAllocator<32>> Targets;
			Targets.Reserve(Points.Num());
			for (const FTautPoint& Point : Points)
			{
				Targets.Add(Point.Position);
			}
			Targets[0] = Input.StartTarget;
			Targets.Last() = Input.EndTarget;

			// A run of consecutive edge contacts on parallel edges has a closed-form
			// taut solution: unroll the edges into a plane and draw one straight line.
			// The flat iteration converges to it only slowly as the edges converge, so
			// seed it directly here; skew runs fall through to the iteration.
			for (int32 RunStart = 1; RunStart + 1 < Points.Num(); )
			{
				if (Points[RunStart].Type != ETautPointType::EdgeContact)
				{
					++RunStart;
					continue;
				}
				const FCollisionEdge* FirstEdge = FindEdge(Points[RunStart].FeatureId, Scene);
				if (!FirstEdge)
				{
					++RunStart;
					continue;
				}
				const FVector3d Direction = (FirstEdge->End - FirstEdge->Start).GetSafeNormal();
				TArray<const FCollisionEdge*, TInlineAllocator<8>> RunEdges;
				RunEdges.Add(FirstEdge);
				int32 RunEnd = RunStart;
				while (RunEnd + 2 < Points.Num() && Points[RunEnd + 1].Type == ETautPointType::EdgeContact)
				{
					const FCollisionEdge* NextEdge = FindEdge(Points[RunEnd + 1].FeatureId, Scene);
					if (!NextEdge)
					{
						break;
					}
					const FVector3d NextDirection = (NextEdge->End - NextEdge->Start).GetSafeNormal();
					if (FMath::Abs(FVector3d::DotProduct(NextDirection, Direction)) < 1.0 - 1.e-4)
					{
						break;
					}
					RunEdges.Add(NextEdge);
					++RunEnd;
				}
				if (RunEnd > RunStart && !Direction.IsNearlyZero())
				{
					const FVector3d A = Targets[RunStart - 1];
					const FVector3d B = Targets[RunEnd + 1];
					auto PerpendicularDistance = [&Direction](const FVector3d& Point, const FVector3d& Origin)
					{
						const FVector3d Relative = Point - Origin;
						return (Relative - Direction * FVector3d::DotProduct(Relative, Direction)).Length();
					};
					// Unrolled horizontal position of each edge: cumulative perpendicular
					// distance walked from A across the parallel edges to B.
					TArray<double, TInlineAllocator<8>> Horizontal;
					Horizontal.Add(PerpendicularDistance(A, RunEdges[0]->Start));
					for (int32 Index = 1; Index < RunEdges.Num(); ++Index)
					{
						Horizontal.Add(Horizontal[Index - 1]
							+ PerpendicularDistance(RunEdges[Index]->Start, RunEdges[Index - 1]->Start));
					}
					const double TotalHorizontal = Horizontal.Last()
						+ PerpendicularDistance(B, RunEdges.Last()->Start);
					const double AlongSpan = FVector3d::DotProduct(B - A, Direction);
					if (TotalHorizontal > Config.TopologyTolerance)
					{
						for (int32 Index = 0; Index < RunEdges.Num(); ++Index)
						{
							const FCollisionEdge* Edge = RunEdges[Index];
							const double AlongTarget = AlongSpan * Horizontal[Index] / TotalHorizontal;
							const double AlongStart = FVector3d::DotProduct(Edge->Start - A, Direction);
							const double AlongEnd = FVector3d::DotProduct(Edge->End - A, Direction);
							const double Denominator = AlongEnd - AlongStart;
							if (FMath::Abs(Denominator) <= Config.TopologyTolerance)
							{
								continue;
							}
							const double Parameter = FMath::Clamp(
								(AlongTarget - AlongStart) / Denominator, 0.0, 1.0);
							Targets[RunStart + Index] = FMath::Lerp(Edge->Start, Edge->End, Parameter);
						}
					}
				}
				RunStart = RunEnd + 1;
			}

			for (int32 LocalIteration = 0;
				LocalIteration < Config.MaximumMovementIterations;
				++LocalIteration)
			{
				++MovementIterations;
				double MaximumDisplacement = 0.0;
				const bool bForward = (LocalIteration & 1) == 0;
				for (int32 Visit = 1; Visit + 1 < Points.Num(); ++Visit)
				{
					const int32 PointIndex = bForward ? Visit : Points.Num() - 1 - Visit;
					const FTautPoint& Point = Points[PointIndex];
					FVector3d NewTarget = Targets[PointIndex];
					if (Point.Type == ETautPointType::EdgeContact)
					{
						const FCollisionEdge* Edge = FindEdge(Point.FeatureId, Scene);
						if (!Edge)
						{
							return Fail(ETautStatus::FeatureUnavailable, Point.FeatureId);
						}
						double Parameter = 0.0;
						if (!CalculateContactPosition(
							Targets[PointIndex - 1], Targets[PointIndex + 1], *Edge,
							Config.TopologyTolerance, NewTarget, Parameter))
						{
							const double Axis = CalculateUnclampedAxisPosition(
								Targets[PointIndex - 1], Targets[PointIndex + 1], *Edge,
								Config.TopologyTolerance);
							NewTarget = Axis <= 0.0 ? Edge->Start : Edge->End;
						}
					}
					else if (Point.Type == ETautPointType::VertexContact)
					{
						const FCollisionVertex* Vertex = FindVertex(Point.FeatureId, Scene);
						if (!Vertex)
						{
							return Fail(ETautStatus::FeatureUnavailable, Point.FeatureId);
						}
						NewTarget = Vertex->Position;
					}
					MaximumDisplacement = FMath::Max(
						MaximumDisplacement,
						FVector3d::Distance(Targets[PointIndex], NewTarget));
					Targets[PointIndex] = NewTarget;
				}
				if (MaximumDisplacement <= Config.MovementConvergenceTolerance)
				{
					break;
				}
			}
			// A partial movement phase is accepted, not failed: the collision sweep and
			// final ValidateState below still gate the path, and later steps continue
			// straightening it.

			bool bCollisionAdded = false;
			for (int32 PointIndex = 0; PointIndex < Points.Num(); ++PointIndex)
			{
				const FVector3d MovingStart = Points[PointIndex].Position;
				const FVector3d MovingTarget = Targets[PointIndex];
				if (FVector3d::Distance(MovingStart, MovingTarget) <= Config.MovementConvergenceTolerance)
				{
					Points[PointIndex].Position = MovingTarget;
					continue;
				}

				FSweepHit BestHit;
				bool bHitLeft = false;
				bool bHasHit = false;
				const FCollisionFeatureId IgnoredFeature =
					Points[PointIndex].Type == ETautPointType::Endpoint
					? FCollisionFeatureId{} : Points[PointIndex].FeatureId;
				if (PointIndex > 0)
				{
					FSweepHit Hit;
					if (FindFirstSweepHit(
						Points[PointIndex - 1].Position, MovingStart, MovingTarget,
						Scene, IgnoredFeature, Config.TopologyTolerance,
						Config.ParametricTolerance, Hit))
					{
						BestHit = Hit;
						bHitLeft = true;
						bHasHit = true;
					}
				}
				if (PointIndex + 1 < Points.Num())
				{
					FSweepHit Hit;
					if (FindFirstSweepHit(
						Points[PointIndex + 1].Position, MovingStart, MovingTarget,
						Scene, IgnoredFeature, Config.TopologyTolerance,
						Config.ParametricTolerance, Hit)
						&& (!bHasHit
							|| Hit.MovementTime < BestHit.MovementTime - Config.ParametricTolerance
							|| (FMath::IsNearlyEqual(
								Hit.MovementTime, BestHit.MovementTime, Config.ParametricTolerance)
								&& FCollisionFeatureId::Less(
									Scene.Edges[Hit.EdgeArrayIndex].Id,
									Scene.Edges[BestHit.EdgeArrayIndex].Id))))
					{
						BestHit = Hit;
						bHitLeft = false;
						bHasHit = true;
					}
				}

				if (!bHasHit)
				{
					Points[PointIndex].Position = MovingTarget;
					continue;
				}
				if (Points.Num() >= Config.MaximumPathPoints)
				{
					return EmitBestEffort(ETautStatus::TopologyBudgetExceeded);
				}
				if (++TopologyEvents > Config.MaximumTopologyEvents)
				{
					return EmitBestEffort(ETautStatus::TopologyBudgetExceeded);
				}
				Points[PointIndex].Position = FMath::Lerp(
					MovingStart, MovingTarget, BestHit.MovementTime);
				const FCollisionEdge& HitEdge = Scene.Edges[BestHit.EdgeArrayIndex];
				FTautPoint Contact;
				Contact.Type = ETautPointType::EdgeContact;
				Contact.FeatureId = HitEdge.Id;
				Contact.Position = BestHit.Position;
				const double EdgeLength = FVector3d::Distance(HitEdge.Start, HitEdge.End);
				Contact.EdgeParameter = EdgeLength > Config.TopologyTolerance
					? FVector3d::Distance(HitEdge.Start, BestHit.Position) / EdgeLength : 0.0;
				const int32 InsertIndex = bHitLeft ? PointIndex : PointIndex + 1;
				Points.Insert(Contact, InsertIndex);
				bCollisionAdded = true;
				break;
			}
			if (bCollisionAdded)
			{
				continue;
			}

			bool bTopologyChanged = false;
			for (int32 PointIndex = Points.Num() - 2; PointIndex >= 1; --PointIndex)
			{
				FTautPoint& Point = Points[PointIndex];
				const FVector3d Previous = Points[PointIndex - 1].Position;
				const FVector3d Next = Points[PointIndex + 1].Position;
				if (Point.Type == ETautPointType::EdgeContact)
				{
					const FCollisionEdge* Edge = FindEdge(Point.FeatureId, Scene);
					if (!Edge)
					{
						return Fail(ETautStatus::FeatureUnavailable, Point.FeatureId);
					}
					FVector3d ContactPosition;
					double Parameter = 0.0;
					if (CalculateContactPosition(
						Previous, Next, *Edge, Config.TopologyTolerance,
						ContactPosition, Parameter))
					{
						Point.Position = ContactPosition;
						Point.EdgeParameter = Parameter;
						// A wrap is removed only when straightening it through is a
						// continuous lift-off: the chord no longer wraps this edge, is
						// clear of geometry, and sweeping the rope from its current bend
						// (Previous -> contact -> Next) onto the straight chord does not
						// cross any other collision edge. A rope wound the long way round
						// an obstacle fails that sweep, so it can never teleport across to
						// a shorter chord -- it can only leave by sliding off an edge end
						// (the vertex transition below).
						if (!RequiresWrap(Previous, Next, *Edge, Config.TopologyTolerance)
							&& SegmentIsCollisionFree(
								Previous, Next, Scene, Config.TopologyTolerance,
								Config.ParametricTolerance)
							&& !StraighteningCrossesEdge(
								Previous, ContactPosition, Next, Point.FeatureId, Scene,
								Config.TopologyTolerance, Config.ParametricTolerance))
						{
							Points.RemoveAt(PointIndex);
							bTopologyChanged = true;
							break;
						}
						continue;
					}

					const double Axis = CalculateUnclampedAxisPosition(
						Previous, Next, *Edge, Config.TopologyTolerance);
					const bool bAtStart = Axis <= 0.0;
					const FCollisionFeatureId VertexId = MakeVertexId(*Edge, bAtStart);
					const FCollisionVertex* Vertex = FindVertex(VertexId, Scene);
					if (!Vertex)
					{
						return Fail(ETautStatus::FeatureUnavailable, VertexId);
					}
					Point.Type = ETautPointType::VertexContact;
					Point.FeatureId = VertexId;
					Point.Position = Vertex->Position;
					Point.EdgeParameter = 0.0;
					bTopologyChanged = true;
					break;
				}

				const FCollisionVertex* Vertex = FindVertex(Point.FeatureId, Scene);
				if (!Vertex)
				{
					return Fail(ETautStatus::FeatureUnavailable, Point.FeatureId);
				}
				if (Vertex->bNonManifold)
				{
					return Fail(ETautStatus::NonManifoldTopology, Point.FeatureId);
				}
				TArray<const FCollisionEdge*, TInlineAllocator<8>> Candidates;
				for (const FCollisionFeatureId& IncidentId : Vertex->IncidentEdges)
				{
					const FCollisionEdge* Incident = FindEdge(IncidentId, Scene);
					if (Incident && IsUsableEdge(*Incident, Config.TopologyTolerance)
						&& RequiresWrap(Previous, Next, *Incident, Config.TopologyTolerance))
					{
						Candidates.Add(Incident);
					}
				}
				if (Candidates.Num() > Config.MaximumIncidentEdges)
				{
					return Fail(ETautStatus::TopologyOverValence, Point.FeatureId);
				}
				if (Candidates.IsEmpty())
				{
					if (SegmentIsCollisionFree(
						Previous, Next, Scene, Config.TopologyTolerance,
						Config.ParametricTolerance))
					{
						Points.RemoveAt(PointIndex);
						bTopologyChanged = true;
						break;
					}
					continue;
				}

				// Resolve the vertex to the shortest collision-free local path: stay on
				// the vertex, or slide onto whichever single incident edge is shorter and
				// stays collision-free. An edge that would not shorten the path is never
				// chosen.
				constexpr int32 ResolveStay = 0;
				constexpr int32 ResolveSingle = 1;
				auto EdgeParameterOf = [](const FCollisionEdge& Edge, const FVector3d& Position, const double Tolerance)
				{
					const double Length = FVector3d::Distance(Edge.Start, Edge.End);
					return Length > Tolerance ? FVector3d::Distance(Edge.Start, Position) / Length : 0.0;
				};

				int32 BestResolution = ResolveStay;
				double BestLength = FVector3d::Distance(Previous, Vertex->Position)
					+ FVector3d::Distance(Vertex->Position, Next);
				const FCollisionEdge* BestEdgeA = nullptr;
				FVector3d BestPosA = Vertex->Position;

				for (const FCollisionEdge* Candidate : Candidates)
				{
					FVector3d Position;
					double Parameter = 0.0;
					if (!CalculateContactPosition(
						Previous, Next, *Candidate, Config.TopologyTolerance, Position, Parameter)
						|| !SegmentIsCollisionFree(Previous, Position, Scene, Config.TopologyTolerance, Config.ParametricTolerance)
						|| !SegmentIsCollisionFree(Position, Next, Scene, Config.TopologyTolerance, Config.ParametricTolerance))
					{
						continue;
					}
					const double Length = FVector3d::Distance(Previous, Position)
						+ FVector3d::Distance(Position, Next);
					const bool bShorter = Length < BestLength - Config.TopologyTolerance;
					const bool bTieBreak = BestResolution == ResolveSingle && BestEdgeA
						&& FMath::IsNearlyEqual(Length, BestLength, Config.TopologyTolerance)
						&& FCollisionFeatureId::Less(Candidate->Id, BestEdgeA->Id);
					if (bShorter || bTieBreak)
					{
						BestResolution = ResolveSingle;
						BestLength = Length;
						BestEdgeA = Candidate;
						BestPosA = Position;
					}
				}

				if (BestResolution == ResolveSingle && BestEdgeA)
				{
					Point.Type = ETautPointType::EdgeContact;
					Point.FeatureId = BestEdgeA->Id;
					Point.Position = BestPosA;
					Point.EdgeParameter = EdgeParameterOf(*BestEdgeA, BestPosA, Config.TopologyTolerance);
					bTopologyChanged = true;
					break;
				}
				// ResolveStay: the caught rope stays on the vertex; nothing changes.
				Point.Position = Vertex->Position;
			}
			if (bTopologyChanged)
			{
				if (++TopologyEvents > Config.MaximumTopologyEvents)
				{
					return EmitBestEffort(ETautStatus::TopologyBudgetExceeded);
				}
				continue;
			}

			if (!ValidateState(&Scene))
			{
				return Fail(ETautStatus::PathBlocked);
			}
			++StepIndex;
			UpdateResult(
				HasUsableEdge(Scene, Config.TopologyTolerance)
					? ETautStatus::Ready : ETautStatus::NoRelevantGeometry,
				MovementIterations, CollisionPhases + 1, TopologyEvents);
			LastResult.bPathCollisionFree = true;
			return LastResult;
		}

		return EmitBestEffort(ETautStatus::CollisionBudgetExceeded);
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
		Points = Snapshot.Points;
		Config = Snapshot.Config;
		LastResult = Snapshot.LastResult;
		StepIndex = Snapshot.StepIndex;
		return ValidateState();
	}

	bool FTautPathSolver::IsFinite(const FVector3d& Value)
	{
		return FMath::IsFinite(Value.X) && FMath::IsFinite(Value.Y) && FMath::IsFinite(Value.Z);
	}

	bool FTautPathSolver::IsValidConfig(const FTautConfig& InConfig)
	{
		return FMath::IsFinite(InConfig.TopologyTolerance) && InConfig.TopologyTolerance > 0.0
			&& FMath::IsFinite(InConfig.MovementConvergenceTolerance)
			&& InConfig.MovementConvergenceTolerance > 0.0
			&& FMath::IsFinite(InConfig.ParametricTolerance) && InConfig.ParametricTolerance > 0.0
			&& InConfig.MaximumMovementIterations > 0 && InConfig.MaximumMovementIterations <= 128
			&& InConfig.MaximumCollisionPhases > 0 && InConfig.MaximumCollisionPhases <= 128
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

	bool FTautPathSolver::HasUsableEdge(const FTautCollisionScene& Scene, const double Tolerance)
	{
		for (const FCollisionEdge& Edge : Scene.Edges)
		{
			if (IsUsableEdge(Edge, Tolerance))
			{
				return true;
			}
		}
		return false;
	}

	const FCollisionEdge* FTautPathSolver::FindEdge(
		const FCollisionFeatureId& FeatureId,
		const FTautCollisionScene& Scene)
	{
		for (const FCollisionEdge& Edge : Scene.Edges)
		{
			if (Edge.Id == FeatureId)
			{
				return &Edge;
			}
		}
		return nullptr;
	}

	const FCollisionVertex* FTautPathSolver::FindVertex(
		const FCollisionFeatureId& FeatureId,
		const FTautCollisionScene& Scene)
	{
		for (const FCollisionVertex& Vertex : Scene.Vertices)
		{
			if (Vertex.Id == FeatureId)
			{
				return &Vertex;
			}
		}
		// A topology vertex may be represented by the canonical id of a cluster
		// spanning several touching shapes. Resolve a non-canonical edge endpoint
		// through the cluster's incident-edge list.
		const FCollisionVertex* BestVertex = nullptr;
		double BestSquaredDistance = TNumericLimits<double>::Max();
		for (const FCollisionEdge& Edge : Scene.Edges)
		{
			if (Edge.Id.ObjectToken != FeatureId.ObjectToken
				|| Edge.Id.ShapeIndex != FeatureId.ShapeIndex)
			{
				continue;
			}
			const bool bStart = Edge.Id.Index0 == FeatureId.Index0;
			const bool bEnd = Edge.Id.Index1 == FeatureId.Index0;
			if (!bStart && !bEnd)
			{
				continue;
			}
			const FVector3d Endpoint = bStart ? Edge.Start : Edge.End;
			for (const FCollisionVertex& Vertex : Scene.Vertices)
			{
				if (!Vertex.IncidentEdges.Contains(Edge.Id))
				{
					continue;
				}
				const double SquaredDistance = (Vertex.Position - Endpoint).SquaredLength();
				if (!BestVertex || SquaredDistance < BestSquaredDistance)
				{
					BestVertex = &Vertex;
					BestSquaredDistance = SquaredDistance;
				}
			}
		}
		return BestVertex;
	}

	bool FTautPathSolver::SegmentIntersectsTriangle(
		const FVector3d& SegmentStart,
		const FVector3d& SegmentEnd,
		const FVector3d& Triangle0,
		const FVector3d& Triangle1,
		const FVector3d& Triangle2,
		const double ParametricTolerance,
		double& OutSegmentParameter,
		FVector3d& OutBarycentric)
	{
		const FVector3d Direction = SegmentEnd - SegmentStart;
		const FVector3d Edge1 = Triangle1 - Triangle0;
		const FVector3d Edge2 = Triangle2 - Triangle0;
		const FVector3d P = FVector3d::CrossProduct(Direction, Edge2);
		const double Determinant = FVector3d::DotProduct(Edge1, P);
		const double Scale = FMath::Sqrt(
			Direction.SquaredLength() * Edge1.SquaredLength() * Edge2.SquaredLength());
		if (Scale <= 1.e-24 || FMath::Abs(Determinant) <= Scale * ParametricTolerance)
		{
			return false;
		}
		const double InverseDeterminant = 1.0 / Determinant;
		const FVector3d T = SegmentStart - Triangle0;
		const double U = FVector3d::DotProduct(T, P) * InverseDeterminant;
		if (U < -ParametricTolerance || U > 1.0 + ParametricTolerance)
		{
			return false;
		}
		const FVector3d Q = FVector3d::CrossProduct(T, Edge1);
		const double V = FVector3d::DotProduct(Direction, Q) * InverseDeterminant;
		if (V < -ParametricTolerance || U + V > 1.0 + ParametricTolerance)
		{
			return false;
		}
		const double SegmentParameter = FVector3d::DotProduct(Edge2, Q) * InverseDeterminant;
		if (SegmentParameter < -ParametricTolerance || SegmentParameter > 1.0 + ParametricTolerance)
		{
			return false;
		}
		OutSegmentParameter = FMath::Clamp(SegmentParameter, 0.0, 1.0);
		OutBarycentric = FVector3d(1.0 - U - V, U, V);
		return true;
	}

	bool FTautPathSolver::SegmentIsCollisionFree(
		const FVector3d& Start,
		const FVector3d& End,
		const FTautCollisionScene& Scene,
		const double DistanceTolerance,
		const double ParametricTolerance)
	{
		const double SegmentLength = FVector3d::Distance(Start, End);
		if (SegmentLength <= DistanceTolerance)
		{
			return true;
		}
		const double EndpointTolerance = FMath::Min(DistanceTolerance / SegmentLength, 0.25);
		for (const FCollisionTriangle& Triangle : Scene.Triangles)
		{
			double Time = 0.0;
			FVector3d Barycentric;
			if (Triangle.IsFinite()
				&& SegmentIntersectsTriangle(
					Start, End,
					Triangle.Vertices[0], Triangle.Vertices[1], Triangle.Vertices[2],
					ParametricTolerance, Time, Barycentric)
				&& Time > EndpointTolerance && Time < 1.0 - EndpointTolerance)
			{
				return false;
			}
		}
		return true;
	}

	bool FTautPathSolver::BuildCollisionFreeSeed(
		const TConstArrayView<FVector3d> PreferredPolyline,
		const FTautCollisionScene& Scene,
		const FTautConfig& InConfig,
		TArray<FTautPoint>& OutPoints)
	{
		OutPoints.Reset();
		if (PreferredPolyline.Num() < 2 || Scene.Vertices.IsEmpty())
		{
			return false;
		}

		struct FVisibilityNode
		{
			FVector3d Position = FVector3d::ZeroVector;
			FCollisionFeatureId VertexId;
		};
		TArray<FVisibilityNode> Nodes;
		Nodes.Reserve(Scene.Vertices.Num() + 2);
		Nodes.Add({PreferredPolyline[0], {}});
		for (const FCollisionVertex& Vertex : Scene.Vertices)
		{
			if (Vertex.IsFinite() && !Vertex.bNonManifold && !Vertex.bOverValence)
			{
				Nodes.Add({Vertex.Position, Vertex.Id});
			}
		}
		const int32 EndNode = Nodes.Add({PreferredPolyline.Last(), {}});
		if (EndNode < 2)
		{
			return false;
		}

		auto DistanceToPreferredPolyline = [&PreferredPolyline](const FVector3d& Position)
		{
			double BestSquaredDistance = TNumericLimits<double>::Max();
			for (int32 Index = 0; Index + 1 < PreferredPolyline.Num(); ++Index)
			{
				const FVector3d Segment = PreferredPolyline[Index + 1] - PreferredPolyline[Index];
				const double SegmentSquaredLength = Segment.SquaredLength();
				const double Parameter = SegmentSquaredLength > 1.e-24
					? FMath::Clamp(FVector3d::DotProduct(
						Position - PreferredPolyline[Index], Segment) / SegmentSquaredLength, 0.0, 1.0)
					: 0.0;
				BestSquaredDistance = FMath::Min(
					BestSquaredDistance,
					(Position - (PreferredPolyline[Index] + Segment * Parameter)).SquaredLength());
			}
			return FMath::Sqrt(BestSquaredDistance);
		};

		TArray<double> Costs;
		TArray<int32> Previous;
		TArray<bool> Visited;
		Costs.Init(TNumericLimits<double>::Max(), Nodes.Num());
		Previous.Init(INDEX_NONE, Nodes.Num());
		Visited.Init(false, Nodes.Num());
		Costs[0] = 0.0;
		for (int32 SearchIteration = 0; SearchIteration < Nodes.Num(); ++SearchIteration)
		{
			int32 Current = INDEX_NONE;
			for (int32 NodeIndex = 0; NodeIndex < Nodes.Num(); ++NodeIndex)
			{
				if (!Visited[NodeIndex]
					&& (Current == INDEX_NONE
						|| Costs[NodeIndex] < Costs[Current] - InConfig.MovementConvergenceTolerance
						|| (FMath::IsNearlyEqual(
							Costs[NodeIndex], Costs[Current], InConfig.MovementConvergenceTolerance)
							&& NodeIndex < Current)))
				{
					Current = NodeIndex;
				}
			}
			if (Current == INDEX_NONE || Costs[Current] == TNumericLimits<double>::Max())
			{
				break;
			}
			if (Current == EndNode)
			{
				break;
			}
			Visited[Current] = true;
			for (int32 Candidate = 0; Candidate < Nodes.Num(); ++Candidate)
			{
				if (Candidate == Current || Visited[Candidate])
				{
					continue;
				}
				const double Distance = FVector3d::Distance(
					Nodes[Current].Position, Nodes[Candidate].Position);
				if (Distance <= InConfig.TopologyTolerance
					|| !SegmentIsCollisionFree(
						Nodes[Current].Position, Nodes[Candidate].Position, Scene,
						InConfig.TopologyTolerance, InConfig.ParametricTolerance))
				{
					continue;
				}
				// Geometric length remains dominant. The small seed-distance term
				// deterministically resolves equally short routes in favour of the
				// side occupied by the visible dynamic rope.
				const double RoutePreference = DistanceToPreferredPolyline(
					0.5 * (Nodes[Current].Position + Nodes[Candidate].Position));
				const double CandidateCost = Costs[Current] + Distance + 0.05 * RoutePreference;
				if (CandidateCost < Costs[Candidate] - InConfig.MovementConvergenceTolerance
					|| (FMath::IsNearlyEqual(
						CandidateCost, Costs[Candidate], InConfig.MovementConvergenceTolerance)
						&& (Previous[Candidate] == INDEX_NONE || Current < Previous[Candidate])))
				{
					Costs[Candidate] = CandidateCost;
					Previous[Candidate] = Current;
				}
			}
		}
		if (Previous[EndNode] == INDEX_NONE)
		{
			return false;
		}

		TArray<int32, TInlineAllocator<32>> ReversePath;
		for (int32 Node = EndNode; Node != INDEX_NONE; Node = Previous[Node])
		{
			ReversePath.Add(Node);
			if (ReversePath.Num() > InConfig.MaximumPathPoints)
			{
				return false;
			}
		}
		for (int32 ReverseIndex = ReversePath.Num() - 1; ReverseIndex >= 0; --ReverseIndex)
		{
			const int32 NodeIndex = ReversePath[ReverseIndex];
			FTautPoint& Point = OutPoints.AddDefaulted_GetRef();
			Point.Position = Nodes[NodeIndex].Position;
			if (NodeIndex == 0 || NodeIndex == EndNode)
			{
				Point.Type = ETautPointType::Endpoint;
			}
			else
			{
				Point.Type = ETautPointType::VertexContact;
				Point.FeatureId = Nodes[NodeIndex].VertexId;
			}
		}
		return OutPoints.Num() >= 2;
	}

	bool FTautPathSolver::FindFirstSweepHit(
		const FVector3d& FixedPoint,
		const FVector3d& MovingStart,
		const FVector3d& MovingTarget,
		const FTautCollisionScene& Scene,
		const FCollisionFeatureId& IgnoredFeature,
		const double DistanceTolerance,
		const double ParametricTolerance,
		FSweepHit& OutHit)
	{
		bool bFound = false;
		for (int32 EdgeIndex = 0; EdgeIndex < Scene.Edges.Num(); ++EdgeIndex)
		{
			const FCollisionEdge& Edge = Scene.Edges[EdgeIndex];
			if (!IsUsableEdge(Edge, DistanceTolerance) || Edge.Id == IgnoredFeature)
			{
				continue;
			}
			double EdgeParameter = 0.0;
			FVector3d Barycentric;
			if (!SegmentIntersectsTriangle(
				Edge.Start, Edge.End, FixedPoint, MovingStart, MovingTarget,
				ParametricTolerance, EdgeParameter, Barycentric))
			{
				continue;
			}
			const FVector3d HitPosition = FMath::Lerp(Edge.Start, Edge.End, EdgeParameter);
			if (FVector3d::Distance(HitPosition, FixedPoint) <= DistanceTolerance
				|| FVector3d::Distance(HitPosition, MovingStart) <= DistanceTolerance)
			{
				continue;
			}
			const double MovingWeight = Barycentric.Y + Barycentric.Z;
			const double MovementTime = MovingWeight > ParametricTolerance
				? FMath::Clamp(Barycentric.Z / MovingWeight, 0.0, 1.0) : 0.0;
			if (!bFound || MovementTime < OutHit.MovementTime - ParametricTolerance
				|| (FMath::IsNearlyEqual(MovementTime, OutHit.MovementTime, ParametricTolerance)
					&& FCollisionFeatureId::Less(Edge.Id, Scene.Edges[OutHit.EdgeArrayIndex].Id)))
			{
				bFound = true;
				OutHit.EdgeArrayIndex = EdgeIndex;
				OutHit.MovementTime = MovementTime;
				OutHit.Position = HitPosition;
			}
		}
		return bFound;
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
		const double Axis = CalculateUnclampedAxisPosition(Start, End, Edge, Tolerance);
		if (Axis < -Tolerance || Axis > EdgeLength + Tolerance)
		{
			return false;
		}
		OutParameter = FMath::Clamp(Axis / EdgeLength, 0.0, 1.0);
		OutPosition = FMath::Lerp(Edge.Start, Edge.End, OutParameter);
		return true;
	}

	bool FTautPathSolver::StraighteningCrossesEdge(
		const FVector3d& Previous,
		const FVector3d& Contact,
		const FVector3d& Next,
		const FCollisionFeatureId& OwnEdge,
		const FTautCollisionScene& Scene,
		const double DistanceTolerance,
		const double ParametricTolerance)
	{
		// Would collapsing the bend Previous -> Contact -> Next onto the straight chord
		// Previous -> Next sweep the rope across another convex edge? Test each usable
		// edge (except the one this contact rides) against the bend triangle.
		for (const FCollisionEdge& Edge : Scene.Edges)
		{
			if (Edge.Id == OwnEdge || !IsUsableEdge(Edge, DistanceTolerance))
			{
				continue;
			}
			double SegmentParameter = 0.0;
			FVector3d Barycentric;
			if (SegmentIntersectsTriangle(
				Edge.Start, Edge.End, Previous, Contact, Next,
				ParametricTolerance, SegmentParameter, Barycentric))
			{
				return true;
			}
		}
		return false;
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

	bool FTautPathSolver::ValidateState(const FTautCollisionScene* Scene) const
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
			if ((Index == 0 || Index == Points.Num() - 1)
				!= (Point.Type == ETautPointType::Endpoint))
			{
				return false;
			}
			if (Index > 0 && Index + 1 < Points.Num() && !Point.FeatureId.IsValid())
			{
				return false;
			}
			if (Point.Type == ETautPointType::EdgeContact
				&& (Point.EdgeParameter < -Config.ParametricTolerance
					|| Point.EdgeParameter > 1.0 + Config.ParametricTolerance))
			{
				return false;
			}
			if (Scene && Point.Type == ETautPointType::EdgeContact)
			{
				const FCollisionEdge* Edge = FindEdge(Point.FeatureId, *Scene);
				if (!Edge || FVector3d::Distance(
					Point.Position,
					FMath::Lerp(Edge->Start, Edge->End, Point.EdgeParameter)) > Config.TopologyTolerance)
				{
					return false;
				}
			}
			if (Scene && Point.Type == ETautPointType::VertexContact)
			{
				const FCollisionVertex* Vertex = FindVertex(Point.FeatureId, *Scene);
				if (!Vertex || FVector3d::Distance(Point.Position, Vertex->Position) > Config.TopologyTolerance)
				{
					return false;
				}
			}
		}
		if (Scene)
		{
			for (int32 Index = 0; Index + 1 < Points.Num(); ++Index)
			{
				if (!SegmentIsCollisionFree(
					Points[Index].Position, Points[Index + 1].Position, *Scene,
					Config.TopologyTolerance, Config.ParametricTolerance))
				{
					return false;
				}
			}
		}
		return true;
	}

	void FTautPathSolver::UpdateResult(
		const ETautStatus Status,
		const int32 MovementIterations,
		const int32 CollisionPhases,
		const int32 TopologyEvents,
		const FCollisionFeatureId& FailureFeature)
	{
		LastResult.Status = Status;
		LastResult.FailureFeature = FailureFeature;
		LastResult.StepIndex = StepIndex;
		LastResult.PointCount = Points.Num();
		LastResult.ContactCount = FMath::Max(Points.Num() - 2, 0);
		LastResult.MovementIterationCount = MovementIterations;
		LastResult.CollisionPhaseCount = CollisionPhases;
		LastResult.TopologyEventCount = TopologyEvents;
		LastResult.PathLength = CalculatePathLength(Points);
		LastResult.bPathCollisionFree = false;
	}
}
