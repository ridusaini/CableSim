#include "CableSimTautSolver.h"

namespace CableSim
{
	namespace
	{
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
				// AdvanceStep is transactional and has restored the graph-repaired
				// route. A symmetric seed can make its optional first relaxation
				// indeterminate even though that route is already valid; ownership
				// should start from the validated route and let later endpoint motion
				// drive the normal GDC matrix.
				if (ValidateState(&Scene))
				{
					StepIndex = 0;
					LastReplayFrame = FTautReplayFrame{};
					LastPairDecisions.Reset();
					LastVertexDecisions.Reset();
					UpdateResult(
						HasUsableEdge(Scene, Config.TopologyTolerance)
							? ETautStatus::Ready : ETautStatus::NoRelevantGeometry,
						0, 0, 0);
					LastResult.bPathCollisionFree = true;
					return true;
				}
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
		// visible rope's homotopy while swept movement inserts persistent contacts;
		// seed-only simplification removes redundant bends before taut ownership.
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
		LastPairDecisions.Reset();
		LastVertexDecisions.Reset();
		StepIndex = 0;
		CurrentOuterSplitCount = 0;
	}

	FTautStepResult FTautPathSolver::AdvanceStep(
		const FTautStepInput& Input,
		const FTautCollisionScene& Scene)
	{
		LastPairDecisions.Reset();
		LastVertexDecisions.Reset();
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
		CurrentOuterSplitCount = 0;

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

			// A fully coplanar run is more general than the parallel-edge special
			// case below: after unfolding, the taut route is one straight chord. If
			// that chord intersects every finite contact edge in path order, seed the
			// exact intersections directly. Unsupported/skew runs simply keep the
			// normal coordinate iteration.
			for (int32 RunStart = 1; RunStart + 1 < Points.Num(); )
			{
				if (Points[RunStart].Type != ETautPointType::EdgeContact)
				{
					++RunStart;
					continue;
				}
				int32 RunEnd = RunStart;
				TArray<const FCollisionEdge*, TInlineAllocator<8>> RunEdges;
				while (RunEnd + 1 < Points.Num() && Points[RunEnd].Type == ETautPointType::EdgeContact)
				{
					const FCollisionEdge* Edge = FindEdge(Points[RunEnd].FeatureId, Scene);
					if (!Edge)
					{
						break;
					}
					RunEdges.Add(Edge);
					++RunEnd;
				}
				--RunEnd;
				if (RunEdges.Num() >= 2)
				{
					const FVector3d A = Targets[RunStart - 1];
					const FVector3d B = Targets[RunEnd + 1];
					const FVector3d Chord = B - A;
					FVector3d PlaneNormal = FVector3d::ZeroVector;
					for (const FCollisionEdge* Edge : RunEdges)
					{
						PlaneNormal = FVector3d::CrossProduct(Chord, Edge->End - Edge->Start).GetSafeNormal();
						if (!PlaneNormal.IsNearlyZero())
						{
							break;
						}
					}
					bool bCoplanar = !PlaneNormal.IsNearlyZero();
					for (const FCollisionEdge* Edge : RunEdges)
					{
						bCoplanar &= FMath::Abs(FVector3d::DotProduct(
							Edge->Start - A, PlaneNormal)) <= Config.TopologyTolerance;
						bCoplanar &= FMath::Abs(FVector3d::DotProduct(
							Edge->End - A, PlaneNormal)) <= Config.TopologyTolerance;
					}
					TArray<FVector3d, TInlineAllocator<8>> Intersections;
					double PreviousChordParameter = -Config.ParametricTolerance;
					for (const FCollisionEdge* Edge : RunEdges)
					{
						const FVector3d EdgeVector = Edge->End - Edge->Start;
						const FVector3d Relative = A - Edge->Start;
						const double ChordSquared = Chord.SquaredLength();
						const double EdgeSquared = EdgeVector.SquaredLength();
						const double Mixed = FVector3d::DotProduct(Chord, EdgeVector);
						const double ChordRelative = FVector3d::DotProduct(Chord, Relative);
						const double EdgeRelative = FVector3d::DotProduct(EdgeVector, Relative);
						const double Denominator = ChordSquared * EdgeSquared - Mixed * Mixed;
						if (!bCoplanar || Denominator <= UE_DOUBLE_SMALL_NUMBER)
						{
							bCoplanar = false;
							break;
						}
						const double ChordParameter =
							(Mixed * EdgeRelative - EdgeSquared * ChordRelative) / Denominator;
						const double EdgeParameter =
							(ChordSquared * EdgeRelative - Mixed * ChordRelative) / Denominator;
						const FVector3d OnChord = A + ChordParameter * Chord;
						const FVector3d OnEdge = Edge->Start + EdgeParameter * EdgeVector;
						if (ChordParameter < PreviousChordParameter
							|| ChordParameter < -Config.ParametricTolerance
							|| ChordParameter > 1.0 + Config.ParametricTolerance
							|| EdgeParameter < -Config.ParametricTolerance
							|| EdgeParameter > 1.0 + Config.ParametricTolerance
							|| FVector3d::Distance(OnChord, OnEdge) > Config.TopologyTolerance)
						{
							bCoplanar = false;
							break;
						}
						PreviousChordParameter = ChordParameter;
						Intersections.Add(OnEdge);
					}
					if (bCoplanar && Intersections.Num() == RunEdges.Num())
					{
						for (int32 Index = 0; Index < Intersections.Num(); ++Index)
						{
							Targets[RunStart + Index] = Intersections[Index];
						}
					}
				}
				RunStart = FMath::Max(RunEnd + 1, RunStart + 1);
			}

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
					if (Point.bHasPendingTarget)
					{
						NewTarget = Point.PendingTarget;
					}
					else if (Point.Type == ETautPointType::EdgeContact)
					{
						const FCollisionEdge* Edge = FindEdge(Point.FeatureId, Scene);
						if (!Edge)
						{
							return Fail(ETautStatus::FeatureUnavailable, Point.FeatureId);
						}
						const FEdgeMotion Motion = EvaluateEdgeMotion(
							Input.StartTarget, Targets[PointIndex - 1], Targets[PointIndex + 1],
							*Edge, Config.TopologyTolerance);
						if (Motion.Kind == EEdgeMotionKind::Interior
							|| Motion.Kind == EEdgeMotionKind::ReachStart
							|| Motion.Kind == EEdgeMotionKind::ReachEnd)
						{
							NewTarget = Motion.Position;
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
					Points[PointIndex].bHasPendingTarget = false;
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
						Config.ParametricTolerance, Hit))
					{
						const bool bCandidateWins = !bHasHit
							|| Hit.MovementTime < BestHit.MovementTime - Config.ParametricTolerance;
						MergeSweepHits(
							Hit, Config.TopologyTolerance, Config.ParametricTolerance,
							BestHit, bHasHit);
						if (bCandidateWins)
						{
							bHitLeft = false;
						}
					}
				}

				if (!bHasHit)
				{
					Points[PointIndex].Position = MovingTarget;
					Points[PointIndex].bHasPendingTarget = false;
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
				FTautPoint Contact;
				Contact.Position = BestHit.Position;
				if (const FCollisionVertex* HitVertex = FindSharedVertex(
					BestHit.EdgeArrayIndices, BestHit.Position, Scene, Config.TopologyTolerance))
				{
					Contact.Type = ETautPointType::VertexContact;
					Contact.FeatureId = HitVertex->Id;
					Contact.Position = HitVertex->Position;
				}
				else
				{
					const int32 HitEdgeIndex = BestHit.EdgeArrayIndices.IsEmpty()
						? INDEX_NONE : BestHit.EdgeArrayIndices[0];
					if (!Scene.Edges.IsValidIndex(HitEdgeIndex))
					{
						return Fail(ETautStatus::NumericalFailure);
					}
					const FCollisionEdge& HitEdge = Scene.Edges[HitEdgeIndex];
					Contact.Type = ETautPointType::EdgeContact;
					Contact.FeatureId = HitEdge.Id;
					const double EdgeLength = FVector3d::Distance(HitEdge.Start, HitEdge.End);
					Contact.EdgeParameter = EdgeLength > Config.TopologyTolerance
						? FVector3d::Distance(HitEdge.Start, BestHit.Position) / EdgeLength : 0.0;
				}
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
			// The reverse of an outer-corner split is two ordered edge contacts arriving
			// at their shared vertex.  Merge before classifying the vertex again.  Fresh
			// split contacts carry pending targets and must be allowed to separate first.
			for (int32 PointIndex = 1; PointIndex + 2 < Points.Num(); ++PointIndex)
			{
				FTautPoint& First = Points[PointIndex];
				const FTautPoint& Second = Points[PointIndex + 1];
				if (First.Type != ETautPointType::EdgeContact
					|| Second.Type != ETautPointType::EdgeContact
					|| First.bHasPendingTarget || Second.bHasPendingTarget
					|| FVector3d::Distance(First.Position, Second.Position) > Config.TopologyTolerance)
				{
					continue;
				}
				const FCollisionVertex* SharedVertex = nullptr;
				for (const FCollisionVertex& CandidateVertex : Scene.Vertices)
				{
					if (FVector3d::Distance(CandidateVertex.Position, First.Position) <= Config.TopologyTolerance
						&& CandidateVertex.IncidentEdges.Contains(First.FeatureId)
						&& CandidateVertex.IncidentEdges.Contains(Second.FeatureId))
					{
						SharedVertex = &CandidateVertex;
						break;
					}
				}
				if (!SharedVertex)
				{
					continue;
				}
				First.Type = ETautPointType::VertexContact;
				First.Position = SharedVertex->Position;
				First.FeatureId = SharedVertex->Id;
				First.EdgeParameter = 0.0;
				First.SingularOrientation = 0;
				Points.RemoveAt(PointIndex + 1);
				bTopologyChanged = true;
				break;
			}
			if (bTopologyChanged)
			{
				if (++TopologyEvents > Config.MaximumTopologyEvents)
				{
					return EmitBestEffort(ETautStatus::TopologyBudgetExceeded);
				}
				continue;
			}
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
					const FEdgeMotion Motion = EvaluateEdgeMotion(
						Input.StartTarget, Previous, Next, *Edge, Config.TopologyTolerance);
					if (Motion.Kind == EEdgeMotionKind::Release)
					{
						// Support loss is the release event. The chord check only protects the
						// path from another obstacle; it does not decide whether this edge is
						// still wrapped.
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
					if (Motion.Kind == EEdgeMotionKind::Singular)
					{
						continue;
					}
					if (Motion.Kind == EEdgeMotionKind::Interior)
					{
						Point.Position = Motion.Position;
						Point.EdgeParameter = Motion.Parameter;
						continue;
					}

					const bool bAtStart = Motion.Kind == EEdgeMotionKind::ReachStart;
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
					Point.SingularOrientation = 0;
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
				TSet<FCollisionFeatureId> LocallyUnsupported;
				for (const FCollisionFeatureId& IncidentId : Vertex->IncidentEdges)
				{
					const FCollisionEdge* Incident = FindEdge(IncidentId, Scene);
					if (Incident && IsUsableEdge(*Incident, Config.TopologyTolerance))
					{
						Candidates.Add(Incident);
					}
				}
				// Build the full GDC matrix for diagnostics, but do not let an edge whose
				// convex wedge has lost the local chord participate in a Stable hold or
				// survive the resolution.
				for (const FCollisionEdge* Candidate : Candidates)
				{
					if (EvaluateEdgeMotion(
						Input.StartTarget, Previous, Next, *Candidate,
						Config.TopologyTolerance).Kind == EEdgeMotionKind::Release)
					{
						LocallyUnsupported.Add(Candidate->Id);
					}
				}
				Candidates.Sort([](const FCollisionEdge& First, const FCollisionEdge& Second)
				{
					return FCollisionFeatureId::Less(First.Id, Second.Id);
				});
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
						if (LastVertexDecisions.Num() < 128)
						{
							LastVertexDecisions.Add({Vertex->Id,
								ETautVertexResolution::Release, 0, 0, 0});
						}
						Points.RemoveAt(PointIndex);
						bTopologyChanged = true;
						break;
					}
					if (LastVertexDecisions.Num() < 128)
					{
						LastVertexDecisions.Add({Vertex->Id,
							ETautVertexResolution::ConflictHold, 0, 0, 0});
					}
					continue;
				}

				TArray<FPairClassification, TInlineAllocator<28>> Pairs;
				int8 SelectedSingularOrientation = 0;
				const FVector3d StartEndpointMotion = Input.StartTarget
					- LastValidState.Points[0].Position;
				const FVector3d EndEndpointMotion = Input.EndTarget
					- LastValidState.Points.Last().Position;
				for (int32 A = 0; A < Candidates.Num(); ++A)
				{
					for (int32 B = A + 1; B < Candidates.Num(); ++B)
					{
						FPairClassification Pair = ClassifyEdgePair(
							Input.StartTarget, Previous, Next, *Vertex,
							*Candidates[A], *Candidates[B],
							Config.TopologyTolerance);
						if (Pair.Action == ETautPairAction::Singular)
						{
							// Evaluate the two continuous limits of the exact predicate. History is
							// allowed to remember only which side of zero was selected, never the
							// Stable/Move/Ignore action produced by an earlier frame.
							const FVector3d RayA = ((FVector3d::Distance(
								Pair.EdgeA->Start, Vertex->Position) <= Config.TopologyTolerance
									? Pair.EdgeA->End : Pair.EdgeA->Start) - Vertex->Position).GetSafeNormal();
							const FVector3d RayB = ((FVector3d::Distance(
								Pair.EdgeB->Start, Vertex->Position) <= Config.TopologyTolerance
									? Pair.EdgeB->End : Pair.EdgeB->Start) - Vertex->Position).GetSafeNormal();
							FVector3d Bias = Pair.EdgeA->FaceNormal0 + Pair.EdgeA->FaceNormal1
								+ Pair.EdgeB->FaceNormal0 + Pair.EdgeB->FaceNormal1
								+ FVector3d::CrossProduct(RayA, RayB);
							if (!Bias.Normalize())
							{
								Bias = FVector3d::CrossProduct(
									RayA, Pair.EdgeA->FaceNormal0).GetSafeNormal();
							}
							if (Bias.IsNearlyZero())
							{
								Bias = Pair.EdgeA->FaceNormal0.GetSafeNormal();
							}
							const double BiasDistance = FMath::Max(
								Config.TopologyTolerance * 2.0, 1.e-6);
							const double PredicateTolerance = FMath::Max(
								Config.TopologyTolerance * 1.e-4, 1.e-9);
							FPairClassification Positive = ClassifyEdgePair(
								Input.StartTarget,
								Previous + Bias * BiasDistance,
								Next - Bias * BiasDistance,
								*Vertex, *Pair.EdgeA, *Pair.EdgeB, PredicateTolerance);
							FPairClassification Negative = ClassifyEdgePair(
								Input.StartTarget,
								Previous - Bias * BiasDistance,
								Next + Bias * BiasDistance,
								*Vertex, *Pair.EdgeA, *Pair.EdgeB, PredicateTolerance);
							int8 Orientation = 0;
							if (Positive.Action != ETautPairAction::Singular
								&& Positive.Action == Negative.Action
								&& Positive.PairCase == Negative.PairCase)
							{
								Pair = Positive;
							}
							else
							{
								const double MotionSide = FVector3d::DotProduct(
									StartEndpointMotion - EndEndpointMotion, Bias);
								if (FMath::Abs(MotionSide) > Config.MovementConvergenceTolerance)
								{
									Orientation = MotionSide > 0.0 ? 1 : -1;
								}
								else if (Point.SingularOrientation != 0)
								{
									Orientation = Point.SingularOrientation;
								}
								else
								{
									Orientation = FCollisionFeatureId::Less(
										Pair.EdgeA->Id, Pair.EdgeB->Id) ? 1 : -1;
								}
								Pair = Orientation > 0 ? Positive : Negative;
								if (Pair.Action == ETautPairAction::Singular)
								{
									Pair = Orientation > 0 ? Negative : Positive;
									Orientation = -Orientation;
								}
							}
							Pair.bUsedOneSidedLimit = true;
							Pair.SingularOrientation = Orientation;
							if (Orientation != 0 && SelectedSingularOrientation == 0)
							{
								SelectedSingularOrientation = Orientation;
							}
						}
						Pairs.Add(Pair);
						if (LastPairDecisions.Num() < 128)
						{
							LastPairDecisions.Add({
								Vertex->Id,
								Pair.EdgeA->Id,
								Pair.EdgeB->Id,
								Pair.PairCase,
								Pair.Action,
								Pair.bUsedOneSidedLimit,
								Pair.SingularOrientation});
						}
					}
				}
				Point.SingularOrientation = SelectedSingularOrientation;

				TSet<FCollisionFeatureId> Ignored;
				for (const FPairClassification& Pair : Pairs)
				{
					if (Pair.Action == ETautPairAction::IgnoreA)
					{
						Ignored.Add(Pair.EdgeA->Id);
					}
					else if (Pair.Action == ETautPairAction::IgnoreB)
					{
						Ignored.Add(Pair.EdgeB->Id);
					}
				}
				auto IsActive = [&Ignored, &LocallyUnsupported](const FCollisionEdge* Edge)
				{
					return !Ignored.Contains(Edge->Id)
						&& !LocallyUnsupported.Contains(Edge->Id);
				};
				int32 ActiveEdgeCount = 0;
				for (const FCollisionEdge* Candidate : Candidates)
				{
					ActiveEdgeCount += IsActive(Candidate) ? 1 : 0;
				}
				int32 IndeterminatePairCount = 0;
				int32 OneSidedPairCount = 0;
				for (const FPairClassification& Pair : Pairs)
				{
					IndeterminatePairCount += Pair.Action == ETautPairAction::Singular ? 1 : 0;
					OneSidedPairCount += Pair.bUsedOneSidedLimit ? 1 : 0;
				}
				auto RecordResolution = [this, Vertex, ActiveEdgeCount,
					IndeterminatePairCount, OneSidedPairCount](const ETautVertexResolution Resolution)
				{
					if (LastVertexDecisions.Num() < 128)
					{
						LastVertexDecisions.Add({Vertex->Id, Resolution, ActiveEdgeCount,
							IndeterminatePairCount, OneSidedPairCount});
					}
				};
				if (ActiveEdgeCount == 0)
				{
					if (SegmentIsCollisionFree(
						Previous, Next, Scene, Config.TopologyTolerance,
						Config.ParametricTolerance))
					{
						RecordResolution(ETautVertexResolution::Release);
						Points.RemoveAt(PointIndex);
						bTopologyChanged = true;
						break;
					}
					Point.Position = Vertex->Position;
					RecordResolution(ETautVertexResolution::ConflictHold);
					continue;
				}

				const bool bStableInner = Pairs.ContainsByPredicate([&IsActive](const FPairClassification& Pair)
				{
					return Pair.Action == ETautPairAction::StableInner
						&& IsActive(Pair.EdgeA) && IsActive(Pair.EdgeB);
				});
				if (bStableInner)
				{
					Point.Position = Vertex->Position;
					RecordResolution(ETautVertexResolution::StableHold);
					continue;
				}

				TSet<FCollisionFeatureId> Abandoned = Ignored;
				TSet<FCollisionFeatureId> Supported;
				auto ChooseUnstableEdge = [&](const FCollisionEdge* EdgeA, const FCollisionEdge* EdgeB)
				{
					auto LocalRouteLength = [&](const FCollisionEdge* Edge)
					{
						const FEdgeMotion Motion = EvaluateEdgeMotion(
							Input.StartTarget, Previous, Next, *Edge, Config.TopologyTolerance);
						const FVector3d TargetPosition = Motion.Kind == EEdgeMotionKind::Singular
							|| Motion.Kind == EEdgeMotionKind::Release
							? Vertex->Position : Motion.Position;
						return FVector3d::Distance(Previous, TargetPosition)
							+ FVector3d::Distance(TargetPosition, Next);
					};
					const double DistanceA = LocalRouteLength(EdgeA);
					const double DistanceB = LocalRouteLength(EdgeB);
					if (!FMath::IsNearlyEqual(DistanceA, DistanceB, Config.TopologyTolerance))
					{
						return DistanceA < DistanceB ? EdgeA : EdgeB;
					}
					return FCollisionFeatureId::Less(EdgeA->Id, EdgeB->Id) ? EdgeA : EdgeB;
				};
				for (const FPairClassification& Pair : Pairs)
				{
					if (Pair.Action == ETautPairAction::IgnoreA)
					{
						Supported.Add(Pair.EdgeB->Id);
						continue;
					}
					if (Pair.Action == ETautPairAction::IgnoreB)
					{
						Supported.Add(Pair.EdgeA->Id);
						continue;
					}
					if (!IsActive(Pair.EdgeA) || !IsActive(Pair.EdgeB))
					{
						continue;
					}
					switch (Pair.Action)
					{
					case ETautPairAction::MoveAlongA:
						Supported.Add(Pair.EdgeA->Id);
						Abandoned.Add(Pair.EdgeB->Id);
						break;
					case ETautPairAction::MoveAlongB:
						Supported.Add(Pair.EdgeB->Id);
						Abandoned.Add(Pair.EdgeA->Id);
						break;
					case ETautPairAction::ChooseAOrB:
					{
						const FCollisionEdge* Chosen = ChooseUnstableEdge(Pair.EdgeA, Pair.EdgeB);
						Supported.Add(Chosen->Id);
						Abandoned.Add(Chosen == Pair.EdgeA ? Pair.EdgeB->Id : Pair.EdgeA->Id);
						break;
					}
					case ETautPairAction::OuterAThenB:
					case ETautPairAction::OuterBThenA:
						Supported.Add(Pair.EdgeA->Id);
						Supported.Add(Pair.EdgeB->Id);
						break;
					default:
						break;
					}
				}
				if (Candidates.Num() == 1 && IsActive(Candidates[0]))
				{
					const FEdgeMotion Motion = EvaluateEdgeMotion(
						Input.StartTarget, Previous, Next, *Candidates[0],
						Config.TopologyTolerance);
					if (Motion.Kind != EEdgeMotionKind::Release
						&& Motion.Kind != EEdgeMotionKind::Singular
						&& FVector3d::Distance(Motion.Position, Vertex->Position)
							> Config.MovementConvergenceTolerance)
					{
						Supported.Add(Candidates[0]->Id);
					}
				}

				TArray<const FCollisionEdge*, TInlineAllocator<8>> Surviving;
				for (const FCollisionEdge* Candidate : Candidates)
				{
					if (IsActive(Candidate)
						&& Supported.Contains(Candidate->Id)
						&& !Abandoned.Contains(Candidate->Id))
					{
						Surviving.Add(Candidate);
					}
				}
				auto MakeEdgeTransition = [&](const FCollisionEdge& Edge)
				{
					FTautPoint Transition;
					Transition.Type = ETautPointType::EdgeContact;
					Transition.FeatureId = Edge.Id;
					Transition.Position = Vertex->Position;
					const double EdgeLength = FVector3d::Distance(Edge.Start, Edge.End);
					Transition.EdgeParameter = EdgeLength > Config.TopologyTolerance
						? FVector3d::Distance(Edge.Start, Vertex->Position) / EdgeLength : 0.0;
					const FEdgeMotion Motion = EvaluateEdgeMotion(
						Input.StartTarget, Previous, Next, Edge, Config.TopologyTolerance);
					const FVector3d TargetPosition = Motion.Kind == EEdgeMotionKind::Singular
						|| Motion.Kind == EEdgeMotionKind::Release
						? Vertex->Position : Motion.Position;
					Transition.PendingTarget = TargetPosition;
					Transition.bHasPendingTarget = FVector3d::Distance(
						TargetPosition, Vertex->Position) > Config.MovementConvergenceTolerance;
					return Transition;
				};

				if (Surviving.IsEmpty())
				{
					if (IndeterminatePairCount > 0)
					{
						const FCollisionEdge* BestSingularEdge = nullptr;
						FTautPoint BestSingularTransition;
						double BestSingularLength = TNumericLimits<double>::Max();
						for (const FCollisionEdge* Candidate : Candidates)
						{
							if (!IsActive(Candidate))
							{
								continue;
							}
							FTautPoint Transition = MakeEdgeTransition(*Candidate);
							if (!Transition.bHasPendingTarget)
							{
								continue;
							}
							const double RouteLength = FVector3d::Distance(
								Previous, Transition.PendingTarget)
								+ FVector3d::Distance(Transition.PendingTarget, Next);
							if (!BestSingularEdge
								|| RouteLength < BestSingularLength - Config.TopologyTolerance
								|| (FMath::IsNearlyEqual(
									RouteLength, BestSingularLength, Config.TopologyTolerance)
									&& FCollisionFeatureId::Less(Candidate->Id, BestSingularEdge->Id)))
							{
								BestSingularEdge = Candidate;
								BestSingularTransition = MoveTemp(Transition);
								BestSingularLength = RouteLength;
							}
						}
						if (BestSingularEdge)
						{
							RecordResolution(ETautVertexResolution::MoveAlong);
							Point = MoveTemp(BestSingularTransition);
							bTopologyChanged = true;
							break;
						}
						Point.Position = Vertex->Position;
						RecordResolution(ETautVertexResolution::SingularHold);
						continue;
					}
					if (SegmentIsCollisionFree(
						Previous, Next, Scene, Config.TopologyTolerance,
						Config.ParametricTolerance))
					{
						RecordResolution(ETautVertexResolution::Release);
						Points.RemoveAt(PointIndex);
						bTopologyChanged = true;
						break;
					}
					Point.Position = Vertex->Position;
					RecordResolution(ETautVertexResolution::ConflictHold);
					continue;
				}
				if (Surviving.Num() == 1)
				{
					FTautPoint Transition = MakeEdgeTransition(*Surviving[0]);
					if (Transition.bHasPendingTarget)
					{
						RecordResolution(ETautVertexResolution::MoveAlong);
						Point = MoveTemp(Transition);
						bTopologyChanged = true;
						break;
					}
					if (SegmentIsCollisionFree(
						Previous, Next, Scene, Config.TopologyTolerance,
						Config.ParametricTolerance))
					{
						RecordResolution(ETautVertexResolution::Release);
						Points.RemoveAt(PointIndex);
						bTopologyChanged = true;
						break;
					}
					Point.Position = Vertex->Position;
					RecordResolution(ETautVertexResolution::ConflictHold);
					continue;
				}

				if (Surviving.Num() == 2 && Points.Num() < Config.MaximumPathPoints)
				{
					const FCollisionEdge* FirstOuter = nullptr;
					const FCollisionEdge* SecondOuter = nullptr;
					for (const FPairClassification& Pair : Pairs)
					{
						const bool bMatches = (Pair.EdgeA == Surviving[0] && Pair.EdgeB == Surviving[1])
							|| (Pair.EdgeA == Surviving[1] && Pair.EdgeB == Surviving[0]);
						if (!bMatches)
						{
							continue;
						}
						if (Pair.Action == ETautPairAction::OuterAThenB)
						{
							FirstOuter = Pair.EdgeA;
							SecondOuter = Pair.EdgeB;
						}
						else if (Pair.Action == ETautPairAction::OuterBThenA)
						{
							FirstOuter = Pair.EdgeB;
							SecondOuter = Pair.EdgeA;
						}
					}
					if (FirstOuter && SecondOuter)
					{
						FTautPoint FirstPoint = MakeEdgeTransition(*FirstOuter);
						FTautPoint SecondPoint = MakeEdgeTransition(*SecondOuter);
						if (FirstPoint.bHasPendingTarget && SecondPoint.bHasPendingTarget)
						{
							RecordResolution(ETautVertexResolution::OuterSplit);
							Point = MoveTemp(FirstPoint);
							Points.Insert(MoveTemp(SecondPoint), PointIndex + 1);
							++CurrentOuterSplitCount;
							bTopologyChanged = true;
							break;
						}
						if (FirstPoint.bHasPendingTarget != SecondPoint.bHasPendingTarget)
						{
							RecordResolution(ETautVertexResolution::MoveAlong);
							Point = FirstPoint.bHasPendingTarget
								? MoveTemp(FirstPoint) : MoveTemp(SecondPoint);
							bTopologyChanged = true;
							break;
						}
					}
				}

				// Multiple incompatible decisive routes are a real matrix conflict, not
				// a stable inner corner. Keep the safe state for this step and expose the
				// reason so it cannot masquerade as a valid permanent catch.
				Point.Position = Vertex->Position;
				RecordResolution(ETautVertexResolution::ConflictHold);
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

	const FCollisionVertex* FTautPathSolver::FindSharedVertex(
		const TConstArrayView<int32> EdgeArrayIndices,
		const FVector3d& Position,
		const FTautCollisionScene& Scene,
		const double Tolerance)
	{
		if (EdgeArrayIndices.IsEmpty())
		{
			return nullptr;
		}
		for (const FCollisionVertex& Vertex : Scene.Vertices)
		{
			if (FVector3d::Distance(Vertex.Position, Position) > Tolerance)
			{
				continue;
			}
			bool bContainsAll = true;
			for (const int32 EdgeIndex : EdgeArrayIndices)
			{
				if (!Scene.Edges.IsValidIndex(EdgeIndex)
					|| !Vertex.IncidentEdges.Contains(Scene.Edges[EdgeIndex].Id))
				{
					bContainsAll = false;
					break;
				}
			}
			if (bContainsAll)
			{
				return &Vertex;
			}
		}
		return nullptr;
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
		OutHit = FSweepHit{};
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
			if (!bFound || MovementTime < OutHit.MovementTime - ParametricTolerance)
			{
				bFound = true;
				OutHit.EdgeArrayIndices.Reset();
				OutHit.EdgeArrayIndices.Add(EdgeIndex);
				OutHit.MovementTime = MovementTime;
				OutHit.Position = HitPosition;
			}
			else if (FMath::IsNearlyEqual(MovementTime, OutHit.MovementTime, ParametricTolerance)
				&& FVector3d::Distance(HitPosition, OutHit.Position) <= DistanceTolerance)
			{
				OutHit.EdgeArrayIndices.Add(EdgeIndex);
			}
		}
		OutHit.EdgeArrayIndices.Sort([&Scene](const int32 First, const int32 Second)
		{
			return FCollisionFeatureId::Less(Scene.Edges[First].Id, Scene.Edges[Second].Id);
		});
		return bFound;
	}

	void FTautPathSolver::MergeSweepHits(
		const FSweepHit& Candidate,
		const double DistanceTolerance,
		const double ParametricTolerance,
		FSweepHit& InOutBest,
		bool& bInOutHasHit)
	{
		if (!bInOutHasHit || Candidate.MovementTime < InOutBest.MovementTime - ParametricTolerance)
		{
			InOutBest = Candidate;
			bInOutHasHit = true;
			return;
		}
		if (!FMath::IsNearlyEqual(
			Candidate.MovementTime, InOutBest.MovementTime, ParametricTolerance)
			|| FVector3d::Distance(Candidate.Position, InOutBest.Position) > DistanceTolerance)
		{
			return;
		}
		for (const int32 EdgeIndex : Candidate.EdgeArrayIndices)
		{
			InOutBest.EdgeArrayIndices.AddUnique(EdgeIndex);
		}
		InOutBest.EdgeArrayIndices.Sort();
	}

	FTautPathSolver::FEdgeMotion FTautPathSolver::EvaluateEdgeMotion(
		const FVector3d& RopeOrigin,
		const FVector3d& Start,
		const FVector3d& End,
		const FCollisionEdge& Edge,
		const double Tolerance)
	{
		FEdgeMotion Result;
		const FVector3d EdgeVector = Edge.End - Edge.Start;
		const double EdgeLength = EdgeVector.Length();
		const FVector3d Normal0 = Edge.FaceNormal0.GetSafeNormal();
		const FVector3d Normal1 = Edge.FaceNormal1.GetSafeNormal();
		if (EdgeLength <= Tolerance || Normal0.IsNearlyZero() || Normal1.IsNearlyZero())
		{
			return Result;
		}

		// A GDC edge point is supported only when its two neighbouring rope segments
		// leave through opposite incident-face regions. Merely intersecting the
		// edge's infinite solid wedge is insufficient: an old contact on the opposite
		// corner of a box lies inside that wedge and can otherwise make two stale
		// contacts support each other forever.
		const double Start0 = FVector3d::DotProduct(Normal0, Start - Edge.Start);
		const double End0 = FVector3d::DotProduct(Normal0, End - Edge.Start);
		const double Start1 = FVector3d::DotProduct(Normal1, Start - Edge.Start);
		const double End1 = FVector3d::DotProduct(Normal1, End - Edge.Start);
		auto FaceRegion = [Tolerance](const double Distance0, const double Distance1)
		{
			const bool bFace0 = Distance0 >= -Tolerance
				&& Distance1 <= Tolerance
				&& Distance0 > Distance1 + Tolerance;
			const bool bFace1 = Distance1 >= -Tolerance
				&& Distance0 <= Tolerance
				&& Distance1 > Distance0 + Tolerance;
			return bFace0 ? 0 : (bFace1 ? 1 : INDEX_NONE);
		};
		const int32 StartRegion = FaceRegion(Start0, Start1);
		const int32 EndRegion = FaceRegion(End0, End1);
		if (StartRegion == INDEX_NONE || EndRegion == INDEX_NONE || StartRegion == EndRegion)
		{
			Result.Kind = EEdgeMotionKind::Release;
			return Result;
		}

		// Establish the same global orientation used by the GDC pair matrix. The
		// chosen incident face is the one facing the rope origin, and +edge is
		// reversed until the origin lies on its right-hand side.
		FVector3d Direction = EdgeVector / EdgeLength;
		FVector3d OrientedStart = Edge.Start;
		const double Facing0 = FVector3d::DotProduct(Normal0, RopeOrigin - Edge.Start);
		const double Facing1 = FVector3d::DotProduct(Normal1, RopeOrigin - Edge.Start);
		const FVector3d FacingNormal = Facing0 >= Facing1 ? Normal0 : Normal1;
		const double RightSide = FVector3d::DotProduct(
			RopeOrigin - Edge.Start, FVector3d::CrossProduct(Direction, FacingNormal));
		if (RightSide < 0.0)
		{
			Direction = -Direction;
			OrientedStart = Edge.End;
		}

		const double StartAxis = FVector3d::DotProduct(Start - OrientedStart, Direction);
		const double EndAxis = FVector3d::DotProduct(End - OrientedStart, Direction);
		const double StartRadius = ((Start - OrientedStart) - Direction * StartAxis).Length();
		const double EndRadius = ((End - OrientedStart) - Direction * EndAxis).Length();
		const double SignedStart = -StartRadius;
		const double SignedEnd = EndRadius;
		const double SignedDenominator = SignedEnd - SignedStart;
		if (SignedDenominator <= Tolerance)
		{
			return Result;
		}

		const double ChordParameter = -SignedStart / SignedDenominator;
		const double OrientedAxis = FMath::Lerp(StartAxis, EndAxis, ChordParameter);
		const FVector3d UnclampedPosition = OrientedStart + Direction * OrientedAxis;
		const FVector3d StoredDirection = EdgeVector / EdgeLength;
		Result.UnclampedAxis = FVector3d::DotProduct(
			UnclampedPosition - Edge.Start, StoredDirection);
		Result.Parameter = FMath::Clamp(Result.UnclampedAxis / EdgeLength, 0.0, 1.0);
		Result.Position = FMath::Lerp(Edge.Start, Edge.End, Result.Parameter);
		if (Result.UnclampedAxis <= Tolerance)
		{
			Result.Kind = EEdgeMotionKind::ReachStart;
		}
		else if (Result.UnclampedAxis >= EdgeLength - Tolerance)
		{
			Result.Kind = EEdgeMotionKind::ReachEnd;
		}
		else
		{
			Result.Kind = EEdgeMotionKind::Interior;
		}
		return Result;
	}

	FTautPathSolver::FPairClassification FTautPathSolver::ClassifyEdgePair(
		const FVector3d& RopeOrigin,
		const FVector3d& Previous,
		const FVector3d& Next,
		const FCollisionVertex& Vertex,
		const FCollisionEdge& EdgeA,
		const FCollisionEdge& EdgeB,
		const double Tolerance)
	{
		FPairClassification Result;
		Result.EdgeA = &EdgeA;
		Result.EdgeB = &EdgeB;

		const double LengthA = FVector3d::Distance(EdgeA.Start, EdgeA.End);
		const double LengthB = FVector3d::Distance(EdgeB.Start, EdgeB.End);
		const double AngularTolerance = FMath::Clamp(
			Tolerance / FMath::Max(FMath::Min(LengthA, LengthB), Tolerance), 1.e-8, 1.e-3);
		auto OrientEdge = [&](const FCollisionEdge& Edge, FVector3d& OutDirection)
		{
			OutDirection = (Edge.End - Edge.Start).GetSafeNormal();
			const double Facing0 = FVector3d::DotProduct(
				Edge.FaceNormal0, RopeOrigin - Vertex.Position);
			const double Facing1 = FVector3d::DotProduct(
				Edge.FaceNormal1, RopeOrigin - Vertex.Position);
			const FVector3d FacingNormal = Facing0 >= Facing1 ? Edge.FaceNormal0 : Edge.FaceNormal1;
			if (OutDirection.IsNearlyZero() || FMath::Max(Facing0, Facing1) <= Tolerance)
			{
				return false;
			}
			// Standing on the outward face and looking along +edge, cross(edge, normal)
			// is screen-right.  Reverse the arbitrary stored endpoint order until the
			// rope origin lies on that required side.
			const double RightSide = FVector3d::DotProduct(
				RopeOrigin - Vertex.Position,
				FVector3d::CrossProduct(OutDirection, FacingNormal));
			if (FMath::Abs(RightSide) <= Tolerance)
			{
				return false;
			}
			if (RightSide < 0.0)
			{
				OutDirection = -OutDirection;
			}
			return true;
		};

		FVector3d DirectionA;
		FVector3d DirectionB;
		if (!OrientEdge(EdgeA, DirectionA) || !OrientEdge(EdgeB, DirectionB))
		{
			return Result;
		}
		FVector3d PairNormal = FVector3d::CrossProduct(DirectionA, DirectionB);
		const double PairNormalLength = PairNormal.Length();
		if (PairNormalLength <= AngularTolerance)
		{
			return Result;
		}
		PairNormal /= PairNormalLength;
		double PreviousHeight = FVector3d::DotProduct(Previous - Vertex.Position, PairNormal);
		double NextHeight = FVector3d::DotProduct(Next - Vertex.Position, PairNormal);
		if (FMath::Abs(PreviousHeight) <= Tolerance || FMath::Abs(NextHeight) <= Tolerance)
		{
			return Result;
		}
		if (PreviousHeight < 0.0)
		{
			PairNormal = -PairNormal;
			PreviousHeight = -PreviousHeight;
			NextHeight = -NextHeight;
		}
		const bool bDoubleCrossing = NextHeight > Tolerance;

		auto OutwardDirection = [&](const FCollisionEdge& Edge)
		{
			const bool bVertexAtStart = FVector3d::Distance(
				Edge.Start, Vertex.Position) <= Tolerance;
			return ((bVertexAtStart ? Edge.End : Edge.Start) - Vertex.Position).GetSafeNormal();
		};
		const FVector3d OutwardA = OutwardDirection(EdgeA);
		const FVector3d OutwardB = OutwardDirection(EdgeB);
		if (OutwardA.IsNearlyZero() || OutwardB.IsNearlyZero())
		{
			return Result;
		}
		auto IsRightOf = [&](const FVector3d& OrientedEdge, const FVector3d& Ray, bool& bOutRight)
		{
			const double Signed = FVector3d::DotProduct(
				FVector3d::CrossProduct(OrientedEdge, Ray), PairNormal);
			if (FMath::Abs(Signed) <= AngularTolerance)
			{
				return false;
			}
			bOutRight = Signed < 0.0;
			return true;
		};
		bool bAOnRightOfB = false;
		bool bBOnRightOfA = false;
		if (!IsRightOf(DirectionB, OutwardA, bAOnRightOfB)
			|| !IsRightOf(DirectionA, OutwardB, bBOnRightOfA))
		{
			return Result;
		}

		if (bDoubleCrossing && bAOnRightOfB != bBOnRightOfA)
		{
			Result.PairCase = ETautPairCase::Outer;
			Result.Action = bAOnRightOfB
				? ETautPairAction::OuterAThenB : ETautPairAction::OuterBThenA;
			return Result;
		}
		if (!bDoubleCrossing && bAOnRightOfB != bBOnRightOfA)
		{
			Result.PairCase = ETautPairCase::Side;
			// If B's physical ray is not on the required side of A, A blocks B.
			Result.Action = !bBOnRightOfA ? ETautPairAction::IgnoreB : ETautPairAction::IgnoreA;
			return Result;
		}
		Result.PairCase = bAOnRightOfB && bBOnRightOfA
			? ETautPairCase::Inner : ETautPairCase::Unstable;

		auto WantsOutward = [&](const FCollisionEdge& Edge, const FVector3d& Outward)
		{
			const FEdgeMotion Motion = EvaluateEdgeMotion(
				RopeOrigin, Previous, Next, Edge, Tolerance);
			const double Axis = Motion.UnclampedAxis;
			const FVector3d AxisPosition = Edge.Start
				+ (Edge.End - Edge.Start).GetSafeNormal() * Axis;
			return FVector3d::DotProduct(AxisPosition - Vertex.Position, Outward) > Tolerance;
		};
		const bool bOutA = WantsOutward(EdgeA, OutwardA);
		const bool bOutB = WantsOutward(EdgeB, OutwardB);
		if (bOutA && !bOutB)
		{
			Result.Action = ETautPairAction::MoveAlongA;
		}
		else if (!bOutA && bOutB)
		{
			Result.Action = ETautPairAction::MoveAlongB;
		}
		else if (bAOnRightOfB && bBOnRightOfA && !bOutA && !bOutB)
		{
			Result.Action = ETautPairAction::StableInner;
		}
		else if (bOutA && bOutB)
		{
			Result.Action = ETautPairAction::ChooseAOrB;
		}
		else
		{
			// This is indeterminate, not a stable inner corner. Reclassification after
			// endpoint motion must be able to release it.
			Result.PairCase = ETautPairCase::Indeterminate;
			Result.Action = ETautPairAction::Singular;
		}
		return Result;
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
			if (!IsFinite(Point.Position) || !FMath::IsFinite(Point.EdgeParameter)
				|| (Point.bHasPendingTarget && !IsFinite(Point.PendingTarget)))
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
		LastResult.OuterSplitCount = CurrentOuterSplitCount;
		LastResult.PathLength = CalculatePathLength(Points);
		LastResult.bPathCollisionFree = false;
	}
}
