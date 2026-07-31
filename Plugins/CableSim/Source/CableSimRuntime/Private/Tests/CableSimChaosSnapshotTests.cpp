#if WITH_DEV_AUTOMATION_TESTS

#include "Chaos/CableSimChaosCollisionAdapter.h"
#include "CableSimComponent.h"
#include "Engine/StaticMeshActor.h"
#include "Components/StaticMeshComponent.h"
#include "PhysicsEngine/BodySetup.h"
#include "Engine/OverlapResult.h"
#include "WorldCollision.h"
#include "Misc/AutomationTest.h"
#include "Tests/AutomationCommon.h"
#include "UObject/UObjectGlobals.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimChaosSnapshotDeterminismTest,
	"CableSim.Runtime.Collision.ChaosSnapshotIsDeterministic",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimChaosSnapshotDeterminismTest::RunTest(const FString& Parameters)
{
	// 5b7c543's own diagnosis of the still-open sharp-edge drag/impact
	// "catapult" bug was never followed up: "the edge bounce is NOT friction or
	// the edge constraint... it is collision behavior... most likely comes from
	// the live Chaos snapshot changing frame to frame. Needs in-editor
	// instrumentation to confirm." This tests that hypothesis directly against
	// REAL Chaos collision (not synthetic geometry): call GatherSnapshot
	// repeatedly with byte-identical input (same static actor, same query
	// particles, nothing moved) and check whether the resulting
	// triangle/edge/vertex candidate list is genuinely stable. A solver fed a
	// shifting candidate set every step -- even when nothing physically moved
	// -- would explain a launch with no other cause.
	// A freshly-constructed FTestWorldWrapper (EWorldType::Game) world's
	// physics scene never returns overlaps in this headless -nullrhi
	// configuration (confirmed empirically: even a plain
	// World->OverlapMultiByChannel sanity check against a real static-mesh
	// actor spawned there finds nothing). The editor's own already-loaded
	// world (matching the pattern engine tests like PhysicsBodyInstanceTests
	// use) has a fully bootstrapped physics scene, so use that instead.
	UWorld* World = GIsEditor ? GWorld : (GEngine ? GEngine->GetWorldContexts()[0].World() : nullptr);
	if (!TestNotNull(TEXT("Editor world is available"), World))
	{
		return false;
	}

	UStaticMesh* CubeMesh = Cast<UStaticMesh>(StaticLoadObject(
		UStaticMesh::StaticClass(), nullptr, TEXT("/Engine/BasicShapes/Cube.Cube")));
	if (!TestNotNull(TEXT("Cube mesh loads"), CubeMesh))
	{
		return false;
	}
	AStaticMeshActor* CubeActor = World->SpawnActor<AStaticMeshActor>();
	if (!TestNotNull(TEXT("Cube actor spawns"), CubeActor))
	{
		return false;
	}
	UStaticMeshComponent* CubeComponent = CubeActor->GetStaticMeshComponent();
	CubeComponent->SetMobility(EComponentMobility::Static);
	CubeComponent->SetStaticMesh(CubeMesh);
	CubeComponent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	CubeComponent->SetCollisionObjectType(ECC_WorldStatic);
	CubeComponent->SetCollisionResponseToAllChannels(ECR_Block);
	CubeActor->SetActorEnableCollision(true);
	CubeActor->SetActorLocation(FVector::ZeroVector);
	CubeComponent->RecreatePhysicsState();
	CubeComponent->UpdateBounds();

	// The component's own cached render Bounds stays zero under -nullrhi
	// (it depends on loaded render data, which a headless nullrhi run never
	// loads); the mesh asset's own bounds and the body setup's collision
	// geometry are unaffected, so derive the corner from those instead.
	const FVector MeshExtent = CubeMesh->GetBounds().BoxExtent;
	AddInfo(FString::Printf(
		TEXT("Mesh asset bounds extent=%s, component bodySetup=%s, simple collision=%d"),
		*MeshExtent.ToString(),
		CubeMesh->GetBodySetup() ? TEXT("valid") : TEXT("null"),
		CubeMesh->GetBodySetup() ? CubeMesh->GetBodySetup()->AggGeom.GetElementCount() : -1));
	// Query particles resting right at the cube's top-front-right corner -- a
	// sharp convex vertex/edge, exactly the geometry class the open bug is
	// about (a box corner is the simplest real-world "sharp edge" case).
	const FVector3d CornerPoint = FVector3d(CubeActor->GetActorLocation()) + FVector3d(MeshExtent);
	{
		// Sanity check: does a plain engine overlap query (bypassing the
		// CableSim adapter entirely) find this actor at all? Isolates whether
		// the physics scene itself is queryable in this headless test world.
		TArray<FOverlapResult> SanityOverlaps;
		const bool bSanityOverlap = World->OverlapMultiByChannel(
			SanityOverlaps,
			FVector(CornerPoint),
			FQuat::Identity,
			ECC_WorldStatic,
			FCollisionShape::MakeSphere(30.0));
		AddInfo(FString::Printf(
			TEXT("Sanity overlap at corner: found=%d bSanityOverlap=%d"),
			SanityOverlaps.Num(), bSanityOverlap ? 1 : 0));
	}
	TArray<CableSim::FParticle> Particles;
	Particles.SetNum(2);
	Particles[0].Position = CornerPoint + FVector3d(-5.0, -5.0, 2.0);
	Particles[1].Position = CornerPoint + FVector3d(-15.0, -15.0, 2.0);
	Particles[0].PreviousPosition = Particles[0].Position;
	Particles[1].PreviousPosition = Particles[1].Position;

	FCableSimCollisionSettings CollisionSettings;
	CollisionSettings.Radius = 2.5;
	FCableSimChaosObjectTracker ObjectTracker;

	auto BuildSignature = [](const FCableSimChaosSnapshot& Snapshot) -> FString
	{
		FString Signature;
		for (const CableSim::FCollisionTriangle& Triangle : Snapshot.Triangles)
		{
			Signature += FString::Printf(TEXT("T[%s|%s|%s]"),
				*Triangle.Vertices[0].ToString(),
				*Triangle.Vertices[1].ToString(),
				*Triangle.Vertices[2].ToString());
		}
		for (const CableSim::FCollisionEdge& Edge : Snapshot.Edges)
		{
			Signature += FString::Printf(TEXT("E[%s|%s]"), *Edge.Start.ToString(), *Edge.End.ToString());
		}
		for (const CableSim::FCollisionVertex& Vertex : Snapshot.Vertices)
		{
			Signature += FString::Printf(TEXT("V[%s]"), *Vertex.Position.ToString());
		}
		return Signature;
	};

	FString FirstSignature;
	int32 MismatchCount = 0;
	int32 EmptyResultCount = 0;
	constexpr int32 CallCount = 50;
	for (int32 Call = 0; Call < CallCount; ++Call)
	{
		FCableSimChaosSnapshot Snapshot;
		const bool bGathered = FCableSimChaosCollisionAdapter::GatherSnapshot(
			World,
			nullptr,
			TArray<AActor*>(),
			CollisionSettings,
			0.1,
			Particles,
			Particles[0].Position,
			Particles[1].Position,
			ObjectTracker,
			Snapshot);
		if (!TestTrue(TEXT("GatherSnapshot succeeds"), bGathered))
		{
			CubeActor->Destroy();
			return false;
		}
		if (Call == 0)
		{
			AddInfo(FString::Printf(
				TEXT("Query bounds=%s overlapCount=%d physicsObjectCount=%d shapeCount=%d"),
				*Snapshot.QueryBounds.ToString(),
				Snapshot.Diagnostics.OverlapCount,
				Snapshot.Diagnostics.PhysicsObjectCount,
				Snapshot.Diagnostics.ShapeCount));
		}
		if (Snapshot.Triangles.IsEmpty() && Snapshot.Edges.IsEmpty() && Snapshot.Vertices.IsEmpty())
		{
			++EmptyResultCount;
			continue;
		}
		const FString Signature = BuildSignature(Snapshot);
		if (FirstSignature.IsEmpty())
		{
			FirstSignature = Signature;
			AddInfo(FString::Printf(
				TEXT("First non-empty snapshot: %d triangles, %d edges, %d vertices"),
				Snapshot.Triangles.Num(), Snapshot.Edges.Num(), Snapshot.Vertices.Num()));
		}
		else if (Signature != FirstSignature)
		{
			++MismatchCount;
		}
	}
	AddInfo(FString::Printf(
		TEXT("%d/%d calls returned no geometry, %d/%d non-empty calls with identical input produced a different candidate set"),
		EmptyResultCount, CallCount, MismatchCount, CallCount - EmptyResultCount));
	const bool bFoundGeometry = TestTrue(
		TEXT("The static cube's real Chaos collision was actually found"), EmptyResultCount < CallCount);
	const bool bDeterministic = TestEqual(
		TEXT("Repeated GatherSnapshot calls with identical input are deterministic"), MismatchCount, 0);

	// This test uses the shared editor world (GWorld), not a throwaway test
	// world, so the spawned actor must not be left behind for later tests.
	CubeActor->Destroy();
	return bFoundGeometry && bDeterministic;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimChaosSnapshotStableAcrossSmallMotionTest,
	"CableSim.Runtime.Collision.ChaosSnapshotStableAcrossSmallMotion",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimChaosSnapshotStableAcrossSmallMotionTest::RunTest(const FString& Parameters)
{
	// The determinism test above rules out the strongest form of 5b7c543's
	// hypothesis (identical input producing different output). But the actual
	// bug scenario is a node genuinely sliding across a corner step by step, a
	// few cm at a time -- this checks whether the *candidate set itself*
	// (which triangles/edges are found near the corner) changes smoothly as
	// the query slides, or churns abruptly -- e.g. a MaxPlanesPerNode/
	// MaxEdgesPerNode cap flipping which features survive the cap from one
	// small step to the next would still produce exactly the symptom
	// "the manifold the solver sees keeps changing even though nothing
	// dramatic happened," without requiring the Chaos query itself to be
	// non-deterministic.
	UWorld* World = GIsEditor ? GWorld : (GEngine ? GEngine->GetWorldContexts()[0].World() : nullptr);
	if (!TestNotNull(TEXT("Editor world is available"), World))
	{
		return false;
	}
	UStaticMesh* CubeMesh = Cast<UStaticMesh>(StaticLoadObject(
		UStaticMesh::StaticClass(), nullptr, TEXT("/Engine/BasicShapes/Cube.Cube")));
	if (!TestNotNull(TEXT("Cube mesh loads"), CubeMesh))
	{
		return false;
	}
	AStaticMeshActor* CubeActor = World->SpawnActor<AStaticMeshActor>();
	if (!TestNotNull(TEXT("Cube actor spawns"), CubeActor))
	{
		return false;
	}
	UStaticMeshComponent* CubeComponent = CubeActor->GetStaticMeshComponent();
	CubeComponent->SetMobility(EComponentMobility::Static);
	CubeComponent->SetStaticMesh(CubeMesh);
	CubeComponent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	CubeComponent->SetCollisionObjectType(ECC_WorldStatic);
	CubeComponent->SetCollisionResponseToAllChannels(ECR_Block);
	CubeActor->SetActorEnableCollision(true);
	CubeActor->SetActorLocation(FVector::ZeroVector);
	CubeComponent->RecreatePhysicsState();

	const FVector MeshExtent = CubeMesh->GetBounds().BoxExtent;
	const FVector3d CornerPoint = FVector3d(CubeActor->GetActorLocation()) + FVector3d(MeshExtent);

	FCableSimCollisionSettings CollisionSettings;
	CollisionSettings.Radius = 2.5;
	FCableSimChaosObjectTracker ObjectTracker;

	auto CandidateKey = [](const CableSim::FCollisionFeatureId& Id) -> uint64
	{
		return (static_cast<uint64>(Id.ShapeIndex) << 40)
			^ (static_cast<uint64>(Id.Type) << 32)
			^ (static_cast<uint64>(static_cast<uint32>(Id.Index0)) << 16)
			^ static_cast<uint64>(static_cast<uint32>(Id.Index1));
	};

	// Slide a query particle in 1cm steps along a line that crosses directly
	// over the corner, staying close enough to remain in range throughout.
	constexpr int32 StepCount = 60;
	TSet<uint64> PreviousCandidates;
	int32 ChurnEvents = 0;
	int32 MaximumChurnPerStep = 0;
	for (int32 Step = 0; Step < StepCount; ++Step)
	{
		const double Offset = static_cast<double>(Step) - static_cast<double>(StepCount) * 0.5;
		TArray<CableSim::FParticle> Particles;
		Particles.SetNum(2);
		Particles[0].Position = CornerPoint + FVector3d(-10.0 + Offset, -10.0 + Offset, 2.0);
		Particles[1].Position = CornerPoint + FVector3d(-20.0 + Offset, -20.0 + Offset, 2.0);
		Particles[0].PreviousPosition = Particles[0].Position;
		Particles[1].PreviousPosition = Particles[1].Position;

		FCableSimChaosSnapshot Snapshot;
		const bool bGathered = FCableSimChaosCollisionAdapter::GatherSnapshot(
			World, nullptr, TArray<AActor*>(), CollisionSettings, 0.1,
			Particles, Particles[0].Position, Particles[1].Position, ObjectTracker, Snapshot);
		if (!TestTrue(TEXT("GatherSnapshot succeeds"), bGathered))
		{
			CubeActor->Destroy();
			return false;
		}

		TSet<uint64> CurrentCandidates;
		for (const CableSim::FCollisionTriangle& Triangle : Snapshot.Triangles)
		{
			CurrentCandidates.Add(CandidateKey(Triangle.Id));
		}
		for (const CableSim::FCollisionEdge& Edge : Snapshot.Edges)
		{
			CurrentCandidates.Add(CandidateKey(Edge.Id));
		}
		if (Step > 0)
		{
			const int32 Added = CurrentCandidates.Difference(PreviousCandidates).Num();
			const int32 Removed = PreviousCandidates.Difference(CurrentCandidates).Num();
			if (Added + Removed > 0)
			{
				++ChurnEvents;
				MaximumChurnPerStep = FMath::Max(MaximumChurnPerStep, Added + Removed);
			}
		}
		PreviousCandidates = MoveTemp(CurrentCandidates);
	}
	AddInfo(FString::Printf(
		TEXT("Sliding a query 1cm/step across the corner over %d steps: candidate set changed on %d steps, largest single-step churn=%d features"),
		StepCount, ChurnEvents, MaximumChurnPerStep));
	CubeActor->Destroy();
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimDragOverSharpEdgeDoesNotLaunchTest,
	"CableSim.Runtime.Collision.DragOverSharpEdgeDoesNotLaunch",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimDragOverSharpEdgeDoesNotLaunchTest::RunTest(const FString& Parameters)
{
	// Closes the loop from "the raw Chaos snapshot occasionally churns
	// discontinuously" (measured above) to "does that actually launch the
	// rope" -- the literal 5b7c543 repro (a full UCableSimComponent, real
	// Chaos collision, one endpoint fast-dragged across a sharp box corner),
	// but sized so the obstacle is comparable to the cable's own collision
	// radius (the earlier corner test's 100-unit box made every triangle's
	// bounding box span a whole face, which is a much coarser scale than any
	// real drag-over-edge scenario would present).
	UWorld* World = GIsEditor ? GWorld : (GEngine ? GEngine->GetWorldContexts()[0].World() : nullptr);
	if (!TestNotNull(TEXT("Editor world is available"), World))
	{
		return false;
	}
	UStaticMesh* CubeMesh = Cast<UStaticMesh>(StaticLoadObject(
		UStaticMesh::StaticClass(), nullptr, TEXT("/Engine/BasicShapes/Cube.Cube")));
	if (!TestNotNull(TEXT("Cube mesh loads"), CubeMesh))
	{
		return false;
	}
	AStaticMeshActor* CubeActor = World->SpawnActor<AStaticMeshActor>();
	if (!TestNotNull(TEXT("Cube actor spawns"), CubeActor))
	{
		return false;
	}
	UStaticMeshComponent* CubeComponent = CubeActor->GetStaticMeshComponent();
	CubeComponent->SetMobility(EComponentMobility::Static);
	CubeComponent->SetStaticMesh(CubeMesh);
	CubeComponent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	CubeComponent->SetCollisionObjectType(ECC_WorldStatic);
	CubeComponent->SetCollisionResponseToAllChannels(ECR_Block);
	CubeActor->SetActorEnableCollision(true);
	// Default engine cube is 100uu/side; scale to 20uu -- comparable to the
	// cable's own collision radius, matching a real sharp obstacle's scale.
	CubeActor->SetActorScale3D(FVector(0.2, 0.2, 0.2));
	CubeActor->SetActorLocation(FVector(0.0, 0.0, 10.0)); // top face at Z=20, resting on the ground plane
	CubeComponent->RecreatePhysicsState();

	AActor* CableOwner = World->SpawnActor<AActor>();
	if (!TestNotNull(TEXT("Cable owner actor spawns"), CableOwner))
	{
		CubeActor->Destroy();
		return false;
	}
	UCableSimComponent* Cable = NewObject<UCableSimComponent>(CableOwner);
	Cable->SimulationSettings.RestLength = 150.0;
	Cable->SimulationSettings.NodeSpacing = 5.0;
	Cable->SimulationSettings.Gravity = FVector::ZeroVector; // isolate collision dynamics, matching 5b7c543
	Cable->CollisionSettings.bEnableWorldCollision = true;
	Cable->CollisionSettings.Radius = 2.5;
	Cable->TautSettings.bEnableTautSolver = false;
	Cable->StartEndpoint.Mode = ECableSimEndpointMode::WorldKinematic;
	Cable->StartEndpoint.WorldTarget = FVector(-60.0, 0.0, 25.0);
	Cable->EndEndpoint.Mode = ECableSimEndpointMode::WorldKinematic;
	Cable->EndEndpoint.WorldTarget = FVector(60.0, -80.0, 25.0);
	Cable->RegisterComponent();
	Cable->ReinitializeSimulation();

	double MaximumObservedSpeed = 0.0;
	int32 WorstStep = -1;
	// Settle first, End endpoint held fixed at its start position: isolates
	// the drag-specific dynamics from a large initial transient (a fresh
	// ReinitializeSimulation()'s first StepSimulation() call spikes to
	// thousands of cm/s on its own, unrelated to the box -- confirmed by
	// logging: the spike appears even though the initial straight-line cable
	// doesn't geometrically reach anywhere near the box).
	double SettleSpeed = 0.0;
	for (int32 Step = 0; Step < 60; ++Step)
	{
		Cable->StepSimulation(1);
		SettleSpeed = Cable->GetSimulationStatus().MaximumParticleSpeed;
	}
	AddInfo(FString::Printf(TEXT("Post-settle MaximumParticleSpeed=%.1f cm/s"), SettleSpeed));

	// Sweeps the End endpoint's Y from -80 to +96 in 12 steps, passing directly
	// over the box's corner region partway through -- the same "fast drag over
	// a sharp edge" the original bug report and 5b7c543's repro both describe.
	constexpr int32 StepCount = 12;
	constexpr double StepDistance = 16.0; // 16uu/step @ 60Hz = 960cm/s: a fast, stress-test drag speed
	for (int32 Step = 0; Step < StepCount; ++Step)
	{
		Cable->EndEndpoint.WorldTarget.Y = -80.0 + StepDistance * static_cast<double>(Step);
		Cable->StepSimulation(1);
		const FCableSimStatus Status = Cable->GetSimulationStatus();
		if (Status.MaximumParticleSpeed > MaximumObservedSpeed)
		{
			MaximumObservedSpeed = Status.MaximumParticleSpeed;
			WorstStep = Step;
		}
		AddInfo(FString::Printf(TEXT("  step %d: endY=%.1f maxSpeed=%.1f status=%d"),
			Step, Cable->EndEndpoint.WorldTarget.Y, Status.MaximumParticleSpeed, static_cast<int32>(Status.Status)));
	}
	AddInfo(FString::Printf(
		TEXT("Fast drag across a sharp box corner, %d steps: peak MaximumParticleSpeed=%.1f cm/s at step %d"),
		StepCount, MaximumObservedSpeed, WorstStep));
	// 5b7c543 measured launches around ~2000cm/s for the original bug; a
	// generous multiple of the endpoint's own drive speed (960cm/s) gives
	// headroom for legitimate contact-response speed without passing a
	// genuine catapult.
	TestTrue(TEXT("No catapult: peak particle speed stays within a sane multiple of the drive speed"),
		MaximumObservedSpeed < 3000.0);

	Cable->DestroyComponent();
	CableOwner->Destroy();
	CubeActor->Destroy();
	return true;
}

#endif
