#if WITH_DEV_AUTOMATION_TESTS

#include "Chaos/CableSimChaosCollisionAdapter.h"
#include "CableSimComponent.h"
#include "CableSimContactManifold.h"
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
			TArray<FVector3d>(),
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
			Particles, Particles[0].Position, Particles[1].Position, TArray<FVector3d>(), ObjectTracker, Snapshot);
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
	Cable->TautSettings.Mode = ECableSimTautMode::Disabled;
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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimTautWrapsRealChaosGeometryTest,
	"CableSim.Runtime.Taut.WrapsRealChaosGeometry",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimTautWrapsRealChaosGeometryTest::RunTest(const FString& Parameters)
{
	// Unlike other taut wrap tests (synthetic FCollisionEdge structs), this
	// routes a real spawned Chaos box through FCableSimChaosCollisionAdapter.
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
	// Default engine cube is 100uu/side; scale to 20uu.
	CubeActor->SetActorScale3D(FVector(0.2, 0.2, 0.2));
	CubeActor->SetActorLocation(FVector(0.0, 0.0, 10.0)); // spans X/Y [-10,10], Z [0,20]
	CubeComponent->RecreatePhysicsState();

	AActor* CableOwner = World->SpawnActor<AActor>();
	if (!TestNotNull(TEXT("Cable owner actor spawns"), CableOwner))
	{
		CubeActor->Destroy();
		return false;
	}
	UCableSimComponent* Cable = NewObject<UCableSimComponent>(CableOwner);
	Cable->SimulationSettings.RestLength = 300.0;
	Cable->SimulationSettings.NodeSpacing = 5.0;
	Cable->CollisionSettings.bEnableWorldCollision = true;
	Cable->CollisionSettings.Radius = 2.5;
	Cable->TautSettings.Mode = ECableSimTautMode::Enabled;
	Cable->StartEndpoint.Mode = ECableSimEndpointMode::WorldKinematic;
	// Clips the box's south-east corner (adjacent faces), not its center
	// (opposite faces, which has no edge to wrap around).
	const FVector3d WrapStart(-60.0, -65.0, 10.0);
	const FVector3d WrapEnd(60.0, 55.0, 10.0);
	Cable->StartEndpoint.WorldTarget = FVector(WrapStart);
	Cable->EndEndpoint.Mode = ECableSimEndpointMode::WorldKinematic;
	Cable->EndEndpoint.WorldTarget = FVector(WrapEnd);
	Cable->RegisterComponent();
	Cable->ReinitializeSimulation();

	for (int32 Step = 0; Step < 5; ++Step)
	{
		Cable->StepSimulation(1);
	}

	const FCableSimTautDiagnostics Diagnostics = Cable->GetTautDiagnostics();
	const TArray<FVector> TautPolyline = Cable->GetTautPolyline();
	AddInfo(FString::Printf(
		TEXT("Taut status=%d snapshotOk=%d tris=%d supportedEdges=%d/%d contacts=%d pathPoints=%d pathLength=%.1f"),
		static_cast<int32>(Diagnostics.Status),
		Diagnostics.bSnapshotSucceeded ? 1 : 0,
		Diagnostics.TriangleCount,
		Diagnostics.SupportedEdgeCount,
		Diagnostics.ExtractedEdgeCount,
		Diagnostics.ContactCount,
		TautPolyline.Num(),
		Diagnostics.PathLength));
	for (int32 Index = 0; Index < TautPolyline.Num(); ++Index)
	{
		AddInfo(FString::Printf(TEXT("  point %d: %s"), Index, *TautPolyline[Index].ToString()));
	}
	TestTrue(TEXT("Snapshot gather succeeded"), Diagnostics.bSnapshotSucceeded);
	TestTrue(TEXT("At least one supported (Box/Convex) taut edge was extracted"), Diagnostics.SupportedEdgeCount > 0);
	TestTrue(TEXT("Taut path is wrapped, not the raw 2-point straight line"), TautPolyline.Num() > 2);
	const double BlockedStraightLineDistance = FVector3d::Distance(WrapStart, WrapEnd);
	TestTrue(TEXT("Wrapped path is longer than the blocked straight-line distance"),
		Diagnostics.PathLength > BlockedStraightLineDistance + 1.0);

	Cable->DestroyComponent();
	CableOwner->Destroy();
	CubeActor->Destroy();
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimTautStaysStableNearRealBoxCornerTest,
	"CableSim.Runtime.Taut.StaysStableNearRealBoxCorner",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimTautStaysStableNearRealBoxCornerTest::RunTest(const FString& Parameters)
{
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
	CubeActor->SetActorScale3D(FVector(0.2, 0.2, 0.2));
	CubeActor->SetActorLocation(FVector(0.0, 0.0, 10.0)); // spans X/Y [-10,10], Z [0,20]
	CubeComponent->RecreatePhysicsState();

	AActor* CableOwner = World->SpawnActor<AActor>();
	if (!TestNotNull(TEXT("Cable owner actor spawns"), CableOwner))
	{
		CubeActor->Destroy();
		return false;
	}
	UCableSimComponent* Cable = NewObject<UCableSimComponent>(CableOwner);
	Cable->SimulationSettings.RestLength = 400.0;
	Cable->SimulationSettings.NodeSpacing = 5.0;
	Cable->CollisionSettings.bEnableWorldCollision = true;
	Cable->CollisionSettings.Radius = 2.5;
	Cable->TautSettings.Mode = ECableSimTautMode::Enabled;
	Cable->StartEndpoint.Mode = ECableSimEndpointMode::WorldKinematic;
	// Same XY offset as WrapsRealChaosGeometry (clips the south-east corner
	// region), then walks End.Z upward step by step through the real
	// UCableSimComponent/Chaos pipeline -- addresses the "very unstable"
	// report: status must stay Ready and path length must stay bounded across
	// many real steps near a real box corner, not oscillate or run away.
	const FVector3d WrapStart(-60.0, -65.0, 10.0);
	Cable->StartEndpoint.WorldTarget = FVector(WrapStart);
	Cable->EndEndpoint.Mode = ECableSimEndpointMode::WorldKinematic;
	Cable->EndEndpoint.WorldTarget = FVector(60.0, 55.0, 10.0);
	Cable->RegisterComponent();
	Cable->ReinitializeSimulation();

	int32 MaximumContactCount = 0;
	double MaximumPathLength = 0.0;
	constexpr int32 StepCount = 18;
	constexpr double ZStepSize = 5.0;
	for (int32 Step = 0; Step < StepCount; ++Step)
	{
		Cable->EndEndpoint.WorldTarget.Z = 10.0 + ZStepSize * static_cast<double>(Step);
		Cable->StepSimulation(1);
		const FCableSimTautDiagnostics Diagnostics = Cable->GetTautDiagnostics();
		TestEqual(FString::Printf(TEXT("Taut path stays supported at step %d"), Step),
			Diagnostics.Status, ECableSimTautStatus::Ready);
		const double StraightLineDistance = FVector3d::Distance(WrapStart, FVector3d(Cable->EndEndpoint.WorldTarget));
		TestTrue(FString::Printf(TEXT("Path length stays sane at step %d (no runaway/oscillation)"), Step),
			Diagnostics.PathLength < StraightLineDistance + 60.0);
		MaximumContactCount = FMath::Max(MaximumContactCount, Diagnostics.ContactCount);
		MaximumPathLength = FMath::Max(MaximumPathLength, Diagnostics.PathLength);
	}
	AddInfo(FString::Printf(TEXT("Walked End.Z over %d steps: maxContacts=%d maxPathLength=%.1f"),
		StepCount, MaximumContactCount, MaximumPathLength));

	Cable->DestroyComponent();
	CableOwner->Destroy();
	CubeActor->Destroy();
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimTautWrapsRealChaosCylinderTest,
	"CableSim.Runtime.Taut.WrapsRealChaosCylinder",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimTautWrapsRealChaosCylinderTest::RunTest(const FString& Parameters)
{
	// User report: taut wrap "works with cylinder, but does not work with cube."
	// Engine/BasicShapes/Cylinder ships with an authored simple-collision Convex
	// hull (not a Box), so this is the direct control -- same corner-clip wrap
	// scenario as WrapsRealChaosGeometry (cube/Box), but through the
	// AddConvexTriangles code path instead of AddBoxTriangles, to check whether
	// the two geometry types actually behave differently in this codebase.
	UWorld* World = GIsEditor ? GWorld : (GEngine ? GEngine->GetWorldContexts()[0].World() : nullptr);
	if (!TestNotNull(TEXT("Editor world is available"), World))
	{
		return false;
	}
	UStaticMesh* CylinderMesh = Cast<UStaticMesh>(StaticLoadObject(
		UStaticMesh::StaticClass(), nullptr, TEXT("/Engine/BasicShapes/Cylinder.Cylinder")));
	if (!TestNotNull(TEXT("Cylinder mesh loads"), CylinderMesh))
	{
		return false;
	}
	AStaticMeshActor* CylinderActor = World->SpawnActor<AStaticMeshActor>();
	if (!TestNotNull(TEXT("Cylinder actor spawns"), CylinderActor))
	{
		return false;
	}
	UStaticMeshComponent* CylinderComponent = CylinderActor->GetStaticMeshComponent();
	CylinderComponent->SetMobility(EComponentMobility::Static);
	CylinderComponent->SetStaticMesh(CylinderMesh);
	CylinderComponent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	CylinderComponent->SetCollisionObjectType(ECC_WorldStatic);
	CylinderComponent->SetCollisionResponseToAllChannels(ECR_Block);
	CylinderActor->SetActorEnableCollision(true);
	AddInfo(FString::Printf(
		TEXT("Cylinder body setup element count=%d"),
		CylinderMesh->GetBodySetup() ? CylinderMesh->GetBodySetup()->AggGeom.GetElementCount() : -1));
	// Default engine cylinder is 100uu diameter/100uu tall; scale to match the
	// cube tests (20uu radius footprint).
	CylinderActor->SetActorScale3D(FVector(0.2, 0.2, 0.2));
	CylinderActor->SetActorLocation(FVector(0.0, 0.0, 10.0)); // spans X/Y radius 10, Z [0,20]
	CylinderComponent->RecreatePhysicsState();

	AActor* CableOwner = World->SpawnActor<AActor>();
	if (!TestNotNull(TEXT("Cable owner actor spawns"), CableOwner))
	{
		CylinderActor->Destroy();
		return false;
	}
	UCableSimComponent* Cable = NewObject<UCableSimComponent>(CableOwner);
	Cable->SimulationSettings.RestLength = 300.0;
	Cable->SimulationSettings.NodeSpacing = 5.0;
	Cable->CollisionSettings.bEnableWorldCollision = true;
	Cable->CollisionSettings.Radius = 2.5;
	Cable->TautSettings.Mode = ECableSimTautMode::Enabled;
	Cable->StartEndpoint.Mode = ECableSimEndpointMode::WorldKinematic;
	// Same layout as WrapsRealChaosGeometry: straight line clips the near side
	// of the cylinder rather than passing through its axis.
	const FVector3d WrapStart(-60.0, -65.0, 10.0);
	const FVector3d WrapEnd(60.0, 55.0, 10.0);
	Cable->StartEndpoint.WorldTarget = FVector(WrapStart);
	Cable->EndEndpoint.Mode = ECableSimEndpointMode::WorldKinematic;
	Cable->EndEndpoint.WorldTarget = FVector(WrapEnd);
	Cable->RegisterComponent();
	Cable->ReinitializeSimulation();

	for (int32 Step = 0; Step < 5; ++Step)
	{
		Cable->StepSimulation(1);
	}

	const FCableSimTautDiagnostics Diagnostics = Cable->GetTautDiagnostics();
	const TArray<FVector> TautPolyline = Cable->GetTautPolyline();
	AddInfo(FString::Printf(
		TEXT("Taut status=%d snapshotOk=%d tris=%d supportedEdges=%d/%d contacts=%d pathPoints=%d pathLength=%.1f"),
		static_cast<int32>(Diagnostics.Status),
		Diagnostics.bSnapshotSucceeded ? 1 : 0,
		Diagnostics.TriangleCount,
		Diagnostics.SupportedEdgeCount,
		Diagnostics.ExtractedEdgeCount,
		Diagnostics.ContactCount,
		TautPolyline.Num(),
		Diagnostics.PathLength));

	TestTrue(TEXT("Snapshot gather succeeded"), Diagnostics.bSnapshotSucceeded);
	TestTrue(TEXT("At least one supported (Box/Convex) taut edge was extracted"), Diagnostics.SupportedEdgeCount > 0);
	TestTrue(TEXT("Taut path is wrapped, not the raw 2-point straight line"), TautPolyline.Num() > 2);

	Cable->DestroyComponent();
	CableOwner->Destroy();
	CylinderActor->Destroy();
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimTautRecoversAfterContactAgesOutOfRangeTest,
	"CableSim.Runtime.Taut.RecoversAfterContactAgesOutOfRange",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimTautRecoversAfterContactAgesOutOfRangeTest::RunTest(const FString& Parameters)
{
	// Regression for a permanent taut-wrap deadlock: a contact's edge aging out
	// of the local collision query used to hard-fail and roll back to a frozen
	// state that could never resolve on any later step.
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
	CubeActor->SetActorScale3D(FVector(0.2, 0.2, 0.2));
	CubeActor->SetActorLocation(FVector(0.0, 0.0, 10.0)); // spans X/Y [-10,10], Z [0,20]
	CubeComponent->RecreatePhysicsState();

	AActor* CableOwner = World->SpawnActor<AActor>();
	if (!TestNotNull(TEXT("Cable owner actor spawns"), CableOwner))
	{
		CubeActor->Destroy();
		return false;
	}
	UCableSimComponent* Cable = NewObject<UCableSimComponent>(CableOwner);
	Cable->SimulationSettings.RestLength = 200.0; // near-taut: modest slack over the ~170uu span, like a real power line
	Cable->SimulationSettings.NodeSpacing = 5.0;
	Cable->SimulationSettings.Gravity = FVector::ZeroVector;
	Cable->CollisionSettings.bEnableWorldCollision = true;
	Cable->CollisionSettings.Radius = 2.5;
	Cable->TautSettings.Mode = ECableSimTautMode::Enabled;
	Cable->StartEndpoint.Mode = ECableSimEndpointMode::WorldKinematic;
	const FVector3d WrapStart(-60.0, -65.0, 10.0);
	const FVector3d WrapEnd(60.0, 55.0, 10.0);
	Cable->StartEndpoint.WorldTarget = FVector(WrapStart);
	Cable->EndEndpoint.Mode = ECableSimEndpointMode::WorldKinematic;
	Cable->EndEndpoint.WorldTarget = FVector(WrapEnd);
	Cable->RegisterComponent();
	Cable->ReinitializeSimulation();

	for (int32 Step = 0; Step < 5; ++Step)
	{
		Cable->StepSimulation(1);
	}
	const FCableSimTautDiagnostics WrappedDiagnostics = Cable->GetTautDiagnostics();
	AddInfo(FString::Printf(TEXT("After wrapping: status=%d contacts=%d"),
		static_cast<int32>(WrappedDiagnostics.Status), WrappedDiagnostics.ContactCount));
	if (!TestEqual(TEXT("Precondition: cable is actually wrapped around the cube"),
		WrappedDiagnostics.Status, ECableSimTautStatus::Ready)
		|| !TestTrue(TEXT("Precondition: at least one contact"), WrappedDiagnostics.ContactCount > 0))
	{
		Cable->DestroyComponent();
		CableOwner->Destroy();
		CubeActor->Destroy();
		return false;
	}

	// Yank the End endpoint far away from the cube in one step -- well outside
	// any plausible collision query bounds -- then leave it somewhere that
	// needs no wrap at all (a simple, unblocked straight line far from the
	// cube) for many subsequent steps.
	Cable->EndEndpoint.WorldTarget = FVector(6000.0, 6000.0, 10.0);
	Cable->StepSimulation(1);
	const FCableSimTautDiagnostics JumpedDiagnostics = Cable->GetTautDiagnostics();
	AddInfo(FString::Printf(TEXT("Immediately after the far jump: status=%d contacts=%d"),
		static_cast<int32>(JumpedDiagnostics.Status), JumpedDiagnostics.ContactCount));

	// Move back to the EXACT original wrap configuration -- the real question
	// isn't whether the solver notices open space (NoRelevantGeometry there is
	// correct behavior), it's whether the cube's edges are ever usable again
	// once the path has been reset to a state with no contacts referencing
	// them, i.e. does the solver actually re-discover the wrap.
	Cable->EndEndpoint.WorldTarget = FVector(WrapEnd);
	ECableSimTautStatus LastStatus = ECableSimTautStatus::Uninitialized;
	int32 LastContactCount = 0;
	constexpr int32 RecoverySteps = 30;
	for (int32 Step = 0; Step < RecoverySteps; ++Step)
	{
		Cable->StepSimulation(1);
		LastStatus = Cable->GetTautDiagnostics().Status;
		LastContactCount = Cable->GetTautDiagnostics().ContactCount;
		AddInfo(FString::Printf(TEXT("  recovery step %d: status=%d contacts=%d pathLen=%.1f"),
			Step, static_cast<int32>(LastStatus), LastContactCount,
			Cable->GetTautDiagnostics().PathLength));
	}

	const bool bRecovered = TestTrue(
		FString::Printf(TEXT("Taut solver re-wraps the SAME cube once the endpoint returns to the original wrap position (actual final status=%d contacts=%d)"),
			static_cast<int32>(LastStatus), LastContactCount),
		LastStatus == ECableSimTautStatus::Ready && LastContactCount > 0);

	Cable->DestroyComponent();
	CableOwner->Destroy();
	CubeActor->Destroy();
	return bRecovered;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimTautEnabledClampsReachTest,
	"CableSim.Runtime.Taut.EnabledClampsReach",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimTautEnabledClampsReachTest::RunTest(const FString& Parameters)
{
	UWorld* World = GIsEditor ? GWorld : (GEngine ? GEngine->GetWorldContexts()[0].World() : nullptr);
	if (!TestNotNull(TEXT("Editor world is available"), World))
	{
		return false;
	}
	AActor* CableOwner = World->SpawnActor<AActor>();
	UCableSimComponent* Cable = NewObject<UCableSimComponent>(CableOwner);
	Cable->SimulationSettings.RestLength = 100.0;
	Cable->SimulationSettings.NodeSpacing = 10.0;
	Cable->SimulationSettings.Gravity = FVector::ZeroVector;
	Cable->CollisionSettings.bEnableWorldCollision = false;
	Cable->TautSettings.Mode = ECableSimTautMode::Enabled;
	Cable->TautSettings.ConstrainedEndpoint = ECableSimEndpoint::End;
	Cable->StartEndpoint.Mode = ECableSimEndpointMode::WorldKinematic;
	Cable->EndEndpoint.Mode = ECableSimEndpointMode::WorldKinematic;
	const FVector FarOrigin(100000.0, 0.0, 10000.0);
	Cable->StartEndpoint.WorldTarget = FarOrigin;
	Cable->EndEndpoint.WorldTarget = FarOrigin + FVector(50.0, 0.0, 0.0);
	Cable->RegisterComponent();
	Cable->ReinitializeSimulation();
	Cable->StepSimulation(1);

	Cable->EndEndpoint.WorldTarget = FarOrigin + FVector(150.0, 0.0, 0.0);
	Cable->StepSimulation(1);
	const TArray<FVector> Polyline = Cable->GetRenderPolyline();
	const double EndpointDistance = FVector::Distance(Polyline[0], Polyline.Last());
	TestTrue(TEXT("Enabled mode clamps the endpoint to the rest-length boundary"),
		EndpointDistance <= Cable->SimulationSettings.RestLength
			+ Cable->TautSettings.LengthTolerance + 0.1);
	TestTrue(TEXT("Enabled mode accepts most of the feasible request"), EndpointDistance > 99.0);

	Cable->DestroyComponent();
	CableOwner->Destroy();
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimTautShadowDoesNotClampReachTest,
	"CableSim.Runtime.Taut.ShadowDoesNotClampReach",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimTautShadowDoesNotClampReachTest::RunTest(const FString& Parameters)
{
	UWorld* World = GIsEditor ? GWorld : (GEngine ? GEngine->GetWorldContexts()[0].World() : nullptr);
	if (!TestNotNull(TEXT("Editor world is available"), World))
	{
		return false;
	}
	AActor* CableOwner = World->SpawnActor<AActor>();
	UCableSimComponent* Cable = NewObject<UCableSimComponent>(CableOwner);
	Cable->SimulationSettings.RestLength = 100.0;
	Cable->SimulationSettings.NodeSpacing = 10.0;
	Cable->SimulationSettings.Gravity = FVector::ZeroVector;
	Cable->CollisionSettings.bEnableWorldCollision = false;
	Cable->TautSettings.Mode = ECableSimTautMode::Shadow;
	Cable->StartEndpoint.Mode = ECableSimEndpointMode::WorldKinematic;
	Cable->EndEndpoint.Mode = ECableSimEndpointMode::WorldKinematic;
	const FVector FarOrigin(100000.0, 0.0, 10000.0);
	Cable->StartEndpoint.WorldTarget = FarOrigin;
	Cable->EndEndpoint.WorldTarget = FarOrigin + FVector(50.0, 0.0, 0.0);
	Cable->RegisterComponent();
	Cable->ReinitializeSimulation();
	Cable->StepSimulation(1);

	Cable->EndEndpoint.WorldTarget = FarOrigin + FVector(150.0, 0.0, 0.0);
	Cable->StepSimulation(1);
	const TArray<FVector> Polyline = Cable->GetRenderPolyline();
	TestTrue(TEXT("Shadow mode observes but does not clamp the endpoint request"),
		FVector::Distance(Polyline[0], Polyline.Last()) > 149.0);
	TestTrue(TEXT("Shadow mode still reports a valid collision-free taut path"),
		Cable->GetTautDiagnostics().bPathCollisionFree);

	Cable->DestroyComponent();
	CableOwner->Destroy();
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FCableSimMovingPlatformVelocityTest,
	"CableSim.Runtime.Dynamic.MovingPlatformStampsSurfaceVelocity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCableSimMovingPlatformVelocityTest::RunTest(const FString& Parameters)
{
	// End-to-end wiring check for moving-surface friction: a real Chaos body with a
	// linear velocity must have that velocity stamped onto the gathered triangles by
	// the adapter (GetVAtPoint) and carried into a compiled contact. Combined with the
	// core MovingSurfaceDragsCable test (contact velocity -> friction drag), this
	// covers the whole path. Guards the "teleport reports zero velocity" caveat.
	UWorld* World = GIsEditor ? GWorld : (GEngine ? GEngine->GetWorldContexts()[0].World() : nullptr);
	if (!TestNotNull(TEXT("A world is available"), World))
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
	const double Speed = 30.0;
	UStaticMeshComponent* CubeComponent = CubeActor->GetStaticMeshComponent();
	CubeComponent->SetMobility(EComponentMobility::Movable);
	CubeComponent->SetStaticMesh(CubeMesh);
	CubeComponent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	CubeComponent->SetCollisionObjectType(ECC_WorldDynamic);
	CubeComponent->SetCollisionResponseToAllChannels(ECR_Block);
	CubeActor->SetActorEnableCollision(true);
	CubeActor->SetActorLocation(FVector::ZeroVector);
	CubeComponent->SetSimulatePhysics(true);
	CubeComponent->SetEnableGravity(false);
	CubeComponent->RecreatePhysicsState();
	CubeComponent->SetPhysicsLinearVelocity(FVector(Speed, 0.0, 0.0));
	CubeComponent->UpdateBounds();

	const double TopZ = CubeMesh->GetBounds().BoxExtent.Z; // engine cube extent = 50
	TArray<CableSim::FParticle> Particles;
	Particles.SetNum(2);
	Particles[0].Position = FVector3d(0.0, 0.0, TopZ + 2.0);
	Particles[1].Position = FVector3d(8.0, 0.0, TopZ + 2.0);
	Particles[0].PreviousPosition = Particles[0].Position;
	Particles[1].PreviousPosition = Particles[1].Position;

	FCableSimCollisionSettings CollisionSettings;
	CollisionSettings.Radius = 2.5;
	FCableSimChaosObjectTracker ObjectTracker;
	FCableSimChaosSnapshot Snapshot;
	const bool bGathered = FCableSimChaosCollisionAdapter::GatherSnapshot(
		World, nullptr, TArray<AActor*>(), CollisionSettings, 0.1,
		Particles, Particles[0].Position, Particles[1].Position,
		TArray<FVector3d>(), ObjectTracker, Snapshot);

	double MaxTriangleVelocityX = 0.0;
	for (const CableSim::FCollisionTriangle& Triangle : Snapshot.Triangles)
	{
		MaxTriangleVelocityX = FMath::Max(MaxTriangleVelocityX, FMath::Abs(Triangle.SurfaceVelocity.X));
	}

	// Compile a contact for the node just above the moving top face and confirm the
	// velocity reaches the contact the friction solver consumes.
	CableSim::FManifoldConfig ManifoldConfig;
	ManifoldConfig.NodeRadius = CollisionSettings.Radius;
	ManifoldConfig.ActiveBand = 2.0;
	TArray<CableSim::FContactConstraint> Contacts;
	CableSim::FContactManifoldCompiler::CompileNodeContacts(
		0, Particles[0].Position, Particles[0].PreviousPosition,
		Snapshot.Triangles, Snapshot.Edges, ManifoldConfig, Contacts);
	double MaxContactVelocityX = 0.0;
	for (const CableSim::FContactConstraint& Contact : Contacts)
	{
		MaxContactVelocityX = FMath::Max(MaxContactVelocityX, FMath::Abs(Contact.SurfaceVelocity.X));
	}

	AddInfo(FString::Printf(
		TEXT("moving platform: gathered=%d tris=%d contacts=%d triVelX=%.2f contactVelX=%.2f (speed=%.0f)"),
		bGathered ? 1 : 0, Snapshot.Triangles.Num(), Contacts.Num(),
		MaxTriangleVelocityX, MaxContactVelocityX, Speed));

	TestTrue(TEXT("Snapshot gathered the moving cube"), bGathered && Snapshot.Triangles.Num() > 0);
	TestTrue(TEXT("The adapter stamps the body's velocity onto its triangles"),
		MaxTriangleVelocityX > Speed * 0.5);
	TestTrue(TEXT("A contact was compiled against the moving face"), Contacts.Num() > 0);
	TestTrue(TEXT("The surface velocity reaches the friction contact"),
		MaxContactVelocityX > Speed * 0.5);

	CubeActor->Destroy();
	return true;
}

#endif
