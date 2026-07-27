# Chaos-backed taut cable feasibility

Status: conditional go for a focused prototype  
Last reviewed: 2026-07-26  
Engine reviewed: Unreal Engine 5.8 source at `/Users/Shared/Epic Games/UE_5.8`

This note answers a narrow question: can CableSim reproduce the topology-aware taut-rope technique from Naughty Dog's GDC talk while using ordinary Chaos collision in a large streamed world, without manually baking separate rope collision?

## Decision

Yes, with one important distinction:

- Chaos should be our collision geometry source and broadphase.
- CableSim must still own the taut-path algorithm. Stock Chaos queries or constraints do not provide Naughty Dog's wrapping topology.

UE 5.8 exposes enough public runtime data to prototype this cleanly. An overlap result carries a Chaos physics-object handle. With a scoped Chaos scene read lock, Runtime code can enumerate that object's shapes, transforms, velocities, filters, and cooked implicit geometry. Public Chaos geometry types expose the topology needed for boxes, convexes, triangle meshes, and heightfields.

The recommended decision is therefore **go**, but only to the validation spike described below. We should not commit to production support for every Chaos shape until that spike proves wrapper transforms, identity stability, streaming lifetime, and cost on representative content.

## What changed from the earlier plan

The earlier research recommended a manually baked `URopeCollisionData` asset because high-level traces return contact results rather than mesh topology. That conclusion was too conservative for UE 5.8.

Low-level, public Chaos APIs let us read the already-cooked collision shape at runtime. More importantly, triangle meshes and heightfields provide bounded triangle visitation using their existing spatial acceleration. We can gather a small local triangle island around the cable rather than scan or copy a whole world mesh.

Consequences:

- No mandatory editor bake.
- No duplicate collision database for the open world.
- No whole-world startup preprocessing.
- No whole-mesh adjacency cache in the first implementation.
- World Partition remains responsible for loading nearby collision.
- The Chaos-specific code stays isolated in `CableSimRuntime`; `CableSimCore` remains Chaos-free.

Manual authoring can remain an optional optimization or repair tool later, not a foundation.

## Evidence from UE 5.8

The following paths are public engine headers unless stated otherwise.

### Getting from a world query to locked shapes

`FOverlapResult` contains:

- `PhysicsObject`
- `PhysicsObjectOwner`
- `Component`
- `ItemIndex`

Source: `Engine/Source/Runtime/Engine/Classes/Engine/OverlapResult.h`

`FPhysicsObjectExternalInterface` provides scoped read locks for a scene, one handle, or an array of handles:

```cpp
FPhysicsObjectExternalInterface::LockRead(Handles)
```

Source: `Engine/Source/Runtime/Engine/Classes/PhysicsEngine/PhysicsObjectExternalInterface.h`

The resulting public `Chaos::FReadPhysicsObjectInterface_External` exposes:

- object transform
- linear and angular velocity, including velocity at a point
- all thread shapes / a shape visitor
- particle geometry and world bounds
- validity, kinematic, dynamic, and sleeping queries

Source: `Engine/Source/Runtime/Experimental/Chaos/Public/Chaos/PhysicsObjectInterface.h`

Epic uses a similar overlap-to-physics-shape extraction path for cloth environment collision in:

`Engine/Source/Runtime/Engine/Private/PhysicsEngine/EnvironmentalCollisions.cpp`

That is useful precedent, although CableSim should use the newer physics-object interface and must not include Engine private headers.

### Geometry topology that is available

`Chaos::FConvex` exposes vertices, planes, face vertex loops, edges, and edge-to-plane relationships. It is almost a direct taut-feature source.

Source: `Engine/Source/Runtime/Experimental/Chaos/Public/Chaos/Convex.h` and Epic's [`FConvex::NumEdges`](https://dev.epicgames.com/documentation/en-us/unreal-engine/API/Runtime/Chaos/Chaos/FConvex/NumEdges) API reference.

`Chaos::FTriangleMeshImplicitObject` exposes:

- cooked particles and index buffer
- face normals
- stable internal triangle and vertex indices
- `FindOverlappingTriangles`
- `VisitTriangles`

`VisitTriangles` uses the mesh's cooked `FastBVH`, so gathering can be local to a cable query box.

Source: `Engine/Source/Runtime/Experimental/Chaos/Public/Chaos/TriangleMeshImplicitObject.h`

`Chaos::FHeightField` exposes grid points, holes, rows/columns, local triangles, and bounded `VisitTriangles`. The visitor guarantees standard winding after scale handling. See Epic's [`FHeightField::VisitTriangles`](https://dev.epicgames.com/documentation/unreal-engine/API/Runtime/Chaos/FHeightField/VisitTriangles) reference.

`Chaos::FShapeInstance` exposes query enablement, shape filters, shape index, world bounds, geometry, leaf geometry, and leaf-relative transform.

Source: `Engine/Source/Runtime/Experimental/Chaos/Public/Chaos/ShapeInstance.h`

### Important API constraints

- `Chaos::FImplicitObjectRef` is a raw pointer. It is not an owning reference.
- No Chaos geometry or shape pointer may escape the scoped read lock in solver-facing state.
- Transformed, instanced, and scaled wrappers must be handled explicitly. The generic `CastHelper` does not cover every triangle-mesh and heightfield combination we need.
- `FPhysicsObjectInterface::GetId` is a body index, not a globally unique object identifier.
- `FOverlapResult` is commonly deduplicated at object/component level, so the adapter must inspect all shapes on each returned physics object and apply bounds/filter tests itself.
- The required headers are public, but several live under `Experimental/Chaos`. The adapter is therefore version-coupled and must be kept behind one Runtime boundary.

## Proposed runtime data path

```text
Cable current/previous path and endpoint targets
                    |
                    v
       swept segment bounds / cable bounds
                    |
                    v
       UWorld overlap using cable channel
                    |
                    v
   deduplicate valid FPhysicsObject handles
                    |
                    v
       one scoped Chaos read-lock batch
                    |
       +------------+-------------+
       |                          |
 shape/filter/transform       bounded local
       snapshot             triangle visitation
       |                          |
       +------------+-------------+
                    |
          copy plain CableSim data
                    |
             release Chaos lock
                    |
                    v
     local adjacency + convex-edge compiler
                    |
                    v
        immutable collision scene snapshot
                    |
          +---------+----------+
          |                    |
  dynamic contact work    taut path solver
       (later)                 (Core)
```

The solver must not call `UWorld`, hold a physics lock, retain a `UObject`, or retain a Chaos pointer during its movement/collision iterations.

### Runtime module boundary

Add private `Chaos` and `PhysicsCore` dependencies to `CableSimRuntime` only. Do not add them to `CableSimCore`.

The first implementation should be a concrete private adapter, not a large provider hierarchy:

```text
CableSimRuntime/Private/Chaos/
  CableSimChaosCollisionAdapter.*
  CableSimChaosGeometrySnapshot.*
```

Introduce an abstract collision provider only when a second real backend exists.

### Solver-facing snapshot

The exact layout should follow the spike, but the ownership model should look like this:

```cpp
struct FCollisionFeatureId
{
    uint64 ObjectToken;
    int32 ShapeIndex;
    EFeatureType Type;
    int32 Index0;
    int32 Index1;
};

struct FCollisionEdge
{
    FCollisionFeatureId Id;
    FVector3d Start;
    FVector3d End;
    FVector3d FaceNormal0;
    FVector3d FaceNormal1;
    bool bConvex;
};

struct FCollisionSceneSnapshot
{
    uint64 StepIndex;
    TArray<FCollisionEdge> Edges;
    TArray<FCollisionTriangle> Triangles;
    TArray<FCollisionObjectMotion> ObjectMotions;
};
```

This is illustrative, not an API commitment. The important property is that every field owns plain data.

## Local topology instead of a global bake or cache

For a triangle mesh, call `VisitTriangles` with the cable's swept query bounds transformed conservatively into the unscaled local mesh space. Copy the returned local triangles and their internal vertex/triangle indices, then release the Chaos lock.

After release:

1. Key every triangle side by its sorted pair of Chaos vertex indices.
2. Pair sides that share the same key.
3. Use the two face normals and opposite vertices to classify the edge as convex, concave, or effectively coplanar.
4. Discard concave and coplanar edges from the taut candidate set.
5. Keep triangle faces because they are useful for validation, debugging, and future dynamic-contact compilation.

If a shared edge is inside the expanded query bounds, both incident triangle AABBs should touch that bounds. The validation spike must explicitly verify this assumption against Chaos's BVH boundary behavior. Expanding by the global topology tolerance gives a conservative margin.

This makes topology construction proportional to nearby triangles, not total mesh or world size. It also avoids a first-version cache-lifetime problem.

Potential caching comes later:

- Cache immutable compiled local islands only after profiling shows a need.
- Never key solely by `GetTypeHash()` or `FPhysicsObjectInterface::GetId()`.
- If an owning `Chaos::FConstImplicitObjectPtr` is retained in a bounded Runtime-only LRU, validate its lifetime behavior in the spike first.
- Otherwise use an owner generation, live geometry address, shape index, and geometry signature, and rebuild safely after streaming/recreation.

## Shape support policy

| Chaos shape | Taut support | Initial treatment |
|---|---|---|
| Box | Excellent | Generate canonical vertices, faces, and 12 convex edges. |
| Convex | Excellent | Copy public face/edge topology; preserve scale and winding. |
| Triangle mesh | Good, conditional | Bounded triangle visit, transient local adjacency, manifold validation. |
| Heightfield / Landscape | Technically possible, higher risk | Bounded triangle visit; defer production support until contact-count and seam tests pass. |
| Sphere | No natural straight edge topology | Deterministically tessellate to an invisible convex proxy, or report unsupported. |
| Capsule | No natural straight edge topology | Deterministically tessellate to an invisible convex proxy, or report unsupported. |
| Infinite plane | No wrapping edge | Dynamic contact only; taut path may reject a crossing. |
| Level set / SDF | No exposed polygon topology | Unsupported initially; use fail-safe query fallback. |
| Geometry Collection / changing union | Lifetime and topology can change | Explicit later milestone. Do not silently retain features. |

For a sphere or capsule, a true shortest path contains curved tangency arcs. That is a different solver. A deterministic runtime convex approximation is acceptable for an invisible gameplay authority if its canonical feature IDs remain stable.

### Triangle mesh validity rules

Initial support should prefer closed, consistently wound, two-manifold collision. For local edges:

- two incident faces: classify normally
- one incident face: boundary edge; mark one-sided/unsupported initially
- more than two incident faces: non-manifold; record diagnostics and use a safe fallback
- degenerate face or zero-length edge: discard with a diagnostic counter
- duplicate/copied vertices within separate shapes: reconcile only at taut intersection gathering using the global tolerance

Do not randomly drop features when a budget is exceeded. Return a visible diagnostic/fallback state.

## Taut representation and finite cable radius

The first taut solver should remain a zero-radius, invisible path-length authority, matching its role in *Uncharted 4*. The existing dynamic rope remains the rendered finite-radius cable and continues handling visible contacts.

Making the taut path itself respect cable radius is substantially harder: offset polygon edges become cylindrical arcs and offset vertices become spherical patches. That should be a separate milestone, not smuggled into the first topology implementation.

For gameplay:

```text
available slack = physical cable length - taut path length
```

When available slack approaches zero, endpoint motion is limited by the collision-free taut path. The dynamic cable is later constrained toward a corridor around that path.

## Shape filters and candidate selection

The broad overlap should use the same CableSim collision channel and ignore list as the current dynamic adapter. Under the read lock, every shape must also pass:

- `GetQueryEnabled()`
- world-bounds intersection with at least one cable swept bound
- simple/complex flags compatible with the query
- the public `FQueryFilterData::NarrowFilter` result against the shape's filter data

Build the query filter with public Engine filtering utilities. Do not depend on the private `FCollisionQueryFilterCallback` header.

Start with one conservative cable-wide AABB. If diagonal or very long cables gather too much unrelated geometry, replace it with a small set of merged per-segment swept AABBs. This is a performance refinement, not an architectural change.

## Identity, lifetime, and streaming

The adapter assigns its own runtime `ObjectToken`. A practical live key combines:

- weak `PhysicsObjectOwner` identity (`FObjectKey` where available)
- the currently observed physics-object handle value
- the shape index

The raw handle value is only an ephemeral identity token; it is never dereferenced outside a lock. Feature IDs append canonical local vertex, triangle, or edge indices.

When the weak owner disappears, a physics object fails validation, or a feature cannot be resolved in the new snapshot, its taut contact becomes invalid and enters a defined recovery path.

[World Partition](https://dev.epicgames.com/documentation/en-us/unreal-engine/world-partition-in-unreal-engine) streams cells around sources, so ordinary Chaos queries naturally see loaded nearby collision rather than the whole map. CableSim should not maintain a world-wide collision cache.

Open-world policy still needs one explicit rule: if a cable can span beyond the world's normal loading range, either its endpoints become streaming sources, or the cable freezes/deactivates when required collision unloads. A solver cannot collide against an unloaded world.

## Moving collision

Static world geometry is the first taut target.

Current transform, linear velocity, angular velocity, and velocity at a point are publicly available. Previous shape transforms are not exposed through the physics-object read interface in the same direct way, so Runtime should track previous transforms by object token.

Full continuous collision against a rotating edge is not part of the first spike. Planned progression:

1. Static shapes.
2. Kinematic translation using relative swept motion.
3. Rotation with bounded angular substeps or a conservative swept-edge test.
4. Dynamic topology-changing objects only after explicit invalidation support.

Teleportation must invalidate previous motion and rebuild/recover rather than create a giant false sweep.

## Mapping Chaos geometry to the Naughty Dog algorithm

The GDC taut solver stores a collision-free polyline whose internal points lie on oriented convex edges. Each frame it alternates:

1. Movement phase: endpoints move toward their targets and edge contacts slide to shorten the polyline.
2. Collision phase: move one rope point at a time; each adjacent line segment sweeps a triangle. Intersections with nearby convex edges insert or split contacts.
3. Repeat until a collision phase adds no contact, subject to a hard iteration/event budget.

At a vertex, all incident edges within one global tolerance are gathered. The talk then classifies edge pairs as stable inner corner, unstable corner, side edge, outer corner, or movement along one edge. This topology is why high-level hit results alone are insufficient.

The slide at 47:45 specifies one consistent tolerance of `0.001 m`, which is `0.1 cm` in Unreal units. Use that as the initial topology tolerance, but keep cable contact skin/radius separate.

The talk also says the production taut solver was never fully robust for arbitrary geometry. Our target should therefore be a bounded, diagnosable contract rather than an impossible “all Chaos geometry” promise. Source: [GDC Vault session](https://gdcvault.com/play/1027351/Rope-Simulation-in-Uncharted-4).

## Failure and fallback contract

Every snapshot/taut step returns a reason, not only a boolean:

- `Complete`
- `NoRelevantGeometry`
- `UnsupportedShape`
- `NonManifoldTopology`
- `FeatureBudgetExceeded`
- `FeatureInvalidated`
- `IterationBudgetExceeded`
- `PhysicsObjectUnavailable`

Safe fallback order:

1. Preserve and revalidate the last collision-free taut path.
2. Attempt a bounded high-level query waypoint recovery for unsupported local geometry.
3. If recovery cannot prove a safe path, clamp endpoint progress for that step and expose the diagnostic.

Never silently replace a failed obstacle-aware path with the straight endpoint distance; that could grant extra cable length through a wall.

## What to build next: validation spike

This spike is deliberately not the taut solver. It proves that Chaos can supply reliable input.

### Spike A — read-only geometry snapshot

Create a Runtime-private experimental adapter and debug view that:

1. Overlaps a box around the current cable.
2. Locks all unique physics-object handles once.
3. Enumerates query-compatible shapes.
4. Copies local features into plain snapshot structs.
5. Releases the lock.
6. Builds/draws convex edges and reports feature IDs.

Test fixtures in `L_Playground`:

- unscaled and non-uniformly scaled box
- rotated convex collision
- static mesh using simple collision
- static mesh using complex-as-simple triangle collision
- ISM/HISM instances with different transforms
- Landscape component
- moving kinematic box
- streamed/unloaded actor if World Partition is enabled

### Spike A acceptance gates

- Uses only public headers and builds with `Chaos`/`PhysicsCore` private Runtime dependencies.
- No Chaos pointer survives the scoped read lock.
- Box, convex, triangle-mesh, and heightfield transforms/winding debug correctly, including negative scale.
- Triangle and vertex IDs remain stable while an unchanged object moves.
- Unload/reload and component recreation invalidate safely.
- Local mesh gathering does not scale with total world triangle count in the test case.
- Counters expose overlap count, object count, shape count, local triangles, compiled edges, lock time, compile time, and discarded features.

### Spike B — one-edge taut core

Only after Spike A passes:

- implement a Core-only synthetic oriented-edge scene
- endpoint swept-triangle collision
- one contact insertion
- one-edge sliding/unfolding
- contact removal when the direct segment becomes valid
- bounded movement/collision loop
- replay/golden tests

Then connect that solver to only Box/Convex snapshots. Triangle meshes and Landscapes should not be on the critical path for proving the taut algorithm.

### Production sequence after the spikes

1. Box and convex static taut wrapping.
2. Local triangle-mesh feature islands.
3. Multiple contacts on distinct edges.
4. Vertex gathering and pairwise topology cases.
5. Taut path length as endpoint gameplay authority.
6. Dynamic-to-taut safe-line coupling.
7. Heightfield seam/contact-budget work.
8. Moving collision.
9. Optional caches only after profiling.

## Go/no-go criteria

Proceed with the Chaos-backed approach if:

- Spike A builds without private Engine dependencies.
- local features are transformed and identified consistently across the required fixtures.
- streaming invalidation is safe.
- the local triangle counts and read-lock duration are acceptable on representative collision.
- Spike B wraps and unwraps around one box edge without tunnelling under the declared endpoint-motion limit.

Pivot to a query-only taut approximation or optional authored collision only if one of those concrete gates fails. Do not add a manual baking pipeline pre-emptively.

## Current recommendation for CableSim

Keep the dynamic system we have and do not refactor it around Chaos topology yet. Build Spike A beside `FCableSimWorldCollisionAdapter`, with no gameplay effect. Once its evidence is good, build the small Core taut solver against synthetic edges, then join them.

This is the lowest-risk route because it independently validates the two uncertain parts:

- Can UE/Chaos give us reliable local topology?
- Can our taut movement/collision loop maintain a correct path?

If both pass, the architecture is sound and still requires no manual environment bake.
