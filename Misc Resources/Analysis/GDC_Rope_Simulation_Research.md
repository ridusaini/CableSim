# Rope Simulation in *Uncharted 4* and *The Last of Us Part II*

Research notes for the CableSim Unreal Engine 5.8 project.

> Update, 2026-07-26: the original manual collision-bake recommendation in this document has been superseded by the UE 5.8 source audit in [`Chaos_Taut_Collision_Feasibility.md`](Chaos_Taut_Collision_Feasibility.md). Public Chaos APIs can provide bounded local triangle/heightfield geometry and convex topology at runtime. Treat the older bake-oriented roadmap below as historical context.

Source talk: Jaroslav Sinecky, Naughty Dog, GDC 2021, [“Rope Simulation in *Uncharted 4* and *The Last of Us 2*”](https://gdcvault.com/play/1027351/Rope-Simulation-in-Uncharted-4).

## Executive conclusion

The Naughty Dog system is not one rope solver. It is a hybrid of two mostly independent systems:

1. **Dynamic rope simulation**: a regularly discretized Position Based Dynamics (PBD) chain used for slack shape, throwing, coils, straps, reins, and visual presentation.
2. **Taut rope simulation**: a geometric, topology-aware shortest-path solver whose points lie on oriented convex collision edges. It is responsible for continuous wrapping, unwrapping, and gameplay-safe length limits.

For *Uncharted 4*, the taut path was the gameplay authority and a “safe line” for the less robust dynamic rope. For *The Last of Us Part II*, the dynamic solver was made much more robust because a thrown rope could exist without a safe line. The taut solver was still created whenever Ellie held the rope and was seeded from the dynamic rope’s current shape.

The best first product target for CableSim is therefore an **Uncharted-style hybrid grapple/winch**, not a completely free, throwable 14 m rope. It reaches useful gameplay sooner and preserves a robust fallback while the dynamic collision system matures.

CableSim now has separate `CableSimCore` and `CableSimRuntime` plugin modules, a dynamic PBD solver, runtime length remeshing, endpoint bindings, replay data, and a high-level world-sweep collision adapter. The later feasibility note above records the current architecture and next collision experiment.

## Analysis artifacts

- `rope_simulation_complete.vtt`: complete, deduplicated English transcript.
- `Frames/Every15Seconds`: 228 timestamped 1280×720 frames.
- `Frames/ContactSheets`: 15 contact sheets covering the whole talk.
- `Frames/PinchingDetail`: one-second frames around the one gap in the supplied subtitles.
- `ExtractVideoFrames.swift`: reusable local frame extractor with optional start/end times.
- `BuildContactSheets.swift`: reusable contact-sheet builder.

The source video is 56:55 long. The technical sections begin at approximately 09:54 (dynamic rope) and 31:35 (taut rope).

## Confidence and disclosure boundaries

The talk gives the architecture, important constraints, several equations, and the core taut-rope process. It does **not** give production source code, complete collision predicates, the full topological decision table, moving-collider handling, all constraint ordering, or the exact coupling constraints between dynamic nodes and the taut path.

This report labels reconstruction as follows:

- **Talk**: explicitly stated or visible on a slide.
- **Derived**: follows mathematically from disclosed material.
- **Recommendation**: proposed CableSim design, not claimed to be Naughty Dog’s exact implementation.

One subtitle error is worth recording: the slide at 47:45 says the taut solver uses a tolerance of `0.001 m` (1 mm). The automatic subtitle says “0.001 millimeter”. The slide is authoritative.

## Requirements revealed by the talk

The production requirements were much broader than “simulate a hanging line”:

- Change total rope length at runtime.
- Change discretization dynamically.
- Keyframe any number of arbitrary internal points on any frame.
- Allow dynamic and keyframed nodes to coexist, including hand-held coils.
- Support a maximum 14 m dynamic rope; the winch prototype used 16 m.
- Allow slack to exist independently in several rope intervals.
- Wrap and slide around collision edges without tunnelling when endpoints move quickly.
- Let gameplay depend on a collision-free taut path and its current path length.
- Throw a rope, leave it in the world, and later pick it up at an arbitrary point.
- Support character collision and moving collision.
- Provide rope-only collision authoring and per-triangle inclusion/exclusion.
- Remain replayable and inspectable in a dedicated frame debugger.

This requirement list should become explicit feature tiers in CableSim. A weapon strap and a throwable puzzle rope should not silently share the same cost or robustness contract.

## System architecture

```text
Gameplay endpoints / animation controls / requested rope length
                              |
              +---------------+---------------+
              |                               |
              v                               v
    Dynamic PBD rope state           Taut path topology state
 particles + constraints +         endpoints + edge contacts +
 local collision constraints        intersection classifications
              |                               |
              +---------------+---------------+
                              |
                   hybrid coupling / policy
              safe-line cage, pickup seeding,
                allowable endpoint distance
                              |
                              v
                  resampled render centreline
                              |
                    tube/spline mesh renderer
```

The two simulations should not be collapsed into a single class or state array. They have different discretizations, different collision representations, and different failure modes.

## Dynamic rope simulation

### State and time step

**Talk:** The rope is a sequence of nodes. A node has position, velocity, mass, and a dynamic or keyframed state. Neighbouring nodes have fixed rest distances.

The disclosed PBD step is:

1. Apply damping to velocities.
2. Apply external acceleration/forces, such as gravity.
3. Predict new positions using the velocities and time step.
4. Iteratively project distance, collision, bending, friction, and other constraints.
5. Reconstruct velocities from corrected and previous positions:

   `v = (p_corrected - p_previous) / dt`

6. Apply any post-solver velocity effects.

**Talk:** Use a fixed iteration count. Stopping on an error threshold changes the effective PBD stiffness from frame to frame.

**Recommendation:** Use a fixed simulation step with an accumulator. Store explicit velocity during the first implementation because the talk’s friction and prediction logic is expressed in velocity terms. Verlet storage can be added as an optimized representation later.

Suggested node state:

```cpp
struct FDynamicRopeNode
{
    FVector3d Position;
    FVector3d PreviousPosition;
    FVector3d PredictedPosition;
    FVector3d Velocity;
    double InverseMass;
    double MaterialCoordinate; // distance along the physical rope
    ERopeNodeMode Mode;        // Dynamic, Keyframed, FreeEnd, etc.
    FRopeCollisionConstraintSet Collision;
};
```

`MaterialCoordinate` is important because physical rope length, internal attachments, and render sampling must survive insertion/removal of simulation nodes.

### Distance constraint

For neighbouring nodes `i` and `j`, current distance `l`, target distance `l0`, and masses `mi`, `mj`, the talk’s scalar displacement magnitudes are:

```text
Δdi = mj / (mi + mj) * (l - l0)
Δdj = (l - l0) - Δdi
```

The corrections act along the normalized separation vector, with the correct sign for each node. Infinite-mass/keyframed nodes receive no correction.

### Bending stiffness

**Talk:** The constraint operates on three consecutive nodes and moves all three along one shared correction direction. Its construction conserves linear and angular momentum and avoids trigonometric functions in the ordinary solve.

Using the notation on the slides:

```text
d1 = (s2 / m1) h
d2 = (s  / m2) h
d3 = (s1 / m3) h

                 e s
h = -----------------------------
    s2²/m1 + s²/m2 + s1²/m3
```

The final correction multiplies `h` by bending stiffness `ζ`.

The solver also allows a free bend before stiffness acts. For free bending angle per metre `αf` and link length `l0`:

```text
ef = l0 cos((π - αf l0) / 2)

             max(e - ef, 0) s
h = ζ --------------------------------
        s2²/m1 + s²/m2 + s1²/m3
```

The important implementation lesson is larger than the exact formula: a visually stable bend projection must conserve momentum. A constraint that injects momentum causes a resting rope to “snake” by itself.

**Recommendation:** Reproduce this constraint before experimenting with the stock Unreal CableComponent’s every-other-node distance constraint. Add unit tests for linear and angular momentum before integrating collision.

### Friction

The talk describes three progressively stronger layers.

#### 1. Post-solver tangential damping

Remember which nodes received collision projections, then reduce their tangential velocity after velocity reconstruction:

```text
vt <- kfp vt
vt <- vt - min(kfc, |vt|) normalize(vt)
```

`kfp` is proportional friction and `kfc` is a constant velocity decrement. This was sufficient for *Uncharted 4*, where the dynamic rope was connected at both ends and usually had little slack.

#### 2. Static rope-tension estimate

PBD does not naturally expose constraint forces, so Naughty Dog estimated tension from the final rope shape after the solve:

- Tension is zero at a free end.
- Add the weight contribution along a freely hanging segment.
- Estimate normal force where rope bends around an edge.
- Estimate normal force from gravity at surface contacts.
- If a large bend is not caused by a collision edge, let tension fall to zero.

This estimate intentionally ignores velocity. It is inaccurate during fast motion, but friction matters most when the rope is slow or resting.

#### 3. In-solver friction constraint

Velocity damping alone cannot produce reliable static rest because the next position solve does not know about friction. The in-solver constraint remembers the node position at the start of the time step and pulls the node back toward it:

```text
Δp = p - p_start
p <- p - (sfp |Δp| + sfc) normalize(Δp)
```

The correction must be clamped so it cannot move past `p_start`. `sfp` and `sfc` are empirical proportional and constant solver-friction coefficients informed by the tension estimate. Inside the `sfc` radius, the node snaps back to its start position, providing a static-friction-like dead zone.

**Recommendation:** Implement the three layers separately and make each observable in debug rendering. Do not expose one undifferentiated “friction” slider.

**Possible later improvement:** XPBD provides constraint-force estimates and time-step/iteration-independent compliance. It may replace part of the heuristic tension model, but it does not solve wrapping topology or collision-geometry problems by itself.

### Convergence

**Talk:** An impulse propagates slowly along a Gauss–Seidel chain. A nearly taut rope is especially bad because local distance corrections are almost perpendicular to the needed global motion. The required iteration count was approximately proportional to node count; because each iteration visits all nodes, cost became approximately `O(n²)`.

Two disclosed accelerations:

1. **Over-relaxation**: pretend the target distance is slightly shorter. The talk used `ω = 1.015`:

   ```text
   effective target = l0 / ω,  ω > 1
   ```

2. **Multigrid**: solve successively finer rope discretizations, beginning with a coarse level. Nodes at collision edges must be retained in the coarsest level; skipping them lets the coarse solve pull the rope through collision and leaves unnatural bends.

The talk’s expensive PS4 profile was a 14 m rope with 272 nodes and 113 iterations, taking roughly 12 ms on CPU. This implies a fine spacing of about 5.15 cm. Treat those figures as evidence of the cost ceiling, not recommended defaults for a modern plugin.

### GPU implementation

**Talk:** The solver processed a 64-node wavefront. It still did more total work than the serial CPU implementation and took roughly 10 ms, but occupied only a small fraction of the PS4 GPU. Keeping shared-resource usage low allowed other compute work to share the compute unit.

**Recommendation:** GPU is a late milestone. A CPU reference solver, deterministic replay, collision topology, and profiling must exist first. Moving an unresolved algorithm to a compute shader will make the hardest failures less inspectable.

### Dynamic collision representation

This is the most production-specific part of the dynamic solver.

**Talk:** Before each solve, estimate how far each node might move using:

- Current velocity and direction.
- Pull directions from neighbouring nodes.
- Motion of nearby keyframed nodes.

Use that estimate to build a node-local AABB and gather nearby collision triangles. Compile those triangles into a very small constraint set:

- Merge sufficiently similar triangle planes.
- Detect pairs of planes that form convex edges.
- Rank/prune constraints based on the current node and occlusion by other planes.
- Retain at most four planes and three convex edges per node.

During the solve, a node collides against planes, while a paired convex edge provides a rounded radius region that lets the finite-radius node move smoothly from one face to the other.

This compression is fast inside the many solver iterations, but it is heuristic. Infinite planes create ghost collisions, groups of three or more convex planes are not represented faithfully, and competing planes can pull a node to the wrong side.

**Talk:** Naughty Dog never made this fully robust. Puzzle deployment areas were limited and tested. Problematic collision was cleaned, and triangles could be marked “ignore rope” or “rope only”.

### Collision-constraint preprocessing tricks

Two useful safety passes are disclosed:

1. **Prevent pinching:** detect contradictory planes trapping a node, prioritize the important collision (typically static world) over lower-priority collision (typically animated character), project out of the important plane, and preprocess the secondary constraint so the pair cannot fight across the important surface. The supplied captions omit several seconds of the exact explanation and the animation gives no final formula, so the precise plane-remapping operation remains undisclosed.
2. **Prevent stretching:** for collision plane `c` on node `i`, find a relevant keyframed node `k`. If the distance from `k` to the plane exceeds the physical rope length between `k` and `i`, the plane can only be satisfied by stretching the rope, so discard that constraint:

   ```text
   distance(keyframe, plane) > ropeArcLength(keyframe, node)
       => discard collision constraint
   ```

The second trick is especially useful for animated hands or props that teleport through static collision.

## Taut rope simulation

### Problem definition

Maintain a collision-free polyline between moving endpoints. Internal points lie on convex collision edges and move along those edges so the path tends toward the shortest collision-respecting route. Endpoint motion is continuous: the path must not tunnel through an edge, even for fast or thin-feature motion.

The computational cost is unrelated to regular rope length or node spacing. It depends primarily on nearby geometric and topological complexity.

### Required collision data

**Talk:** The algorithm reasons explicitly about oriented convex edges and their incident faces. At a vertex/intersection, it needs all incident edges within the global tolerance, not just the first numerically detected edge.

**Recommendation:** Bake a rope-specific collision asset containing:

```cpp
struct FRopeCollisionVertex
{
    FVector3d Position;
    TArray<int32> IncidentEdges;
};

struct FRopeCollisionEdge
{
    int32 Vertex0;
    int32 Vertex1;
    int32 Face0;
    int32 Face1;
    FVector3d Direction;
    bool bConvex;
};

struct FRopeCollisionFace
{
    FVector3d Normal;
    double PlaneOffset;
    TArray<int32> BoundaryEdges;
};
```

The bake should weld vertices using the same tolerance policy as runtime, calculate adjacency and convexity, remove degenerate features, and build a BVH over triangles and edges. Do not reconstruct this topology from arbitrary `UWorld` trace hits every tick.

### Taut path state

**Derived:** A rope path point must be more than a position:

```cpp
struct FTautRopePoint
{
    ETautPointType Type;        // Endpoint, EdgeContact, Intersection
    FVector3d Position;
    int32 PrimaryEdge;
    double EdgeParameter;
    TStaticArray<int32, 8> IncidentEdges;
    uint8 IncidentEdgeCount;
    uint64 StableFeatureId;
};
```

The talk’s implementation supported at most eight edges per rope point. Stable IDs are a CableSim recommendation to make replay, merging, and transformed collision sources reliable.

### Alternating movement and collision phases

This is the core disclosed algorithm.

```text
repeat
    Movement phase:
        Accept target endpoint positions.
        Move edge contacts along their current edges toward a shorter path.

    Collision phase:
        Move only one path point at a time.
        Sweep the adjacent line segment(s), producing one or two triangles.
        Intersect those swept triangles with nearby convex edges.

        If a new contact is found:
            Stop this collision phase.
            Move only as far as the contact.
            Insert/augment an edge-contact point.
            Restart with a new movement phase.
until a collision phase produces no new contact
```

Moving one endpoint while the other is fixed sweeps one triangle. Moving an internal edge point sweeps two triangles, one for each adjacent rope segment. The contact is the intersection of a swept triangle and a convex edge.

**Derived:** Select the earliest valid contact along the point’s movement and gather every incident edge within tolerance of that earliest position. Otherwise feature ordering will depend on traversal order.

### Sliding one contact

Given an edge contact `C` between path points `A` and `B`:

1. Compute perpendicular distances `rA` and `rB` from the edge.
2. Rotate one adjacent point about the edge into the plane containing the edge and the other point.
3. In that 2D plane, intersect the straight `A'B` line with the edge.
4. Use the intersection as the target parameter for `C`.
5. Run a collision phase before accepting the movement.

This is an unfolding operation: it converts the shortest broken 3D path around an edge into a straight 2D line.

### Sliding multiple contacts

**Talk:** For points `A-B-C-D`, solve contact `B` using triplet `A-B-C`, then solve `C` using `B-C-D`, and iterate until the path is as straight as its supporting edges allow.

This local iteration stalls when consecutive edges approach one another. If consecutive edges are coplanar (including parallel), unfold the entire edge chain into a common 2D plane, draw a straight line between unfolded endpoints, intersect it with every unfolded edge, and fold the contact positions back into 3D. If consecutive edges are skew, retain the iterative method.

### Edge intersections and topology

This is the difficult bookkeeping layer.

The solver establishes:

- A direction along the rope from an origin endpoint to the other endpoint.
- An orientation for each edge. Viewed along the positive edge direction from the incident face facing the rope origin, the rope must pass on the right side.
- An oriented plane for each pair of incident edges.
- The direction the rope would move along each edge if that edge were considered alone.

It classifies each pair of incident edges, then combines the pairwise results for the full intersection:

- **Inner corner**: can be stable (rope remains caught), move along A, or move along B.
- **Unstable corner**: choose A or B, move along A, or move along B.
- **Side edge**: one edge is topologically inaccessible and is ignored.
- **Outer corner**: path order is A-then-B or B-then-A; one intersection point can split into two edge contacts.

The rope can topologically cross the edge-pair plane twice even though both crossings coincide geometrically at the intersection. The return crossing reverses the required side of each oriented edge. This is why a purely positional “closest edge” implementation is insufficient.

For an intersection with many incident edges, the disclosed reduction is:

1. Build the pairwise classification matrix.
2. Remove side edges.
3. If any remaining pair is a stable inner corner, retain one intersection contact.
4. Otherwise use move-along classifications to abandon edges.
5. Use outer-corner classifications to split one contact into two ordered contacts.
6. When two contacts later come within tolerance, merge them and reclassify.

The talk explicitly notes that evaluating pairs and single-edge movement in isolation has obscure failure cases. It is a pragmatic approximation, not a complete general topology solver.

### Numerical robustness

Use one coherent tolerance policy through:

- Point/edge distance tests.
- Edge/edge intersections.
- Swept-triangle contact collection.
- Feature welding.
- Contact split/merge decisions.
- Parallel/coplanar/skew predicates.
- Orientation predicates.

Naughty Dog used `0.001 m`. In Unreal’s default centimetre world units this is `0.1 cm`.

Do not scatter `KINDA_SMALL_NUMBER` and custom epsilons throughout the implementation. Define a scale-aware `FRopeGeometryTolerance` and pass it explicitly to every predicate.

Near singular configurations—especially rope nearly parallel to an edge—orientation may be undefined. Prefer temporal coherence from the previous valid classification, a deterministic feature-ID tie break, or a conservative “stay on current feature” result. Never let tiny floating-point noise randomly flip the required side of an edge.

### Known taut-solver limits

- Never reached 100% robustness.
- Numerical predicate bugs.
- Pairwise simplification of multi-edge intersections.
- Desired movement evaluated one edge at a time.
- Maximum eight edges stored per rope point.
- Rapid motion almost parallel to a complex surface hits many singular cases.

The talk shows local clipping during the jeep sequence. Other contacts tend to keep the path near the surface and eventually pull it back out, limiting catastrophic gameplay failures.

## Coupling the two solvers

### Uncharted-style safe line

The taut solver owns a collision-free polyline and its path length. Gameplay prevents endpoints from separating beyond available rope length.

The dynamic nodes receive additional constraints that keep them within a corridor around the taut path. As the dynamic rope approaches taut, this allowed distance shrinks. The talk does not give the exact function.

**Recommendation:** For material coordinate `s` on a dynamic node:

1. Map `s` onto the corresponding point of the taut path by accumulated taut arc length.
2. Let global slack be `max(ropeLength - tautPathLength, 0)`.
3. Define a tunable corridor radius proportional to local allocated slack, with a nonzero visual minimum only when gameplay permits.
4. Project the node toward the corridor if it exceeds the allowed distance.

This constraint must not replace normal collision; it is a safety cage and convergence aid.

### Seeding taut state from a dynamic rope

When a free dynamic rope is picked up, *The Last of Us Part II* uses the current dynamic shape to quickly find the invisible taut path.

The talk does not disclose the conversion algorithm. A CableSim approach is:

1. Simplify the dynamic centreline while preserving collision-feature changes.
2. Trace consecutive simplified segments through the baked edge BVH.
3. Insert candidate edge contacts in material order.
4. Run the normal taut movement/collision loop to canonicalize and shorten the path.
5. Reject or fall back if topology cannot be made collision-free within a bounded iteration count.

This should be a later milestone. It is unnecessary for a grapple/winch MVP.

## Unreal Engine 5.8 assessment

### Existing project

- `CableSim.uproject` targets Unreal Engine 5.8.
- `Source/CableSim` contains only the default primary game module.
- There is no project plugin directory or cable implementation yet.

### Built-in CableComponent

The engine source is at:

`/Users/Shared/Epic Games/UE_5.8/Engine/Plugins/Runtime/CableComponent`

It is useful as a renderer/component reference, but not as the simulation foundation for this target:

- Verlet particles with current and old positions.
- Uniform segment length.
- Only start/end attachment controls.
- Fixed number of segments.
- Gauss–Seidel distance constraints.
- Optional every-other-particle distance constraints for stiffness.
- Per-particle sphere sweep collision after the distance solve.
- Simple tangential friction via implied Verlet velocity.
- No collision-plane compilation, convex-edge topology, multigrid, internal keyframes, dynamic discretization, taut path, or hybrid safe line.

The existing scene-proxy tube generation is a useful reference, but copying the whole component and extending it would accumulate the wrong assumptions in the simulation core.

### Recommended plugin layout

```text
Plugins/CableSim/
  CableSim.uplugin
  Source/
    CableSimCore/        # UObject-free math, dynamic solver, taut solver, replay
    CableSimRuntime/     # Components, world adapters, rendering, Blueprint API
    CableSimEditor/      # collision baker, visualization, asset validation
    CableSimTests/       # low-level deterministic automation tests
```

`CableSimCore` should use plain structs and injected interfaces. It must be runnable in tests without constructing a `UWorld`.

Suggested interfaces:

```cpp
class IRopeCollisionProvider
{
public:
    virtual void QueryDynamicNodeGeometry(
        const FBox3d& Bounds,
        TArray<FRopeTriangle>& OutTriangles) const = 0;

    virtual void QueryConvexEdges(
        const FBox3d& Bounds,
        TArray<int32>& OutEdgeIds) const = 0;
};
```

The runtime implementation reads baked static rope collision plus explicitly supported moving primitives. The solver does not call `UWorld` directly.

### Collision authoring strategy

For the first robust implementation:

1. Add an editor-only baker that reads selected collision meshes.
2. Weld vertices with the configured rope tolerance.
3. Build face adjacency and label convex edges.
4. Allow per-component/triangle rope inclusion and exclusion.
5. Store a cooked `URopeCollisionData` asset and BVH.
6. Visualize rejected, non-manifold, T-junction, over-valence, and near-degenerate features.

General Chaos collision should remain a separate adapter. High-level traces are suitable for a simple dynamic-rope prototype, but they do not expose the stable edge topology required by the taut solver.

### Coordinate and determinism policy

- Author public measurements in Unreal centimetres, but convert once at the core boundary.
- Keep one explicit 1 mm topology tolerance (`0.1 cm`) initially.
- Simulate in a local origin near the rope to preserve precision in large worlds.
- Use stable collision feature IDs and deterministic ordering before all reductions.
- Fixed time step and fixed iteration counts.
- No unordered container iteration in core solve decisions.
- Record configuration hashes in replay dumps.

Local deterministic replay does not automatically imply cross-platform bitwise deterministic multiplayer. Network design should be a separate milestone.

## Debugger requirements

The talk calls the rope debugger essential. Build it before complex collision.

Per-frame ring-buffer capture should include:

- Time step and solver configuration.
- Endpoint/keyframe targets.
- Every dynamic node position, velocity, mass, and mode.
- Generated distance/bend/friction/collision constraints.
- Taut points, edge IDs, parameters, and pairwise classifications.
- Collision-query inputs and returned feature IDs.
- Every split, merge, insert, discard, and tie-break event.
- Solver iteration counts, residuals, and fallback reason.

Required controls:

- Pause, step backward, and replay forward.
- Repeat one frame from identical input.
- Toggle each constraint family.
- Draw planes, normals, convex edges, swept triangles, tolerance regions, safe line, and material coordinates.
- Save/load a self-contained rope dump with nearby collision data.

Automated bug reports should attach the smallest replayable dump, not a level name and screenshot.

## Implementation roadmap

### Milestone 0 — plugin, core, tests, and debugger skeleton

- Create the four-module plugin layout.
- Establish units, fixed step, tolerance policy, logging, stats, and feature IDs.
- Add replayable core input/output structs and a small headless test harness.
- Add debug draw commands before real constraints.

Exit criterion: a recorded synthetic frame replays to the identical state and event sequence.

### Milestone 1 — reference dynamic rope without world collision

- Dynamic/keyframed nodes.
- Arbitrary internal keyframes.
- Distance constraints and fixed iterations.
- Runtime rope length and node insertion/removal by material coordinate.
- Gravity, damping, and velocity reconstruction.
- Simple component and debug-line rendering.

Exit criteria: hanging, two-endpoint, moving-keyframe, and dynamic-length tests conserve length within a specified bound and replay deterministically.

### Milestone 2 — bending and controlled friction

- Momentum-conserving three-node bend constraint.
- Free bend angle.
- Ground plane and simple analytical primitives.
- Post-solver tangential damping.
- Static tension estimate and in-solver friction constraint.

Exit criteria: no self-propelled resting rope; rope remains plausibly at rest in the talk’s bar and sloped-contact tests.

### Milestone 3 — rope collision asset and local constraint compiler

- Editor collision baker and validator.
- Static BVH.
- Node motion bounds.
- Triangle-plane merging, convex-edge pairing, ranking, and caps.
- Pinch and impossible-stretch preprocessing.
- Rope-only collision layers.

Exit criteria: window corner, convex corner, ghost-plane, animated pinch, and fast-pull regression dumps.

### Milestone 4 — basic taut path

- Static oriented convex edges.
- Endpoint swept-triangle collision.
- Insert/remove a single edge contact.
- One-edge unfolding and sliding.
- Alternating movement/collision phases with bounded termination.
- Gameplay path-length query.

Exit criterion: a moving segment wraps and unwraps around one isolated box edge without tunnelling.

### Milestone 5 — multi-contact taut path and topology

- Iterative triplet sliding.
- Coplanar-chain 2D unfolding.
- Intersection edge gathering.
- Pairwise classification matrix.
- Inner, unstable, side, and outer corner cases.
- Contact split/merge and temporal-coherence rules.

Exit criteria: deterministic golden tests for every classification and a five-edge intersection matching the talk demonstration.

### Milestone 6 — hybrid grapple/winch

- Dynamic-to-taut material mapping.
- Safe-line corridor constraints.
- Gameplay movement limit from taut path length.
- Winch length changes.
- Renderer resampling independent of simulation nodes.

Exit criterion: a grapple swing and winch path remain playable even if the visual dynamic rope is deliberately destabilized.

### Milestone 7 — free throwable rope

- Robust dynamic collision over constrained authored areas.
- Throwing and coiling workflows.
- Pick up at arbitrary material coordinates.
- Seed taut topology from dynamic shape.
- Recovery and failure policies.

This is the *Last of Us Part II* tier and should be treated as a separate product milestone.

### Milestone 8 — optimization and platform work

- Profile representative ropes and collision density.
- Multigrid and conservative over-relaxation.
- Task/thread scheduling.
- GPU experiment only if CPU profiling justifies it.
- Replication/reconciliation policy.

## Tests to create before gameplay content

### Dynamic solver

- One fixed end; two fixed ends; free fall.
- Unequal masses and internal keyframes.
- Runtime length increase/decrease.
- Linear/angular momentum check for bending.
- Free bend threshold.
- Static friction on a plane.
- Rope draped over a bar with unequal hanging lengths.
- Nearly taut convergence.
- Collision-edge nodes retained in coarse multigrid levels.
- Node pinched between static and animated collision.
- Impossible collision plane requiring more arc length than available.

### Taut solver

- One endpoint sweep into one edge.
- One contact sliding both directions and leaving the edge.
- Two coplanar parallel edges and two skew edges.
- Stable inner corner.
- Inner corner moving along A and B.
- Unstable corner with deterministic tie break.
- Side-edge elimination.
- Outer-corner split in both orders.
- Merge two contacts at a vertex.
- Rope parallel to a face/edge.
- Two meshes separated by a gap smaller and larger than tolerance.
- More than eight incident edges with a defined overflow policy.
- High-speed motion along a surface.

Every test should have a saved input, expected topology/event sequence, and a visual debug snapshot.

## What not to do first

- Do not extend the built-in CableComponent into the final architecture.
- Do not begin with a GPU solver.
- Do not make arbitrary Chaos collision geometry a day-one promise.
- Do not combine rendering samples with physics nodes.
- Do not depend on high-level trace hits for stable taut topology.
- Do not expose dozens of empirical coefficients before building diagnostic scenes.
- Do not claim “fully deterministic multiplayer” from local replay determinism.
- Do not make a throwable 14 m rope the first acceptance test.

## Open questions for the CableSim product

These choices change scope materially and should be decided before implementation:

1. Is the first shippable use case a visual cable, grapple, winch, or throwable puzzle rope?
2. Must the rope interact with arbitrary level collision, or can users author/bake rope collision volumes?
3. Which moving colliders are required initially: characters, rigid bodies, skeletal meshes, or none?
4. Is gameplay force feedback into attached rigid bodies required, or is the rope initially kinematic/one-way?
5. What are the target platforms and maximum simultaneous ropes?
6. Is network replication a first-release requirement?

The recommended defaults are: hybrid grapple/winch, baked static rope collision, simple character primitives, one-way endpoint constraints, CPU reference solver, and no first-release network guarantee.

## Follow-up reading

- Müller, Heidelberger, Hennix, Ratcliff (2007), [*Position Based Dynamics*](https://doi.org/10.1016/j.jvcir.2007.01.005).
- Macklin, Müller, Chentanez (2016), [*XPBD: Position-Based Simulation of Compliant Constrained Dynamics*](https://mmacklin.com/xpbd.pdf).
- Müller, Chentanez, Jeschke, Macklin (2018), [*Cable Joints*](https://matthias-research.github.io/pages/publications/cableJoints.pdf).

XPBD is relevant to compliance and force estimates. Cable Joints is relevant as an alternative way to represent long inextensible spans between contacts. Neither replaces the taut solver’s continuous edge topology as disclosed in this talk.
