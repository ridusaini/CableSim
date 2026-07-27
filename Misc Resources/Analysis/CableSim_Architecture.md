# CableSim architecture

This note records the current implementation boundary after the GDC-alignment pass.

## Runtime flow

1. `UCableSimComponent` samples explicit endpoint bindings at frame rate and feeds fixed substeps.
2. `CableSimCore::FSolver` owns dynamic particles and all deterministic PBD state.
3. `FCableSimWorldCollisionAdapter` performs Unreal queries and maintains a collider-identity cache; each query pass replaces the solver's active contact set.
4. `CableSimCore::FTautPathSolver` owns a separate ordered topological path. It never shares particles with the dynamic solver.
5. The taut path limits a carried kinematic endpoint and supplies iteration-independent guide corridors to the dynamic rope.
6. `UCableSimComponent` publishes its last valid state through a persistent `FDebugRenderSceneProxy`; viewport rendering is independent of simulation ticks and the world's transient line batcher.

`CableSimCore` has no `UWorld`, Chaos, actor, or component dependency. Unreal object resolution and collision extraction remain private to `CableSimRuntime`.

## Endpoint contract

- `Simulated`: a normal dynamic endpoint.
- `CableLocalKinematic`: follows a target in the cable component's local frame.
- `WorldKinematic`: follows an explicit world target.
- `ComponentKinematic`: follows one explicit component/socket and local offset.

There is no actor-tag search and no fallback to a different ownership model. If a component or socket disappears, the endpoint freezes at its last valid target and reports the binding failure.

Reach ownership is explicit. `TautSettings.ConstrainedEndpoint` selects Start or End, the opposite non-simulated endpoint is the anchor, and failed topology freezes the constrained end at its last accepted reachable position.

## Collision and friction

The active contact array is current-pass state. Persistence belongs only to the runtime cache. Cache entries retain collider identity, face/item identity, a collider-local plane and friction anchor, and current surface velocity. Remeshing, teleporting, endpoint mode changes, and collision-setting changes clear the cache.

Friction is applied once per particle, not once per contact per solver iteration. The core builds an orthonormal normal basis, uses the previous frame's estimated tension/normal load, caps static position correction for the complete step, and applies a single dynamic velocity decrement after velocity reconstruction.

Bending stiffness is also a complete-step strength. The solver derives its per-iteration strength so changing the fixed iteration count does not silently change the cable material. The default flexible-cable profile uses `0.20` step strength and `90 degrees/metre` of free bend.

## Taut topology

The taut solver retains up to 32 ordered points and 32 topology events per step. Static box/convex snapshots contain deterministic triangle, edge, and shared-vertex IDs with sorted incident-edge lists. The solver supports separate multi-edge contacts, sliding, unwrapping, shared-vertex transitions, full-path reach calculation, and visible failures for missing, non-manifold, or over-valence topology.

Guide coupling fades in with smoothstep between its activation path ratio and full extension. This avoids an abrupt constraint change as a slack cable approaches tautness.

## Visualization

The cyan centreline is a persistent cable preview controlled by `PreviewSettings`, not a diagnostic flag. Particles, contacts, friction/load, candidate topology, taut routing, and status remain independently selectable diagnostics. The proxy owns an immutable copy until the component supplies a replacement, so editing or remeshing one cable cannot blank any other cable. Normal editor worlds build a non-advancing authoring preview; PIE and game worlds render interpolated solved positions. Diagnostic overlays are excluded from Shipping builds.

The dynamic world-sweep system is deliberately **not** converted into nearby geometry planes/convex-edge manifolds in this pass. That remains a dedicated collision-representation experiment. A full frame ring buffer/dump debugger and multigrid solver are also deferred; current diagnostics are live visual state, and node-count scaling is covered by an automation benchmark.
