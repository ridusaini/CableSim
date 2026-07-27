# Taut Path Playground Validation

The taut system reads loaded Chaos collision and maintains a separate zero-radius topological path. The particle rope remains the rendered simulation, while the taut path limits kinematic endpoint reach and supplies guide constraints to the dynamic solver.

## Enable it

In `L_Playground`, select the actor that owns `CableSimComponent` and set:

- **Taut Settings > Enable Taut Solver**: enabled
- **Taut Settings > Constrained Endpoint**: choose the carried/moved endpoint; the opposite kinematic endpoint is the anchor
- **Preview Settings > Visible**: enabled
- **Debug Settings > Draw**: enabled
- **Debug Settings > Draw Taut Path**: enabled
- **Debug Settings > Draw Taut Contacts**: enabled
- **Debug Settings > Draw Taut Status**: enabled
- **Debug Settings > Draw Extracted Edges**: enabled while validating topology, then disable it for normal iteration
- **Debug Settings > Draw Snapshot Bounds**: optional

No level asset was edited by the implementation.

## Debug legend

- Cyan line: persistent cable preview generated from the interpolated particle cable.
- Magenta line: taut shadow path.
- Yellow sphere: retained taut edge contact.
- Green edge: convex extracted edge and potential taut feature.
- Red edge: concave edge.
- Blue edge: coplanar triangulation edge, rejected by the taut solver.
- Yellow edge: open boundary edge, rejected by the taut solver.
- Purple box: transient Chaos snapshot query bounds.

The status text reports extracted triangles, vertices, supported edges, contact count, endpoint reach, and snapshot/compile/solve time.

## Minimum fixture

Use two kinematic endpoints and add one ordinary **static** cube or box-collision obstacle between them. Keep the obstacle Mobility set to Static and ensure it responds to the component's collision channel. The current topology path accepts static Box or Convex Chaos geometry as solver input.

1. Start with both endpoints on the same side of a cube edge. Expected status: `Ready` or `NoRelevantGeometry`, with a two-point magenta path.
2. Move one endpoint around a visible convex edge while the other endpoint remains on the other incident face. The endpoint sweep must cross the edge. Expected: a yellow contact and a three-point magenta path.
3. Move the endpoints along the edge axis. Expected: the yellow contact slides along the edge.
4. Return both endpoints to the same incident face. Expected: the contact is removed and the magenta path returns to two points.
5. Move far enough along the edge that the mathematical contact reaches a shared vertex. Expected: a deterministic transition to the next valid convex edge, or contact removal if the direct path no longer wraps the obstacle.

## Boundary checks

- Change the obstacle to Movable or simulate physics. Its features may still be visible in extracted-edge debug, but `SupportedEdgeCount` should exclude them.
- Test a Landscape or complex triangle mesh. Triangles and edges should be visible for extraction diagnostics, but they are intentionally not connected to the taut solver in this slice.
- Route across two separated convex features. Expected: two ordered retained contacts and a four-point path; reversing the move unwraps them in order.
- Exercise non-manifold geometry or a vertex with more than the configured incident-edge budget. Expected: `NonManifoldTopology` or `TopologyOverValence`, with the last valid path preserved for diagnosis.
- Remove or unload an obstacle that owns an active contact. Expected: `FeatureInvalidated`; the last valid path is preserved for diagnosis.

## Current limits

- Zero-radius taut path only; finite cable thickness still belongs to the dynamic solver.
- Up to 32 ordered path points and 32 topology events per step.
- Static Box/Convex solver geometry only.
- Loaded World Partition collision only; there is no manual or global bake.
- Zero-radius topological routing; particle radius and contact response still belong to the dynamic solver.
- Taut reach and guide coupling are implemented, but force transfer to rigid bodies is not.

These limits are intentional gates. The next topology pass should be selected from playground evidence, with finite-radius routing and broader Chaos shape support treated as separate work.
