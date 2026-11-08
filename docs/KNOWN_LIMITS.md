# Known limits and open defects

Numbers come from the runs described here. To replay any fuzz case:

```sh
cargo run --release -p coreshape --example replay -- <gen|al> <seed>
```

## Fuzz results

25,000 randomized cases, each running union, subtract and intersect. Every returned result is
checked three independent ways: validity with a watertight mesh, the volume identities, and
point membership against both inputs.

| Category | Cases | Refused | Wrong |
|---|---|---|---|
| Boxes on an integer grid | 5,000 | 0 | 0 |
| Boxes in arbitrary orientations | 5,000 | 0 | 0 |
| Prisms and plates on a grid | 5,000 | 0 | 0 |
| Any primitive in general position | 5,000 | 4 (0.08%) | 0 |
| Primitives aligned on a grid | 5,000 | 254 (5.1%) | 0 |

"Refused" counts cases where at least one of the three operations returned an error.

**General-position refusals,** all still unexplained at the level of a root cause. They are
not tangency cases by construction, so they deserve a closer look:

| Seed | Message |
|---|---|
| 1305 | A sphere region crosses the other body but was not cut there |
| 1881 | The result does not close: an edge is used by 1 face |
| 2056 | Two edges claim the same continuation, on a cone |
| 4609 | Two edges claim the same continuation, on a sphere |

**Aligned refusals** are nearly all exact tangency between surfaces with no closed-form
intersection, which integer grids produce often. For example, a plane tangent to a torus at a
saddle point gives a curve that crosses itself there. A few of these report as internal errors
("two edges claim the same continuation", "the result does not close") rather than naming the
tangency, because the curve touches a face boundary tangentially at a vertex.

## Cases OpenCASCADE handles and this kernel refuses

Found by an independent review harness and still open:

| Case | This kernel | OpenCASCADE |
|---|---|---|
| Cylinder r = 5.00001 over r = 5, coaxial: subtract the thin shell | Refused: "the result does not close" | Volume 0.003 |
| Cylinder r = 5 with its axis offset 1e-5: subtract the sliver | Refused: "the result does not close" | Volume 0.001 |
| Fillet an elliptical rim | Refused: unsupported curve | 7835.628 |
| Fillet the saddle edge of a tee pipe | Refused: unsupported curve | 20449.687 |

The first two are slivers near the tolerance, and a refusal there is the safe answer, but the
message should name the sliver instead of blaming closure.

## What Booleans refuse by design

- **General tangency:** a cylinder internally tangent to a sphere, or a plane tangent to a torus
  away from its axis, where the intersection crosses itself. Slide to "Known limit: tangent bore"
  in the Stress cases demo to see one. Tangency handled in closed form does work: coaxial
  surfaces, a plane tangent to a cylinder, tangent parallel cylinders, and so on.
- **Very small features:** intersection loops under roughly 1/100 of the smaller surface's radius,
  and edges within tolerance of each other.
- **Tiny differences:** the tolerance is 1e-7 mm, and nothing shorter than 1e-6 mm can be built.

## Fillets and chamfers

Supported:

- Straight edges between planes, or between a plane and a cylinder parallel to the edge.
- Circular edges between surfaces sharing the circle's axis: plane, cylinder, cone, sphere, torus.
- Convex and concave edges. Picking one piece of a circle selects the whole rim.
- Tangent chains of those edges, such as the rim of a rounded rectangle. Volumes match
  OpenCASCADE to 1e-9 where the ball's path is unique.
- A rim whose corner radius equals the fillet radius: a sphere at each corner, matching
  OpenCASCADE to 1e-9.
- A rim whose corner radius is below the fillet radius: see the difference below.
- A sphere patch where three convex edges between planes meet.

Not supported, returning an error:

- Edges on traced intersection curves or ellipses, such as the rim of a hole through a cylinder.
- Variable radius.
- Chamfers do not follow tangent neighbours. Fillets do by default (`fillet_with` turns it off).
  That is deliberate for now, and easy to change.
- A radius that runs off a neighbouring face. Real kernels can absorb the overrun and restructure
  the neighbours. This one refuses.

Works, but differs from commercial CAD:

- **A rim tighter than the fillet radius has no exact rolling-ball solution.** This kernel uses
  the exact surface the section sweeps: the lens-shaped inner part of a self-intersecting torus,
  ending in a tip on the axis, with the straight fillets meeting in mitres. OpenCASCADE fits a
  free-form patch with a degenerate edge, which bulges 0.011 mm past the original wall. Volumes
  differ by 7e-4 of the total (43883.00 against 43851.31 on the review's case), so that case is
  tested against direct integration, not against OpenCASCADE. A later fillet on a neighbouring
  edge differs for the same reason (23801.82 against 23800.77).

- Two filleted edges meeting at a corner with a third edge left sharp end in a plain mitre, with
  no vertex blend.
- Mixed convex and concave edges at one vertex have little test coverage. The Boolean checks
  still guard the result, but the shape may not be the one you'd pick.

**Random slider positions on the demos** (200 per model): no invalid bodies. Refused: bracket 12,
flange 6, plate 11, crossed cylinders 7, and stress 11 (always the deliberate known-limit case).
Most refusals are dimensions that truly don't fit. The rest are exact tangencies, for example
crossed cylinders whose radius difference equals the axis offset.

## Documents

- Each feature keeps one cached result, so switching back and forth between two states recomputes
  whatever changed each time.
- All patches made by one fillet or chamfer feature share a face name. A name can also cover several
  pieces of a face split by a later feature. Selecting by name takes all of them; combine it with
  `near` or `in_box` to pick one.
- Coordinates beyond 10 km are refused, because tolerances stop meaning anything there.
- A sketch can make at most 4000 regions, and a region can have at most 999 holes before names stop
  being unique. Sketch geometry is limited to what the shape types can write; the constraint solver
  will add solved sketches.

## Numerics

- Arithmetic is `f64` with a tolerance, not exact arithmetic. Geometry within a few tolerances of
  degenerate is where failures concentrate.
- The same input gives the same bits on one platform, in any process. Results may differ in the last
  bits between platforms, because math libraries differ.
- Traced curves are refined samples with at most 12 degrees of turn between them. STEP gets them as
  C1 cubic B-splines through those samples, which approximate the true curve between samples.
  OpenCASCADE's volume agrees with ours to about 1e-7 relative on these models, compared with
  1e-15 on purely analytic ones.
- Bodies inside a multi-step operation get lighter checks. Only the final result gets the mesh check.
- OpenCASCADE's shell check depends on face order for shells that touch themselves: rotating the face
  list of a body with a zero-thickness pinch flips "Unorientable shape". The STEP writer emits faces
  in depth-first connected order, which passed from all 12 start faces tried (breadth-first 10 of 12,
  random about half). This is an observed property of that checker, not a rule from the standard.

## Not tested

- **On a real iPhone or in Safari.** The viewer has only run in headless Chrome with a software
  renderer. The controls, shaders and touch handling are written for iOS but haven't been used on it.
- **The hosted page itself.** It has been published but not opened. If the host's content policy
  blocks WebAssembly, the page shows an error saying it could not start.
- **Phone performance.** Measured on this desktop in wasm under headless Chrome, a slider change
  took 45-110 ms on the bracket, 195-270 ms on the flange, about 300 ms on the rounded block and
  390-630 ms on the mounting plate, before the later scaling work. That is a few times slower than
  native. A phone's speed has not been measured.
- **Running on macOS and Windows.** The kernel compiles for both, but tests have only run on Linux.
- **The download path on the hosted page.** Saving uses the host's download capability with the
  file wrapped in a zip, because `.step` and `.stl` aren't on its allowlist.
- **Other CAD systems.** STEP files have been checked in OpenCASCADE only.
