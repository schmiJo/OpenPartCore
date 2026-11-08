# Known limits and open defects

Numbers come from the runs described here. To replay any fuzz case:

```sh
cargo run --release -p coreshape --example replay -- <gen|al> <seed>
```

## Open defect

**Aligned seed 3833: a returned solid that won't mesh at some settings.** A sphere (centre
(0, -1, 2), radius 3) minus a torus (centre (1, 2, 2), R 4, r 1). `subtract` returns a body
that passes every check, and its volume identities hold, but `tessellate` fails on it at a
coarser chord setting than the one the kernel checks with. `union` on the same pair is refused.
An edge pair runs too close together on the sphere face. The geometry is not known to be wrong,
but a body that only meshes at some settings shouldn't be returned. One case in 25,000.

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
| Primitives aligned on a grid | 5,000 | 316 (6.3%) | 1 (seed 3833, above) |

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
- Tangent chains of those edges, such as the rim of a rounded rectangle.
- A sphere patch where three convex edges between planes meet.

Not supported, returning an error:

- Edges on traced intersection curves or ellipses, such as the rim of a hole through a cylinder.
- Variable radius.
- A radius that runs off a neighbouring face. Real kernels can absorb the overrun and restructure
  the neighbours. This one refuses.

Works, but differs from commercial CAD:

- Two filleted edges meeting at a corner with a third edge left sharp end in a plain mitre, with
  no vertex blend.
- Mixed convex and concave edges at one vertex have little test coverage. The Boolean checks
  still guard the result, but the shape may not be the one you'd pick.

**Random slider positions on the demos** (150 per model): no invalid bodies. Refused:
bracket 11, plate 4, crossed cylinders 5, and stress 14 (always the deliberate known-limit case).
Most refusals are dimensions that truly don't fit. The rest are exact tangencies, for example
crossed cylinders whose radius difference equals the axis offset.

## Numerics

- Arithmetic is `f64` with a tolerance, not exact arithmetic. Geometry within a few tolerances of
  degenerate is where failures concentrate.
- Results may differ in the last bits between platforms, because math libraries differ.
- Traced curves are refined samples with at most 12 degrees of turn between them. STEP gets them as
  C1 cubic B-splines through those samples, which approximate the true curve between samples.
  OpenCASCADE's volume agrees with ours to about 1e-7 relative on these models, compared with
  1e-15 on purely analytic ones.
- Bodies inside a multi-step operation get lighter checks. Only the final result gets the mesh check.

## Not tested

- **On a real iPhone or in Safari.** The viewer has only run in headless Chrome with a software
  renderer. The controls, shaders and touch handling are written for iOS but haven't been used on it.
- **The hosted page itself.** It has been published but not opened. If the host's content policy
  blocks WebAssembly, the page shows an error saying it could not start.
- **Phone performance.** Measured on this desktop in wasm under headless Chrome, a slider change
  takes 45-110 ms on the bracket, 195-270 ms on the flange, about 300 ms on the rounded block and
  390-630 ms on the mounting plate. That is a few times slower than native, and the plate will feel
  laggy. A phone's speed has not been measured.
- **Running on macOS and Windows.** The kernel compiles for both, but tests have only run on Linux.
- **The download path on the hosted page.** Saving uses the host's download capability with the
  file wrapped in a zip, because `.step` and `.stl` aren't on its allowlist.
- **Other CAD systems.** STEP files have been checked in OpenCASCADE only.
