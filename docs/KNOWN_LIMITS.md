# Known limits and open defects

What the kernel refuses, what it only approximates, and what hasn't been tested. The sketch solver's limits and results are in
[SKETCH_SOLVER.md](SKETCH_SOLVER.md), and the MCP server's in [MCP.md](MCP.md). Every section was last compared with the code on 2026-11-08.

Numbers come from the runs described here. To replay any fuzz case:

```sh
cargo run --release -p coreshape --example replay -- <grid|rot|prism|gen|al|ext|rev> <seed>
```

## What isn't built

These are missing, not failing. [PLAN.md](PLAN.md) has the order.

- **Free-form surfaces.** No lofts, and no `B_SPLINE_SURFACE` from a file. The only spline surfaces are curves swept along a line or around an axis. This is phase 3 of [NURBS_DESIGN.md](NURBS_DESIGN.md), which isn't decided.
- **Helix, delete face and thicken.** Not started. A sweep path can only be lines and arcs.
- **Offset, shell and draft that change a body's topology.** They refuse instead: see [below](#offset-shell-and-draft).
- **Fillets and chamfers beyond straight and circular edges.** No ellipses, traced curves or spline faces, no variable radius, and no general vertex blends: see [below](#fillets-and-chamfers).
- **Assemblies and drawings.** A STEP assembly is refused.

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

**Spline shapes,** in the same three checks, against a primitive or another of their kind. Run with `SPLINES=1`:

| Category | Cases | Refused | Wrong |
|---|---|---|---|
| Free-form splines and ellipses, extruded | 1,000 | 1 (0.1%) | 0 |
| Spline profiles, revolved | 1,000 | 1 (0.1%) | 0 |

The refusals are "extrusion and torus meet in a loop too small to trace" (`ext 775`) and "plane and torus are tangent at a point on their
intersection" (`rev 115`). An earlier run reported one wrong result, a mesh that wouldn't close on a seed-96 extrusion. The profile really did cross
itself, within the last sample before a corner, and outlines are now checked more closely toward corners, so that one is now refused.

**In OpenCASCADE,** all 50 parts of the gallery read back valid, 15 of them spline parts. On those 15, volumes agree to 3e-7 or better on 13. On `spline_outline` and `spline_drilled`, OpenCASCADE's
reported volume is 1.8e-4 and 1.6e-4 above this kernel's. Its own numbers settle this in the kernel's favour: for `spline_outline`, a polygon through 20,001 points
of OpenCASCADE's own edges gives a cap area of 812.6155, and this kernel's is 812.6155, while its `Area` for that face says 812.9296. A mesh of the solid made by OpenCASCADE
has volume 8126.09, against this kernel's 8126.16 and its reported 8127.58.

## STEP import

`coreshape::import::step::read_step`, or the `import_step` feature. It reads closed solids on planes, cylinders, cones, spheres, tori, and curves swept along a line or around an axis, in millimetres, metres, inches and so on.

- **Refused, naming the surface and how many faces use it:** any other surface, such as `B_SPLINE_SURFACE`, so that no result has faces missing. That's free-form surfaces, which is phase 3 of [NURBS_DESIGN.md](NURBS_DESIGN.md).
- **Refused:** assemblies, since parts placed by transforms aren't read yet; open shells; any other edge curve, such as a hyperbola; a vertex over 1e-3 mm off the surfaces around it; a surface of revolution whose curve turns too far for one face, which an extrusion's curve can do because it gets cut up.
- **Faces change shape on the way in.** This kernel's edges have two different ends and its curved faces stay within half a turn, so a seamed full cylinder comes in as two halves under one name, and so on. Volumes and areas don't change.
- **Traced edges.** A B-spline edge between two surfaces, which is how other systems store an intersection curve, gets resampled onto both surfaces, so it follows them to this kernel's tolerance and not the file's. That's why `tee` and `cross_drilled` differ from OpenCASCADE by about 1e-6 relative.
- **A file's orientation flags aren't trusted.** OpenCASCADE writes `FACE_BOUND` flags that its own reader repairs by looking at the shape, so this works out each loop's direction from its neighbours, and from a flat face to set the sign.

**Checked** by `scripts/check_step_import.sh`, which has OpenCASCADE write 22 shapes, then reads each, compares volume and area, meshes it, cuts a box out of it, writes it back, and has OpenCASCADE check the result. All 21 that can be represented match, with volumes within 2.2e-6 relative and most about 1e-13, watertight meshes, and results valid in OpenCASCADE. The one free-form loft is refused by name. 3,000 randomly damaged files were each read as a valid solid or refused, with no panics, and everything this kernel writes comes back the same. Other writers than OpenCASCADE haven't been tried.

## Spline surfaces

Curves swept along a line or around an axis: see [NURBS_DESIGN.md](NURBS_DESIGN.md).

- **Refused by name:** fillets, chamfers, offset, shell and draft on these faces, because the true result isn't a spline. Their neighbours can still be filleted if the edge
  doesn't touch a spline face.
- **Two different splines sharing part of their length** are refused when their faces overlap there. Identical curves are fine, so two extrusions from one profile work.
- **A spline can touch a revolve axis only at its ends.**
- **An outline that crosses itself is found by sampling,** 16 pieces per knot interval and closer toward its ends, so a crossing much smaller than that could get through to the mesher, which
  refuses it there.
- **Volumes are integrated,** so `MassProps::volume_error` reports an estimate from comparing two rules, typically about 1e-12 relative. It isn't a proof, but tests hold it against known answers: an ellipse's prism, a torus, a sphere, a spheroid and a slanted cut.
- **Speed.** Each distance on one of these surfaces costs a search of about a microsecond, where an analytic one costs nanoseconds. Run natively, the spline demos take about 130 ms a rebuild, 230 ms for a full mesh of the vase, and 510 ms for the cam's mass properties, since its cross hole makes traced curves on spline faces. A slider's draft mesh skips mass properties. In the browser they're the slowest demos: see [Slider speed](#slider-speed).

## Cases OpenCASCADE handles and this kernel refuses

Found by an independent review harness and still open. `--example review named <file>` reruns them, and all four were refused the same way on 2026-11-08:

| Case | This kernel | OpenCASCADE |
|---|---|---|
| Cylinder r = 5.00001 over r = 5, coaxial: subtract the thin shell | Refused: "the result does not close" | Volume 0.003 |
| Cylinder r = 5 with its axis offset 1e-5: subtract the sliver | Refused: "the result does not close" | Volume 0.001 |
| Fillet an elliptical rim | Refused: "fillets and chamfers work on straight and circular edges only" | 7835.628 |
| Fillet the saddle edge of a tee pipe | Refused the same way | 20449.687 |

The first two are slivers near the tolerance, and a refusal there is the safe answer, but the
message should name the sliver instead of blaming closure. OpenCASCADE also reads free-form surfaces and
assemblies from STEP files, which this kernel refuses.

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
- Any edge touching a spline face.
- Variable radius.
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

**Random slider positions on the demos** (200 per model, `--example scan` in `crates/wasm`): no invalid bodies.

| Demo | Refused | Why |
|---|---|---|
| Bracket | 12 | The chamfer or fillet runs off its neighbour (8), or a chamfered edge no longer exists at those dimensions (4) |
| Mounting plate | 11 | A fillet ends up tangent along an intersection (5), the holes cross (3), or the fillet runs off its neighbour (2), and 1 more |
| Crossed cylinders | 7 | An exact tangency, such as a radius difference equal to the axis offset |
| Stress cases | 11 | Always the deliberate known-limit case |
| Flange, rounded block, sliding pockets, vase, cam, knob, ring sector | 0 | |

The vase was refused at 13 of 200 until this scan was rerun on 2026-11-08. That was a bug and not a limit: when a point lay exactly where the closest-point search halves
a span, the search missed it and reported the point as 32 mm from its own curve. The validity check refused every one, so no wrong shape came out. It's fixed, with a test. The fuzz
hadn't found it because its curves have several spans, while the vase's single span puts the check's sample exactly on the midpoint.

## Offset, shell and draft

These keep a body's topology and move its surfaces, so they are exact. OpenShape has asked for versions that survive a change of topology, which aren't started. Today:

- **A change big enough to alter the topology is refused,** such as an offset that makes a small face vanish or
  closes a hole. Commercial kernels restructure the neighbours. This one says the change is too big.
- **Outward offsets keep sharp edges sharp,** meeting further out. There are no rounded joins.
- **Faces that meet smoothly must move together.** Shelling with one of a smooth pair open is refused, as is
  drafting half of one.
- **Draft takes planes along the pull and cylinders on it.** Spheres, cones and tori aren't draftable, and neither is a
  smooth edge that isn't straight, so a fillet running around the top of a prism blocks drafting its walls.
- **Offset walls aren't checked against each other beyond the edges.** A wall that would pass through a distant
  part of the same body isn't caught by the offset itself. In a shell, the cavity's vertices must lie inside the part, and the
  Boolean's own checks apply. Other cases have little test coverage.

Checked in OpenCASCADE: `offset_out` and `offset_in` on a fully rounded box match its own `makeOffsetShape` volumes (12497.7143
and 5404.3606), and all the results read as valid solids.

## Sweeps

- **The path is lines and arcs,** and each step carries on from the last one's direction, so there are no corners, free curves or helices.
- **A bend tighter than the profile is wide is refused,** since the profile would cross the bend's center.
- **Each step is built separately and joined by a Boolean,** so a path that runs into itself gives the union of its pieces and isn't refused, and it can hit the Boolean's limits above.
- **Spline profiles along a path have no test of their own.**

## Many tools at once

Tools go in together if their bounding boxes don't touch, and are joined first if they do, one by one. So heavily overlapping tools
are no faster than before. 400 holes: 0.44 s at once, 7.9 s one by one, and 19.7 s in OpenCASCADE one by one. 1600 holes: 5.3 s at once. The one-by-one figures are from before the later speed work.

## Documents

- Each feature keeps one cached result, so switching back and forth between two states recomputes
  whatever changed each time.
- **A face name is stable but not unique.** No feature renames a face, but a name means every face that has it:
  a pattern or mirror copy has the same name as its original, even in a different plane; all the faces of one fillet or chamfer
  feature share a name; a whole circle's wall is two faces under one name; and a later cut leaves pieces under one name. Offset
  and draft keep one face per name, and a shell's inner walls are `f3:side.N` after the face they follow. A name disappears when its face does.
  [FILE_FORMAT.md](FILE_FORMAT.md) has the table and `crates/document/tests/face_names.rs` holds it to that.
- **A face key tells them apart,** such as `f2:end@f5.1` or `f3:side.1[f2:end|f2:side.0]`: see [FILE_FORMAT.md](FILE_FORMAT.md#face-keys).
- **Where keys stop being reliable.** A piece number, `#1`, goes by position, so it renumbers if pieces are added or removed or pass each other. At an exact coincidence, a cylinder's two halves can
  also swap for a different pair: a step away from the crossed-cylinders demo's exactly coaxial default turned them 90 degrees. A key whose face is gone is refused and doesn't move to another face, but a piece
  number can land on a different piece without any error. A face merged across a mirror plane answers only to the original's key.
- **`shell` and `draft` take plain names,** so they can't open or tilt the face of only one copy, and refuse a key rather than ignore part of it.
- **How well keys hold,** on all the demos, comparing every key before and after one step of a random slider: 2,294 of 2,364 stayed on the same kind of face in about the same place. Of the other 70, 40 were
  a bolt count changing, which moves the bolts, and 30 were the crossed cylinders leaving their exactly coaxial default, where faces vanish or change shape.
- Coordinates beyond 10 km are refused, because tolerances stop meaning anything there.
- A sketch can make at most 4000 regions, and a region can have at most 999 holes before names stop
  being unique. Constrained sketches name faces by entity, so they have no such limit.

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

## Slider speed

Measured in Chrome as WebAssembly on this desktop, for one step of a slider, in milliseconds. The two figures are the last and the first
slider, since the first usually affects more features. `python3 scripts/selftest_web.py` prints these.

| Model | Before | While dragging now | When a drag pauses now |
|---|---|---|---|
| Bracket | 22, 71 | 11, 31 | 18, 31 |
| Flange | 81, 119 | 33, 46 | 58, 83 |
| Rounded block | 178, 165 | 62, 53 | 90, 71 |
| Mounting plate | 188, 296 | 47, 93 | 87, 163 |
| Crossed cylinders | 73, 7 | 16, 5 | 33, 4 |
| Turned knob | 27, 31 | 7, 8 | 24, 25 |
| Ring sector | 88, 93 | 11, 10 | 38, 37 |
| Spline vase | | 179, 159 | 340, 371 |
| Spline cam | | 162, 148 | 719, 634 |

"Before" is before the speed work, and the rest were measured again on 2026-11-08. The spline demos came later, and they're the slowest: each distance on a spline surface costs a search. Most of the cam's pause is its mass properties.

While dragging, a slider gets a lighter mesh (about a fifth of the triangles on curved parts) and volume and area wait, shown dimmed. All of the
kernel's checks still run: skipping the slower ones was measured, and saved under 10%, so they stay on. About 140 ms after the slider stops,
the full mesh and the exact numbers follow. Saves and exports always use the finished result.

The kernel itself got two to four times faster on these models:

- **Choosing where to test a region** took about 40% of a rebuild. A quick search now settles most regions, and the old thorough search still runs
  if the quick points are too few or disagree, so it can't add a refusal. The same 258 fuzz cases are refused as before.
- **Blends** apply tools that don't touch together, as separate shells, instead of cutting a growing body again for each.

## Not tested

- **Safari in depth.** The page has been used on a real iPhone, where it ran and looked right. Nothing has measured it there, and the self test only runs in headless Chrome.
- **Phone performance.** By hand on the iPhone after the speed work, sliders felt fast, with the mounting plate the slowest and acceptable. That was before the spline demos, which are now
  the slowest by far, and they haven't been tried on a phone. The slider times above are from this desktop.
- **The hosted page itself, as far as this repository's checks go.** Nothing automated opens it. If the host's content policy
  blocks WebAssembly, the page shows an error saying it could not start.
- **Whether the hosted page allows workers.** The kernel runs in a worker so that a rebuild can't hold up the page. If a host forbids
  workers it runs in the page instead, with the same results and checks, and the page can stall during a rebuild. The self test runs both
  ways, but not on the host itself.
- **Running on macOS and Windows.** The kernel compiles for both, but tests have only run on Linux.
- **The download path on the hosted page.** Saving uses the host's download capability with the
  file wrapped in a zip, because `.step` and `.stl` aren't on its allowlist.
- **Other CAD systems.** STEP files have been written and read back with OpenCASCADE only. Files from other writers and large real-world parts haven't been tried, and the 22 import shapes are small test parts.
- **A real MCP client.** Only the test suite has connected to the server: see [MCP.md](MCP.md).
- **Splines in the sketch solver against PlaneGCS.** Lines, arcs, circles, ellipses and distances were compared, and splines only have the solver's own fuzz: see [SKETCH_SOLVER.md](SKETCH_SOLVER.md).
