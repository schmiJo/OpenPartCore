# Known limits and open defects

What the kernel refuses, what it only approximates, and what hasn't been tested. The sketch solver's limits and results are in
[SKETCH_SOLVER.md](SKETCH_SOLVER.md), and the MCP server's in [MCP.md](MCP.md). Every section was last compared with the code on 2026-11-09.

Numbers come from the runs described here. To replay any fuzz case:

```sh
cargo run --release -p coreshape --example replay -- <grid|rot|prism|gen|al|ext|rev|loft> <seed>
```

## What isn't built

These are missing, not failing. [PLAN.md](PLAN.md) has the order.

- **More kinds of loft.** No loft to a point, closed loop, guide curve or tangency at the ends: see [below](#free-form-surfaces-and-lofts).
- **Offset, shell and draft that change a body's topology.** They refuse instead: see [below](#offset-shell-and-draft).
- **Draft on spline and free-form faces.** Offset and shell work on those now, and draft is still refused.
- **Sheet bodies.** Everything here is a closed solid, so thicken takes faces of a solid, and a surface can't exist alone.
- **More kinds of blend.** No variable radius and no general corner blends, and fillets don't yet reshape a neighbouring face they'd run off: see [below](#fillets-and-chamfers).
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
| Lofts through mixed sections | 1,000 | 1 (0.1%) | 0 |

The refusals are "extrusion and torus meet in a loop too small to trace" (`ext 775`) and "plane and torus are tangent at a point on their
intersection" (`rev 115`). The loft's is "two edges claim the same continuation" (`loft 912`), an internal error near a tangency that should name it. About one generated loft in twelve is refused when it's built, as a fold or as sides that cross, and doesn't count as a case. An earlier run reported one wrong result, a mesh that wouldn't close on a seed-96 extrusion. The profile really did cross
itself, within the last sample before a corner, and outlines are now checked more closely toward corners, so that one is now refused.

**In OpenCASCADE,** all 63 parts of the gallery read back valid: 15 swept spline parts, 13 lofts and free-form parts, and the rest analytic. On the 15 spline parts, volumes agree to 3e-7 or better on 13. On `spline_outline` and `spline_drilled`, OpenCASCADE's
reported volume is 1.8e-4 and 1.6e-4 above this kernel's. Its own numbers settle this in the kernel's favour: for `spline_outline`, a polygon through 20,001 points
of OpenCASCADE's own edges gives a cap area of 812.6155, and this kernel's is 812.6155, while its `Area` for that face says 812.9296. A mesh of the solid made by OpenCASCADE
has volume 8126.09, against this kernel's 8126.16 and its reported 8127.58.

## STEP import

`coreshape::import::step::read_step`, or the `import_step` feature. It reads closed solids on planes, cylinders, cones, spheres, tori, curves swept along a line or around an axis, and rational B-spline surfaces, in millimetres, metres, inches and so on.

- **Refused, naming the surface and how many faces use it:** any other surface, such as `OFFSET_SURFACE`, so that no result has faces missing.
- **Refused, saying why:** a B-spline surface with a fold or of degree above 16, a face that runs across its surface's seam, or a closed one with no edges of its own at all, as OpenCASCADE writes a whole sphere or torus converted to B-splines. A closed one whose seam is on the face's own boundary is fine: it's cut into two faces along its parameter. A pole is fine too, where a whole side shrinks to a point.
- **Refused:** assemblies, since parts placed by transforms aren't read yet; open shells; any other edge curve, such as a hyperbola; a surface of revolution whose curve turns too far for one face, which an extrusion's curve can do because it gets cut up.
- **Faces change shape on the way in.** This kernel's edges have two different ends and its curved faces stay within half a turn, so a seamed full cylinder comes in as two halves under one name, and so on. Volumes and areas don't change.
- **Traced edges.** A B-spline edge between two surfaces, which is how other systems store an intersection curve, gets resampled onto both surfaces, so it follows them to this kernel's tolerance and not the file's. That's why `tee` and `cross_drilled` differ from OpenCASCADE by about 1e-6 relative.
- **A file's flags aren't trusted.** Writers get these wrong in practice: `FACE_BOUND` senses that OpenCASCADE's own reader repairs by looking at the shape, an edge's sense against its line when the two ends say otherwise, and a hole listed before a face's outer loop. So this works out each loop's direction from its neighbours, lets simple faces vote on which way round the group goes, takes a line's direction from its two ends, and puts the largest loop first.
- **Units are the ones the geometry is assigned,** through its representation's context. An earlier version took the first in the file, and read every inch part as millimetres.
- **A file's gaps are healed or stated, within a limit.** Vertices move onto the surfaces around them, by up to a thousandth of the part and never more than 0.25 mm. Where surfaces cross, edges get traced onto both exactly. Where they're tangent, as around a fillet, nothing can bring them together, so the file's curve stays. The widest gap left over becomes `Body::tolerance`, which that body's validity is held to in place of the kernel's own 5e-7, and which Booleans on it inherit. So a body read this way is not as exact as one built here, and a gap can make a later Boolean refuse: of the parts below, 1 of 30 refused a box cut through it. Past the limit, a file is refused.

**Checked on real parts** with NIST's public MBE PMI test files, 33 STEP files of 11 parts from CATIA, NX, Creo, SolidWorks and others, up to 671 faces. `NIST-PMI-STEP-Files.zip` is from nist.gov, and stays out of this repository: `scripts/step_import/check_nist.sh <folder>` runs on a folder of those files.

- **30 of 33 read.** The refusals are one file holding no solid at all, which is right, one with an edge 1.4 mm off its faces, and one cone that can't be cut into half turns.
- **25 of those 30 match OpenCASCADE's reading of the same file** to 1e-3 in volume and area. Most are far closer. In the other 5, OpenCASCADE's own reading disagrees with itself: on CTC 02 it gives 47.85M, 47.10M and 45.47M from three files of one part, where this reads 47.10M, 47.10M and 47.11M, and on STC 08 it gives 388,155 against 503,592 from its own reading of the sibling file, which is what this reads.
- **The first run read 5 of 33,** and quietly read every inch part at the wrong scale. Those bugs are fixed, with tests, but it says what to expect from a set this hasn't met yet.

Checked further by `scripts/check_step_import.sh`, which has OpenCASCADE write 31 shapes, then reads each, compares volume and area, meshes it, cuts a box out of it, writes it back, and has OpenCASCADE check the result. 29 are read, watertight, and valid in OpenCASCADE afterwards. Most match to about 1e-13. Three flagged ones are B-spline conversions of a cylinder, cone and frustum, where this reads 4021.2385965950 for a true 4021.2385965949, and OpenCASCADE's own reported volume is 4058.05. The two refused are a sphere and a torus as B-splines with no edges at all. 3,000 randomly damaged files were each read as a valid solid or refused, with no panics, and everything this kernel writes comes back the same.

## Swept spline surfaces

Curves swept along a line or around an axis: see [NURBS_DESIGN.md](NURBS_DESIGN.md).

- **Refused by name:** draft on these faces. Offset and shell turn the face into the free-form surface it exactly is, moved: see [below](#offset-shell-and-draft). Fillets and chamfers are fitted: see [below](#fillets-and-chamfers).
- **Two different splines sharing part of their length** are refused when their faces overlap there. Identical curves are fine, so two extrusions from one profile work.
- **A spline can touch a revolve axis only at its ends.**
- **An outline that crosses itself is found by sampling,** 16 pieces per knot interval and closer toward its ends, so a crossing much smaller than that could get through to the mesher, which
  refuses it there.
- **Volumes are integrated,** so `MassProps::volume_error` reports an estimate from comparing two rules, typically about 1e-12 relative. It isn't a proof, but tests hold it against known answers: an ellipse's prism, a torus, a sphere, a spheroid and a slanted cut.
- **Speed.** Each distance on one of these surfaces costs a search of about a microsecond, where an analytic one costs nanoseconds. Run natively, the spline demos take about 130 ms a rebuild, 230 ms for a full mesh of the vase, and 510 ms for the cam's mass properties, since its cross hole makes traced curves on spline faces. A slider's draft mesh skips mass properties. In the browser they're the slowest demos: see [Slider speed](#slider-speed).

## Free-form surfaces and lofts

Rational B-spline patches, from a loft or a file: see [NURBS_DESIGN.md](NURBS_DESIGN.md#phase-3-free-form-patches).

- **Refused by name:** draft on these faces, as on swept splines. A pole or a fold, found by sampling each knot interval at 169 points for a normal that vanishes or turns over between neighbours, so one much smaller than that could get through.
- **Two surfaces lying on each other are refused unless they're the same patch.** A patch that's exactly a cylinder, against a real cylinder of that size, is refused as a tangency, and nothing guesses that they're the same.
- **The proof has edges.** The closest point is proven for every point of the patch. Past its boundary the surface carries on along its tangent planes, where the value is only a height, and that's only used near faces' edges. Far from a surface, if a great many points are almost equally close, a search can run out of budget and report a smaller distance it can prove. A patch's tightest bend, which sets how fine a trace is, is sampled and not proven, so a very small feature between samples could be stepped over. The checks on every result still apply.
- **Lofts take one outline per section, with the same number of holes,** and they can't end in a point, close into a ring, follow guide curves or set tangency at the ends.
- **Sections are matched by counting.** Each outline's pieces are matched in order, with the longest cut in half until the counts agree, starting from the rotation that moves them least. So a rectangle to a circle twists by up to 45 degrees, as the nozzle demo does, and you steer it by turning a sketch or adding corners to it.
- **A loft through itself is refused:** one section through another, or sides that cross, by looking at 23 stages along it at 12 points per piece. A crossing much smaller than that could get through. It only looks along the direction of travel, so a loft that turns back on itself sharply may be refused when it needn't be.
- **Volumes are integrated,** so `MassProps::volume_error` reports an estimate from comparing two rules, typically about 1e-10 relative. Tests hold it against known answers: a true cylinder, frustum and ring, a tilted cut and a revolved curve.

**Checked by comparing with real surfaces.** A loft between circles is exactly a cylinder or cone, written as rational splines. 24 random solids of that kind, each with a random box, sphere, cylinder or torus through all three Booleans, have to match the analytic version to 1e-5 relative, and they do, as does the fixed set in `crates/kernel/tests/m11_loft.rs`. Closest points are checked against brute force, and a torus written as a patch has a torus's distance to 1e-9. The fuzz above found two real bugs on the way: a face's bounding box missed its bulge past its edges, which gave wrong answers and refusals, and a Gauss rule ran across a traced curve's crossing of a knot line, which left volume identities off by about 1e-4 until the rule stopped there, bringing them to about 1e-11.

**In OpenCASCADE,** all 13 lofts and Booleans on them read back valid. 10 agree on volume to 7e-6 or better. On `loft_cylinder`, `loft_frustum` and `loft_ring`, OpenCASCADE's reported volume is 2e-3 above this kernel's, and its circle's area is off too: 314.117 for a true 314.159. These three have exact answers, which this kernel matches to 1e-9, and a fine mesh of OpenCASCADE's own shape goes to the same place, 9424.71 on a true 9424.78. It's the same effect as on `spline_outline` above. A first version was wrong in the other direction: OpenCASCADE read one trimmed sphere face as the rest of the sphere, by repairing a loop around the sphere's pole, so the STEP writer now puts poles and seams off a face where it can.

**Speed is the weak point.** A distance takes about 3 microseconds on the surface and 10 near it, where a swept spline's takes one and an analytic one takes nanoseconds, and that's after three rounds of work on it. Booleans on two lofts took about 1.5 s natively in the fuzz, with a similar time for their mass properties, while the simpler nozzle demo takes under 200 ms a rebuild. Each knot interval that gets touched also keeps about 170 kB, or 370 kB if it's rational and cubic, so a surface with thousands of them would be a problem. That hasn't been tried.

## Cases OpenCASCADE handles and this kernel refuses

Found by an independent review harness and still open. `--example review named <file>` reruns them. The two it also found on fillets, an elliptical rim and a tee's saddle, now work: see [below](#fillets-and-chamfers).

| Case | This kernel | OpenCASCADE |
|---|---|---|
| Cylinder r = 5.00001 over r = 5, coaxial: subtract the thin shell | Refused: "the result does not close" | Volume 0.003 |
| Cylinder r = 5 with its axis offset 1e-5: subtract the sliver | Refused: "the result does not close" | Volume 0.001 |

These are slivers near the tolerance, and a refusal there is the safe answer, but the
message should name the sliver instead of blaming closure. OpenCASCADE also reads surfaces with poles and
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

Supported, but fitted and not exact, which makes them the only results in this kernel that are neither exact nor refused:

- Traced intersection curves, ellipses, and any edge on a spline or free-form face, such as the rim of a hole through a cylinder or a tee's saddle.
- The surface is a cubic through exact sections of the true rolling-ball surface, with stations added until its two long sides are within 1.5e-7 mm of the faces they lie on. Between stations its middle is a fit too, and isn't measured directly.

Not supported, returning an error:

- Variable radius.
- A radius that runs off a neighbouring face. Real kernels can absorb the overrun and restructure
  the neighbours. This one refuses.
- A fitted blend passing a vertex where a sharp edge meets it, or that ends where more than one face lies beyond it. Those need corner blends.
- A fitted blend's end at a reflex corner, where its side would end past the neighbouring edge, which takes reshaping that neighbour.
- A surface so kinked that the fit doesn't reach its tolerance within 1,024 stations.

**How fitted blends were checked.** On edges an exact fillet also handles, a straight edge, a concave one and whole rims, the fitted route has to match it, and does to 1e-7 relative. On an elliptical rim the solid is convex, so a fillet is by definition what's within the radius of the solid shrunk by the radius, and that volume can be worked out with none of the kernel: at radius 2 it removes 71.8840 and heading for about 71.8845 as that sum gets finer, this kernel removes 71.8850, and OpenCASCADE's fillet removes 71.768. Its fillets on a tee and a drilled wall are within 0.1 of these in added or removed volume, where its Booleans alone differ by about that much. All 6 fitted parts read back valid in OpenCASCADE, and 5 of them agree on volume to 4e-8 or better. The other, on a spline, is 1.1e-4 off, the same effect as on `spline_outline`.

**Fuzz** (`--example fuzz_blends`), on random curved edges of random Booleans: of 300 seeds, 210 had an edge to try, 80 blended and 130 were refused, and 0 were wrong by validity, a closed mesh, a believable change of volume, membership away from the edges, and surviving a later Boolean. Most refusals were the two corner cases above, 36 each. 9 didn't reach the fit, and 25 didn't fit at all at a radius up to 3% of the part. So a closed rim or an edge between two simple ends generally works, and an edge ending in a complicated corner often won't.

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
| Flange, rounded block, sliding pockets, vase, nozzle, cam, knob, ring sector | 0 | |

The vase was refused at 13 of 200 until this scan was rerun on 2026-11-08. That was a bug and not a limit: when a point lay exactly where the closest-point search halves
a span, the search missed it and reported the point as 32 mm from its own curve. The validity check refused every one, so no wrong shape came out. It's fixed, with a test. The fuzz
hadn't found it because its curves have several spans, while the vase's single span puts the check's sample exactly on the midpoint.

## Offset, shell and draft

These keep a body's topology and move its surfaces, so they are exact. OpenShape has asked for versions that survive a change of topology, which aren't started. Today:

- **A free-form or swept spline face moves exactly.** Its offset isn't a spline, so it isn't stored as one: the surface is the patch plus a distance, and the distance to it is the distance to the patch less that one. Checked on a vase, by a one-dimensional integral from its own profile, at offsets from 0.25 to 3: 6152.276608 against 6152.276661 at 0.75, and 12447.1833 against 12447.1844 at 3. On a spline sphere, 5575.279763 against 5575.279763.
- **Past the end of a free-form face, a moved surface carries on along its tangent.** An outward offset of a part with caps needs that, as a side has to reach a cap that has moved away from it, and the true offset of the side isn't defined there. The extended part is a ruled surface and not a continuation of the curvature.
- **Refused on a free-form surface:** a move past a center of curvature, found by sampling, where it would fold; a surface with a sharp crease at a knot; and a pole with a point to it, like the tip of a cone, as each of those would need a gap filled. A smooth pole is fine.
- **The never-overstating promise is weaker on a moved surface.** It holds as long as the moved surface hasn't come back near its own unmoved one, which is what a fold or a thin neck does. A search out of budget reports where it got to, rather than a lower bound, as the difference between a lower bound and the offset would look like a real gap.
- **In a STEP file a moved free-form face is a fit, within 1e-5.** STEP has `OFFSET_SURFACE`, which this kernel reads, but OpenCASCADE refuses one on a surface that isn't C1 ("Offset with no C1 Surface"), and a circle's double knots aren't. So it's a B-spline of up to degree 9 on few pieces. A two-face vase goes from 10 KB to 185 KB, and reading that back here takes about 5 s, as closest points on degree 9 are slow to set up. The kernel's own copy stays exact.
- **OpenCASCADE disagrees about thin moved solids.** On `thick_vase` it gets a volume of 617.12, and on `vase_hollow` 843.71. Both differ by about 1%, and the same one-dimensional integral gives 623.19315 and 836.33638, which is what this kernel gets. The gallery flags these two, with the three lofts it has flagged before, for the same reason.

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

## Helix

A profile carried by a screw motion: a spring, a thread, or with the section across the axis, a twisted bar.

- **Stated tolerance, not exact.** Each section is exact, as a section is carried rigidly. Between them the true surface has no rational form, so each control point follows a quintic matching its true position and first two derivatives at both ends of a piece, with enough pieces to keep the bound `r h^6 / 46080` under 1e-7. Measured at radii from 1 to 1000: 8.7e-8 to 9.5e-8 from the true helix.
- **Neighbouring faces share their control points,** so the fit can't open a gap between them.
- **A side is one face per half turn, counted from the start,** so more turns leave the earlier faces where they were. Each half turn has a key of its own, as `f4:side.0@f4.3`, and those are kept. What a thread cuts from a shaft isn't so lucky: the wall is many pieces under one name, numbered by position, so a slider move can change which is `#2`. Name a piece of a cut-up wall only if nothing moves it.
- **Checks that the turns clear each other,** by following 8 points per curve piece around their helixes to see whether any lands back inside the section, at 64 steps a turn. A section touching itself exactly counts as a collision. A narrower collision than those samples could get through.
- **Refused by name:** a section touching or crossing the axis while holding it, a section lying along the path, a pitch of zero, and over 200 turns.
- **A thread on a shaft of the same radius is a tangency,** as a helical face isn't exactly on the cylinder, so a ridge has to bury itself in the shaft, as the tests' does, and a groove has to reach past its surface.
- **Fuzz:** 300 random helixes (`--example fuzz_features -- 300 helix`) against the closed-form volume: 259 right, 0 refused, 0 wrong, and 41 skipped as too tall for their pitch, where a refusal is right.
- **OpenCASCADE:** the square spring's volume and area match it to 1.5e-11, the twisted bar's to 1.7e-10, and the thread to 9.6e-7. On round wire it differs by 7.6e-5 at 2131.67, where the closed form is 2131.8345, which is also what this kernel gets.

## Delete face

- **Takes:** holes, pockets, bosses and dimples, as islands whose whole border is inner loops of their neighbours. A void or one of several separate pieces. Fillets, chamfers and rabbets, as strips, including chains around a closed rim and corner patches where three chains meet. Corner notches and cut corners, with three sides. Slots across a face, with one plane on both sides.
- **Oracle:** taking away what a feature added has to give the part as it was, in volume and area. All 11 test groups do that, including fitted fillets on an elliptical rim and a free-form rim. Every combination of the 4 top edges of a block, with fillets and chamfers, comes back to exactly its volume.
- **Faces only grow,** along surfaces that are already there, so nothing new is approximate unless the two faces that now meet have no exact intersection.
- **Refused by name:** any other shape of gap. The commonest in practice is **a slot or boss cut through an earlier fillet,** whose gap has more sides than this handles. For example, `subtract(fillet(block, edge), slot_box)`, and delete the slot's faces.
- **Half a surface is refused** when what's left of it can't close, such as one half of a hole's wall. The hole has to go whole.
- **It doesn't check the faces that grew against the rest of the body.** Extending a face far enough to run into a distant part of the same solid isn't caught, except for a vertex that would land over three times the part's size away.
- **Fuzz:** 300 random parts with a random feature (`--example fuzz_features -- 300 delete`): 276 given back exactly, 16 refused, 0 wrong and 8 skipped. All 16 refusals are the case above.

## Thicken

A solid between faces of a body and a copy of them moved along their normals, with walls along those normals.

- **Faces of a solid only,** as there are no sheet bodies. Several faces have to meet smoothly. Faces meeting at a sharp edge are refused, and can be thickened one at a time.
- **Far faces are exact on every surface type,** including free-form ones, as those are a surface moved by a distance.
- **Walls are exact where they have a form of their own:** a plane from a straight edge whose normal doesn't change, and a plane, cylinder or cone from a round edge whose normal keeps its angle to the axis. Elsewhere they're ruled patches through the true normals, fitted to within 1.5e-7, with up to 1024 stations.
- **Oracles, all met to 1e-9 or better:** a flat face's area times the thickness, tube walls, spherical and toroidal shells, and a free-form cylinder against a true one. A thin layer on a free-form vase settles to its area. Steiner's formula checks a cone, and a 1D integral gives 623.19315 for the vase at 0.5, which is what comes out.
- **Fuzz:** 300 random faces (`--example fuzz_features -- 300 thicken`): 296 fine, 4 refused and 0 wrong, the refusals being an edge whose radius would go to nothing inward. Those are checked for validity and against their own mesh, since no closed form exists for most.

## Sweeps

- **The path is lines and arcs,** and each step carries on from the last one's direction, so there are no corners or free curves. A helix is its own feature: see [above](#helix).
- **A bend tighter than the profile is wide is refused,** since the profile would cross the bend's center.
- **Each step is built separately and joined by a Boolean,** so a path that runs into itself gives the union of its pieces and isn't refused, and it can hit the Boolean's limits above.
- **Spline profiles along a path have no test of their own.** Free-form sweeps along a free path aren't built.

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
| Bracket | 22, 71 | 15, 32 | 16, 48 |
| Flange | 81, 119 | 29, 28 | 34, 50 |
| Rounded block | 178, 165 | 75, 68 | 121, 77 |
| Mounting plate | 188, 296 | 34, 100 | 59, 140 |
| Crossed cylinders | 73, 7 | 11, 4 | 23, 4 |
| Turned knob | 27, 31 | 3, 3 | 13, 12 |
| Ring sector | 88, 93 | 4, 6 | 31, 22 |
| Spline vase |  | 134, 102 | 195, 215 |
| Free-form nozzle |  | 202, 227 | 434, 381 |
| Spline cam |  | 163, 158 | 739, 621 |

"Before" is before the speed work, and the rest were measured again on 2026-11-08. The spline and free-form demos came later, and they're the slowest: each distance on one of those surfaces costs a search. Most of the cam's pause is its mass properties.

**How many triangles the page sends,** as these demos stand at their default sizes:

| Model | For an STL | On screen | While dragging |
|---|---|---|---|
| Flange | 5,688 | 1,356 | 960 |
| Mounting plate | 11,610 | 3,028 | 1,188 |
| Turned knob | 5,504 | 2,018 | 654 |
| Ring sector | 8,796 | 2,712 | 948 |
| Spline vase | 9,336 | 3,912 | 972 |
| Free-form nozzle | 9,124 | 5,106 | 2,566 |
| Spline cam | 9,610 | 6,014 | 2,428 |

While dragging, a slider gets a light mesh, where a circle has eight sides, and volume and area wait, shown dimmed. All of the
kernel's checks still run: skipping the slower ones was measured, and saved under 10%, so they stay on. About 140 ms after the slider stops,
the full mesh and the exact numbers follow. Saves and exports always use the finished result. What's shown isn't what's exported: the screen's mesh is within a thousandth of the part's size, which is under half a pixel on a phone, and an STL is within 4e-4 of it, which takes two to four times the triangles.

The kernel itself got two to four times faster on these models:

- **Choosing where to test a region** took about 40% of a rebuild. A quick search now settles most regions, and the old thorough search still runs
  if the quick points are too few or disagree, so it can't add a refusal. The same 258 fuzz cases are refused as before.
- **Blends** apply tools that don't touch together, as separate shells, instead of cutting a growing body again for each.

## Not tested

- **Safari in depth.** The page has been used on a real iPhone, where it ran and looked right. Nothing has measured it there, and the self test only runs in headless Chrome.
- **Phone performance.** By hand on the iPhone after the speed work, sliders felt fast, with the mounting plate the slowest and acceptable. That was before the spline and free-form demos, which are now
  the slowest by far, and they haven't been tried on a phone. The slider times above are from this desktop.
- **The hosted page itself, as far as this repository's checks go.** Nothing automated opens it. If the host's content policy
  blocks WebAssembly, the page shows an error saying it could not start.
- **Whether the hosted page allows workers.** The kernel runs in a worker so that a rebuild can't hold up the page. If a host forbids
  workers it runs in the page instead, with the same results and checks, and the page can stall during a rebuild. The self test runs both
  ways, but not on the host itself.
- **Running on macOS and Windows.** The kernel compiles for both, but tests have only run on Linux.
- **The download path on the hosted page.** Saving uses the host's download capability with the
  file wrapped in a zip, because `.step` and `.stl` aren't on its allowlist.
- **Other CAD systems.** Files this kernel writes have only been read back in OpenCASCADE. Files from other systems have only been NIST's 33 test parts, which are real but few, and their free-form surfaces are a small share. Expect the next set to find problems these haven't. CAx-IF's benchmark files haven't been tried, and they're the obvious next test.
- **A real MCP client.** Only the test suite has connected to the server: see [MCP.md](MCP.md).
- **Splines in the sketch solver against PlaneGCS.** Lines, arcs, circles, ellipses and distances were compared, and splines only have the solver's own fuzz: see [SKETCH_SOLVER.md](SKETCH_SOLVER.md).
