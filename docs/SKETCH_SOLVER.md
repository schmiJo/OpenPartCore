# The sketch solver

`crates/sketch` is a 2D constraint solver with no dependencies, which builds for WebAssembly as it
is. It knows nothing about the kernel. `crates/document` joins the two.

## What it does

A sketch is plain data: entities and constraints with permanent ids, written to and read from JSON
(see [FILE_FORMAT.md](FILE_FORMAT.md)).

| Call | Result |
|---|---|
| `solve` | A new sketch, with remaining freedom, what can still move, and what's redundant |
| `analyze` | The same report, at the geometry as it stands |
| `drag`, `DragSession` | Moves what was grabbed and disturbs the rest as little as possible |
| `arrangement` | Every closed region the curves make, split at crossings and T-junctions, each with a stable id, plus the curves that make none and why. It never refuses. |
| `regions` | The regions to build from: those picked with `pick_regions`, or by default every other level of nesting. These become kernel profiles through `profiles_from_regions`. |

Solving returns a new sketch and never changes the one it was given, so a failure can't leave anything
half done.

## Distances and reference dimensions

`distance` works between any two of a point, a line and a circle or arc:

| Between | Measures | Which side |
|---|---|---|
| Point and circle or arc | The gap to the curve. An arc counts as its full circle | The side the point is on now, unless the circle is too small for that |
| Line and circle or arc | The gap from the line to the curve. If the line cuts the circle, it's to the side of the curve nearer the line | The same |
| Two circles or arcs | The gap between the curves, outside each other or one inside the other | As they are now. Circles crossing each other go to outside each other |
| Two lines | From the middle of `a` to `b`, with no point to pick | Either |

Two lines measured this way are the distance between them when they're parallel. The dimension doesn't make them parallel, so
add a `parallel` too. Without one, it means just what it says: the middle of `a` to the line through `b`.

**Reference dimensions** report what the geometry measures and take no part in solving. They use no freedom, can't conflict with
anything, and aren't listed as redundant. `add_reference(rule)` adds one, with any value in the rule, and `set_reference` turns a
dimension into one or back. Solving and dragging write the measured value into it, so a saved or returned sketch shows it.
`measure(id)` and `reference_values()` read any dimension at the geometry as it stands. In JSON it's `"reference": true`, and the
`value` may be left out.

**Tested** by 5,000 random sketches of these forms in this solver's own fuzz, with no refusals or wrong answers, and 1,000 against
PlaneGCS. PlaneGCS accepted every answer without moving it, with the same freedom on all, and no wrong answers on either side. It
refused 16 that this solver solved. In 16 of 1,000 drags this solver stopped short where PlaneGCS got there, by at most 0.18 mm and
with every constraint still holding, which is a higher rate than the other categories' and isn't explained yet. FreeCAD has no form
of a distance between two lines without picking a point, so in that comparison PlaneGCS measures from one line's start point.
That's the same thing only because those lines are also constrained parallel. Measuring is checked directly against worked values.

## Ellipses

An ellipse is five lengths: its center, where its major axis ends, and its minor radius. An elliptical arc adds its two ends and an
equation to keep each on the ellipse, as an arc does.

| Works with | How |
|---|---|
| `coincident` | A point on it, or its `center`, `major`, `start` or `end` with other points |
| `horizontal`, `vertical`, `distance`, `fix` and the rest | On `7.center` and `7.major`, which are points like any other |
| `major_radius`, `minor_radius` | Dimensions, which can be references too |
| `tangent` | With a line, anywhere. With a line, arc, circle or another ellipse at a point they share |
| `concentric`, `equal` | With circles and arcs, and between ellipses |

An ellipse has no closed-form distance, so "on the ellipse" is the level set in first-order millimetres. It's exact at the solution, and the
independent check measures the true distance by a plain scan. These equations get their gradients from numbers that carry them along
(`dual.rs`), which is exact, and the gradient test runs them against differences like every other equation.

**In regions** ellipses are cut like anything else. Where an ellipse meets a curve there's no closed form used: pieces that come close get Newton's
method on both parameters, in a damped form that still settles where they only touch.

**Into the kernel** an ellipse is exactly a rational spline, so `profiles_from_regions` produces spline segments, which sweep into spline surfaces.

**Tested** by 5,000 random sketches in this solver's own fuzz with no refusals or wrong answers, and 300 against PlaneGCS. PlaneGCS accepted all 294 it could
build without moving them, with the same freedom on all and no wrong answers on either side. The other 6 raised an error when building the
FreeCAD geometry. In 19 of 261 drags this solver stopped short where PlaneGCS got there, by at most 0.32 mm and with every constraint holding, which is
more often than elsewhere and isn't explained yet. PlaneGCS's own drags broke a constraint in 8 of them. Regions with ellipses are tested against the raster below.

## Splines

A spline is its control points, 2 lengths each, with its weights and knots staying as they are. It keeps the sketch's rule that every variable is a length, so
"move as little as possible" still means something.

| Works with | How |
|---|---|
| `7.start`, `7.end`, `7.p2` | Points like any other, for `coincident`, `distance`, `horizontal`, `fix` and the rest. A clamped spline passes through its first and last |
| `tangent` | At an end it shares with a line, arc, circle, ellipse or spline. That's along its first or last control leg, whatever its weights, so the leg acts as a line in the equation |
| `fix` | The whole thing, with all its control points in order |
| `drag` | By any control point. By the edge, it moves the whole thing |

`SplineShape::through(points)`, or `"through"` in a file, makes the smooth cubic through some points by chord length. Only the control points stay in the sketch, so moving one
later doesn't keep the curve on those points.

**In regions** a spline is cut at its crossings with anything, and with itself, and these checks found two real bugs on the way to passing: a curve crossing itself
wasn't cut at all, and where two strands of one curve pass close together, the search for a contact could land on the wrong strand and lose it.

**Into the kernel,** each piece of a spline in a region becomes a kernel spline, and sweeps into `Surface::Extruded` or `Surface::Revolved`. Faces keep their entity's name.

**Tested** by 5,000 random sketches in this solver's own fuzz with no refusals or wrong answers, including rational ones, and against the raster on 1,113 random sketches with
9,302 pockets and 6,858 regions, with no disagreements.

## Regions

`arrangement` cuts every curve that isn't construction geometry at each crossing, T-junction and touching point, then
walks the faces: at each junction it leaves by the next piece clockwise from the one it arrived on. A walk running
counter-clockwise goes round one region, and one running clockwise goes round the outside of a connected group, which is
a hole in whatever region surrounds it.

| Result | Meaning |
|---|---|
| `regions[].id` | Permanent name, `r` and twelve hex digits |
| `regions[].depth` | How many connected groups surround this one. Even is material by default and odd is a hole |
| `regions[].outer`, `.holes` | Loops ready for `profiles_from_regions` |
| `ignored[].entity`, `.whole`, `.pieces` | A curve, or the pieces of one, that bound no region |
| `ignored[].why` | `FreeEnd { at }`, `Bridge`, `Overlap { with }`, `NoLength` or `NoArea` |

**Ids** come from the curves around a region's outside and the direction each is walked in. So they survive solving,
dragging, dimension changes, reordering and edits elsewhere. A new hole doesn't rename the region around it, and neither
does a spur added from outside. Splitting a region gives its parts new ids. Lobes with the same curves around them are told
apart by their order along the first curve, which could shift if a change carries a lobe past where a whole circle starts.

**Picking.** `region_at(point)` finds what was clicked. `pick_regions(&[id])` stores the choice in the sketch, and it's saved
as `picked_regions`. If a picked region stops existing, `regions` refuses and lists what's there, so a part can't silently
turn into something else. Regions sharing an edge and both in use merge into one solid when extruded.

**Tested** against a raster that shares nothing with the walk: draw the curves on a 0.1 mm grid, flood-fill from outside, and
every pocket left must lie in a region, with the pockets in a region adding up to its area. On 1,113 random sketches of lines,
arcs, circles, ellipses, elliptical arcs and splines, with 9,302 pockets and 6,858 regions, there were no disagreements. 36 pockets were reported at first only
because walls a cell wide closed real gaps, and a finer grid confirmed each of those. 1,387 further random sketches were left out of this comparison
because an end stops less than 0.4 mm from another curve, which a raster can't tell from touching. Those gaps have their own worked tests.

**Speed.** 202 lines crossing into 10,000 regions takes about 0.7 s in a debug build. It's quadratic in the number of curves
where their boxes overlap.

## How it works

**Variables.** Every variable is a length: coordinates and radii, never angles. So "move as little as
possible" has one meaning across the sketch. An arc is a center, a radius and two end points, with both
ends held on the circle. Its 7 variables and 2 equations leave the 5 freedoms an arc has. A later ellipse
or spline fits the same pattern, as more points and lengths.

**Equations.** Each constraint becomes one or two equations whose value is a length: millimetres of
error at the size of the geometry involved. One tolerance, 1e-9 mm, then applies everywhere, well inside
the kernel's 1e-7. Every gradient is checked against finite differences in the tests.

**Parts.** Equations that share no variable are solved separately, so editing one corner of a large
sketch leaves unrelated geometry identical to the bit.

**Each step** is the smallest move that meets the linearized equations, with damping when a step makes
things worse. The linear system is `[W Jᵀ; J -Λ]`, factored by a sparse LDLᵀ. Using this and not `J Jᵀ`
matters when one line is used by hundreds of constraints: its column would make `J Jᵀ` completely dense,
while here it's simply eliminated last. Radii get costlier to change as they shrink, so one that isn't
truly forced to zero never reaches it.

**When a solve stalls,** it tries following a path, shrinking the errors in stages, and then disturbing
only the part that's stuck. In a large sketch Newton solves almost everything and sticks on one block, so
the rest stays where it landed.

**Dragging** keeps constraints hard and the pointer soft. So the grabbed thing stops at the nearest place
it can reach if the pointer goes further, and the sketch stays valid. Disturbance is measured from where
the drag began, not from the last frame, so nothing creeps: bring the pointer back and the sketch comes
back. Each step is followed by a pull back onto the constraints, with a line search, since a strong pull
on a curved constraint otherwise overshoots.

**Redundancy and conflict** come from eliminating the equations in order, so the constraint added last is
the one called redundant, along with those that imply it. A failure is shrunk by removing constraints one
at a time, leaving a smallest set: removing any one of them resolves it. The words depend on the evidence:

| Message | Means |
|---|---|
| "these constraints can't all hold" | Proven: linear equations that don't fit each other |
| "these constraints conflict: no arrangement ... was found" | Nothing met them from here or from 24 scattered starts. Strong evidence, not proof. |
| "no solution was found near this position, though one exists elsewhere" | A start elsewhere works, so this is a failure to get there |
| "these constraints can only be met by collapsing entity N" | They hold only if a line or radius goes to nothing, which is refused |

## Checked against FreeCAD's PlaneGCS

Random sketches are built from an exact true arrangement, so a solution is known to exist, and then
nudged by up to 5 mm for each solver to find its way back. That's a lot on parts a few millimetres across.
Two solvers can land on different valid solutions, so positions aren't compared. Instead each answer is
given to the other: an answer is right if the other accepts it without moving it.

To reproduce:

```sh
cargo run --release -p coreshape-sketch --example oracle -- export <dir> 1000 20000
scripts/sketch_oracle/run.sh <dir>
cargo run --release -p coreshape-sketch --example oracle -- compare <dir>
```

Results on 11,050 sketches this solver had not been tuned on (seeds from 20000). A first run on seeds from
0 gave the same picture.

| Category | Cases | Both solved | PlaneGCS accepted my answer unmoved | Freedom differs | I refused, it solved | It refused, I solved | Neither | Wrong |
|---|---|---|---|---|---|---|---|---|
| Rectilinear outlines | 1000 | 705 | 1000 | 0 | 0 | 295 | 0 | 0 |
| Polygons | 1000 | 995 | 1000 | 0 | 0 | 5 | 0 | 0 |
| Rectangles at any angle | 1000 | 1000 | 1000 | 0 | 0 | 0 | 0 | 0 |
| Slots | 1000 | 999 | 1000 | 0 | 0 | 1 | 0 | 0 |
| Rounded outlines | 1000 | 996 | 1000 | 0 | 0 | 4 | 0 | 0 |
| Circles, tangents, normals | 1000 | 998 | 1000 | 0 | 0 | 2 | 0 | 0 |
| Symmetry and midpoints | 1000 | 1000 | 1000 | 0 | 0 | 0 | 0 | 0 |
| Several blocks together | 1000 | 645 | 1000 | 0 | 0 | 355 | 0 | 0 |
| Large, about 130 to 290 entities | 50 | 6 | 50 | 0 | 0 | 44 | 0 | 0 |
| An implied constraint added | 1000 | 486 | 1000 | 0 | 0 | 514 | 0 | 0 |
| Two constraints in conflict | 1000 | 0 | n/a | n/a | 0 | 0 | 1000 | 0 |
| A triangle that can't close | 1000 | 0 | n/a | n/a | 0 | 0 | 1000 | 0 |

- **"It refused" is mostly a policy, not a failure.** In the first run I looked at the status codes: every
  PlaneGCS refusal in the rectilinear, blocks and implied categories named a redundant constraint. It stops
  on an unsolved sketch with redundancy, where this solver solves it and names the redundancy. Its true
  failures to converge were the 12 in polygons, slots, rounded and circles.
- **PlaneGCS answers that fail:** in each run it reported success 10 times with a wrong result: 9 by
  collapsing geometry (a slot's line to a point, for example) and 1 that fails the constraints.
- **Naming a conflict.** On conflicts it named constraints in 837 of 1000. On impossible triangles it named
  none. This solver named a smallest set for all 2000, and its own fuzz checks that removing any one of them
  resolves it.
- **Redundancy:** the two never disagreed on whether there was any. They name different constraints in about
  1% of cases, which is a free choice, and a `fix` is two equations in PlaneGCS.
- **Not compared:** 640 cases had a normal between two curves with no shared point, which FreeCAD has no form
  of. PlaneGCS was given those without that one constraint, and this solver's answers for them are checked by
  its own independent evaluator only.

**Speed against PlaneGCS,** measured around `solve` only, so FreeCAD's overhead around it isn't counted:
33 µs against 258 µs on outlines, 156 µs against 5.2 ms on several blocks, and 2.5 ms against 4.4 s on large
sketches. PlaneGCS's time includes its dense rank analysis, and this solver's includes its sparse one.

**Dragging against PlaneGCS,** 6,591 drags of a free point by 1 to 15 mm:

| Category | Drags | PlaneGCS broke a constraint | I couldn't reach | It couldn't reach | It reached, I stopped short | Both finished | Others moved, mine | Others moved, PlaneGCS |
|---|---|---|---|---|---|---|---|---|
| Rectilinear | 578 | 0 | 213 | 213 | 0 | 365 | 24.15 | 24.15 |
| Polygons | 991 | 0 | 122 | 120 | 2 | 869 | 12.34 | 12.43 |
| Rectangles | 736 | 1 | 193 | 193 | 0 | 542 | 15.94 | 15.97 |
| Slots | 1000 | 0 | 277 | 444 | 0 | 519 | 18.57 | 24.65 |
| Rounded | 695 | 0 | 52 | 265 | 0 | 402 | 30.55 | 31.25 |
| Circles | 1000 | 0 | 95 | 161 | 4 | 485 | 4.60 | 4.99 |
| Symmetry | 999 | 0 | 0 | 0 | 0 | 999 | 6.54 | 6.55 |
| Blocks | 592 | 4 | 136 | 187 | 3 | 302 | 18.43 | 19.36 |

"Others moved" is the root of the summed squares of everything else's movement, in mm, over the drags where
both finished with every constraint kept. This solver never broke a constraint in a drag, by its own check.
The 9 it stopped short on are in Limits below.

## Its own fuzz

60,000 cases with no FreeCAD involved, each checked by an evaluator with formulas separate from the solver's
equations. Each solved sketch is also solved again, which must not move it. `cargo run --release -p
coreshape-sketch --example fuzz -- 5000` prints this, and `--example replay -- <category> <seed>` explains a case.

| Category | Cases | Refused | Wrong |
|---|---|---|---|
| Rectilinear outlines | 5,000 | 4 | 0 |
| Polygons | 5,000 | 0 | 0 |
| Rectangles | 5,000 | 0 | 0 |
| Slots | 5,000 | 0 | 0 |
| Rounded outlines | 5,000 | 0 | 0 |
| Circles | 5,000 | 0 | 0 |
| Symmetry | 5,000 | 0 | 0 |
| Several blocks | 5,000 | 2 | 0 |
| Large, about 130 to 290 entities | 5,000 | 13 | 0 |
| Implied constraint added | 5,000 | 0 | 0 |
| Conflict added | 5,000 | 0 | 0 |
| Triangle that can't close | 5,000 | 0 | 0 |

"Wrong" also covers a conflict reported without naming the constraints added for it. All 19 refusals have a
solution by construction. They are reported as no solution near here, except:

- **2 collapse** ("the only solution found collapses entity N"), for example `large 173`.
- **1 calls itself a conflict when a solution exists,** `large 1304`, reported as "no arrangement ... was found".
  A short side of a stepped outline starts pointing backward, and closing the outline would need that side to pass
  through zero length, which a length constraint can't do. So the message's wording is honest about what was
  tried, but the conclusion it invites is wrong for this case.

## Speed

Measured in Chrome as WebAssembly, on this desktop. A 302 KiB module. In milliseconds, and Chrome's clock
rounds to 0.1 ms. `python3 scripts/sketch_wasm_bench.py` and `cargo run --release -p coreshape-sketch --example
bench` reproduce these. A phone will be slower, and hasn't been measured.

| Workload | Entities | Constraints | Solve | Start a drag | Pointer move, median | 95th percentile | Worst |
|---|---|---|---|---|---|---|---|
| Rectangles in a strip | 100 | 238 | 0.9 | 1.5 | 0.1 | 0.2 | 0.3 |
| | 300 | 713 | 3.2 | 3.0 | 0.2 | 0.3 | 0.4 |
| | 500 | 1188 | 5.8 | 7.0 | 0.4 | 0.5 | 0.6 |
| Every line parallel to one | 100 | 199 | 1.1 | 1.2 | 0.1 | 0.2 | 0.3 |
| | 300 | 599 | 8.3 | 8.3 | 0.4 | 0.6 | 0.7 |
| | 500 | 999 | 17.0 | 18.1 | 0.5 | 0.8 | 1.6 |
| A chain of tangent arcs | 100 | 200 | 1.3 | 1.3 | 1.5 | 3.7 | 5.7 |
| | 300 | 600 | 7.4 | 7.8 | 4.7 | 11.3 | 15.4 |
| | 500 | 1000 | 26.9 | 19.4 | 7.3 | 18.5 | 24.8 |
| Mixed blocks | 101 | 231 | 1.1 | 1.0 | 0.5 | 0.9 | 1.0 |
| | 304 | 723 | 5.7 | 5.0 | 2.8 | 4.4 | 5.9 |
| | 505 | 1181 | 16.8 | 13.5 | 12.7 | 36.6 | 101.6 |

A pointer move at 300 entities takes at most about 16 ms, the length of a frame at 60 Hz, and usually far less.
The one slow frame at 505 entities is 100 ms.

## Limits

- **It finds the nearest solution, and can't flip one.** A length constraint keeps a line pointing the way it
  points, and a tangency keeps the side it starts on. So geometry far from what the constraints ask can stall or
  land somewhere unintended. Drawing near the intent matters.
- **A drag can stop short** where a different arrangement would reach the pointer: 9 of 6,591 here where PlaneGCS
  got there. In the two largest (1.4 and 2.2 mm in the first run), the grabbed line shrank toward zero length under
  an angle constraint, and getting further would have meant first moving other things farther away. The sketch stays
  valid and the report says how far it is from the pointer.
- **A grabbed point lands within about 1e-7 mm of the pointer,** not exactly, because the pointer is soft. Use a `fix`
  or a dimension to place something exactly.
- **A curve's end has to land within 1e-7 mm of another curve to join it.** A larger gap is a real gap, and the piece
  beyond it is listed as ignored. Solved sketches sit far inside this, but hand-placed geometry may not.
- **On an ellipse, not there yet:** a distance to its curve, and `normal`. Use its center or `7.major`, and `tangent`.
- **A curve tangent to an ellipse needs a shared point.** A line doesn't: its tangency has a closed form.
- **On a spline, not there yet:** a point on the curve between its ends, a tangency anywhere but an end, a distance to its curve, and `normal`. These are
  refused by name. Use its ends and control points. A constraint can hold any of those.
- **Splines aren't compared with PlaneGCS.** Reaching a control point in FreeCAD goes through weight circles with freedoms of their own, and that mapping
  isn't written. What the solver does with a spline is point equations already compared there, with the end tangency as an ordinary parallel or
  perpendicular between lines. It's still checked by this solver's own fuzz, whose check evaluates the curve itself.
- **A conflict set of more than 64 constraints** isn't shrunk to a smallest one. The message says where it stalled.
- **Solving is `f64` with tolerances.** Near-degenerate sketches, such as a line nearly parallel to one it must be
  perpendicular to, are where failures concentrate.
