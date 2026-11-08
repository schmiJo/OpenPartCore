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
| `regions` | Closed loops as outer boundaries and holes, which become kernel profiles |

Solving returns a new sketch and never changes the one it was given, so a failure can't leave anything
half done.

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
- **Regions refuse branches.** Three edges meeting at a point, or two loops touching, is refused with the entities
  named. Picking which of several overlapping regions to use isn't supported.
- **Not there yet:** ellipses and splines; a distance to a circle; a distance between two parallel lines without
  picking a point; reference dimensions that report without driving.
- **A conflict set of more than 64 constraints** isn't shrunk to a smallest one. The message says where it stalled.
- **Solving is `f64` with tolerances.** Near-degenerate sketches, such as a line nearly parallel to one it must be
  perpendicular to, are where failures concentrate.
