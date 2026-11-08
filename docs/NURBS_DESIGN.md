# NURBS: a design note

Written before any NURBS code, because the kernel's robustness rests on something NURBS don't have. Phases 1 and 2 are now built.
See [What was built](#what-was-built) at the end, which also says where this note turned out wrong.

## What the kernel relies on today

Every surface is one of five analytic types, and each provides things that are exact and cheap:

| What | Where it's used | What it gives |
|---|---|---|
| A signed distance `sdf(p)` that is 1-Lipschitz | `boolean/roots.rs` | A value of `d` proves no surface lies within `d`, so a curve piece can be discarded with certainty, not by sampling density |
| The sign of `sdf` | `boolean/classify.rs` | Inside or outside the surface, everywhere in space, with no search |
| `project` and `normal` in closed form | `surface.rs`, `tweak.rs` | Snapping and tangency tests with no iteration |
| Closed-form intersections for common pairs | `boolean/ssi.rs` | Exact lines, circles and ellipses, and all branches found without search |
| A chart with 3D distance ≤ chart distance | `surface.rs`, `tess/` | 2D topology runs in the chart, and a chart distance bounds a 3D one |
| Exact offsets | `tweak.rs`, `blend.rs` | Offset of a cylinder is a cylinder, so fillets are exact |
| Closed surfaces with no seams or poles | `topo.rs` | Splitting at 180 degrees removes those cases before any algorithm sees them |

## What NURBS take away

**There is no signed distance.** A patch is a bounded piece of a surface with no inside. Its nearest point is found by iteration, has
several local minima, and has no closed form. A number that's only a local minimum proves nothing, so the root finder's certificate has
no equivalent, and using a Newton distance in its place would silently turn a proof into a guess. This is the central point.

**The sign has no meaning alone.** Classifying a point against a patch only makes sense near it, so point-in-body has to come from ray
casting over faces, with its own hard cases.

**Intersections have no closed form,** and the hard part isn't following a curve, it's being sure of finding every branch. Marching from
a sampled seed misses small loops, which is the classic silent failure.

**The natural chart distorts.** Parameter space is the obvious chart, but 3D distance per parameter unit varies over a patch, and vanishes at
a degenerate edge. So the bound `3D ≤ chart` holds only after scaling by a speed bound, which can be loose.

**Offsets aren't NURBS.** An exact offset of a spline surface is generally no spline, so fillets and offsets become approximations with a
tolerance, unlike every operation the kernel has now.

## What replaces the certificate

NURBS have a different kind of guarantee: **a rational B-spline lies within the convex hull of its control points** (for positive weights),
and subdividing makes that hull converge quadratically to the surface. Derivatives are B-splines too, so their control points bound the
normals. Those give:

- **Exclusion.** Two hulls that don't overlap prove their pieces don't meet. It's a yes-or-maybe answer like the distance test, reached by
  subdividing instead of evaluating.
- **Uniqueness.** Interval Newton (a Krawczyk test) proves there is exactly one root in a box, or none.
- **No hidden loops.** If two pieces' normal cones don't overlap, their intersection can have no closed loop within them (Sederberg's
  condition). Subdividing until that holds is what turns "we probably found every branch" into a proof.

So the shape of the guarantee survives, but the cost changes, from one evaluation to a subdivision tree. The kernel's general contour tracer
and its `critical_point` test would remain for following curves, with these providing the seeds.

## The design

**Keep analytic surfaces analytic.** Never convert a plane or cylinder to a spline to simplify the code. A NURBS kernel that treats everything
as NURBS is what this kernel set out not to be, and analytic faces would lose their exactness.

**One narrow interface, added first, with no change in behaviour:**

```rust
/// Can this curve piece touch this surface? Decided with certainty, or not at all.
enum Exclusion { Apart, Unknown }
fn may_touch(&self, curve_piece: &CurvePiece) -> Exclusion;

/// Where this point is relative to the face's surface, if that can be known locally.
fn side_of(&self, p: Vec3) -> Option<Side>;
```

`Surface::sdf` stays for analytic surfaces and implements these. A `Surface::Spline` holds control points, weights, knots and a lazily built tree of
hull boxes. Every caller of `sdf` in `boolean/` moves behind this interface first, with the whole fuzz suite unchanged as the proof, before any
spline exists.

**Pairs, in order of difficulty:**

| Pair | Method | Difficulty |
|---|---|---|
| Spline curve with analytic surface | Exclusion by hull, then interval Newton | Low |
| Plane with spline surface | Reduces to contouring one function over the chart, with hull exclusion | Medium |
| Other analytic surface with spline surface | `sdf` of the analytic one evaluated over the chart, contoured: the analytic side still gives a true distance | Medium |
| Spline with spline | Subdivide both, normal cones for loops, then the tracer | High |

The middle rows are a real advantage of this kernel: wherever one side is analytic, its signed distance is still a certificate.

**Point classification** uses a ray test against spline faces, with the existing sample-agreement voting, and refuses on disagreement as it does now.

**Charts.** Use parameter space scaled by a speed bound from the derivative's control points, computed per face. Refuse degenerate edges (poles) at
first. Closed surfaces are split at seams as now, which keeps "no seams, no poles" true.

**Tessellation** uses the CDT already there, on the chart, with a chord bound from the second derivative's control points. **Mass properties** still go
through Green's theorem, with numerical quadrature and an error estimate, so a volume stops being exact to rounding on those faces and says so.

**Files.** STEP's `B_SPLINE_SURFACE_WITH_KNOTS`, rational form included, is direct to write. OpenCASCADE reads it, so the existing oracle carries over.

**What stays refused at first:** fillets, offset, shell and draft on spline faces, since they can't be exact, and pretending otherwise would break this
kernel's promise that an answer is either right or refused.

## How it would be tested

The strongest test available costs almost nothing: **represent an analytic shape exactly as a NURBS, and require the same answer.** A rational quadratic
spline can be a true cylinder, cone, sphere or torus. Every existing fuzz category then runs with one or both inputs converted, and the volumes must
match the analytic results to the kernel's tolerance. That turns 25,000 known-good cases into an oracle before OpenCASCADE is involved at all. Beyond
that, free-form cases go against OpenCASCADE, with the same volume identities and membership checks as today.

## Phases

| Phase | Work | Value by itself |
|---|---|---|
| 0 | The interface above, with analytic surfaces only and no behaviour change | Makes everything after it safe |
| 1 | Spline curves in sketches, with the solver gaining splines. Extruding one gives a general cylinder, whose distance is a 2D problem and easier to certify | Most of what people want from "splines" in mechanical parts |
| 2 | Revolving spline curves | Turned parts with free-form profiles |
| 3 | Free-form patches: spline with analytic, then spline with spline | The full feature, and the bulk of the risk |

Phases 1 and 2 keep a true distance in one or two dimensions, so they can stay nearly as trustworthy as the analytic kernel. Phase 3 is where it would
stop being a small kernel. It's a reasonable place to stop and reassess.

## Decisions needed

1. **Scope.** Stop after phase 2, or commit to phase 3? I recommend building through phase 2 and deciding on 3 from what's left unserved, since most
   mechanical parts need spline profiles, and few need free-form patches.
2. **Is a refusal acceptable for spline fillets?** I'd keep refusing at first. Approximate fillets would be the first results in this kernel that are
   neither exact nor refused, and they deserve their own decision.
3. **Volumes with an error bar.** Are numerically integrated volumes acceptable on spline faces, reported with their bound?

## What was built

Decided: build through phase 2, refuse spline fillets at first, report volumes with an error estimate, and decide on phase 3 later.

**Phase 0 wasn't needed, and this note was wrong to put it first.** Its point was to move every caller of `sdf` behind an interface that
doesn't promise a distance. But in phases 1 and 2 the surfaces still have a true signed distance, so every caller stays as it was.
Phase 3 would still need that interface.

**The certificate carries over.** The distance from a point to a rational curve has an exact polynomial form: the stationary points are
the roots of `(A - p w) . (A' w - A w')`, whose Bernstein coefficients are linear in `p`. Sign changes among them bound the roots, so every
local minimum is isolated with certainty, at the cost of a few dozen multiplications per knot interval. Each face's curve turns less
than 150 degrees and carries on along its end tangents, which makes it a graph over one direction. That divides the plane in two, so
the signed distance is 1-Lipschitz everywhere, as tested directly. A revolved curve's distance is the same thing in the half-plane through the
point, and stays 1-Lipschitz through the change to those coordinates.

**What exists:**

| Piece | What it does |
|---|---|
| `Spline` | Rational B-splines up to degree 7: evaluate, closest point, sub-curves, reversal, affine maps, cubic through points, exact elliptical arcs |
| `Surface::Extruded`, `Surface::Revolved` | A curve swept along a line or around an axis. A curve may end on the axis, where the chart is polar about the tip |
| Exact sections | A plane across an extrusion gives a spline, including at a slant, since a rational spline follows an affine map exactly. A plane along the sweep, or a parallel cylinder or extrusion, gives straight rulings. Coaxial surfaces give circles. A plane through the axis of a revolution gives the curve itself |
| Everything else | The existing contour tracer, unchanged, which needs only a true distance and a chart |
| Mass properties | Integrated along edges with the inner integral in closed form, with `volume_error` from comparing two rules |
| STEP | `SURFACE_OF_LINEAR_EXTRUSION`, `SURFACE_OF_REVOLUTION`, rational `B_SPLINE_CURVE_WITH_KNOTS` |

**Analytic stays analytic.** A straight stretch of a spline becomes a plane, or a cone or cylinder when revolved, and not a spline surface.

**Refused:** fillets, chamfers, offset, shell and draft on spline faces, and two different splines sharing part of their length. A spline can
touch a revolve axis only at its ends.

**Phase 3 is still open.** OpenShape has now asked for lofts and for free-form surfaces from imported files, in that order, after extruded
splines. Both are phase 3, and this note's warning stands: that's where this stops being a small kernel.
