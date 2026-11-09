# Sheet bodies: design note

A sheet is an open surface that exists as a body of its own: a flat patch, a curved panel, a tube with no ends. Until now everything in this kernel has been a closed solid, so a surface couldn't exist alone, thicken could only take faces from a solid, and a STEP file holding only surfaces was refused.

This note says what breaks when a body isn't closed, and how solids are kept exactly as they are. It comes before the code because nearly every check in the kernel leans on closedness, and a careless change here would weaken the checks that make solids trustworthy.

## The representation

A sheet is an ordinary `Body`, with vertices, edges, faces and loops as they are, plus one new field:

```rust
pub struct Body { ..., pub sheet: bool }
```

Faces keep everything they have: a surface, a `flipped` flag and loops that keep the face on their left, seen from the side the normal points to. An edge is used by one coedge, which makes it a **boundary edge**, or by two running opposite ways, as in a solid. There's no thickness, no material side and no inside. A sheet's normal only says which way it faces.

**Why a flag, when openness could be worked out from the edges.** Working it out would make "an edge used once" mean "a sheet" everywhere, and in this kernel it means "this solid is broken". Every Boolean, fillet and shell ends in `validate`, and that check catching an unclosed result is what stops a wrong answer getting out. A solid that came out with a hole in it must never pass as a sheet. So being a sheet is something a body is declared to be, by the few operations that make one. Anything else builds a body with the flag off and gets the strict check.

A sheet that happens to close, like a sphere's surface from revolving a half circle, stays a sheet until it's sewn, which is what turns it into a solid.

## What breaks, and what's done about it

### The distance functions and their certificates

`Surface::sdf` and the root finding built on it (`curve_surface_contact`, the Lipschitz bounds in `ssi`) are about one surface, not one body. A surface's signed distance and its never-overstating property have nothing to do with whether a face on it has neighbours. **Nothing changes here,** and sheets get the same certificates.

The sign is the one thing to be careful of. It says which side of the *surface* a point is on. With a closed body that's a proxy for inside and outside, but nothing uses it that way without a parity count from `Index`.

### Inside and outside

`Index::locate_in_body` casts rays and counts crossings. On an open body the count means nothing: a ray can leave through the boundary without crossing anything, so whether a point is "inside" a bowl depends on the ray. **So there's no inside for a sheet, and no code asks.** Everything that asks is a Boolean or a check on one, and those refuse sheets at the door. `locate_on_face` is about one face and works as it is. It's what a sheet's checks use.

### Validation

`validate` has three parts that assume closedness, and each takes a separate path when `body.sheet` is set, leaving the solid path as it is, line for line:

| Check | Solid | Sheet |
|---|---|---|
| Edge use | Exactly two, opposite ways | One, or two opposite ways |
| Fans at a vertex | One closed fan | One fan, open or closed. Two fans is still a pinch |
| Euler-Poincare | `V - E + F - H = 2 - 2g` | `V - E + F - H = 2 - 2g - b`, with `b` boundary loops counted from the edges |
| Geometry and windings | | The same |

The open form of Euler's formula is a real check and not a loosening: a sheet with a boundary loop miscounted, or a face wrongly joined, gives an odd or negative genus. What it can't see is whether a sheet "should" have been closed, as a sheet has no such thing as should. That is why the flag is explicit.

### Mass properties

Volume comes from a flux integral over a closed surface, and over an open one it depends on the reference point, so it's meaningless. Area is per face and doesn't care. So `mass_properties` on a sheet counts area everywhere, and volume only for shells that do close. A sheet's volume is **zero** unless it closes. A solid takes the old path, with the same numbers to the bit.

`Outcome::body` holds all of a document's bodies side by side, and can hold a sheet beside a solid. That body has the flag set, so its volume is the solids' and its area is everything's. Validating it applies the sheet rules to the whole, which is weaker for its solids, but each solid has already passed the strict check when it was made.

### Meshing

The mesher works face by face on shared edge polylines, so nothing needs to change for it to mesh a sheet: shared edges stay watertight and boundary edges are simply used once. `Mesh::open_edge_count` isn't zero on a sheet. It should be **exactly the segments of the boundary edges**, and that is the check: an open mesh edge anywhere else means a tear, as it does on a solid. `Mesh::volume` is as meaningless as the B-rep's. Rendering shades both sides already, so a sheet is visible from behind.

### Booleans

`boolean_with` assumes closedness in three places, each of which is why it can't simply be pointed at a sheet:

1. **Classifying.** Each piece of A is classified against B by `locate_in_body`, and that needs B closed. A sheet as a tool has no inside to classify against.
2. **Assembly.** `assemble` wants every edge used an even number of times, and says "the result does not close" otherwise. Refusing that is the safety net for solids.
3. **Membership.** `verify_membership` asks whether points are in the result, by rays.

So **`union`, `subtract` and `intersect` refuse a sheet,** with a message saying what to use instead. Cutting a sheet is a separate plan through the same machinery: the arrangement, the surface intersections and the tracing are used as they are, since they only need faces. Only the sheet's faces are traced and kept, they're classified against the solid, which is closed and has a real inside, and assembly allows boundary edges for this plan alone.

A cut gets its own checks in place of membership: every piece kept is on the right side of the solid, every sampled point of the original on that side is on the result, and the mesh is torn nowhere but its boundary. Tests hold area to its formula, and `area(outside) + area(inside) = area(sheet)` is an identity that doesn't depend on anything computed.

A sheet lying exactly on a solid's face isn't inside the solid, so it goes with what's outside. That way the two parts always make up the sheet, and nothing is counted twice. An early version kept it on both sides, and the identity above is what showed that up.

### Cleanup

`simplify` merges faces and edges where a face is on both sides of an edge, or a vertex has two edges. It only pairs what's there, so a boundary edge is never merged across. `merge_edges` does what's wanted on a boundary, taking out a vertex nothing else uses. It's checked through the validity of its results.

### Everything else that takes a body

Fillet, chamfer, offset, shell, draft and delete face all rebuild neighbouring faces and end in the strict check. **They refuse a sheet, at the door, with a clear message,** rather than getting partway and failing with "edge 3 is used by 1 coedges, not 2". Thicken doesn't need a solid at all, as it already handles an edge with only one chosen face: it's the one that joins up directly. Moving, mirroring and patterning a sheet's copies are the same as for any body.

### Documents

- **Default scope.** A feature with no `bodies` acts on all of them, and a `remove` shouldn't start failing because a sheet is lying around. So **the default scope is solids only,** as Onshape's merge scope is. A sheet is acted on when it's named, by what can act on it.
- **A solid feature naming a sheet** is refused, in the kernel's own words.
- **Merge modes** don't apply. A new mode, `surface`, makes a sheet from what `extrude`, `revolve`, `sweep`, `loft` and `helix` would have made, without their caps. These always make new bodies, one for each connected piece, and depend on nothing else in the document.
- **An open sketch curve** used to be refused at the sketch. It still is, unless a `surface` feature uses that sketch.
- **Names** work as they do for solids. A face swept from an open curve is named for its segment or entity, and `f3:end` is a filled region.

### STEP

A sheet goes out as `SHELL_BASED_SURFACE_MODEL` holding an `OPEN_SHELL`, in a `MANIFOLD_SURFACE_SHAPE_REPRESENTATION`. Files with only solids come out exactly as before. On reading, `read_step` keeps its meaning for any file with solids in it, so the numbers measured against OpenCASCADE don't move, and a file with no solids at all now comes in as sheets rather than being refused. `read_step_all` gives both.

## How solids stay as they are

- `sheet` is false by default, and only a sheet-making operation sets it. Nothing that existed before sets it.
- Each check with a different rule for sheets branches once at the top, and the solid branch is the code that was there.
- `union`, `subtract` and `intersect` are unchanged for solids: the door check is the only new thing they do.
- Before each commit touching shared code, the Boolean, spline, feature and blend fuzzers run with 0 wrong, and the STEP import check stays where it was.

## The order of the work

1. A body can be open: the flag, `validate`, `mass_properties`, and the mesh check.
2. Making one: fill a region, sides of what's swept, open curves, and copying faces out of a solid. Through document features, the file format and MCP.
3. From a sheet to a solid: thicken, and sewing sheets that close.
4. Cutting: a sheet with a solid, with a plane, and with a sketch's outline.
5. STEP in and out.

## Not attempted

- **Cutting a sheet with another sheet,** and Booleans between sheets. A sheet has no inside to classify against, so this takes a different classifier and has to deal with curves that stop in the middle of a face. It's refused, naming what it found.
- **Offsetting a sheet, and filleting between sheets.** Each needs its own care with the boundary.
- **A loft between open curves.** Its code matches closed loops around, so it isn't the same code with a piece thrown away, as an extrude and a revolve are.
- **Non-manifold sheets,** where three faces meet along an edge. A body's edges have at most two faces, and a sew that would make three is refused.
