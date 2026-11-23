# The CoreShape document format

A `.coreshape` file is UTF-8 JSON. It holds what the model *is*: parameters and an ordered list of
features. Results are never stored, because a rebuild recreates them exactly.

```json
{
  "format": "coreshape-document",
  "version": 1,
  "units": "mm",
  "next_id": 4,
  "parameters": [
    {"name": "size", "value": 30},
    {"name": "h", "value": "size / 3"}
  ],
  "features": [
    {"id": 1, "name": "Outline", "type": "sketch",
     "regions": [{"outer": {"type": "rect", "w": "size", "h": "size"}}]},
    {"id": 2, "name": "Plate", "type": "extrude", "sketch": 1, "length": "h"},
    {"id": 3, "name": "Round", "type": "fillet", "edges": {"on_face": "f2:end"}, "radius": 1}
  ]
}
```

Readers refuse anything they don't fully understand, including unknown fields, since quietly dropping
part of a model would be worse than an error. Fields shown as defaulted may be left out; the writer
always writes them.

## Dimensions

Every dimension is a number or an expression in quotes: `"width / 2 - inset"`, `"sqrt(a^2 + b^2)"`.
Expressions use `+ - * / ^`, parentheses, parameters, `pi`, and `sin cos tan asin acos atan atan2 sqrt
abs floor ceil round min max`. Angles are in degrees, and lengths in millimetres.

## Names

A feature's `id` never changes and is never reused, and `next_id` keeps that true across saves. Faces
are named by the feature that made them:

| Name | Face |
|---|---|
| `f2:start`, `f2:end` | The caps of an extrude or partial revolve |
| `f2:side.3` | Swept from segment 3 of the outer wire |
| `f2:hole.1.side.0` | Swept from segment 0 of hole 1 |
| `f2:region.2.side.0` | Swept from segment 0 of the outer wire of region 2 |
| `f2:wall.0` | The walls an extrude swept from the edges of the first face in its `faces`: see [below](#extruding-faces) |
| `f2:inner.side.3`, `f2:start_of.side.0`, `f2:end_of.side.4` | A thin wall's side to the left of segment 3, and the ends of a wall along an open curve: see [Thin walls](#thin-walls) |
| `f2:bore.7`, `f2:cbore.7`, `f2:cbore_floor.7`, `f2:csink.7`, `f2:tip.7`, `f2:floor.7`, `f2:top.7` | The faces of hole 7 of a hole feature: see [Holes](#holes) |

Segments number from 0 in drawing order. A rectangle runs bottom, right, top, left. Both halves of a
circle are segment 0. Primitives are built by the same sweeps, so `f2:end` is the top of a box.
A name stays with a face through Booleans, and can cover several pieces if the face gets split.

**A name is stable but not unique.** No feature renames a face, but a name means every face that has it, and it disappears if
they all do. `crates/document/tests/face_names.rs` pins this down:

| Through | What happens |
|---|---|
| Offset, draft | Every name kept, with one face each |
| Shell | Every name kept. An inner wall is named for the shell, after the face it follows: `f9:side.3` is behind `f2:side.3` |
| Fillet, chamfer | Every name kept. All the new faces of one feature share one name, `f9:side.1`, so one can't be picked out by name |
| Mirror, pattern | A copy has its original's name, so `f2:end` can mean several faces in different places, and different planes |
| A cut | Each piece of a split face keeps the name. They're all in the same surface |
| A whole circle | Two half faces under one name |
| A face swallowed by a later feature | The name is gone. Anything using it should fail and say so |

So to attach to a planar face by name, pieces of a split face are safe because they share a plane, while mirror and pattern
copies are ambiguous unless something else says which one is meant.

## Features

Common fields: `id`, `name`, `suppressed` (default false), `bodies` (default all of them): see [Bodies](#bodies), and `app`: anything an app wants kept with the feature, such as where a sketch's dimension labels sit. It's kept as it is and changes nothing in the model, and the document can have one too.

| `type` | Fields |
|---|---|
| `sketch` | `plane` (default XY; `{"on_plane": "top"}` or `{"on_plane": 7}` puts it on a [plane](#planes) and it follows it, and with `"on_face": "f2:end"` it follows that flat face, keeping its own origin), `regions` |
| `plane` | A plane, not a part: `plane_type` and its fields, and `flip`. See [Planes](#planes) |
| `sheet_metal` | `sketch`, `thickness`, `bend_radius` (default the thickness), `k_factor` (default 0.45), `depth` for a chain of lines, `flip`, and the part's defaults for what comes after: `minimal_gap` (default a tenth of the thickness) and `corner` (`open`, the default, or `closed`). A new sheet metal part. See [Sheet metal](#sheet-metal) |
| `flange` | `edges` (a [selection](#edge-selections)), `length`, `degrees` (default 90), `alignment` (`outer`, the default, `inner` or `bend`), `bend_radius` (default the part's), `flip`, `corner` (default the part's) and `gap` (default the part's minimal gap) where its edges meet. See [Sheet metal](#sheet-metal) |
| `hem` | `edges`, `kind` (`straight`, `rounded`, `teardrop` or `rolled`), `length` (not for `rolled`), `gap` (`straight` only, default the part's minimal gap), `bend_radius` (not for `straight`, default the part's), `degrees` (`teardrop`, default 225, and `rolled`, default 270), `alignment`, `flip`. See [Sheet metal](#sheet-metal) |
| `tab` | `sketch`, whose regions are drawn on the plane of a sheet metal wall. See [Sheet metal](#sheet-metal) |
| `bend` | `sketch`, holding one line on the plane of a sheet metal wall, `degrees` (default 90), `bend_radius` (default the part's), `position` (`center`, the default, or `start`), `moves` (`left`, the default, or `right`), `flip`. See [Sheet metal](#sheet-metal) |
| `extrude` | `sketch`, `length`, `start` (default 0), `mode` (default `add`), and optionally `regions`, `faces`, `end`, `opposite` and `thin`: see [below](#extrude-regions-and-ends) and [Thin walls](#thin-walls) |
| `loft` | `sections`, in order, each a sketch id, for a sketch holding one outline, or a region `{"sketch": 2, "region": "r..."}`, so that one sketch with several outlines can feed it, and `mode`. Straight between two, and a smooth cubic through three or more. A region that has gone fails the feature, saying what's there. Optionally `thin`: see [Thin walls](#thin-walls), and the options in [Lofts](#lofts) below: to a point, round in a ring, along guides, and square or rounded ends |
| `revolve` | `sketch`, `degrees` (default 360), `mode`, and `axis`: `{"point": [x, y], "direction": [dx, dy]}`, a line in the sketch's plane, in its coordinates, to turn around as drawn, right-handed around its direction, or any [line](#planes) lying in the sketch's plane, which the revolve then follows: `{"sketch": 3, "line": 7}` for a line of its sketch, or a straight edge. One out of the plane is refused, saying how far off it is. Without an `axis`, profile x is the distance from the sketch's Z axis, and profile y the height along it. Optionally `regions`, as an extrude has them: see [below](#extrude-regions-and-ends), and `thin`: see [Thin walls](#thin-walls) |
| `sweep` | `sketch`, `path`, `mode`, and optionally `thin`: see [Thin walls](#thin-walls). `path` is a list of steps, `{"sketch": 5}` or `{"edges": selection}`: see below |
| `helix` | `sketch`, `pitch` (how far along the axis one turn goes), `turns`, `point` (default origin) and `axis` (default +z) for the axis, or `axis` alone naming any [line](#planes), which it then follows, `left_handed` (default false), `mode`. The sketch can hold the axis, for a spring or a thread, or lie across it, for a twisted bar |
| `solid` | `shape`, its dimensions, `at` (default XY), `mode` |
| `pattern` | `of` (feature ids, which can be other patterns and mirrors; leave out for the whole body), and `direction`, `count`, `spacing`, or `center` (default origin), `axis` (default +z), `count`, `degrees` (default 360). `count` includes the original. `direction` is `[x, y, z]` or any [line](#planes), whose direction it takes, and `axis` is `[x, y, z]` through `center`, or a line, which gives the center too, so there's no `center` with one: `{"axis": {"axis_of": "f4:side.0"}, "count": 6}` goes round a boss. A circular pattern turns right-handed about the axis's direction |
| `mirror` | `of` (as above), and `plane`, any [plane](#planes), or `point` (default origin) and `normal` |
| `fillet` | `edges`, `radius`, `propagate` (default true: follow tangent neighbours) |
| `chamfer` | `edges`, `distance`, `propagate` (default true: follow tangent neighbours) |
| `fillet_radii` | `sets`: a list of `{"edges": ..., "radius": r}`, or `{"edges": ..., "along": [{"at": 0, "radius": 2}, {"at": 1, "radius": 5}], "start": [x, y, z]}` for a radius going in straight lines between places along the chain of edges, from 0 to 1, measured from the end nearest `start`, or without one from the end with the lowest x, then y, then z. Different radii can meet at a corner: see [KNOWN_LIMITS.md](KNOWN_LIMITS.md#fillets-and-chamfers) |
| `import_step` | `text`: a whole STEP file, kept in the document, and `mode`. Its faces are named by their place in the file: `f3:side.0` is the first. A wide face cut into pieces on the way in keeps one name for them all. An assembly's parts are placed where it puts them. With `mode` `new_body`, which is what an assembly wants, each solid is a part of its own. With `add`, those that touch are merged by Boolean, which can fail where they meet, and says to use `new_body`. Each solid is built on its own, and one that can't come in is left out: the feature still builds, with a note for each saying which it is and why, `left out solid 75 of 711, "Zahnrad Servo" (#61327): ...`, by its place among the file's solids, its part's name, and its entity number. Only a file with nothing that can come in fails the feature. A solid whose faces meet at a single point, which this kernel can't have as one corner, is given a corner for each side, in the same place, with a note `repaired solid ...`. Each body is named after its part, and a solid that's in the file more than once, entity for entity, is built once. |
| `import_step_parts` | `text`: a whole STEP file, kept in the document. Each distinct solid in it comes in once as a new body, in its own coordinates, for an assembly to use, however many times the file uses it. Its faces are named as for `import_step`, and one that can't come in is left out with a note in the same way |
| `transform` | `motion`, the `bodies` to move, and `copy` (default false), which leaves them and adds moved copies as new parts. `motion` is `{"type": "translate", "by": [x, y, z]}`, `{"type": "rotate", "point": [x, y, z], "axis": [x, y, z], "degrees": d}`, or with `axis` naming any [line](#planes) and no `point`: `{"type": "rotate", "axis": {"axis_of": "f4:side.0"}, "degrees": 90}`; `{"type": "frames", "from": placement, "to": placement}`; or `{"type": "scale", "factor": k, "point": p}`, the same in every direction about any [point](#planes) (default the origin), so 25.4 brings a part drawn in inches to millimetres. A scaled face keeps its name and its kind of surface. A factor from 1e-6 to 1e6 is allowed, and one that would take the part beyond 10 km or shrink an edge below 1e-6 mm is refused. Or `{"type": "stretch", "axis": [x, y, z], "point": p, "factor": k}`, or with `axis` naming any [line](#planes) and no `point`: stretched along that direction only, about the plane through the point square to it, which stays put, everything else going `k` times as far from it. Three stretches make any x, y and z scale. Every face keeps its name. A round face whose axis isn't along the stretch is no longer round, and becomes the free-form surface it exactly is; a face moved by an offset or a shell is refused, as stretched it's no longer a surface moved by a distance |
| `delete_bodies` | The `bodies` to delete, which it has to name |
| `boolean` | `operation`: `union`, `subtract` or `intersect`, `tools`: a list of bodies, and `keep_tools` (default false). `subtract` cuts the tools from the `bodies`, its targets, which it has to name. The others take the `bodies` and `tools` together, and what's left has the first one's name. Bodies that don't meet are refused |
| `delete_face` | `faces` (names or keys): those faces go, and their neighbours grow to close the gap. It takes holes, pockets, bosses, fillets, chamfers, rabbets, slots and corner notches, and refuses by name what it can't close |
| `thicken` | `faces` (names or keys), `thickness` (outward, or inward if negative), `mode` (default `add`). A solid between the faces and a copy moved along their normals. Near faces are this feature's `start`, far ones its `end`, and walls its `side.N`. Each keeps the face it came from in its key |
| `fill` | `sketch`: a flat [sheet](#sheets) filling each of its regions, facing along the sketch's normal, named as this feature's end cap: `f3:end` |
| `copy_faces` | `faces` (names or keys): a copy of them as a sheet. What they're on stays as it is |
| `sew` | `tolerance` (default 1e-6), and the `bodies` to join, by default every sheet. See [Sheets](#sheets) |
| `rib` | `sketch`, holding open curves; `thickness`; `direction`, `parallel` (the default) or `normal`; `flip` (default false); `extend` (default true). A wall along each curve that runs until it meets the parts the rib acts on, and joins them: see [Ribs](#ribs) |
| `hole` | Where: `sketch`, a constrained sketch, with `points` naming its points as constraints do (`"7"`, `"9.center"`), or without them every point in it; or `plane`, any [plane](#planes), with `points` as `[x, y]` in its coordinates. What: `style` (`simple`, the default, `counterbore` or `countersink`), `size` (a metric size, `"M1.6"` to `"M36"`), `fit` (`close`, `normal` or `loose`), `tapped`, `diameter`, `cbore_diameter`, `cbore_depth`, `csink_diameter`, `csink_angle`, `tip_angle`. How deep: `end` (`blind`, the default, with `depth`, `through_all` or `up_to_next`). And `from_sketch` and `opposite` (both default false). Cuts a hole at each point, from the parts it acts on: see [Holes](#holes) |
| `split` | What cuts: `plane`, any [plane](#planes); `face`, a face's name or key, cutting along the whole surface it's on, as far as that goes: a plane, facing out of its part as the face does, a cylinder, cone, sphere or ring-shaped torus, an extruded or turned curve, carried on along the curve's end tangents and past the part, or a free-form face, as far as the face itself goes; or `tool`, a part's name, a solid or a sheet. And `keep_tool` (default true). Each part it acts on, those it names or any it reaches, sheets too, comes apart into pieces that are parts of their own. The piece in front of the plane or a sheet, the way its faces face (for a face, out of its part), or inside the round surface or the solid, keeps the part's name, and the others are new parts named for the split (`f9`, `f9.1`, ...). The cut faces are named for the split: `start` along a plane, and `side.N` along any other face; by a part, they keep that part's names. A part it names but doesn't cut through is refused, and so is a split that cuts nothing, a sheet that ends inside a part, which would only slit it, one that leaves a part in one piece, and a sheet split along a sheet |
| `trim` | What cuts: `tool`, a solid's name like `"f3"`, or `plane`, any [plane](#planes), or `point` and `normal`, a plane as numbers, or `sketch`, whose regions cut straight through along its normal. And `keep`: `outside` (the default), `inside` or `both`. Acts on the `bodies` it names, by default every sheet. See [Sheets](#sheets) |
| `offset` | `distance`: every face moves out, or in if negative. Sharp edges stay sharp. |
| `shell` | `thickness`, `open` (face names, default none). An inner wall is named for this feature, with the slot of the face it follows: `f9:side.3` lies behind `f2:side.3`. |
| `draft` | `faces` (names), `degrees`, and `neutral`: the plane where faces stay put, any [plane](#planes), whose normal is the pull, or a point on it, `[x, y, z]` (default the origin), with `pull` (default +z). A positive angle narrows the part along the pull, and widens a hole, so a negative one drafts the other way. |

A file's text can be large: a 711-part assembly is 114 MB. An app that keeps a document between visits can keep the text
apart from it. `document_show` with `"omit_texts_over": 100000` gives each such `text` as
`{"bytes": 114295480, "sha256": "3f2a..."}`, and `document_open` takes that form back with the texts as `"blobs"`, an object
of each text under its SHA-256 in hex. A text under a hash that isn't its own is refused. Saved files always have the text
itself.

`mode` is `add`, `remove`, `intersect`, `new_body` or `surface`. A placement is `{"origin": [x, y, z], "z": [x, y, z], "x": [x, y, z]}`,
where each part has a default: the origin, world Z, and world X (world Y if z runs along X). Or it's `{"on_plane": plane}`
alone, any [plane](#planes) by name, which it then follows, taking its origin, normal and x from it: a sketch's, a solid's
`at`, and a transform's `from` and `to` can be, so a transform by frames can carry a part from one plane onto another.

Solids: `box` (`dx dy dz`), `cylinder` (`radius height`), `cone` (`r0 r1 height`), `sphere` (`radius`),
`torus` (`major minor`).

**Sweep paths** are one of three things:

| `path` | Goes along |
|---|---|
| `[{"line": 10}, {"arc_radius": 5, "degrees": 90, "toward_degrees": 0}, ...]` | Steps from the sketch plane, heading along its normal. Each step carries on from the direction the last ended in, so a path is always smooth. A step is `{"line": length}`, or `{"arc_radius": r, "degrees": a, "toward_degrees": d}`, a bend around a center `r` away in the direction `d` degrees counter-clockwise from the sketch's x axis |
| `{"sketch": 5}` | That sketch's curves, end to end: its open curves, or if it has none, the outline of its one region. A constrained sketch's open curves are allowed when a sweep uses it as a path |
| `{"edges": {"on_face": "f2:end"}}` | Those edges of a part, end to end, picked by any [edge selection](#edge-selections) |

Along a sketch's curves or a part's edges, as Onshape's sweep does, the pieces can come in any order and run either way. The profile goes along the path from its end in the profile's plane, or if the path is closed, from where it crosses that plane nearest the profile, so a closed path makes a ring with no ends. The profile keeps its place across the path and stays square to it, so it has to be drawn square to the path where that starts, but needn't be centred on it. The path has to be smooth, made of lines and arcs. What doesn't fit is refused, saying what and where: a path that starts off the profile's plane or leaves it at a slant, a corner, a spline or ellipse, and pieces that branch or don't join.

In every form, bends must stay wider than the profile, or the profile would cross the center. Faces are named from the profile, as a sweep's always were: `f6:side.N`, and `f6:start` and `f6:end` for the ends.

**Copies keep names.** A face in a pattern's or mirror's copy has the name of the face it came from, so `f2:end` covers
the original and every copy, and a fillet between two named faces applies to them all. To pick one, use a [face key](#face-keys).
A pattern of features applies each one's solids as it did, adding or removing.

## Planes

A `plane` feature makes a plane, not a part, as Onshape's Plane feature does. It's worked out again at every rebuild from
what it names, so it moves when they do, and so does anything on it. It has a `plane_type`, the fields that type takes, and
`flip` (default false), which turns its normal round, keeping its origin and x. If what it names has gone or doesn't fit,
such as three points in a line or a face that isn't flat, the feature fails and says what's wrong, as other features do,
and what's after it waits.

| `plane_type` | Fields | The plane |
|---|---|---|
| `offset` | `plane`, `distance` | Parallel to `plane`, `distance` along its normal, with its x |
| `plane_point` | `plane`, `point` | Parallel to `plane`, through `point`, which is its origin |
| `line_angle` | `line`, `degrees` (default 0), `reference` (a plane) | Through `line`, turned about it by `degrees`, right-handed about the line's direction. At 0 it's the plane through the line that comes nearest to being parallel to `reference`, or without one, to whichever of Top, Front and Right the line is most nearly parallel to (in that order when it's equally so). Its origin is the line's point, and its x runs along the line |
| `point_normal` | `point`, `line` | Through `point`, square to `line`, facing along it |
| `three_point` | `points`: three points | Through them: its origin is the first, its x points toward the second, and it faces the side from which the three go round counter-clockwise |
| `mid_plane` | `planes`: two planes, or `points`: two points | Halfway between two parallel planes, facing as the first does, or for two that meet, the plane through where they meet that bisects the angle between them, facing between their normals once the second is turned to face the same side as the first. With x from the first. Between two points, it's square to the line from the first to the second, at its middle |
| `curve_point` | `edge` (a selection picking one edge), and `point`, or `fraction` from 0 to 1 | Square to the edge, facing along it, at the point on it nearest `point`, or a `fraction` of its length from its start. An edge runs from its start to its end as `edges_list` gives them |
| `tangent` | `face` (a cylinder, cone or sphere), and `point`, or `degrees` | Touching the face, facing out of the part, at the point on it nearest `point`, or on a cylinder or cone, at an angle round its axis, right-handed about the axis's direction, from the direction a plane facing along the axis would have as its x, at the height of the middle of the face. On a cylinder or cone its x runs along the axis |

```json
{"id": 4, "name": "Plane 1", "type": "plane", "plane_type": "offset", "plane": "f2:end", "distance": 5}
{"id": 5, "name": "Plane 2", "type": "plane", "plane_type": "three_point", "points": [{"sketch": 3, "point": "1"}, {"sketch": 3, "point": "2.start"}, {"faces": ["f2:end", "f2:side.1", "f2:side.2"]}]}
{"id": 6, "name": "Plane 3", "type": "plane", "plane_type": "line_angle", "line": {"between": ["f2:end", "f2:side.0"]}, "degrees": 30, "reference": "top"}
{"id": 7, "name": "Plane 4", "type": "plane", "plane_type": "mid_plane", "planes": ["f2:side.1", "f2:side.3"], "flip": true}
{"id": 8, "name": "Plane 5", "type": "plane", "plane_type": "curve_point", "edge": {"on_face": "f2:end"}, "fraction": 0.25}
{"id": 9, "name": "Plane 6", "type": "plane", "plane_type": "tangent", "face": "f4:side.0", "degrees": 90}
{"id": 10, "name": "Plane 7", "type": "plane", "plane_type": "plane_point", "plane": "front", "point": [0, -12, 5]}
{"id": 11, "name": "Plane 8", "type": "plane", "plane_type": "point_normal", "point": "origin", "line": {"axis_of": "f4:side.0"}}
{"id": 12, "name": "Sketch 3", "type": "sketch", "plane": {"on_plane": 4}, "regions": [...]}
```

Mirror, trim and draft take any plane, an extrude can end `up_to_face` at one, a pattern can go along or round any line,
and a placement can be `{"on_plane": ...}`: see [Features](#features). Each follows what it names.

Each plane has an origin, a normal and an x direction, and its y is the normal across x. The MCP's `planes_list` gives where
each is, as the history stands or as it stood after any position. Planes, points and lines are named the same way wherever
a feature takes one:

| Reference | Forms |
|---|---|
| A plane | `"top"`, `"front"` or `"right"`; a plane feature's id, like `4`; a flat face's name or key, like `"f2:end"`; or `{"point": [x, y, z], "normal": [x, y, z]}` |
| A point | `"origin"`; `[x, y, z]`; a corner, `{"faces": [...], "near": [x, y, z]}`, as `vertices_list` and `edges_list` give them; or a constrained sketch's point, `{"sketch": 4, "point": "7.start"}`, with an entity's id for a point, or a curve's `start`, `end`, `center` or `major`, as constraints name them |
| A line | A straight edge's [selection](#edge-selections), picking one edge, which runs from its start to its end, or the same as `{"edge": selection}`; a constrained sketch's line, `{"sketch": 4, "line": 9}`, from its start to its end, or with a circle, arc or ellipse there, its axis, through its center along the sketch's normal; `{"axis_of": "f3:side.0"}`, the axis of a cylinder, cone, torus or turned face, pointing the way `faces_list` gives it, or `{"axis_of": selection}`, of a circle, arc or ellipse; or `{"point": [x, y, z], "direction": [x, y, z]}` |

Top, Front and Right are Onshape's: Top is the XY plane facing +Z with x along X, Front is the XZ plane facing -Y with x along
X and y up, and Right is the YZ plane facing +X with x along Y and y up. A flat face's plane faces out of the part, and its
origin is where the world's origin is seen along its normal, so that it doesn't move when the face grows or gets a hole.
Where nothing gives a plane its x, it runs along X on a plane nearer level than upright, and on any other it's level, to
the right as you face the plane. A name that covers several faces, such as a pattern's copies, has to have them all in one
plane, or one surface for an axis or a tangent: otherwise use a [key](#face-keys) for one of them.

## Sheet metal

A sheet metal part is one thickness all through, made of flat walls and the bends between them, and it can be laid out
flat. `sheet_metal` makes one from a sketch, as a new part:

- **From one region,** a plate: the region `thickness` thick, on the side of the sketch its normal points to, or the
  other with `flip`. Its holes are holes in the plate.
- **From one chain of lines,** an `open_path` or a constrained sketch's open curves: walls `depth` deep along the sketch's
  normal, on the chain's left as it runs, or its right with `flip`, with a bend at each corner. The lines are one face of
  the sheet: where the chain turns toward the sheet, they're the bend's outside, and its inside radius is `bend_radius`.

A `flange` adds a bend and a wall along each edge it's given, where a wall's face meets its side, the face that's the
sheet's thickness across. It bends toward the face the edge is on, by `degrees`, or away from it with `flip`. `length` is
from the outer virtual sharp, where the outsides of the wall and the flange would meet without the bend, to the flange's
end. `alignment` says where the bend goes: `outer` puts the flange's outside face through the edge, so the part's outside
size stays the same; `inner` puts its inside face through the edge; and `bend` starts the bend at the edge. For `outer`
and `inner`, the wall is cut back to make room for the bend. Where two edges of one flange feature meet at a corner, each
flange stops as far short of the corner as the wall is cut back, which leaves the corner open with a square cut out of it.

```json
{"id": 2, "name": "Tray", "type": "sheet_metal", "sketch": 1, "thickness": 2, "bend_radius": 2, "k_factor": 0.45}
{"id": 3, "name": "Sides", "type": "flange", "edges": {"any": [{"between": ["f2:end", "f2:side.0"]}, {"between": ["f2:end", "f2:side.2"]}]}, "length": 20}
{"id": 5, "name": "Bracket", "type": "sheet_metal", "sketch": 4, "thickness": 1.5, "depth": 30}
```

A flange's faces are named as an extrude's of its outline across the edge: `side.0` the bend's outside, `side.1` the
wall's outside, `side.2` its end, `side.3` its inside and `side.4` the bend's inside, with `start` and `end` at the
flange's two ends. The flange for the second edge a feature is given is `region.1`, and so on. A chain's faces go along
the lines first, a wall and then a bend at each corner, then across the far end, back along the other side, and across
the near end, so `side.0` is the first wall's face on the lines' side.

**Corners.** With `"corner": "closed"`, two flanges of one feature that meet at a corner close it instead. Each one's
bend still stops short, but its wall runs on into the corner: going round anticlockwise, seen from the side they bend
toward, each flange's wall runs over the end of the next one's, out to that one's outside, and the next one's runs up to
within `gap` of the first one's inside. That leaves a square hole where the bends would meet. It's built for flanges of
90 degrees on edges that meet square at an outside corner of the wall, and `gap` has to be more than 0 and less than the
bend radius. Each piece that runs on is named as its flange's wall, and its underside, over the hole, is `side.5`.

**Hems** fold an edge back over its wall, toward the face the edge is on, or the other way with `flip`:

- `straight`: half a turn with `gap` between the wall and what comes back over it, so the fold's inner radius is half
  the gap, then straight to `length`, from the outside of the fold.
- `rounded`: the same at `bend_radius`.
- `teardrop`: a turn of `degrees`, more than 180, at `bend_radius`, then straight on for `length`, leaning back toward
  the wall. One that would come back down onto the wall is refused.
- `rolled`: a turn of `degrees`, more than 180 and less than 360, at `bend_radius`, and nothing after.

`alignment` is as for a flange: `outer` puts the outside of the fold where the edge was, so the part's outside size
stays the same, `inner` puts the inside of the fold there, and `bend` starts the fold at the edge, and the wall is cut
back to make room. Hems on edges of one feature that meet at a corner each stop short of it by as far as they reach
back over the wall, and the part's minimal gap more, so that they don't touch, and so do flanges of more than 90
degrees, which lean back over it. A hem's faces are named as a flange's; a roll has no wall, so its end is `side.1`, the inside of its
fold `side.2`.

**A tab** is a region drawn on the plane of a wall, on either of its faces, made the sheet's thickness through the wall
and joined to it. It can overlap the wall or only meet its side. Where it overlaps the part, that has to be flat wall
square through the sheet: a tab over a bend is refused. Its faces are named as an extrude's of its region.

**A bend along a line** folds a part along the one line in its sketch, which is drawn on the plane of a wall and has to
go right across it, cutting the part in two. The part on the line's left moves, seen from along the sketch's normal, or
on its right with `"moves": "right"`, and it bends toward the side the sketch's normal points to, or the other way with
`flip`. The strip of wall that the bend's neutral line takes when laid flat becomes the bend, so the part's flat
pattern stays just as it was. With `"position": "center"` that strip is centred on the line, and with `start` it starts
at the line, on the side that moves. Everything on the side that moves goes with it, flanges included. The strip has to
be plain flat wall right across, and the side that moves mustn't run into the rest of the part.

```json
{"id": 3, "name": "Sides", "type": "flange", "edges": {"any": [{"near": [50, 0, 2]}, {"near": [100, 30, 2]}]}, "length": 20, "corner": "closed"}
{"id": 4, "name": "Lip", "type": "hem", "edges": {"near": [50, 60, 2]}, "kind": "rounded", "length": 12, "bend_radius": 3}
{"id": 6, "name": "Ear", "type": "tab", "sketch": 5}
{"id": 8, "name": "Fold", "type": "bend", "sketch": 7, "degrees": 90, "moves": "right"}
```

The MCP's `flat_pattern` lays a part out flat. It's unfolded from the plate's face on its sketch, or the first wall's face
on the lines' side, which stays where it is, and each wall beyond a bend is laid down where it would be with the bend
undone, the bend itself a strip as long as its neutral line: its angle times its inner radius plus `k_factor` times the
thickness. Holes cut through walls come with it. It gives the outline and holes, and each bend's middle line, its angle,
inner radius, which way it goes, up toward the side away from the starting face or down, and the strip's width.

## Face keys

A name can cover several faces. A key adds what tells them apart, and each part only narrows, so a plain name is still a key that means all of them:

| Key | Picks |
|---|---|
| `f2:end` | Every face with that name |
| `f2:end@f5.1` | Copy 1 made by pattern or mirror `f5`. Copy 0 is the original |
| `f2:end@f5.1@f7.0` | A copy of a copy: copy 1 of `f5`, in the original of `f7` |
| `f3:side.1[f2:end\|f2:side.0]` | The fillet or chamfer face that replaced the edge between those two faces |
| `f4:side.1[f2:side.1]` | A shell's inner wall following `f2:side.1` |
| `f5:wall.0[f2:side.3]` | The wall an extrude swept from the edge of its first face that's against `f2:side.3` |
| `f5:end[f2:end]` | The far end of the extrude of face `f2:end` |
| `f2:end{f6:side.1}` | The piece of `f2:end` that touches `f6:side.1`, where a cut has left several |
| `f2:side.0{f4:region.1.side.0,f4:region.2.side.0}` | The piece that touches both, where no one face tells it apart |
| `f4:side.0~1` | Piece 1 by where it is, where nothing it touches tells it apart |
| `f4:side.0#1` | Face 1 by where it is, as files written before 2026-11-20 count them. Still read, no longer written |

`face_keys(body)` gives each face a key, and `find_faces(body, key)` goes the other way. `{"on_face": key}` and `{"between": [key, key]}` take keys, so a
fillet can round one copy. `shell` and `draft` still take plain names, and refuse a key.

- **A whole round face is one face.** The kernel keeps a whole cylinder, cone, sphere or turned face as two halves, and a key picks both,
  as nothing but where the kernel put its seam tells them apart. The same goes for any faces with the same name, on one surface, that meet:
  they're one piece. So a rim between such a face and another is one edge, `{"between": ["f2:end", "f4:side.0"]}`.
- **A piece is told apart by what it touches,** not where it is, so that it keeps its key when a dimension moves pieces past each other, or
  another piece comes or goes. What it touches is named without saying which piece, and faces from earlier features are chosen first, as what
  a part started as lasts longer than what was done to it later.
- **A key for one piece that comes to fit several is refused,** saying it can't tell which was meant, rather than being read as all of them.
  So is a key whose piece has gone.
- **A key is worked out from the model and isn't stored,** except where a selection uses one. It adds nothing to files that don't.
- **A copy number stays with the copy,** so `@f5.1` stays the first copy when the pattern's count or spacing changes, and a key to a copy that
  a smaller count leaves out is refused.
- **Position is the last resort,** `~1`, for pieces that touch the same things, such as the two halves of a slot's wall that a bore cuts in
  two. It counts only the pieces that agree on everything else, by their place in the surface's own coordinates, which move with the part:
  first coordinate, then second, then third, to 10 nanometres. It goes wrong if a dimension moves them past each other along the coordinate
  that decides.
- **A face merged across a mirror plane** keeps the original's key, since it's now both.

## Sketch regions

```json
{"outer": <shape>, "holes": [<shape>, ...], "repeat": <repeat>}
```

| Shape `type` | Fields |
|---|---|
| `rect` | `x y` (default 0), `w h` |
| `rounded_rect` | `cx cy` (default 0), `w h r`. With `r` half of `w` or `h` it's a slot, and each side that's left keeps its number |
| `circle` | `cx cy` (default 0), `r` |
| `polygon` | `points`: a list of `[x, y]` |
| `path` | `start`, `moves`: `{"line_to": [x, y]}`, `{"arc_to": [x, y], "center": [x, y], "ccw": true}`, or `{"spline_through": [[x, y], ...]}`, a smooth curve on from where the path is. Closes itself with a line if needed. |
| `ellipse` | `cx cy` (default 0), `rx ry`, `degrees` (default 0) to turn it |

Repeats make copies, each of which counts as its own region:

- `{"type": "circular", "count": n, "radius": r, "start_degrees": a}`: the region is drawn centred on
  the sketch origin and moved out to evenly spaced angles.
- `{"type": "grid", "nx": a, "ny": b, "dx": x, "dy": y}`: copies from where the region is drawn.

## Edge selections

| Form | Meaning |
|---|---|
| `"all"` | Every edge |
| `{"between": ["f2:end", "f2:side.0"]}` | Edges between two named faces, or faces two keys pick |
| `{"between": [a, b], "ends": [c]}` | Where faces `a` and `b` meet more than once: the place whose ends touch `c` |
| `{"between": [a, b], "nth": 1}` | Where nothing at the ends tells them apart: place 1 by where it is, as for `~1` above |
| `{"on_face": "f2:end"}` | Edges bounding a named face |
| `{"near": [x, y, z], "tolerance": d}` | Edges whose midpoint is within `d` (default 0.001) |
| `{"in_box": [[x, y, z], [x, y, z]]}` | Edges whose midpoint lies in a box |
| `{"parallel": [x, y, z]}` | Straight edges along a direction |
| `{"radius": r}` | Circular edges of a radius |
| `{"any": [...]}`, `{"both": [...]}` | Union and intersection of selections |

A place where two faces meet is a chain of edges on one curve, end to end, such as both halves of a rim, and a selection picks all of it.

**What an app saves for a click** is what `edges_list` gives as each edge's `select`: the two faces' names if they meet nowhere else, their keys
if the names fit other places, `ends` or `nth` if even the keys do, and `near` only for a sheet's open edge, which has one face.

**A list of picks needs each one.** In `{"any": [...]}`, a pick that names something, `between`, `on_face` or `near`, and picks nothing now
makes the feature refuse, naming the pick, rather than leaving that edge out without saying so. A query that asks for whatever fits,
`radius`, `parallel` or `in_box`, may pick nothing. A selection on its own that picks nothing is refused, as before.

## Bodies

A body is a part: one connected solid, with any voids inside it, or a connected [sheet](#sheets). This works as Onshape's parts and merge scope do.

- **A feature acts on all the bodies unless it lists them,** in `bodies`, which is Onshape's merge scope: `{"id": 8, "type": "extrude", "mode": "remove", "bodies": ["f3.2"], ...}`. It leaves alone any it doesn't reach.
- **`mode`** says what a new solid does to them. `add` joins whatever it touches, into one under the first one's name, and if it touches nothing it's a new part. `remove` and `intersect` cut what they reach, and a part with nothing left is gone. `new_body` joins nothing, so that parts can touch or overlap. Regions of one sketch join if they touch, and are separate parts if they don't. Each solid of a STEP file is a part.
- **Features that pick faces or edges** work on whichever parts those are on.
- **A part's name** is the feature that made it, and which of that feature's parts it is, counting from 0: `f3` for the first or only one, and `f3.2` for the third. Whatever is done to it later, it keeps that name.
- **A cut that splits a part** leaves the name on the piece with the most faces from before, or the larger of those, and the others are named for the feature that split it. So after a change that swaps which piece that is, the names swap too.
- **A change to one part leaves the others alone,** when what comes after names which parts it acts on. A feature with no list depends on all of them.

## Extrude regions and ends

An extrude can choose what it takes and where it stops, as Onshape's can. These are all optional, and an extrude without them is written as it always was.

**`regions`** takes these regions rather than whatever its sketch gives, so that one sketch can feed many extrudes without changing it. Ids are from the `sketch_arrangement` tool, and last through changes to the sketch's dimensions.

```json
{ "id": 5, "type": "extrude", "sketch": 2, "regions": ["r0123456789ab", {"sketch": 3, "region": "r00000000abcd"}], "length": 10, "mode": "new_body" }
```

A plain id is from `sketch`, and `{"sketch", "region"}` is from any constrained sketch before it, in which case `sketch` can be left out. Each goes along its own sketch's normal, and regions that touch become one part. A region that has gone fails the feature, saying what's there, and so does a sketch that's suppressed.

**A revolve takes `regions` the same way.** With an `axis`, the axis is drawn in the revolve's `sketch`, or the first region's sketch if that's left out, and a region from another sketch has to lie in the same plane, facing either way. One that doesn't is refused, saying how far off it is. Without an `axis`, each region goes around its own sketch's Z axis. A second sketch's faces are named `f5:region.1.side.N`, as an extrude's are.

**`end`** says where it stops, in place of `length`. If there's a `length` as well, it's kept, unused, for going back to it.

| `end` | Stops |
|---|---|
| `"blind"` | At `length`. This is the default |
| `"through_all"` | Past every part it acts on. A new solid stops flush with the furthest |
| `"up_to_next"` | At the nearest surface in the way among the parts it acts on. A cut goes on through the material to its far side |
| `{"type": "up_to_face", "face": "f1:end"}` | At that face's surface, extended as far as it needs to be. A plane, cylinder, cone, sphere or torus. `face` can also be a [plane](#planes): `"top"`, `"front"`, `"right"`, a plane feature's id, like `4`, or `{"point": [x, y, z], "normal": [x, y, z]}` |
| `{"type": "up_to_part", "part": "f3"}` | At that part's nearest surface, going on past anything else |
| `{"type": "up_to_vertex", "vertex": {"faces": [...], "near": [x, y, z]}}` | At a plane through the vertex those faces share, square to the direction. `near` says which if they share several. `edges_list` gives these for each edge's ends as `start_vertex` and `end_vertex` |
| `{"type": "up_to_vertex", "vertex": {"point": [x, y, z]}}` | The same, through a place |

**`"opposite": true`** goes the other way. An end behind the sketch is refused, and the message says so. The far end is named as a blind extrude's is, however it's shaped: `f5:end` going forward, and `f5:start` in the opposite direction, as the kernel names caps by where they are.

### Extruding faces

**`faces`** takes flat faces of parts, by name or [key](#face-keys), as Onshape's extrude takes faces as well as sketch regions. It can be on its own, with no `sketch`, or with `regions` or a `sketch`, whose regions go as they always did. Each face goes along its own outward normal, holes and all, so the top of a block goes up and its side goes sideways. `start`, `end` and `opposite` work as they do for a sketch, with the face where the sketch would be: `"opposite": true` and `"mode": "remove"` cut into the part.

```json
{ "id": 6, "type": "extrude", "faces": ["f2:end", "f3:side.1"], "length": 5, "mode": "add" }
```

The new faces are named by this feature, as a sketch's are, and their keys say where each came from:

| Name | Key | Face |
|---|---|---|
| `f6:end`, `f6:start` | `f6:end[f2:end]` | The far end and the near one, holding the face they came from |
| `f6:wall.0` | `f6:wall.0[f1:side.3]` | A wall swept from an edge of the faces picked by the first entry in `faces`, holding the face across that edge. `wall.1` is for the second entry, and so on |

A name that picks several faces, such as a patterned boss's top, extrudes them all, and their walls share the entry's `wall.N`. A face two entries pick goes once, named for the first. A whole circle's edge gives two half walls, as a whole circle always gives two half faces, and they're one face with one key.

With `"mode": "add"`, a wall that carries straight on from a face of the part, as the walls over a block's top do, becomes part of that face, which keeps its name. So `f6:wall.0` names only the walls that stand out on their own, and with `new_body` or `surface` it names them all.

A face has to be flat, and bounded by edges with an exact form: lines, arcs, ellipses and splines. An edge that's an intersection curve held as samples, such as where a plane cuts a free-form face, is refused, saying which face. So is a curved face, and a face that has gone.


### Lofts

Besides `sections`, a loft can take:

| Key | Value | What it does |
|---|---|---|
| `from` | a point: `[x, y, z]`, `"origin"`, a part's corner `{"faces": [...], "near": [x, y, z]}` as `vertices_list` gives it, or a sketch's point `{"sketch": 4, "point": "7"}` | Starts at that point, before the first section, as a cone starts at its tip. `sections` can then be a single outline |
| `to` | a point, as `from` | Ends at that point, after the last section |
| `closed` | `true` | Goes on from the last section back round to the first, as a ring, with no caps. At least three sections, and no points or ends |
| `guides` | a list of sketch ids | Each open chain of curves in each sketch is a guide the sides follow. A guide has to run through every section's outline, crossing each section's plane on its outline, and to the point if there is one. Where it meets an outline between corners, the outline is cut there. Either way round will do |
| `start`, `end` | `"normal"`, `{"normal": 1.5}`, `"round"` or `{"round": 1.5}`, or `"free"`, the default | `normal`: leave the first section, or come into the last, square to its plane. `round`: round over a point at that end, as a dome does, level there. The number is how hard it pulls, an expression above 0 and at most 100: at 1, as fast as going straight from end to end |

```json
{ "id": 9, "type": "loft", "sections": [5], "to": [0, 0, 30], "end": "round" }
{ "id": 10, "type": "loft", "sections": [5, 6, 7, 8], "closed": true }
{ "id": 11, "type": "loft", "sections": [5, 6], "guides": [12], "start": {"normal": 1.5} }
```

The sections are still gone through exactly: guides bend what's between them, and set ends only change how it leaves
them. A thin loft can be closed or have its ends set, but can't go to a point or follow guides yet. A loft can't yet both
follow guides and set its ends, or be closed and follow guides: see [KNOWN_LIMITS.md](KNOWN_LIMITS.md#free-form-surfaces-and-lofts).

### Thin walls

**`"thin": {"thickness": t, "side": "inside"}`** on an extrude makes a wall along the sketch's outlines and open curves, in place of its regions, as Onshape's thin extrude does: a ring round each outline of each region it takes, and a strip along each open curve of a whole sketch, with square ends. `side` is `inside` (the default), into the region an outline goes round, or to the left of an open curve as it's drawn; `outside`, the other way; or `both`, half each way. Everything else about the extrude, its `regions`, `end`, `opposite` and `mode`, works as it does without it, so a thin cut `through_all` makes a groove.

```json
{ "id": 4, "type": "extrude", "sketch": 3, "length": 20, "thin": {"thickness": 1.5, "side": "both"} }
```

A `revolve`, a `sweep` and a `loft` take `thin` too, and do with the walls what they do with regions: turn them round the axis, take them along the path, or go through them section by section.

- **Revolve:** an open curve from the axis, square to it, turns into a cup with a floor: its wall's end there lies on the axis. One that leaves the axis at a slant, or any wall that reaches across it, is refused.
- **Sweep:** a circle makes a tube, and an open curve a strip or a gutter. A bend tighter than the wall reaches round is refused.
- **Loft:** each section's walls are made in its own plane, as thick as it says there, and the loft goes through them: the first wall of each section, then the second, and so on, so each section needs the same outlines and curves in the same order, and a section that has an outline where the first has an open curve is refused, saying which. Between sections the wall is what the loft makes of them, so where the loft runs at a slant to its sections, its wall is thinner than `thickness` measured square to it.

```json
{ "id": 7, "type": "loft", "sections": [5, 6], "thin": {"thickness": 1.5} }
```

For a revolve and a sweep, the sketch's plane is square to the wall, so the wall is `thickness` thick everywhere.

Each curve is moved sideways exactly: a line along its normal, an arc to a larger or smaller radius about the same center. Where the moved curves part at a corner, each is carried on to where they cross, so a wall's corners stay sharp, and where they overlap, each is cut back to the same point. A wall follows lines and arcs, and a spline or an ellipse is refused, as one moved sideways isn't one. So is a wall thicker than an arc is round, a corner so sharp that its outside would reach out more than 50 times the thickness, and a wall too thick for a segment between two corners, whose other side would run backwards: each says where. It can't take a part's `faces`, or make a surface.

The faces on the right of a curve, as drawn, keep the names an extrude would give them, so on an outline that's the side away from the region. Those on its left are named `inner.` and the same, and a strip's ends after its first and last segments:

| Name | Face |
|---|---|
| `f4:side.3`, `f4:hole.0.side.1`, `f4:region.1.side.0` | The wall's side to the right of that segment |
| `f4:inner.side.3`, `f4:inner.hole.0.side.1`, `f4:region.1.inner.side.0` | Its side to the left |
| `f4:start_of.side.0`, `f4:end_of.side.4` | The ends of a strip along an open curve that starts with segment 0 and ends with segment 4 |
| `f4:start`, `f4:end` | The ends of the extrude, as always |

A constrained sketch's segments are its entities, so a wall's faces follow them through changes to its dimensions. One sketch can give a thin feature at most 200 regions and open curves.

## Ribs

A rib is a wall along each of a sketch's open curves, `thickness` thick, half each side, that runs until it meets the parts it acts on, and is joined to them, as Onshape's rib is.

```json
{ "id": 7, "type": "rib", "sketch": 6, "thickness": 2, "direction": "parallel", "flip": true }
```

With `"direction": "parallel"`, the default, it runs across from each line in the sketch's plane: to the line's left as it's drawn, or its right with `flip`, and is as thick across the plane. That's a gusset: sketch a line across the corner of a bracket on the plane through its middle, and the rib fills the triangle between the line and the bracket. It follows straight lines, and an arc or a spline is refused. With `"direction": "normal"` it runs along the sketch's normal, or against it with `flip`, and follows lines and arcs: a line sketched on an open box's rim gives a rib down to its floor.

With `extend`, the default, each curve's ends carry straight on until they meet the parts too, so a line short of a box's walls still reaches them. Without it the rib stops where the curve does. Each rib is what's left of a slab along its curve once the parts are cut out of it, next to the curve, so a curve that runs into a part stops there. One that runs on past the parts without meeting them is refused, saying which way it went, rather than left sticking out, and so is a curve inside a part. Its faces are named as a [thin wall's](#thin-walls) are: `side.N` and `inner.side.N` on its two faces along segment N, and `start` on the face where it begins: in the sketch's plane for a rib along the normal, and along the line for one across from it.

## Holes

A hole feature drills a hole at each of a sketch's points, or at points on a plane, as Onshape's does, and cuts them from the
parts it acts on.

```json
{"id": 7, "type": "hole", "sketch": 6, "style": "counterbore", "size": "M6", "end": "through_all"}
{"id": 8, "type": "hole", "plane": "f2:end", "points": [[10, 10], ["w - 10", 10]], "size": "M5", "tapped": {"depth": 10}, "depth": 14}
```

- **Where.** A constrained sketch's points: those `points` names, as constraints name them, so a circle's center or a line's end
  will do, or with no `points` every point entity in it. Or `[x, y]` points on any [plane](#planes): on a face's plane, they move
  with the face. Each hole goes in against the sketch's or the plane's normal, which for a face's plane is into the part, or along
  it with `opposite`.
- **Where it starts.** Where its axis first meets the parts, going in, even if the sketch is above the part or inside it: then
  it's where the axis comes out behind the sketch. On a curved or slanted face, its top, at its widest, carries on back from
  there as far as the face it goes in through stands proud within that width, and a fiftieth more, so it opens cleanly. On a flat
  face square to it, nothing is carried back. Something standing over the hole is left alone, and all beyond it on that side:
  a face that, going out from the axis, rises steeper than 80 degrees, such as a wall or a rib with a draft, which the hole
  undercuts if it's wider than the floor at the wall's foot. A countersink carries on back as a cylinder at its
  diameter, not as its cone, which would cut a slanted face ever wider. With `from_sketch` it starts at the sketch's plane
  instead, and goes no further back; one that would then leave a sealed hollow inside a part is refused.
- **What it's like.** `simple`, `counterbore` (`cbore_diameter`, `cbore_depth`) or `countersink` (`csink_diameter`, and
  `csink_angle`, default 90). `diameter` is the hole's own. A blind hole has a drill point, `tip_angle` degrees (default 118), or
  a flat bottom at 180. A `size` fills in whatever isn't given, from the tables below.
- **Tapped.** `"tapped": true`, or `{"pitch": p, "depth": d}`, needs a `size`. It's drilled at the tap drill, and the thread
  isn't modelled: it's in the outcome's `holes`, with its size, pitch and depth, as a cosmetic thread. The pitch is the size's
  coarse one by default. A fine one takes ISO 2306's drill for it, and one ISO 2306 doesn't list for the size is drilled at the
  major diameter less the pitch, with a note saying so. The thread goes as deep as the hole by default, or through.
- **How deep.** `"end": "blind"` with `depth`, measured from where the hole starts to the end of its full diameter, the drill
  point going on past it; `through_all`; or `up_to_next`, through the first wall it meets: as deep as rays down its middle and
  in two rings of 24 round it find that wall's far side, and half way on to what's behind. A blind hole deeper than the part
  along its axis goes through, as asked, and the feature notes it. Parts that touch are one wall. If what's behind
  comes within the hole's width before the wall ends all round, as with a hole half over a step, it's refused.
- **Names.** Holes are numbered by their point's entity id when the feature takes every point of a sketch, so that deleting
  one doesn't renumber the others, and otherwise by their place in `points`. Hole 7's faces are `bore.7` (its own diameter),
  `cbore.7` and `cbore_floor.7`, `csink.7` (its cone, and the cylinder carried back from it), `tip.7` (a drill point),
  `floor.7` (a flat bottom) and `top.7` (the end of the top carried back, which shows only where something stands over the
  hole, as the ceiling of what the hole takes from under it). Two points at the same place make one hole,
  and the feature notes it.
- **Callouts.** The outcome's `holes` list each hole: where it starts, which way it goes, and what a drawing would say, such
  as `M6 clearance, ⌀6.6 through, counterbore ⌀11 6.4 deep` or `M6×1 tapped 12 deep, drill ⌀5 15 deep, 118° point`.

| Size | Clearance, close / normal / loose | Tap drills (pitch) | Counterbore ⌀ × depth | Countersink ⌀ |
|---|---|---|---|---|
| M1.6 | 1.7 / 1.8 / 2 | 1.25 (0.35) | 3.5 × 2 | |
| M2 | 2.2 / 2.4 / 2.6 | 1.6 (0.4) | 4.4 × 2.4 | |
| M2.5 | 2.7 / 2.9 / 3.1 | 2.05 (0.45), 2.15 (0.35) | 5.5 × 2.9 | |
| M3 | 3.2 / 3.4 / 3.6 | 2.5 (0.5), 2.65 (0.35) | 6.5 × 3.4 | 6.94 |
| M4 | 4.3 / 4.5 / 4.8 | 3.3 (0.7), 3.5 (0.5) | 8 × 4.4 | 9.18 |
| M5 | 5.3 / 5.5 / 5.8 | 4.2 (0.8), 4.5 (0.5) | 10 × 5.4 | 11.47 |
| M6 | 6.4 / 6.6 / 7 | 5 (1), 5.2 (0.75) | 11 × 6.4 | 13.71 |
| M8 | 8.4 / 9 / 10 | 6.8 (1.25), 7 (1), 7.2 (0.75) | 15 × 8.6 | 18.25 |
| M10 | 10.5 / 11 / 12 | 8.5 (1.5), 8.8 (1.25), 9 (1), 9.2 (0.75) | 18 × 10.6 | 22.73 |
| M12 | 13 / 13.5 / 14.5 | 10.2 (1.75), 10.5 (1.5), 10.8 (1.25), 11 (1) | 20 × 12.6 | 27.21 |
| M14 | 15 / 15.5 / 16.5 | 12 (2), 12.5 (1.5), 12.8 (1.25), 13 (1) | 24 × 14.6 | 31.19 |
| M16 | 17 / 17.5 / 18.5 | 14 (2), 14.5 (1.5), 15 (1) | 26 × 16.6 | 33.99 |
| M20 | 21 / 22 / 24 | 17.5 (2.5), 18 (2), 18.5 (1.5), 19 (1) | 33 × 20.6 | 40.71 |
| M24 | 25 / 26 / 28 | 21 (3), 22 (2), 22.5 (1.5), 23 (1) | 40 × 24.8 | |
| M30 | 31 / 33 / 35 | 26.5 (3.5), 27 (3), 28 (2), 28.5 (1.5), 29 (1) | 50 × 31 | |
| M36 | 37 / 39 / 42 | 32 (4), 33 (3), 34 (2), 34.5 (1.5) | 58 × 37 | |

The first tap drill is for the coarse pitch, the default. Each column is from a standard, read on 2026-11-22 from a copy or a
reproduction of it: clearance holes are ISO 273's fine, medium and coarse series; pitches and tap drills are ISO 2306's tables 1
and 7; a counterbore is DIN 974-1:2008's series 1, for an ISO 4762 socket head cap screw with no washer; a countersink is
DIN 74:2003's form F, 90°, for an ISO 10642 countersunk socket head. DIN 974-1 leaves a counterbore's depth to the designer and
recommends the head's height, which for ISO 4762 is the size, and an allowance: 0.4 from M3 to M6, 0.6 from M8 to M20, 0.8
at M24 and 1 at M36, as one table of depths gives them, and M5's also a second source; 0.4 below M3 and 1 at M30 are assumed.
So a head sits just under the face, and a
countersunk head too, as form F is a little wider than the head. There's no countersink where form F has none. Where each
was read, and how sure each is: [KNOWN_LIMITS.md](KNOWN_LIMITS.md#holes).

## Sheets

A sheet is an open surface that's a body of its own. It has faces and edges like any body, and no inside: its volume is zero unless it closes. An edge with a face on only one side is its boundary. See [SHEET_BODIES_DESIGN.md](SHEET_BODIES_DESIGN.md).

**Making one:**

| Feature | What it makes |
|---|---|
| `fill` | A flat sheet for each of a sketch's regions |
| `extrude`, `revolve` with `"mode": "surface"` | The sides of what a region's outline sweeps, with no ends, and a sheet from each open curve in the sketch |
| `sweep`, `loft`, `helix` with `"mode": "surface"` | The sides of what they'd have made, with no ends |
| `copy_faces` | A copy of faces from a solid or another sheet |
| `import_step` | A file holding only surfaces comes in as sheets, in any mode. With `"mode": "surface"` it takes the surfaces from beside a file's solids, and leaves the solids |

Each connected piece is a body of its own, and each depends on nothing else in the document. A sheet from an open curve faces to the right of the way the curve runs, seen from along the sketch's normal, and its faces are named for their segments, as on a solid: `f3:side.0`.

**An open curve** in a plain sketch is a region whose `outer` is an `open_path`, with `start` and `moves` as a `path` has them, and nothing else in the region. In a constrained sketch it's any curves that close nothing, joined end to end into chains. A chain stops where three curves meet, so that no edge has three faces. A constrained sketch's open curves are refused, as they always were, unless a `surface` feature uses that sketch.

**Keeping out of the way.** A feature with no `bodies` acts only on solids, as Onshape's merge scope does, so a sheet lying around doesn't get in the way of a cut or a fillet. `transform`, `thicken`, `copy_faces`, `sew` and `trim` take sheets as well, and a `pattern` or `mirror` of a feature that made one makes more. A sheet can be named in `bodies` by anything, and what needs a solid then refuses it, saying so: `boolean`, `fillet`, `chamfer`, `offset`, `shell`, `draft`, `delete_face`, and `remove`, `intersect` or `add` when they reach it.

**From a sheet to a solid.** `thicken` takes a sheet's faces by name, or with `"faces": []` and `bodies`, all the faces of the sheets it names. `sew` joins sheets along boundaries that run together, cutting an edge to fit where one runs along part of another, and turning over whatever faces the wrong way. If nothing is left open, what comes out is a solid. Otherwise it's still a sheet. It keeps the first one's name.

**Cutting.** `trim` cuts a sheet where a solid's surface, a plane or a sketch's outline crosses it. A piece keeps its face's name, and a sheet cut in two is two bodies. `outside` is what's outside a solid or a sketch's regions, or on the side a plane's normal points to, and `inside` is the rest. A sheet lying exactly on a solid's face or a plane isn't inside, so it's with `outside`, and the two parts always make up the sheet. With `both`, the part outside keeps the sheet's name.

## Projects and assemblies

A project is an Onshape document: tabs of Part Studios and assemblies. **A project with only one Part Studio is written as the plain document it always was,** and a reader takes either.

```json
{ "format": "coreshape-project", "version": 1, "next_id": 3, "tabs": [
  { "id": 1, "name": "Part Studio 1", "type": "part_studio", "document": { "format": "coreshape-document", "...": "..." } },
  { "id": 2, "name": "Assembly 1", "type": "assembly", "assembly": { "next_id": 7, "instances": [], "connectors": [], "mates": [], "groups": [], "relations": [] } }
] }
```

An assembly never changes a part. It holds uses of parts and where they are, so a part changed in its Part Studio changes in every assembly, what's mated to it follows, and nothing about moving an instance goes in a history. It can also have an `app`, kept for an app as a document's is.

| Thing | Fields |
|---|---|
| Instance | `id`, `name`, `source` as `{"tab": 1, "body": "f3"}` or `{"assembly": 2}`, `at` as `{"origin", "z", "x"}`, and `fixed`, `suppressed`, `locked` if true |
| Connector | `id`, `instance` as a list of ids down to a part, `on`, and `offset`, `rotation` in degrees, `flip`, `quarter_turns`, `implicit` if set |
| Mate | `id`, `name`, `type`, `a` and `b` as connectors' ids, and `offset`, `rotation`, `flip`, `quarter_turns`, `limits`, `suppressed`, `turned` if set |
| Group | `id`, `name`, `instances` as lists of ids, `suppressed` if true |
| Relation | `id`, `type`, `a` and `b` as mates' ids, its amount, and `reverse`, `offset`, `suppressed` if set |
| Pattern | `id`, `name`, `of` as top-level instances' ids, `type` `linear` with `spacing` or `circular` with `degrees`, `count` in all, the instances first, `axis` as `{"point", "direction"}` or `{"connector": id}`, and `suppressed` if true |
| Replicate | `id`, `name`, `of` as top-level instances' ids, `connector` they're placed at, `at` as a list of more `on` places on that connector's instance, or left out for every face like the connector's, and `suppressed` if true |
| Exploded view | `id`, `name`, `steps`, each `{"instances": [ids or paths], "shift": [x, y, z], "turn": {"point", "axis", "degrees"}}`, under `explodes` |
| Named position | `id`, `name`, `values`, each `{"mate": id, "dof", "value"}` in millimetres and degrees, under `positions` |

Patterns, replicates, exploded views and named positions are written only when there are some, so other assemblies save as
they did.

**A connector's `on`** is what it's found from, so it follows the part:

| `on` | Where |
|---|---|
| `{"face": "f1:end"}` | The centroid of a flat face, with z straight out of it, or the middle of a round face's axis, with z along it. `"point"` can also be `axis_start`, `axis_end` or `apex` |
| `{"face": "f1:end", "near": [x, y, z]}` | The nearest of the face's corners, edge middles and arc centers |
| `{"edge": ["f1:end", "f1:side.0"], "point": "middle"}` | On the edge between two faces: `middle`, `start`, `end`, or `center` for a round one. Its z runs along a straight edge, and along the axis of a round one |
| `{"between": ["f1:side.0", "f1:side.2"]}` | Midway between two parallel flat faces: the first's centroid moved halfway to the second's plane, z along the first's normal. For a width mate, across a slot and across a tab |
| `{"frame": {"origin", "z", "x"}}` | Just a place, in the part's own coordinates, which follows nothing |

If what it's found from has gone, that mate fails, says which face, and the rest of the assembly still solves. The first of a key's faces is used if it covers several.

**A mate's `type`** says what stays free between its two connectors, whose z axes end up pointing the same way:

| `type` | Free |
|---|---|
| `fastened` | Nothing |
| `revolute` | Turn about z |
| `slider` | Slide along z |
| `cylindrical` | Turn about z, slide along z |
| `pin_slot` | Turn about z, slide along x |
| `planar` | Turn about z, slide along x and y |
| `ball` | Any turn |
| `parallel` | Turn about z, any slide |
| `tangent` | Whatever keeps two faces touching: each connector on a face, flat, cylindrical or spherical. The other part goes outside a flat face, on the side its normal points, and round faces go outside each other; `flip` takes the other side, or one inside the other. A cylinder lies level on a plane, and two cylinders keep their axes parallel |
| `width` | Everything but `b`'s origin leaving `a`'s xy plane: with both ends `between` two faces, a tab centered in a slot |

Two faces' z axes both point out of them, so two faces need `flip` to lie against each other. `offset` is from `a` to `b` along `a`'s axes, `rotation` is about `a`'s z, and `flip` and `quarter_turns` act on `b`. `limits` is a list of `{"dof": "x", "y", "z" or "angle", "min", "max"}`, in millimetres and degrees. `turned` is how far it has turned in radians, with its turns, kept so that the next turn can be told from this one.

**A relation** ties mates' motions together, as `b = ratio * a + offset`:

| `type` | Between | Amount |
|---|---|---|
| `gear` | Two that turn | `ratio` |
| `rack_and_pinion` | One that turns, and one that slides | `distance` for each whole turn |
| `screw` | One cylindrical mate, named as both | `lead` for each whole turn |
| `linear` | Two that slide | `ratio` |

`offset` is set when it's added, from where they are, so that adding it moves nothing.

**A pattern or a replicate** makes copies of instances. Each copy's path is the pattern's or replicate's id, which copy it
is from 1, and the instance's own path, so `[12, 2, 4]` is pattern 12's second copy of instance 4. Copies are placed after
the rest is solved, from wherever their instances are, so they follow them and their mates, and they can't be mated
themselves. A pattern's axis given as a connector follows that connector's part. A replicate places each copy on its
place as the instances are placed on the connector, turned to point the way the connector does, and without `at` finds
its places itself: every other face of the connector's instance like the connector's own, a cylinder of the same radius
and the same way, a ball of the same radius, or a flat face facing the same way. Each place is found from its face's key,
so the copies follow the part when it changes.

**An exploded view** only moves what's shown. Its steps go one after another, each moving the instances it lists,
everything under them included, by its shift and turn, in the assembly's coordinates; shown part of the way through, each
step moves its share.

**A named position** keeps each top-level mate's values. Going back to one takes all of them there together, a little at a
time, and stops where other mates stop them, as taking one mate to a value does.

**A sub-assembly** is an instance of an assembly tab, and as in Onshape its mates go on working in the assembly above, so its parts can move there. Where they've got to is kept in that assembly's `moved`, by path, relative to the top-level instance they're in, so moving the instance takes them along, and how far its mates have turned there in `moved_turns`, by the sub-assembly's path and the mate's id. `fixed` counts only in the assembly it's set in. `locked` keeps the parts as their own tab solves them, moving as one. A connector reaches into a sub-assembly by a longer `instance` list, which has to go all the way down to a part.

## Rules a reader enforces

- A feature may only refer to features before it, including through `bodies`, and `sketch` must name a sketch.
- Parameters may refer to each other but not in a cycle.
- A version other than this reader's is refused with a message saying so.
- Everything an assembly refers to has to be there: its mates' connectors, its relations' mates, and its instances' tabs, as the right kind of tab. An id can't appear twice.

## Constrained sketches

A `constrained_sketch` feature holds a sketch drawn with constraints, which is solved at each rebuild:

```json
{"id": 1, "name": "Outline", "type": "constrained_sketch", "plane": {"origin": [0, 0, 0]},
 "sketch": {
   "format": "coreshape-sketch", "version": 1,
   "entities": [
     {"id": 1, "type": "line", "start": [0, 0], "end": [30, 1]},
     {"id": 2, "type": "line", "start": [30, 0], "end": [30, 20]},
     {"id": 3, "type": "arc", "center": [0, 0], "radius": 5, "start": [5, 0], "end": [0, 5]},
     {"id": 4, "type": "circle", "center": [10, 10], "radius": 3},
     {"id": 5, "type": "point", "at": [0, 0], "construction": true}
   ],
   "constraints": [
     {"id": 1, "type": "coincident", "a": "1.end", "b": "2.start"},
     {"id": 2, "type": "horizontal", "a": "1"},
     {"id": 3, "type": "distance", "a": "1", "value": "width / 2"}
   ]
 }}
```

A face swept from entity 7 of feature 3 is named `f3:side.7`, whatever order things were drawn in. Lines,
arcs and circles that aren't `construction` are cut at every crossing and T-junction, and each area they
enclose is a region with a permanent id. By default a sketch builds from every other level of nesting, so a
loop inside another is a hole, and a loop inside a hole starts a new region. Curves that enclose nothing
are left out without failing. A sketch with no closed region at all, such as a few points or one line to place a plane
by, or a sweep's path, still builds, so that planes, sweeps and constraints can name its points and lines. Only a feature
that needs a region from it fails: an extrude, revolve, loft, sweep, helix, fill or trim says that it needs one, and why
the sketch has none, such as a free end. To build from particular regions, list them in an optional top-level
`"picked_regions": ["r0123456789ab"]`. If one of them stops existing, the feature fails and says so. The
field is only written when something is picked, so sketches with no picks are saved as before. The geometry
is only a starting position: it needn't meet the constraints, and the nearer it is, the likelier it lands on
the solution you meant.

An arc runs counter-clockwise from `start` to `end`.

An `ellipse` has `center`, `major` (the point where its major axis ends) and `minor` (the other radius). An `elliptical_arc`
adds `start` and `end`, and runs counter-clockwise between them. `"7.major"` is the end of the major axis.

A `spline` has `points` (its control points), with optional `degree` (default 3), `knots` (default evenly spaced, and always clamped) and `weights`
(default all 1). By hand, `{"type": "spline", "through": [[0, 0], [5, 4], [10, 0]]}` makes the smooth curve through those points, and it's saved as
control points after that. `"7.p2"` is control point 2, and `"7.start"` and `"7.end"` are the first and last.

### Text

Text goes in an optional top-level list, `texts`, beside `entities`:

```json
"texts": [{"id": 9, "at": 7, "up": 8, "text": "M6 \u00d7 20\nSTEEL", "font": "sans"}]
```

Each is placed by two of the sketch's point entities, so constraints place it as they place anything: `at` is on its
baseline where its first character starts, and `up` is straight above `at`, by the height of a capital letter. A distance
between them sizes it, and making them vertical keeps it upright; turning `up` round `at` turns it. Its characters' outlines
are curves of the sketch, which make regions as any curves do: a letter is material and the inside of its loop, as in `o`, a
hole, by the usual every other level of nesting, and text inside a closed outline is a hole in it. The faces they sweep are
named by the text's id, `f3:side.9`, which is one of the sketch's entity ids. Lines after a `\n` are a line's spacing lower.
Removing either point removes the text, and removing the text removes its points.

The only font is `sans`, the default: 220 characters of Noto Sans Regular, printable ASCII, Latin-1 and a few more, under the
SIL Open Font License that `crates/sketch/fonts/OFL.txt` holds. A character it hasn't got is left out with a space's room,
and the sketch's feature notes it; `⌀`, which it hasn't got, is set as `Ø`. See
[KNOWN_LIMITS.md](KNOWN_LIMITS.md#text-in-sketches).

### Projected geometry

A `projected` entity is part geometry seen along the sketch's normal, as Onshape's links to a part and its Use make: a point on a
part's corner, or a part's edge brought in as a curve. It's worked out again at every rebuild, from the model as it stands just
before the sketch, so whatever holds to it follows the part when the part changes.

```json
{"id": 9, "type": "projected", "from": {"vertex": {"faces": ["f2:end", "f2:side.0", "f2:side.1"], "near": [x, y, z]}}}
{"id": 10, "type": "projected", "from": {"edge": {"between": ["f2:end", "f2:side.0"]}}, "construction": false,
 "geometry": {"type": "line", "start": [0, 0], "end": [40, 0]}}
```

- **`from`** is a vertex, as `vertices_list` gives it and an extrude's `up_to_vertex` takes it, or an edge selection that picks
  one edge, as `edges_list` gives it in `select`.
- **What it becomes:** a vertex is a `point`. An edge is whatever its projection is: a `line`, an `arc` or `circle`, an `ellipse`
  or `elliptical_arc`, or a `spline`. An edge seen end on is a `point`, and a round edge seen edge on is a `line` over as far as
  it reaches. A round edge is never a whole circle, as the kernel splits one into halves, so a hole's rim is two arcs, each
  projected on its own. An intersection curve the kernel only holds as samples is refused, as it has no exact form.
- **It's fixed.** It takes no freedom, nothing in the sketch moves it, and `sketch_drag` refuses to drag it. A constraint between
  two of them that doesn't already hold can't be met, and says so.
- **Constraints use it like anything else,** with its parts: `"10"`, `"10.start"`, `"10.end"`, `"10.center"`. So a point
  coincident with a corner or on an edge, tangent to a rim, and dimensions to a part all work.
- **It's construction unless `"construction": false`.** Then it's one of the sketch's curves and can bound regions, as Onshape's
  Use does, and a face swept from it is named for it: `f5:side.10`.
- **`geometry`** is where it last was, which a saved file keeps, so it reads sensibly on its own. One without it has no place
  until its document works it out, and solving it alone says to give the sketch tools a `"context"`.
- **If what it comes from has gone,** or an edge selection picks no edge or several, the sketch fails and says which entity, as a
  gone region does. Deleting the features it projects from is refused while the sketch is there.

**Reference dimensions.** Any dimension can carry `"reference": true`. It then reports what the geometry measures and takes no
part in solving. Solving writes the measured value into it, and a file may leave `value` out.

**References.** `"7"` is a whole entity. `"7.start"`, `"7.end"`, `"7.center"`, `"7.major"` and `"7.p2"` are its parts. `"origin"`,
`"x_axis"` and `"y_axis"` are fixed reference geometry, and so is anything [projected](#projected-geometry) from a part.

| Constraint `type` | Fields | Meaning |
|---|---|---|
| `coincident` | `a`, `b` | Two points together, or a point on a line, circle or arc (`b` is the whole edge). A line counts as infinite and an arc as its full circle. |
| `horizontal`, `vertical` | `a`, optional `b` | A line, or two points level or upright with each other |
| `parallel`, `perpendicular` | `a`, `b` | Two lines |
| `tangent` | `a`, `b` | A line and a curve, or two curves. If they share an end through a coincident constraint, they are tangent there. |
| `equal` | `a`, `b` | Two lines of one length, or two curves of one radius |
| `midpoint` | `point`, `line` | A point at the middle of a line |
| `concentric` | `a`, `b` | Two curves with one center |
| `symmetric` | `a`, `b`, `about` | Two points mirrored about a whole line, or through a point |
| `normal` | `a`, `b` | A line through a curve's center, or two curves crossing at right angles |
| `fix` | `what`, `at` | Holds something at the values in `at`: x, y for a point; both ends for a line; x, y, radius for a circle; those then both ends for an arc |
| `distance` | `a`, optional `b`, `value` | A line's length (`b` absent), or between any two of a point, a line and a circle or arc. To a circle or arc, it's the gap to the curve, with an arc counting as its full circle. Between two lines, it's from the middle of `a` to `b`, so add `parallel` for the usual meaning. |
| `horizontal_distance`, `vertical_distance` | `a`, `b`, `value` | `b.x - a.x` or `b.y - a.y`, which may be negative |
| `major_radius`, `minor_radius` | `of`, `value` | An ellipse's or elliptical arc's radii |
| `radius`, `diameter` | `of`, `value` | A circle or arc. Positive. |
| `angle` | `a`, optional `b`, `value` | Degrees, counter-clockwise from line `a`'s direction to line `b`'s. With `a` alone: from +x to a line, or an arc's span. |

A `value` is a number or an expression over the document's parameters.
