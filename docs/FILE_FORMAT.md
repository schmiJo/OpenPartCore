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
| `sheet_metal` | `sketch`, `thickness`, `bend_radius` (default the thickness), `k_factor` (default 0.45), `depth` for a chain of lines, `flip`. A new sheet metal part. See [Sheet metal](#sheet-metal) |
| `flange` | `edges` (a [selection](#edge-selections)), `length`, `degrees` (default 90), `alignment` (`outer`, the default, `inner` or `bend`), `bend_radius` (default the part's), `flip`. See [Sheet metal](#sheet-metal) |
| `extrude` | `sketch`, `length`, `start` (default 0), `mode` (default `add`), and optionally `regions`, `faces`, `end` and `opposite`: see [below](#extrude-regions-and-ends) |
| `loft` | `sections`, in order, each a sketch id, for a sketch holding one outline, or a region `{"sketch": 2, "region": "r..."}`, so that one sketch with several outlines can feed it, and `mode`. Straight between two, and a smooth cubic through three or more. A region that has gone fails the feature, saying what's there |
| `revolve` | `sketch`, `degrees` (default 360), `mode`, and `axis`: `{"point": [x, y], "direction": [dx, dy]}`, a line in the sketch's plane, in its coordinates, to turn around as drawn, right-handed around its direction. Without an `axis`, profile x is the distance from the sketch's Z axis, and profile y the height along it. Optionally `regions`, as an extrude has them: see [below](#extrude-regions-and-ends) |
| `sweep` | `sketch`, `path`, `mode`. `path` is a list of steps, `{"sketch": 5}` or `{"edges": selection}`: see below |
| `helix` | `sketch`, `pitch` (how far along the axis one turn goes), `turns`, `point` (default origin) and `axis` (default +z) for the axis, `left_handed` (default false), `mode`. The sketch can hold the axis, for a spring or a thread, or lie across it, for a twisted bar |
| `solid` | `shape`, its dimensions, `at` (default XY), `mode` |
| `pattern` | `of` (feature ids, which can be other patterns and mirrors; leave out for the whole body), and `direction`, `count`, `spacing`, or `center` (default origin), `axis` (default +z), `count`, `degrees` (default 360). `count` includes the original. `direction` is `[x, y, z]` or any [line](#planes), whose direction it takes, and `axis` is `[x, y, z]` through `center`, or a line, which gives the center too, so there's no `center` with one: `{"axis": {"axis_of": "f4:side.0"}, "count": 6}` goes round a boss. A circular pattern turns right-handed about the axis's direction |
| `mirror` | `of` (as above), and `plane`, any [plane](#planes), or `point` (default origin) and `normal` |
| `fillet` | `edges`, `radius`, `propagate` (default true: follow tangent neighbours) |
| `chamfer` | `edges`, `distance`, `propagate` (default true: follow tangent neighbours) |
| `fillet_radii` | `sets`: a list of `{"edges": ..., "radius": r}`, or `{"edges": ..., "along": [{"at": 0, "radius": 2}, {"at": 1, "radius": 5}], "start": [x, y, z]}` for a radius going in straight lines between places along the chain of edges, from 0 to 1, measured from the end nearest `start`, or without one from the end with the lowest x, then y, then z. Different radii can meet at a corner: see [KNOWN_LIMITS.md](KNOWN_LIMITS.md#fillets-and-chamfers) |
| `import_step` | `text`: a whole STEP file, kept in the document, and `mode`. Its faces are named by their place in the file: `f3:side.0` is the first. A wide face cut into pieces on the way in keeps one name for them all. An assembly's parts are placed where it puts them. With `mode` `new_body`, which is what an assembly wants, each solid is a part of its own. With `add`, those that touch are merged by Boolean, which can fail where they meet, and says to use `new_body`. A file it can't represent fails the feature, and says what's in the way. |
| `import_step_parts` | `text`: a whole STEP file, kept in the document. Each distinct solid in it comes in once as a new body, in its own coordinates, for an assembly to use, however many times the file uses it. Its faces are named as for `import_step` |
| `transform` | `motion`, the `bodies` to move, and `copy` (default false), which leaves them and adds moved copies as new parts. `motion` is `{"type": "translate", "by": [x, y, z]}`, `{"type": "rotate", "point": [x, y, z], "axis": [x, y, z], "degrees": d}` or `{"type": "frames", "from": placement, "to": placement}` |
| `delete_bodies` | The `bodies` to delete, which it has to name |
| `boolean` | `operation`: `union`, `subtract` or `intersect`, `tools`: a list of bodies, and `keep_tools` (default false). `subtract` cuts the tools from the `bodies`, its targets, which it has to name. The others take the `bodies` and `tools` together, and what's left has the first one's name. Bodies that don't meet are refused |
| `delete_face` | `faces` (names or keys): those faces go, and their neighbours grow to close the gap. It takes holes, pockets, bosses, fillets, chamfers, rabbets, slots and corner notches, and refuses by name what it can't close |
| `thicken` | `faces` (names or keys), `thickness` (outward, or inward if negative), `mode` (default `add`). A solid between the faces and a copy moved along their normals. Near faces are this feature's `start`, far ones its `end`, and walls its `side.N`. Each keeps the face it came from in its key |
| `fill` | `sketch`: a flat [sheet](#sheets) filling each of its regions, facing along the sketch's normal, named as this feature's end cap: `f3:end` |
| `copy_faces` | `faces` (names or keys): a copy of them as a sheet. What they're on stays as it is |
| `sew` | `tolerance` (default 1e-6), and the `bodies` to join, by default every sheet. See [Sheets](#sheets) |
| `trim` | What cuts: `tool`, a solid's name like `"f3"`, or `plane`, any [plane](#planes), or `point` and `normal`, a plane as numbers, or `sketch`, whose regions cut straight through along its normal. And `keep`: `outside` (the default), `inside` or `both`. Acts on the `bodies` it names, by default every sheet. See [Sheets](#sheets) |
| `offset` | `distance`: every face moves out, or in if negative. Sharp edges stay sharp. |
| `shell` | `thickness`, `open` (face names, default none). An inner wall is named for this feature, with the slot of the face it follows: `f9:side.3` lies behind `f2:side.3`. |
| `draft` | `faces` (names), `degrees`, and `neutral`: the plane where faces stay put, any [plane](#planes), whose normal is the pull, or a point on it, `[x, y, z]` (default the origin), with `pull` (default +z). A positive angle narrows the part along the pull, and widens a hole, so a negative one drafts the other way. |

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
| A line | A straight edge's [selection](#edge-selections), picking one edge, which runs from its start to its end, or the same as `{"edge": selection}`; a constrained sketch's line, `{"sketch": 4, "line": 9}`, from its start to its end; `{"axis_of": "f3:side.0"}`, the axis of a cylinder, cone, torus or turned face, pointing the way `faces_list` gives it, or `{"axis_of": selection}`, of a circle, arc or ellipse; or `{"point": [x, y, z], "direction": [x, y, z]}` |

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
| `f4:side.0#1` | Piece 1, counting faces that agree on everything else |

`face_keys(body)` gives each face a key that picks only that face, and `find_faces(body, key)` goes the other way. `{"on_face": key}` and `{"between": [key, key]}` take keys, so a
fillet can round one copy. `shell` and `draft` still take plain names, and refuse a key.

- **A key is worked out from the model and isn't stored,** except where a selection uses one. It adds nothing to files that don't.
- **A copy number stays with the copy,** so `@f5.1` stays the first copy when the pattern's count or spacing changes.
- **A piece number goes by position,** in the surface's own coordinates. It's the weakest part: it renumbers if pieces are added or removed, or pass each other. Prefer anything else that tells them apart.
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
| `{"between": ["f2:end", "f2:side.0"]}` | Edges between two named faces |
| `{"on_face": "f2:end"}` | Edges bounding a named face |
| `{"near": [x, y, z], "tolerance": d}` | Edges whose midpoint is within `d` (default 0.001) |
| `{"in_box": [[x, y, z], [x, y, z]]}` | Edges whose midpoint lies in a box |
| `{"parallel": [x, y, z]}` | Straight edges along a direction |
| `{"radius": r}` | Circular edges of a radius |
| `{"any": [...]}`, `{"both": [...]}` | Union and intersection of selections |

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

A name that picks several faces, such as a patterned boss's top, extrudes them all, and their walls share the entry's `wall.N`. A face two entries pick goes once, named for the first. A whole circle's edge gives two half walls under one key, as a whole circle always gives two half faces, and they're told apart by `#0` and `#1`.

With `"mode": "add"`, a wall that carries straight on from a face of the part, as the walls over a block's top do, becomes part of that face, which keeps its name. So `f6:wall.0` names only the walls that stand out on their own, and with `new_body` or `surface` it names them all.

A face has to be flat, and bounded by edges with an exact form: lines, arcs, ellipses and splines. An edge that's an intersection curve held as samples, such as where a plane cuts a free-form face, is refused, saying which face. So is a curved face, and a face that has gone.

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

**A connector's `on`** is what it's found from, so it follows the part:

| `on` | Where |
|---|---|
| `{"face": "f1:end"}` | The centroid of a flat face, with z straight out of it, or the middle of a round face's axis, with z along it. `"point"` can also be `axis_start`, `axis_end` or `apex` |
| `{"face": "f1:end", "near": [x, y, z]}` | The nearest of the face's corners, edge middles and arc centers |
| `{"edge": ["f1:end", "f1:side.0"], "point": "middle"}` | On the edge between two faces: `middle`, `start`, `end`, or `center` for a round one. Its z runs along a straight edge, and along the axis of a round one |
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

Two faces' z axes both point out of them, so two faces need `flip` to lie against each other. `offset` is from `a` to `b` along `a`'s axes, `rotation` is about `a`'s z, and `flip` and `quarter_turns` act on `b`. `limits` is a list of `{"dof": "x", "y", "z" or "angle", "min", "max"}`, in millimetres and degrees. `turned` is how far it has turned in radians, with its turns, kept so that the next turn can be told from this one.

**A relation** ties mates' motions together, as `b = ratio * a + offset`:

| `type` | Between | Amount |
|---|---|---|
| `gear` | Two that turn | `ratio` |
| `rack_and_pinion` | One that turns, and one that slides | `distance` for each whole turn |
| `screw` | One cylindrical mate, named as both | `lead` for each whole turn |
| `linear` | Two that slide | `ratio` |

`offset` is set when it's added, from where they are, so that adding it moves nothing.

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
