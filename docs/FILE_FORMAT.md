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
| `sketch` | `plane` (default XY, and with `"on_face": "f2:end"` it follows that flat face), `regions` |
| `extrude` | `sketch`, `length`, `start` (default 0), `mode` (default `add`), and optionally `regions`, `end` and `opposite`: see [below](#extrude-regions-and-ends) |
| `loft` | `sections`: a list of sketch ids, in order, each holding one outline, and `mode`. Straight between two, and a smooth cubic through three or more |
| `revolve` | `sketch`, `degrees` (default 360), `mode`, and `axis`: `{"point": [x, y], "direction": [dx, dy]}`, a line in the sketch's plane, in its coordinates, to turn around as drawn, right-handed around its direction. Without an `axis`, profile x is the distance from the sketch's Z axis, and profile y the height along it. |
| `sweep` | `sketch`, `path`, `mode`: see below |
| `helix` | `sketch`, `pitch` (how far along the axis one turn goes), `turns`, `point` (default origin) and `axis` (default +z) for the axis, `left_handed` (default false), `mode`. The sketch can hold the axis, for a spring or a thread, or lie across it, for a twisted bar |
| `solid` | `shape`, its dimensions, `at` (default XY), `mode` |
| `pattern` | `of` (feature ids, which can be other patterns and mirrors; leave out for the whole body), and `direction`, `count`, `spacing`, or `center` (default origin), `axis` (default +z), `count`, `degrees` (default 360). `count` includes the original. |
| `mirror` | `of` (as above), `point` (default origin), `normal` |
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
| `trim` | What cuts: `tool`, a solid's name like `"f3"`, or `point` and `normal`, a plane, or `sketch`, whose regions cut straight through along its normal. And `keep`: `outside` (the default), `inside` or `both`. Acts on the `bodies` it names, by default every sheet. See [Sheets](#sheets) |
| `offset` | `distance`: every face moves out, or in if negative. Sharp edges stay sharp. |
| `shell` | `thickness`, `open` (face names, default none). An inner wall is named for this feature, with the slot of the face it follows: `f9:side.3` lies behind `f2:side.3`. |
| `draft` | `faces` (names), `degrees`, `pull` (default +z), `neutral` (a point on the plane where faces stay put; default the origin). A positive angle narrows the part along the pull, and widens a hole. |

`mode` is `add`, `remove`, `intersect`, `new_body` or `surface`. A placement is `{"origin": [x, y, z], "z": [x, y, z], "x": [x, y, z]}`,
where each part has a default: the origin, world Z, and world X (world Y if z runs along X).

Solids: `box` (`dx dy dz`), `cylinder` (`radius height`), `cone` (`r0 r1 height`), `sphere` (`radius`),
`torus` (`major minor`).

**Sweep paths** start at the sketch plane heading along its normal, and each step carries on from the direction
the last ended in, so a path is always smooth. A step is `{"line": length}`, or `{"arc_radius": r, "degrees": a,
"toward_degrees": d}`, a bend around a center `r` away in the direction `d` degrees counter-clockwise from the
sketch's x axis. Bends must stay wider than the profile, or the profile would cross the center.

**Copies keep names.** A face in a pattern's or mirror's copy has the name of the face it came from, so `f2:end` covers
the original and every copy, and a fillet between two named faces applies to them all. To pick one, use a [face key](#face-keys).
A pattern of features applies each one's solids as it did, adding or removing.

## Face keys

A name can cover several faces. A key adds what tells them apart, and each part only narrows, so a plain name is still a key that means all of them:

| Key | Picks |
|---|---|
| `f2:end` | Every face with that name |
| `f2:end@f5.1` | Copy 1 made by pattern or mirror `f5`. Copy 0 is the original |
| `f2:end@f5.1@f7.0` | A copy of a copy: copy 1 of `f5`, in the original of `f7` |
| `f3:side.1[f2:end\|f2:side.0]` | The fillet or chamfer face that replaced the edge between those two faces |
| `f4:side.1[f2:side.1]` | A shell's inner wall following `f2:side.1` |
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

**`end`** says where it stops, in place of `length`. If there's a `length` as well, it's kept, unused, for going back to it.

| `end` | Stops |
|---|---|
| `"blind"` | At `length`. This is the default |
| `"through_all"` | Past every part it acts on. A new solid stops flush with the furthest |
| `"up_to_next"` | At the nearest surface in the way among the parts it acts on. A cut goes on through the material to its far side |
| `{"type": "up_to_face", "face": "f1:end"}` | At that face's surface, extended as far as it needs to be. A plane, cylinder, cone, sphere or torus |
| `{"type": "up_to_part", "part": "f3"}` | At that part's nearest surface, going on past anything else |
| `{"type": "up_to_vertex", "vertex": {"faces": [...], "near": [x, y, z]}}` | At a plane through the vertex those faces share, square to the direction. `near` says which if they share several. `edges_list` gives these for each edge's ends as `start_vertex` and `end_vertex` |
| `{"type": "up_to_vertex", "vertex": {"point": [x, y, z]}}` | The same, through a place |

**`"opposite": true`** goes the other way. An end behind the sketch is refused, and the message says so. The far end is named as a blind extrude's is, however it's shaped: `f5:end` going forward, and `f5:start` in the opposite direction, as the kernel names caps by where they are.

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
are left out without failing. To build from particular regions, list them in an optional top-level
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

**Reference dimensions.** Any dimension can carry `"reference": true`. It then reports what the geometry measures and takes no
part in solving. Solving writes the measured value into it, and a file may leave `value` out.

**References.** `"7"` is a whole entity. `"7.start"`, `"7.end"`, `"7.center"`, `"7.major"` and `"7.p2"` are its parts. `"origin"`,
`"x_axis"` and `"y_axis"` are fixed reference geometry.

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
