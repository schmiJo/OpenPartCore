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

Common fields: `id`, `name`, and `suppressed` (default false).

| `type` | Fields |
|---|---|
| `sketch` | `plane` (default XY), `regions` |
| `extrude` | `sketch`, `length`, `start` (default 0), `mode` (default `add`) |
| `revolve` | `sketch`, `degrees` (default 360), `mode`. Profile x is the distance from the sketch's Z axis, and profile y the height along it. |
| `sweep` | `sketch`, `path`, `mode`: see below |
| `solid` | `shape`, its dimensions, `at` (default XY), `mode` |
| `pattern` | `of` (feature ids; leave out for the whole body), and `direction`, `count`, `spacing`, or `center` (default origin), `axis` (default +z), `count`, `degrees` (default 360). `count` includes the original. |
| `mirror` | `of` (as above), `point` (default origin), `normal` |
| `fillet` | `edges`, `radius`, `propagate` (default true: follow tangent neighbours) |
| `chamfer` | `edges`, `distance`, `propagate` (default true: follow tangent neighbours) |
| `import_step` | `text`: a whole STEP file, kept in the document, and `mode`. Its faces are named by their place in the file: `f3:side.0` is the first. A wide face cut into pieces on the way in keeps one name for them all. A file it can't represent fails the feature, and says what's in the way. |
| `offset` | `distance`: every face moves out, or in if negative. Sharp edges stay sharp. |
| `shell` | `thickness`, `open` (face names, default none). An inner wall is named for this feature, with the slot of the face it follows: `f9:side.3` lies behind `f2:side.3`. |
| `draft` | `faces` (names), `degrees`, `pull` (default +z), `neutral` (a point on the plane where faces stay put; default the origin). A positive angle narrows the part along the pull, and widens a hole. |

`mode` is `add`, `remove` or `intersect`. A placement is `{"origin": [x, y, z], "z": [x, y, z], "x": [x, y, z]}`,
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
| `rounded_rect` | `cx cy` (default 0), `w h r` |
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

## Rules a reader enforces

- A feature may only refer to features before it, and `sketch` must name a sketch.
- Parameters may refer to each other but not in a cycle.
- A version other than this reader's is refused with a message saying so.

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
