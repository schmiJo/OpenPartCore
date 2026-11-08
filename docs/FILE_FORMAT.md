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

## Features

Common fields: `id`, `name`, and `suppressed` (default false).

| `type` | Fields |
|---|---|
| `sketch` | `plane` (default XY), `regions` |
| `extrude` | `sketch`, `length`, `start` (default 0), `mode` (default `add`) |
| `revolve` | `sketch`, `degrees` (default 360), `mode`. Profile x is the distance from the sketch's Z axis, and profile y the height along it. |
| `solid` | `shape`, its dimensions, `at` (default XY), `mode` |
| `fillet` | `edges`, `radius`, `propagate` (default true: follow tangent neighbours) |
| `chamfer` | `edges`, `distance` |

`mode` is `add`, `remove` or `intersect`. A placement is `{"origin": [x, y, z], "z": [x, y, z], "x": [x, y, z]}`,
where each part has a default: the origin, world Z, and world X (world Y if z runs along X).

Solids: `box` (`dx dy dz`), `cylinder` (`radius height`), `cone` (`r0 r1 height`), `sphere` (`radius`),
`torus` (`major minor`).

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
| `path` | `start`, `moves`: `{"line_to": [x, y]}` or `{"arc_to": [x, y], "center": [x, y], "ccw": true}`. Closes itself with a line if needed. |

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
