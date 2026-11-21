# The MCP server

`crates/mcp` is an MCP server, so an AI agent can model with the kernel and see the result. It speaks JSON-RPC 2.0, one
message per line, on stdin and stdout.

```sh
cargo build --release -p coreshape-mcp
claude mcp add coreshape -- "$PWD/target/release/coreshape-mcp" --root /path/to/a/work/folder
```

For another MCP client, the command is `coreshape-mcp --root <folder>` with no other setup. Files are read and written only
under the root, which is the current folder unless given. A path that leaves it, directly or through a link, is refused.

## Tools

| Tool | What it does |
|---|---|
| `format_guide` | The fields of every feature, constraint and edge selection |
| `document_new`, `document_open`, `document_save`, `document_show` | The whole document. `document_show` with `omit_texts_over` gives a large text, such as an imported STEP file's, by its size and SHA-256, and `document_open` takes the text back in `blobs`: see [FILE_FORMAT.md](FILE_FORMAT.md#features) |
| `parameter_set`, `parameter_remove` | Named values and expressions |
| `feature_add`, `feature_edit`, `feature_rename`, `feature_suppress`, `feature_delete`, `feature_move`, `features_list` | The history. Each change rebuilds and reports what happened. |
| `rebuild` | Each feature's state, and the solid's volume, area, size and validity. `upto` shows an earlier state. A feature that built but has something to say, such as the parts of a STEP file it left out and why, has `notes`, and they're in the summary's `notes` too |
| `bodies_list` | With several bodies: each one's id, its `name` where it has one, such as a STEP part's, what made it, volume, area, faces and size |
| `faces_list` | Stable names, surface type, area, center, normal, radius, and a round face's axis. With several bodies, whose each is, and `body` keeps to one |
| `edges_list` | Stable names, curve type, length and ends, a selection ready to use in a fillet or chamfer, and each end named as `up_to_vertex` takes it. `body` keeps to one |
| `export` | STEP, STL or the native format. Every body, or the one `body` names |
| `render` | A PNG: iso, front, back, left, right, top, bottom, four views at once, or any angle. Optional face labels, one tint per face, and highlighted faces and edges |
| `sketch_solve`, `sketch_drag`, `sketch_regions`, `sketch_arrangement` | The sketch solver, without changing the document. `sketch_arrangement` lists every closed region with its id, and the curves that make none. A sketch with [projected](FILE_FORMAT.md#projected-geometry) entities takes `"context": {"plane": placement, "position": n}`, `n` being where the sketch is or will go in the history, and they're put where the parts made by the features before `n` put them. `plane` defaults to XY and `position` to the end, and `plane` can be `{"on_plane": 4}`, a plane as a sketch's can |
| `flat_pattern` | A sheet metal part laid out flat: the `frame` it lies in, with z into the sheet, its `outline` and holes as lines and arcs in that frame's x and y, each of its `bends` with its middle `line`, `degrees`, inner `radius`, `direction` (`up` or `down`) and the `width` of its strip, and its `size`, `area`, `thickness` and `k_factor`. `body` picks a part, and without it there has to be only one. `upto` gives it as the history stood after that position. See [Sheet metal](FILE_FORMAT.md#sheet-metal) |
| `planes_list` | Top, Front and Right, then each plane feature and each sketch, in history order: its id, name, type, `plane_type`, position and state, and where it is, `origin`, `normal`, `x` and `y`, or why it didn't build. `upto` gives them as the history stood after that position, for rolling back past them. See [Planes](FILE_FORMAT.md#planes) |
| `vertices_list` | Each corner: where it is, the edges that meet there, and the `vertex` that picks it, `{"faces": [...], "near": [x, y, z]}`, ready for a projected entity or `up_to_vertex`. `body` keeps to one |

| `tab_list`, `tab_new`, `tab_select`, `tab_rename`, `tab_delete`, `project_show` | Tabs of Part Studios and assemblies |
| `instance_add`, `instance_edit`, `instance_delete` | Parts and whole assemblies in the selected assembly |
| `connector_candidates`, `connector_add`, `connector_edit`, `connector_delete` | Mate connectors. Candidates are the places a face or edge offers, each ready to use |
| `mate_add`, `mate_edit`, `mate_delete`, `mate_set_value` | Mates, whose ends can be connectors or made on the spot, and taking one to a value |
| `group_add`, `group_delete`, `relation_add`, `relation_delete` | Instances that move as one, and gears, racks, screws and linear relations |
| `assembly_solve`, `assembly_drag` | Where every part is, what can still move, each mate's values, and which mates can't be met and why |
| `interference_check`, `bom`, `assembly_import_step` | What overlaps and by how much, how many of each part, and a STEP assembly as new tabs |

Each face in `faces_list` has a `key` as well as a `name`, since a name can cover several faces, and `highlight_faces` and edge selections take either. An edge's `select` uses keys when names alone
would fit several edges, and where even those fit several places, what's at its ends or its place among them: see
[FILE_FORMAT.md](FILE_FORMAT.md#face-keys) and [edge selections](FILE_FORMAT.md#edge-selections). A whole round face's two
halves share one key, so picking either picks the face.

## Progress

A long call, such as `feature_add` with a large STEP file, says how far it's got if its request asks, with a
`progressToken` in `params._meta` as MCP has it. Each `notifications/progress` has `progress` counting up and a `message`
such as `f3: solids 120 of 711`. The stages are `parsing`, counted in bytes, `solids`, `rebuild`, counted in features, and
`measuring`, the bodies the summary measures. Each is told at most once per whole percent, so there are a few hundred
messages however large the file. A call that doesn't ask hears nothing.

An app that uses `Session` directly, as SchmiShape's engine does, sets the kernel's hook on the thread that calls the
tools, and gets the same as JSON:

```rust
coreshape::progress::set_hook(|p| {
    let json = p.json(); // {"feature":3,"stage":"solids","done":120,"total":711}
    unsafe { sx_progress(json.as_ptr(), json.len()) }; // an import the browser provides
});
```

The kernel has no clock, as WebAssembly in a browser can't read one, so it doesn't throttle by time: whole percents keep it
to about a hundred calls a stage. `coreshape::progress::clear()` stops it.

## Several bodies

A body is a part, as in Onshape: one connected solid. A feature acts on all of them unless it lists `"bodies": ["f3.2"]`, and leaves alone any it doesn't reach. `"mode": "new_body"` makes parts that stay separate, and `import_step` with that mode makes one for each solid in the file. `transform`, `delete_bodies` and `boolean` are features too. `bodies_list` gives the names, and `render` tints each body differently unless told otherwise with `colors`, and takes `body` to show just one. `app_set` keeps data for an app with a feature or the document, without touching the model. See the format guide for the details.

## Extrude ends

`feature_add` takes an extrude's `regions` from `sketch_arrangement`, its `faces` of parts, its `end` and `opposite`, a revolve's `regions`, a loft's sections as regions, and a sweep's `path` along a sketch's curves or a part's edges: see [FILE_FORMAT.md](FILE_FORMAT.md#extrude-regions-and-ends). `edges_list` names each edge's ends as `start_vertex` and `end_vertex`, ready for `up_to_vertex`, and `faces_list` gives a cylinder, cone or torus its `axis`, with a `point` and a `direction`, and a cone its `half_angle` in degrees.

## Meshes for an app

An app that uses `Session` directly gets what to draw as flat arrays, in the same layout as a graphics card takes them: positions, normals, a face for each vertex, triangle indices, edge segments and an edge for each. They aren't tools, as JSON would be the wrong way to pass them.

- `Session::part_mesh(part, tab, draft)` gives one part's `FlatMesh`, in its own coordinates. Face and edge numbers are the ones `faces_list` and `edges_list` give with that `body`.
- `Session::instance_meshes(draft)` gives the selected assembly, solved, with an `InstanceMesh` for each instance: its path, name, part, `at`, `matrix` by columns for WebGL, and `mesh`.
- In a Part Studio, a body that's a rigid copy of another says so in its `NamedBody::copy_of`: that body, and `at`, where it's moved to. Every use of a part in an imported STEP file is one, and so is each of several parts that are the same in the file. Meshing the body it copies once and drawing it at each `at` saves meshing it for every copy: a 711-part assembly has 2,218 bodies and 270 different shapes. A body's `name` is its part's name from the file.

Every instance of a part shares one mesh, and a mesh is only worked out again when its part changes, so a drag costs a solve and no meshing. `FlatMesh::placed(&at)` gives a copy in the assembly's coordinates, for an app that would rather have it all in one space.

## Sheets

A sheet is an open surface that's a body of its own: `bodies_list` marks it `"sheet": true`, with how many `boundary_edges` it has, and its volume is 0. It's made by features: `fill`, `copy_faces`, or `extrude`, `revolve`, `sweep`, `loft` and `helix` with `"mode": "surface"`. `thicken`, `sew` and `trim` take it on from there. In `edges_list` an edge with a face on one side only says `"boundary": true`, and its `select` goes by where it is. [FILE_FORMAT.md](FILE_FORMAT.md#sheets) has the fields.

```
feature_add { "feature": { "type": "sketch", "regions": [{ "outer": { "type": "open_path", "start": [0, 0], "moves": [{ "line_to": [10, 0] }, { "line_to": [10, 7] }] } }] } }
feature_add { "feature": { "type": "extrude", "sketch": 1, "length": 5, "mode": "surface" } }
feature_add { "feature": { "type": "trim", "point": [0, 0, 2], "normal": [0, 0, 1] } }
```

## Assemblies

The tools for features always work on a Part Studio: the last one selected, even with an assembly selected. `render` and `export` follow the selected tab, so with an assembly they draw it, with each instance its own colour, and write it as a STEP assembly, or an STL of it all. `body` still means a part of the Part Studio. `document_save`, `document_open` and `export` with `coreshape` take the whole project, which is a plain document if it's one Part Studio.

A new mate brings its first end to the second, and if that's the only mate on it and it leaves a turn free, it comes to rest with its connectors lined up, at an angle of zero. Every tool that changes how an assembly goes together solves it and answers with its state: `freedom`, each instance's pose in `at`, each mate's `values`, and a `failing` on any mate that can't be met. A mate that fails isn't an error, as in Onshape, and neither is a part that's gone from its Part Studio, which comes back under `problems`. `failing_relations` names any relation that can't be met, and `failing_in_sub_assemblies` any mate inside a sub-assembly. `connector_add` and `connector_delete` answer only for themselves, as a connector no mate uses changes nothing.

```
tab_new { "type": "assembly" }
instance_add { "part": "f1", "fixed": true, "name": "Plate" }     gives id 1
instance_add { "part": "f2", "name": "Pin" }                      gives id 2
mate_add { "type": "revolute", "flip": true,                      gives id 5, as its two connectors are 3 and 4
           "a": { "instance": 2, "on": { "face": "f2:start" } },
           "b": { "instance": 1, "on": { "face": "f1:end" } } }
mate_set_value { "id": 5, "dof": "angle", "value": 400 }
```

An app can get the same with `Session::solved`, which gives each part's `Body` and where it goes, `Session::active_tab`, and `Session::with_project` for anything else. A mate inside a sub-assembly moves when it's dragged from above, and `mate_set_value` and `mate_edit` are for the assembly that has the mate. See [FILE_FORMAT.md](FILE_FORMAT.md#projects-and-assemblies) for what each thing means, and [KNOWN_LIMITS.md](KNOWN_LIMITS.md#assemblies) for how far it goes.

## How a session goes

1. `parameter_set`, then `feature_add` a sketch and an extrude.
2. `render` with `labels: true` to see which face is which, and `edges_list` for names.
3. `feature_add` a fillet using an edge's `select`. It keeps working after dimensions change.
4. `parameter_set` to change a dimension, and read what the rebuild says.
5. `export`.

`scripts/mcp_session.py <folder>` runs such a session against the real binary and saves what it sees.

A failed feature comes back with its reason and leaves the last good solid in place. A cut in the wrong place isn't an error, so
the answer carries a note when a feature left no faces on the solid. An unexpected kernel failure comes back as a failed
result and the server carries on.

## What's been checked

`crates/mcp/tests/protocol.rs` drives the real binary over its pipes: the handshake, schemas, protocol errors, a part built and
edited through names, images decoded, and files kept inside the root, and assemblies built, moved, saved and opened again through
every tool. A session's STEP export opens in OpenCASCADE as one valid solid with the same volume to six decimals.

**It has not yet been connected to a real MCP client.** It follows the specification, with protocol versions 2024-11-05 to
2025-11-25 accepted, but the only client so far is the test suite.

Rendering is software only, so it needs no GPU or display, and the same body gives the same bytes. The tests measure the
pictures: a sphere's coverage matches its circle within 3%, views look from where they say, near things hide far ones, and
labels land on their faces. The PNGs are also read back by zlib.
