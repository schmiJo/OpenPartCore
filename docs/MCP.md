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
| `document_new`, `document_open`, `document_save`, `document_show` | The whole document |
| `parameter_set`, `parameter_remove` | Named values and expressions |
| `feature_add`, `feature_edit`, `feature_rename`, `feature_suppress`, `feature_delete`, `feature_move`, `features_list` | The history. Each change rebuilds and reports what happened. |
| `rebuild` | Each feature's state, and the solid's volume, area, size and validity. `upto` shows an earlier state. |
| `faces_list` | Stable names, surface type, area, center, normal and radius |
| `edges_list` | Stable names, curve type, length and ends, and a selection ready to use in a fillet or chamfer |
| `export` | STEP, STL or the native format |
| `render` | A PNG: iso, front, back, left, right, top, bottom, four views at once, or any angle. Optional face labels, one tint per face, and highlighted faces and edges |
| `sketch_solve`, `sketch_drag`, `sketch_regions` | The sketch solver, without changing the document |

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
edited through names, images decoded, and files kept inside the root. A session's STEP export opens in OpenCASCADE as one valid
solid with the same volume to six decimals.

**It has not yet been connected to a real MCP client.** It follows the specification, with protocol versions 2024-11-05 to
2025-11-25 accepted, but the only client so far is the test suite.

Rendering is software only, so it needs no GPU or display, and the same body gives the same bytes. The tests measure the
pictures: a sphere's coverage matches its circle within 3%, views look from where they say, near things hide far ones, and
labels land on their faces. The PNGs are also read back by zlib.
