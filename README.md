# CoreShape

A small CAD kernel in Rust with its own exact B-rep geometry, and no dependencies, with a sketch
constraint solver and an MCP server, so an AI agent can model with it and see the result.
It runs natively and in the browser through WebAssembly.

**What it does**

- **Geometry:** exact planes, cylinders, cones, spheres and tori, with lines, circles,
  ellipses and traced intersection curves.
- **Modelling:** 2D profiles that extrude and revolve, and union, subtract and intersect
  on all five surface types, including coplanar and touching contacts.
- **Blends:** constant-radius fillets and chamfers, exact on straight and circular edges, and fitted to a stated tolerance on any other.
- **Free-form:** lofts through sections and rational B-spline surfaces, in Booleans and in STEP files both ways.
- **More modelling:** many tools in one Boolean, patterns, mirror, sweeps along lines and arcs,
  offset, shell and draft, all on the same exact surfaces. Springs and threads along a helix, to a stated tolerance.
  Delete face, which closes the gap by letting the neighbours grow, and thicken.
- **Documents:** parameters and features as plain data, saved as readable JSON. Features can
  be edited, suppressed, deleted and reordered. References go by name, and a rebuild reuses
  every step whose inputs didn't change. A document can hold several bodies, so an imported
  assembly's parts stay parts, and a change to one leaves the others alone.
- **Sketches:** a 2D constraint solver in its own crate. It reports remaining freedom, names
  redundant and conflicting constraints, supports dragging, and is fast enough for every mouse
  move. Checked against FreeCAD's PlaneGCS: see [docs/SKETCH_SOLVER.md](docs/SKETCH_SOLVER.md).
- **Output:** exact volume and area, a watertight mesh, STL and STEP, and offscreen PNG images.
- **Extrudes as in Onshape:** any regions from any constrained sketches, ending at a length, through all, up to the next surface, or up to a face, a part or a vertex, on planes and on curved surfaces extended as far as they need to go.
- **Assemblies, as in Onshape:** tabs of Part Studios and assemblies, instances that follow their parts, mate connectors found from faces and edges, eight kinds of mate with limits, gears, racks and screws, sub-assemblies, dragging, interference and a parts list, and STEP assemblies in and out.
- **Input:** STEP files with analytic surfaces, swept curves, free-form surfaces, faceted solids and assemblies, healing what real files get wrong, and tested against OpenCASCADE on 108 files from other systems: 100 read. Anything else is refused by name.
- **For agents:** an MCP server for building, editing and inspecting parts by stable names. See
  [docs/MCP.md](docs/MCP.md).

Limits and open defects are in [docs/KNOWN_LIMITS.md](docs/KNOWN_LIMITS.md). The original
plan is in [docs/PLAN.md](docs/PLAN.md).

## Try it

```sh
# Install Rust if needed
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --no-modify-path
rustup target add wasm32-unknown-unknown

cargo test --workspace                       # all tests
cargo run --release -p coreshape --example fuzz -- 5000   # randomized Booleans, with a report
cargo run --release -p coreshape --example replay -- gen 1881   # replay one fuzz case
cargo run --release -p coreshape-wasm --example scan -- 150      # random slider positions on the demos

python3 scripts/build_web.py                 # build the viewer into dist/
python3 scripts/selftest_web.py              # drive it in headless Chrome
scripts/check_step.sh /tmp/step-check        # read every model's STEP file in OpenCASCADE
scripts/check_step_import.sh /tmp/step-in    # read files OpenCASCADE wrote, and send the results back

cargo run --release -p coreshape-sketch --example fuzz -- 5000     # randomized sketches, with a report
cargo run --release -p coreshape-sketch --example bench            # solver speed
python3 scripts/mcp_session.py /tmp/mcp-session                    # model a part through the MCP server
```

`dist/index.html` is one self-contained file, about 800 KB including the kernel. Open it
from any static host.

## Using the kernel

```rust
use coreshape::*;
use coreshape_document::*;

let mut doc = Document::new();
doc.set_parameter("w", 40.0)?;
doc.set_parameter("depth", "w / 2")?; // an expression over other parameters
let sketch = doc.add(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::centered("w", "depth"))]))?;
let block = doc.add(Op::extrude(sketch, 0.0, 10.0, Mode::Add))?;
// Named faces, not indices: this still means the same edge after any rebuild.
doc.add(Op::fillet(EdgeSelect::Between(block.end_cap(), block.side(0)), 2.0))?;

let outcome = doc.rebuild(None);
let body = outcome.body.unwrap();
println!("{} mm3", mass_properties(&body).volume);
std::fs::write("block.step", export::step::write_step(&body, "block"))?;

// A document is plain data.
std::fs::write("block.coreshape", doc.save_text())?;
let again = Document::load_text(&std::fs::read_to_string("block.coreshape")?)?;
```

The file format is described in [docs/FILE_FORMAT.md](docs/FILE_FORMAT.md).

## How it stays robust

- **Bodies are plain values.** Every operation takes bodies and returns a new one.
- **No seams and no poles.** Closed surfaces are split into faces of at most 180 degrees, and
  each face gets a 2D chart with no singularities on it. All 2D topology runs in charts.
- **Exact distance functions.** Each surface's signed distance is 1-Lipschitz, so a distance
  of `d` proves no surface lies within `d`. Root finding uses that to discard curve pieces
  with certainty instead of relying on sampling density.
- **Shared contacts.** Every contact between two bodies becomes one vertex or edge used by
  both sides, so results close by construction rather than by stitching within a tolerance.
- **The same input gives the same bits.** Hash tables use a fixed seed, so a failure reproduces
  exactly, in any process.
- **Results are checked before they are returned.** Each Boolean's final result passes the
  validator, a watertight mesh, agreement across samples on every face region, and point
  membership against both inputs. A failure is an error, not a broken solid.

## Layout

| Path | Contents |
|---|---|
| `crates/kernel/src/surface.rs`, `curve.rs` | Exact geometry, distance functions, charts |
| `crates/kernel/src/topo.rs`, `validate.rs` | The B-rep and its validity checks |
| `crates/kernel/src/profile.rs`, `build.rs` | Sketch profiles, extrude, revolve, primitives |
| `crates/kernel/src/boolean/` | Root finding, intersections, face tracing, classification, cleanup |
| `crates/kernel/src/blend.rs` | Fillets and chamfers |
| `crates/kernel/src/multi.rs`, `tweak.rs` | Many tools at once, sweeps; offset, shell and draft |
| `crates/kernel/src/helix.rs`, `delete.rs`, `thicken.rs` | Helix, delete face and thicken |
| `crates/kernel/src/mass.rs` | Volume and area from the B-rep through Green's theorem |
| `crates/kernel/src/tess/` | Exact-predicate triangulator and meshing |
| `crates/kernel/src/export/` | STL and STEP |
| `crates/kernel/src/import/` | STEP, analytic surfaces and swept curves |
| `crates/kernel/src/spline.rs` | Rational B-splines, with a certified closest point |
| `crates/kernel/src/fuzz.rs` | Replayable randomized Boolean testing |
| `crates/sketch/` | The constraint solver, with expressions and JSON. Knows nothing about the kernel |
| `crates/document/` | Parameters, features, rebuilding and the file format, joining the kernel and solver |
| `crates/assembly/` | Assemblies as in Onshape: instances, mate connectors, mates and their solver, and a project of tabs |
| `crates/render/` | Offscreen rendering and a PNG encoder |
| `crates/mcp/` | The MCP server |
| `crates/wasm/` | C ABI and the demo models |
| `web/viewer.html` | The WebGL2 viewer |

## Dependencies

None at runtime or for testing. Each crate depends only on others in this repository: the kernel and the
sketch solver on nothing at all. The triangulator, random numbers, JSON, PNG encoding and the MCP protocol are all
written here, and the browser layer is a hand-written C ABI with no binding generator. Development uses only the Rust toolchain. The browser self test needs Chrome, and
the STEP check uses an existing FreeCAD build.
