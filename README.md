# CoreShape

A small CAD kernel in Rust with its own exact B-rep geometry, and no dependencies.
It runs natively and in the browser through WebAssembly.

**What it does**

- **Geometry:** exact planes, cylinders, cones, spheres and tori, with lines, circles,
  ellipses and traced intersection curves.
- **Modelling:** 2D profiles that extrude and revolve, and union, subtract and intersect
  on all five surface types, including coplanar and touching contacts.
- **Blends:** constant-radius fillets and chamfers on straight and circular edges.
- **Documents:** parameters and features as plain data, saved as readable JSON. Features can
  be edited, suppressed, deleted and reordered. References go by name, and a rebuild reuses
  every step whose inputs didn't change.
- **Output:** exact volume and area, a watertight mesh, STL and STEP.

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
```

`dist/index.html` is one self-contained file, about 800 KB including the kernel. Open it
from any static host.

## Using the kernel

```rust
use coreshape::document::*;
use coreshape::*;

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
| `crates/kernel/src/document.rs` | Parameters, features, rebuilding, the file format |
| `crates/kernel/src/expr.rs`, `json.rs` | Dimension expressions, and a JSON reader and writer |
| `crates/kernel/src/mass.rs` | Volume and area from the B-rep through Green's theorem |
| `crates/kernel/src/tess/` | Exact-predicate triangulator and meshing |
| `crates/kernel/src/export/` | STL and STEP |
| `crates/kernel/src/fuzz.rs` | Replayable randomized Boolean testing |
| `crates/wasm/` | C ABI and the demo models |
| `web/viewer.html` | The WebGL2 viewer |

## Dependencies

None at runtime or for testing. The kernel, its triangulator and its random number generator
are all in this repository, and the browser layer is a hand-written C ABI with no binding
generator. Development uses only the Rust toolchain. The browser self test needs Chrome, and
the STEP check uses an existing FreeCAD build.
