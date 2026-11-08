//! Prints a checksum of every demo's exact result: its STEP text, mesh and mass properties.
//! Two runs of this program must print the same lines, which checks that nothing in the
//! kernel depends on a per-process seed. See `scripts/check_determinism.sh`.
use coreshape::*;
use coreshape_wasm::models;

fn mix(h: u64, bytes: &[u8]) -> u64 {
    bytes.iter().fold(h, |h, &b| (h ^ b as u64).wrapping_mul(0x100000001b3))
}

fn main() {
    for m in models::catalog() {
        let variants = m.params.iter().find(|p| !p.choices.is_empty()).map_or(1, |p| p.max as usize + 1);
        for variant in 0..variants {
            let mut doc = m.document(variant);
            let Some(b) = doc.rebuild(None).body else { continue };
            let props = mass_properties(&b);
            let mesh = tessellate(&b, &TessParams::for_size(b.aabb().diagonal())).unwrap();
            let mut h = 0xcbf29ce484222325;
            h = mix(h, coreshape::export::step::write_step(&b, "m").as_bytes());
            h = mix(h, &props.volume.to_bits().to_le_bytes());
            h = mix(h, &props.area.to_bits().to_le_bytes());
            for t in &mesh.triangles {
                for v in t {
                    h = mix(h, &v.to_le_bytes());
                }
            }
            println!("{} {variant} {h:016x}", m.id);
        }
    }
}
