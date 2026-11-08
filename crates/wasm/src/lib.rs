//! A plain C ABI over the kernel for the browser viewer.
//!
//! There is no binding generator: JavaScript passes numbers, and reads results
//! out of linear memory through a table of `(pointer, length)` pairs. This keeps
//! the crate free of dependencies and the download small.

pub mod models;

use coreshape::*;
use coreshape_document::{Document, Outcome, StepState};
use std::cell::RefCell;
use std::collections::HashMap;
use std::fmt::Write;
use std::sync::Arc;

#[derive(Default)]
struct Output {
    positions: Vec<f32>,
    normals: Vec<f32>,
    kinds: Vec<u8>,
    indices: Vec<u32>,
    lines: Vec<f32>,
    info: String,
    export: Vec<u8>,
    body: Option<Arc<Body>>,
    /// One document per model and variant, each keeping its own cached results.
    documents: HashMap<(usize, usize), Document>,
    /// The document shown by the last build.
    current: Option<(usize, usize)>,
    table: [u32; 12],
}

thread_local! {
    static STATE: RefCell<Output> = RefCell::new(Output::default());
    static CATALOG: RefCell<String> = const { RefCell::new(String::new()) };
}

fn json_str(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            c if (c as u32) < 0x20 => {
                let _ = write!(out, "\\u{:04x}", c as u32);
            }
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

fn surface_kind(s: &Surface) -> u8 {
    match s {
        Surface::Plane(_) => 0,
        Surface::Cylinder { .. } => 1,
        Surface::Cone { .. } => 2,
        Surface::Sphere { .. } => 3,
        Surface::Torus { .. } => 4,
        Surface::Extruded { .. } | Surface::Revolved { .. } => 5,
    }
}

/// The catalogue as JSON. Returns a pointer; `cs_last_len` gives its length.
#[no_mangle]
pub extern "C" fn cs_catalog() -> *const u8 {
    CATALOG.with(|c| {
        let mut c = c.borrow_mut();
        if c.is_empty() {
            let mut s = String::from("[");
            for (i, m) in models::catalog().iter().enumerate() {
                if i > 0 {
                    s.push(',');
                }
                let _ = write!(
                    s,
                    "{{\"id\":{},\"name\":{},\"blurb\":{},\"params\":[",
                    json_str(m.id),
                    json_str(m.name),
                    json_str(m.blurb)
                );
                for (j, p) in m.params.iter().enumerate() {
                    if j > 0 {
                        s.push(',');
                    }
                    let _ = write!(
                        s,
                        "{{\"key\":{},\"label\":{},\"unit\":{},\"min\":{},\"max\":{},\"step\":{},\"value\":{},\"choices\":[{}]}}",
                        json_str(p.key),
                        json_str(p.label),
                        json_str(p.unit),
                        p.min,
                        p.max,
                        p.step,
                        p.default,
                        p.choices.iter().map(|c| json_str(c)).collect::<Vec<_>>().join(",")
                    );
                }
                s.push_str("]}");
            }
            s.push(']');
            *c = s;
        }
        c.as_ptr()
    })
}

#[no_mangle]
pub extern "C" fn cs_catalog_len() -> usize {
    CATALOG.with(|c| c.borrow().len())
}

/// Reserves space for JavaScript to write a parameter list into.
#[no_mangle]
pub extern "C" fn cs_alloc_f64(n: usize) -> *mut f64 {
    let mut v = Vec::<f64>::with_capacity(n);
    let p = v.as_mut_ptr();
    std::mem::forget(v);
    p
}

/// Releases space from `cs_alloc_f64`.
///
/// # Safety
/// `p` must come from `cs_alloc_f64(n)` and must not be used afterwards.
#[no_mangle]
pub unsafe extern "C" fn cs_free_f64(p: *mut f64, n: usize) {
    drop(Vec::from_raw_parts(p, 0, n));
}

fn features_json(outcome: &Outcome) -> String {
    let mut text = String::from("[");
    for (i, step) in outcome.steps.iter().enumerate() {
        if i > 0 {
            text.push(',');
        }
        let error = match &step.state {
            StepState::Failed(why) => why.as_str(),
            _ => "",
        };
        let _ = write!(
            text,
            "{{\"id\":{},\"name\":{},\"kind\":{},\"state\":{},\"error\":{}}}",
            step.id.0,
            json_str(&step.name),
            json_str(step.kind),
            json_str(step.state.word()),
            json_str(error)
        );
    }
    text.push(']');
    text
}

fn clear_mesh(out: &mut Output) {
    out.positions.clear();
    out.normals.clear();
    out.kinds.clear();
    out.indices.clear();
    out.lines.clear();
}

fn fill_mesh(out: &mut Output, body: &Body, draft: bool) -> Result<coreshape::Mesh> {
    let size = body.aabb().diagonal();
    let params = if draft { TessParams::draft_for_size(size) } else { TessParams::for_size(size) };
    let mesh = tessellate(body, &params)?;
    clear_mesh(out);
    for p in &mesh.positions {
        out.positions.extend_from_slice(&[p.x as f32, p.y as f32, p.z as f32]);
    }
    for n in &mesh.normals {
        out.normals.extend_from_slice(&[n.x as f32, n.y as f32, n.z as f32]);
    }
    out.kinds.resize(mesh.positions.len(), 0);
    for (t, &f) in mesh.triangles.iter().zip(&mesh.face_of_triangle) {
        let k = surface_kind(&body.faces[f as usize].surface);
        for &v in t {
            out.kinds[v as usize] = k;
            out.indices.push(v);
        }
    }
    for line in &mesh.edge_lines {
        for w in line.windows(2) {
            out.lines.extend_from_slice(&[
                w[0].x as f32,
                w[0].y as f32,
                w[0].z as f32,
                w[1].x as f32,
                w[1].y as f32,
                w[1].z as f32,
            ]);
        }
    }
    Ok(mesh)
}

/// Fills the output for a rebuild's outcome. A draft is for following a drag: a lighter mesh and no
/// mass properties, which are what the time goes on once the solid itself is built. The solid is the
/// same one, with every check done.
fn fill(out: &mut Output, outcome: &Outcome, draft: bool) {
    let features = features_json(outcome);
    let failed = outcome.failed.map_or("null".to_string(), |i| i.to_string());
    let rebuilt = outcome.steps.iter().filter(|s| s.state == StepState::Rebuilt).count();
    let head = format!("\"features\":{features},\"failed\":{failed},\"rebuilt\":{rebuilt}");

    let Some(body) = outcome.body.clone() else {
        clear_mesh(out);
        out.body = None;
        out.info = format!("{{\"ok\":true,\"empty\":true,{head}}}");
        return;
    };
    let mesh = match fill_mesh(out, &body, draft) {
        Ok(m) => m,
        Err(e) => {
            out.info = format!("{{\"ok\":false,\"error\":{},{head}}}", json_str(&e.to_string()));
            return;
        }
    };
    let report = match validate(&body) {
        Ok(r) => r,
        Err(e) => {
            out.info = format!("{{\"ok\":false,\"error\":{},{head}}}", json_str(&e.join("; ")));
            return;
        }
    };
    let (volume, area) = if draft {
        ("null".to_string(), "null".to_string())
    } else {
        let m = mass_properties(&body);
        (m.volume.to_string(), m.area.to_string())
    };
    let mut kinds = [0usize; 5];
    for f in &body.faces {
        kinds[surface_kind(&f.surface) as usize] += 1;
    }
    let genus: Vec<String> = report.genus.iter().map(|g| g.to_string()).collect();
    let bounds = mesh.bounds();
    out.info = format!(
        "{{\"ok\":true,\"draft\":{draft},{head},\"volume\":{},\"area\":{},\"vertices\":{},\"edges\":{},\"faces\":{},\"shells\":{},\"genus\":[{}],\"kinds\":[{},{},{},{},{}],\"triangles\":{},\"openEdges\":{},\"contacts\":{},\"bounds\":[{},{},{},{},{},{}]}}",
        volume,
        area,
        report.vertices,
        report.edges,
        report.faces,
        report.shells,
        genus.join(","),
        kinds[0],
        kinds[1],
        kinds[2],
        kinds[3],
        kinds[4],
        mesh.triangles.len(),
        mesh.open_edge_count(),
        body.contact_vertices(),
        bounds.min.x,
        bounds.min.y,
        bounds.min.z,
        bounds.max.x,
        bounds.max.y,
        bounds.max.z,
    );
    out.body = Some(body);
}

/// Rebuilds a model. `rollback` is the index of the last feature to apply, or
/// negative for all of them. `draft` is non-zero for a lighter result, while a dimension is
/// being dragged. Returns a pointer to a table of twelve `u32`s: pointer
/// and length for positions, normals, surface kinds, indices, edge segments and the
/// JSON info string.
///
/// # Safety
/// `params` must point to `n` readable `f64`s.
#[no_mangle]
pub unsafe extern "C" fn cs_build(model: usize, params: *const f64, n: usize, rollback: i32, draft: i32) -> *const u32 {
    let values = std::slice::from_raw_parts(params, n);
    STATE.with(|s| {
        let mut out = s.borrow_mut();
        let out = &mut *out;
        let catalog = models::catalog();
        if model < catalog.len() {
            let def = &catalog[model];
            let variant = def.variant(values);
            let doc = out.documents.entry((model, variant)).or_insert_with(|| def.document(variant));
            def.apply(doc, values);
            let upto = (rollback >= 0).then_some(rollback as usize);
            let outcome = doc.rebuild(upto);
            out.current = Some((model, variant));
            fill(out, &outcome, draft != 0);
        }
        out.table = [
            out.positions.as_ptr() as u32,
            out.positions.len() as u32,
            out.normals.as_ptr() as u32,
            out.normals.len() as u32,
            out.kinds.as_ptr() as u32,
            out.kinds.len() as u32,
            out.indices.as_ptr() as u32,
            out.indices.len() as u32,
            out.lines.as_ptr() as u32,
            out.lines.len() as u32,
            out.info.as_ptr() as u32,
            out.info.len() as u32,
        ];
        out.table.as_ptr()
    })
}

/// Writes the current body as binary STL. Returns its pointer; `cs_export_len`
/// gives its length. A null pointer means there is no body yet.
#[no_mangle]
pub extern "C" fn cs_export_stl() -> *const u8 {
    STATE.with(|s| {
        let mut out = s.borrow_mut();
        let out = &mut *out;
        let Some(body) = &out.body else {
            return std::ptr::null();
        };
        match tessellate(body, &TessParams::for_size(body.aabb().diagonal())) {
            Ok(mesh) => {
                out.export = coreshape::export::stl::write_binary(&mesh, "model");
                out.export.as_ptr()
            }
            Err(_) => std::ptr::null(),
        }
    })
}

/// Writes the current body as STEP. Same conventions as `cs_export_stl`.
#[no_mangle]
pub extern "C" fn cs_export_step() -> *const u8 {
    STATE.with(|s| {
        let mut out = s.borrow_mut();
        let out = &mut *out;
        let Some(body) = &out.body else {
            return std::ptr::null();
        };
        out.export = coreshape::export::step::write_step(body, "model").into_bytes();
        out.export.as_ptr()
    })
}

/// Writes the current document as text, in the native file format. Same conventions.
#[no_mangle]
pub extern "C" fn cs_export_document() -> *const u8 {
    STATE.with(|s| {
        let mut out = s.borrow_mut();
        let out = &mut *out;
        let Some(doc) = out.current.and_then(|k| out.documents.get(&k)) else {
            return std::ptr::null();
        };
        out.export = doc.save_text().into_bytes();
        out.export.as_ptr()
    })
}

#[no_mangle]
pub extern "C" fn cs_export_len() -> usize {
    STATE.with(|s| s.borrow().export.len())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every demo must build at its defaults.
    #[test]
    fn every_model_builds_at_defaults() {
        for m in models::catalog() {
            let mut doc = m.document(0);
            let o = doc.rebuild(None);
            assert!(o.failed.is_none(), "{}: {:?}", m.id, o.steps);
            validate(o.body.as_ref().unwrap()).unwrap();
        }
    }

    /// Each demo is plain data: saved and reloaded, it must build the identical solid.
    #[test]
    fn every_model_survives_a_save_and_load() {
        for m in models::catalog() {
            let max_variant = m.params.iter().find(|p| !p.choices.is_empty()).map_or(0, |p| p.max as usize);
            for variant in 0..=max_variant {
                let mut doc = m.document(variant);
                let text = doc.save_text();
                let mut back = Document::load_text(&text).unwrap_or_else(|e| panic!("{} {variant}: {e}\n{text}", m.id));
                assert_eq!(back.save_text(), text, "{} {variant}", m.id);
                let (a, b) = (doc.rebuild(None), back.rebuild(None));
                assert_eq!(a.failed, b.failed, "{} {variant}", m.id);
                if let (Some(a), Some(b)) = (a.body, b.body) {
                    assert_eq!(mass_properties(&a).volume.to_bits(), mass_properties(&b).volume.to_bits(), "{} {variant}", m.id);
                }
            }
        }
    }

    /// The same document must give the same bits, every build in every process. A second
    /// process would catch a seed that varies per process, which is what this once did;
    /// the two `determinism` example runs in `scripts/check_determinism.sh` compare that.
    #[test]
    fn building_twice_gives_identical_bits() {
        for m in models::catalog() {
            let variants = m.params.iter().find(|p| !p.choices.is_empty()).map_or(1, |p| p.max as usize + 1);
            for variant in 0..variants {
                let build = || {
                    let mut doc = m.document(variant);
                    doc.rebuild(None).body
                };
                let (Some(a), Some(b)) = (build(), build()) else { continue };
                let tag = format!("{} {variant}", m.id);
                assert_eq!(mass_properties(&a).volume.to_bits(), mass_properties(&b).volume.to_bits(), "{tag}");
                let mesh = |x: &Body| tessellate(x, &TessParams::for_size(x.aabb().diagonal())).unwrap();
                let (ma, mb) = (mesh(&a), mesh(&b));
                assert_eq!(ma.triangles, mb.triangles, "{tag}");
                assert_eq!(coreshape::export::step::write_step(&a, "m"), coreshape::export::step::write_step(&b, "m"), "{tag}");
            }
        }
    }

    /// Out and back in again must give the same solid, on real parts, whenever it's allowed at all.
    /// Refusing is fine. A wrong volume isn't.
    #[test]
    fn offsetting_the_demos_out_and_back_returns_their_volume() {
        let (mut tried, mut refused) = (0, 0);
        for m in models::catalog() {
            let Some(body) = m.document(0).rebuild(None).body else { continue };
            for d in [0.25, -0.25] {
                tried += 1;
                let moved = coreshape::offset(&body, d).and_then(|b| coreshape::offset(&b, -d));
                match moved {
                    Ok(back) => {
                        let (a, b) = (mass_properties(&back).volume, mass_properties(&body).volume);
                        assert!((a - b).abs() < 1e-5 * b, "{} by {d}: {a} against {b}", m.id);
                        validate(&back).unwrap();
                    }
                    Err(_) => refused += 1,
                }
            }
        }
        eprintln!("offset out and back: {refused} of {tried} refused");
        assert!(refused < tried, "every demo refused, so this tested nothing");
    }

    /// The stress cases must keep behaving as the slider promises: every case builds,
    /// except the last, which is the documented refusal.
    #[test]
    fn stress_cases_behave_as_documented() {
        let stress = models::catalog().iter().find(|m| m.id == "stress").unwrap();
        for case in 0..14 {
            let mut doc = stress.document(case);
            let o = doc.rebuild(None);
            assert_eq!(o.failed.is_some(), case == 13, "case {case}: {:?}", o.steps);
        }
    }

    /// Sliders may reach combinations with no valid solid. That must be an error
    /// reported on a feature, never a panic, and anything returned must be valid.
    #[test]
    fn sliders_never_panic() {
        let mut rng = Rng::new(99);
        for m in models::catalog() {
            let mut docs: HashMap<usize, Document> = HashMap::new();
            let mut failures = 0;
            let runs = 60;
            for _ in 0..runs {
                let values: Vec<f64> = m
                    .params
                    .iter()
                    .map(|p| {
                        let steps = ((p.max - p.min) / p.step).round();
                        p.min + rng.int(0, steps as i64) as f64 * p.step
                    })
                    .collect();
                let variant = m.variant(&values);
                let doc = docs.entry(variant).or_insert_with(|| m.document(variant));
                m.apply(doc, &values);
                let o = doc.rebuild(None);
                if o.failed.is_some() {
                    failures += 1;
                }
                if let Some(b) = &o.body {
                    validate(b).unwrap_or_else(|e| panic!("{} {values:?}: {e:?}", m.id));
                }
            }
            eprintln!("{}: {failures} of {runs} random slider positions were refused", m.id);
        }
    }
}
