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
    /// Each of the document's bodies, for writing them out as what they are.
    bodies: Vec<Arc<Body>>,
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
        Surface::Extruded { .. } | Surface::Revolved { .. } | Surface::Free { .. } => 5,
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

/// How many tints there are for telling bodies apart. A vertex's kind says both: its surface's kind, and its body's tint.
const KINDS: u8 = 6;

fn fill_mesh(out: &mut Output, outcome: &Outcome, body: &Body, draft: bool) -> Result<coreshape::Mesh> {
    let size = body.aabb().diagonal();
    let params = if draft { TessParams::draft_for_size(size) } else { TessParams::display_for_size(size) };
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
        let tint = outcome.bodies.partition_point(|b| b.first_face <= f as usize).saturating_sub(1) % KINDS as usize;
        let k = surface_kind(&body.faces[f as usize].surface) + KINDS * tint as u8;
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

    out.bodies = outcome.bodies.iter().map(|b| b.body.clone()).collect();
    let Some(body) = outcome.body.clone() else {
        clear_mesh(out);
        out.body = None;
        out.info = format!("{{\"ok\":true,\"empty\":true,{head}}}");
        return;
    };
    let mesh = match fill_mesh(out, outcome, &body, draft) {
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
    let mut kinds = [0usize; 6];
    for f in &body.faces {
        kinds[surface_kind(&f.surface) as usize] += 1;
    }
    let genus: Vec<String> = report.genus.iter().map(|g| g.to_string()).collect();
    let bounds = mesh.bounds();
    let bodies = outcome.bodies.len();
    out.info = format!(
        "{{\"ok\":true,\"draft\":{draft},{head},\"bodies\":{bodies},\"volume\":{},\"area\":{},\"vertices\":{},\"edges\":{},\"faces\":{},\"shells\":{},\"genus\":[{}],\"kinds\":[{},{},{},{},{}],\"triangles\":{},\"openEdges\":{},\"contacts\":{},\"bounds\":[{},{},{},{},{},{}]}}",
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
        if out.bodies.is_empty() {
            return std::ptr::null();
        }
        let list: Vec<&Body> = out.bodies.iter().map(Arc::as_ref).collect();
        out.export = coreshape::export::step::write_step_bodies(&list, "model").into_bytes();
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
    fn bolted_joint_demo() {
        use std::f64::consts::PI;
        let def = models::catalog().iter().find(|m| m.id == "joint").unwrap();
        let mut doc = def.document(0);
        let o = doc.rebuild(None);
        assert_eq!(o.failed, None, "{:?}", o.steps);
        let n = 6.0;
        // Two flanges, and a bolt and a nut in each hole, all of them separate.
        assert_eq!(o.bodies.len(), 2 + 2 * 6);
        let at = |key: &str| def.params.iter().find(|p| p.key == key).unwrap().default;
        let (outer, thick, bolt) = (at("outer") / 2.0, at("thick"), at("bolt") / 2.0);
        let hole = PI * bolt * bolt;
        let lower = (PI * outer * outer - n * hole) * thick;
        let v = |i: usize| coreshape::mass_properties(&o.bodies[i].body).volume;
        assert!((v(0) - lower).abs() < 1e-6, "{} against {lower}", v(0));
        // The upper one's rim is rounded, and nothing else is.
        assert!(v(1) < lower && v(1) > 0.99 * lower);
        let (nut_r, nut_h) = (0.95 * at("bolt"), 0.8 * at("bolt"));
        let nut = (3.0 * 3f64.sqrt() / 2.0 * nut_r * nut_r - hole) * nut_h;
        let (head_r, head_h, tail) = (0.85 * at("bolt"), 0.6 * at("bolt"), 3.0);
        let length = nut_h + tail + 2.0 * thick;
        let screw = hole * length + PI * head_r * head_r * head_h;
        for b in &o.bodies[2..] {
            let got = coreshape::mass_properties(&b.body).volume;
            assert!((got - screw).abs() < 1e-6 || (got - nut).abs() < 1e-6, "{} is {got}, not {screw} or {nut}", b.id.text());
        }
        // They touch all around, and each of them is whole.
        let whole = o.body.as_ref().unwrap();
        validate(whole).unwrap();
        let mesh = tessellate(whole, &TessParams::for_size(whole.aabb().diagonal())).unwrap();
        assert_eq!(mesh.open_edge_count(), 0);
        // A flange's size changes the flanges, and nothing that's bolted through them.
        doc.set_parameter("outer", at("outer") + 10.0).unwrap();
        let o = doc.rebuild(None);
        assert_eq!(o.failed, None);
        let rebuilt: Vec<&str> = o.steps.iter().filter(|s| s.state == StepState::Rebuilt).map(|s| s.kind).collect();
        assert!(!rebuilt.contains(&"pattern") && !rebuilt.contains(&"revolve"), "{rebuilt:?}");
        assert!(rebuilt.contains(&"extrude"));
    }

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
            let runs = if matches!(m.id, "screw" | "cup" | "spring") { 8 } else { 60 };
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

    /// Every face of every demo has a key that picks out only that face, wherever the sliders are.
    #[test]
    fn every_face_has_a_key_of_its_own_at_random_slider_positions() {
        use coreshape_document::{face_keys, find_faces, FaceKey};
        let mut rng = coreshape::Rng::new(77);
        for m in models::catalog() {
            let mut docs = std::collections::HashMap::new();
            let tries = if matches!(m.id, "screw" | "cup" | "spring") { 3 } else { 12 };
            for _ in 0..tries {
                let values: Vec<f64> = m
                    .params
                    .iter()
                    .map(|p| p.min + rng.int(0, ((p.max - p.min) / p.step).round() as i64) as f64 * p.step)
                    .collect();
                let variant = m.variant(&values);
                let doc = docs.entry(variant).or_insert_with(|| m.document(variant));
                m.apply(doc, &values);
                let Some(body) = doc.rebuild(None).body else { continue };
                for (i, key) in face_keys(&body).iter().enumerate() {
                    let back = FaceKey::parse(&key.text()).unwrap();
                    assert_eq!(find_faces(&body, &back), vec![i], "{} {values:?}: {}", m.id, key.text());
                }
            }
        }
    }

    /// A small slider move keeps every key on the same kind of face in about the same place, which is what
    /// makes a key worth saving. Two things are left out because a key isn't what changes there: a slider
    /// that's a count, which moves what it repeats, and the crossed cylinders, whose default is exactly
    /// coaxial, so one step can remove a face or change which seam a cylinder keeps. On the threaded shaft, a thread cuts
    /// the shaft's wall into many pieces under one name, and those are numbered by position, so their numbers aren't kept.
    #[test]
    fn a_key_means_the_same_face_after_a_small_slider_move() {
        use coreshape_document::{face_keys, FaceKey};
        use std::collections::HashMap;
        let mut rng = coreshape::Rng::new(78);
        let (mut compared, mut kept, mut lost, mut moved) = (0usize, 0usize, Vec::new(), Vec::new());
        for m in models::catalog() {
            // The crossing walls demo is left out as well: faces going away as a slider moves is its whole point.
            if m.id == "crossed" || m.id == "walls" {
                continue;
            }
            // The slowest ones get fewer tries, as this is the same check on each.
            let tries = if matches!(m.id, "screw" | "cup" | "spring") { 4 } else { 12 };
            for _ in 0..tries {
                let mut values: Vec<f64> = m.params.iter().map(|p| p.default).collect();
                let which = rng.int(0, m.params.len() as i64 - 1) as usize;
                let p = &m.params[which];
                if p.unit.is_empty() {
                    continue;
                }
                let step = if values[which] + p.step <= p.max { p.step } else { -p.step };
                let index = |b: &coreshape::Body| -> HashMap<FaceKey, (&'static str, coreshape::math::Vec3)> {
                    face_keys(b).into_iter().enumerate().map(|(i, k)| (k, (b.faces[i].surface.name(), b.face_aabb(i).center()))).collect()
                };
                let mut doc = m.document(m.variant(&values));
                m.apply(&mut doc, &values);
                let Some(before) = doc.rebuild(None).body else { continue };
                values[which] += step;
                if m.variant(&values) != m.variant(&m.params.iter().map(|p| p.default).collect::<Vec<_>>()) {
                    continue;
                }
                m.apply(&mut doc, &values);
                let outcome = doc.rebuild(None);
                if outcome.failed.is_some() {
                    continue;
                }
                let Some(after) = outcome.body else { continue };
                let (a, b) = (index(&before), index(&after));
                for (key, (kind, at)) in &a {
                    if m.id == "screw" && key.piece.is_some() {
                        continue;
                    }
                    compared += 1;
                    match b.get(key) {
                        None => lost.push(format!("{} {}: {}", m.id, p.key, key.text())),
                        Some((other, _)) if other != kind => moved.push(format!("{} {}: {} became a {other}", m.id, p.key, key.text())),
                        // A face can shift with the dimension, but not by far more than the step. The last half turn of a
                        // helix also gets longer or shorter, which can move its middle by about the helix's radius.
                        Some((_, to)) if to.dist(*at) > 8.0 * step.abs() + 1.0 + if m.id == "screw" { 8.0 } else { 0.0 } => {
                            moved.push(format!("{} {}: {} moved {:.2} for a step of {step}", m.id, p.key, key.text(), to.dist(*at)))
                        }
                        _ => kept += 1,
                    }
                }
            }
        }
        eprintln!("{kept} of {compared} keys kept; lost {}, moved {}", lost.len(), moved.len());
        for line in lost.iter().chain(&moved) {
            eprintln!("  {line}");
        }
        assert!(moved.is_empty() && lost.is_empty(), "{} lost, {} moved", lost.len(), moved.len());
    }
}

#[cfg(test)]
mod crossing_walls_demo {
    use crate::models;
    use coreshape_document::Document;

    /// The demo's whole point: as the slider moves, a wall gives way, and the volume keeps matching what's worked out by hand.
    #[test]
    fn the_walls_give_way_and_the_volume_is_right() {
        let m = models::catalog().iter().find(|m| m.id == "walls").unwrap();
        let mut doc = Document::new();
        for p in m.params {
            doc.set_parameter(p.key, p.default).unwrap();
        }
        (m.build)(&mut doc, 0);
        // Pockets 12 by 18, the first from the side wall at 2, then a wall of 2, floors at 5, in a block 60 by 30 by 20.
        let want = |d: f64| {
            let (x0, x1) = (d, 60.0 - d);
            let (a0, a1, b0, b1) = ((2.0 - d).max(x0), 14.0 + d, 16.0 - d, 28.0 + d);
            let cut = if a1 >= b0 { b1 - a0 } else { (a1 - a0) + (b1 - b0) };
            // Moving in takes the top down and a pocket's floor down too, by the same amount, so a pocket stays 15 deep,
            // until its floor passes the bottom coming up.
            let deep = (20.0 - d) - (5.0 - d).max(d);
            (x1 - x0) * (30.0 - 2.0 * d) * (20.0 - 2.0 * d) - cut * (18.0 + 2.0 * d) * deep
        };
        for d in [0.5, 0.75, 1.25, 1.5, 2.5, 3.0] {
            doc.set_parameter("inward", d).unwrap();
            let o = doc.rebuild(None);
            assert_eq!(o.failed, None, "moved in by {d}: {:?}", o.steps);
            let v = coreshape::mass_properties(o.body.as_ref().unwrap()).volume;
            assert!((v - want(d)).abs() < 1e-6 * want(d), "moved in by {d}: {v} against {}", want(d));
        }
    }
}
