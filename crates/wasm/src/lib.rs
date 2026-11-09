//! A plain C ABI over the kernel for the browser viewer.
//!
//! There is no binding generator: JavaScript passes numbers, and reads results
//! out of linear memory through a table of `(pointer, length)` pairs. This keeps
//! the crate free of dependencies and the download small.

pub mod models;

use coreshape::*;
use coreshape_assembly::Solved;
use coreshape_document::{Document, NamedBody, Outcome, StepState};
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
    /// One assembly per mechanism among the models, with the Part Studio it's made from.
    mechanisms: HashMap<usize, models::Mechanism>,
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
    // Each part its own tint, with the copies of one part alike.
    let mut parts = Vec::new();
    let tint_of: Vec<usize> = outcome
        .bodies
        .iter()
        .map(|b| {
            parts.iter().position(|id| *id == b.id).unwrap_or_else(|| {
                parts.push(b.id);
                parts.len() - 1
            })
        })
        .collect();
    for (t, &f) in mesh.triangles.iter().zip(&mesh.face_of_triangle) {
        let body_at = outcome.bodies.partition_point(|b| b.first_face <= f as usize).saturating_sub(1);
        let tint = tint_of.get(body_at).copied().unwrap_or(0) % KINDS as usize;
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

/// A solved assembly as an outcome to show: each part where it goes, as a body of its own.
fn assembled(steps: Vec<coreshape_document::Step>, failed: Option<usize>, solved: &Solved) -> Outcome {
    let moved: Vec<Body> = solved.placed.iter().map(|p| p.body.transformed(&p.at)).collect();
    let list: Vec<&Body> = moved.iter().collect();
    let body = (!list.is_empty()).then(|| Arc::new(Body::side_by_side(&list)));
    let (mut first_face, mut first_edge) = (0, 0);
    let mut bodies = Vec::with_capacity(moved.len());
    for (p, b) in solved.placed.iter().zip(&moved) {
        let (faces, edges) = (b.faces.len(), b.edges.len());
        bodies.push(NamedBody { id: p.part, body: Arc::new(b.clone()), first_face, first_edge });
        first_face += faces;
        first_edge += edges;
    }
    Outcome { steps, body, bodies, failed }
}

/// What a mechanism adds to a build's info: how free it is, where it's got to, and whether it got there.
fn assembly_json(step: &models::AssemblyStep, m: &models::Mechanism, solved: &Solved, asked: f64) -> String {
    let reached = solved.values.iter().find(|v| v.0 == m.drive && v.1 == coreshape_assembly::Dof::Angle).map_or(f64::NAN, |v| v.2);
    // Asked for 720 and stopped at 30 is a stop. Solved to within what the solver goes to is not.
    let stopped = (reached - asked).abs() > 1e-5;
    let failing: Vec<String> = solved.failing.iter().map(|f| format!("{{\"mate\":{},\"why\":{}}}", f.0, json_str(&f.1))).collect();
    let problems: Vec<String> = solved.problems.iter().map(|p| json_str(p)).collect();
    let readout: Vec<String> = (step.readout)(solved).iter().map(|r| format!("{{\"label\":{},\"value\":{},\"unit\":{}}}", json_str(r.0), r.1, json_str(r.2))).collect();
    let mates = m.project.assembly(m.assembly).map_or(0, |a| a.mates.len());
    format!(
        "{{\"parts\":{},\"mates\":{mates},\"freedom\":{},\"asked\":{asked},\"reached\":{reached},\"stopped\":{stopped},\"failing\":[{}],\"problems\":[{}],\"readout\":[{}]}}",
        solved.placed.len(),
        solved.freedom,
        failing.join(","),
        problems.join(","),
        readout.join(",")
    )
}

/// Builds a model at these slider values into the output.
fn build_into(out: &mut Output, model: usize, values: &[f64], rollback: i32, draft: bool) {
    let catalog = models::catalog();
    let def = &catalog[model];
    let variant = def.variant(values);
    let upto = (rollback >= 0).then_some(rollback as usize);
    out.current = Some((model, variant));
    if let Some(step) = &def.assembly {
        let m = out.mechanisms.entry(model).or_insert_with(|| def.mechanism().expect("it has an assembly"));
        let Ok(doc) = m.project.studio_mut(m.studio) else { return };
        def.apply(doc, values);
        let parts = doc.rebuild(upto);
        // Rolled back or stopped short, the parts are all there is to show, as a Part Studio has them.
        if upto.is_some() || parts.failed.is_some() {
            fill(out, &parts, draft);
            return;
        }
        let Some((solved, asked)) = def.move_mechanism(m, values) else {
            fill(out, &parts, draft);
            return;
        };
        let info = assembly_json(step, m, &solved, asked);
        let shown = assembled(parts.steps, parts.failed, &solved);
        fill(out, &shown, draft);
        if out.info.ends_with('}') {
            out.info.pop();
            let _ = write!(out.info, ",\"assembly\":{info}}}");
        }
        return;
    }
    let doc = out.documents.entry((model, variant)).or_insert_with(|| def.document(variant));
    def.apply(doc, values);
    let outcome = doc.rebuild(upto);
    fill(out, &outcome, draft);
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
            build_into(out, model, values, rollback, draft != 0);
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
        // A mechanism goes out as what it is: each part once, and a use of it for each instance.
        if let Some(m) = out.current.and_then(|k| out.mechanisms.get_mut(&k.0)) {
            let name = models::catalog().get(out.current.map_or(0, |k| k.0)).map_or("model", |d| d.name);
            let Ok(text) = m.project.step_text(m.assembly, name) else {
                return std::ptr::null();
            };
            out.export = text.into_bytes();
            return out.export.as_ptr();
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
        if let Some(m) = out.current.and_then(|k| out.mechanisms.get(&k.0)) {
            out.export = m.project.save_text().into_bytes();
            return out.export.as_ptr();
        }
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

    // ----- The slider-crank -----

    fn crank_model() -> usize {
        models::catalog().iter().position(|m| m.id == "crank").expect("the model is there")
    }

    fn crank_at(out: &mut Output, crank: f64, rod: f64, angle: f64) -> coreshape_document::json::Value {
        build_into(out, crank_model(), &[crank, rod, angle], -1, true);
        coreshape_document::json::parse(&out.info).unwrap_or_else(|e| panic!("{e}: {}", out.info))
    }

    fn field(v: &coreshape_document::json::Value, key: &str) -> f64 {
        v.get("assembly").and_then(|a| a.get(key)).and_then(|x| x.as_f64()).unwrap_or_else(|| panic!("no {key} in {}", v.to_text()))
    }

    fn piston(v: &coreshape_document::json::Value) -> f64 {
        let list = v.get("assembly").and_then(|a| a.get("readout")).and_then(|r| r.as_array()).expect("a readout");
        list[0].get("value").and_then(|x| x.as_f64()).expect("a value")
    }

    /// Where the piston's hole should be from the pivot, by the formula for a crank and rod.
    fn by_formula(crank: f64, rod: f64, degrees: f64) -> f64 {
        let a = degrees.to_radians();
        crank * a.cos() + (rod * rod - (crank * a.sin()).powi(2)).sqrt()
    }

    fn placed_at(out: &Output, name: &str) -> Frame {
        let m = out.mechanisms.get(&crank_model()).expect("it's been built");
        let mut project = m.project.clone();
        project.placed(m.assembly).unwrap().iter().find(|q| q.name == name).expect("that part is there").at
    }

    fn stopped(v: &coreshape_document::json::Value) -> bool {
        v.get("assembly").and_then(|a| a.get("stopped")).and_then(|x| x.as_bool()).expect("a stopped flag")
    }

    #[test]
    fn the_slider_crank_follows_its_formula_over_two_turns_at_any_lengths() {
        let def = &models::catalog()[crank_model()];
        let mut m = def.mechanism().unwrap();
        let mut worst = 0.0f64;
        for (crank, rod) in [(30.0, 100.0), (15.0, 160.0), (80.0, 160.0), (50.0, 60.0), (30.0, 40.0)] {
            for k in 0..=48 {
                let angle = k as f64 * 15.0;
                let (solved, asked) = def.move_mechanism(&mut m, &[crank, rod, angle]).unwrap();
                assert_eq!(asked, angle);
                assert!(solved.failing.is_empty() && solved.problems.is_empty(), "{crank} {rod} {angle}: {:?} {:?}", solved.failing, solved.problems);
                assert_eq!(solved.freedom, 1);
                let reached = solved.values.iter().find(|v| v.0 == m.drive && v.1 == coreshape_assembly::Dof::Angle).unwrap().2;
                assert!((reached - angle).abs() < 1e-5, "{crank} {rod} {angle}: reached {reached}");
                let at = |name: &str| solved.placed.iter().find(|q| q.name == name).unwrap().at;
                worst = worst.max((at("Piston").origin.x - by_formula(crank, rod, angle)).abs());
                // Its parts are where the layers and the guides say.
                let piston = at("Piston");
                assert!(piston.origin.y.abs() < 1e-6 && (piston.origin.z - 16.0).abs() < 1e-6, "{piston:?}");
                assert!(piston.x.dist(Vec3::X) < 1e-6 && piston.z.dist(Vec3::Z) < 1e-6, "the piston turned: {piston:?}");
                let tip = v3(crank * angle.to_radians().cos(), crank * angle.to_radians().sin(), 8.0);
                assert!(at("Rod").origin.dist(tip) < 1e-6, "{crank} {rod} {angle}: {:?} isn't {tip:?}", at("Rod").origin);
            }
        }
        assert!(worst < 1e-6, "the piston was off by {worst}");
    }

    #[test]
    fn what_the_page_gets_is_what_the_mechanism_did() {
        let mut out = Output::default();
        for (crank, rod, angle) in [(30.0, 100.0, 0.0), (30.0, 100.0, 77.0), (45.0, 70.0, 300.0), (45.0, 70.0, 720.0)] {
            let info = crank_at(&mut out, crank, rod, angle);
            assert_eq!(info.get("ok").and_then(|x| x.as_bool()), Some(true), "{}", info.to_text());
            assert_eq!(info.get("openEdges").and_then(|x| x.as_f64()), Some(0.0));
            assert!(!stopped(&info), "{}", info.to_text());
            assert_eq!(field(&info, "freedom"), 1.0);
            assert!((field(&info, "reached") - angle).abs() < 1e-5, "{}", info.to_text());
            assert!((piston(&info) - by_formula(crank, rod, angle)).abs() < 1e-6, "{}", info.to_text());
        }
    }

    #[test]
    fn changing_a_length_carries_the_mates_with_it_and_rebuilds_only_that_part() {
        let mut out = Output::default();
        crank_at(&mut out, 30.0, 100.0, 40.0);
        let (crank_before, frame_before) = (placed_at(&out, "Crank"), placed_at(&out, "Frame"));
        let info = crank_at(&mut out, 30.0, 137.5, 40.0);
        assert!((piston(&info) - by_formula(30.0, 137.5, 40.0)).abs() < 1e-6);
        // The rod is new, and so is the frame, whose guides reach past it. The crank isn't, and hasn't moved.
        assert_eq!(placed_at(&out, "Crank"), crank_before);
        assert_eq!(placed_at(&out, "Frame"), frame_before);
        let list = info.get("features").and_then(|f| f.as_array()).expect("features");
        let state = |name: &str| list.iter().find(|f| f.get("name").and_then(|n| n.as_str()) == Some(name)).and_then(|f| f.get("state")).and_then(|x| x.as_str()).map(str::to_string);
        assert_eq!(state("Crank").as_deref(), Some("reused"), "{}", info.to_text());
        assert_eq!(state("Rod").as_deref(), Some("rebuilt"), "{}", info.to_text());
        // Moving only the angle rebuilds nothing at all.
        let info = crank_at(&mut out, 30.0, 137.5, 200.0);
        assert_eq!(info.get("rebuilt").and_then(|x| x.as_f64()), Some(0.0), "{}", info.to_text());
        assert!((piston(&info) - by_formula(30.0, 137.5, 200.0)).abs() < 1e-6);
    }

    #[test]
    fn a_crank_longer_than_its_rod_stops_where_the_rod_runs_out_and_says_so() {
        let mut out = Output::default();
        // With a rod half its length, the crank can't get past 30 degrees from the X axis.
        let info = crank_at(&mut out, 80.0, 40.0, 10.0);
        assert!(!stopped(&info), "{}", info.to_text());
        assert!((piston(&info) - by_formula(80.0, 40.0, 10.0)).abs() < 1e-6);
        let info = crank_at(&mut out, 80.0, 40.0, 90.0);
        assert_eq!(info.get("ok").and_then(|x| x.as_bool()), Some(true));
        assert!(stopped(&info), "{}", info.to_text());
        let reached = field(&info, "reached");
        assert!((reached - 30.0).abs() < 0.01 && reached <= 30.0, "stopped at {reached}");
        assert!(info.get("assembly").and_then(|a| a.get("failing")).and_then(|f| f.as_array()).expect("a list").is_empty());
        // It really is there, with the mates all met: the rod spans from the crank's tip to the piston at its own length.
        // Right at the stop the formula is no use, as a thousandth of a degree moves the piston by half a millimetre.
        let a = reached.to_radians();
        let tip = v3(80.0 * a.cos(), 80.0 * a.sin(), 0.0);
        assert!((tip.dist(v3(piston(&info), 0.0, 0.0)) - 40.0).abs() < 1e-5, "{tip:?} to {}", piston(&info));
        // Coming back works.
        let info = crank_at(&mut out, 80.0, 40.0, 20.0);
        assert!(!stopped(&info));
        assert!((piston(&info) - by_formula(80.0, 40.0, 20.0)).abs() < 1e-6);
    }

    #[test]
    fn lengths_that_rule_out_where_it_is_move_it_to_where_it_can_be() {
        let mut out = Output::default();
        // At 90 degrees with these, the rod reaches. Then the crank grows until it can't.
        crank_at(&mut out, 30.0, 100.0, 90.0);
        let info = crank_at(&mut out, 80.0, 40.0, 90.0);
        assert_eq!(info.get("ok").and_then(|x| x.as_bool()), Some(true), "{}", info.to_text());
        assert!(stopped(&info), "{}", info.to_text());
        // Wherever it went, the rod spans from the crank's tip to the piston's hole, at the rod's own length.
        let reached = field(&info, "reached").to_radians();
        let tip = v3(80.0 * reached.cos(), 80.0 * reached.sin(), 0.0);
        let hole = v3(piston(&info), 0.0, 0.0);
        assert!((tip.dist(hole) - 40.0).abs() < 1e-5, "{tip:?} and {hole:?}");
        // Back to lengths that can do it, it gets back to what's asked.
        let info = crank_at(&mut out, 30.0, 100.0, 90.0);
        assert!(!stopped(&info), "{}", info.to_text());
        assert!((piston(&info) - by_formula(30.0, 100.0, 90.0)).abs() < 1e-6);
    }

    #[test]
    fn a_crank_as_long_as_its_rod_stays_on_its_mates_through_the_dead_point() {
        let mut out = Output::default();
        for k in 0..=24 {
            let angle = k as f64 * 15.0;
            let info = crank_at(&mut out, 40.0, 40.0, angle);
            assert_eq!(info.get("ok").and_then(|x| x.as_bool()), Some(true), "{}", info.to_text());
            let reached = field(&info, "reached").to_radians();
            let tip = v3(40.0 * reached.cos(), 40.0 * reached.sin(), 0.0);
            assert!((tip.dist(v3(piston(&info), 0.0, 0.0)) - 40.0).abs() < 1e-5, "at {angle}: {}", info.to_text());
        }
    }

    #[test]
    fn the_slider_crank_goes_out_as_an_assembly_and_comes_back() {
        let model = crank_model();
        let values = [30.0, 100.0, 60.0];
        // Through the same calls the page makes.
        unsafe { cs_build(model, values.as_ptr(), values.len(), -1, 0) };
        let export = |p: *const u8| unsafe { String::from_utf8(std::slice::from_raw_parts(p, cs_export_len()).to_vec()).unwrap() };
        let step = export(cs_export_step());
        let structure = coreshape::import::step::read_step_structure(&step).unwrap();
        assert_eq!(structure.names, vec!["Frame", "Crank", "Rod", "Piston", "Pin"]);
        assert_eq!(structure.instances.len(), 7);
        assert_eq!(step.matches("MANIFOLD_SOLID_BREP").count(), 5);
        // The rod's where the formula says, in the file.
        let (rod, at) = structure.instances[2];
        assert_eq!(rod, 2);
        assert!(at.origin.dist(v3(30.0 * 60f64.to_radians().cos(), 30.0 * 60f64.to_radians().sin(), 8.0)) < 1e-6, "{:?}", at.origin);
        // Read back, every use is whole.
        let bodies = coreshape::import::step::read_step(&step, 1).unwrap();
        assert_eq!(bodies.len(), 7);
        let text = export(cs_export_document());
        assert!(text.contains("coreshape-project"));
        let mut back = coreshape_assembly::Project::load_text(&text).unwrap();
        let tab = back.tabs[1].id;
        let solved = back.solve(tab).unwrap();
        assert!(solved.failing.is_empty(), "{:?}", solved.failing);
        assert_eq!(solved.freedom, 1);
        let piston = solved.placed.iter().find(|q| q.name == "Piston").unwrap();
        assert!((piston.at.origin.x - by_formula(30.0, 100.0, 60.0)).abs() < 1e-6);
    }

    #[test]
    fn the_slider_cranks_parts_dont_run_into_each_other_as_it_turns() {
        let def = &models::catalog()[crank_model()];
        let mut m = def.mechanism().unwrap();
        for k in 0..4 {
            let angle = 40.0 + k as f64 * 90.0;
            let (solved, _) = def.move_mechanism(&mut m, &[30.0, 100.0, angle]).unwrap();
            assert!(solved.failing.is_empty());
            let (found, unknown) = m.project.interferences(m.assembly).unwrap();
            assert!(found.is_empty(), "at {angle}: {found:?}");
            assert!(unknown.is_empty(), "at {angle}: {unknown:?}");
        }
        let bom = m.project.bom(m.assembly).unwrap();
        let counts: Vec<(&str, usize)> = bom.rows.iter().map(|r| (r.name.as_str(), r.count)).collect();
        assert_eq!(counts, vec![("Frame", 1), ("Crank", 1), ("Rod", 1), ("Piston", 1), ("Pin", 3)]);
    }

    #[test]
    fn rolled_back_a_mechanism_shows_its_parts_as_a_part_studio_has_them() {
        let mut out = Output::default();
        build_into(&mut out, crank_model(), &[30.0, 100.0, 45.0], 1, true);
        let info = coreshape_document::json::parse(&out.info).unwrap();
        assert!(info.get("assembly").is_none(), "{}", info.to_text());
        assert_eq!(info.get("bodies").and_then(|x| x.as_f64()), Some(1.0));
        // And rolled forward, it's the mechanism again.
        let info = crank_at(&mut out, 30.0, 100.0, 45.0);
        assert_eq!(info.get("bodies").and_then(|x| x.as_f64()), Some(7.0));
        assert_eq!(field(&info, "parts"), 7.0);
    }

    #[test]
    fn copies_of_a_part_share_a_tint_and_other_parts_dont() {
        let mut out = Output::default();
        crank_at(&mut out, 30.0, 100.0, 45.0);
        let mut tints: Vec<u8> = out.kinds.iter().map(|k| k / KINDS).collect();
        tints.sort_unstable();
        tints.dedup();
        // Frame, crank, rod, piston and pin, though there are seven instances. An instance each would have them wrap round.
        assert_eq!(tints, vec![0, 1, 2, 3, 4], "{tints:?}");
    }

    // ----- The rest -----

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
