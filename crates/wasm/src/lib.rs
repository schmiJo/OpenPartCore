//! A plain C ABI over the kernel for the browser viewer.
//!
//! There is no binding generator: JavaScript passes numbers, and reads results
//! out of linear memory through a table of `(pointer, length)` pairs. This keeps
//! the crate free of dependencies and the download small.

pub mod models;

use coreshape::*;
use std::cell::RefCell;
use std::fmt::Write;

#[derive(Default)]
struct Output {
    positions: Vec<f32>,
    normals: Vec<f32>,
    kinds: Vec<u8>,
    indices: Vec<u32>,
    lines: Vec<f32>,
    info: String,
    export: Vec<u8>,
    body: Option<Body>,
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
                        "{{\"key\":{},\"label\":{},\"unit\":{},\"min\":{},\"max\":{},\"step\":{},\"value\":{}}}",
                        json_str(p.key),
                        json_str(p.label),
                        json_str(p.unit),
                        p.min,
                        p.max,
                        p.step,
                        p.default
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

fn fill(out: &mut Output, body: &Body) -> Result<()> {
    let size = body.aabb().diagonal();
    let mesh = tessellate(body, &TessParams::for_size(size))?;
    out.positions.clear();
    out.normals.clear();
    out.kinds.clear();
    out.indices.clear();
    out.lines.clear();
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

    let report = validate(body).map_err(Error::InvalidBody)?;
    let m = mass_properties(body);
    let mut kinds = [0usize; 5];
    for f in &body.faces {
        kinds[surface_kind(&f.surface) as usize] += 1;
    }
    let genus: Vec<String> = report.genus.iter().map(|g| g.to_string()).collect();
    let bounds = mesh.bounds();
    out.info = format!(
        "{{\"ok\":true,\"volume\":{},\"area\":{},\"meshVolume\":{},\"vertices\":{},\"edges\":{},\"faces\":{},\"shells\":{},\"genus\":[{}],\"kinds\":[{},{},{},{},{}],\"triangles\":{},\"openEdges\":{},\"bounds\":[{},{},{},{},{},{}]}}",
        m.volume,
        m.area,
        mesh.volume(),
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
        bounds.min.x,
        bounds.min.y,
        bounds.min.z,
        bounds.max.x,
        bounds.max.y,
        bounds.max.z,
    );
    Ok(())
}

/// Builds a model. Returns a pointer to a table of twelve `u32`s: pointer and
/// length for positions, normals, surface kinds, indices, edge segments and the
/// JSON info string.
///
/// # Safety
/// `params` must point to `n` readable `f64`s.
#[no_mangle]
pub unsafe extern "C" fn cs_build(model: usize, params: *const f64, n: usize) -> *const u32 {
    let values = std::slice::from_raw_parts(params, n);
    STATE.with(|s| {
        let mut out = s.borrow_mut();
        let out = &mut *out;
        let result = models::catalog()
            .get(model)
            .ok_or_else(|| Error::InvalidInput("no such model".into()))
            .and_then(|m| (m.build)(values))
            .and_then(|body| {
                fill(out, &body)?;
                out.body = Some(body);
                Ok(())
            });
        if let Err(e) = result {
            // Leave the last good mesh in place so the view does not go blank.
            out.info = format!("{{\"ok\":false,\"error\":{}}}", json_str(&e.to_string()));
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
        let size = body.aabb().diagonal();
        match tessellate(body, &TessParams::for_size(size)) {
            Ok(mesh) => {
                out.export = coreshape::export::stl::write_binary(&mesh, "model");
                out.export.as_ptr()
            }
            Err(_) => std::ptr::null(),
        }
    })
}

#[no_mangle]
pub extern "C" fn cs_export_len() -> usize {
    STATE.with(|s| s.borrow().export.len())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every demo must build at its defaults and at all slider extremes that the
    /// viewer offers, or fail with an error rather than a panic.
    #[test]
    fn every_model_builds_at_defaults() {
        for m in models::catalog() {
            let values: Vec<f64> = m.params.iter().map(|p| p.default).collect();
            let body = (m.build)(&values).unwrap_or_else(|e| panic!("{}: {e}", m.id));
            validate(&body).unwrap();
        }
    }

    #[test]
    fn sliders_never_panic() {
        let mut rng = Rng::new(99);
        for m in models::catalog() {
            for _ in 0..200 {
                let values: Vec<f64> = m.params.iter().map(|p| rng.range(p.min, p.max)).collect();
                if let Ok(body) = (m.build)(&values) {
                    let report = validate(&body);
                    assert!(report.is_ok(), "{} {values:?}: {report:?}", m.id);
                    let mesh = tessellate(&body, &TessParams::for_size(body.aabb().diagonal())).unwrap();
                    assert_eq!(mesh.open_edge_count(), 0, "{} {values:?}", m.id);
                }
            }
        }
    }
}
