//! What an app shows: flat arrays for a part, and for each instance of an assembly.

use coreshape::*;
use coreshape_document::json::{self, Value};
use coreshape_mcp::{FlatMesh, Session};
use std::path::PathBuf;
use std::sync::Arc;

fn session() -> Session {
    Session::new(PathBuf::from("."))
}

fn call(s: &mut Session, tool: &str, args: &str) -> String {
    let answer = s.call_safely(tool, &json::parse(args).unwrap());
    let text: Vec<&str> = answer.get("content").and_then(Value::as_array).unwrap().iter().filter_map(|c| c.get("text").and_then(Value::as_str)).collect();
    let text = text.join("\n");
    assert_eq!(answer.get("isError").and_then(Value::as_bool), Some(false), "{tool} {args}: {text}");
    text
}

fn two_parts() -> Session {
    let mut s = session();
    call(&mut s, "feature_add", r#"{"feature":{"type":"solid","shape":"box","dx":10,"dy":20,"dz":30,"mode":"new_body"}}"#);
    call(&mut s, "feature_add", r#"{"feature":{"type":"solid","shape":"cylinder","radius":3,"height":10,"at":{"origin":[50,0,0]},"mode":"new_body"}}"#);
    s
}

fn bounds(m: &FlatMesh) -> ([f32; 3], [f32; 3]) {
    let (mut lo, mut hi) = ([f32::MAX; 3], [f32::MIN; 3]);
    for p in m.positions.chunks_exact(3) {
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    (lo, hi)
}

#[test]
fn a_parts_mesh_is_flat_arrays_of_that_part_alone() {
    let mut s = two_parts();
    let m = s.part_mesh("f1", None, false).unwrap();
    assert_eq!(bounds(&m), ([0.0; 3], [10.0, 20.0, 30.0]));
    // Everything lines up: three numbers a vertex, one face a vertex, triangles, and six numbers a segment.
    assert_eq!(m.positions.len() % 3, 0);
    assert_eq!(m.normals.len(), m.positions.len());
    assert_eq!(m.faces.len() * 3, m.positions.len());
    assert_eq!(m.indices.len() % 3, 0);
    assert_eq!(m.segments.len(), m.segment_ids.len() * 6);
    assert!(m.indices.iter().all(|i| (*i as usize) < m.faces.len()));
    // Its numbers are the ones its own faces_list and edges_list use.
    assert!(m.faces.iter().all(|f| *f < 6), "{:?}", m.faces.iter().max());
    assert!(m.segment_ids.iter().all(|e| *e < 12));
    let other = s.part_mesh("f2", None, false).unwrap();
    assert_eq!(bounds(&other).0[0], 47.0);
    assert!(s.part_mesh("f9", None, false).unwrap_err().contains("f1, f2"));
    assert!(s.part_mesh("nonsense", None, false).is_err());
    // A draft is lighter on anything round.
    assert!(s.part_mesh("f2", None, true).unwrap().indices.len() < other.indices.len());
}

#[test]
fn a_closed_surfaces_seam_isnt_drawn_as_an_edge() {
    let mut s = two_parts();
    let m = s.part_mesh("f2", None, false).unwrap();
    // A cylinder's two halves meet in two straight lines along it, which aren't edges to show.
    let along_it = m.segments.chunks_exact(6).filter(|w| (w[0] - w[3]).abs() < 1e-3 && (w[1] - w[4]).abs() < 1e-3).count();
    assert_eq!(along_it, 0);
    assert!(!m.segments.is_empty());
}

#[test]
fn a_mesh_is_only_worked_out_again_when_the_part_changes() {
    let mut s = two_parts();
    let first = s.part_mesh("f1", None, false).unwrap();
    assert!(Arc::ptr_eq(&first, &s.part_mesh("f1", None, false).unwrap()));
    assert!(!Arc::ptr_eq(&first, &s.part_mesh("f1", None, true).unwrap()));
    // A change to the other part leaves this one's alone.
    let round = s.part_mesh("f2", None, false).unwrap();
    call(&mut s, "feature_edit", r#"{"id":2,"feature":{"type":"solid","shape":"cylinder","radius":4,"height":10,"at":{"origin":[50,0,0]},"mode":"new_body"}}"#);
    assert!(Arc::ptr_eq(&first, &s.part_mesh("f1", None, false).unwrap()));
    let changed = s.part_mesh("f2", None, false).unwrap();
    assert!(!Arc::ptr_eq(&round, &changed));
    assert_eq!(bounds(&changed).0[0], 46.0);
}

#[test]
fn an_assembly_gives_a_mesh_and_a_place_for_each_instance() {
    let mut s = two_parts();
    call(&mut s, "tab_new", r#"{"type":"assembly"}"#);
    call(&mut s, "instance_add", r#"{"part":"f1","fixed":true,"name":"Block"}"#);
    call(&mut s, "instance_add", r#"{"part":"f2","name":"Pin","at":{"origin":[0,0,100]}}"#);
    call(&mut s, "instance_add", r#"{"part":"f2","name":"Other pin","at":{"origin":[0,0,200],"z":[0,1,0],"x":[1,0,0]}}"#);
    let (solved, list) = s.instance_meshes(false).unwrap();
    assert_eq!(list.len(), 3);
    assert_eq!(solved.placed.len(), 3);
    assert_eq!(list[1].name, "Pin");
    // One mesh for both pins, in the part's own coordinates.
    assert!(Arc::ptr_eq(&list[1].mesh, &list[2].mesh));
    assert_eq!(bounds(&list[1].mesh).0[0], 47.0);
    // Its matrix by columns, as WebGL takes it.
    assert_eq!(&list[1].matrix[12..16], &[0.0, 0.0, 100.0, 1.0]);
    assert_eq!(&list[2].matrix[8..11], &[0.0, 1.0, 0.0]);
    // In one space if that's wanted: a point on the first pin's top, moved up by 100.
    let placed = list[1].mesh.placed(&list[1].at);
    assert_eq!(bounds(&placed).1[2], 110.0);
    assert_eq!(placed.faces, list[1].mesh.faces);
    assert_eq!(placed.indices, list[1].mesh.indices);
    // And a normal only turns: the turned pin's end cap faces along y, not off somewhere by 200.
    let turned = list[2].mesh.placed(&list[2].at);
    assert!(turned.normals.chunks_exact(3).all(|n| (n[0] * n[0] + n[1] * n[1] + n[2] * n[2] - 1.0).abs() < 1e-3));
    assert!(turned.normals.chunks_exact(3).any(|n| (n[1] - 1.0).abs() < 1e-3));

    // Moving something doesn't work out any mesh again.
    let before = list[1].mesh.clone();
    call(&mut s, "assembly_drag", &format!(r#"{{"instance":{},"to":[9,9,9]}}"#, list[1].path[0]));
    let (_, again) = s.instance_meshes(false).unwrap();
    assert!(Arc::ptr_eq(&before, &again[1].mesh));
    assert_eq!(again[1].at.origin.dist(v3(9.0, 9.0, 9.0)) < 1e-6, true);
    // With no assembly selected, it says so.
    call(&mut s, "tab_select", r#"{"tab":1}"#);
    assert!(s.instance_meshes(false).unwrap_err().contains("no assembly"));
}
