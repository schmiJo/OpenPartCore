//! STL export. STL only holds triangles, so this writes the tessellation.

use crate::tess::Mesh;

/// Binary STL. Normals are stored per facet, computed from the triangle.
pub fn write_binary(mesh: &Mesh, name: &str) -> Vec<u8> {
    let mut out = Vec::with_capacity(84 + mesh.triangles.len() * 50);
    let mut header = [0u8; 80];
    let label = format!("coreshape {name}");
    let n = label.len().min(80);
    header[..n].copy_from_slice(&label.as_bytes()[..n]);
    out.extend_from_slice(&header);
    out.extend_from_slice(&(mesh.triangles.len() as u32).to_le_bytes());
    for t in &mesh.triangles {
        let (a, b, c) = (mesh.positions[t[0] as usize], mesh.positions[t[1] as usize], mesh.positions[t[2] as usize]);
        let n = (b - a).cross(c - a);
        let n = if n.norm() > 0.0 { n.normalized() } else { n };
        for v in [n, a, b, c] {
            for x in [v.x, v.y, v.z] {
                out.extend_from_slice(&(x as f32).to_le_bytes());
            }
        }
        out.extend_from_slice(&[0, 0]);
    }
    out
}

/// ASCII STL, for inspection and diffing.
pub fn write_ascii(mesh: &Mesh, name: &str) -> String {
    use std::fmt::Write;
    let mut s = format!("solid {name}\n");
    for t in &mesh.triangles {
        let (a, b, c) = (mesh.positions[t[0] as usize], mesh.positions[t[1] as usize], mesh.positions[t[2] as usize]);
        let n = (b - a).cross(c - a);
        let n = if n.norm() > 0.0 { n.normalized() } else { n };
        let _ = writeln!(s, "  facet normal {} {} {}\n    outer loop", n.x, n.y, n.z);
        for v in [a, b, c] {
            let _ = writeln!(s, "      vertex {} {} {}", v.x, v.y, v.z);
        }
        let _ = writeln!(s, "    endloop\n  endfacet");
    }
    let _ = writeln!(s, "endsolid {name}");
    s
}
