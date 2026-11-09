//! Meshes for an app to show, as flat arrays that go straight to a graphics card. A part's mesh is kept while the part stays
//! the same, so moving things around an assembly costs nothing here.

use super::*;
use coreshape::Frame;
use coreshape_assembly::Solved;

/// A mesh as flat arrays. Vertices aren't shared between faces, so each has exactly one, and its number and each edge
/// segment's are those in `faces_list` and `edges_list` for that body.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct FlatMesh {
    /// Three numbers for each vertex.
    pub positions: Vec<f32>,
    /// Three numbers for each vertex.
    pub normals: Vec<f32>,
    /// The face each vertex is on.
    pub faces: Vec<u32>,
    /// Three for each triangle.
    pub indices: Vec<u32>,
    /// Six numbers for each edge segment. The seam between two halves of a closed surface isn't drawn.
    pub segments: Vec<f32>,
    /// The edge each segment is on.
    pub segment_ids: Vec<u32>,
}

impl FlatMesh {
    /// Lighter for following a drag if `draft`, and otherwise fine enough to look round at CAD zoom levels.
    pub fn of(body: &Body, draft: bool) -> R<FlatMesh> {
        let size = body.aabb().diagonal();
        let params = if draft { TessParams::draft_for_size(size) } else { TessParams { chord: (size * 5e-4).max(1e-4), angle: 11f64.to_radians() } };
        let mesh = tessellate(body, &params).map_err(|e| e.to_string())?;
        let mut out = FlatMesh::default();
        for p in &mesh.positions {
            out.positions.extend_from_slice(&[p.x as f32, p.y as f32, p.z as f32]);
        }
        for n in &mesh.normals {
            out.normals.extend_from_slice(&[n.x as f32, n.y as f32, n.z as f32]);
        }
        out.faces.resize(mesh.positions.len(), 0);
        for (t, &f) in mesh.triangles.iter().zip(&mesh.face_of_triangle) {
            for &v in t {
                out.faces[v as usize] = f;
                out.indices.push(v);
            }
        }
        let uses = body.edge_uses();
        for (e, line) in mesh.edge_lines.iter().enumerate() {
            if matches!(uses[e].as_slice(), [a, b] if body.faces[a.0].surface == body.faces[b.0].surface) {
                continue;
            }
            for w in line.windows(2) {
                out.segments.extend_from_slice(&[w[0].x as f32, w[0].y as f32, w[0].z as f32, w[1].x as f32, w[1].y as f32, w[1].z as f32]);
                out.segment_ids.push(e as u32);
            }
        }
        Ok(out)
    }

    /// The same mesh moved to where something is, for an app that would rather have it all in one space.
    pub fn placed(&self, at: &Frame) -> FlatMesh {
        let moved = |list: &[f32], point: bool| -> Vec<f32> {
            let mut out = Vec::with_capacity(list.len());
            for v in list.chunks_exact(3) {
                let p = coreshape::v3(v[0] as f64, v[1] as f64, v[2] as f64);
                let q = if point { at.to_world(p) } else { at.dir_to_world(p) };
                out.extend_from_slice(&[q.x as f32, q.y as f32, q.z as f32]);
            }
            out
        };
        FlatMesh { positions: moved(&self.positions, true), normals: moved(&self.normals, false), faces: self.faces.clone(), indices: self.indices.clone(), segments: moved(&self.segments, true), segment_ids: self.segment_ids.clone() }
    }
}

/// One instance of an assembly, ready to draw: its part's mesh, in the part's own coordinates, and where that goes.
#[derive(Clone, Debug)]
pub struct InstanceMesh {
    /// The instances down to it, by id.
    pub path: Vec<u32>,
    pub name: String,
    pub tab: TabId,
    pub part: BodyId,
    pub at: Frame,
    /// `at` as a matrix, by columns, as WebGL takes it.
    pub matrix: [f32; 16],
    /// Shared by every instance of the same part.
    pub mesh: Arc<FlatMesh>,
}

fn matrix_of(at: &Frame) -> [f32; 16] {
    let (x, y, z, o) = (at.x, at.y, at.z, at.origin);
    [x.x as f32, x.y as f32, x.z as f32, 0.0, y.x as f32, y.y as f32, y.z as f32, 0.0, z.x as f32, z.y as f32, z.z as f32, 0.0, o.x as f32, o.y as f32, o.z as f32, 1.0]
}

/// How many meshes to keep. An old one is only ever worked out again.
const MESHES_KEPT: usize = 256;

impl Session {
    fn kept_mesh(&mut self, body: &Arc<Body>, draft: bool) -> R<Arc<FlatMesh>> {
        if let Some(found) = self.meshes.iter().find(|(b, d, _)| Arc::ptr_eq(b, body) && *d == draft) {
            return Ok(found.2.clone());
        }
        let mesh = Arc::new(FlatMesh::of(body, draft)?);
        if self.meshes.len() >= MESHES_KEPT {
            self.meshes.remove(0);
        }
        self.meshes.push((body.clone(), draft, mesh.clone()));
        Ok(mesh)
    }

    /// One part's mesh, in its own coordinates, by its name in `bodies_list`. From this Part Studio unless a tab's given.
    pub fn part_mesh(&mut self, part: &str, tab: Option<TabId>, draft: bool) -> R<Arc<FlatMesh>> {
        let id = BodyId::parse(part).map_err(|e| e.to_string())?;
        let studio = tab.unwrap_or(self.studio);
        let body = self.with_project(|p| -> R<Arc<Body>> {
            let outcome = p.studio_mut(studio).map_err(|e| e.to_string())?.rebuild(None);
            let found = outcome.find_body(id).map(|b| b.body.clone());
            found.ok_or_else(|| {
                let there: Vec<String> = outcome.bodies.iter().map(|b| b.id.text()).collect();
                format!("tab {} has no part {part}: there's {}", studio.0, if there.is_empty() { "none yet".to_string() } else { there.join(", ") })
            })
        })?;
        self.kept_mesh(&body, draft)
    }

    /// The selected assembly, solved, with a mesh and a place for each instance.
    pub fn instance_meshes(&mut self, draft: bool) -> R<(Solved, Vec<InstanceMesh>)> {
        let solved = self.solved()?;
        let mut out = Vec::with_capacity(solved.placed.len());
        for p in &solved.placed {
            let mesh = self.kept_mesh(&p.body, draft)?;
            out.push(InstanceMesh { path: p.path.clone(), name: p.name.clone(), tab: p.studio, part: p.part, at: p.at, matrix: matrix_of(&p.at), mesh });
        }
        Ok((solved, out))
    }
}
