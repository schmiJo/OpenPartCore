//! A STEP file of many parts, some of which can't come in as they are. Each test builds its file as text, after what a
//! 711-solid assembly from Autodesk's exporter did: one solid whose faces met at a single point, and one pin whose notch
//! had an edge on the line of its round face's seam, each of which refused the whole file. Volumes and areas are checked
//! against closed formulas.

use coreshape::import::step::{read_step, read_step_each, read_step_parts};
use coreshape::*;
use std::f64::consts::PI;

/// Builds a STEP file one entity at a time.
struct File {
    lines: Vec<String>,
}

impl File {
    fn new() -> File {
        File { lines: vec!["#1=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));".into()] }
    }
    fn add(&mut self, body: impl AsRef<str>) -> usize {
        let id = self.lines.len() + 1;
        self.lines.push(format!("#{id}={};", body.as_ref()));
        id
    }
    fn point(&mut self, p: Vec3) -> usize {
        self.add(format!("CARTESIAN_POINT('',({:.15e},{:.15e},{:.15e}))", p.x, p.y, p.z))
    }
    fn direction(&mut self, d: Vec3) -> usize {
        self.add(format!("DIRECTION('',({:.15e},{:.15e},{:.15e}))", d.x, d.y, d.z))
    }
    fn placement(&mut self, origin: Vec3, z: Vec3, x: Vec3) -> usize {
        let (o, a, b) = (self.point(origin), self.direction(z), self.direction(x));
        self.add(format!("AXIS2_PLACEMENT_3D('',#{o},#{a},#{b})"))
    }
    fn plane(&mut self, origin: Vec3, z: Vec3, x: Vec3) -> usize {
        let p = self.placement(origin, z, x);
        self.add(format!("PLANE('',#{p})"))
    }
    fn vertex(&mut self, p: Vec3) -> usize {
        let q = self.point(p);
        self.add(format!("VERTEX_POINT('',#{q})"))
    }
    fn line(&mut self, a: Vec3, b: Vec3, va: usize, vb: usize) -> usize {
        let (o, d) = (self.point(a), self.direction((b - a).normalized()));
        let v = self.add(format!("VECTOR('',#{d},1.)"));
        let l = self.add(format!("LINE('',#{o},#{v})"));
        self.add(format!("EDGE_CURVE('',#{va},#{vb},#{l},.T.)"))
    }
    /// An arc of a circle about the z axis, at height `z`, counter-clockwise from `va` to `vb`.
    fn arc(&mut self, z: f64, r: f64, va: usize, vb: usize) -> usize {
        let p = self.placement(v3(0.0, 0.0, z), Vec3::Z, Vec3::X);
        let c = self.add(format!("CIRCLE('',#{p},{r:.15e})"));
        self.add(format!("EDGE_CURVE('',#{va},#{vb},#{c},.T.)"))
    }
    fn face(&mut self, loops: &[Vec<(usize, bool)>], surface: usize, same: bool) -> usize {
        let mut bounds = Vec::new();
        for lp in loops {
            let list: Vec<String> = lp.iter().map(|&(e, along)| format!("#{}", self.add(format!("ORIENTED_EDGE('',*,*,#{e},.{}.)", if along { "T" } else { "F" })))).collect();
            let l = self.add(format!("EDGE_LOOP('',({}))", list.join(",")));
            bounds.push(format!("#{}", self.add(format!("FACE_BOUND('',#{l},.T.)"))));
        }
        self.add(format!("ADVANCED_FACE('',({}),#{surface},.{}.)", bounds.join(","), if same { "T" } else { "F" }))
    }
    fn solid(&mut self, faces: &[usize]) -> usize {
        let list: Vec<String> = faces.iter().map(|f| format!("#{f}")).collect();
        let s = self.add(format!("CLOSED_SHELL('',({}))", list.join(",")));
        self.add(format!("MANIFOLD_SOLID_BREP('',#{s})"))
    }
    /// Triangles over the given corners, by their points, as a faceted solid. A corner shared by two solids' triangles is
    /// one vertex.
    fn faceted(&mut self, corners: &[usize], at: &[Vec3], triangles: &[[usize; 3]]) -> usize {
        let mut faces = Vec::new();
        for t in triangles {
            let list: Vec<String> = t.iter().map(|&k| format!("#{}", corners[k])).collect();
            let lp = self.add(format!("POLY_LOOP('',({}))", list.join(",")));
            let bound = self.add(format!("FACE_OUTER_BOUND('',#{lp},.T.)"));
            let (a, b, c) = (at[t[0]], at[t[1]], at[t[2]]);
            let n = (b - a).cross(c - a).normalized();
            let plane = self.plane(a, n, (b - a).normalized());
            faces.push(format!("#{}", self.add(format!("FACE_SURFACE('',(#{bound}),#{plane},.T.)"))));
        }
        let shell = self.add(format!("CLOSED_SHELL('',({}))", faces.join(",")));
        self.add(format!("FACETED_BREP('',#{shell})"))
    }
    fn text(&self) -> String {
        format!("ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n{}\nENDSEC;\nEND-ISO-10303-21;\n", self.lines.join("\n"))
    }
}

fn close(a: f64, b: f64, tol: f64, what: &str) {
    assert!((a - b).abs() <= tol * b.abs().max(1.0), "{what}: {a} against {b}");
}

/// A tetrahedron's corners from one corner and three edges out of it, and its four triangles, each facing out.
fn tetrahedron(at: Vec3, a: Vec3, b: Vec3, c: Vec3) -> ([Vec3; 4], Vec<[usize; 3]>) {
    let p = [at, at + a, at + b, at + c];
    let middle = (p[0] + p[1] + p[2] + p[3]) * 0.25;
    let mut tris = Vec::new();
    for t in [[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]] {
        let n = (p[t[1]] - p[t[0]]).cross(p[t[2]] - p[t[0]]);
        tris.push(if n.dot(p[t[0]] - middle) > 0.0 { t } else { [t[0], t[2], t[1]] });
    }
    (p, tris)
}

/// A tetrahedron of its own, with points of its own.
fn lone_tetrahedron(f: &mut File, at: Vec3, size: f64) -> usize {
    let (p, tris) = tetrahedron(at, Vec3::X * size, Vec3::Y * size, Vec3::Z * size);
    let ids: Vec<usize> = p.iter().map(|q| f.point(*q)).collect();
    f.faceted(&ids, &p, &tris)
}

/// A tetrahedron with one face missing, which its file still calls closed.
fn open_tetrahedron(f: &mut File, at: Vec3) -> usize {
    let (p, tris) = tetrahedron(at, Vec3::X * 10.0, Vec3::Y * 10.0, Vec3::Z * 10.0);
    let ids: Vec<usize> = p.iter().map(|q| f.point(*q)).collect();
    f.faceted(&ids, &p, &tris[..3])
}

#[test]
fn a_solid_that_cant_come_in_leaves_the_others_and_is_named() {
    let mut f = File::new();
    lone_tetrahedron(&mut f, Vec3::ZERO, 10.0);
    let bad = open_tetrahedron(&mut f, v3(50.0, 0.0, 0.0));
    lone_tetrahedron(&mut f, v3(100.0, 0.0, 0.0), 20.0);
    let text = f.text();
    let read = read_step_each(&text, 1).unwrap();
    assert_eq!(read.parts.len(), 3);
    assert!(read.parts[0].is_ok() && read.parts[1].is_err() && read.parts[2].is_ok());
    assert_eq!(read.entities[1], bad);
    let notes = read.notes();
    assert_eq!(notes.len(), 1, "{notes:?}");
    assert!(notes[0].contains("solid 2 of 3") && notes[0].contains(&format!("#{bad}")) && notes[0].contains("isn't closed"), "{}", notes[0]);
    // Those that came in are as they'd be on their own, and where the file has them.
    let placed = read.placed(1);
    assert_eq!(placed.iter().map(|p| p.0).collect::<Vec<_>>(), vec![0, 2]);
    close(mass_properties(&placed[0].1).volume, 1000.0 / 6.0, 1e-9, "first");
    close(mass_properties(&placed[1].1).volume, 8000.0 / 6.0, 1e-9, "third");
    // Each keeps the name it would have with all three in, so the one left out can be put right without renaming the others.
    let all_good = {
        let mut g = File::new();
        lone_tetrahedron(&mut g, Vec3::ZERO, 10.0);
        lone_tetrahedron(&mut g, v3(50.0, 0.0, 0.0), 10.0);
        lone_tetrahedron(&mut g, v3(100.0, 0.0, 0.0), 20.0);
        read_step(&g.text(), 1).unwrap()
    };
    assert_eq!(placed[1].1.faces[0].origin, all_good[2].faces[0].origin);
    // Those that bring in all or nothing still refuse, and say which one.
    for why in [read_step(&text, 1).unwrap_err().to_string(), read_step_parts(&text, 1).unwrap_err().to_string()] {
        assert!(why.contains("isn't closed"), "{why}");
    }
    assert!(read_step_parts(&text, 1).unwrap_err().to_string().contains(&format!("solid 2 of 3, \"Part 2\" (#{bad})")));
}

#[test]
fn faces_meeting_at_a_point_have_a_corner_each() {
    // Two tetrahedra in one shell, touching at one point only, which the file has as one vertex.
    let mut f = File::new();
    let (p, mut tris) = tetrahedron(Vec3::ZERO, Vec3::X * 10.0, Vec3::Y * 10.0, Vec3::Z * 10.0);
    let (q, more) = tetrahedron(Vec3::ZERO, Vec3::X * -8.0, Vec3::Y * -8.0, Vec3::Z * -8.0);
    let mut at: Vec<Vec3> = p.to_vec();
    at.extend_from_slice(&q[1..]);
    // The second's corner 0 is the first's.
    tris.extend(more.iter().map(|t| t.map(|k| if k == 0 { 0 } else { k + 3 })));
    let ids: Vec<usize> = at.iter().map(|x| f.point(*x)).collect();
    f.faceted(&ids, &at, &tris);
    let read = read_step_each(&f.text(), 1).unwrap();
    let body = read.parts[0].as_ref().unwrap_or_else(|e| panic!("{e}"));
    validate(body).unwrap();
    close(mass_properties(body).volume, (1000.0 + 512.0) / 6.0, 1e-9, "volume");
    assert_eq!(body.vertices.len(), 8);
    assert_eq!(body.contact_vertices(), 1);
    let notes = read.notes();
    assert!(notes.len() == 1 && notes[0].starts_with("repaired solid 1 of 1") && notes[0].contains("(0.0000, 0.0000, 0.0000)"), "{notes:?}");
    let mesh = tessellate(body, &TessParams::for_size(body.aabb().diagonal())).unwrap();
    assert_eq!(mesh.open_edge_count(), 0);
    // Those that bring in all or nothing take the repair too.
    assert_eq!(read_step(&f.text(), 1).unwrap().len(), 1);
}

#[test]
fn copies_of_a_part_come_in_the_same_and_a_near_copy_does_not() {
    // Two parts written out in full, entity for entity the same but for their numbers, and a third with one corner moved
    // a hair. A file of the same part over and over is built once, and the near copy mustn't be taken for it.
    let mut f = File::new();
    lone_tetrahedron(&mut f, Vec3::ZERO, 10.0);
    lone_tetrahedron(&mut f, Vec3::ZERO, 10.0);
    lone_tetrahedron(&mut f, Vec3::ZERO, 10.000001);
    let read = read_step_each(&f.text(), 1).unwrap();
    let v: Vec<f64> = read.parts.iter().map(|p| mass_properties(p.as_ref().unwrap()).volume).collect();
    assert_eq!(v[0], v[1]);
    close(v[2], 1000.000300000003 / 6.0, 1e-12, "the near copy");
    assert!(v[2] > v[0]);
    assert_eq!(format!("{:?}", read.parts[0]), format!("{:?}", read.parts[1]));
}

/// A pin of radius 5 and height 10 about the z axis, with its round face's seam at angle 0, and a notch out of its foot
/// 4 high, from the seam's line round to 60 degrees, or from 300 degrees round to it. Either way, one side of the notch
/// is on the seam's line.
fn notched_pin(after_seam: bool) -> String {
    let (r, h, depth) = (5.0, 10.0, 4.0);
    let a = if after_seam { PI / 3.0 } else { 5.0 * PI / 3.0 };
    let rim = |z: f64, t: f64| v3(r * t.cos(), r * t.sin(), z);
    let mut f = File::new();
    let top = f.vertex(rim(h, 0.0));
    let seam_foot = f.vertex(rim(depth, 0.0));
    let foot = f.vertex(rim(0.0, 0.0));
    let notch_top = f.vertex(rim(depth, a));
    let notch_foot = f.vertex(rim(0.0, a));
    let axis_top = f.vertex(v3(0.0, 0.0, depth));
    let axis_foot = f.vertex(Vec3::ZERO);
    let rim_top = f.arc(h, r, top, top);
    let seam = f.line(rim(depth, 0.0), rim(h, 0.0), seam_foot, top);
    let on_seam_line = f.line(rim(0.0, 0.0), rim(depth, 0.0), foot, seam_foot);
    let notch_side = f.line(rim(0.0, a), rim(depth, a), notch_foot, notch_top);
    let axis = f.line(Vec3::ZERO, v3(0.0, 0.0, depth), axis_foot, axis_top);
    let to_foot = f.line(Vec3::ZERO, rim(0.0, 0.0), axis_foot, foot);
    let to_notch_foot = f.line(Vec3::ZERO, rim(0.0, a), axis_foot, notch_foot);
    let to_seam = f.line(v3(0.0, 0.0, depth), rim(depth, 0.0), axis_top, seam_foot);
    let to_notch_top = f.line(v3(0.0, 0.0, depth), rim(depth, a), axis_top, notch_top);
    let cylinder = {
        let p = f.placement(Vec3::ZERO, Vec3::Z, Vec3::X);
        f.add(format!("CYLINDRICAL_SURFACE('',#{p},{r:.15e})"))
    };
    let up = f.plane(v3(0.0, 0.0, h), Vec3::Z, Vec3::X);
    let down = f.plane(Vec3::ZERO, -Vec3::Z, Vec3::X);
    let ceiling = f.plane(v3(0.0, 0.0, depth), -Vec3::Z, Vec3::X);
    let mut faces = vec![f.face(&[vec![(rim_top, true)]], up, true)];
    if after_seam {
        // The notch runs from 0 to 60 degrees: the round face goes round from 60 to 360 at its foot.
        let foot_arc = f.arc(0.0, r, notch_foot, foot);
        let ceiling_arc = f.arc(depth, r, seam_foot, notch_top);
        faces.push(f.face(&[vec![(foot_arc, true), (on_seam_line, true), (seam, true), (rim_top, false), (seam, false), (ceiling_arc, true), (notch_side, false)]], cylinder, true));
        faces.push(f.face(&[vec![(foot_arc, false), (to_notch_foot, false), (to_foot, true)]], down, true));
        faces.push(f.face(&[vec![(to_seam, false), (to_notch_top, true), (ceiling_arc, false)]], ceiling, true));
        let wall0 = f.plane(Vec3::ZERO, Vec3::Y, Vec3::X);
        faces.push(f.face(&[vec![(axis, true), (to_seam, true), (on_seam_line, false), (to_foot, false)]], wall0, true));
        let n = v3(a.sin(), -a.cos(), 0.0);
        let wall = f.plane(Vec3::ZERO, n, Vec3::Z);
        faces.push(f.face(&[vec![(to_notch_foot, true), (notch_side, true), (to_notch_top, false), (axis, false)]], wall, true));
    } else {
        // The notch runs from 300 to 360 degrees: the round face goes round from 0 to 300 at its foot.
        let foot_arc = f.arc(0.0, r, foot, notch_foot);
        let ceiling_arc = f.arc(depth, r, notch_top, seam_foot);
        faces.push(f.face(&[vec![(foot_arc, true), (notch_side, true), (ceiling_arc, true), (seam, true), (rim_top, false), (seam, false), (on_seam_line, false)]], cylinder, true));
        faces.push(f.face(&[vec![(to_foot, false), (to_notch_foot, true), (foot_arc, false)]], down, true));
        faces.push(f.face(&[vec![(to_notch_top, false), (to_seam, true), (ceiling_arc, false)]], ceiling, true));
        let wall0 = f.plane(Vec3::ZERO, -Vec3::Y, Vec3::X);
        faces.push(f.face(&[vec![(to_foot, true), (on_seam_line, true), (to_seam, false), (axis, false)]], wall0, true));
        let n = v3(-a.sin(), a.cos(), 0.0);
        let wall = f.plane(Vec3::ZERO, n, Vec3::Z);
        faces.push(f.face(&[vec![(axis, true), (to_notch_top, true), (notch_side, false), (to_notch_foot, false)]], wall, true));
    }
    f.solid(&faces);
    f.text()
}

#[test]
fn a_notch_with_a_side_on_the_line_of_the_seam_comes_in_whole() {
    let (r, h, depth): (f64, f64, f64) = (5.0, 10.0, 4.0);
    let volume = PI * r * r * h - PI * r * r * depth / 6.0;
    let area = PI * r * r * (1.0 + 5.0 / 6.0 + 1.0 / 6.0) + 2.0 * PI * r * h - 2.0 * PI * r * depth / 6.0 + 2.0 * r * depth;
    for after_seam in [true, false] {
        let what = if after_seam { "notch after the seam" } else { "notch before the seam" };
        let all = read_step(&notched_pin(after_seam), 1).unwrap_or_else(|e| panic!("{what}: {e}"));
        assert_eq!(all.len(), 1, "{what}");
        let b = &all[0];
        validate(b).unwrap_or_else(|e| panic!("{what}: {e:?}"));
        let m = mass_properties(b);
        close(m.volume, volume, 1e-9, what);
        close(m.area, area, 1e-9, what);
        let mesh = tessellate(b, &TessParams::for_size(b.aabb().diagonal())).unwrap();
        assert_eq!(mesh.open_edge_count(), 0, "{what}");
        // And it works as a solid: a cut through the notch's corner takes what it should.
        let tool = box_solid(&Frame::translation(v3(-1.0, -1.0, -1.0)), 2.0, 2.0, 12.0, 2).unwrap();
        let cut = subtract(b, &tool).unwrap_or_else(|e| panic!("{what}: {e}"));
        // The box's square across the notch's 60 degrees, from its side on the x axis: half of it up to 45 degrees, and a
        // triangle from there to where 60 degrees meets the square's side.
        let in_notch = 0.5 + 0.5 * (1.0 - 1.0 / 3f64.sqrt());
        let taken = 2.0 * 2.0 * h - in_notch * depth;
        close(mass_properties(&cut).volume, volume - taken, 1e-9, what);
    }
}
