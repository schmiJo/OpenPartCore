//! What other systems' STEP files do that this kernel's own don't. Each of these came from reading 75 files from many
//! writers against OpenCASCADE. The files themselves aren't in this repository, so each test builds its situation as
//! text, and each result is checked against a closed formula or against the same part in plain form.
//! `scripts/step_import/check_nist.sh` runs a whole folder of files against OpenCASCADE.

use coreshape::export::step::write_step;
use coreshape::import::step::read_step;
use coreshape::patch::Patch;
use coreshape::topo::Edge;
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
    /// A straight edge between two points.
    fn line(&mut self, a: Vec3, b: Vec3, va: usize, vb: usize) -> usize {
        let (o, d) = (self.point(a), self.direction((b - a).normalized()));
        let v = self.add(format!("VECTOR('',#{d},1.)"));
        let l = self.add(format!("LINE('',#{o},#{v})"));
        self.add(format!("EDGE_CURVE('',#{va},#{vb},#{l},.T.)"))
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
    fn text(&self) -> String {
        format!("ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n{}\nENDSEC;\nEND-ISO-10303-21;\n", self.lines.join("\n"))
    }
}

fn close(a: f64, b: f64, tol: f64, what: &str) {
    assert!((a - b).abs() <= tol * b.abs().max(1.0), "{what}: {a} against {b}");
}

/// Reads one solid and checks it against what's wanted, as a body that works.
fn reads_as(text: &str, volume: f64, area: f64, what: &str) -> Body {
    let mut all = read_step(text, 1).unwrap_or_else(|e| panic!("{what}: {e}"));
    assert_eq!(all.len(), 1, "{what}");
    let b = all.remove(0);
    let m = mass_properties(&b);
    close(m.volume, volume, 1e-6, what);
    close(m.area, area, 1e-6, what);
    validate(&b).unwrap_or_else(|e| panic!("{what}: {e:?}"));
    let mesh = tessellate(&b, &TessParams::for_size(b.aabb().diagonal())).unwrap_or_else(|e| panic!("{what}: {e}"));
    assert_eq!(mesh.open_edge_count(), 0, "{what}: gaps in the mesh");
    b
}

fn props(b: &Body) -> (f64, f64) {
    let m = mass_properties(b);
    (m.volume, m.area)
}

/// The numbers in a line of a STEP file that refer to entities.
fn ids(line: &str) -> Vec<usize> {
    let mut out = Vec::new();
    let mut rest = line;
    while let Some(i) = rest.find('#') {
        rest = &rest[i + 1..];
        let end = rest.find(|c: char| !c.is_ascii_digit()).unwrap_or(rest.len());
        if let Ok(n) = rest[..end].parse() {
            out.push(n);
        }
        rest = &rest[end..];
    }
    out
}

/// The first line of a file holding an entity of this name.
fn line_of<'a>(text: &'a str, name: &str) -> &'a str {
    let wanted = format!("={name}(");
    text.lines().find(|l| l.contains(&wanted)).unwrap_or_else(|| panic!("no {name}"))
}

fn biggest_id(text: &str) -> usize {
    text.lines().filter(|l| l.starts_with('#')).map(|l| ids(l)[0]).max().unwrap_or(0)
}

fn with_lines(text: &str, lines: &str) -> String {
    let at = text.rfind("ENDSEC;").unwrap();
    format!("{}{}\n{}", &text[..at], lines, &text[at..])
}

/// A plate's text, and the ids of its origin placement, its solid, its representation and its context.
fn part() -> (Body, String, usize, usize, usize, usize) {
    let hole = cylinder(&Frame::translation(v3(15.0, 10.0, -1.0)), 4.0, 8.0, 2).unwrap();
    let b = subtract(&box_solid(&Frame::WORLD, 30.0, 20.0, 6.0, 1).unwrap(), &hole).unwrap();
    let text = write_step(&b, "plate");
    let rep_line = line_of(&text, "ADVANCED_BREP_SHAPE_REPRESENTATION");
    let all = ids(rep_line);
    let (rep, origin, solid, ctx) = (all[0], all[1], all[2], all[3]);
    (b, text, origin, solid, rep, ctx)
}

fn center(b: &Body) -> Vec3 {
    b.aabb().center()
}

#[test]
fn entity_names_can_be_their_short_forms() {
    let (b, text, ..) = part();
    let mut short = text.clone();
    for (long, name) in [
        ("ADVANCED_FACE", "ADVFC"),
        ("CLOSED_SHELL", "CLSSHL"),
        ("MANIFOLD_SOLID_BREP", "MNSLBR"),
        ("CARTESIAN_POINT", "CRTPNT"),
        ("DIRECTION", "DRCTN"),
        ("EDGE_CURVE", "EDGCRV"),
        ("EDGE_LOOP", "EDGLP"),
        ("ORIENTED_EDGE", "ORNEDG"),
        ("VERTEX_POINT", "VRTPNT"),
        ("AXIS2_PLACEMENT_3D", "A2PL3D"),
        ("CYLINDRICAL_SURFACE", "CYLSRF"),
    ] {
        short = short.replace(&format!("={long}("), &format!("={name}("));
    }
    assert_ne!(short, text);
    let (v, a) = props(&b);
    reads_as(&short, v, a, "short names");
}

#[test]
fn an_edges_curve_can_be_wrapped_in_another_entity() {
    let (b, text, ..) = part();
    let (v, a) = props(&b);
    for wrapper in ["INTERSECTION_CURVE", "SURFACE_CURVE", "SEAM_CURVE"] {
        let mut next = biggest_id(&text) + 1;
        let mut out = Vec::new();
        for l in text.lines() {
            if l.contains("=CIRCLE(") {
                let id = ids(l)[0];
                let body = &l[l.find('=').unwrap() + 1..];
                out.push(format!("#{id}={wrapper}('',#{next},(#{next},#{next}),.CURVE_3D.);"));
                out.push(format!("#{next}={body}"));
                next += 1;
            } else {
                out.push(l.to_string());
            }
        }
        reads_as(&out.join("\n"), v, a, wrapper);
    }
}

/// A tetrahedron as a faceted solid: corners without vertices, polygons without edges.
fn faceted(f: &mut File, shift: Vec3) -> usize {
    let c = [v3(0.0, 0.0, 0.0), v3(10.0, 0.0, 0.0), v3(0.0, 10.0, 0.0), v3(0.0, 0.0, 10.0)].map(|p| f.point(p + shift));
    let faces = [([0, 2, 1], v3(0.0, 0.0, -1.0)), ([0, 1, 3], v3(0.0, -1.0, 0.0)), ([0, 3, 2], v3(-1.0, 0.0, 0.0)), ([1, 2, 3], v3(1.0, 1.0, 1.0).normalized())];
    let mut ids = Vec::new();
    for (corners, n) in faces {
        let list: Vec<String> = corners.iter().map(|&k: &usize| format!("#{}", c[k])).collect();
        let lp = f.add(format!("POLY_LOOP('',({}))", list.join(",")));
        let bound = f.add(format!("FACE_OUTER_BOUND('',#{lp},.T.)"));
        let on_it = [v3(0.0, 0.0, 0.0), v3(0.0, 0.0, 0.0), v3(0.0, 0.0, 0.0), v3(10.0, 0.0, 0.0)][ids.len()] + shift;
        let plane = f.plane(on_it, n, n.any_perpendicular());
        ids.push(f.add(format!("FACE_SURFACE('',(#{bound}),#{plane},.T.)")));
    }
    let list: Vec<String> = ids.iter().map(|i| format!("#{i}")).collect();
    let shell = f.add(format!("CLOSED_SHELL('',({}))", list.join(",")));
    f.add(format!("FACETED_BREP('',#{shell})"))
}

#[test]
fn a_faceted_solid_is_read_and_never_lost() {
    let mut f = File::new();
    faceted(&mut f, Vec3::ZERO);
    let volume = 1000.0 / 6.0;
    let area = 3.0 * 50.0 + 3f64.sqrt() / 2.0 * 100.0;
    reads_as(&f.text(), volume, area, "tetrahedron");
    // One written beside an advanced solid used to go missing, with nothing to say so.
    let mut f = File::new();
    faceted(&mut f, Vec3::ZERO);
    faceted(&mut f, v3(100.0, 0.0, 0.0));
    let all = read_step(&f.text(), 1).unwrap();
    assert_eq!(all.len(), 2);
    let total: f64 = all.iter().map(|b| mass_properties(b).volume).sum();
    close(total, 2.0 * volume, 1e-9, "both");
}

#[test]
fn a_solid_of_another_kind_is_refused_by_name() {
    let (_, text, ..) = part();
    for kind in ["CSG_SOLID", "EXTRUDED_AREA_SOLID", "REVOLVED_AREA_SOLID", "BOOLEAN_RESULT"] {
        let id = biggest_id(&text) + 1;
        let why = read_step(&with_lines(&text, &format!("#{id}={kind}('',#1);")), 1).unwrap_err().to_string();
        assert!(why.contains(kind), "{kind}: {why}");
    }
}

#[test]
fn a_face_can_be_used_through_an_oriented_face() {
    let (b, text, ..) = part();
    let (v, a) = props(&b);
    let shell = line_of(&text, "CLOSED_SHELL").to_string();
    let all = ids(&shell);
    let first = all[1];
    for (sense, reversed) in [("T", false), ("F", true)] {
        let id = biggest_id(&text) + 1;
        // The shell's own id comes first in its line, and the first face after it.
        let new_shell = shell.replacen(&format!("(#{first}"), &format!("(#{id}"), 1);
        let mut changed = text.replace(&shell, &new_shell);
        // Used the other way round, the face itself has to say the opposite for the part to be the same part.
        if reversed {
            let face = text.lines().find(|l| l.starts_with(&format!("#{first}=ADVANCED_FACE("))).unwrap();
            let refs = ids(face);
            let flip = |l: &str| if l.ends_with(".T.);") { l.replace(".T.);", ".F.);") } else { l.replace(".F.);", ".T.);") };
            changed = changed.replace(face, &flip(face));
            for b in &refs[1..refs.len() - 1] {
                let bound = text.lines().find(|l| l.starts_with(&format!("#{b}="))).unwrap();
                changed = changed.replace(bound, &flip(bound));
            }
        }
        changed = with_lines(&changed, &format!("#{id}=ORIENTED_FACE('',*,#{first},.{sense}.);"));
        let got = read_step(&changed, 1).unwrap_or_else(|e| panic!("reversed {reversed}: {e}"));
        let m = mass_properties(&got[0]);
        close(m.volume, v, 1e-6, "oriented face");
        close(m.area, a, 1e-6, "oriented face");
    }
}

/// An assembly placing the part at (100, 0, 0): a parent representation holding a placement there, and a relationship
/// between the two. `parent_first` and `item_of_parent_first` are the things writers do differently.
fn placed_part(text: &str, origin: usize, rep: usize, ctx: usize, parent_first: bool, item_of_parent_first: bool) -> String {
    let n = biggest_id(text);
    let (a, b, c, place, parent, t, r) = (n + 1, n + 2, n + 3, n + 4, n + 5, n + 6, n + 7);
    let (parent_def, parent_shape, parent_sdr, usage, usage_shape, cdsr) = (n + 8, n + 9, n + 10, n + 11, n + 12, n + 13);
    let child_def = ids(line_of(text, "PRODUCT_DEFINITION_SHAPE"))[1];
    let (first, second) = if parent_first { (parent, rep) } else { (rep, parent) };
    let (one, two) = if item_of_parent_first { (place, origin) } else { (origin, place) };
    with_lines(
        text,
        &format!(
            "#{a}=CARTESIAN_POINT('',(100.,0.,0.));\n#{b}=DIRECTION('',(0.,0.,1.));\n#{c}=DIRECTION('',(1.,0.,0.));\n\
             #{place}=AXIS2_PLACEMENT_3D('',#{a},#{b},#{c});\n#{parent}=SHAPE_REPRESENTATION('',(#{place}),#{ctx});\n\
             #{t}=ITEM_DEFINED_TRANSFORMATION('','',#{one},#{two});\n\
             #{r}=(REPRESENTATION_RELATIONSHIP('','',#{first},#{second})REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#{t})SHAPE_REPRESENTATION_RELATIONSHIP());
             #{parent_def}=PRODUCT_DEFINITION('','',#1,#1);
#{parent_shape}=PRODUCT_DEFINITION_SHAPE('','',#{parent_def});
             #{parent_sdr}=SHAPE_DEFINITION_REPRESENTATION(#{parent_shape},#{parent});
             #{usage}=NEXT_ASSEMBLY_USAGE_OCCURRENCE('','','',#{parent_def},#{child_def},$);
#{usage_shape}=PRODUCT_DEFINITION_SHAPE('','',#{usage});
             #{cdsr}=CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#{r},#{usage_shape});"
        ),
    )
}

#[test]
fn an_assembly_puts_its_part_in_place_however_the_writer_orders_things() {
    let (b, text, origin, _, rep, ctx) = part();
    let (v, a) = props(&b);
    let want = center(&b) + v3(100.0, 0.0, 0.0);
    // The standard's way is the child first. One writer lists the parent first and leaves the items child first, and put
    // the part at -100 until each item was looked up in the representation really holding it.
    for parent_first in [false, true] {
        for item_of_parent_first in [false, true] {
            let got = reads_as(&placed_part(&text, origin, rep, ctx, parent_first, item_of_parent_first), v, a, "placed");
            assert!(center(&got).dist(want) < 1e-9, "parent first {parent_first}, its item first {item_of_parent_first}: {:?}", center(&got));
        }
    }
}

#[test]
fn a_part_used_twice_comes_in_twice_with_names_of_its_own() {
    let (b, text, origin, _, rep, ctx) = part();
    let once = placed_part(&text, origin, rep, ctx, false, false);
    let n = biggest_id(&once);
    // The same parent also places it at (0, 50, 0).
    let parent_line = line_of(&once, "SHAPE_REPRESENTATION").to_string();
    let parent = ids(&parent_line)[0];
    let first_place = ids(&parent_line)[1];
    let (a, c, d, place, t, r) = (n + 1, n + 2, n + 3, n + 4, n + 5, n + 6);
    let twice = with_lines(
        &once.replace(&parent_line, &format!("#{parent}=SHAPE_REPRESENTATION('',(#{first_place},#{place}),#{ctx});")),
        &format!(
            "#{a}=CARTESIAN_POINT('',(0.,50.,0.));\n#{c}=DIRECTION('',(0.,0.,1.));\n#{d}=DIRECTION('',(1.,0.,0.));\n\
             #{place}=AXIS2_PLACEMENT_3D('',#{a},#{c},#{d});\n#{t}=ITEM_DEFINED_TRANSFORMATION('','',#{origin},#{place});\n\
             #{r}=(REPRESENTATION_RELATIONSHIP('','',#{rep},#{parent})REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#{t})SHAPE_REPRESENTATION_RELATIONSHIP());"
        ),
    );
    let all = read_step(&twice, 1).unwrap();
    assert_eq!(all.len(), 2);
    let centers: Vec<Vec3> = all.iter().map(center).collect();
    let wanted = [center(&b) + v3(100.0, 0.0, 0.0), center(&b) + v3(0.0, 50.0, 0.0)];
    for w in wanted {
        assert!(centers.iter().any(|c| c.dist(w) < 1e-9), "{w:?} not in {centers:?}");
    }
    assert_ne!(all[0].faces[0].origin.copies, all[1].faces[0].origin.copies, "two copies with one name");
}

#[test]
fn a_mapped_item_places_a_part_and_one_only_for_showing_it_does_not() {
    let (b, text, origin, _, rep, ctx) = part();
    let n = biggest_id(&text);
    let (a, c, d, place, map, item, parent) = (n + 1, n + 2, n + 3, n + 4, n + 5, n + 6, n + 7);
    let lines = format!(
        "#{a}=CARTESIAN_POINT('',(100.,0.,0.));\n#{c}=DIRECTION('',(0.,0.,1.));\n#{d}=DIRECTION('',(1.,0.,0.));\n\
         #{place}=AXIS2_PLACEMENT_3D('',#{a},#{c},#{d});\n#{map}=REPRESENTATION_MAP(#{origin},#{rep});\n\
         #{item}=MAPPED_ITEM('',#{map},#{place});\n"
    );
    let placed = with_lines(&text, &format!("{lines}#{parent}=SHAPE_REPRESENTATION('',(#{item},#{place}),#{ctx});"));
    let got = read_step(&placed, 1).unwrap();
    assert_eq!(got.len(), 1);
    assert!(center(&got[0]).dist(center(&b) + v3(100.0, 0.0, 0.0)) < 1e-9, "{:?}", center(&got[0]));
    // A view of a model is mapped from a camera, in a plain representation. That places nothing, and isn't an error.
    let (camera, view_map, view_item, view) = (n + 8, n + 9, n + 10, n + 11);
    let viewed = with_lines(
        &text,
        &format!(
            "{lines}#{camera}=CAMERA_MODEL_D3('',#{place},#{ctx});\n#{view_map}=REPRESENTATION_MAP(#{camera},#{rep});\n\
             #{view_item}=MAPPED_ITEM('',#{view_map},#{place});\n#{view}=REPRESENTATION('',(#{camera},#{view_item}),#{ctx});"
        ),
    );
    let got = read_step(&viewed, 1).unwrap();
    assert_eq!(got.len(), 1);
    assert!(center(&got[0]).dist(center(&b)) < 1e-9, "{:?}", center(&got[0]));
    // A draughting model maps the same things to give them a colour. Nothing moves, and nothing's there twice.
    let shown = with_lines(&text, &format!("{lines}#{parent}=DRAUGHTING_MODEL('',(#{item},#{place}),#{ctx});"));
    let got = read_step(&shown, 1).unwrap();
    assert_eq!(got.len(), 1);
    assert!(center(&got[0]).dist(center(&b)) < 1e-9, "{:?}", center(&got[0]));
}

#[test]
fn a_placement_beside_a_solid_in_a_lone_part_is_where_the_part_goes() {
    let (b, text, origin, solid, rep, ctx) = part();
    let n = biggest_id(&text);
    let (a, c, d, place) = (n + 1, n + 2, n + 3, n + 4);
    // ACIS writes a body's own transform like this. A quarter turn about z, then a move.
    let moved = with_lines(
        &text.replace(&format!("#{rep}=ADVANCED_BREP_SHAPE_REPRESENTATION('',(#{origin},#{solid}),#{ctx});"), &format!("#{rep}=ADVANCED_BREP_SHAPE_REPRESENTATION('',(#{place},#{solid}),#{ctx});")),
        &format!("#{a}=CARTESIAN_POINT('',(7.,8.,9.));\n#{c}=DIRECTION('',(0.,0.,1.));\n#{d}=DIRECTION('',(0.,1.,0.));\n#{place}=AXIS2_PLACEMENT_3D('',#{a},#{c},#{d});"),
    );
    let got = read_step(&moved, 1).unwrap();
    let m = center(&b);
    let want = v3(7.0 - m.y, 8.0 + m.x, 9.0 + m.z);
    assert!(center(&got[0]).dist(want) < 1e-9, "{:?} against {want:?}", center(&got[0]));
}

#[test]
fn a_shape_that_only_describes_a_part_is_not_another_part() {
    // Validation properties list the same solid again, and that's no copy of it.
    let (b, text, _, solid, _, ctx) = part();
    let n = biggest_id(&text);
    let (def, sr, sd) = (n + 1, n + 2, n + 3);
    let again = with_lines(
        &text,
        &format!(
            "#{def}=PROPERTY_DEFINITION('','Shape for Validation Properties',#{ctx});\n#{sr}=SHAPE_REPRESENTATION('',(#{solid}),#{ctx});\n\
             #{sd}=SHAPE_DEFINITION_REPRESENTATION(#{def},#{sr});"
        ),
    );
    let got = read_step(&again, 1).unwrap();
    assert_eq!(got.len(), 1);
    assert!(center(&got[0]).dist(center(&b)) < 1e-9);
    // A feature tree's history of how a hole was made has solids of its own, which aren't the part either.
    let mut f = File::new();
    let main = faceted(&mut f, Vec3::ZERO);
    let extra = faceted(&mut f, v3(100.0, 0.0, 0.0));
    let context = f.add("GEOMETRIC_REPRESENTATION_CONTEXT(3)");
    let definition = f.add("PRODUCT_DEFINITION('','',#1,#1)");
    let shape = f.add(format!("PRODUCT_DEFINITION_SHAPE('','',#{definition})"));
    let rep = f.add(format!("SHAPE_REPRESENTATION('',(#{main}),#{context})"));
    f.add(format!("SHAPE_DEFINITION_REPRESENTATION(#{shape},#{rep})"));
    let aspect = f.add(format!("SHAPE_ASPECT('hole','',#{shape},.T.)"));
    let its_shape = f.add(format!("PRODUCT_DEFINITION_SHAPE('','',#{aspect})"));
    let its_rep = f.add(format!("SHAPE_REPRESENTATION('explicit geometry branch',(#{extra}),#{context})"));
    f.add(format!("SHAPE_DEFINITION_REPRESENTATION(#{its_shape},#{its_rep})"));
    let got = read_step(&f.text(), 1).unwrap();
    assert_eq!(got.len(), 1);
    assert!(got[0].aabb().center().x < 50.0);
}

#[test]
fn an_edge_can_be_any_of_the_implied_knot_forms_of_a_spline() {
    let (b, text, ..) = part();
    let (v, a) = props(&b);
    for (name, tail) in [("QUASI_UNIFORM_CURVE", ".POLYLINE_FORM.,.F.,.U."), ("BEZIER_CURVE", ".POLYLINE_FORM.,.F.,.U."), ("UNIFORM_CURVE", ".POLYLINE_FORM.,.F.,.U.")] {
        // Each line's two ends as control points, degree one.
        let mut changed = text.clone();
        let edges: Vec<&str> = text.lines().filter(|l| l.contains("=EDGE_CURVE(")).collect();
        for e in edges {
            let all = ids(e);
            let (v0, v1, curve) = (all[1], all[2], all[3]);
            let is_line = text.lines().any(|l| l.starts_with(&format!("#{curve}=LINE(")));
            if !is_line {
                continue;
            }
            let at = |v: usize| ids(text.lines().find(|l| l.starts_with(&format!("#{v}=VERTEX_POINT("))).unwrap())[1];
            let old = text.lines().find(|l| l.starts_with(&format!("#{curve}=LINE("))).unwrap();
            changed = changed.replace(old, &format!("#{curve}={name}('',1,(#{},#{}),{tail});", at(v0), at(v1)));
        }
        assert_ne!(changed, text);
        reads_as(&changed, v, a, name);
    }
}

/// A half disc and a triangle closing up a half cone whose side is a surface given as `side`.
fn half_cone(f: &mut File, r: f64, h: f64, side: impl FnOnce(&mut File) -> usize) -> usize {
    let (a, b, tip) = (v3(r, 0.0, 0.0), v3(-r, 0.0, 0.0), v3(0.0, 0.0, h));
    let (va, vb, vt) = (f.vertex(a), f.vertex(b), f.vertex(tip));
    let place = f.placement(Vec3::ZERO, Vec3::Z, Vec3::X);
    let circle = f.add(format!("CIRCLE('',#{place},{r:.6})"));
    let arc = f.add(format!("EDGE_CURVE('',#{va},#{vb},#{circle},.T.)"));
    let across = f.line(a, b, va, vb);
    let up_a = f.line(a, tip, va, vt);
    let up_b = f.line(b, tip, vb, vt);
    let surface = side(f);
    // Its normal is its parameters' way round: from the rim up to the tip, then along the rim. That's inward.
    let cone = f.face(&[vec![(arc, true), (up_b, true), (up_a, false)]], surface, false);
    let down = f.plane(Vec3::ZERO, -Vec3::Z, Vec3::X);
    let base = f.face(&[vec![(across, true), (arc, false)]], down, true);
    let back = f.plane(Vec3::ZERO, -Vec3::Y, Vec3::X);
    let flat = f.face(&[vec![(across, false), (up_a, true), (up_b, false)]], back, true);
    f.solid(&[cone, base, flat])
}

#[test]
fn a_spline_face_ending_in_a_tip_keeps_all_its_area() {
    // One row of control points is all one point. No edge runs along that side of its parameters, and the integral
    // used to miss what lay between that side and its reference: exactly a quarter of the cone.
    let (r, h) = (10.0, 30.0);
    let mut f = File::new();
    half_cone(&mut f, r, h, |f| {
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let rim = [v3(r, 0.0, 0.0), v3(r, r, 0.0), v3(0.0, r, 0.0), v3(-r, r, 0.0), v3(-r, 0.0, 0.0)];
        let row: Vec<String> = rim.iter().map(|p| format!("#{}", f.point(*p))).collect();
        let tip = f.point(v3(0.0, 0.0, h));
        let top: Vec<String> = rim.iter().map(|_| format!("#{tip}")).collect();
        let weights = format!("1.,{w:.15},1.,{w:.15},1.");
        f.add(format!(
            "(BOUNDED_SURFACE()B_SPLINE_SURFACE(1,2,(({}),({})),.UNSPECIFIED.,.F.,.F.,.F.)B_SPLINE_SURFACE_WITH_KNOTS((2,2),(3,2,3),(0.,1.),(0.,1.,2.),.UNSPECIFIED.)GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_SURFACE((({weights}),({weights})))REPRESENTATION_ITEM('')SURFACE())",
            row.join(","),
            top.join(",")
        ))
    });
    let slant = (r * r + h * h).sqrt();
    let volume = PI * r * r * h / 6.0;
    let area = PI * r * slant / 2.0 + PI * r * r / 2.0 + r * h;
    reads_as(&f.text(), volume, area, "half cone");
}

#[test]
fn a_surface_of_implied_knots_can_end_in_a_tip_too() {
    // A flat triangle, as a bilinear patch with one side all one point, in each of the forms that imply their knots.
    for (name, close_form) in [("BEZIER_SURFACE", ".F.,.F.,.F."), ("QUASI_UNIFORM_SURFACE", ".F.,.F.,.F."), ("UNIFORM_SURFACE", ".F.,.F.,.F.")] {
        let (s, h) = (10.0, 10.0);
        let mut f = File::new();
        let (a, b, c, tip) = (v3(0.0, 0.0, 0.0), v3(s, 0.0, 0.0), v3(0.0, s, 0.0), v3(0.0, 0.0, h));
        let (va, vb, vc, vt) = (f.vertex(a), f.vertex(b), f.vertex(c), f.vertex(tip));
        let (ab, bc, ca) = (f.line(a, b, va, vb), f.line(b, c, vb, vc), f.line(c, a, vc, va));
        let (at, bt, ct) = (f.line(a, tip, va, vt), f.line(b, tip, vb, vt), f.line(c, tip, vc, vt));
        let bottom = f.plane(a, -Vec3::Z, Vec3::X);
        let on_y = f.plane(a, -Vec3::Y, Vec3::X);
        let on_x = f.plane(a, -Vec3::X, Vec3::Y);
        let (p, q, t) = (f.point(b), f.point(c), f.point(tip));
        let patch = f.add(format!("{name}('',1,1,((#{p},#{q}),(#{t},#{t})),.UNSPECIFIED.,{close_form})"));
        let faces = [
            f.face(&[vec![(ab, false), (ca, false), (bc, false)]], bottom, true),
            f.face(&[vec![(ab, true), (bt, true), (at, false)]], on_y, true),
            f.face(&[vec![(ca, true), (at, true), (ct, false)]], on_x, true),
            // Its parameters' way round makes its normal point in.
            f.face(&[vec![(bc, true), (ct, true), (bt, false)]], patch, false),
        ];
        f.solid(&faces);
        let volume = s * s * h / 6.0;
        let area = 3.0 * s * s / 2.0 + 3f64.sqrt() / 2.0 * s * s;
        reads_as(&f.text(), volume, area, name);
    }
}

#[test]
fn a_curve_in_an_upright_plane_extruded_isnt_read_as_a_plane() {
    // Its direction in x and y alone never changes, as it only curves in x and z. The surface is still curved.
    let points: Vec<Vec3> = [(0.0, 0.0), (10.0, 6.0), (20.0, 2.0), (30.0, 8.0)].iter().map(|p| v3(p.0, p.1, 0.0)).collect();
    let wire = PathBuilder::new(v2(0.0, 0.0)).spline_to(Spline::through(&points).unwrap()).line_to(v2(30.0, -10.0)).line_to(v2(0.0, -10.0)).close();
    // Sketch x along world x, sketch y along world z, extruded along world y.
    let frame = Frame::new(Vec3::ZERO, -Vec3::Y, Vec3::X);
    let b = extrude(&Profile::new(wire), &frame, 15.0, 1).unwrap();
    let (v, a) = props(&b);
    let got = reads_as(&write_step(&b, "upright"), v, a, "upright curve");
    assert!(got.faces.iter().any(|f| matches!(f.surface, Surface::Extruded { .. })));
}

#[test]
fn a_straight_section_extruded_is_a_plane_whichever_way_it_goes() {
    // Sloping in every direction, so that nothing about it lines up with the axes.
    let mut f = File::new();
    let d = v3(0.3, 0.2, 1.0).normalized();
    let (a, b, c) = (v3(0.0, 0.0, 0.0), v3(10.0, 1.0, -2.0), v3(3.0, 9.0, 1.0));
    let h = 12.0;
    let up = |p: Vec3| p + d * h;
    let (va, vb, vc, ua, ub, uc) = (f.vertex(a), f.vertex(b), f.vertex(c), f.vertex(up(a)), f.vertex(up(b)), f.vertex(up(c)));
    let (ab, bc, ca) = (f.line(a, b, va, vb), f.line(b, c, vb, vc), f.line(c, a, vc, va));
    let (de, ef, fd) = (f.line(up(a), up(b), ua, ub), f.line(up(b), up(c), ub, uc), f.line(up(c), up(a), uc, ua));
    let (l0, l1, l2) = (f.line(a, up(a), va, ua), f.line(b, up(b), vb, ub), f.line(c, up(c), vc, uc));
    // Each side as a line swept along, given as the kind of curve that hides it.
    let swept = |f: &mut File, p: Vec3, q: Vec3| {
        let (x, y) = (f.point(p), f.point(q));
        let curve = f.add(format!("QUASI_UNIFORM_CURVE('',1,(#{x},#{y}),.POLYLINE_FORM.,.F.,.U.)"));
        let dir = f.direction(d);
        let v = f.add(format!("VECTOR('',#{dir},1.)"));
        f.add(format!("SURFACE_OF_LINEAR_EXTRUSION('',#{curve},#{v})"))
    };
    let (s0, s1, s2) = (swept(&mut f, a, b), swept(&mut f, b, c), swept(&mut f, c, a));
    let normal = (b - a).cross(c - a).normalized();
    let (under, over) = (f.plane(a, -normal, (b - a).normalized()), f.plane(up(a), normal, (b - a).normalized()));
    // The prism's right-handed way round depends on its corners, so the shell's sense is left to its faces' area.
    let faces = [
        f.face(&[vec![(ab, true), (l1, true), (de, false), (l0, false)]], s0, true),
        f.face(&[vec![(bc, true), (l2, true), (ef, false), (l1, false)]], s1, true),
        f.face(&[vec![(ca, true), (l0, true), (fd, false), (l2, false)]], s2, true),
        f.face(&[vec![(ca, false), (bc, false), (ab, false)]], under, true),
        f.face(&[vec![(de, true), (ef, true), (fd, true)]], over, true),
    ];
    f.solid(&faces);
    let height = h * d.dot(normal).abs();
    let base = (b - a).cross(c - a).norm() / 2.0;
    let sides = (a.dist(b) * (b - a).cross(d).norm() / a.dist(b) + b.dist(c) * (c - b).cross(d).norm() / b.dist(c) + c.dist(a) * (a - c).cross(d).norm() / c.dist(a)) * h;
    let got = reads_as(&f.text(), base * height, 2.0 * base + sides, "sloping prism");
    assert!(got.faces.iter().all(|x| matches!(x.surface, Surface::Plane(_))));
}

#[test]
fn a_chain_of_straight_stretches_going_different_ways_is_no_straight_line() {
    // Every span of this is straight, and they meet at far too small an angle to count as corners, but it still bends.
    let points: Vec<Vec3> = (0..7).map(|k| v3(k as f64 * 10.0, if k % 2 == 1 { 1e-7 } else { 0.0 }, 0.0)).collect();
    let chain = Spline::new(1, vec![0.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 6.0], points, vec![1.0; 7]).unwrap();
    assert!(chain.face_pieces().unwrap().len() > 1);
    // And a straight line through many control points is one piece.
    let points: Vec<Vec3> = (0..7).map(|k| v3(k as f64 * 10.0, k as f64 * 5.0, 0.0)).collect();
    let line = Spline::new(1, vec![0.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 6.0], points, vec![1.0; 7]).unwrap();
    assert_eq!(line.face_pieces().unwrap().len(), 1);
}

#[test]
fn an_edge_starting_a_hair_from_a_knot_leaves_a_file_that_can_be_read_again() {
    // A file's edge on a spline with a knot where it starts, or as near as the file's digits say, and lying a little off
    // the surfaces, so that it gets traced onto them. The stretch between the two is nothing, and samples along it used to
    // pile up into knots that no text can tell apart, so this kernel couldn't read its own file back. That takes a hair
    // this small and luck in the last digits to show, so what's checked is that the samples don't pile up.
    let (r, h) = (10.0, 20.0);
    let hair = 1e-12;
    let mut f = File::new();
    let (a, b) = (v3(r, 0.0, 0.0), v3(-r, 0.0, 0.0));
    // Just before the knot at (r, 0, h), where the curve is heading in +y.
    let (ta, tb) = (v3(r, -hair, h), v3(-r, hair, h));
    let (va, vb, ua, ub) = (f.vertex(a), f.vertex(b), f.vertex(ta), f.vertex(tb));
    let place = f.placement(Vec3::ZERO, Vec3::Z, Vec3::X);
    let lower = f.add(format!("CIRCLE('',#{place},{r:.3})"));
    let arc_low = f.add(format!("EDGE_CURVE('',#{va},#{vb},#{lower},.T.)"));
    // A whole circle as a spline a hair off the cylinder, with a knot at each quarter, of which the edge uses a half.
    let w = std::f64::consts::FRAC_1_SQRT_2;
    let l = r * (1.0 + 1e-6);
    let corners = [v3(0.0, -l, h), v3(l, -l, h), v3(l, 0.0, h), v3(l, l, h), v3(0.0, l, h), v3(-l, l, h), v3(-l, 0.0, h), v3(-l, -l, h), v3(0.0, -l, h)];
    let ids: Vec<String> = corners.iter().map(|p| format!("#{}", f.point(*p))).collect();
    let spline = f.add(format!(
        "(BOUNDED_CURVE()B_SPLINE_CURVE(2,({}),.UNSPECIFIED.,.F.,.F.)B_SPLINE_CURVE_WITH_KNOTS((3,2,2,2,3),(0.,1.,2.,3.,4.),.UNSPECIFIED.)CURVE()GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_CURVE((1.,{w:.15},1.,{w:.15},1.,{w:.15},1.,{w:.15},1.))REPRESENTATION_ITEM(''))",
        ids.join(",")
    ));
    let arc_high = f.add(format!("EDGE_CURVE('',#{ua},#{ub},#{spline},.T.)"));
    let (ra, rb) = (f.line(a, ta, va, ua), f.line(b, tb, vb, ub));
    let (across_low, across_high) = (f.line(a, b, va, vb), f.line(ta, tb, ua, ub));
    let round = f.add(format!("CYLINDRICAL_SURFACE('',#{place},{r:.3})"));
    let down = f.plane(Vec3::ZERO, -Vec3::Z, Vec3::X);
    let up = f.plane(v3(0.0, 0.0, h), Vec3::Z, Vec3::X);
    let back = f.plane(Vec3::ZERO, -Vec3::Y, Vec3::X);
    let faces = [
        f.face(&[vec![(arc_low, true), (rb, true), (arc_high, false), (ra, false)]], round, true),
        f.face(&[vec![(across_low, true), (arc_low, false)]], down, true),
        f.face(&[vec![(arc_high, true), (across_high, false)]], up, true),
        f.face(&[vec![(across_low, false), (ra, true), (across_high, true), (rb, false)]], back, true),
    ];
    f.solid(&faces);
    let first = read_step(&f.text(), 1).unwrap().remove(0);
    validate(&first).unwrap();
    let traced: Vec<_> = first.edges.iter().filter_map(|e| if let Curve::Sampled(t) = &e.curve { Some(t) } else { None }).collect();
    assert!(!traced.is_empty(), "the edge wasn't traced, so this tests nothing");
    for t in traced {
        let closest = t.points.windows(2).map(|w| w[0].dist(w[1])).fold(f64::INFINITY, f64::min);
        assert!(closest > 1e-6, "samples only {closest} apart");
    }
    let again = read_step(&write_step(&first, "again"), 1).unwrap_or_else(|e| panic!("its own file: {e}")).remove(0);
    close(mass_properties(&again).volume, mass_properties(&first).volume, 1e-6, "volume");
    close(mass_properties(&first).volume, PI * r * r / 2.0 * h, 1e-4, "formula");
}

#[test]
fn a_cut_a_hair_from_a_knot_makes_no_cell_of_no_width() {
    let knots_u = vec![0.0, 0.0, 1.0, 1.0];
    let knots_v = vec![0.0, 0.0, 1.0, 2.0, 2.0];
    let points: Vec<Vec3> = [0.0, 1.0].iter().flat_map(|&u| [0.0, 1.0, 2.0].map(|v| v3(u, v, (v as f64 * 0.7).sin()))).collect();
    let patch = Patch::new((1, 1), (knots_u, knots_v), (2, 3), points, vec![1.0; 6]).unwrap();
    // As a half turn's arithmetic can land: just past the knot in the middle.
    let part = patch.restricted((0.0, 1.0), (0.0, 1.0 + 2.5e-16)).unwrap();
    assert_eq!(part.domain().1 .1, 1.0);
    let part = patch.restricted((0.0, 1.0), (1.0 + 2.5e-16, 2.0)).unwrap();
    assert_eq!(part.domain().1 .0, 1.0);
}

fn turned_surface(text: &str, surface: usize, replacement: &str, flip_sense: bool) -> String {
    let old = text.lines().filter(|l| l.starts_with(&format!("#{surface}="))).next().unwrap();
    let mut out = text.replace(old, &format!("#{surface}={replacement};"));
    if flip_sense {
        let wanted = format!(",#{surface},");
        for l in text.lines().filter(|l| l.contains("=ADVANCED_FACE(") && l.contains(&wanted)) {
            let changed = if l.ends_with(".T.);") { l.replace(".T.);", ".F.);") } else { l.replace(".F.);", ".T.);") };
            out = out.replace(l, &changed);
        }
    }
    out
}

/// Replaces every surface of a kind in a part with what `make` gives for it, adding what it needs.
fn replacing(text: &str, kind: &str, flip: bool, make: impl Fn(usize) -> (String, String)) -> String {
    let surfaces: Vec<usize> = text.lines().filter(|l| l.contains(&format!("={kind}("))).map(|l| ids(l)[0]).collect();
    assert!(!surfaces.is_empty());
    let mut out = text.to_string();
    for (k, s) in surfaces.iter().enumerate() {
        let (body, extra) = make(biggest_id(text) + 1 + k * 40);
        out = turned_surface(&out, *s, &body, flip);
        out = with_lines(&out, &extra);
    }
    out
}

/// The entities for a straight line from a to b, trimmed, and an axis along z, and the surface sweeping one around the other.
fn revolved_line(n: usize, a: Vec3, b: Vec3) -> (String, String) {
    let d = (b - a).normalized();
    let length = a.dist(b);
    (
        format!("SURFACE_OF_REVOLUTION('',#{},#{})", n + 4, n + 7),
        format!(
            "#{}=CARTESIAN_POINT('',({:.12},{:.12},{:.12}));\n#{}=DIRECTION('',({:.15},{:.15},{:.15}));\n#{}=VECTOR('',#{},1.);\n#{}=LINE('',#{},#{});\n\
             #{}=TRIMMED_CURVE('',#{},(0.),({:.12}),.T.,.UNSPECIFIED.);\n#{}=CARTESIAN_POINT('',(0.,0.,0.));\n#{}=DIRECTION('',(0.,0.,1.));\n#{}=AXIS1_PLACEMENT('',#{},#{});",
            n, a.x, a.y, a.z, n + 1, d.x, d.y, d.z, n + 2, n + 1, n + 3, n, n + 2, n + 4, n + 3, length, n + 5, n + 6, n + 7, n + 5, n + 6
        ),
    )
}

#[test]
fn a_straight_line_revolved_is_a_cylinder_or_a_cone() {
    // Up its side, the normal to its right is outward, which is what the part already has. Down it, it's inward, and a
    // file wanting the same solid says so with its face's sense, against the surface's own.
    let (r, h) = (5.0, 12.0);
    let c = cylinder(&Frame::WORLD, r, h, 1).unwrap();
    let (v, a) = props(&c);
    let text = write_step(&c, "cylinder");
    let up = replacing(&text, "CYLINDRICAL_SURFACE", false, |n| revolved_line(n, v3(r, 0.0, 0.0), v3(r, 0.0, h)));
    let b = reads_as(&up, v, a, "cylinder, up");
    assert!(b.faces.iter().any(|f| matches!(f.surface, Surface::Cylinder { .. })));
    let down = replacing(&text, "CYLINDRICAL_SURFACE", true, |n| revolved_line(n, v3(r, 0.0, h), v3(r, 0.0, 0.0)));
    reads_as(&down, v, a, "cylinder, down");

    let (r0, r1) = (8.0, 3.0);
    let c = cone_frustum(&Frame::WORLD, r0, r1, h, 1).unwrap();
    let (v, a) = props(&c);
    let text = write_step(&c, "frustum");
    let up = replacing(&text, "CONICAL_SURFACE", false, |n| revolved_line(n, v3(r0, 0.0, 0.0), v3(r1, 0.0, h)));
    let b = reads_as(&up, v, a, "cone, up");
    assert!(b.faces.iter().any(|f| matches!(f.surface, Surface::Cone { .. })));
    let down = replacing(&text, "CONICAL_SURFACE", true, |n| revolved_line(n, v3(r1, 0.0, h), v3(r0, 0.0, 0.0)));
    reads_as(&down, v, a, "cone, down");
}

#[test]
fn a_circle_revolved_is_a_torus_with_the_sense_of_its_direction() {
    let (major, minor) = (12.0, 3.0);
    let t = torus(&Frame::WORLD, major, minor, 1).unwrap();
    let (v, a) = props(&t);
    close(v, 2.0 * PI * PI * major * minor * minor, 1e-9, "formula");
    let text = write_step(&t, "torus");
    // Normal to the circle along -y: counter-clockwise seen with distance from the axis to the right and height upward.
    for (normal, flip) in [(-1.0, false), (1.0, true)] {
        let swap = replacing(&text, "TOROIDAL_SURFACE", flip, |n| {
            (
                format!("SURFACE_OF_REVOLUTION('',#{},#{})", n + 4, n + 8),
                format!(
                    "#{}=CARTESIAN_POINT('',({major:.6},0.,0.));\n#{}=DIRECTION('',(0.,{normal:.1},0.));\n#{}=DIRECTION('',(1.,0.,0.));\n#{}=AXIS2_PLACEMENT_3D('',#{},#{},#{});\n#{}=CIRCLE('',#{},{minor:.6});\n\
                     #{}=CARTESIAN_POINT('',(0.,0.,0.));\n#{}=DIRECTION('',(0.,0.,1.));\n#{}=AXIS1_PLACEMENT('',#{},#{});\n#{}=AXIS1_PLACEMENT('',#{},#{});",
                    n, n + 1, n + 2, n + 3, n, n + 1, n + 2, n + 4, n + 3, n + 5, n + 6, n + 7, n + 5, n + 6, n + 8, n + 5, n + 6
                ),
            )
        });
        let got = read_step(&swap, 1).unwrap_or_else(|e| panic!("normal {normal}: {e}"));
        assert!(got[0].faces.iter().all(|f| matches!(f.surface, Surface::Torus { .. })));
        close(mass_properties(&got[0]).volume, v, 1e-6, "torus");
        close(mass_properties(&got[0]).area, a, 1e-6, "torus");
    }
}

/// An elliptical rod with both ends closed by one edge each, whose vertex is at this angle around the ellipse.
fn elliptical_rod(start: f64) -> String {
    let (a, b, h) = (20.0, 9.0, 30.0);
    let mut f = File::new();
    let at = |z: f64| v3(a * start.cos(), b * start.sin(), z);
    let (low, high) = (f.vertex(at(0.0)), f.vertex(at(h)));
    let (p, q) = (f.placement(Vec3::ZERO, Vec3::Z, Vec3::X), f.placement(v3(0.0, 0.0, h), Vec3::Z, Vec3::X));
    let (el, eh) = (f.add(format!("ELLIPSE('',#{p},{a:.3},{b:.3})")), f.add(format!("ELLIPSE('',#{q},{a:.3},{b:.3})")));
    let bottom = f.add(format!("EDGE_CURVE('',#{low},#{low},#{el},.T.)"));
    let top = f.add(format!("EDGE_CURVE('',#{high},#{high},#{eh},.T.)"));
    let d = f.direction(Vec3::Z);
    let along = f.add(format!("VECTOR('',#{d},1.)"));
    let side = f.add(format!("SURFACE_OF_LINEAR_EXTRUSION('',#{el},#{along})"));
    let wall = f.face(&[vec![(bottom, true)], vec![(top, false)]], side, true);
    let under = f.plane(Vec3::ZERO, -Vec3::Z, Vec3::X);
    let over = f.plane(v3(0.0, 0.0, h), Vec3::Z, Vec3::X);
    let lid = f.face(&[vec![(top, true)]], over, true);
    let base = f.face(&[vec![(bottom, false)]], under, true);
    f.solid(&[wall, lid, base]);
    f.text()
}

#[test]
fn a_closed_curves_extrusion_can_have_no_seam_and_a_corner_anywhere() {
    let (a, b, h) = (20.0, 9.0, 30.0);
    let volume = PI * a * b * h;
    // The ellipse's perimeter, by an integral good to many digits.
    let n = 200_000;
    let perimeter: f64 = (0..n).map(|k| ((k as f64 + 0.5) / n as f64 * 2.0 * PI).sin_cos()).map(|(s, c)| (a * s).hypot(b * c) * 2.0 * PI / n as f64).sum();
    let area = perimeter * h + 2.0 * PI * a * b;
    for start in [0.0, 1.0, 2.5] {
        reads_as(&elliptical_rod(start), volume, area, &format!("corner at {start}"));
    }
}

#[test]
fn a_sphere_with_corners_on_its_own_poles_can_still_be_cut() {
    // A ball with a thin wedge out of it, the wedge's two corners being the poles the file gives its sphere.
    let r = 5.0;
    let gap = 0.05f64;
    let (s, n) = (v3(0.0, 0.0, -r), v3(0.0, 0.0, r));
    let mut f = File::new();
    let (vs, vn) = (f.vertex(s), f.vertex(n));
    let sphere_place = f.placement(Vec3::ZERO, Vec3::Z, Vec3::X);
    let sphere = f.add(format!("SPHERICAL_SURFACE('',#{sphere_place},{r:.6})"));
    let meridian = |f: &mut File, phase: f64| {
        let out = v3(phase.cos(), phase.sin(), 0.0);
        let place = f.placement(Vec3::ZERO, out.cross(Vec3::Z), out);
        let c = f.add(format!("CIRCLE('',#{place},{r:.6})"));
        f.add(format!("EDGE_CURVE('',#{vs},#{vn},#{c},.T.)"))
    };
    let (m0, m1) = (meridian(&mut f, 0.0), meridian(&mut f, gap));
    let axis = f.line(n, s, vn, vs);
    let ball = f.face(&[vec![(m1, true), (m0, false)]], sphere, true);
    let first = f.plane(Vec3::ZERO, -Vec3::Y, Vec3::X);
    let second = f.plane(Vec3::ZERO, v3(-gap.sin(), gap.cos(), 0.0), v3(gap.cos(), gap.sin(), 0.0));
    let wall0 = f.face(&[vec![(m0, true), (axis, true)]], first, false);
    let wall1 = f.face(&[vec![(m1, false), (axis, false)]], second, false);
    f.solid(&[ball, wall0, wall1]);
    let kept = 1.0 - gap / (2.0 * PI);
    let volume = 4.0 / 3.0 * PI * r * r * r * kept;
    let area = 4.0 * PI * r * r * kept + PI * r * r;
    reads_as(&f.text(), volume, area, "ball with a wedge out");
}

#[test]
fn a_sphere_with_a_seam_to_its_pole_keeps_its_axis() {
    // A hemisphere with a corner on its pole too, but its seam is a meridian of that axis, so the axis can't be turned.
    let r = 6.0;
    let mut f = File::new();
    let (rim, pole) = (f.vertex(v3(r, 0.0, 0.0)), f.vertex(v3(0.0, 0.0, r)));
    let place = f.placement(Vec3::ZERO, Vec3::Z, Vec3::X);
    let circle = f.add(format!("CIRCLE('',#{place},{r:.3})"));
    let round = f.add(format!("EDGE_CURVE('',#{rim},#{rim},#{circle},.T.)"));
    let upright = f.placement(Vec3::ZERO, -Vec3::Y, Vec3::X);
    let arc = f.add(format!("CIRCLE('',#{upright},{r:.3})"));
    let seam = f.add(format!("EDGE_CURVE('',#{rim},#{pole},#{arc},.T.)"));
    let ball = f.add(format!("SPHERICAL_SURFACE('',#{place},{r:.3})"));
    let dome = f.face(&[vec![(round, true), (seam, true), (seam, false)]], ball, true);
    let under = f.plane(Vec3::ZERO, -Vec3::Z, Vec3::X);
    let base = f.face(&[vec![(round, false)]], under, true);
    f.solid(&[dome, base]);
    reads_as(&f.text(), 2.0 / 3.0 * PI * r * r * r, 3.0 * PI * r * r, "hemisphere");
}

#[test]
fn a_circles_coarse_polygon_doesnt_make_it_look_like_the_hole() {
    // A ring with a loop of many short sides just inside a circle. That loop encloses more than a polygon of few points
    // around the circle does, which used to make it the outer one, and turn half of the part inside out.
    let (r, inner, h, sides) = (100.0, 99.9, 5.0, 64usize);
    let mut f = File::new();
    let (lo, hi) = (f.vertex(v3(r, 0.0, 0.0)), f.vertex(v3(r, 0.0, h)));
    let (p, q) = (f.placement(Vec3::ZERO, Vec3::Z, Vec3::X), f.placement(v3(0.0, 0.0, h), Vec3::Z, Vec3::X));
    let (cl, ch) = (f.add(format!("CIRCLE('',#{p},{r:.3})")), f.add(format!("CIRCLE('',#{q},{r:.3})")));
    let bottom_rim = f.add(format!("EDGE_CURVE('',#{lo},#{lo},#{cl},.T.)"));
    let top_rim = f.add(format!("EDGE_CURVE('',#{hi},#{hi},#{ch},.T.)"));
    let round = f.add(format!("CYLINDRICAL_SURFACE('',#{p},{r:.3})"));
    let outside = f.face(&[vec![(bottom_rim, true)], vec![(top_rim, false)]], round, true);
    let at = |k: usize, z: f64| {
        let a = k as f64 / sides as f64 * 2.0 * PI;
        v3(inner * a.cos(), inner * a.sin(), z)
    };
    let low: Vec<usize> = (0..sides).map(|k| f.vertex(at(k, 0.0))).collect();
    let high: Vec<usize> = (0..sides).map(|k| f.vertex(at(k, h))).collect();
    let next = |k: usize| (k + 1) % sides;
    let low_edges: Vec<usize> = (0..sides).map(|k| f.line(at(k, 0.0), at(next(k), 0.0), low[k], low[next(k)])).collect();
    let high_edges: Vec<usize> = (0..sides).map(|k| f.line(at(k, h), at(next(k), h), high[k], high[next(k)])).collect();
    let ups: Vec<usize> = (0..sides).map(|k| f.line(at(k, 0.0), at(k, h), low[k], high[k])).collect();
    let mut faces = vec![outside];
    for k in 0..sides {
        let (a, c) = (at(k, 0.0), at(next(k), 0.0));
        let middle = (a + c) * 0.5;
        let plane = f.plane(middle, -middle.normalized(), (c - a).normalized());
        faces.push(f.face(&[vec![(low_edges[k], false), (ups[k], true), (high_edges[k], true), (ups[next(k)], false)]], plane, true));
    }
    let under = f.plane(Vec3::ZERO, -Vec3::Z, Vec3::X);
    let over = f.plane(v3(0.0, 0.0, h), Vec3::Z, Vec3::X);
    let hole_below: Vec<(usize, bool)> = low_edges.iter().map(|&e| (e, true)).collect();
    let hole_above: Vec<(usize, bool)> = high_edges.iter().rev().map(|&e| (e, false)).collect();
    faces.push(f.face(&[vec![(bottom_rim, false)], hole_below], under, true));
    faces.push(f.face(&[vec![(top_rim, true)], hole_above], over, true));
    f.solid(&faces);
    let n = sides as f64;
    let polygon = n / 2.0 * inner * inner * (2.0 * PI / n).sin();
    let side = 2.0 * inner * (PI / n).sin();
    let flat = PI * r * r - polygon;
    reads_as(&f.text(), flat * h, 2.0 * flat + 2.0 * PI * r * h + n * side * h, "thin ring");
}

#[test]
fn a_gap_is_not_closed_by_a_long_trip_along_near_tangent_surfaces() {
    // A prism with a kink of half a degree down one side. With its vertex a hair off, the place all three surfaces meet
    // is about a hundred times further along than the gap is wide, and following that would distort whatever's small.
    let bend = 0.5f64.to_radians();
    let corner = v2(30.0, 0.0);
    let far = v2(30.0 + 40.0 * bend.cos(), 40.0 * bend.sin());
    let wire = PathBuilder::new(v2(0.0, 0.0)).line_to(corner).line_to(far).line_to(v2(30.0, 30.0)).line_to(v2(0.0, 30.0)).close();
    let prism = extrude(&Profile::new(wire), &Frame::WORLD, 10.0, 1).unwrap();
    let text = write_step(&prism, "prism");
    // The kink's two vertices, moved along the second side. That's a long way for how far it leaves them off the first.
    let along = 0.02;
    let off = along * bend.sin();
    let mut moved = text.clone();
    let mut wanted = Vec::new();
    for l in text.lines().filter(|l| l.contains("=VERTEX_POINT(")) {
        let p = ids(l)[1];
        let point = text.lines().find(|m| m.starts_with(&format!("#{p}=CARTESIAN_POINT("))).unwrap();
        let inside = &point[point.find("',(").unwrap() + 3..point.find("));").unwrap()];
        let c: Vec<f64> = inside.split(',').map(|x| x.parse().unwrap()).collect();
        if (c[0] - corner.x).abs() < 1e-9 && c[1].abs() < 1e-9 {
            let at = v3(c[0] + along * bend.cos(), off, c[2]);
            wanted.push(at);
            moved = moved.replace(point, &format!("#{p}=CARTESIAN_POINT('',({:.15e},{:.15e},{:.15e}));", at.x, at.y, at.z));
        }
    }
    assert_eq!(wanted.len(), 2);
    let back = read_step(&moved, 1).unwrap().remove(0);
    for w in wanted {
        let nearest = back.vertices.iter().map(|v| v.dist(w)).fold(f64::INFINITY, f64::min);
        assert!(nearest < 1e-6, "a vertex was moved {nearest} to close a gap of {off}");
    }
    assert!(back.tolerance > 0.0 && back.tolerance <= 1e-3, "{}", back.tolerance);
    validate(&back).unwrap();
}

#[test]
fn a_shell_called_closed_that_isnt_says_so() {
    // Take a face out of the list, and what's left has edges with a face on one side only.
    let (_, text, ..) = part();
    let shell = line_of(&text, "CLOSED_SHELL");
    let first = ids(shell)[1];
    let open = text.replace(shell, &shell.replacen(&format!("#{first},"), "", 1));
    let why = read_step(&open, 1).unwrap_err().to_string();
    assert!(why.contains("isn't closed"), "{why}");
}

#[test]
fn a_band_thinner_than_a_chord_strays_still_winds_the_right_way_and_meshes() {
    // A chamfer 1.5 microns wide round a rim 4 mm in radius, as a CAD system's tolerance can leave. A chord of its curved
    // edges strays much further than that, and when the two rims are cut in different places, as here, the polygons
    // through them cross, which used to get the part refused.
    let (r, h, w) = (4.24, 3.0, 1.5e-3);
    let wire = PathBuilder::new(v2(0.0, 0.0)).line_to(v2(r, 0.0)).line_to(v2(r, h - w)).line_to(v2(r - w, h)).line_to(v2(0.0, h)).close();
    let mut b = revolve(&Profile::new(wire), &Frame::WORLD, TAU, 1).unwrap();
    let want = mass_properties(&b);
    // One of the rim's edges cut in two, at a sixth of its length, and only that one.
    let on_upper = b.edges.iter().position(|e| {
        let p = e.curve.eval((e.t0 + e.t1) / 2.0);
        matches!(e.curve, Curve::Circle { .. }) && (p.z - h).abs() < 1e-9
    });
    let e = on_upper.unwrap();
    let old = b.edges[e].clone();
    let t = old.t0 + (old.t1 - old.t0) / 6.0;
    let v = b.vertices.len();
    b.vertices.push(old.curve.eval(t));
    b.edges[e] = Edge { curve: old.curve.clone(), t0: old.t0, t1: t, v0: old.v0, v1: v };
    let f = b.edges.len();
    b.edges.push(Edge { curve: old.curve.clone(), t0: t, t1: old.t1, v0: v, v1: old.v1 });
    for face in &mut b.faces {
        for lp in &mut face.loops {
            if let Some(k) = lp.iter().position(|c| c.edge == e) {
                let reversed = lp[k].reversed;
                let (first, second) = if reversed { (f, e) } else { (e, f) };
                lp[k] = Coedge { edge: first, reversed };
                lp.insert(k + 1, Coedge { edge: second, reversed });
            }
        }
    }
    validate(&b).unwrap();
    let got = mass_properties(&b);
    close(got.volume, want.volume, 1e-12, "volume");
    close(got.area, want.area, 1e-12, "area");
    let mesh = tessellate(&b, &TessParams::for_size(b.aabb().diagonal())).unwrap();
    assert_eq!(mesh.open_edge_count(), 0);
}
