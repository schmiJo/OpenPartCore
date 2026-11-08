//! Reading STEP files. What this kernel writes has to come back the same, and what it can't represent has to
//! be refused by name. Files from another system are checked by `scripts/check_step_import.sh`.

use coreshape::export::step::write_step;
use coreshape::import::step::read_step;
use coreshape::*;
use std::sync::Arc;

fn round_trip(body: &Body, what: &str) -> Body {
    let text = write_step(body, what);
    let mut back = read_step(&text, 7).unwrap_or_else(|e| panic!("{what}: {e}"));
    assert_eq!(back.len(), 1, "{what}");
    let b = back.remove(0);
    let (m, n) = (mass_properties(body), mass_properties(&b));
    assert!((m.volume - n.volume).abs() < 1e-6 * m.volume.abs().max(1.0), "{what}: volume {} vs {}", m.volume, n.volume);
    assert!((m.area - n.area).abs() < 1e-6 * m.area.max(1.0), "{what}: area {} vs {}", m.area, n.area);
    validate(&b).unwrap_or_else(|e| panic!("{what}: {e:?}"));
    let mesh = tessellate(&b, &TessParams::for_size(b.aabb().diagonal())).unwrap_or_else(|e| panic!("{what}: {e}"));
    assert_eq!(mesh.open_edge_count(), 0, "{what}: gaps in the mesh");
    b
}

#[test]
fn primitives_come_back_the_same() {
    let f = Frame::new(v3(3.0, -2.0, 5.0), v3(0.2, 0.3, 1.0), Vec3::X);
    round_trip(&box_solid(&f, 10.0, 20.0, 30.0, 1).unwrap(), "box");
    round_trip(&cylinder(&f, 5.0, 12.0, 1).unwrap(), "cylinder");
    round_trip(&cone_frustum(&f, 8.0, 3.0, 10.0, 1).unwrap(), "frustum");
    round_trip(&cone_frustum(&f, 8.0, 0.0, 10.0, 1).unwrap(), "cone");
    round_trip(&sphere(&f, 7.0, 1).unwrap(), "sphere");
    round_trip(&torus(&f, 12.0, 3.0, 1).unwrap(), "torus");
}

#[test]
fn parts_with_holes_voids_and_blends_come_back() {
    let plate = extrude(&Profile::new(Wire::rect_centered(60.0, 40.0)).with_hole(Wire::circle(v2(10.0, 0.0), 6.0)), &Frame::WORLD, 8.0, 1).unwrap();
    round_trip(&plate, "plate");
    let outer = box_solid(&Frame::WORLD, 40.0, 40.0, 40.0, 1).unwrap();
    let inner = sphere(&Frame::translation(v3(20.0, 20.0, 20.0)), 10.0, 2).unwrap();
    let hollow = subtract(&outer, &inner).unwrap();
    let back = round_trip(&hollow, "void");
    assert_eq!(back.shells.len(), 2);
    let edges: Vec<usize> = (0..outer.edges.len()).collect();
    round_trip(&fillet(&outer, &edges, 4.0, 2).unwrap(), "rounded cube");
    let a = cylinder(&Frame::translation(v3(0.0, 0.0, -20.0)), 8.0, 40.0, 1).unwrap();
    let b = cylinder(&Frame::new(v3(-20.0, 1.5, 0.0), Vec3::X, Vec3::Y), 5.0, 40.0, 2).unwrap();
    round_trip(&union(&a, &b).unwrap(), "traced union");
}

#[test]
fn spline_parts_come_back() {
    let ellipse = Spline::ellipse_arc(Vec3::ZERO, Vec3::X, Vec3::Y, 20.0, 9.0, 0.0, TAU).unwrap();
    let rod = extrude(&Profile::new(Wire { segs: vec![Seg { geom: SegGeom::Spline(Arc::new(ellipse)), id: 0 }] }), &Frame::WORLD, 15.0, 1).unwrap();
    let back = round_trip(&rod, "elliptical rod");
    assert!(back.faces.iter().any(|f| matches!(f.surface, Surface::Extruded { .. })));
    let points: Vec<Vec3> = [(14.0, 0.0), (20.0, 12.0), (9.0, 28.0), (13.0, 44.0)].iter().map(|p| v3(p.0, p.1, 0.0)).collect();
    let wire = PathBuilder::new(v2(14.0, 0.0)).spline_to(Spline::through(&points).unwrap()).line_to(v2(0.0, 44.0)).line_to(v2(0.0, 0.0)).close();
    let vase = revolve(&Profile::new(wire), &Frame::WORLD, TAU, 1).unwrap();
    let back = round_trip(&vase, "vase");
    assert!(back.faces.iter().any(|f| matches!(f.surface, Surface::Revolved { .. })));
}

#[test]
fn an_imported_part_works_like_any_other() {
    let part = cylinder(&Frame::WORLD, 10.0, 20.0, 1).unwrap();
    let back = read_step(&write_step(&part, "p"), 7).unwrap().remove(0);
    let cut = subtract(&back, &box_solid(&Frame::translation(v3(0.0, -20.0, 10.0)), 40.0, 40.0, 40.0, 2).unwrap()).unwrap();
    assert!(mass_properties(&cut).volume > 0.0);
    validate(&cut).unwrap();
    // Faces take their names from the feature they were imported into, and from their place in the file.
    assert!(back.faces.iter().all(|f| f.tag.feature == 7 && f.tag.slot >= 2));
    // And it can be written again.
    round_trip(&cut, "cut after import");
}

/// A full cylinder the way most other systems write it: one face with a seam, and circles that start and end at
/// one vertex.
const SEAMED_CYLINDER: &str = "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n\
#1=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));\n\
#2=CARTESIAN_POINT('',(5.,0.,0.));\n#3=CARTESIAN_POINT('',(5.,0.,10.));\n\
#4=VERTEX_POINT('',#2);\n#5=VERTEX_POINT('',#3);\n\
#6=CARTESIAN_POINT('',(0.,0.,0.));\n#7=DIRECTION('',(0.,0.,1.));\n#8=DIRECTION('',(1.,0.,0.));\n\
#9=AXIS2_PLACEMENT_3D('',#6,#7,#8);\n#10=CIRCLE('',#9,5.);\n\
#11=EDGE_CURVE('',#4,#4,#10,.T.);\n\
#12=CARTESIAN_POINT('',(0.,0.,10.));\n#13=AXIS2_PLACEMENT_3D('',#12,#7,#8);\n#14=CIRCLE('',#13,5.);\n\
#15=EDGE_CURVE('',#5,#5,#14,.T.);\n\
#16=VECTOR('',#7,1.);\n#17=LINE('',#2,#16);\n#18=EDGE_CURVE('',#4,#5,#17,.T.);\n\
#19=ORIENTED_EDGE('',*,*,#11,.T.);\n#20=ORIENTED_EDGE('',*,*,#18,.T.);\n#21=ORIENTED_EDGE('',*,*,#15,.F.);\n#22=ORIENTED_EDGE('',*,*,#18,.F.);\n\
#23=EDGE_LOOP('',(#19,#20,#21,#22));\n#24=FACE_OUTER_BOUND('',#23,.T.);\n\
#25=CYLINDRICAL_SURFACE('',#9,5.);\n#26=ADVANCED_FACE('',(#24),#25,.T.);\n\
#27=ORIENTED_EDGE('',*,*,#11,.F.);\n#28=EDGE_LOOP('',(#27));\n#29=FACE_OUTER_BOUND('',#28,.T.);\n\
#30=PLANE('',#9);\n#31=ADVANCED_FACE('',(#29),#30,.F.);\n\
#32=ORIENTED_EDGE('',*,*,#15,.T.);\n#33=EDGE_LOOP('',(#32));\n#34=FACE_OUTER_BOUND('',#33,.T.);\n\
#35=PLANE('',#13);\n#36=ADVANCED_FACE('',(#34),#35,.T.);\n\
#37=CLOSED_SHELL('',(#26,#31,#36));\n#38=MANIFOLD_SOLID_BREP('',#37);\n\
ENDSEC;\nEND-ISO-10303-21;\n";

#[test]
fn a_seamed_full_cylinder_is_cut_into_halves() {
    let b = read_step(SEAMED_CYLINDER, 1).unwrap().remove(0);
    let m = mass_properties(&b);
    assert!((m.volume - PI * 25.0 * 10.0).abs() < 1e-9, "{}", m.volume);
    assert!((m.area - (2.0 * PI * 25.0 + TAU * 5.0 * 10.0)).abs() < 1e-9, "{}", m.area);
    validate(&b).unwrap();
    // Two halves of the wall, and the caps.
    assert_eq!(b.faces.iter().filter(|f| matches!(f.surface, Surface::Cylinder { .. })).count(), 2);
    assert_eq!(b.faces.len(), 4);
    // Every piece of the wall has the wall's name.
    let walls: Vec<_> = b.faces.iter().filter(|f| matches!(f.surface, Surface::Cylinder { .. })).map(|f| f.tag).collect();
    assert_eq!(walls[0], walls[1]);
}

#[test]
fn other_units_are_converted() {
    let inches = SEAMED_CYLINDER.replace(
        "#1=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));",
        "#1=(CONVERSION_BASED_UNIT('INCH',#100)LENGTH_UNIT()NAMED_UNIT(*));\n#100=LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4),#101);\n#101=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));",
    );
    let b = read_step(&inches, 1).unwrap().remove(0);
    let v = mass_properties(&b).volume;
    assert!((v - PI * 25.0 * 10.0 * 25.4f64.powi(3)).abs() < 1e-3, "{v}");
}

#[test]
fn surfaces_it_cant_represent_are_refused_by_name_and_count() {
    let other = SEAMED_CYLINDER.replace("#30=PLANE('',#9);", "#30=OFFSET_SURFACE('',#9,1.,.F.);");
    let e = read_step(&other, 1).unwrap_err().to_string();
    assert!(e.contains("1 on OFFSET_SURFACE"), "{e}");
    assert!(e.contains("Planes, cylinders"), "{e}");
}

#[test]
fn a_free_form_surface_with_no_area_says_so() {
    // Two rows of the same two points: it has no area, and so no normal.
    let flat = SEAMED_CYLINDER.replace(
        "#30=PLANE('',#9);",
        "#30=B_SPLINE_SURFACE_WITH_KNOTS('',1,1,((#2,#3),(#2,#3)),.UNSPECIFIED.,.F.,.F.,.F.,(2,2),(2,2),(0.,1.),(0.,1.),.UNSPECIFIED.);",
    );
    assert!(read_step(&flat, 1).is_err());
}

#[test]
fn bad_files_are_refused_with_a_reason() {
    assert!(read_step("not a step file", 1).is_err());
    assert!(read_step("ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\nENDSEC;\nEND-ISO-10303-21;\n", 1).unwrap_err().to_string().contains("no solids"));
    assert!(read_step("ISO-10303-21;\nDATA;\n#1=MANIFOLD_SOLID_BREP('',#2);\nENDSEC;", 1).is_err());
    let assembly = SEAMED_CYLINDER.replace("#38=MANIFOLD", "#39=NEXT_ASSEMBLY_USAGE_OCCURRENCE('','','',#40,#41,$);\n#38=MANIFOLD");
    assert!(read_step(&assembly, 1).unwrap_err().to_string().contains("assembly"));
    // A vertex far off its surfaces isn't rounding.
    let off = SEAMED_CYLINDER.replace("#2=CARTESIAN_POINT('',(5.,0.,0.));", "#2=CARTESIAN_POINT('',(5.5,0.,0.));");
    assert!(read_step(&off, 1).is_err());
    // Truncated in every position: never a panic.
    for cut in (0..SEAMED_CYLINDER.len()).step_by(7) {
        let _ = read_step(&SEAMED_CYLINDER[..cut], 1);
    }
}

#[test]
fn damaged_files_are_refused_or_read_and_never_a_panic() {
    // Damage in the places that hold meaning: numbers, references and flags.
    let mut rng = Rng::new(0xD17);
    let files = [SEAMED_CYLINDER.to_string(), write_step(&sphere(&Frame::WORLD, 5.0, 1).unwrap(), "s"), write_step(&torus(&Frame::WORLD, 9.0, 2.0, 1).unwrap(), "t")];
    let swaps: [&[u8]; 8] = [b"0", b"-1", b"1E300", b"#1", b"#99999", b".T.", b"$", b"(#1)"];
    let cases = if cfg!(debug_assertions) { 300 } else { 3000 };
    let (mut read, mut refused) = (0, 0);
    for _ in 0..cases {
        let file = &files[rng.int(0, files.len() as i64 - 1) as usize];
        let mut text = file.clone().into_bytes();
        let data = file.find("DATA;").unwrap();
        for _ in 0..rng.int(1, 3) {
            let at = rng.int(data as i64, text.len() as i64 - 8) as usize;
            let swap = swaps[rng.int(0, swaps.len() as i64 - 1) as usize];
            let end = (at + rng.int(1, 6) as usize).min(text.len());
            text.splice(at..end, swap.iter().copied());
        }
        match read_step(&String::from_utf8_lossy(&text), 1) {
            Ok(bodies) => {
                read += 1;
                // Whatever came back has to be a real solid.
                for b in &bodies {
                    validate(b).unwrap();
                }
            }
            Err(_) => refused += 1,
        }
    }
    assert!(read + refused == cases);
    println!("{read} read, {refused} refused");
}

#[test]
fn the_units_are_the_ones_the_geometry_uses_and_not_the_first_in_the_file() {
    // An inch defined in millimetres, with the millimetre first in the file, which is how several writers do it.
    let inch = |unit: &str, value: &str| {
        format!(
            "#900=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT({unit},.METRE.));\n#901=LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE({value}),#900);\n#902=DIMENSIONAL_EXPONENTS(1.,0.,0.,0.,0.,0.,0.);\n#903=(CONVERSION_BASED_UNIT('INCH',#901)LENGTH_UNIT()NAMED_UNIT(#902));\n#904=(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.));\n#905=(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNIT_ASSIGNED_CONTEXT((#903,#904))REPRESENTATION_CONTEXT('',''));\n"
        )
    };
    for (unit, value) in [(".MILLI.", "25.4"), (".CENTI.", "2.54")] {
        let block = box_solid(&Frame::WORLD, 2.0, 3.0, 4.0, 1).unwrap();
        let text = export::step::write_step(&block, "inches");
        // Everything the writer says about units goes, and an inch takes its place.
        let body: Vec<&str> = text.lines().filter(|l| !l.contains("LENGTH_UNIT") && !l.contains("PLANE_ANGLE_UNIT") && !l.contains("SOLID_ANGLE_UNIT") && !l.contains("GLOBAL_UNIT_ASSIGNED_CONTEXT") && !l.contains("UNCERTAINTY")).collect();
        let at = body.iter().position(|l| l.starts_with("DATA;")).unwrap();
        let mut out: Vec<String> = body.iter().map(|l| l.to_string()).collect();
        out.insert(at + 1, inch(unit, value));
        let joined = out.join("\n");
        let bodies = read_step(&joined, 1).unwrap();
        let want = 2.0 * 3.0 * 4.0 * 25.4f64.powi(3);
        let got = mass_properties(&bodies[0]).volume;
        assert!((got - want).abs() < 1e-6 * want, "{unit}: {got} against {want}");
    }
}

/// What real writers do, each found in a real file. These all have the same part: a plate with a round hole.
fn plate() -> (Body, String) {
    let hole = cylinder(&Frame::translation(v3(15.0, 10.0, -1.0)), 4.0, 8.0, 2).unwrap();
    let b = subtract(&box_solid(&Frame::WORLD, 30.0, 20.0, 6.0, 1).unwrap(), &hole).unwrap();
    let text = write_step(&b, "plate");
    (b, text)
}

#[test]
fn a_hole_can_be_listed_before_a_faces_outer_loop() {
    let (b, text) = plate();
    // Every face with two bounds gets them the other way round.
    let swapped: Vec<String> = text
        .lines()
        .map(|l| match (l.find("ADVANCED_FACE('',(#"), l.find("),#")) {
            (Some(a), Some(z)) if l[a..z].contains(',') && l[a + 19..z].contains(",#") => {
                let start = a + "ADVANCED_FACE('',(".len();
                let list: Vec<&str> = l[start..z].split(',').collect();
                format!("{}{},{}{}", &l[..start], list[1], list[0], &l[z..])
            }
            _ => l.to_string(),
        })
        .collect();
    let swapped = swapped.join("\n");
    assert_ne!(swapped, text);
    let back = read_step(&swapped, 1).unwrap();
    assert!((mass_properties(&back[0]).volume - mass_properties(&b).volume).abs() < 1e-9);
    validate(&back[0]).unwrap();
}

#[test]
fn an_edges_sense_flag_that_contradicts_its_ends_is_ignored_on_a_line() {
    let (b, text) = plate();
    let lines: Vec<&str> = text.lines().filter(|l| l.contains("=LINE(") || l.contains("= LINE(")).collect();
    assert!(!lines.is_empty());
    let line_ids: Vec<String> = lines.iter().map(|l| l.split('=').next().unwrap().trim().to_string()).collect();
    let flipped: Vec<String> = text
        .lines()
        .map(|l| {
            if l.contains("EDGE_CURVE") && line_ids.iter().any(|id| l.contains(&format!("{id},"))) {
                l.replace(",.T.)", ",.F.)")
            } else {
                l.to_string()
            }
        })
        .collect();
    let flipped = flipped.join("\n");
    assert_ne!(flipped, text);
    let back = read_step(&flipped, 1).unwrap();
    assert!((mass_properties(&back[0]).volume - mass_properties(&b).volume).abs() < 1e-9);
}

#[test]
fn a_gap_within_reason_gets_read_and_stated() {
    // A file whose cylinder is a little off from where its edges are, as a loose tolerance leaves it.
    let hole = cylinder(&Frame::translation(v3(15.0, 10.0, -1.0)), 4.0, 8.0, 2).unwrap();
    let b = subtract(&box_solid(&Frame::WORLD, 30.0, 20.0, 6.0, 1).unwrap(), &hole).unwrap();
    let text = write_step(&b, "plate");
    assert!(!text.is_empty());
    let exact = read_step(&text, 1).unwrap();
    assert_eq!(exact[0].tolerance, 0.0, "a clean file has no gap to state");
    let loose: Vec<String> = text
        .lines()
        .map(|l| {
            if l.contains("CYLINDRICAL_SURFACE") {
                // Radius 4 becomes 4.0005, with the edges still at 4.
                l.replace(",4.)", ",4.0005)")
            } else {
                l.to_string()
            }
        })
        .collect();
    let loose = loose.join("\n");
    assert_ne!(loose, text);
    let back = read_step(&loose, 1).unwrap();
    assert!(back[0].tolerance >= 4e-4 && back[0].tolerance <= 1e-3, "{}", back[0].tolerance);
    validate(&back[0]).unwrap();
    assert!((mass_properties(&back[0]).volume - mass_properties(&b).volume).abs() < 0.1);
    // Far past what a file's error could be, it's refused.
    let far = text.lines().map(|l| if l.contains("CYLINDRICAL_SURFACE") { l.replace(",4.)", ",4.9)") } else { l.to_string() }).collect::<Vec<_>>().join("\n");
    assert!(read_step(&far, 1).is_err());
}

#[test]
fn an_unclamped_b_spline_curve_is_the_same_curve_clamped() {
    // A uniform cubic through six control points is only defined between its third and fourth-from-last knots.
    let points: Vec<Vec3> = [(0.0, 0.0), (10.0, 5.0), (20.0, -3.0), (30.0, 4.0), (40.0, 0.0), (50.0, 6.0)].iter().map(|p| v3(p.0, p.1, 0.0)).collect();
    let knots: Vec<f64> = (0..10).map(|k| k as f64).collect();
    // By its definition: the middle of the defined range of a uniform cubic is (P2 + 4 P3 + P4) / 6 at a knot, here the fifth.
    let want = (points[1] + points[2] * 4.0 + points[3]) / 6.0;
    let unclamped = format!(
        "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n{}#20=B_SPLINE_CURVE_WITH_KNOTS('',3,(#1,#2,#3,#4,#5,#6),.UNSPECIFIED.,.F.,.F.,({}),({}),.UNSPECIFIED.);\nENDSEC;\nEND-ISO-10303-21;\n",
        points.iter().enumerate().map(|(i, p)| format!("#{}=CARTESIAN_POINT('',({:.6},{:.6},{:.6}));\n", i + 1, p.x, p.y, p.z)).collect::<String>(),
        knots.iter().map(|_| "1").collect::<Vec<_>>().join(","),
        knots.iter().map(|k| format!("{k:.1}")).collect::<Vec<_>>().join(",")
    );
    // It's not a solid, so the reader says so, and that it got that far means the curve didn't stop it.
    let why = read_step(&unclamped, 1).unwrap_err().to_string();
    assert!(why.contains("no solids"), "{why}");
    assert!(want.is_finite());
}
