//! Independent review harness (lives only in a scratch copy).
use coreshape::*;
use std::fmt::Write as _;
use std::io::Write as _;
use std::sync::mpsc;
use std::time::{Duration, Instant};

#[derive(Clone, Debug)]
enum Prim {
    Box(Frame, f64, f64, f64),
    Cyl(Frame, f64, f64),
    Cone(Frame, f64, f64, f64),
    Sph(Frame, f64),
    Tor(Frame, f64, f64),
}

fn build(p: &Prim) -> Result<Body> {
    match *p {
        Prim::Box(f, a, b, c) => box_solid(&f, a, b, c, 1),
        Prim::Cyl(f, r, h) => cylinder(&f, r, h, 1),
        Prim::Cone(f, a, b, h) => cone_frustum(&f, a, b, h, 1),
        Prim::Sph(f, r) => sphere(&f, r, 1),
        Prim::Tor(f, a, b) => torus(&f, a, b, 1),
    }
}

fn vj(v: Vec3) -> String {
    format!("[{:.17e},{:.17e},{:.17e}]", v.x, v.y, v.z)
}
fn fj(f: &Frame) -> String {
    format!("\"o\":{},\"x\":{},\"y\":{},\"z\":{}", vj(f.origin), vj(f.x), vj(f.y), vj(f.z))
}
fn pj(p: &Prim) -> String {
    match p {
        Prim::Box(f, a, b, c) => format!("{{\"t\":\"box\",{},\"p\":[{a:.17e},{b:.17e},{c:.17e}]}}", fj(f)),
        Prim::Cyl(f, r, h) => format!("{{\"t\":\"cyl\",{},\"p\":[{r:.17e},{h:.17e}]}}", fj(f)),
        Prim::Cone(f, a, b, h) => format!("{{\"t\":\"cone\",{},\"p\":[{a:.17e},{b:.17e},{h:.17e}]}}", fj(f)),
        Prim::Sph(f, r) => format!("{{\"t\":\"sph\",{},\"p\":[{r:.17e}]}}", fj(f)),
        Prim::Tor(f, a, b) => format!("{{\"t\":\"tor\",{},\"p\":[{a:.17e},{b:.17e}]}}", fj(f)),
    }
}

enum Run {
    Ok(Body, f64),
    Err(String, f64),
    Panic(f64),
    Hang,
}

fn guarded<F>(limit: f64, f: F) -> Run
where
    F: FnOnce() -> Result<Body> + Send + 'static,
{
    let (tx, rx) = mpsc::channel();
    std::thread::spawn(move || {
        let t = Instant::now();
        let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(f));
        let ms = t.elapsed().as_secs_f64() * 1e3;
        let _ = tx.send(match r {
            Ok(Ok(b)) => Run::Ok(b, ms),
            Ok(Err(e)) => Run::Err(e.to_string(), ms),
            Err(_) => Run::Panic(ms),
        });
    });
    rx.recv_timeout(Duration::from_secs_f64(limit)).unwrap_or(Run::Hang)
}

fn clean(s: &str) -> String {
    s.chars().map(|c| if c == '"' || c == '\\' || c == '\n' { ' ' } else { c }).take(140).collect()
}

fn frame_on(o: Vec3, z: Vec3) -> Frame {
    // A deterministic X hint that is not parallel to z.
    let hint = if z.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
    Frame::new(o, z, hint)
}

/// Random primitives, either in general position or with "CAD-like" relations.
fn gen(rng: &mut Rng, mode: u32) -> Prim {
    match mode {
        // General position, with a different size distribution from the repo's fuzz.
        0 => {
            let o = v3(rng.range(-4.0, 4.0), rng.range(-4.0, 4.0), rng.range(-4.0, 4.0));
            let mut z = v3(rng.range(-1.0, 1.0), rng.range(-1.0, 1.0), rng.range(-1.0, 1.0));
            if z.norm() < 0.1 {
                z = Vec3::Z;
            }
            let f = frame_on(o, z.normalized());
            match rng.int(0, 4) {
                0 => Prim::Box(f, rng.range(0.3, 9.0), rng.range(0.3, 9.0), rng.range(0.3, 9.0)),
                1 => Prim::Cyl(f, rng.range(0.2, 5.0), rng.range(0.3, 10.0)),
                2 => Prim::Sph(f, rng.range(0.3, 6.0)),
                3 => {
                    let a = rng.range(0.3, 5.0);
                    let b = rng.range(0.3, 5.0);
                    Prim::Cone(f, a, if (a - b).abs() < 1e-3 { b + 0.5 } else { b }, rng.range(0.3, 9.0))
                }
                _ => {
                    let major = rng.range(1.5, 6.0);
                    Prim::Tor(f, major, rng.range(0.2, major * 0.8))
                }
            }
        }
        // CAD-like: integer positions and sizes, axes along X, Y, Z or a diagonal.
        // This is full of equal radii, shared planes and tangencies between
        // perpendicular axes, which the repo's aligned category (Z axes only) has not.
        _ => {
            let g = |rng: &mut Rng, a: i64, b: i64| rng.int(a, b) as f64;
            let o = v3(g(rng, -3, 3), g(rng, -3, 3), g(rng, -3, 3));
            let axes = [Vec3::X, Vec3::Y, Vec3::Z, v3(1.0, 1.0, 0.0).normalized(), v3(1.0, 0.0, 1.0).normalized()];
            let z = axes[rng.int(0, 4) as usize];
            let f = frame_on(o, z);
            match rng.int(0, 4) {
                0 => Prim::Box(f, g(rng, 1, 6), g(rng, 1, 6), g(rng, 1, 6)),
                1 => Prim::Cyl(f, g(rng, 1, 3), g(rng, 1, 8)),
                2 => Prim::Sph(f, g(rng, 1, 4)),
                3 => {
                    let a = g(rng, 1, 3);
                    let b = g(rng, 0, 3);
                    Prim::Cone(f, a, if a == b { b + 1.0 } else { b }, g(rng, 1, 6))
                }
                _ => {
                    let major = g(rng, 2, 5);
                    Prim::Tor(f, major, g(rng, 1, major as i64 - 1))
                }
            }
        }
    }
}

fn differential(path: &str, first: u64, count: u64, mode: u32) {
    let mut out = std::fs::OpenOptions::new().create(true).append(true).open(path).unwrap();
    for seed in first..first + count {
        let mut rng = Rng::new(seed.wrapping_mul(2654435761).wrapping_add(mode as u64 * 97 + 12345));
        let (pa, pb) = (gen(&mut rng, mode), gen(&mut rng, mode));
        let (Ok(a), Ok(b)) = (build(&pa), build(&pb)) else {
            writeln!(
                out,
                "{{\"mode\":{mode},\"seed\":{seed},\"build\":\"failed\",\"a\":{},\"b\":{}}}",
                pj(&pa),
                pj(&pb)
            )
            .unwrap();
            continue;
        };
        let mut line = format!(
            "{{\"mode\":{mode},\"seed\":{seed},\"a\":{},\"b\":{},\"va\":{:.12e},\"vb\":{:.12e},\"res\":{{",
            pj(&pa),
            pj(&pb),
            mass_properties(&a).volume,
            mass_properties(&b).volume
        );
        for (i, (name, op)) in
            [("union", BoolOp::Union), ("cut", BoolOp::Subtract), ("common", BoolOp::Intersect)].iter().enumerate()
        {
            let (a2, b2, op2) = (a.clone(), b.clone(), *op);
            let run = guarded(20.0, move || match op2 {
                BoolOp::Union => union(&a2, &b2),
                BoolOp::Subtract => subtract(&a2, &b2),
                BoolOp::Intersect => intersect(&a2, &b2),
            });
            if i > 0 {
                line.push(',');
            }
            match run {
                Run::Ok(r, ms) => {
                    let mesh = tessellate(&r, &TessParams::for_size(r.aabb().diagonal().max(1e-3)));
                    let open = mesh.as_ref().map(|m| m.open_edge_count() as i64).unwrap_or(-1);
                    write!(
                        line,
                        "\"{name}\":{{\"s\":\"ok\",\"v\":{:.12e},\"ms\":{ms:.2},\"faces\":{},\"open\":{open}}}",
                        mass_properties(&r).volume,
                        r.faces.len()
                    )
                    .unwrap();
                }
                Run::Err(e, ms) => {
                    write!(line, "\"{name}\":{{\"s\":\"refused\",\"msg\":\"{}\",\"ms\":{ms:.2}}}", clean(&e)).unwrap()
                }
                Run::Panic(ms) => write!(line, "\"{name}\":{{\"s\":\"panic\",\"ms\":{ms:.2}}}").unwrap(),
                Run::Hang => write!(line, "\"{name}\":{{\"s\":\"hang\"}}").unwrap(),
            }
        }
        line.push_str("}}");
        writeln!(out, "{line}").unwrap();
    }
}

fn top_edges(b: &Body, z: f64) -> Vec<usize> {
    b.edges_where(|i, _| (b.edge_midpoint(i).z - z).abs() < 1e-6)
}

type Case = (&'static str, Option<f64>, Box<dyn FnOnce() -> Result<Body> + Send>);

fn c(name: &'static str, expected: Option<f64>, f: impl FnOnce() -> Result<Body> + Send + 'static) -> Case {
    (name, expected, Box::new(f))
}

fn named() -> Vec<Case> {
    let w = Frame::WORLD;
    let zf = |x: f64, y: f64, z: f64| Frame::translation(v3(x, y, z));
    let xf = |x: f64, y: f64, z: f64| frame_on(v3(x, y, z), Vec3::X);
    let yf = |x: f64, y: f64, z: f64| frame_on(v3(x, y, z), Vec3::Y);
    let rb = |a: f64, b: f64, c: f64, r: f64| {
        let (x, y, z) = (a - 2.0 * r, b - 2.0 * r, c - 2.0 * r);
        x * y * z + 2.0 * r * (x * y + x * z + y * z) + PI * r * r * (x + y + z) + 4.0 / 3.0 * PI * r * r * r
    };
    let rounded = move || extrude(&Profile::new(Wire::rounded_rect(60.0, 40.0, 8.0)), &w, 12.0, 1);
    vec![
        // ------- Booleans with a known answer (* = OCCT result used instead when None)
        c("B steinmetz 2 cylinders r=5: intersect", Some(16.0 * 125.0 / 3.0), move || {
            intersect(&cylinder(&zf(0.0, 0.0, -20.0), 5.0, 40.0, 1)?, &cylinder(&xf(-20.0, 0.0, 0.0), 5.0, 40.0, 2)?)
        }),
        c("B steinmetz 2 cylinders r=5: union", Some(2.0 * PI * 25.0 * 40.0 - 16.0 * 125.0 / 3.0), move || {
            union(&cylinder(&zf(0.0, 0.0, -20.0), 5.0, 40.0, 1)?, &cylinder(&xf(-20.0, 0.0, 0.0), 5.0, 40.0, 2)?)
        }),
        c("B tricylinder (3 equal cylinders) intersect", Some(8.0 * (2.0 - 2f64.sqrt()) * 125.0), move || {
            let a = intersect(
                &cylinder(&zf(0.0, 0.0, -20.0), 5.0, 40.0, 1)?,
                &cylinder(&xf(-20.0, 0.0, 0.0), 5.0, 40.0, 2)?,
            )?;
            intersect(&a, &cylinder(&yf(0.0, -20.0, 0.0), 5.0, 40.0, 3)?)
        }),
        c("B tee pipe: equal r cylinders, end on axis, union", None, move || {
            union(&cylinder(&zf(0.0, 0.0, 0.0), 5.0, 30.0, 1)?, &cylinder(&xf(-20.0, 0.0, 15.0), 5.0, 40.0, 2)?)
        }),
        c(
            "B sphere r=5 in cylinder r=5 (tangent circle) subtract",
            Some(PI * 25.0 * 40.0 - 4.0 / 3.0 * PI * 125.0),
            move || subtract(&cylinder(&zf(0.0, 0.0, -20.0), 5.0, 40.0, 1)?, &sphere(&w, 5.0, 2)?),
        ),
        c("B sphere r=5 union cylinder r=5 same axis", Some(PI * 25.0 * 40.0), move || {
            union(&cylinder(&zf(0.0, 0.0, -20.0), 5.0, 40.0, 1)?, &sphere(&w, 5.0, 2)?)
        }),
        c("B sphere on plate, touching at one point: union", Some(1000.0 + 4.0 / 3.0 * PI * 8.0), move || {
            union(&box_solid(&zf(-5.0, -5.0, -10.0), 10.0, 10.0, 10.0, 1)?, &sphere(&zf(0.0, 0.0, 2.0), 2.0, 2)?)
        }),
        c("B sphere on plate, touching at one point: intersect", Some(0.0), move || {
            intersect(&box_solid(&zf(-5.0, -5.0, -10.0), 10.0, 10.0, 10.0, 1)?, &sphere(&zf(0.0, 0.0, 2.0), 2.0, 2)?)
        }),
        c("B cylinder along plate, line contact: union", Some(1000.0 + PI * 4.0 * 10.0), move || {
            union(
                &box_solid(&zf(-5.0, -5.0, -10.0), 10.0, 10.0, 10.0, 1)?,
                &cylinder(&xf(-5.0, 0.0, 2.0), 2.0, 10.0, 2)?,
            )
        }),
        c("B cylinder inscribed in box (4 tangent lines): subtract", Some(1000.0 - PI * 25.0 * 10.0), move || {
            subtract(&box_solid(&zf(-5.0, -5.0, 0.0), 10.0, 10.0, 10.0, 1)?, &cylinder(&w, 5.0, 10.0, 2)?)
        }),
        c("B cylinder r=3 inside r=5, tangent inside line: subtract", Some(PI * (25.0 - 9.0) * 10.0), move || {
            subtract(&cylinder(&w, 5.0, 10.0, 1)?, &cylinder(&zf(2.0, 0.0, 0.0), 3.0, 10.0, 2)?)
        }),
        c("B sphere r=3 inside sphere r=5, tangent: subtract", Some(4.0 / 3.0 * PI * (125.0 - 27.0)), move || {
            subtract(&sphere(&w, 5.0, 1)?, &sphere(&zf(2.0, 0.0, 0.0), 3.0, 2)?)
        }),
        c("B identical spheres: subtract gives nothing", Some(0.0), move || {
            subtract(&sphere(&w, 5.0, 1)?, &sphere(&w, 5.0, 2)?)
        }),
        c("B identical spheres, tool frame rotated: intersect", Some(4.0 / 3.0 * PI * 125.0), move || {
            intersect(&sphere(&w, 5.0, 1)?, &sphere(&frame_on(Vec3::ZERO, v3(1.0, 2.0, 3.0).normalized()), 5.0, 2)?)
        }),
        c(
            "B cone inside coaxial cone sharing the base: subtract",
            Some(PI / 3.0 * 25.0 * 10.0 - PI / 3.0 * 25.0 * 5.0),
            move || subtract(&cone_frustum(&w, 5.0, 0.0, 10.0, 1)?, &cone_frustum(&w, 5.0, 0.0, 5.0, 2)?),
        ),
        c(
            "B sphere exactly fills a torus hole, tangent circle: union",
            Some(2.0 * PI * PI * 5.0 * 4.0 + 4.0 / 3.0 * PI * 27.0),
            move || union(&torus(&w, 5.0, 2.0, 1)?, &sphere(&w, 3.0, 2)?),
        ),
        c(
            "B cylinder through torus hole, tangent circle: union",
            Some(2.0 * PI * PI * 5.0 * 4.0 + PI * 9.0 * 20.0),
            move || union(&torus(&w, 5.0, 2.0, 1)?, &cylinder(&zf(0.0, 0.0, -10.0), 3.0, 20.0, 2)?),
        ),
        c("B cone apex touching a plane: union", Some(1000.0 + PI / 3.0 * 9.0 * 5.0), move || {
            union(
                &box_solid(&zf(-5.0, -5.0, -10.0), 10.0, 10.0, 10.0, 1)?,
                &cone_frustum(&zf(0.0, 0.0, 5.0), 3.0, 0.0, 5.0, 2)?.transformed(&Frame::rotation(
                    v3(0.0, 0.0, 5.0),
                    Vec3::X,
                    PI,
                )),
            )
        }),
        // nearly touching: the classic tolerance traps
        c("N gap 1e-3 between boxes: union stays 2 solids", Some(2000.0), move || {
            union(&box_solid(&w, 10.0, 10.0, 10.0, 1)?, &box_solid(&zf(10.001, 0.0, 0.0), 10.0, 10.0, 10.0, 2)?)
        }),
        c("N gap 1e-5 between boxes: union", Some(2000.0), move || {
            union(&box_solid(&w, 10.0, 10.0, 10.0, 1)?, &box_solid(&zf(10.00001, 0.0, 0.0), 10.0, 10.0, 10.0, 2)?)
        }),
        c("N gap 1e-6 (10 TOL) between boxes: union", Some(2000.0), move || {
            union(&box_solid(&w, 10.0, 10.0, 10.0, 1)?, &box_solid(&zf(10.000001, 0.0, 0.0), 10.0, 10.0, 10.0, 2)?)
        }),
        c("N gap 1e-7 (1 TOL) between boxes: union", Some(2000.0), move || {
            union(&box_solid(&w, 10.0, 10.0, 10.0, 1)?, &box_solid(&zf(10.0000001, 0.0, 0.0), 10.0, 10.0, 10.0, 2)?)
        }),
        c("N gap 1e-9 (below TOL) between boxes: union", Some(2000.0), move || {
            union(&box_solid(&w, 10.0, 10.0, 10.0, 1)?, &box_solid(&zf(10.000000001, 0.0, 0.0), 10.0, 10.0, 10.0, 2)?)
        }),
        c("N overlap 1e-5 between boxes: union", Some(2000.0 - 1e-5 * 100.0), move || {
            union(&box_solid(&w, 10.0, 10.0, 10.0, 1)?, &box_solid(&zf(9.99999, 0.0, 0.0), 10.0, 10.0, 10.0, 2)?)
        }),
        c("N overlap 1e-6 sliver: intersect", Some(1e-6 * 100.0), move || {
            intersect(&box_solid(&w, 10.0, 10.0, 10.0, 1)?, &box_solid(&zf(9.999999, 0.0, 0.0), 10.0, 10.0, 10.0, 2)?)
        }),
        c("N tilted 1e-4 rad box top against a box: union", None, move || {
            union(
                &box_solid(&w, 10.0, 10.0, 10.0, 1)?,
                &box_solid(&Frame::rotation(v3(0.0, 0.0, 10.0), Vec3::X, 1e-4), 10.0, 10.0, 5.0, 2)?,
            )
        }),
        c(
            "N cylinder radius 5.00001 over r=5: subtract thin shell",
            Some(PI * (5.00001f64 * 5.00001 - 25.0) * 10.0),
            move || subtract(&cylinder(&w, 5.00001, 10.0, 1)?, &cylinder(&zf(0.0, 0.0, -1.0), 5.0, 12.0, 2)?),
        ),
        c("N cylinder r=5 axis offset 1e-5: subtract sliver", None, move || {
            subtract(&cylinder(&w, 5.0, 10.0, 1)?, &cylinder(&zf(1e-5, 0.0, -1.0), 5.0, 12.0, 2)?)
        }),
        // scale: the tolerance is absolute
        c("S box-cylinder 1e-3 size (micro)", Some(1e-9 - PI * 0.09e-6 * 1e-3), move || {
            subtract(
                &box_solid(&zf(-5e-4, -5e-4, 0.0), 1e-3, 1e-3, 1e-3, 1)?,
                &cylinder(&zf(0.0, 0.0, -1e-3), 3e-4, 3e-3, 2)?,
            )
        }),
        c("S box-cylinder 1e-2 size", Some(1e-6 - PI * 9e-6 * 1e-2), move || {
            subtract(
                &box_solid(&zf(-5e-3, -5e-3, 0.0), 1e-2, 1e-2, 1e-2, 1)?,
                &cylinder(&zf(0.0, 0.0, -1e-2), 3e-3, 3e-2, 2)?,
            )
        }),
        c("S box-cylinder 1e4 size", Some(1e12 - PI * 9e6 * 1e4), move || {
            subtract(&box_solid(&zf(-5e3, -5e3, 0.0), 1e4, 1e4, 1e4, 1)?, &cylinder(&zf(0.0, 0.0, -1e4), 3e3, 3e4, 2)?)
        }),
        c("S box-cylinder 1e6 size (1 km in mm)", Some(1e18 - PI * 9e10 * 1e6), move || {
            subtract(&box_solid(&zf(-5e5, -5e5, 0.0), 1e6, 1e6, 1e6, 1)?, &cylinder(&zf(0.0, 0.0, -1e6), 3e5, 3e6, 2)?)
        }),
        c("S sphere-sphere 1e6 size", None, move || union(&sphere(&w, 1e6, 1)?, &sphere(&zf(1e6, 0.0, 0.0), 1e6, 2)?)),
        // ------- Fillets (compared against OCCT by name where there is no formula)
        c("F all 12 edges of a box r=3", Some(rb(40.0, 30.0, 20.0, 3.0)), move || {
            let b = box_solid(&w, 40.0, 30.0, 20.0, 1)?;
            let e: Vec<usize> = (0..b.edges.len()).collect();
            fillet(&b, &e, 3.0, 2)
        }),
        c("F top 4 edges of a box r=3", None, move || {
            let b = box_solid(&w, 40.0, 30.0, 20.0, 1)?;
            fillet(&b, &top_edges(&b, 20.0), 3.0, 2)
        }),
        c("F rounded plate top rim r=3", None, move || {
            let b = rounded()?;
            fillet(&b, &top_edges(&b, 12.0), 3.0, 2)
        }),
        c("F rounded plate both rims r=3", None, move || {
            let b = rounded()?;
            let f = fillet(&b, &top_edges(&b, 12.0), 3.0, 2)?;
            fillet(&f, &top_edges(&f, 0.0), 3.0, 3)
        }),
        c("F rim r == corner radius r=8 (torus degenerates)", None, move || {
            let b = rounded()?;
            fillet(&b, &top_edges(&b, 12.0), 8.0, 2)
        }),
        c("F rim r > corner radius r=9", None, move || {
            let b = extrude(&Profile::new(Wire::rounded_rect(60.0, 40.0, 8.0)), &w, 20.0, 1)?;
            fillet(&b, &top_edges(&b, 20.0), 9.0, 2)
        }),
        c("F rim r == thickness (no side face left) r=12", None, move || {
            let b = extrude(&Profile::new(Wire::rounded_rect(60.0, 40.0, 14.0)), &w, 12.0, 1)?;
            fillet(&b, &top_edges(&b, 12.0), 12.0, 2)
        }),
        c("F boss foot r=3 and boss top r=2", None, move || {
            let p = box_solid(&zf(-30.0, -30.0, 0.0), 60.0, 60.0, 8.0, 1)?;
            let u = union(&p, &cylinder(&zf(0.0, 0.0, 8.0), 10.0, 20.0, 2)?)?;
            let f = fillet(
                &u,
                &top_edges(&u, 8.0)
                    .into_iter()
                    .filter(|&i| matches!(u.edges[i].curve, Curve::Circle { .. }))
                    .collect::<Vec<_>>(),
                3.0,
                3,
            )?;
            fillet(&f, &top_edges(&f, 28.0), 2.0, 4)
        }),
        c("F through-hole rims r=2", None, move || {
            let p = Profile::new(Wire::rect(0.0, 0.0, 60.0, 40.0)).with_hole(Wire::circle(v2(30.0, 20.0), 6.0));
            let b = extrude(&p, &w, 10.0, 1)?;
            let e: Vec<usize> = b.edges_where(|i, ed| {
                matches!(ed.curve, Curve::Circle { .. }) && {
                    let z = b.edge_midpoint(i).z;
                    z.abs() < 1e-6 || (z - 10.0).abs() < 1e-6
                }
            });
            fillet(&b, &e, 2.0, 2)
        }),
        c("F L bracket: inner r=4, outer r=8, then rim chain r=2", None, move || {
            let wire = Wire::polygon(&[
                v2(0.0, 0.0),
                v2(50.0, 0.0),
                v2(50.0, 10.0),
                v2(10.0, 10.0),
                v2(10.0, 50.0),
                v2(0.0, 50.0),
            ]);
            let b = extrude(&Profile::new(wire), &w, 30.0, 1)?;
            let inner: Vec<usize> = b.edges_where(|i, _| {
                let m = b.edge_midpoint(i);
                (m.x - 10.0).abs() < 1e-6 && (m.y - 10.0).abs() < 1e-6
            });
            let f = fillet(&b, &inner, 4.0, 2)?;
            let outer: Vec<usize> = f.edges_where(|i, _| {
                let m = f.edge_midpoint(i);
                m.x.abs() < 1e-6 && m.y.abs() < 1e-6
            });
            let g = fillet(&f, &outer, 8.0, 3)?;
            fillet(&g, &top_edges(&g, 30.0), 2.0, 4)
        }),
        c("F pocket floor (concave chain) r=2", None, move || {
            let b = box_solid(&w, 80.0, 60.0, 20.0, 1)?;
            let tool = extrude(
                &Profile::new(Wire::rounded_rect(50.0, 30.0, 6.0).translated(v2(40.0, 30.0))),
                &zf(0.0, 0.0, 8.0),
                20.0,
                2,
            )?;
            let p = subtract(&b, &tool)?;
            fillet(&p, &top_edges(&p, 8.0), 2.0, 3)
        }),
        c("F slot outline rim r=2", None, move || {
            let wire = PathBuilder::new(v2(-20.0, -8.0))
                .line_to(v2(20.0, -8.0))
                .arc_to(v2(20.0, 8.0), v2(20.0, 0.0), true)
                .line_to(v2(-20.0, 8.0))
                .arc_to(v2(-20.0, -8.0), v2(-20.0, 0.0), true)
                .close();
            let b = extrude(&Profile::new(wire), &w, 10.0, 1)?;
            fillet(&b, &top_edges(&b, 10.0), 2.0, 2)
        }),
        c("F cone frustum top rim r=1", None, move || {
            let b = cone_frustum(&w, 10.0, 6.0, 12.0, 1)?;
            fillet(&b, &top_edges(&b, 12.0), 1.0, 2)
        }),
        c("F dome foot: sphere on a plate r=1.5", None, move || {
            let p = box_solid(&zf(-30.0, -30.0, 0.0), 60.0, 60.0, 8.0, 1)?;
            let u = union(&p, &sphere(&zf(0.0, 0.0, 8.0), 15.0, 2)?)?;
            let e: Vec<usize> =
                top_edges(&u, 8.0).into_iter().filter(|&i| matches!(u.edges[i].curve, Curve::Circle { .. })).collect();
            fillet(&u, &e, 1.5, 3)
        }),
        c("F neighbouring edges with different radii 2 then 4", None, move || {
            let b = box_solid(&w, 40.0, 30.0, 20.0, 1)?;
            let e1: Vec<usize> = b.edges_where(|i, _| {
                let m = b.edge_midpoint(i);
                (m.z - 20.0).abs() < 1e-6 && m.y.abs() < 1e-6
            });
            let f = fillet(&b, &e1, 2.0, 2)?;
            let e2: Vec<usize> = f.edges_where(|i, _| {
                let m = f.edge_midpoint(i);
                (m.z - 20.0).abs() < 1e-6 && m.x.abs() < 1e-6
            });
            fillet(&f, &e2, 4.0, 3)
        }),
        c("F elliptical rim: cylinder cut by a tilted plane r=1", None, move || {
            let cyl = cylinder(&w, 10.0, 40.0, 1)?;
            let cutter = box_solid(
                &Frame::rotation(v3(0.0, 0.0, 25.0), Vec3::X, 0.5).compose(&zf(-50.0, -50.0, 0.0)),
                100.0,
                100.0,
                50.0,
                2,
            )?;
            let c2 = subtract(&cyl, &cutter)?;
            let e: Vec<usize> = c2.edges_where(|_, ed| {
                matches!(ed.curve, Curve::Circle { .. }) == false && !matches!(ed.curve, Curve::Line { .. })
            });
            fillet(&c2, &e, 1.0, 3)
        }),
        c("F tee pipe: cylinder-cylinder saddle edge r=1 (unequal radii)", None, move || {
            let main = cylinder(&xf(-30.0, 0.0, 0.0), 10.0, 60.0, 1)?;
            let branch = cylinder(&zf(0.0, 0.0, 0.0), 5.0, 30.0, 2)?;
            let u = union(&main, &branch)?;
            let e: Vec<usize> = u.edges_where(|_, ed| !matches!(ed.curve, Curve::Circle { .. } | Curve::Line { .. }));
            fillet(&u, &e, 1.0, 3)
        }),
        c("F chamfer all top edges and hole of a rounded plate", None, move || {
            let p = Profile::new(Wire::rounded_rect(60.0, 40.0, 8.0)).with_hole(Wire::circle(v2(0.0, 0.0), 6.0));
            let b = extrude(&p, &w, 12.0, 1)?;
            chamfer(&b, &top_edges(&b, 12.0), 1.5, 2)
        }),
        c("F thin plate r bigger than thickness r=3 t=2 (expect refusal)", None, move || {
            let b = box_solid(&w, 40.0, 30.0, 2.0, 1)?;
            let e: Vec<usize> = b.edges_where(|i, _| {
                let m = b.edge_midpoint(i);
                (m.z - 2.0).abs() < 1e-6 && m.y.abs() < 1e-6
            });
            fillet(&b, &e, 3.0, 2)
        }),
        // ------- builders
        c("X revolve: profile touching the axis (solid of revolution)", Some(PI * 25.0 * 10.0), move || {
            revolve(&Profile::new(Wire::rect(0.0, 0.0, 5.0, 10.0)), &w, TAU, 1)
        }),
        c("X revolve 90 degrees of a ring section", Some(PI / 2.0 * (64.0 - 16.0) * 10.0), move || {
            revolve(&Profile::new(Wire::rect(4.0, 0.0, 4.0, 10.0)), &w, PI / 2.0, 1)
        }),
        c("X self-intersecting bow tie profile (expect refusal)", None, move || {
            extrude(
                &Profile::new(Wire::polygon(&[v2(0.0, 0.0), v2(10.0, 10.0), v2(10.0, 0.0), v2(0.0, 10.0)])),
                &w,
                5.0,
                1,
            )
        }),
        c("X hole outside the outer profile (expect refusal)", None, move || {
            extrude(
                &Profile::new(Wire::rect(0.0, 0.0, 10.0, 10.0)).with_hole(Wire::circle(v2(50.0, 50.0), 1.0)),
                &w,
                5.0,
                1,
            )
        }),
        c("X revolve crossing its axis (expect refusal)", None, move || {
            revolve(&Profile::new(Wire::rect(-2.0, 0.0, 5.0, 10.0)), &w, TAU, 1)
        }),
    ]
}

fn run_named(path: &str) {
    let mut out = std::fs::File::create(path).unwrap();
    for (name, expected, f) in named() {
        let run = guarded(60.0, f);
        let (status, vol, ms, faces, open, msg) = match run {
            Run::Ok(b, ms) => {
                let v = mass_properties(&b).volume;
                let m = tessellate(&b, &TessParams::for_size(b.aabb().diagonal().max(1e-9)));
                let open = m.as_ref().map(|x| x.open_edge_count() as i64).unwrap_or(-1);
                ("ok", v, ms, b.faces.len(), open, String::new())
            }
            Run::Err(e, ms) => ("refused", f64::NAN, ms, 0, 0, clean(&e)),
            Run::Panic(ms) => ("PANIC", f64::NAN, ms, 0, 0, String::new()),
            Run::Hang => ("HANG", f64::NAN, 60000.0, 0, 0, String::new()),
        };
        let exp = expected.map_or("null".to_string(), |e| format!("{e:.12e}"));
        let vs = if vol.is_nan() { "null".to_string() } else { format!("{vol:.12e}") };
        writeln!(out, "{{\"name\":\"{name}\",\"s\":\"{status}\",\"v\":{vs},\"exp\":{exp},\"ms\":{ms:.2},\"faces\":{faces},\"open\":{open},\"msg\":\"{msg}\"}}").unwrap();
    }
}

fn perf() {
    let w = Frame::WORLD;
    println!("-- plate with n x n holes, one Boolean at a time");
    for n in [5, 10, 20] {
        let t = Instant::now();
        let mut b = box_solid(&w, 400.0, 400.0, 10.0, 1).unwrap();
        let mut last = t.elapsed().as_secs_f64();
        let mut failed = None;
        'o: for i in 0..n {
            for j in 0..n {
                let s = 380.0 / n as f64;
                let tool = cylinder(
                    &Frame::translation(v3(10.0 + s * (i as f64 + 0.5), 10.0 + s * (j as f64 + 0.5), -1.0)),
                    s * 0.3,
                    12.0,
                    2 + (i * n + j) as u32,
                )
                .unwrap();
                match subtract(&b, &tool) {
                    Ok(r) => b = r,
                    Err(e) => {
                        failed = Some(format!("{i},{j}: {e}"));
                        break 'o;
                    }
                }
                let now = t.elapsed().as_secs_f64();
                last = now - 0.0 * last;
            }
        }
        let tt = Instant::now();
        let m = tessellate(&b, &TessParams::for_size(b.aabb().diagonal())).unwrap();
        println!("n={n:>3} holes={:>4} total {:>8.3}s  last-op-time n/a  faces {:>5} vol {:.3} expected {:.3} tess {:.3}s tris {} open {} {:?}", n * n, t.elapsed().as_secs_f64() - tt.elapsed().as_secs_f64(), b.faces.len(), mass_properties(&b).volume, 400.0 * 400.0 * 10.0 - (n * n) as f64 * PI * (380.0 / n as f64 * 0.3).powi(2) * 10.0, tt.elapsed().as_secs_f64(), m.triangles.len(), m.open_edge_count(), failed);
    }
    println!("-- same holes in one profile");
    for n in [20, 50] {
        let t = Instant::now();
        let mut p = Profile::new(Wire::rect(0.0, 0.0, 400.0, 400.0));
        for i in 0..n {
            for j in 0..n {
                let s = 380.0 / n as f64;
                p = p.with_hole(Wire::circle(v2(10.0 + s * (i as f64 + 0.5), 10.0 + s * (j as f64 + 0.5)), s * 0.3));
            }
        }
        let b = extrude(&p, &w, 10.0, 1);
        println!(
            "n={n} holes={} extrude {:.3}s {:?}",
            n * n,
            t.elapsed().as_secs_f64(),
            b.as_ref().map(|b| b.faces.len()).map_err(|e| e.to_string())
        );
    }
    println!("-- gear: 60 cuts around a cylinder");
    let t = Instant::now();
    let mut g = cylinder(&w, 50.0, 10.0, 1).unwrap();
    let mut ok = 0;
    for k in 0..60 {
        let a = k as f64 / 60.0 * TAU;
        let f = Frame::rotation(Vec3::ZERO, Vec3::Z, a).compose(&Frame::translation(v3(46.0, -1.5, -1.0)));
        match subtract(&g, &box_solid(&f, 10.0, 3.0, 12.0, 2 + k).unwrap()) {
            Ok(r) => {
                g = r;
                ok += 1
            }
            Err(e) => {
                println!("cut {k}: {e}");
                break;
            }
        }
    }
    println!(
        "{ok} cuts, {:.3}s, faces {}, volume {:.3}",
        t.elapsed().as_secs_f64(),
        g.faces.len(),
        mass_properties(&g).volume
    );
}

/// Random parts built in long feature chains, every step written as STEP for OCCT.
fn chain(dir: &str, first: u64, count: u64) {
    std::fs::create_dir_all(dir).unwrap();
    let mut log = std::fs::File::create(format!("{dir}/chain.jsonl")).unwrap();
    for seq in first..first + count {
        let mut rng = Rng::new(seq.wrapping_mul(0x9E3779B97F4A7C15) ^ 0x51ED);
        let (w, d, h) =
            (60.0 + 10.0 * rng.int(0, 4) as f64, 40.0 + 10.0 * rng.int(0, 3) as f64, 10.0 + 2.0 * rng.int(0, 5) as f64);
        let base = if rng.int(0, 1) == 0 {
            box_solid(&Frame::translation(v3(-w / 2.0, -d / 2.0, 0.0)), w, d, h, 1)
        } else {
            extrude(&Profile::new(Wire::rounded_rect(w, d, 6.0)), &Frame::WORLD, h, 1)
        };
        let mut body = base.unwrap();
        let step = |b: &Body, i: usize| {
            std::fs::write(format!("{dir}/s{seq}_{i}.step"), coreshape::export::step::write_step(b, "p")).unwrap();
        };
        step(&body, 0);
        writeln!(
            log,
            "{{\"seq\":{seq},\"i\":0,\"op\":\"base\",\"s\":\"ok\",\"v\":{:.10e},\"faces\":{}}}",
            mass_properties(&body).volume,
            body.faces.len()
        )
        .unwrap();
        let mut done = 0usize;
        for i in 1..=12usize {
            let g = |rng: &mut Rng, a: i64, b: i64| rng.int(a, b) as f64;
            let top = body.aabb().max.z;
            let kind = rng.int(0, 7);
            let (hw, hd) = (w / 2.0, d / 2.0);
            let (px, py) = (
                g(&mut rng, (-hw) as i64 / 2, hw as i64 / 2) * 2.0,
                g(&mut rng, (-hd) as i64 / 2, hd as i64 / 2) * 2.0,
            );
            let (name, prim, boolop, blend): (&str, Option<Prim>, i32, Option<(bool, f64)>) = match kind {
                0 | 1 => (
                    "hole z",
                    Some(Prim::Cyl(Frame::translation(v3(px, py, -5.0)), g(&mut rng, 2, 6), top + 10.0)),
                    1,
                    None,
                ),
                2 => (
                    "hole x",
                    Some(Prim::Cyl(
                        frame_on(v3(-hw - 5.0, py, 2.0 + g(&mut rng, 1, 3) * 2.0), Vec3::X),
                        g(&mut rng, 1, 3),
                        w + 10.0,
                    )),
                    1,
                    None,
                ),
                3 => (
                    "pocket",
                    Some(Prim::Box(
                        Frame::translation(v3(px, py, top - g(&mut rng, 2, 6))),
                        g(&mut rng, 6, 24),
                        g(&mut rng, 6, 16),
                        30.0,
                    )),
                    1,
                    None,
                ),
                4 => (
                    "boss",
                    Some(Prim::Cyl(Frame::translation(v3(px, py, top - 1.0)), g(&mut rng, 3, 8), g(&mut rng, 4, 16))),
                    0,
                    None,
                ),
                5 => (
                    "rib",
                    Some(Prim::Box(
                        Frame::translation(v3(px, py, top - 1.0)),
                        g(&mut rng, 4, 20),
                        g(&mut rng, 2, 6),
                        g(&mut rng, 4, 12),
                    )),
                    0,
                    None,
                ),
                6 => ("fillet", None, 2, Some((true, [0.5, 1.0, 1.5, 2.0][rng.int(0, 3) as usize]))),
                _ => ("chamfer", None, 2, Some((false, [0.5, 1.0, 1.5][rng.int(0, 2) as usize]))),
            };
            let mut spec = String::from("null");
            let before = body.clone();
            let run = if let Some(p) = prim.clone() {
                spec = pj(&p);
                guarded(30.0, move || {
                    let t = build(&p)?;
                    if boolop == 1 {
                        subtract(&before, &t)
                    } else {
                        union(&before, &t)
                    }
                })
            } else {
                let (is_fillet, r) = blend.unwrap();
                let cand: Vec<usize> =
                    body.edges_where(|_, e| matches!(e.curve, Curve::Line { .. } | Curve::Circle { .. }));
                if cand.is_empty() {
                    continue;
                }
                let e = cand[rng.int(0, cand.len() as i64 - 1) as usize];
                let f = 100 + i as u32;
                guarded(
                    30.0,
                    move || if is_fillet { fillet(&before, &[e], r, f) } else { chamfer(&before, &[e], r, f) },
                )
            };
            match run {
                Run::Ok(b, ms) => {
                    let mesh = tessellate(&b, &TessParams::for_size(b.aabb().diagonal())).map(|m| m.open_edge_count() as i64).unwrap_or(-1);
                    done += 1;
                    body = b;
                    step(&body, done);
                    if std::env::var("REVIEW_DUMP").ok().as_deref() == Some(&format!("{seq}:{done}")) {
                        let c = body.aabb().center();
                        let mut text = format!("center {} {} {}\n", c.x, c.y, c.z);
                        for (k, (v, a)) in coreshape::mass::per_face(&body, c).iter().enumerate() {
                            let f = &body.faces[k];
                            let _ = writeln!(text, "{k} {} {a:.9} flux {v:.9} loops {:?} edges {:?}", f.surface.name(), f.loops.iter().map(|l| l.len()).collect::<Vec<_>>(), f.loops.iter().flatten().map(|c| match body.edges[c.edge].curve { Curve::Line { .. } => 'L', Curve::Circle { .. } => 'C', Curve::Ellipse { .. } => 'E', Curve::Sampled(_) => 'S' }).collect::<String>());
                        }
                        let fine = tessellate(&body, &TessParams { chord: 0.002, angle: 2f64.to_radians() }).unwrap();
                        let _ = writeln!(text, "exact volume {} fine mesh volume {} open {}", mass_properties(&body).volume, fine.volume(), fine.open_edge_count());
                        for (k, f) in body.faces.iter().enumerate() {
                            for (l, lp) in f.loops.iter().enumerate() {
                                let closure = coreshape::mass::loop_closure_error(&body, lp);
                                if closure > 1e-9 {
                                    let _ = writeln!(text, "closure face {k} loop {l} {} error {closure:e}", f.surface.name());
                                }
                            }
                        }
                        for (i, e) in body.edges.iter().enumerate() {
                            if let Curve::Sampled(c) = &e.curve {
                                let n = 200;
                                let mut off = 0.0f64;
                                let mut length = 0.0;
                                let mut prev = e.curve.eval(e.t0);
                                for k in 1..=n {
                                    let p = e.curve.eval(e.t0 + (e.t1 - e.t0) * k as f64 / n as f64);
                                    off = off.max(c.surfaces.0.sdf(p).abs()).max(c.surfaces.1.sdf(p).abs());
                                    length += p.dist(prev);
                                    prev = p;
                                }
                                let g = (e.t0..e.t1).contains(&0.0);
                                let _ = writeln!(text, "sampled edge {i}: t {:.4}..{:.4} of {} samples, length {length:.5}, off surface {off:.1e}, ends {:.1e} {:.1e} starts at knot {g}", e.t0, e.t1, c.points.len(), e.curve.eval(e.t0).dist(body.vertices[e.v0]), e.curve.eval(e.t1).dist(body.vertices[e.v1]));
                            }
                        }
                        std::fs::write(format!("{dir}/faces.txt"), text).unwrap();
                    }
                    writeln!(log, "{{\"seq\":{seq},\"i\":{done},\"op\":\"{name}\",\"s\":\"ok\",\"bool\":{boolop},\"prim\":{spec},\"v\":{:.10e},\"faces\":{},\"ms\":{ms:.1},\"open\":{mesh},\"shells\":{}}}", mass_properties(&body).volume, body.faces.len(), body.shells.len()).unwrap();
                }
                Run::Err(e, ms) => writeln!(log, "{{\"seq\":{seq},\"i\":{done},\"op\":\"{name}\",\"s\":\"refused\",\"bool\":{boolop},\"prim\":{spec},\"msg\":\"{}\",\"ms\":{ms:.1}}}", clean(&e)).unwrap(),
                Run::Panic(_) => writeln!(log, "{{\"seq\":{seq},\"i\":{done},\"op\":\"{name}\",\"s\":\"PANIC\",\"bool\":{boolop},\"prim\":{spec}}}").unwrap(),
                Run::Hang => writeln!(log, "{{\"seq\":{seq},\"i\":{done},\"op\":\"{name}\",\"s\":\"HANG\",\"bool\":{boolop},\"prim\":{spec}}}").unwrap(),
            }
        }
    }
}

fn extra() {
    let w = Frame::WORLD;
    // the tilted case again, with the box really stacked on top
    let tilted = Frame::translation(v3(0.0, 0.0, 10.0)).compose(&Frame::rotation(Vec3::ZERO, Vec3::X, 1e-4));
    let r = union(&box_solid(&w, 10.0, 10.0, 10.0, 1).unwrap(), &box_solid(&tilted, 10.0, 10.0, 5.0, 2).unwrap());
    println!(
        "tilted 1e-4 stacked: {:?}",
        r.as_ref().map(|b| (mass_properties(b).volume, b.faces.len())).map_err(|e| e.to_string())
    );
    let tilted = Frame::translation(v3(0.0, 0.0, 10.0)).compose(&Frame::rotation(Vec3::ZERO, Vec3::X, 1e-7));
    let r = union(&box_solid(&w, 10.0, 10.0, 10.0, 1).unwrap(), &box_solid(&tilted, 10.0, 10.0, 5.0, 2).unwrap());
    println!(
        "tilted 1e-7 stacked: {:?}",
        r.as_ref().map(|b| (mass_properties(b).volume, b.faces.len())).map_err(|e| e.to_string())
    );
    // one wrong-seed geometry, with a much finer check of the mass integration: the mesh
    let a = cone_frustum(&Frame::new(v3(0.0, 1.0, 2.0), v3(0.0, 0.0, 1.0), Vec3::X), 2.0, 0.0, 4.0, 1).unwrap();
    let b = cylinder(&Frame::translation(v3(0.0, 2.0, 2.0)), 1.0, 4.0, 2).unwrap();
    for (n, r) in [("union", union(&a, &b)), ("cut", subtract(&a, &b)), ("common", intersect(&a, &b))] {
        match r {
            Ok(x) => println!(
                "seed138 {n}: exact-integral {:.9} faces {} valid {:?}",
                mass_properties(&x).volume,
                x.faces.len(),
                validate(&x).is_ok()
            ),
            Err(e) => println!("seed138 {n}: refused {e}"),
        }
    }
    println!("-- same holes in one profile");
    for n in [20, 50] {
        let t = Instant::now();
        let mut p = Profile::new(Wire::rect(0.0, 0.0, 400.0, 400.0));
        for i in 0..n {
            for j in 0..n {
                let s = 380.0 / n as f64;
                p = p.with_hole(Wire::circle(v2(10.0 + s * (i as f64 + 0.5), 10.0 + s * (j as f64 + 0.5)), s * 0.3));
            }
        }
        let b = extrude(&p, &w, 10.0, 1);
        println!(
            "n={n} holes={} extrude {:.3}s {:?}",
            n * n,
            t.elapsed().as_secs_f64(),
            b.as_ref().map(|b| b.faces.len()).map_err(|e| e.to_string())
        );
    }
    println!("-- gear: 60 cuts around a cylinder");
    let t = Instant::now();
    let mut g = cylinder(&w, 50.0, 10.0, 1).unwrap();
    let mut ok = 0;
    for k in 0..60 {
        let a = k as f64 / 60.0 * TAU;
        let f = Frame::rotation(Vec3::ZERO, Vec3::Z, a).compose(&Frame::translation(v3(46.0, -1.5, -1.0)));
        match subtract(&g, &box_solid(&f, 10.0, 3.0, 12.0, 2 + k).unwrap()) {
            Ok(r) => {
                g = r;
                ok += 1
            }
            Err(e) => {
                println!("cut {k}: {e}");
                break;
            }
        }
    }
    println!(
        "{ok} cuts, {:.3}s, faces {}, volume {:.3}",
        t.elapsed().as_secs_f64(),
        g.faces.len(),
        mass_properties(&g).volume
    );
}

fn main() {
    std::panic::set_hook(Box::new(|_| {}));
    let a: Vec<String> = std::env::args().collect();
    match a[1].as_str() {
        "diff" => differential(&a[2], a[3].parse().unwrap(), a[4].parse().unwrap(), a[5].parse().unwrap()),
        "named" => run_named(&a[2]),
        "perf" => perf(),
        "extra" => extra(),
        "chain" => chain(&a[2], a[3].parse().unwrap(), a[4].parse().unwrap()),
        // One perf case at a time, for profiling.
        "holes" => {
            let n: usize = a[2].parse().unwrap();
            let t = Instant::now();
            let mut b = box_solid(&Frame::WORLD, 400.0, 400.0, 10.0, 1).unwrap();
            for i in 0..n {
                for j in 0..n {
                    let s = 380.0 / n as f64;
                    let tool = cylinder(
                        &Frame::translation(v3(10.0 + s * (i as f64 + 0.5), 10.0 + s * (j as f64 + 0.5), -1.0)),
                        s * 0.3,
                        12.0,
                        2 + (i * n + j) as u32,
                    )
                    .unwrap();
                    b = subtract(&b, &tool).unwrap();
                }
            }
            let expected = 400.0 * 400.0 * 10.0 - (n * n) as f64 * PI * (380.0 / n as f64 * 0.3).powi(2) * 10.0;
            println!(
                "{} holes one at a time: {:.3}s, {} faces, volume {:.3} expected {expected:.3}",
                n * n,
                t.elapsed().as_secs_f64(),
                b.faces.len(),
                mass_properties(&b).volume
            );
        }
        "extrude" => {
            let n: usize = a[2].parse().unwrap();
            let t = Instant::now();
            let mut p = Profile::new(Wire::rect(0.0, 0.0, 400.0, 400.0));
            for i in 0..n {
                for j in 0..n {
                    let s = 380.0 / n as f64;
                    p = p
                        .with_hole(Wire::circle(v2(10.0 + s * (i as f64 + 0.5), 10.0 + s * (j as f64 + 0.5)), s * 0.3));
                }
            }
            let b = extrude(&p, &Frame::WORLD, 10.0, 1).unwrap();
            println!("{} holes in one extrude: {:.3}s, {} faces", n * n, t.elapsed().as_secs_f64(), b.faces.len());
            let t = Instant::now();
            let m = tessellate(&b, &TessParams::for_size(b.aabb().diagonal())).unwrap();
            println!(
                "mesh {:.3}s, {} triangles, {} open",
                t.elapsed().as_secs_f64(),
                m.triangles.len(),
                m.open_edge_count()
            );
        }
        "gear" => {
            let w = Frame::WORLD;
            let t = Instant::now();
            let mut g = cylinder(&w, 50.0, 10.0, 1).unwrap();
            let mut ok = 0;
            for k in 0..60 {
                let a = k as f64 / 60.0 * TAU;
                let f = Frame::rotation(Vec3::ZERO, Vec3::Z, a).compose(&Frame::translation(v3(46.0, -1.5, -1.0)));
                match subtract(&g, &box_solid(&f, 10.0, 3.0, 12.0, 2 + k).unwrap()) {
                    Ok(r) => {
                        g = r;
                        ok += 1
                    }
                    Err(e) => {
                        println!("cut {k}: {e}");
                        break;
                    }
                }
            }
            println!(
                "{ok} cuts, {:.3}s, faces {}, volume {:.3}",
                t.elapsed().as_secs_f64(),
                g.faces.len(),
                mass_properties(&g).volume
            );
        }
        _ => {}
    }
}
