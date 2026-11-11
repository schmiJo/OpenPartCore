//! Booleans with many tools at once, mirroring, and sweeps along lines and arcs.
//!
//! Applying a hundred tools one by one rebuilds the target a hundred times. Tools whose boxes
//! don't touch can't interact, so they go together as separate shells of one body, and the
//! Boolean only has to run once. Tools that do overlap are joined first.

use crate::boolean::{boolean_with, BoolOp, Checks};
use crate::build::{extrude, revolve};
use crate::error::{Error, Result};
use crate::math::*;
use crate::profile::{Profile, Seg, SegGeom, Wire};
use crate::topo::Body;

/// Several bodies as one, each as its own shells. They must not touch: nothing checks that here.
pub fn compound(bodies: &[Body]) -> Body {
    let mut out = Body::default();
    for b in bodies {
        let (vs, es, fs) = (out.vertices.len(), out.edges.len(), out.faces.len());
        out.vertices.extend_from_slice(&b.vertices);
        for e in &b.edges {
            let mut e = e.clone();
            e.v0 += vs;
            e.v1 += vs;
            out.edges.push(e);
        }
        for f in &b.faces {
            let mut f = f.clone();
            for lp in &mut f.loops {
                for c in lp {
                    c.edge += es;
                }
            }
            out.faces.push(f);
        }
        out.shells.extend(b.shells.iter().map(|s| s.iter().map(|f| f + fs).collect::<Vec<usize>>()));
    }
    out
}

/// Joins tools into one body: those that may overlap by a real union, the rest side by side.
pub fn merged_tools(tools: &[Body]) -> Result<Body> {
    let n = tools.len();
    if n == 0 {
        return Err(Error::InvalidInput("there are no tools".into()));
    }
    // Clusters of boxes that overlap, by a sweep along x.
    let boxes: Vec<Aabb> = tools.iter().map(|t| t.aabb().inflated(4.0 * TOL)).collect();
    let mut parent: Vec<usize> = (0..n).collect();
    fn find(parent: &mut [usize], mut v: usize) -> usize {
        while parent[v] != v {
            parent[v] = parent[parent[v]];
            v = parent[v];
        }
        v
    }
    let mut order: Vec<usize> = (0..n).collect();
    order.sort_by(|&a, &b| boxes[a].min.x.total_cmp(&boxes[b].min.x));
    let mut active: Vec<usize> = Vec::new();
    for &i in &order {
        active.retain(|&j| boxes[j].max.x >= boxes[i].min.x);
        for &j in &active {
            let (a, b) = (&boxes[i], &boxes[j]);
            if a.min.y <= b.max.y && b.min.y <= a.max.y && a.min.z <= b.max.z && b.min.z <= a.max.z {
                let (x, y) = (find(&mut parent, i), find(&mut parent, j));
                parent[x] = y;
            }
        }
        active.push(i);
    }
    let mut clusters: Vec<Vec<usize>> = Vec::new();
    let mut cluster_of = vec![usize::MAX; n];
    for i in 0..n {
        let root = find(&mut parent, i);
        if cluster_of[root] == usize::MAX {
            cluster_of[root] = clusters.len();
            clusters.push(Vec::new());
        }
        clusters[cluster_of[root]].push(i);
    }
    let mut joined: Vec<Body> = Vec::with_capacity(clusters.len());
    for cluster in clusters {
        let mut current = tools[cluster[0]].clone();
        for &t in &cluster[1..] {
            current = boolean_with(&current, &tools[t], BoolOp::Union, Checks::Interim)?;
        }
        joined.push(current);
    }
    if joined.len() == 1 {
        return Ok(joined.pop().unwrap());
    }
    Ok(compound(&joined))
}

/// Applies many tools to a target at once. `Union` adds them all, `Subtract` removes them all, and
/// `Intersect` keeps what lies inside any of them.
pub fn boolean_many(target: &Body, tools: &[Body], op: BoolOp) -> Result<Body> {
    boolean_many_with(target, tools, op, Checks::Full)
}

/// As `boolean_many`, with control over how much the result is checked.
pub fn boolean_many_with(target: &Body, tools: &[Body], op: BoolOp, checks: Checks) -> Result<Body> {
    if tools.is_empty() {
        return Ok(target.clone());
    }
    if op == BoolOp::Intersect {
        let tool = merged_tools(tools)?;
        return boolean_with(target, &tool, op, checks);
    }
    // Adding or removing tools can go in any order, so each round takes as many as don't touch each
    // other, as separate shells of one body. A rounded box's 20 tools need about 5 rounds this way.
    let rounds = rounds_of(tools);
    let total = rounds.len();
    let mut current = target.clone();
    for (k, round) in rounds.into_iter().enumerate() {
        let tool = if round.len() == 1 {
            tools[round[0]].clone()
        } else {
            compound(&round.iter().map(|&t| tools[t].clone()).collect::<Vec<Body>>())
        };
        let level = if k + 1 == total { checks } else { Checks::Interim };
        current = boolean_with(&current, &tool, op, level)?;
    }
    Ok(current)
}

/// Sorts tools into rounds whose members' boxes don't touch. Greedy, biggest first: any answer is
/// correct, and fewer rounds is only faster.
fn rounds_of(tools: &[Body]) -> Vec<Vec<usize>> {
    let boxes: Vec<Aabb> = tools.iter().map(|t| t.aabb().inflated(4.0 * TOL)).collect();
    let touching = |a: &Aabb, b: &Aabb| {
        a.min.x <= b.max.x && b.min.x <= a.max.x && a.min.y <= b.max.y && b.min.y <= a.max.y && a.min.z <= b.max.z && b.min.z <= a.max.z
    };
    let mut order: Vec<usize> = (0..tools.len()).collect();
    order.sort_by(|&a, &b| boxes[b].diagonal().total_cmp(&boxes[a].diagonal()));
    let mut rounds: Vec<Vec<usize>> = Vec::new();
    for t in order {
        match rounds.iter().position(|r| r.iter().all(|&o| !touching(&boxes[t], &boxes[o]))) {
            Some(r) => rounds[r].push(t),
            None => rounds.push(vec![t]),
        }
    }
    for r in &mut rounds {
        r.sort_unstable();
    }
    rounds
}

/// One step of a sweep. Each carries on from the direction the last one ended in, so a path is
/// always smooth.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PathStep {
    /// Straight ahead for a distance.
    Line { length: f64 },
    /// Bends through `angle` radians around a center `radius` away from the path. `toward` is the
    /// direction of that center, in radians counter-clockwise from the section's x axis.
    Arc { radius: f64, angle: f64, toward: f64 },
}

/// A wire under a rigid 2D map: `(u, v)` goes to `offset + m (u, v)`, with `m` orthogonal.
fn mapped(wire: &Wire, m: [[f64; 2]; 2], offset: Vec2) -> Wire {
    let at = |p: Vec2| v2(offset.x + m[0][0] * p.x + m[0][1] * p.y, offset.y + m[1][0] * p.x + m[1][1] * p.y);
    let turn = |x: f64, y: f64| (m[1][0] * x + m[1][1] * y).atan2(m[0][0] * x + m[0][1] * y);
    let sense = if m[0][0] * m[1][1] - m[0][1] * m[1][0] > 0.0 { 1.0 } else { -1.0 };
    Wire {
        segs: wire
            .segs
            .iter()
            .map(|s| {
                let geom = match s.geom {
                    SegGeom::Line { a, b } => SegGeom::Line { a: at(a), b: at(b) },
                    SegGeom::Arc { center, radius, a0, a1 } => {
                        let start = turn(a0.cos(), a0.sin());
                        SegGeom::Arc { center: at(center), radius, a0: start, a1: start + sense * (a1 - a0) }
                    }
                    SegGeom::Spline(ref c) => SegGeom::Spline(std::sync::Arc::new(c.mapped(|p| {
                        let q = at(v2(p.x, p.y));
                        v3(q.x, q.y, p.z)
                    }))),
                };
                Seg { geom, id: s.id }
            })
            .collect(),
    }
}

/// A profile under a rigid 2D map, with its wires put back in their proper directions and ids kept.
pub fn mapped_profile(p: &Profile, m: [[f64; 2]; 2], offset: Vec2) -> Profile {
    let outer = mapped(&p.outer, m, offset);
    let outer = if outer.signed_area() < 0.0 { outer.reversed() } else { outer };
    let holes = p
        .holes
        .iter()
        .map(|h| {
            let h = mapped(h, m, offset);
            if h.signed_area() > 0.0 {
                h.reversed()
            } else {
                h
            }
        })
        .collect();
    Profile { outer, holes }
}

/// Sweeps a profile along a smooth path of lines and arcs, starting at `frame` and heading along
/// its z axis. Straight runs are extrusions and bends are revolutions, so the surfaces stay exact:
/// planes, cylinders, cones, spheres and tori.
pub fn sweep(profile: &Profile, frame: &Frame, path: &[PathStep], feature: u32) -> Result<Body> {
    if path.is_empty() {
        return Err(Error::InvalidInput("a sweep needs at least one step".into()));
    }
    let mut section = *frame;
    let mut pieces: Vec<Body> = Vec::with_capacity(path.len());
    for (i, step) in path.iter().enumerate() {
        match *step {
            PathStep::Line { length } => {
                if !(length > 0.0) {
                    return Err(Error::InvalidInput(format!("sweep step {i}: a length must be positive, not {length}")));
                }
                pieces.push(extrude(profile, &section, length, feature)?);
                section.origin = section.origin + section.z * length;
            }
            PathStep::Arc { radius, angle, toward } => {
                if !(radius > 0.0) {
                    return Err(Error::InvalidInput(format!("sweep step {i}: a bend's radius must be positive, not {radius}")));
                }
                if !(angle > 0.0 && angle <= TAU + 1e-9) {
                    return Err(Error::InvalidInput(format!("sweep step {i}: a bend's angle must be above 0 and at most a full turn")));
                }
                let to_center = section.x * toward.cos() + section.y * toward.sin();
                let center = section.origin + to_center * radius;
                let x = -to_center;
                let axis = x.cross(section.z);
                let turning = Frame { origin: center, x, y: section.z, z: axis };
                // Distance from the axis and height along it, for a point (u, v) of the section.
                let m = [[section.x.dot(x), section.y.dot(x)], [section.x.dot(axis), section.y.dot(axis)]];
                let moved = mapped_profile(profile, m, v2(radius, 0.0));
                pieces.push(revolve(&moved, &turning, angle, feature).map_err(|e| match e {
                    Error::InvalidProfile(why) if why.contains("axis") => {
                        Error::InvalidProfile(format!("sweep step {i}: the bend is too tight, and the profile crosses its center"))
                    }
                    other => other,
                })?);
                let spin = Frame::rotation(center, axis, angle);
                section = Frame {
                    origin: spin.to_world(section.origin),
                    x: spin.dir_to_world(section.x),
                    y: spin.dir_to_world(section.y),
                    z: spin.dir_to_world(section.z),
                };
            }
        }
    }
    let total = pieces.len();
    let mut pieces = pieces.into_iter();
    let mut current = pieces.next().unwrap();
    for (k, piece) in pieces.enumerate() {
        let checks = if k + 2 == total { Checks::Full } else { Checks::Interim };
        current = boolean_with(&current, &piece, BoolOp::Union, checks)?;
    }
    Ok(current)
}

/// One piece of a sweep's path in space.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PathCurve {
    Line { from: Vec3, to: Vec3 },
    /// From `from`, around `center` by `angle` radians, right-handed about the unit vector `axis`.
    Arc { center: Vec3, axis: Vec3, from: Vec3, angle: f64 },
}

impl PathCurve {
    /// The point a fraction `s` of the way along.
    pub fn point(&self, s: f64) -> Vec3 {
        match *self {
            PathCurve::Line { from, to } => from + (to - from) * s,
            PathCurve::Arc { center, axis, from, angle } => Frame::rotation(center, axis, angle * s).to_world(from),
        }
    }

    pub fn start(&self) -> Vec3 {
        self.point(0.0)
    }

    pub fn end(&self) -> Vec3 {
        self.point(1.0)
    }

    /// The unit direction of travel a fraction `s` of the way along.
    pub fn tangent(&self, s: f64) -> Vec3 {
        match *self {
            PathCurve::Line { from, to } => (to - from).normalized(),
            PathCurve::Arc { center, axis, .. } => axis.cross(self.point(s) - center).normalized(),
        }
    }

    pub fn length(&self) -> f64 {
        match *self {
            PathCurve::Line { from, to } => from.dist(to),
            PathCurve::Arc { center, from, angle, .. } => center.dist(from) * angle,
        }
    }

    pub fn reversed(&self) -> PathCurve {
        match *self {
            PathCurve::Line { from, to } => PathCurve::Line { from: to, to: from },
            PathCurve::Arc { center, axis, angle, .. } => PathCurve::Arc { center, axis: -axis, from: self.end(), angle },
        }
    }

    /// The two parts either side of a fraction `s` of the way along.
    fn split(&self, s: f64) -> (PathCurve, PathCurve) {
        let p = self.point(s);
        match *self {
            PathCurve::Line { from, to } => (PathCurve::Line { from, to: p }, PathCurve::Line { from: p, to }),
            PathCurve::Arc { center, axis, from, angle } => (PathCurve::Arc { center, axis, from, angle: angle * s }, PathCurve::Arc { center, axis, from: p, angle: angle * (1.0 - s) }),
        }
    }

    /// Fractions along where it crosses a plane, as `n . (q - o) = 0`.
    fn crossings(&self, o: Vec3, n: Vec3) -> Vec<f64> {
        let mut out = Vec::new();
        match *self {
            PathCurve::Line { from, to } => {
                let (a, b) = (n.dot(from - o), n.dot(to - o));
                if a != b && a * b <= 0.0 {
                    out.push(a / (a - b));
                }
            }
            PathCurve::Arc { center, axis, from, angle } => {
                // n . (center + c cos t + (axis x c) sin t - o), with c from the center to the start.
                let c = from - center;
                let (p, q, r) = (n.dot(c), n.dot(axis.cross(c)), n.dot(center - o));
                let amp = p.hypot(q);
                if amp > 0.0 && r.abs() <= amp {
                    let base = q.atan2(p);
                    let spread = (-r / amp).acos();
                    for t in [base + spread, base - spread] {
                        let t = t.rem_euclid(TAU);
                        if t <= angle {
                            out.push(t / angle);
                        }
                    }
                }
            }
        }
        out
    }
}

/// Sweeps a profile drawn in `frame`'s plane along a smooth chain of lines and arcs, in any order and either way round, as
/// Onshape's sweep does. The profile goes from the end of the path in its plane, or for a closed path from where it crosses
/// that plane nearest the profile, and stays square to the path, so it has to be drawn square to it there. Straight runs
/// are extrusions and bends are revolutions, so the surfaces stay exact.
pub fn sweep_along(profile: &Profile, frame: &Frame, pieces: &[PathCurve], feature: u32) -> Result<Body> {
    let refuse = |why: String| Error::InvalidInput(format!("sweeping along that path: {why}"));
    if pieces.is_empty() {
        return Err(refuse("there's nothing in it".into()));
    }
    let size = pieces.iter().map(|c| c.start().norm().max(c.end().norm()) + c.length()).fold(1.0f64, f64::max);
    let join = 1e-7 * size;
    let at = |p: Vec3| format!("({:.4}, {:.4}, {:.4})", p.x, p.y, p.z);

    // End to end. Each end meets one other end, or none if it's an end of the whole path.
    let ends: Vec<Vec3> = pieces.iter().flat_map(|c| [c.start(), c.end()]).collect();
    let others = |k: usize| -> Vec<usize> { (0..ends.len()).filter(|&j| j != k && ends[j].dist(ends[k]) <= join).collect() };
    let mut free = Vec::new();
    for k in 0..ends.len() {
        match others(k).len() {
            0 => free.push(k),
            1 => {}
            n => return Err(refuse(format!("{} pieces meet at {}, so it branches", n + 1, at(ends[k])))),
        }
    }
    let closed = free.is_empty();
    if !closed && free.len() != 2 {
        return Err(refuse(format!("it's in {} pieces that don't join", free.len() / 2)));
    }
    let mut chain: Vec<PathCurve> = Vec::with_capacity(pieces.len());
    let mut used = vec![false; pieces.len()];
    let mut k = if closed { 0 } else { free[0] };
    loop {
        let i = k / 2;
        used[i] = true;
        chain.push(if k % 2 == 0 { pieces[i] } else { pieces[i].reversed() });
        // On from the other end of this piece.
        let next = others(k ^ 1).into_iter().find(|j| !used[j / 2]);
        match next {
            Some(j) => k = j,
            None => break,
        }
    }
    if used.iter().any(|u| !u) {
        return Err(refuse(format!("it's in more than one piece: {} of its {} curves don't join the rest", used.iter().filter(|u| !**u).count(), pieces.len())));
    }

    // Where it starts: the end in the profile's plane, or where a closed one crosses that plane nearest the profile.
    let (o, n) = (frame.origin, frame.z);
    let middle = profile_middle(profile, frame);
    let off = |p: Vec3| n.dot(p - o).abs();
    if closed {
        let mut best: Option<(f64, usize, f64)> = None;
        for (i, c) in chain.iter().enumerate() {
            for s in c.crossings(o, n) {
                let d = c.point(s).dist(middle);
                if best.is_none_or(|b| d < b.0) {
                    best = Some((d, i, s));
                }
            }
        }
        let (_, i, s) = best.ok_or_else(|| refuse("it's closed and never crosses the profile's plane".into()))?;
        chain.rotate_left(i);
        if s > 1e-9 && s < 1.0 - 1e-9 {
            let (a, b) = chain[0].split(s);
            chain[0] = b;
            chain.push(a);
        } else if s >= 1.0 - 1e-9 {
            chain.rotate_left(1);
        }
    } else {
        let (first, last) = (chain[0].start(), chain[chain.len() - 1].end());
        let (a, b) = (off(first), off(last));
        if b < a - join || (b <= a + join && last.dist(middle) < first.dist(middle)) {
            chain = chain.iter().rev().map(PathCurve::reversed).collect();
        }
    }
    let start = chain[0].start();
    if off(start) > join {
        return Err(refuse(format!(
            "it starts {:.4} from the profile's plane. The profile goes along the path from its start, so the path has to start in the profile's plane",
            off(start)
        )));
    }
    let heading = chain[0].tangent(0.0);
    let square = heading.dot(n).abs();
    if square < 1.0 - 1e-9 {
        return Err(refuse(format!(
            "it leaves the profile's plane at {:.3} degrees from square to it. The profile stays square to the path, so it has to be drawn square to the path where that starts",
            square.min(1.0).acos().to_degrees()
        )));
    }
    for w in 0..chain.len() - usize::from(!closed) {
        let (a, b) = (&chain[w], &chain[(w + 1) % chain.len()]);
        let turn = a.tangent(1.0).dot(b.tangent(0.0)).clamp(-1.0, 1.0).acos();
        if turn > 1e-6 {
            return Err(refuse(format!("it turns a corner of {:.3} degrees at {}, and a sweep's path has to be smooth", turn.to_degrees(), at(b.start()))));
        }
    }

    // The section starts on the path, square to it, with the profile's x axis, and the profile in its coordinates.
    let ahead = heading.dot(n) > 0.0;
    let section = Frame { origin: start, x: frame.x, y: if ahead { frame.y } else { -frame.y }, z: if ahead { n } else { -n } };
    let shift = frame.to_local(start);
    let sense = if ahead { 1.0 } else { -1.0 };
    let moved = mapped_profile(profile, [[1.0, 0.0], [0.0, sense]], v2(-shift.x, -sense * shift.y));

    // Each piece as a step, following the section as `sweep` will.
    let mut steps = Vec::with_capacity(chain.len());
    let mut here = section;
    for c in &chain {
        match *c {
            PathCurve::Line { .. } => {
                let length = c.length();
                steps.push(PathStep::Line { length });
                here.origin = here.origin + here.z * length;
            }
            PathCurve::Arc { center, angle, .. } => {
                let to = center - here.origin;
                let radius = to.norm();
                let toward = to.dot(here.y).atan2(to.dot(here.x));
                steps.push(PathStep::Arc { radius, angle, toward });
                let to_center = here.x * toward.cos() + here.y * toward.sin();
                let axis = (-to_center).cross(here.z);
                let spin = Frame::rotation(here.origin + to_center * radius, axis, angle);
                here = Frame { origin: spin.to_world(here.origin), x: spin.dir_to_world(here.x), y: spin.dir_to_world(here.y), z: spin.dir_to_world(here.z) };
            }
        }
    }
    let end = chain[chain.len() - 1].end();
    if here.origin.dist(end) > 1e3 * join {
        return Err(Error::Internal(format!("following the path's steps ends {:.2e} from its end", here.origin.dist(end))));
    }
    sweep(&moved, &section, &steps, feature)
}

/// Roughly the middle of a profile's outline, in space, for telling which crossing of its plane is nearest it.
fn profile_middle(profile: &Profile, frame: &Frame) -> Vec3 {
    let mut sum = v2(0.0, 0.0);
    let mut count = 0.0;
    for seg in &profile.outer.segs {
        for i in 0..8 {
            sum = sum + seg.at(i as f64 / 8.0);
            count += 1.0;
        }
    }
    let m = sum * (1.0 / count);
    frame.point(m.x, m.y, 0.0)
}
