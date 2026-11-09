//! Solves where rigid things go, given how they're mated. Each group of things that move together has a pose, and each mate
//! says which of the differences between two frames must be zero. Those are solved together by Newton's method, taking at
//! each step the smallest move that would satisfy them all if they were straight lines. So what isn't constrained stays
//! where it is, and a mechanism moved a little at a time stays on the same branch of its motion.

use coreshape::math::*;

/// The kinds of mate, as Onshape has them. Named by what's left free between their two frames: rotation about z, or
/// translation along it, and so on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MateKind {
    /// Nothing.
    Fastened,
    /// Rotation about z.
    Revolute,
    /// Translation along z.
    Slider,
    /// Rotation about z, and translation along it.
    Cylindrical,
    /// Rotation about z, and translation along x.
    PinSlot,
    /// Rotation about z, and translation along x and y.
    Planar,
    /// Rotation about any axis.
    Ball,
    /// Rotation about z, and translation along any axis.
    Parallel,
}

impl MateKind {
    /// How many degrees of freedom it leaves.
    pub fn freedom(self) -> usize {
        match self {
            MateKind::Fastened => 0,
            MateKind::Revolute | MateKind::Slider => 1,
            MateKind::Cylindrical | MateKind::PinSlot => 2,
            MateKind::Planar | MateKind::Ball => 3,
            MateKind::Parallel => 4,
        }
    }

    /// The motions a value can be read for, as limits and drives want them.
    pub fn values(self) -> &'static [Dof] {
        match self {
            MateKind::Fastened | MateKind::Ball | MateKind::Parallel => &[],
            MateKind::Revolute => &[Dof::Angle],
            MateKind::Slider => &[Dof::Z],
            MateKind::Cylindrical => &[Dof::Z, Dof::Angle],
            MateKind::PinSlot => &[Dof::X, Dof::Angle],
            MateKind::Planar => &[Dof::X, Dof::Y, Dof::Angle],
        }
    }
}

/// One way a mate can move.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Dof {
    X,
    Y,
    Z,
    Angle,
}

/// How far a mate may go. An angle is in radians.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Limit {
    pub dof: Dof,
    pub min: f64,
    pub max: f64,
}

/// A mate between two things, with its frames in their own coordinates. Any offset or flip is already in those frames.
#[derive(Clone, Debug, PartialEq)]
pub struct Joint {
    pub a: usize,
    pub b: usize,
    pub on_a: Frame,
    pub on_b: Frame,
    pub kind: MateKind,
    pub limits: Vec<Limit>,
    /// The angle it was last at, in radians, turns and all, so that a limit or a gear can tell one turn from the next.
    pub turned: f64,
}

/// One motion tied to another: `b = ratio * a + offset`. A gear, a rack and pinion, a screw, or a linear relation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Coupling {
    pub a: (usize, Dof),
    pub b: (usize, Dof),
    pub ratio: f64,
    pub offset: f64,
}

/// A motion held at a value.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Drive {
    pub joint: usize,
    pub dof: Dof,
    pub value: f64,
}

/// What's being solved. `poses` are where things are now, in world coordinates.
#[derive(Clone, Debug, Default)]
pub struct Problem {
    pub poses: Vec<Frame>,
    pub fixed: Vec<bool>,
    /// Things that move together, by index. Each thing is in at most one.
    pub groups: Vec<Vec<usize>>,
    pub joints: Vec<Joint>,
    pub couplings: Vec<Coupling>,
    pub drives: Vec<Drive>,
    /// Things that should do the moving, given a choice. Others move only if they have to.
    pub prefer: Vec<usize>,
    /// How big the things themselves are, which the poses and mates alone can't say.
    pub extent: f64,
}

/// How a solve went.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Report {
    /// Mates that couldn't be met along with the others, by index. They're left out, and the rest are met.
    pub failing: Vec<usize>,
    /// Couplings that couldn't be met, by index.
    pub failing_couplings: Vec<usize>,
    /// How many ways the whole can still move.
    pub freedom: usize,
    /// Whether each thing can still move.
    pub free: Vec<bool>,
    /// The worst that anything is off by, in millimetres or the equivalent at the size of the assembly.
    pub residual: f64,
}

/// How far a mate has to come to, position or the equivalent at the size of the assembly.
const TOLERANCE: f64 = 1e-8;
const MOST_ITERATIONS: usize = 60;
/// Each step goes no further than this share of the assembly's size, or this angle, to stay on a branch of the motion.
const FURTHEST: f64 = 0.4;

/// One equation: its value, which should be zero, and how it changes with each cluster's motion, as `(cluster, v, w)`.
struct Row {
    value: f64,
    parts: Vec<(usize, Vec3, Vec3)>,
}

#[derive(Clone)]
struct Cluster {
    pose: Frame,
    fixed: bool,
    weight: f64,
}

/// Things that move together, with where each is in its cluster.
struct Rig {
    clusters: Vec<Cluster>,
    cluster_of: Vec<usize>,
    within: Vec<Frame>,
    /// The size of the assembly, which is what rotations are measured against translations by.
    size: f64,
    /// Joints that have nothing left to say, as what they join now moves as one.
    settled: Vec<bool>,
}

/// Whether a fastened mate already holds, so that what it joins can simply move as one.
fn fastened_and_met(problem: &Problem, joint: &Joint) -> bool {
    if joint.kind != MateKind::Fastened || joint.a == joint.b || joint.a >= problem.poses.len() || joint.b >= problem.poses.len() {
        return false;
    }
    let (fa, fb) = (problem.poses[joint.a].compose(&joint.on_a), problem.poses[joint.b].compose(&joint.on_b));
    fa.origin.dist(fb.origin) <= TOLERANCE && fa.x.dist(fb.x) <= 1e-10 && fa.z.dist(fb.z) <= 1e-10
}

impl Rig {
    fn new(problem: &Problem) -> Rig {
        let n = problem.poses.len();
        let mut cluster_of = vec![usize::MAX; n];
        let mut clusters: Vec<Cluster> = Vec::new();
        let mut within = vec![Frame::WORLD; n];
        // What moves as one: each group, and whatever fastened mates that already hold have joined. Those are most of the
        // mates in most assemblies, so this is what keeps them small.
        let mut joined: Vec<usize> = (0..n).collect();
        fn root(joined: &mut [usize], mut v: usize) -> usize {
            while joined[v] != v {
                joined[v] = joined[joined[v]];
                v = joined[v];
            }
            v
        }
        for group in &problem.groups {
            let mut members = group.iter().copied().filter(|&i| i < n);
            if let Some(first) = members.next() {
                for other in members {
                    let (x, y) = (root(&mut joined, first), root(&mut joined, other));
                    joined[y] = x;
                }
            }
        }
        let mut settled = vec![false; problem.joints.len()];
        for (j, joint) in problem.joints.iter().enumerate() {
            if fastened_and_met(problem, joint) {
                settled[j] = true;
                let (x, y) = (root(&mut joined, joint.a), root(&mut joined, joint.b));
                joined[y] = x;
            }
        }
        for i in 0..n {
            let top = root(&mut joined, i);
            if top != i && cluster_of[top] == usize::MAX {
                cluster_of[top] = clusters.len();
                clusters.push(Cluster { pose: problem.poses[top], fixed: false, weight: 1.0 });
            }
            if top != i {
                let id = cluster_of[top];
                cluster_of[i] = id;
                within[i] = problem.poses[top].inverse().compose(&problem.poses[i]);
            }
        }
        for i in 0..n {
            if cluster_of[i] == usize::MAX {
                cluster_of[i] = clusters.len();
                clusters.push(Cluster { pose: problem.poses[i], fixed: false, weight: 1.0 });
            }
        }
        for i in 0..n {
            if problem.fixed.get(i).copied().unwrap_or(false) {
                clusters[cluster_of[i]].fixed = true;
            }
        }
        // With a preference, what isn't preferred moves a thousandth as readily.
        if !problem.prefer.is_empty() {
            for c in &mut clusters {
                c.weight = 1e-3;
            }
            for &i in &problem.prefer {
                if i < n {
                    clusters[cluster_of[i]].weight = 1.0;
                }
            }
        }
        // Whatever spans the things being mated. A unit of it is what a radian is worth.
        let mut bounds = Aabb::EMPTY;
        for p in &problem.poses {
            bounds.add(p.origin);
        }
        for j in &problem.joints {
            if j.a < n && j.b < n {
                bounds.add(problem.poses[j.a].to_world(j.on_a.origin));
                bounds.add(problem.poses[j.b].to_world(j.on_b.origin));
            }
        }
        let size = bounds.diagonal().max(problem.extent).clamp(1.0, 1e7);
        Rig { clusters, cluster_of, within, size, settled }
    }

    fn pose_of(&self, thing: usize) -> Frame {
        self.clusters[self.cluster_of[thing]].pose.compose(&self.within[thing])
    }

    /// The world frame of a joint's end, and the cluster it moves with.
    fn end(&self, thing: usize, on: &Frame) -> (Frame, usize) {
        (self.pose_of(thing).compose(on), self.cluster_of[thing])
    }

    fn write_back(&self, poses: &mut [Frame]) {
        for (i, p) in poses.iter_mut().enumerate() {
            *p = self.pose_of(i);
        }
    }

    fn snapshot(&self) -> Vec<Frame> {
        self.clusters.iter().map(|c| c.pose).collect()
    }

    fn restore(&mut self, poses: &[Frame]) {
        for (c, p) in self.clusters.iter_mut().zip(poses) {
            c.pose = *p;
        }
    }
}

fn rotated(v: Vec3, w: Vec3) -> Vec3 {
    let angle = w.norm();
    if angle < 1e-300 {
        return v;
    }
    let axis = w * (1.0 / angle);
    let (s, c) = angle.sin_cos();
    v * c + axis.cross(v) * s + axis * (axis.dot(v) * (1.0 - c))
}

/// The rotation that carries frame `b`'s axes onto frame `a`'s, as a vector in world coordinates.
fn rotation_between(a: &Frame, b: &Frame) -> Vec3 {
    // R = a * b^T, whose axis and angle come most safely through a quaternion.
    let col = |i: usize| -> [f64; 3] {
        let (ax, bx) = ([a.x, a.y, a.z], [b.x, b.y, b.z]);
        let pick = |v: Vec3| [v.x, v.y, v.z];
        let (mut out, mut k) = ([0.0; 3], 0);
        while k < 3 {
            let (ak, bk) = (pick(ax[k]), pick(bx[k]));
            for r in 0..3 {
                out[r] += ak[r] * bk[i];
            }
            k += 1;
        }
        out
    };
    let (c0, c1, c2) = (col(0), col(1), col(2));
    let (m00, m10, m20, m01, m11, m21, m02, m12, m22) = (c0[0], c0[1], c0[2], c1[0], c1[1], c1[2], c2[0], c2[1], c2[2]);
    let trace = m00 + m11 + m22;
    let (qw, qx, qy, qz);
    if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        qw = 0.25 * s;
        qx = (m21 - m12) / s;
        qy = (m02 - m20) / s;
        qz = (m10 - m01) / s;
    } else if m00 > m11 && m00 > m22 {
        let s = (1.0 + m00 - m11 - m22).sqrt() * 2.0;
        qw = (m21 - m12) / s;
        qx = 0.25 * s;
        qy = (m01 + m10) / s;
        qz = (m02 + m20) / s;
    } else if m11 > m22 {
        let s = (1.0 + m11 - m00 - m22).sqrt() * 2.0;
        qw = (m02 - m20) / s;
        qx = (m01 + m10) / s;
        qy = 0.25 * s;
        qz = (m12 + m21) / s;
    } else {
        let s = (1.0 + m22 - m00 - m11).sqrt() * 2.0;
        qw = (m10 - m01) / s;
        qx = (m02 + m20) / s;
        qy = (m12 + m21) / s;
        qz = 0.25 * s;
    }
    let sign = if qw < 0.0 { -1.0 } else { 1.0 };
    let v = v3(qx, qy, qz) * sign;
    let length = v.norm();
    if length < 1e-300 {
        return Vec3::ZERO;
    }
    v * (2.0 * length.atan2(qw * sign) / length)
}

/// The rotation that carries direction `b` onto direction `a`, the short way round.
fn rotation_onto(a: Vec3, b: Vec3, across: Vec3) -> Vec3 {
    let cross = b.cross(a);
    let (s, c) = (cross.norm(), b.dot(a));
    if s < 1e-14 {
        return if c > 0.0 { Vec3::ZERO } else { across * std::f64::consts::PI };
    }
    cross * (s.atan2(c) / s)
}

fn wrapped(mut angle: f64) -> f64 {
    while angle > PI {
        angle -= TAU;
    }
    while angle < -PI {
        angle += TAU;
    }
    angle
}

/// A motion's value: how far `b`'s frame is along `a`'s axis, or turned around its z, and the row for that.
fn value_row(rig: &Rig, joint: &Joint, dof: Dof) -> Row {
    let (fa, ca) = rig.end(joint.a, &joint.on_a);
    let (fb, cb) = rig.end(joint.b, &joint.on_b);
    match dof {
        Dof::Angle => {
            let near = joint.turned;
            let raw = fb.x.dot(fa.y).atan2(fb.x.dot(fa.x));
            let value = near + wrapped(raw - near);
            Row { value, parts: vec![(cb, Vec3::ZERO, fa.z), (ca, Vec3::ZERO, -fa.z)] }
        }
        _ => {
            let axis = match dof {
                Dof::X => fa.x,
                Dof::Y => fa.y,
                _ => fa.z,
            };
            along_row(rig, &fa, ca, &fb, cb, axis)
        }
    }
}

/// How far `fb`'s origin is from `fa`'s, along a direction fixed in `fa`.
fn along_row(rig: &Rig, fa: &Frame, ca: usize, fb: &Frame, cb: usize, axis: Vec3) -> Row {
    let d = fb.origin - fa.origin;
    let ra = fa.origin - rig.clusters[ca].pose.origin;
    let rb = fb.origin - rig.clusters[cb].pose.origin;
    Row { value: d.dot(axis), parts: vec![(cb, axis, rb.cross(axis)), (ca, -axis, axis.cross(d) - ra.cross(axis))] }
}

/// The equations a joint makes.
fn joint_rows(rig: &Rig, joint: &Joint, out: &mut Vec<Row>) {
    let (fa, ca) = rig.end(joint.a, &joint.on_a);
    let (fb, cb) = rig.end(joint.b, &joint.on_b);
    let mut along = |axis: Vec3| out.push(along_row(rig, &fa, ca, &fb, cb, axis));
    match joint.kind {
        MateKind::Fastened | MateKind::Revolute | MateKind::Ball => {
            along(fa.x);
            along(fa.y);
            along(fa.z);
        }
        MateKind::Slider | MateKind::Cylindrical => {
            along(fa.x);
            along(fa.y);
        }
        MateKind::PinSlot => {
            along(fa.y);
            along(fa.z);
        }
        MateKind::Planar => along(fa.z),
        MateKind::Parallel => {}
    }
    let scale = rig.size;
    // The rows of a rotation, each saying how far off it is about a direction fixed in `a`.
    let mut about = |w: Vec3, axis: Vec3| {
        let (value, parts) = (-w.dot(axis) * scale, vec![(cb, Vec3::ZERO, axis * scale), (ca, Vec3::ZERO, -axis * scale)]);
        out.push(Row { value, parts });
    };
    match joint.kind {
        MateKind::Fastened | MateKind::Slider => {
            let w = rotation_between(&fa, &fb);
            about(w, fa.x);
            about(w, fa.y);
            about(w, fa.z);
        }
        MateKind::Revolute | MateKind::Cylindrical | MateKind::PinSlot | MateKind::Planar | MateKind::Parallel => {
            let w = rotation_onto(fa.z, fb.z, fa.x);
            about(w, fa.x);
            about(w, fa.y);
        }
        MateKind::Ball => {}
    }
}

/// How a value's row is scaled to be measured in millimetres or the equivalent.
fn value_scale(rig: &Rig, dof: Dof) -> f64 {
    if dof == Dof::Angle {
        rig.size
    } else {
        1.0
    }
}

/// The equations that must hold, for the joints that count. A limit is an equation only while it's being passed.
fn hard_rows(rig: &Rig, problem: &Problem, skip: &[bool], skip_couplings: &[bool]) -> Vec<Row> {
    let mut out = Vec::new();
    for (j, joint) in problem.joints.iter().enumerate() {
        if skip[j] || rig.settled[j] {
            continue;
        }
        joint_rows(rig, joint, &mut out);
        for limit in &joint.limits {
            let row = value_row(rig, joint, limit.dof);
            let scale = value_scale(rig, limit.dof);
            let past = if row.value > limit.max {
                row.value - limit.max
            } else if row.value < limit.min {
                row.value - limit.min
            } else {
                continue;
            };
            out.push(scaled(row, past, scale));
        }
    }
    for drive in &problem.drives {
        if skip.get(drive.joint).copied().unwrap_or(true) {
            continue;
        }
        let row = value_row(rig, &problem.joints[drive.joint], drive.dof);
        let (scale, off) = (value_scale(rig, drive.dof), row.value - drive.value);
        out.push(scaled(row, off, scale));
    }
    for (c, coupling) in problem.couplings.iter().enumerate() {
        if skip_couplings[c] || skip.get(coupling.a.0).copied().unwrap_or(true) || skip.get(coupling.b.0).copied().unwrap_or(true) {
            continue;
        }
        let a = value_row(rig, &problem.joints[coupling.a.0], coupling.a.1);
        let b = value_row(rig, &problem.joints[coupling.b.0], coupling.b.1);
        // In radians or millimetres, each by what makes it comparable to the other.
        let (sa, sb) = (value_scale(rig, coupling.a.1), value_scale(rig, coupling.b.1));
        let off = b.value - coupling.ratio * a.value - coupling.offset;
        let scale = sa.max(sb);
        let mut parts = Vec::new();
        for (cluster, v, w) in b.parts {
            parts.push((cluster, v * scale, w * scale));
        }
        for (cluster, v, w) in a.parts {
            parts.push((cluster, v * (-coupling.ratio * scale), w * (-coupling.ratio * scale)));
        }
        out.push(Row { value: off * scale, parts });
    }
    out
}

fn scaled(row: Row, value: f64, scale: f64) -> Row {
    Row { value: value * scale, parts: row.parts.into_iter().map(|(c, v, w)| (c, v * scale, w * scale)).collect() }
}

fn worst(rows: &[Row]) -> f64 {
    rows.iter().fold(0.0f64, |m, r| m.max(r.value.abs()))
}

/// How far off a set of rows is altogether. A wish can be blocked along one axis and still have room along another, which
/// the worst of them wouldn't show.
fn distance(rows: &[Row]) -> f64 {
    rows.iter().map(|r| r.value * r.value).sum::<f64>().sqrt()
}

/// A column of unknowns for each thing that can move: three of translation and three of rotation, the latter in
/// units of the assembly's size, so that a step's length means the same for both.
struct Columns {
    index: Vec<Option<usize>>,
    count: usize,
}

impl Columns {
    fn of(rig: &Rig, members: &[bool]) -> Columns {
        let mut index = vec![None; rig.clusters.len()];
        let mut count = 0;
        for (c, cluster) in rig.clusters.iter().enumerate() {
            if !cluster.fixed && members[c] {
                index[c] = Some(count);
                count += 6;
            }
        }
        Columns { index, count }
    }
}

/// A row as a dense list of coefficients over the columns.
fn dense(row: &Row, columns: &Columns, rig: &Rig) -> Vec<f64> {
    let mut out = vec![0.0; columns.count];
    for (cluster, v, w) in &row.parts {
        let Some(at) = columns.index[*cluster] else { continue };
        let weight = rig.clusters[*cluster].weight;
        out[at] += v.x * weight;
        out[at + 1] += v.y * weight;
        out[at + 2] += v.z * weight;
        out[at + 3] += w.x * weight / rig.size;
        out[at + 4] += w.y * weight / rig.size;
        out[at + 5] += w.z * weight / rig.size;
    }
    out
}

/// Solves `(M + small) x = b` for a symmetric, positive matrix held by rows. The small part is what lets redundant mates
/// through, as they'd otherwise make it singular.
fn solve_symmetric(mut m: Vec<Vec<f64>>, b: &[f64]) -> Vec<f64> {
    let n = b.len();
    let trace: f64 = (0..n).map(|i| m[i][i]).sum();
    let small = (trace / n.max(1) as f64).max(1e-12) * 1e-11;
    for (i, row) in m.iter_mut().enumerate() {
        row[i] += small;
    }
    // Cholesky.
    for i in 0..n {
        for j in 0..=i {
            let mut sum = m[i][j];
            for k in 0..j {
                sum -= m[i][k] * m[j][k];
            }
            if i == j {
                m[i][i] = sum.max(small * 1e-3).sqrt();
            } else {
                m[i][j] = sum / m[j][j];
            }
        }
    }
    let mut x = b.to_vec();
    for i in 0..n {
        for k in 0..i {
            x[i] -= m[i][k] * x[k];
        }
        x[i] /= m[i][i];
    }
    for i in (0..n).rev() {
        for k in i + 1..n {
            x[i] -= m[k][i] * x[k];
        }
        x[i] /= m[i][i];
    }
    x
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

/// The smallest move that would bring these rows to what they ask, as they are at this point.
fn smallest_step(rows: &[Vec<f64>], want: &[f64]) -> Vec<f64> {
    let m = rows.len();
    let n = rows.first().map_or(0, Vec::len);
    let mut normal = vec![vec![0.0; m]; m];
    for i in 0..m {
        for j in 0..=i {
            let v = dot(&rows[i], &rows[j]);
            normal[i][j] = v;
            normal[j][i] = v;
        }
    }
    let y = solve_symmetric(normal, want);
    let mut out = vec![0.0; n];
    for (row, k) in rows.iter().zip(&y) {
        for (o, r) in out.iter_mut().zip(row) {
            *o += r * k;
        }
    }
    out
}

/// Applies a share of a step to the clusters.
fn apply_step(rig: &mut Rig, columns: &Columns, step: &[f64], share: f64) {
    for c in 0..rig.clusters.len() {
        let Some(at) = columns.index[c] else { continue };
        let weight = rig.clusters[c].weight;
        let v = v3(step[at], step[at + 1], step[at + 2]) * (weight * share);
        let w = v3(step[at + 3], step[at + 4], step[at + 5]) * (weight * share / rig.size);
        let pose = rig.clusters[c].pose;
        let (z, x) = (rotated(pose.z, w), rotated(pose.x, w));
        rig.clusters[c].pose = Frame::new(pose.origin + v, z, x);
    }
}

/// The share of a step that keeps it short enough to stay on a branch of the motion.
fn step_share(rig: &Rig, columns: &Columns, step: &[f64]) -> f64 {
    let mut furthest = 0.0f64;
    for c in 0..rig.clusters.len() {
        let Some(at) = columns.index[c] else { continue };
        let weight = rig.clusters[c].weight;
        let v = v3(step[at], step[at + 1], step[at + 2]).norm() * weight;
        let w = v3(step[at + 3], step[at + 4], step[at + 5]).norm() * weight;
        furthest = furthest.max(v.max(w) / rig.size);
    }
    if furthest > FURTHEST {
        FURTHEST / furthest
    } else {
        1.0
    }
}

/// Newton's method on the hard equations. Whether they all came to their tolerance.
fn settle(rig: &mut Rig, problem: &Problem, skip: &[bool], skip_couplings: &[bool], members: &[bool]) -> bool {
    let columns = Columns::of(rig, members);
    for _ in 0..MOST_ITERATIONS {
        let rows = hard_rows(rig, problem, skip, skip_couplings);
        let now = worst(&rows);
        if now <= TOLERANCE {
            return true;
        }
        if columns.count == 0 {
            return false;
        }
        let dense_rows: Vec<Vec<f64>> = rows.iter().map(|r| dense(r, &columns, rig)).collect();
        let want: Vec<f64> = rows.iter().map(|r| -r.value).collect();
        let step = smallest_step(&dense_rows, &want);
        let mut share = step_share(rig, &columns, &step);
        // Back off if the whole step makes things worse, which only happens far from an answer.
        let before = rig.snapshot();
        let mut improved = false;
        for _ in 0..8 {
            apply_step(rig, &columns, &step, share);
            let after = worst(&hard_rows(rig, problem, skip, skip_couplings));
            if after < now * 0.999 || after <= TOLERANCE {
                improved = true;
                break;
            }
            rig.restore(&before);
            share *= 0.5;
        }
        if !improved {
            // Nowhere better from here. A last whole step can get past a saddle, once.
            apply_step(rig, &columns, &step, step_share(rig, &columns, &step));
            let after = worst(&hard_rows(rig, problem, skip, skip_couplings));
            if after >= now {
                rig.restore(&before);
                return false;
            }
        }
    }
    worst(&hard_rows(rig, problem, skip, skip_couplings)) <= TOLERANCE
}

/// Which clusters can affect each other, through mates. Fixed ones don't connect anything.
fn components(rig: &Rig, problem: &Problem) -> Vec<Vec<bool>> {
    let n = rig.clusters.len();
    let mut parent: Vec<usize> = (0..n).collect();
    fn find(parent: &mut [usize], mut v: usize) -> usize {
        while parent[v] != v {
            parent[v] = parent[parent[v]];
            v = parent[v];
        }
        v
    }
    let mut join = |a: usize, b: usize| {
        if !rig.clusters[a].fixed && !rig.clusters[b].fixed {
            let (x, y) = (find(&mut parent, a), find(&mut parent, b));
            parent[x] = y;
        }
    };
    for j in &problem.joints {
        join(rig.cluster_of[j.a], rig.cluster_of[j.b]);
    }
    // A relation ties whatever moves at either of its mates to whatever moves at the other.
    for c in &problem.couplings {
        let (a, b) = (&problem.joints[c.a.0], &problem.joints[c.b.0]);
        let ends = [rig.cluster_of[a.a], rig.cluster_of[a.b], rig.cluster_of[b.a], rig.cluster_of[b.b]];
        let moving: Vec<usize> = ends.iter().copied().filter(|e| !rig.clusters[*e].fixed).collect();
        for w in moving.windows(2) {
            join(w[0], w[1]);
        }
    }
    let mut out: Vec<Vec<bool>> = Vec::new();
    let mut seen: Vec<usize> = Vec::new();
    for c in 0..n {
        if rig.clusters[c].fixed {
            continue;
        }
        let root = find(&mut parent, c);
        let at = match seen.iter().position(|r| *r == root) {
            Some(at) => at,
            None => {
                seen.push(root);
                out.push(vec![false; n]);
                seen.len() - 1
            }
        };
        out[at][c] = true;
    }
    out
}

/// Which mates and couplings touch these clusters. The rest can't be helped by moving them.
fn outside(rig: &Rig, problem: &Problem, members: &[bool]) -> (Vec<bool>, Vec<bool>) {
    let touches = |j: &Joint| members[rig.cluster_of[j.a]] || members[rig.cluster_of[j.b]];
    let skip: Vec<bool> = problem.joints.iter().map(|j| !touches(j)).collect();
    let skip_couplings: Vec<bool> = problem.couplings.iter().map(|c| !touches(&problem.joints[c.a.0]) && !touches(&problem.joints[c.b.0])).collect();
    (skip, skip_couplings)
}

/// Remembers how far each joint has turned, so that the next turn is told from this one.
fn remember_turns(rig: &Rig, joints: &mut [Joint]) {
    for j in joints.iter_mut() {
        if j.kind.values().contains(&Dof::Angle) {
            let row = value_row(rig, j, Dof::Angle);
            j.turned = row.value;
        }
    }
}

/// How many ways things can move, given the equations, and which of them can.
fn freedom(rig: &Rig, problem: &Problem, skip: &[bool], skip_couplings: &[bool]) -> (usize, Vec<bool>) {
    let everything = vec![true; rig.clusters.len()];
    let columns = Columns::of(rig, &everything);
    if columns.count == 0 {
        return (0, vec![false; problem.poses.len()]);
    }
    // All with even weights, as a preference doesn't change what's possible. Limits aren't counted: they don't take freedom.
    let mut even = Rig { clusters: rig.clusters.clone(), cluster_of: rig.cluster_of.clone(), within: rig.within.clone(), size: rig.size, settled: rig.settled.clone() };
    for c in &mut even.clusters {
        c.weight = 1.0;
    }
    let free_problem = Problem { joints: problem.joints.iter().map(|j| Joint { limits: Vec::new(), ..j.clone() }).collect(), drives: Vec::new(), couplings: problem.couplings.clone(), ..Problem::default() };
    let rows = hard_rows(&even, &free_problem, skip, skip_couplings);
    let mut m: Vec<Vec<f64>> = rows.iter().map(|r| dense(r, &columns, &even)).collect();
    // Gram-Schmidt by rows, leaving an independent set.
    let mut basis: Vec<Vec<f64>> = Vec::new();
    for row in &mut m {
        let start = dot(row, row).sqrt();
        if start < 1e-12 {
            continue;
        }
        for b in &basis {
            let k = dot(row, b);
            for (r, x) in row.iter_mut().zip(b) {
                *r -= k * x;
            }
        }
        let left = dot(row, row).sqrt();
        if left > 1e-7 * start {
            basis.push(row.iter().map(|x| x / left).collect());
        }
    }
    let total = columns.count - basis.len();
    // A cluster can move if some motion of it isn't ruled out by the equations.
    let mut free = vec![false; problem.poses.len()];
    for (thing, slot) in free.iter_mut().enumerate() {
        let Some(at) = columns.index[even.cluster_of[thing]] else { continue };
        *slot = (0..6).any(|k| {
            let mut e = vec![0.0; columns.count];
            e[at + k] = 1.0;
            for b in &basis {
                let c = b[at + k];
                for (x, y) in e.iter_mut().zip(b) {
                    *x -= c * y;
                }
            }
            dot(&e, &e).sqrt() > 1e-6
        });
    }
    (total, free)
}

/// Solves a problem, moving `poses` as little as it can to meet the mates, and remembering how far each has turned. A mate
/// that can't be met along with the ones before it is left out and reported, and the rest are met.
pub fn solve(problem: &mut Problem) -> Report {
    let mut rig = Rig::new(problem);
    let mut report = Report::default();
    let groups = components(&rig, problem);
    // A mate between fixed things is in no component, and can only be checked.
    let mut all_skip = vec![false; problem.joints.len()];
    let mut all_skip_couplings = vec![false; problem.couplings.len()];
    for (j, joint) in problem.joints.iter().enumerate() {
        let (a, b) = (rig.cluster_of[joint.a], rig.cluster_of[joint.b]);
        if rig.clusters[a].fixed && rig.clusters[b].fixed {
            let mut skip = vec![true; problem.joints.len()];
            skip[j] = false;
            if worst(&hard_rows(&rig, problem, &skip, &vec![true; problem.couplings.len()])) > TOLERANCE {
                report.failing.push(j);
                all_skip[j] = true;
            }
        }
    }
    // So is a relation with nothing free at either of its mates.
    for (c, coupling) in problem.couplings.iter().enumerate() {
        let (ja, jb) = (&problem.joints[coupling.a.0], &problem.joints[coupling.b.0]);
        let held = [ja.a, ja.b, jb.a, jb.b].iter().all(|&t| rig.clusters[rig.cluster_of[t]].fixed);
        if held {
            let (a, b) = (value_row(&rig, ja, coupling.a.1), value_row(&rig, jb, coupling.b.1));
            let scale = value_scale(&rig, coupling.a.1).max(value_scale(&rig, coupling.b.1));
            if (b.value - coupling.ratio * a.value - coupling.offset).abs() * scale > TOLERANCE {
                report.failing_couplings.push(c);
                all_skip_couplings[c] = true;
            }
        }
    }
    for members in &groups {
        let (skip, skip_couplings) = outside(&rig, problem, members);
        let skip: Vec<bool> = skip.iter().zip(&all_skip).map(|(a, b)| *a || *b).collect();
        let start = rig.snapshot();
        if settle(&mut rig, problem, &skip, &skip_couplings, members) {
            continue;
        }
        // It can't all be met. Take the mates in order, keeping each that can be met along with what came before.
        rig.restore(&start);
        let mut trial = vec![true; problem.joints.len()];
        let mut off_couplings = vec![true; problem.couplings.len()];
        for j in 0..problem.joints.len() {
            if skip[j] {
                continue;
            }
            trial[j] = false;
            let before = rig.snapshot();
            if !settle(&mut rig, problem, &trial, &off_couplings, members) {
                rig.restore(&before);
                trial[j] = true;
                report.failing.push(j);
                all_skip[j] = true;
            }
        }
        for c in 0..problem.couplings.len() {
            if skip_couplings[c] {
                continue;
            }
            off_couplings[c] = false;
            let before = rig.snapshot();
            if !settle(&mut rig, problem, &trial, &off_couplings, members) {
                rig.restore(&before);
                off_couplings[c] = true;
                report.failing_couplings.push(c);
                all_skip_couplings[c] = true;
            }
        }
    }
    report.failing.sort_unstable();
    report.residual = worst(&hard_rows(&rig, problem, &all_skip, &all_skip_couplings));
    let (total, free) = freedom(&rig, problem, &all_skip, &all_skip_couplings);
    report.freedom = total;
    report.free = free;
    rig.write_back(&mut problem.poses);
    remember_turns(&rig, &mut problem.joints);
    report
}

/// Where a drag wants something to go.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Goal {
    /// A point on a thing, in its own coordinates, to a place in the world. It turns however it has to.
    Point { thing: usize, local: Vec3, to: Vec3 },
    /// A thing's whole pose.
    Pose { thing: usize, to: Frame },
}

/// How a drag went.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Dragged {
    /// How far it still is from where it was wanted, in millimetres or the equivalent at the size of the assembly.
    pub remaining: f64,
    pub report: Report,
}

/// The equations a goal makes. They're wishes, and only as far as the mates allow.
fn goal_rows(rig: &Rig, goal: &Goal) -> Vec<Row> {
    match goal {
        Goal::Point { thing, local, to } => {
            let c = rig.cluster_of[*thing];
            let p = rig.pose_of(*thing).to_world(*local);
            let r = p - rig.clusters[c].pose.origin;
            [Vec3::X, Vec3::Y, Vec3::Z].iter().map(|&axis| Row { value: (p - *to).dot(axis), parts: vec![(c, axis, r.cross(axis))] }).collect()
        }
        Goal::Pose { thing, to } => {
            let c = rig.cluster_of[*thing];
            let now = rig.pose_of(*thing);
            let r = now.origin - rig.clusters[c].pose.origin;
            let w = rotation_between(to, &now);
            let mut rows: Vec<Row> = [Vec3::X, Vec3::Y, Vec3::Z].iter().map(|&axis| Row { value: (now.origin - to.origin).dot(axis), parts: vec![(c, axis, r.cross(axis))] }).collect();
            for axis in [Vec3::X, Vec3::Y, Vec3::Z] {
                rows.push(Row { value: -w.dot(axis) * rig.size, parts: vec![(c, Vec3::ZERO, axis * rig.size)] });
            }
            rows
        }
    }
}

/// Moves something toward where it's wanted, as far as its mates let it, a little at a time. What it's mated to moves
/// with it, as needed, and anything not connected to it stays.
pub fn drag(problem: &mut Problem, goal: &Goal) -> Dragged {
    let mut rig = Rig::new(problem);
    let thing = match goal {
        Goal::Point { thing, .. } | Goal::Pose { thing, .. } => *thing,
    };
    let cluster = rig.cluster_of[thing];
    let groups = components(&rig, problem);
    let members: Vec<bool> = groups.iter().find(|m| m[cluster]).cloned().unwrap_or_else(|| vec![false; rig.clusters.len()]);
    let (skip, skip_couplings) = outside(&rig, problem, &members);
    if rig.clusters[cluster].fixed {
        let remaining = distance(&goal_rows(&rig, goal));
        return Dragged { remaining, report: solve(problem) };
    }
    let mut best = distance(&goal_rows(&rig, goal));
    // Getting nowhere: near a straight chain's reach, whole steps go on gaining a little, and it doesn't shrink. Closing on
    // the nearest it can get to something out of reach gains little too, but less each time, and that's nearly done.
    let (mut slow, mut last_gain) = (0, f64::INFINITY);
    // With nothing holding it, there's no branch to stay on, and steps can be as long as the way there.
    if hard_rows(&rig, problem, &skip, &skip_couplings).is_empty() {
        rig.size = rig.size.max(best);
    }
    // Asked to prefer something, only that moves at first. A smaller weight on the others wouldn't do it: they'd still take
    // whatever part of the wish the preferred ones can't meet straight away, though swinging round would get there. If it
    // can't get there alone, the others have to move, so it goes on with all of them alike.
    let prefers = !problem.prefer.is_empty();
    let preferred: Vec<bool> = rig.clusters.iter().map(|c| c.weight >= 1.0).collect();
    for c in &mut rig.clusters {
        c.weight = 1.0;
    }
    for phase in 0..2 {
        let moving: Vec<bool> = members.iter().zip(&preferred).map(|(m, p)| *m && (phase == 1 || !prefers || *p)).collect();
        let columns = Columns::of(&rig, &moving);
        if columns.count == 0 {
            continue;
        }
        let mut stuck = 0;
        for _ in 0..400 {
            let wishes = goal_rows(&rig, goal);
            let now = distance(&wishes);
            if now <= TOLERANCE {
                break;
            }
            // The mates count for far more than the wish does, so the step goes where they allow.
            let hard = hard_rows(&rig, problem, &skip, &skip_couplings);
            let mut rows: Vec<Vec<f64>> = hard.iter().map(|r| dense(r, &columns, &rig).iter().map(|x| x * 1e3).collect()).collect();
            let mut want: Vec<f64> = hard.iter().map(|r| -r.value * 1e3).collect();
            rows.extend(wishes.iter().map(|r| dense(r, &columns, &rig)));
            want.extend(wishes.iter().map(|r| -r.value));
            // With the rows as they are, a wish the mates rule out is what's left over, and not what's taken.
            let step = least_squares_step(&rows, &want, 1e-6);
            let before = rig.snapshot();
            // Short steps, as a drag follows the mechanism along its motion.
            let mut share = step_share(&rig, &columns, &step);
            let whole_step = share >= 1.0;
            let mut moved = false;
            for _ in 0..6 {
                apply_step(&mut rig, &columns, &step, share);
                if settle(&mut rig, problem, &skip, &skip_couplings, &moving) {
                    let after = distance(&goal_rows(&rig, goal));
                    if after < now {
                        moved = true;
                        break;
                    }
                }
                rig.restore(&before);
                share *= 0.5;
            }
            if !moved {
                break;
            }
            let after = distance(&goal_rows(&rig, goal));
            if after > best - 1e-9 {
                stuck += 1;
                if stuck >= 3 {
                    break;
                }
            } else {
                stuck = 0;
                // A step cut short is just a long way to go.
                let gain = best - after;
                slow = if whole_step && after > best * 0.7 && gain > 0.5 * last_gain { slow + 1 } else { 0 };
                last_gain = gain;
                best = after;
                if slow >= 4 {
                    break;
                }
            }
        }
        if !prefers || distance(&goal_rows(&rig, goal)) <= TOLERANCE {
            break;
        }
        (slow, last_gain) = (0, f64::INFINITY);
        best = distance(&goal_rows(&rig, goal));
    }
    let remaining = distance(&goal_rows(&rig, goal));
    rig.write_back(&mut problem.poses);
    remember_turns(&rig, &mut problem.joints);
    let (total, free) = freedom(&rig, problem, &vec![false; problem.joints.len()], &vec![false; problem.couplings.len()]);
    let report = Report { failing: Vec::new(), failing_couplings: Vec::new(), freedom: total, free, residual: worst(&hard_rows(&rig, problem, &vec![false; problem.joints.len()], &vec![false; problem.couplings.len()])) };
    Dragged { remaining, report }
}

/// The step that best meets rows that can't all be met, with a little damping, by the normal equations in row space.
fn least_squares_step(rows: &[Vec<f64>], want: &[f64], damping: f64) -> Vec<f64> {
    let m = rows.len();
    let n = rows.first().map_or(0, Vec::len);
    let mut normal = vec![vec![0.0; m]; m];
    for i in 0..m {
        for j in 0..=i {
            let v = dot(&rows[i], &rows[j]);
            normal[i][j] = v;
            normal[j][i] = v;
        }
    }
    for (i, row) in normal.iter_mut().enumerate() {
        row[i] += damping;
    }
    let y = solve_symmetric(normal, want);
    let mut out = vec![0.0; n];
    for (row, k) in rows.iter().zip(&y) {
        for (o, r) in out.iter_mut().zip(row) {
            *o += r * k;
        }
    }
    out
}

/// The size of what's being solved, which is what a turn is measured against a distance by.
pub fn problem_size(problem: &Problem) -> f64 {
    Rig::new(problem).size
}

/// A mate's current value for one of its motions: millimetres, or radians with its turns.
pub fn joint_value(problem: &Problem, joint: usize, dof: Dof) -> f64 {
    let rig = Rig::new(problem);
    value_row(&rig, &problem.joints[joint], dof).value
}
