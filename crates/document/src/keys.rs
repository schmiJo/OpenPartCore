//! Telling apart faces that share a name. A name like `f2:end` is stable but not unique: it means every
//! face with that name, which covers mirror and pattern copies, all the faces of one fillet, the halves
//! of a whole circle and the pieces left by a later cut. A key adds what the name leaves out:
//!
//! ```text
//! f2:end@f5.1                        copy 1 made by pattern f5
//! f3:side.1[f2:end|f2:side.0]        the fillet face that replaced the edge between those two faces
//! f2:end{f6:side.1}                  the piece of f2:end that touches f6:side.1
//! f2:end{f6:side.1,f6:side.3@f7.2}   the piece that touches both
//! f4:side.0~1                        the second of the pieces nothing else tells apart, by position
//! f4:side.0#1                        the second face by position, as files written before 2026-11-20 count them
//! ```
//!
//! Each part only narrows, so a plain name is still a key and still means all of them. A whole round face that the
//! kernel keeps as two halves is one face here, and its key picks both, as there's nothing to tell them apart but
//! where the kernel put its seam. `face_keys` gives each face a key that picks it, and its other half if it has one,
//! and `find_faces` goes the other way.
//!
//! A piece is told apart by what it touches rather than by where it is, so that it keeps its key when a dimension
//! moves the pieces past each other, or another piece comes or goes. A key made to pick one piece that picks several
//! is refused rather than read as all of them: see `find_one`.

use crate::{FaceRef, FeatureId};
use coreshape::error::{Error, Result};
use coreshape::surface::Surface;
use coreshape::topo::{Body, CopyStep};
use std::cell::{OnceCell, RefCell};
use std::collections::{BTreeSet, HashMap, HashSet};

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct FaceKey {
    pub face: FaceRef,
    /// For a fillet or chamfer, the faces whose edge it replaced. For a shell's inner wall, the face it
    /// follows. Empty means it doesn't matter.
    pub between: Vec<FaceRef>,
    /// Which copy, for the patterns and mirrors listed. Index 0 is the original.
    pub copies: Vec<CopyStep>,
    /// Faces the piece touches, each named without saying which piece, which tell it from the other pieces under its
    /// name. Empty means it doesn't matter.
    pub touching: Vec<FaceKey>,
    /// Which piece by position, of those that agree on everything else, a round face's halves counting as one. The last
    /// resort, where nothing they touch tells them apart. Written `~1`.
    pub nth: Option<u32>,
    /// Which face by position, as files written before 2026-11-20 count them, among faces that agree on the name,
    /// what it replaced and which copy, a round face's halves counting apart. Written `#1`. Read, and no longer written.
    pub piece: Option<u32>,
}

impl FaceKey {
    pub fn plain(face: FaceRef) -> FaceKey {
        FaceKey { face, between: Vec::new(), copies: Vec::new(), touching: Vec::new(), nth: None, piece: None }
    }

    /// One copy of a face made by patterns or mirrors.
    pub fn copy(face: FaceRef, copies: Vec<CopyStep>) -> FaceKey {
        FaceKey { copies, ..FaceKey::plain(face) }
    }

    /// Whether this is only a name.
    pub fn is_plain(&self) -> bool {
        self.between.is_empty() && self.copies.is_empty() && !self.is_narrowed()
    }

    /// Whether it was made to pick one piece of several, so that picking more than one piece now means it can't tell
    /// which was meant.
    pub fn is_narrowed(&self) -> bool {
        !self.touching.is_empty() || self.nth.is_some() || self.piece.is_some()
    }

    /// The name, what it replaced and which copy: everything but which piece.
    pub fn base(&self) -> FaceKey {
        FaceKey { touching: Vec::new(), nth: None, piece: None, ..self.clone() }
    }

    pub fn text(&self) -> String {
        let mut out = self.face.name();
        if !self.between.is_empty() {
            let names: Vec<String> = self.between.iter().map(|f| f.name()).collect();
            out.push_str(&format!("[{}]", names.join("|")));
        }
        for step in &self.copies {
            out.push_str(&format!("@f{}.{}", step.feature, step.index));
        }
        if !self.touching.is_empty() {
            let names: Vec<String> = self.touching.iter().map(FaceKey::text).collect();
            out.push_str(&format!("{{{}}}", names.join(",")));
        }
        if let Some(n) = self.nth {
            out.push_str(&format!("~{n}"));
        }
        if let Some(piece) = self.piece {
            out.push_str(&format!("#{piece}"));
        }
        out
    }

    pub fn parse(text: &str) -> Result<FaceKey> {
        let bad = || {
            Error::InvalidInput(format!(
                "\"{text}\" isn't a face key: it should look like f2:end, f3:side.1[f2:end|f2:side.0], f2:end@f5.1, f2:end{{f6:side.1}} or f2:end~0"
            ))
        };
        let mut rest = text;
        let number = |mark: char, rest: &mut &str| -> Result<Option<u32>> {
            match rest.rsplit_once(mark) {
                Some((before, n)) if !n.contains(['}', '{', '@', '[', ']']) => {
                    *rest = before;
                    Ok(Some(n.parse::<u32>().map_err(|_| bad())?))
                }
                _ => Ok(None),
            }
        };
        let piece = number('#', &mut rest)?;
        let nth = number('~', &mut rest)?;
        let mut touching = Vec::new();
        if let Some(inside) = rest.strip_suffix('}') {
            let (head, list) = inside.rsplit_once('{').ok_or_else(bad)?;
            rest = head;
            for item in list.split(',') {
                let key = FaceKey::parse(item)?;
                if !key.touching.is_empty() || key.nth.is_some() || key.piece.is_some() {
                    return Err(bad());
                }
                touching.push(key);
            }
            touching.sort_by_key(FaceKey::text);
        }
        if rest.contains(['{', '}', '~', '#']) {
            return Err(bad());
        }
        let mut parts = rest.split('@');
        let head = parts.next().ok_or_else(bad)?;
        let mut copies = Vec::new();
        for step in parts {
            let (feature, index) = step.strip_prefix('f').and_then(|s| s.split_once('.')).ok_or_else(bad)?;
            copies.push(CopyStep { feature: feature.parse().map_err(|_| bad())?, index: index.parse().map_err(|_| bad())? });
        }
        let (name, between) = match head.split_once('[') {
            Some((name, list)) => {
                let list = list.strip_suffix(']').ok_or_else(bad)?;
                let mut between = list.split('|').map(FaceRef::parse).collect::<Result<Vec<_>>>()?;
                between.sort_by_key(|f| f.tag());
                (name, between)
            }
            None => (head, Vec::new()),
        };
        Ok(FaceKey { face: FaceRef::parse(name)?, between, copies, touching, nth, piece })
    }

    /// The features this key refers to, which it needs to still be there.
    pub(crate) fn references(&self, out: &mut Vec<FeatureId>) {
        out.push(self.face.feature);
        out.extend(self.between.iter().map(|f| f.feature));
        out.extend(self.copies.iter().map(|s| FeatureId(s.feature)));
        for t in &self.touching {
            t.references(out);
        }
    }
}

/// Where a set of faces is, in the first one's surface coordinates, so that the order of pieces doesn't change when the
/// whole part is moved or copied. Rounded, so that it sorts the same on every platform.
fn position(body: &Body, faces: &[usize]) -> [i64; 3] {
    let frame = body.faces[faces[0]].surface.frame();
    let (mut sum, mut n) = (coreshape::math::Vec3::ZERO, 0.0);
    for &f in faces {
        for c in body.faces[f].loops.iter().flatten() {
            sum = sum + frame.to_local(body.coedge_point(*c, 0.5));
            n += 1.0;
        }
    }
    let mean = sum * (1.0 / f64::max(n, 1.0));
    [mean.x, mean.y, mean.z].map(|x| (x * 1e5).round() as i64)
}

/// Things in order of where they are: by their first coordinate, then the second, then the third, each rounded to 10
/// nanometres, so that places equal but for rounding count as equal, and it sorts the same on every platform.
fn in_order(places: &[[f64; 3]]) -> Vec<usize> {
    let mut order: Vec<usize> = (0..places.len()).collect();
    order.sort_by_key(|&i| (places[i].map(|x| (x * 1e5).round() as i64), i));
    order
}

/// Pieces in order of where they are, each in its own surface's coordinates, so that the order doesn't change when the
/// whole part is moved or copied.
fn pieces_in_order(body: &Body, pieces: &Pieces, list: &[usize]) -> Vec<usize> {
    let places: Vec<[f64; 3]> = list.iter().map(|&q| position(body, &pieces.faces[q]).map(|x| x as f64 / 1e5)).collect();
    in_order(&places).into_iter().map(|i| list[i]).collect()
}

/// Whether two faces lie on one surface, as a round face's two halves do.
fn same_surface(body: &Body, x: usize, y: usize, tolerance: f64) -> bool {
    let (a, b) = (&body.faces[x], &body.faces[y]);
    if a.flipped != b.flipped || std::mem::discriminant(&a.surface) != std::mem::discriminant(&b.surface) {
        return false;
    }
    if a.surface == b.surface {
        return true;
    }
    let on = |s: &Surface, f: usize| body.faces[f].loops.iter().flatten().all(|c| [0.25, 0.5, 0.75].iter().all(|t| s.sdf(body.coedge_point(*c, *t)).abs() <= tolerance));
    on(&a.surface, y) && on(&b.surface, x)
}

/// What a body's faces are, as keys see them: each face's name, what it replaced and which copy, and the pieces those
/// make, a piece being faces with all of that in common that meet on one surface.
struct Pieces {
    base: Vec<FaceKey>,
    /// Which piece each face is in.
    piece_of: Vec<usize>,
    /// Each piece's faces.
    faces: Vec<Vec<usize>>,
    /// Each piece's neighbours: the faces across its edges.
    around: Vec<Vec<usize>>,
    /// Each face's number by position as files written before 2026-11-20 count them.
    legacy: Vec<Option<u32>>,
}

impl Pieces {
    fn of(body: &Body) -> Pieces {
        // Every face under a name lists the same features, so a copy's key has the same form as the original's.
        let mut features: HashMap<_, BTreeSet<u32>> = HashMap::new();
        for f in &body.faces {
            features.entry(f.tag).or_default().extend(f.origin.copies.iter().map(|s| s.feature));
        }
        let base: Vec<FaceKey> = body
            .faces
            .iter()
            .map(|f| {
                let copies = features[&f.tag]
                    .iter()
                    .map(|&feature| CopyStep {
                        feature,
                        index: f.origin.copies.iter().rev().find(|s| s.feature == feature).map_or(0, |s| s.index),
                    })
                    .collect();
                let between = f.origin.between.iter().map(|&t| FaceRef::from_tag(t)).collect();
                FaceKey { between, ..FaceKey::copy(FaceRef::from_tag(f.tag), copies) }
            })
            .collect();
        let n = body.faces.len();
        let uses = body.edge_uses();
        let tolerance = 1e-7 * body.rough_size();
        let mut parent: Vec<usize> = (0..n).collect();
        fn root(parent: &mut [usize], mut x: usize) -> usize {
            while parent[x] != x {
                parent[x] = parent[parent[x]];
                x = parent[x];
            }
            x
        }
        for list in &uses {
            if let [x, y] = list.as_slice() {
                let (x, y) = (x.0, y.0);
                if x != y && base[x] == base[y] && same_surface(body, x, y, tolerance) {
                    let (a, b) = (root(&mut parent, x), root(&mut parent, y));
                    parent[a.max(b)] = a.min(b);
                }
            }
        }
        let mut number = HashMap::new();
        let mut piece_of = vec![0; n];
        let mut faces: Vec<Vec<usize>> = Vec::new();
        for f in 0..n {
            let r = root(&mut parent, f);
            let p = *number.entry(r).or_insert_with(|| {
                faces.push(Vec::new());
                faces.len() - 1
            });
            piece_of[f] = p;
            faces[p].push(f);
        }
        let mut around: Vec<BTreeSet<usize>> = vec![BTreeSet::new(); faces.len()];
        for list in &uses {
            for x in list {
                for y in list {
                    if piece_of[x.0] != piece_of[y.0] {
                        around[piece_of[x.0]].insert(y.0);
                    }
                }
            }
        }
        // Numbered as they were: every face that agrees on the rest, by where it is.
        let mut legacy = vec![None; n];
        let mut same: HashMap<&FaceKey, Vec<usize>> = HashMap::new();
        for (f, key) in base.iter().enumerate() {
            same.entry(key).or_default().push(f);
        }
        for (_, mut list) in same {
            if list.len() > 1 {
                list.sort_by_key(|&f| (position(body, &[f]), f));
                for (k, &f) in list.iter().enumerate() {
                    legacy[f] = Some(k as u32);
                }
            }
        }
        Pieces { base, piece_of, faces, around: around.into_iter().map(|s| s.into_iter().collect()).collect(), legacy }
    }
}

/// Whether a face has the name, what it replaced and which copy that a key asks for, whichever piece it's in.
fn base_matches(body: &Body, f: usize, key: &FaceKey) -> bool {
    let face = &body.faces[f];
    if face.tag != key.face.tag() {
        return false;
    }
    if !key.between.is_empty() && !key.between.iter().map(|r| r.tag()).eq(face.origin.between.iter().copied()) {
        return false;
    }
    key.copies.iter().all(|step| face.origin.copies.iter().rev().find(|s| s.feature == step.feature).map_or(0, |s| s.index) == step.index)
}

/// One key per face, in face order, each picking out that face, and the other half of a round face if it has one.
pub fn face_keys(body: &Body) -> Vec<FaceKey> {
    keys_of(body, &Pieces::of(body))
}

fn keys_of(body: &Body, pieces: &Pieces) -> Vec<FaceKey> {
    let mut piece_key: Vec<FaceKey> = pieces.faces.iter().map(|faces| pieces.base[faces[0]].clone()).collect();
    let mut by_base: HashMap<&FaceKey, Vec<usize>> = HashMap::new();
    for (p, faces) in pieces.faces.iter().enumerate() {
        by_base.entry(&pieces.base[faces[0]]).or_default().push(p);
    }
    for (base, list) in by_base {
        if list.len() < 2 {
            continue;
        }
        // What each piece touches, by name, what it replaced and which copy, other than pieces of its own name.
        let near: Vec<HashSet<&FaceKey>> = list.iter().map(|&p| pieces.around[p].iter().map(|&h| &pieces.base[h]).filter(|k| *k != base).collect()).collect();
        // Earlier features first, as what a part started as lasts longer than what was done to it later.
        let rank = |k: &FaceKey| (k.face.feature.0, k.text());
        for (i, &p) in list.iter().enumerate() {
            let alone = |set: &[&FaceKey]| near.iter().enumerate().all(|(j, other)| j == i || !set.iter().all(|k| other.contains(k)));
            let mut mine: Vec<&FaceKey> = near[i].iter().copied().collect();
            mine.sort_by_cached_key(|k| rank(k));
            let single = mine.iter().filter(|k| alone(&[**k])).min_by_key(|k| rank(k));
            let touching: Option<Vec<&FaceKey>> = match single {
                Some(k) => Some(vec![*k]),
                None => {
                    let mut best: Option<(Vec<&FaceKey>, _)> = None;
                    for a in 0..mine.len() {
                        for b in a + 1..mine.len() {
                            if alone(&[mine[a], mine[b]]) {
                                let r = rank(mine[a]).max(rank(mine[b]));
                                if best.as_ref().is_none_or(|(_, s)| r < *s) {
                                    best = Some((vec![mine[a], mine[b]], r));
                                }
                            }
                        }
                    }
                    best.map(|(set, _)| set)
                }
            };
            match touching {
                Some(set) => {
                    let mut set: Vec<FaceKey> = set.into_iter().cloned().collect();
                    set.sort_by_key(FaceKey::text);
                    piece_key[p].touching = set;
                }
                None => {
                    // Nothing it touches tells it apart, so where it is has to.
                    let order = pieces_in_order(body, pieces, &list);
                    piece_key[p].nth = Some(order.iter().position(|&q| q == p).expect("in the list") as u32);
                }
            }
        }
    }
    (0..body.faces.len()).map(|f| piece_key[pieces.piece_of[f]].clone()).collect()
}

/// The faces a key picks, in face order. A plain name gives every face with that name.
pub fn find_faces(body: &Body, key: &FaceKey) -> Vec<usize> {
    let keys = Keys::default();
    let found: Vec<usize> = (0..body.faces.len()).filter(|&f| keys.matches(body, f, key)).collect();
    crate::trace::record(body, crate::trace::Kind::Face, || key.text(), &found);
    found
}

/// The faces a key picks, which have to be there, and, for a key made to pick one piece, be one piece. Picking two
/// would be guessing that both were meant.
pub fn find_one(body: &Body, key: &FaceKey) -> Result<Vec<usize>> {
    let found = find_faces(body, key);
    if found.is_empty() {
        return Err(Error::InvalidInput(format!("the face {} no longer exists in the model", key.text())));
    }
    Keys::default().one_piece(body, key, &found)?;
    Ok(found)
}

/// Whether two edges lie on one line, circle or ellipse, as a whole rim's two halves do.
fn same_curve(body: &Body, a: usize, b: usize, tolerance: f64) -> bool {
    use coreshape::Curve;
    let parallel = |x: coreshape::math::Vec3, y: coreshape::math::Vec3| x.cross(y).norm() <= 1e-9;
    match (&body.edges[a].curve, &body.edges[b].curve) {
        (Curve::Line { origin: p, dir: d }, Curve::Line { origin: q, dir: e }) => parallel(*d, *e) && (*q - *p).cross(*d).norm() <= tolerance,
        (Curve::Circle { frame: f, radius: r }, Curve::Circle { frame: g, radius: s }) => {
            f.origin.dist(g.origin) <= tolerance && (r - s).abs() <= tolerance && parallel(f.z, g.z)
        }
        (Curve::Ellipse { frame: f, major: m, minor: n }, Curve::Ellipse { frame: g, major: p, minor: q }) => {
            f.origin.dist(g.origin) <= tolerance && (m - p).abs() <= tolerance && (n - q).abs() <= tolerance && parallel(f.z, g.z) && parallel(f.x, g.x)
        }
        _ => false,
    }
}

/// A body's edges in chains: edges between the same two pieces, on one curve, meeting end to end, as the two halves of a
/// rim do. A chain is one edge to someone looking at the part.
pub(crate) struct Chains {
    /// Each edge's chain.
    pub of: Vec<usize>,
    /// Each chain's edges.
    pub members: Vec<Vec<usize>>,
    /// For each chain, the faces at its two ends other than its own two pieces. A closed chain has none.
    pub ends: Vec<Vec<usize>>,
}

impl Chains {
    fn of(body: &Body, pieces: &Pieces) -> Chains {
        let uses = body.edge_uses();
        let n = body.edges.len();
        let tolerance = 1e-7 * body.rough_size();
        let pair = |e: usize| match uses[e].as_slice() {
            [x, y] => {
                let (p, q) = (pieces.piece_of[x.0], pieces.piece_of[y.0]);
                Some((p.min(q), p.max(q)))
            }
            _ => None,
        };
        let mut at: Vec<Vec<usize>> = vec![Vec::new(); body.vertices.len()];
        for (e, edge) in body.edges.iter().enumerate() {
            at[edge.v0].push(e);
            if edge.v1 != edge.v0 {
                at[edge.v1].push(e);
            }
        }
        let mut parent: Vec<usize> = (0..n).collect();
        fn root(parent: &mut [usize], mut x: usize) -> usize {
            while parent[x] != x {
                parent[x] = parent[parent[x]];
                x = parent[x];
            }
            x
        }
        for list in &at {
            for (i, &a) in list.iter().enumerate() {
                for &b in &list[i + 1..] {
                    if pair(a).is_some() && pair(a) == pair(b) && same_curve(body, a, b, tolerance) {
                        let (x, y) = (root(&mut parent, a), root(&mut parent, b));
                        parent[x.max(y)] = x.min(y);
                    }
                }
            }
        }
        let mut number = HashMap::new();
        let mut of = vec![0; n];
        let mut members: Vec<Vec<usize>> = Vec::new();
        for e in 0..n {
            let r = root(&mut parent, e);
            let c = *number.entry(r).or_insert_with(|| {
                members.push(Vec::new());
                members.len() - 1
            });
            of[e] = c;
            members[c].push(e);
        }
        // The faces at each vertex.
        let mut faces_at: Vec<Vec<usize>> = vec![Vec::new(); body.vertices.len()];
        for (f, face) in body.faces.iter().enumerate() {
            for c in face.loops.iter().flatten() {
                let v = body.start_vertex(*c);
                if !faces_at[v].contains(&f) {
                    faces_at[v].push(f);
                }
            }
        }
        let ends: Vec<Vec<usize>> = members
            .iter()
            .map(|list| {
                let mut count: HashMap<usize, usize> = HashMap::new();
                for &e in list {
                    *count.entry(body.edges[e].v0).or_default() += 1;
                    *count.entry(body.edges[e].v1).or_default() += 1;
                }
                let own: Vec<usize> = pair(list[0]).map_or(Vec::new(), |(p, q)| vec![p, q]);
                let mut out: Vec<usize> = count
                    .iter()
                    .filter(|(_, k)| **k == 1)
                    .flat_map(|(v, _)| faces_at[*v].iter().copied())
                    .filter(|f| !own.contains(&pieces.piece_of[*f]))
                    .collect();
                out.sort_unstable();
                out.dedup();
                out
            })
            .collect();
        Chains { of, members, ends }
    }
}

/// Face keys worked out once, the first time something needs to tell pieces apart.
#[derive(Default)]
pub(crate) struct Keys {
    pieces: OnceCell<Pieces>,
    chains: OnceCell<Chains>,
    /// For each key with a position, the pieces it counts among, in order.
    ordered: RefCell<HashMap<FaceKey, Vec<usize>>>,
    /// For each edge selection with a position, the chains it counts among, in order.
    chain_lists: RefCell<HashMap<String, Vec<usize>>>,
}

impl Keys {
    fn pieces(&self, body: &Body) -> &Pieces {
        self.pieces.get_or_init(|| Pieces::of(body))
    }

    pub(crate) fn chains(&self, body: &Body) -> &Chains {
        self.chains.get_or_init(|| Chains::of(body, self.pieces(body)))
    }

    /// Chains in order of where they are, in the coordinates of the surface of the first face `along` picks.
    pub(crate) fn order_chains(&self, body: &Body, along: &FaceKey, list: &[usize]) -> Vec<usize> {
        let Some(f) = (0..body.faces.len()).find(|&f| self.matches(body, f, along)) else { return list.to_vec() };
        let frame = body.faces[f].surface.frame();
        let chains = self.chains(body);
        let places: Vec<[f64; 3]> = list
            .iter()
            .map(|&c| {
                let edges = &chains.members[c];
                let sum = edges.iter().fold(coreshape::math::Vec3::ZERO, |s, &e| s + frame.to_local(body.edge_midpoint(e)));
                let mean = sum * (1.0 / edges.len() as f64);
                [mean.x, mean.y, mean.z]
            })
            .collect();
        in_order(&places).into_iter().map(|i| list[i]).collect()
    }

    /// The chains, in order, among which an edge selection with a position counts: those with an edge that `fits`.
    /// `text` is the selection, which they're kept under.
    pub(crate) fn chain_order(&self, body: &Body, text: &str, along: &FaceKey, fits: impl Fn(usize) -> bool) -> Vec<usize> {
        if let Some(list) = self.chain_lists.borrow().get(text) {
            return list.clone();
        }
        let chains = self.chains(body);
        let mut list: Vec<usize> = (0..body.edges.len()).filter(|&e| fits(e)).map(|e| chains.of[e]).collect();
        list.sort_unstable();
        list.dedup();
        let list = self.order_chains(body, along, &list);
        self.chain_lists.borrow_mut().insert(text.to_string(), list.clone());
        list
    }

    /// The key a face has without saying which piece it is.
    pub(crate) fn base(&self, body: &Body, f: usize) -> &FaceKey {
        &self.pieces(body).base[f]
    }

    /// Whether a face has the name, what it replaced and which copy that a key asks for, whichever piece it's in.
    pub(crate) fn base_matches(body: &Body, f: usize, key: &FaceKey) -> bool {
        base_matches(body, f, key)
    }

    pub(crate) fn matches(&self, body: &Body, f: usize, key: &FaceKey) -> bool {
        if !base_matches(body, f, key) {
            return false;
        }
        if !key.is_narrowed() {
            return true;
        }
        let pieces = self.pieces(body);
        let p = pieces.piece_of[f];
        if !key.touching.iter().all(|t| pieces.around[p].iter().any(|&h| base_matches(body, h, t))) {
            return false;
        }
        if let Some(n) = key.nth {
            let mut ordered = self.ordered.borrow_mut();
            let list = ordered.entry(key.clone()).or_insert_with(|| {
                let list: Vec<usize> = (0..pieces.faces.len())
                    .filter(|&q| {
                        base_matches(body, pieces.faces[q][0], key)
                            && key.touching.iter().all(|t| pieces.around[q].iter().any(|&h| base_matches(body, h, t)))
                    })
                    .collect();
                pieces_in_order(body, pieces, &list)
            });
            if list.get(n as usize) != Some(&p) {
                return false;
            }
        }
        // The only face there is counts as face 0, so a key survives its neighbour being filled back in.
        match key.piece {
            None => true,
            Some(piece) => pieces.legacy[f].unwrap_or(0) == piece,
        }
    }

    /// Refuses a key made to pick one piece that picks faces of more than one. `found` is what it picks.
    pub(crate) fn one_piece(&self, body: &Body, key: &FaceKey, found: &[usize]) -> Result<()> {
        if !key.is_narrowed() || found.len() < 2 {
            return Ok(());
        }
        let pieces = self.pieces(body);
        let mut which: Vec<usize> = found.iter().map(|&f| pieces.piece_of[f]).collect();
        which.sort_unstable();
        which.dedup();
        if which.len() > 1 {
            return Err(Error::InvalidInput(format!(
                "{} was one face, and it now fits {} faces, so which it meant can't be told: pick it again",
                key.text(),
                which.len()
            )));
        }
        Ok(())
    }
}
