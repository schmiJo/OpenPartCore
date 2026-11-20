//! What an app saves when someone clicks a face, an edge or a corner: a reference that picks it and, as far as the model
//! allows, only it. A face is picked by its key, from `face_keys`. These are the edges and corners, worked out from
//! those keys, so that all three follow the model the same way.

use crate::keys::Keys;
use crate::{edge_name, EdgeSelect, Expr, FaceKey, FaceRef, VertexRef};
use coreshape::math::Vec3;
use coreshape::topo::Body;
use std::collections::{BTreeSet, HashMap, HashSet};

/// Rounded to a nanometre, as places are written for an app to keep.
fn rounded(p: Vec3) -> [Expr; 3] {
    [p.x, p.y, p.z].map(|x| Expr::number((x * 1e6).round() / 1e6))
}

/// A selection of the edges near a place: the last resort, where nothing about the model tells an edge from another.
fn near(p: Vec3) -> EdgeSelect {
    EdgeSelect::Near { point: rounded(p), tolerance: Expr::number(1e-3) }
}

/// The edges between two faces, as a selection. Plain names make the simpler selection, which reads the same.
fn between(a: &FaceKey, b: &FaceKey) -> EdgeSelect {
    if a.is_plain() && b.is_plain() {
        EdgeSelect::Between(a.face, b.face)
    } else {
        EdgeSelect::BetweenKeys(a.clone(), b.clone())
    }
}

/// For each edge, a selection that picks it, and the rest of its chain: the other half of a whole rim, say, which is
/// the same edge to someone looking at the part. `keys` are the body's face keys, as `face_keys` gives them.
///
/// Two faces that meet once are picked by the two faces. Where they meet more than once, each place is told apart by
/// a face at its ends that the others don't reach. Where nothing tells them apart, as for the two cuts a slot makes in
/// one side of a tube, it's by where it is: its order along the way the places are most spread out, in the first face's
/// own coordinates, which move with the part. That only goes wrong if a dimension moves them past each other.
pub fn edge_picks(body: &Body, keys: &[FaceKey]) -> Vec<EdgeSelect> {
    let uses = body.edge_uses();
    let lookup = Keys::default();
    let chains = lookup.chains(body);
    let names: Vec<String> = (0..body.edges.len()).map(|e| edge_name(body, e, &uses)).collect();
    let key_pairs: Vec<Vec<FaceKey>> = (0..body.edges.len())
        .map(|e| {
            let mut pair: Vec<FaceKey> = uses[e].iter().map(|u| keys[u.0].clone()).collect();
            pair.sort_by_key(FaceKey::text);
            pair
        })
        .collect();
    // The chains along which each pair of faces meets.
    let mut meet: HashMap<&[FaceKey], BTreeSet<usize>> = HashMap::new();
    for (e, pair) in key_pairs.iter().enumerate() {
        if pair.len() == 2 {
            meet.entry(pair.as_slice()).or_default().insert(chains.of[e]);
        }
    }
    let mut named: HashMap<&str, BTreeSet<usize>> = HashMap::new();
    for (e, name) in names.iter().enumerate() {
        named.entry(name.as_str()).or_default().insert(chains.of[e]);
    }
    // What's at each chain's ends, by name, what it replaced and which copy.
    let ends: Vec<HashSet<&FaceKey>> = chains.ends.iter().map(|list| list.iter().map(|&f| lookup.base(body, f)).collect()).collect();
    (0..body.edges.len())
        .map(|e| {
            // A sheet's boundary edge has one face, so it's picked by where it is.
            if uses[e].len() < 2 {
                return near(body.edge_midpoint(e));
            }
            // A name that fits only this chain is all it takes.
            let mine = chains.of[e];
            if named[names[e].as_str()].len() == 1 {
                let mut two = uses[e].iter().map(|u| FaceRef::from_tag(body.faces[u.0].tag)).collect::<Vec<_>>();
                two.sort_by_key(|f| f.name());
                return EdgeSelect::Between(two[0], two[1]);
            }
            let (a, b) = (&key_pairs[e][0], &key_pairs[e][1]);
            let others: Vec<usize> = meet[key_pairs[e].as_slice()].iter().copied().filter(|&c| c != mine).collect();
            if others.is_empty() {
                return between(a, b);
            }
            // Earlier features first, as what a part started as lasts longer than what was done to it later.
            let alone = ends[mine]
                .iter()
                .filter(|k| others.iter().all(|&c| !ends[c].contains(*k)))
                .min_by_key(|k| (k.face.feature.0, k.text()));
            match alone {
                Some(k) => EdgeSelect::Chain { between: (a.clone(), b.clone()), ends: vec![(*k).clone()], nth: None },
                None => {
                    // Where it is, among the places these two faces meet, along the way they're most spread out.
                    let mut all = meet[key_pairs[e].as_slice()].iter().copied().collect::<Vec<_>>();
                    all = lookup.order_chains(body, a, &all);
                    let nth = all.iter().position(|&c| c == mine).map(|n| n as u32);
                    EdgeSelect::Chain { between: (a.clone(), b.clone()), ends: Vec::new(), nth }
                }
            }
        })
        .collect()
}

/// For each vertex, the faces it's on, and where it is if those faces share another vertex too. `keys` are the body's
/// face keys.
pub fn vertex_picks(body: &Body, keys: &[FaceKey]) -> Vec<VertexRef> {
    let mut on: Vec<Vec<String>> = vec![Vec::new(); body.vertices.len()];
    for (f, face) in body.faces.iter().enumerate() {
        for c in face.loops.iter().flatten() {
            let list = &mut on[body.start_vertex(*c)];
            let key = keys[f].text();
            if !list.contains(&key) {
                list.push(key);
            }
        }
    }
    for list in &mut on {
        list.sort();
    }
    (0..body.vertices.len())
        .map(|v| {
            let fits = (0..on.len()).filter(|w| on[*w].iter().all(|k| on[v].contains(k)) && on[v].iter().all(|k| on[*w].contains(k))).count();
            VertexRef::Faces {
                faces: on[v].iter().map(|k| FaceKey::parse(k).expect("a key reads back")).collect(),
                near: (fits > 1).then(|| rounded(body.vertices[v])),
            }
        })
        .collect()
}
