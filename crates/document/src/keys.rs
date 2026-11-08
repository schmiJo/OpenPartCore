//! Telling apart faces that share a name. A name like `f2:end` is stable but not unique: it means every
//! face with that name, which covers mirror and pattern copies, all the faces of one fillet, the halves
//! of a whole circle and the pieces left by a later cut. A key adds what the name leaves out:
//!
//! ```text
//! f2:end@f5.1                      copy 1 made by pattern f5
//! f3:side.1[f2:end|f2:side.0]      the fillet face that replaced the edge between those two faces
//! f4:side.0#1                      the second piece, counted by position
//! f3:side.1[f2:end|f2:side.0]@f5.1#0   all of those together
//! ```
//!
//! Each part only narrows, so a plain name is still a key and still means all of them. `face_keys` gives
//! each face one that is its own, and `find_faces` goes the other way.

use crate::{FaceRef, FeatureId};
use coreshape::error::{Error, Result};
use coreshape::topo::{Body, CopyStep};
use std::cell::OnceCell;
use std::collections::{BTreeSet, HashMap};

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct FaceKey {
    pub face: FaceRef,
    /// For a fillet or chamfer, the faces whose edge it replaced. For a shell's inner wall, the face it
    /// follows. Empty means it doesn't matter.
    pub between: Vec<FaceRef>,
    /// Which copy, for the patterns and mirrors listed. Index 0 is the original.
    pub copies: Vec<CopyStep>,
    /// Which piece, counting pieces that agree on everything else. `None` means all of them.
    pub piece: Option<u32>,
}

impl FaceKey {
    pub fn plain(face: FaceRef) -> FaceKey {
        FaceKey { face, between: Vec::new(), copies: Vec::new(), piece: None }
    }

    /// Whether this is only a name.
    pub fn is_plain(&self) -> bool {
        self.between.is_empty() && self.copies.is_empty() && self.piece.is_none()
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
        if let Some(piece) = self.piece {
            out.push_str(&format!("#{piece}"));
        }
        out
    }

    pub fn parse(text: &str) -> Result<FaceKey> {
        let bad = || {
            Error::InvalidInput(format!(
                "\"{text}\" isn't a face key: it should look like f2:end, f3:side.1[f2:end|f2:side.0], f2:end@f5.1 or f2:end#0"
            ))
        };
        let (rest, piece) = match text.rsplit_once('#') {
            Some((rest, n)) => (rest, Some(n.parse::<u32>().map_err(|_| bad())?)),
            None => (text, None),
        };
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
        Ok(FaceKey { face: FaceRef::parse(name)?, between, copies, piece })
    }

    /// The features this key refers to, which it needs to still be there.
    pub(crate) fn references(&self, out: &mut Vec<FeatureId>) {
        out.push(self.face.feature);
        out.extend(self.between.iter().map(|f| f.feature));
        out.extend(self.copies.iter().map(|s| FeatureId(s.feature)));
    }
}

/// Where a face is, in its own surface's coordinates, so that the order of pieces doesn't change when the
/// whole part is moved or copied. Rounded, so that it sorts the same on every platform.
fn position(body: &Body, f: usize) -> [i64; 3] {
    let face = &body.faces[f];
    let frame = face.surface.frame();
    let (mut sum, mut n) = (coreshape::math::Vec3::ZERO, 0.0);
    for c in face.loops.iter().flatten() {
        sum = sum + frame.to_local(body.coedge_point(*c, 0.5));
        n += 1.0;
    }
    let mean = sum * (1.0 / f64::max(n, 1.0));
    [mean.x, mean.y, mean.z].map(|x| (x * 1e5).round() as i64)
}

/// One key per face, in face order, each picking out only that face.
pub fn face_keys(body: &Body) -> Vec<FaceKey> {
    // Every face under a name lists the same features, so a copy's key has the same form as the original's.
    let mut features: HashMap<_, BTreeSet<u32>> = HashMap::new();
    for f in &body.faces {
        features.entry(f.tag).or_default().extend(f.origin.copies.iter().map(|s| s.feature));
    }
    let mut keys: Vec<FaceKey> = body
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
            FaceKey { face: FaceRef::from_tag(f.tag), between, copies, piece: None }
        })
        .collect();
    let mut same: HashMap<FaceKey, Vec<usize>> = HashMap::new();
    for (i, key) in keys.iter().enumerate() {
        same.entry(key.clone()).or_default().push(i);
    }
    for (_, mut list) in same {
        if list.len() > 1 {
            list.sort_by_key(|&f| (position(body, f), f));
            for (piece, &f) in list.iter().enumerate() {
                keys[f].piece = Some(piece as u32);
            }
        }
    }
    keys
}

/// The faces a key picks, in face order. A plain name gives every face with that name.
pub fn find_faces(body: &Body, key: &FaceKey) -> Vec<usize> {
    let keys = Keys::default();
    (0..body.faces.len()).filter(|&f| keys.matches(body, f, key)).collect()
}

/// Face keys worked out once, the first time something needs a piece number.
#[derive(Default)]
pub(crate) struct Keys(OnceCell<Vec<FaceKey>>);

impl Keys {
    pub(crate) fn matches(&self, body: &Body, f: usize, key: &FaceKey) -> bool {
        let face = &body.faces[f];
        if face.tag != key.face.tag() {
            return false;
        }
        if !key.between.is_empty() && !key.between.iter().map(|r| r.tag()).eq(face.origin.between.iter().copied()) {
            return false;
        }
        for step in &key.copies {
            let index = face.origin.copies.iter().rev().find(|s| s.feature == step.feature).map_or(0, |s| s.index);
            if index != step.index {
                return false;
            }
        }
        // The only piece there is counts as piece 0, so a key survives its neighbour being filled back in.
        match key.piece {
            None => true,
            Some(piece) => self.0.get_or_init(|| face_keys(body))[f].piece.unwrap_or(0) == piece,
        }
    }
}
