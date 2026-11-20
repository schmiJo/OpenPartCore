//! Whether what's picked stays picked as dimensions change, for tests.
//!
//! A pick is followed by continuity. Moving a dimension a little moves each face, edge and corner a little, so the one
//! nearest to where the picked one was is where it has got to, unless another is about as near, when this can't tell. A
//! pick that then resolves to anything else has moved to something else, and if its feature still builds, it has done
//! so without saying. That's the failure this looks for. A feature that refuses has said so, which is fine.
//!
//! `as_clicked` first rewrites a document's references into what an app saves when each face, edge and corner is
//! clicked, as `face_keys`, `edge_picks` and `vertex_picks` give them, since those are what people's files hold.

use crate::trace::{self, Kind, Pick};
use crate::*;
use std::collections::HashMap;

/// One rebuild's picks.
pub struct Seen {
    pub value: f64,
    pub picks: Vec<Pick>,
    /// The id of the feature that failed, if one did.
    pub failed: Option<u32>,
    pub why: String,
}

/// Rebuilds a fresh copy of the document, so that nothing comes from a cache, recording what each reference picks.
pub fn seen(doc: &Document, value: f64) -> Seen {
    let mut fresh = Document::from_json(&doc.to_json()).expect("a document reads back");
    trace::start();
    let outcome = fresh.rebuild(None);
    let picks = trace::stop();
    let failed = outcome.failed.map(|i| outcome.steps[i].id.0);
    let why = outcome.failed.map_or(String::new(), |i| match &outcome.steps[i].state {
        StepState::Failed(w) => w.clone(),
        _ => String::new(),
    });
    Seen { value, picks, failed, why }
}

/// Each parameter's value at a list of places, rebuilt at each.
pub fn sweep(doc: &Document, param: &str, values: &[f64]) -> Vec<Seen> {
    let mut doc = Document::from_json(&doc.to_json()).expect("a document reads back");
    values
        .iter()
        .map(|&v| {
            doc.set_parameter(param, v).expect("a parameter there is");
            seen(&doc, v)
        })
        .collect()
}

/// How far apart two shapes are: the furthest any point of one is from the other.
fn apart(p: &[Vec3], q: &[Vec3]) -> f64 {
    let one_way = |a: &[Vec3], b: &[Vec3]| a.iter().map(|x| b.iter().map(|y| x.dist(*y)).fold(f64::MAX, f64::min)).fold(0.0, f64::max);
    one_way(p, q).max(one_way(q, p))
}

#[derive(Default, Debug, Clone)]
pub struct Tally {
    /// Picked the same thing it was following.
    pub kept: usize,
    /// Picked something else, and its feature built.
    pub moved: usize,
    /// Picked nothing, while what it picked is still there, and its feature built.
    pub dropped: usize,
    /// Its feature refused, or one before it did.
    pub refused: usize,
    /// Continuity can't tell where it went: two candidates are about as near.
    pub unclear: usize,
    /// One line for each moved or dropped pick.
    pub lines: Vec<String>,
}

impl Tally {
    pub fn add(&mut self, other: &Tally) {
        self.kept += other.kept;
        self.moved += other.moved;
        self.dropped += other.dropped;
        self.refused += other.refused;
        self.unclear += other.unclear;
        self.lines.extend(other.lines.iter().cloned());
    }

    pub fn summary(&self) -> String {
        format!("{} kept, {} moved, {} dropped, {} refused, {} unclear", self.kept, self.moved, self.dropped, self.refused, self.unclear)
    }
}

/// Compares the picks of two rebuilds a small step apart.
pub fn compare(a: &Seen, b: &Seen, label: &str) -> Tally {
    let mut out = Tally::default();
    let index = |s: &Seen| {
        let mut by: HashMap<(u32, Kind, String), Vec<usize>> = HashMap::new();
        for (i, p) in s.picks.iter().enumerate() {
            by.entry((p.feature, p.kind, p.text.clone())).or_default().push(i);
        }
        by
    };
    let later = index(b);
    let mut seen_count: HashMap<(u32, Kind, String), usize> = HashMap::new();
    for pa in &a.picks {
        let id = (pa.feature, pa.kind, pa.text.clone());
        let n = seen_count.entry(id.clone()).or_insert(0);
        let occurrence = *n;
        *n += 1;
        if pa.found.is_empty() {
            continue;
        }
        let failed_here = |b: &Seen| b.failed.is_some_and(|f| f <= pa.feature);
        let Some(pb) = later.get(&id).and_then(|list| list.get(occurrence)).map(|&i| &b.picks[i]) else {
            // Never resolved, because a feature before it, or it, refused.
            if failed_here(b) {
                out.refused += 1;
            } else {
                out.unclear += 1;
            }
            continue;
        };
        if pb.found.is_empty() {
            // Picking nothing while the feature goes on building is leaving something out without saying so, whether or
            // not what it picked is still there.
            if failed_here(b) {
                out.refused += 1;
            } else {
                out.dropped += 1;
                out.lines.push(format!("{label}: dropped {} in f{} at {} (was fine at {})", pa.text, pa.feature, b.value, a.value));
            }
            continue;
        }
        // Where each thing it picked has got to: the nearest now, if that's far nearer than anything else, and the step
        // moved it far less than the gap to its nearest neighbour before.
        let mut tracked = Vec::new();
        let mut clear = true;
        for &f in &pa.found {
            let from = &pa.all[f];
            let gap = pa.all.iter().enumerate().filter(|(j, _)| *j != f).map(|(_, s)| apart(from, s)).fold(f64::MAX, f64::min);
            let (mut best, mut d1, mut d2) = (usize::MAX, f64::MAX, f64::MAX);
            for (j, shape) in pb.all.iter().enumerate() {
                let d = apart(from, shape);
                if d < d1 {
                    (d2, d1, best) = (d1, d, j);
                } else if d < d2 {
                    d2 = d;
                }
            }
            if best == usize::MAX || !(d1 < 1e-9 || (d1 * 3.0 < d2 && d1 * 3.0 < gap)) {
                clear = false;
                break;
            }
            tracked.push(best);
        }
        if !clear {
            out.unclear += 1;
            continue;
        }
        tracked.sort_unstable();
        tracked.dedup();
        let mut got = pb.found.clone();
        got.sort_unstable();
        got.dedup();
        if std::env::var("FOLLOW_DEBUG").is_ok_and(|w| pa.text.contains(&w)) {
            let at = |s: &[Vec3]| {
                let c = s.iter().fold(Vec3::ZERO, |sum, p| sum + *p) * (1.0 / s.len().max(1) as f64);
                format!("({:.2}, {:.2}, {:.2})", c.x, c.y, c.z)
            };
            eprintln!(
                "{} at {}: picked {:?}; at {}: picked {:?}, tracked {:?}",
                pa.text,
                a.value,
                pa.found.iter().map(|&f| at(&pa.all[f])).collect::<Vec<_>>(),
                b.value,
                got.iter().map(|&f| at(&pb.all[f])).collect::<Vec<_>>(),
                tracked.iter().map(|&f| at(&pb.all[f])).collect::<Vec<_>>()
            );
        }
        if failed_here(b) {
            out.refused += 1;
        } else if got == tracked {
            out.kept += 1;
        } else {
            out.moved += 1;
            let at = |s: &[Vec3]| {
                let c = s.iter().fold(Vec3::ZERO, |sum, p| sum + *p) * (1.0 / s.len().max(1) as f64);
                format!("({:.2}, {:.2}, {:.2})", c.x, c.y, c.z)
            };
            let wanted: Vec<String> = tracked.iter().map(|&j| at(&pb.all[j])).collect();
            let picked: Vec<String> = got.iter().map(|&j| at(&pb.all[j])).collect();
            out.lines.push(format!(
                "{label}: {} in f{} moved going from {} to {}: it's at {} and picked {}",
                pa.text,
                pa.feature,
                a.value,
                b.value,
                wanted.join(" "),
                picked.join(" ")
            ));
        }
    }
    out
}

/// Sweeps a parameter out from where it is to each of two ends, in steps, and compares each rebuild with the one before.
///
/// A step that seems to move a pick is taken again in ten smaller steps, twice over if need be, and the move only counts
/// if it's still there: a step longer than the gap between two faces makes the wrong one look nearest afterwards, which
/// is continuity being fooled, not the pick. A step where continuity can't tell is taken again in ten steps once.
pub fn follow(doc: &Document, param: &str, down: &[f64], up: &[f64], label: &str) -> Tally {
    let mut out = Tally::default();
    let now = doc.environment().ok().and_then(|env| env.get(param).copied()).unwrap_or(0.0);
    let label = format!("{label} {param}");
    for side in [down, up] {
        let mut values = vec![now];
        values.extend_from_slice(side);
        let list = sweep(doc, param, &values);
        for pair in list.windows(2) {
            out.add(&closer(doc, param, &pair[0], &pair[1], &label, 2));
        }
    }
    out
}

/// Compares two rebuilds, and if a pick seems to move between them, the steps between them as well. Picks that
/// continuity can't follow get one round of smaller steps too.
fn closer(doc: &Document, param: &str, a: &Seen, b: &Seen, label: &str, depth: usize) -> Tally {
    let first = compare(a, b, label);
    let again = (first.moved > 0 && depth > 0) || (first.unclear > 0 && depth == 2);
    if !again {
        return first;
    }
    let mut values = vec![a.value];
    values.extend((1..10).map(|k| a.value + (b.value - a.value) * k as f64 / 10.0));
    let mut list = sweep(doc, param, &values);
    list.push(Seen { value: b.value, picks: b.picks.clone(), failed: b.failed, why: b.why.clone() });
    list[0] = Seen { value: a.value, picks: a.picks.clone(), failed: a.failed, why: a.why.clone() };
    let mut out = Tally::default();
    for pair in list.windows(2) {
        out.add(&closer(doc, param, &pair[0], &pair[1], label, depth - 1));
    }
    out
}

/// The parameters that count something, such as a pattern's copies. Changing one moves every copy at once, so continuity
/// can't follow a pick across it, and what a pick should do there is tested on its own.
pub fn counts(doc: &Document) -> Vec<String> {
    fn walk(v: &Value, out: &mut Vec<String>) {
        match v {
            Value::Obj(items) => {
                for (k, x) in items {
                    if ["count", "nx", "ny", "turns", "copies", "instances"].contains(&k.as_str()) {
                        if let Some(t) = x.as_str() {
                            out.extend(t.split(|c: char| !(c.is_alphanumeric() || c == '_')).filter(|w| w.chars().next().is_some_and(char::is_alphabetic)).map(str::to_string));
                        }
                    }
                    walk(x, out);
                }
            }
            Value::Arr(list) => list.iter().for_each(|x| walk(x, out)),
            _ => {}
        }
    }
    let mut out = Vec::new();
    walk(&doc.to_json(), &mut out);
    out.sort();
    out.dedup();
    out
}

/// Turns off each feature that refuses as the document stands, so that the rest can be followed, as a person would.
pub fn without_refusals(doc: &mut Document) -> Vec<String> {
    let mut off = Vec::new();
    for _ in 0..doc.features().len() {
        let outcome = doc.rebuild(None);
        let Some(i) = outcome.failed else { break };
        let id = outcome.steps[i].id;
        off.push(format!("f{} {}", id.0, outcome.steps[i].kind));
        if doc.suppress(id, true).is_err() {
            break;
        }
    }
    off
}

/// The document as an app would have saved it had each of its references been made by clicking: each edge, face and
/// corner picked by itself, as `edge_picks`, `face_keys` and `vertex_picks` give them. A reference that can't be
/// resolved as the document stands is left as it is. Gives the new document and how many references it rewrote.
pub fn as_clicked(doc: &Document) -> Result<(Document, usize)> {
    let mut v = doc.to_json();
    let env = doc.environment()?;
    let mut work = Document::from_json(&v)?;
    let mut changed = 0;
    let n = doc.features().len();
    let Value::Obj(items) = &mut v else { unreachable!("a document is an object") };
    let features = items.iter_mut().find(|(k, _)| k == "features").map(|(_, f)| f).expect("a document has features");
    let Value::Arr(list) = features else { unreachable!("features are a list") };
    for i in 1..n {
        if doc.features()[i].suppressed {
            continue;
        }
        let Some(body) = work.rebuild(Some(i - 1)).body else { continue };
        let keys = face_keys(&body);
        let picks = edge_picks(&body, &keys);
        let corners = vertex_picks(&body, &keys);
        let kind = list[i].get("type").and_then(Value::as_str).unwrap_or("").to_string();
        let Value::Obj(fields) = &mut list[i] else { continue };
        for (name, value) in fields.iter_mut() {
            match name.as_str() {
                "edges" => changed += clicked_edges(value, &body, &env, &picks) as usize,
                "sets" => {
                    if let Value::Arr(sets) = value {
                        for set in sets {
                            if let Value::Obj(f) = set {
                                for (k, e) in f.iter_mut() {
                                    if k == "edges" {
                                        changed += clicked_edges(e, &body, &env, &picks) as usize;
                                    }
                                }
                            }
                        }
                    }
                }
                "path" => {
                    if let Value::Obj(f) = value {
                        for (k, e) in f.iter_mut() {
                            if k == "edges" {
                                changed += clicked_edges(e, &body, &env, &picks) as usize;
                            }
                        }
                    }
                }
                // Shell and draft take plain names, so they're as clicked already.
                "faces" if kind != "shell" && kind != "draft" => changed += clicked_faces(value, &body, &keys) as usize,
                "end" => {
                    if let Value::Obj(f) = value {
                        for (k, e) in f.iter_mut() {
                            match k.as_str() {
                                "face" if e.as_str().is_some() => changed += clicked_face(e, &body, &keys) as usize,
                                "vertex" => changed += clicked_corner(e, &body, &env, &corners) as usize,
                                _ => {}
                            }
                        }
                    }
                }
                _ => {}
            }
        }
    }
    Ok((Document::from_json(&v)?, changed))
}

fn clicked_edges(value: &mut Value, body: &Body, env: &Env, picks: &[EdgeSelect]) -> bool {
    let Ok(query) = read_edge_select(value) else { return false };
    let Ok(found) = select_edges(&query, body, env) else { return false };
    let mut list: Vec<EdgeSelect> = Vec::new();
    for e in found {
        if !list.contains(&picks[e]) {
            list.push(picks[e].clone());
        }
    }
    let new = if list.len() == 1 { list.pop().unwrap() } else { EdgeSelect::Any(list) };
    let text = edge_select_value(&new);
    let different = text.to_text() != value.to_text();
    *value = text;
    different
}

fn clicked_faces(value: &mut Value, body: &Body, keys: &[FaceKey]) -> bool {
    let Some(list) = value.as_array() else { return false };
    let mut out: Vec<String> = Vec::new();
    for k in list {
        let Some(key) = k.as_str().and_then(|t| FaceKey::parse(t).ok()) else { return false };
        let found = find_faces(body, &key);
        if found.is_empty() {
            return false;
        }
        for f in found {
            let text = keys[f].text();
            if !out.contains(&text) {
                out.push(text);
            }
        }
    }
    let new = Value::Arr(out.iter().map(Value::str).collect());
    let different = new.to_text() != value.to_text();
    *value = new;
    different
}

fn clicked_face(value: &mut Value, body: &Body, keys: &[FaceKey]) -> bool {
    let Some(key) = value.as_str().and_then(|t| FaceKey::parse(t).ok()) else { return false };
    let found = find_faces(body, &key);
    // Clicking picks one face, so a name that covers several isn't a click.
    let [one] = found.as_slice() else { return false };
    let new = Value::str(keys[*one].text());
    let different = new.to_text() != value.to_text();
    *value = new;
    different
}

fn clicked_corner(value: &mut Value, body: &Body, env: &Env, corners: &[VertexRef]) -> bool {
    let Ok(VertexRef::Faces { faces, near }) = read_vertex_ref(value) else { return false };
    let mut common: Option<Vec<usize>> = None;
    for key in &faces {
        let here: Vec<usize> = find_faces(body, key).iter().flat_map(|f| body.faces[*f].loops.iter().flatten().map(|c| body.start_vertex(*c))).collect();
        common = Some(match common {
            Some(list) => list.into_iter().filter(|v| here.contains(v)).collect(),
            None => here,
        });
    }
    let mut list = common.unwrap_or_default();
    list.sort_unstable();
    list.dedup();
    let chosen = match (list.as_slice(), &near) {
        ([one], _) => *one,
        ([], _) => return false,
        (many, Some(p)) => {
            let Ok(p) = (|| -> Result<Vec3> { Ok(v3(num(&p[0], env)?, num(&p[1], env)?, num(&p[2], env)?)) })() else { return false };
            many.iter().copied().fold(many[0], |best, v| if body.vertices[v].dist(p) < body.vertices[best].dist(p) { v } else { best })
        }
        _ => return false,
    };
    let new = vertex_ref_value(&corners[chosen]);
    let different = new.to_text() != value.to_text();
    *value = new;
    different
}
