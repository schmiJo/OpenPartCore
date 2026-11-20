//! A record of what each feature's references picked during a rebuild, so that a test can check that a pick stays on
//! the face, edge or corner it was made on as dimensions change. It's off unless a test turns it on, and then only on
//! that test's thread, and costs a check of a flag otherwise.

use coreshape::math::Vec3;
use coreshape::topo::Body;
use std::cell::RefCell;
use std::sync::Arc;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Kind {
    Face,
    Edge,
    Vertex,
}

/// One reference, as one rebuild resolved it.
#[derive(Clone, Debug)]
pub struct Pick {
    /// The feature being rebuilt, or 0 outside a rebuild.
    pub feature: u32,
    pub kind: Kind,
    /// The reference, as it's written.
    pub text: String,
    /// What it picked, as indices into `all`.
    pub found: Vec<usize>,
    /// Every face, edge or vertex there was to pick from, each as points on it.
    pub all: Arc<Vec<Vec<Vec3>>>,
}

#[derive(Default)]
struct State {
    feature: u32,
    picks: Vec<Pick>,
}

thread_local! {
    static STATE: RefCell<Option<State>> = const { RefCell::new(None) };
}

/// Starts recording on this thread, forgetting anything recorded before.
pub fn start() {
    STATE.with(|s| *s.borrow_mut() = Some(State::default()));
}

/// Stops recording, and gives what was recorded.
pub fn stop() -> Vec<Pick> {
    STATE.with(|s| s.borrow_mut().take().map(|s| s.picks).unwrap_or_default())
}

pub(crate) fn on() -> bool {
    STATE.with(|s| s.borrow().is_some())
}

/// Which feature the picks that follow belong to.
pub(crate) fn feature(id: u32) {
    STATE.with(|s| {
        if let Some(s) = s.borrow_mut().as_mut() {
            s.feature = id;
        }
    });
}

/// Points along each of a body's faces, edges or vertices, which are what a pick is followed by.
pub(crate) fn shapes(body: &Body, kind: Kind) -> Vec<Vec<Vec3>> {
    match kind {
        Kind::Vertex => body.vertices.iter().map(|v| vec![*v]).collect(),
        Kind::Edge => body
            .edges
            .iter()
            .map(|e| (0..=4).map(|i| e.curve.eval(e.t0 + (e.t1 - e.t0) * i as f64 / 4.0)).collect())
            .collect(),
        Kind::Face => (0..body.faces.len())
            .map(|f| {
                let face = &body.faces[f];
                let mut points: Vec<Vec3> =
                    face.loops.iter().flatten().flat_map(|c| [body.coedge_point(*c, 0.25), body.coedge_point(*c, 0.75)]).collect();
                if points.is_empty() {
                    points.push(face.surface.frame().origin);
                }
                points
            })
            .collect(),
    }
}

/// Records a pick among a body's faces, edges or vertices.
pub(crate) fn record(body: &Body, kind: Kind, text: impl FnOnce() -> String, found: &[usize]) {
    if !on() {
        return;
    }
    record_among(Arc::new(shapes(body, kind)), kind, text(), found.to_vec());
}

/// Records a pick among shapes already worked out, such as the vertices of several bodies.
pub(crate) fn record_among(all: Arc<Vec<Vec<Vec3>>>, kind: Kind, text: String, found: Vec<usize>) {
    STATE.with(|s| {
        if let Some(s) = s.borrow_mut().as_mut() {
            let feature = s.feature;
            s.picks.push(Pick { feature, kind, text, found, all });
        }
    });
}
