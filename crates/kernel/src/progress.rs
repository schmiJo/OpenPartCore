//! How far a long piece of work has got, for whoever wants to show it: a bar under a STEP import, say. It's off
//! unless a hook is set, and then only on the thread that set it, and costs a check otherwise.
//!
//! The kernel has no clock (a browser's WebAssembly has none it can read), so it doesn't throttle by time. It calls
//! the hook when the stage changes, when the work is done, and otherwise only when it's a whole percent further on,
//! so a stage makes at most about a hundred calls however much there is to do.

use std::cell::RefCell;

/// Where a piece of work is.
#[derive(Clone, Debug, PartialEq)]
pub struct Progress {
    /// The document feature being rebuilt, if the work is part of one.
    pub feature: Option<u32>,
    /// What's being done, as a word an app can switch on: `"parsing"`, `"solids"`, `"rebuild"`.
    pub stage: &'static str,
    /// How much of it is done, out of `total`, in whatever the stage counts: bytes, solids, features.
    pub done: usize,
    pub total: usize,
}

impl Progress {
    /// As JSON, `{"feature": 3, "stage": "solids", "done": 120, "total": 711}`, leaving out a feature it hasn't got.
    pub fn json(&self) -> String {
        let feature = self.feature.map_or(String::new(), |f| format!("\"feature\":{f},"));
        format!("{{{feature}\"stage\":\"{}\",\"done\":{},\"total\":{}}}", self.stage, self.done, self.total)
    }
}

type Hook = Box<dyn FnMut(&Progress)>;

struct State {
    hook: Hook,
    feature: Option<u32>,
    /// The stage, total and percent last told, so the same isn't told twice.
    last: Option<(&'static str, usize, usize)>,
}

thread_local! {
    static STATE: RefCell<Option<State>> = const { RefCell::new(None) };
}

/// Calls `hook` with the progress of whatever this thread does from now on, until `clear`. Replaces any hook already set.
pub fn set_hook(hook: impl FnMut(&Progress) + 'static) {
    STATE.with(|s| *s.borrow_mut() = Some(State { hook: Box::new(hook), feature: None, last: None }));
}

/// Stops telling.
pub fn clear() {
    STATE.with(|s| *s.borrow_mut() = None);
}

/// Whether anyone's listening, for work that would cost something to count.
pub fn on() -> bool {
    STATE.with(|s| s.borrow().is_some())
}

/// Which feature the work that follows belongs to, or none.
pub fn feature(id: Option<u32>) {
    STATE.with(|s| {
        if let Some(s) = s.borrow_mut().as_mut() {
            s.feature = id;
            s.last = None;
        }
    });
}

/// Says that `done` of `total` of `stage` are done. The hook hears it if the stage is new, the work is finished, or it's a
/// whole percent further on than what it last heard.
pub fn report(stage: &'static str, done: usize, total: usize) {
    STATE.with(|s| {
        let mut s = s.borrow_mut();
        let Some(state) = s.as_mut() else { return };
        let percent = if total == 0 { 100 } else { (done.min(total) as u128 * 100 / total as u128) as usize };
        let fresh = match state.last {
            Some((st, t, p)) => st != stage || t != total || p != percent,
            None => true,
        };
        if !fresh {
            return;
        }
        state.last = Some((stage, total, percent));
        let p = Progress { feature: state.feature, stage, done: done.min(total), total };
        // The hook could itself report, so it isn't called with the state borrowed.
        let mut hook = std::mem::replace(&mut state.hook, Box::new(|_| {}));
        drop(s);
        hook(&p);
        STATE.with(|s| {
            if let Some(state) = s.borrow_mut().as_mut() {
                state.hook = hook;
            }
        });
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::rc::Rc;

    #[test]
    fn tells_each_percent_once_and_each_stage() {
        let heard = Rc::new(RefCell::new(Vec::new()));
        let h = heard.clone();
        set_hook(move |p| h.borrow_mut().push(p.clone()));
        feature(Some(3));
        for k in 0..=10_000 {
            report("parsing", k, 10_000);
        }
        report("solids", 0, 2);
        report("solids", 1, 2);
        report("solids", 1, 2);
        report("solids", 2, 2);
        clear();
        report("solids", 2, 2);
        let heard = heard.borrow();
        assert_eq!(heard.iter().filter(|p| p.stage == "parsing").count(), 101);
        assert_eq!(heard.iter().filter(|p| p.stage == "solids").count(), 3);
        assert!(heard.iter().all(|p| p.feature == Some(3)));
        assert_eq!(heard.last().unwrap().json(), r#"{"feature":3,"stage":"solids","done":2,"total":2}"#);
    }
}
