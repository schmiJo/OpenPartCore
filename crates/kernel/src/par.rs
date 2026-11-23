//! Doing many independent pieces of work at once, on every core there is, with nothing but the standard library. In a
//! browser's WebAssembly there are no threads, and the work is done one piece after another.
//!
//! The calling thread works too, and is the one that hears how far it's all got, as the progress hook is its own.

/// `f` of each item, in the items' order, with the work shared among the cores. `told` hears how many are done, from
/// the calling thread, now and then and at the end. A panic in `f` comes back out of this as a panic.
pub fn map<T: Sync, R: Send>(items: &[T], f: impl Fn(&T) -> R + Sync, mut told: impl FnMut(usize)) -> Vec<R> {
    // Work shared out already has every core busy: a piece of it that shares its own work out does it itself.
    let threads = if inside() { 1 } else { threads().min(items.len()) };
    // Helpers as far as there are free ones.
    #[cfg(not(target_arch = "wasm32"))]
    let threads = if threads > 1 { 1 + take_helpers(threads - 1) } else { 1 };
    if threads <= 1 {
        let mut out = Vec::with_capacity(items.len());
        for (i, item) in items.iter().enumerate() {
            out.push(f(item));
            told(i + 1);
        }
        return out;
    }
    run(items, &f, &mut told, threads)
}

std::thread_local! {
    /// Whether this thread is doing a share of some work, or sharing some out.
    static INSIDE: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// Whether this thread is doing a share of work shared out, so that sharing more would only crowd the cores.
pub fn inside() -> bool {
    INSIDE.with(|i| i.get())
}

/// A cap set by `set_threads`, or 0 for none.
static CAP: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

/// Caps how many threads work is shared among, for a program that runs threads of its own and doesn't want more on top of
/// them: 1 shares nothing out. It takes the place of `COSHAPE_THREADS`.
pub fn set_threads(n: usize) {
    CAP.store(n.max(1), std::sync::atomic::Ordering::Relaxed);
}

/// How many threads to share work among: the cores there are, or one where there are no threads.
pub fn threads() -> usize {
    #[cfg(target_arch = "wasm32")]
    {
        1
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        // A cap from `set_threads`, or COSHAPE_THREADS, as for leaving cores to other work, or 1 for none shared at all.
        let cores = std::thread::available_parallelism().map_or(1, |n| n.get());
        let cap = match CAP.load(std::sync::atomic::Ordering::Relaxed) {
            0 => std::env::var("COSHAPE_THREADS").ok().and_then(|v| v.trim().parse::<usize>().ok()).filter(|n| *n >= 1),
            n => Some(n),
        };
        cap.map_or(cores, |n| n.min(cores))
    }
}

/// Helper threads busy now, across everything sharing work out. All of it together has no more helpers than there are
/// other cores, so threads doing work of their own that each share some out don't crowd the cores many times over.
#[cfg(not(target_arch = "wasm32"))]
static HELPERS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

/// Takes up to `want` helpers from what's free, giving how many it got.
#[cfg(not(target_arch = "wasm32"))]
fn take_helpers(want: usize) -> usize {
    use std::sync::atomic::Ordering;
    let most = threads().saturating_sub(1);
    let mut busy = HELPERS.load(Ordering::Relaxed);
    loop {
        let got = want.min(most.saturating_sub(busy));
        if got == 0 {
            return 0;
        }
        match HELPERS.compare_exchange_weak(busy, busy + got, Ordering::AcqRel, Ordering::Relaxed) {
            Ok(_) => return got,
            Err(now) => busy = now,
        }
    }
}

#[cfg(target_arch = "wasm32")]
fn run<T: Sync, R: Send>(items: &[T], f: &(impl Fn(&T) -> R + Sync), told: &mut impl FnMut(usize), _threads: usize) -> Vec<R> {
    items
        .iter()
        .enumerate()
        .map(|(i, item)| {
            let r = f(item);
            told(i + 1);
            r
        })
        .collect()
}

#[cfg(not(target_arch = "wasm32"))]
fn run<T: Sync, R: Send>(items: &[T], f: &(impl Fn(&T) -> R + Sync), told: &mut impl FnMut(usize), threads: usize) -> Vec<R> {
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::sync::Mutex;
    // Each thread takes the next item not yet taken, so a slow one doesn't hold up the rest.
    let next = AtomicUsize::new(0);
    let done = AtomicUsize::new(0);
    let results: Mutex<Vec<Option<R>>> = Mutex::new((0..items.len()).map(|_| None).collect());
    let work = || loop {
        INSIDE.with(|x| x.set(true));
        let i = next.fetch_add(1, Ordering::Relaxed);
        if i >= items.len() {
            break;
        }
        let r = f(&items[i]);
        results.lock().unwrap_or_else(|e| e.into_inner())[i] = Some(r);
        done.fetch_add(1, Ordering::Relaxed);
    };
    // Put back as it was when done, even by a panic, and the helpers given back.
    struct Back(bool, usize);
    impl Drop for Back {
        fn drop(&mut self) {
            INSIDE.with(|x| x.set(self.0));
            HELPERS.fetch_sub(self.1, Ordering::AcqRel);
        }
    }
    let _back = Back(INSIDE.with(|x| x.replace(true)), threads - 1);
    std::thread::scope(|scope| {
        let helpers: Vec<_> = (1..threads).map(|_| scope.spawn(work)).collect();
        // This thread works too, telling how far it's all got after each of its own.
        loop {
            let i = next.fetch_add(1, Ordering::Relaxed);
            if i >= items.len() {
                break;
            }
            let r = f(&items[i]);
            results.lock().unwrap_or_else(|e| e.into_inner())[i] = Some(r);
            told(done.fetch_add(1, Ordering::Relaxed) + 1);
        }
        for h in helpers {
            if let Err(why) = h.join() {
                std::panic::resume_unwind(why);
            }
        }
    });
    told(items.len());
    results.into_inner().unwrap_or_else(|e| e.into_inner()).into_iter().map(|r| r.expect("every item was done")).collect()
}

#[cfg(test)]
mod tests {
    #[test]
    fn keeps_order_and_counts_to_the_end() {
        let items: Vec<u64> = (0..1000).collect();
        let mut last = 0;
        let out = super::map(&items, |x| x * x, |n| last = n);
        assert_eq!(out, items.iter().map(|x| x * x).collect::<Vec<_>>());
        assert_eq!(last, 1000);
        assert!(super::map(&Vec::<u8>::new(), |x| *x, |_| {}).is_empty());
    }
}
