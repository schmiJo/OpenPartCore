//! The speed benchmark as a WebAssembly module, run in a browser by `scripts/sketch_wasm_bench.py`.
//! Also proves the crate builds for WebAssembly with nothing added. Natively it builds to nothing.
#[cfg(target_arch = "wasm32")]
mod inner {
    use coreshape_sketch::workloads::*;
    use std::fmt::Write;

    #[link(wasm_import_module = "env")]
    extern "C" {
        fn now() -> f64;
    }

    static mut OUT: Vec<u8> = Vec::new();

    #[no_mangle]
    pub extern "C" fn run() -> *const u8 {
        let clock = || unsafe { now() };
        let mut text = String::new();
        for w in ALL_WORKLOADS {
            for n in [50, 100, 200, 300, 500, 1000] {
                let t = measure(w, n, &clock);
                let _ = writeln!(
                    text,
                    "{:<14} {:>8} {:>11} {:>5} {:>7.2}ms {:>7.2}ms {:>7.2}ms {:>7.3}ms {:>7.3}ms {:>7.3}ms",
                    t.workload, t.entities, t.constraints, t.dof, t.solve, t.analyze, t.begin, t.move_median, t.move_p95, t.move_worst
                );
            }
        }
        unsafe {
            let out = &mut *std::ptr::addr_of_mut!(OUT);
            *out = text.into_bytes();
            out.as_ptr()
        }
    }

    #[no_mangle]
    pub extern "C" fn len() -> usize {
        unsafe { (*std::ptr::addr_of!(OUT)).len() }
    }
}

