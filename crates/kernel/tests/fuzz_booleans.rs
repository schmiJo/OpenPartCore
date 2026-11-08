//! A short randomized run for every `cargo test`. `examples/fuzz.rs` runs the same
//! cases at larger counts and prints a report.
use coreshape::fuzz::*;

#[test]
fn no_wrong_answers() {
    for category in ALL_CATEGORIES {
        let cases = match category {
            Category::GeneralPrimitives | Category::AlignedPrimitives => 40,
            _ => 150,
        };
        for seed in 0..cases {
            let out = run_case(category, seed, 60);
            assert!(out.wrong.is_empty(), "{category:?} seed {seed}: {:#?}", out.wrong);
        }
    }
}

/// Planar solids have no hard cases left: every run must fully succeed.
#[test]
fn planar_cases_are_never_refused() {
    for category in [Category::GridBoxes, Category::RotatedBoxes, Category::GridPrisms] {
        for seed in 0..150 {
            let out = run_case(category, seed, 0);
            assert!(out.refusals.is_empty(), "{category:?} seed {seed}: {:?}", out.refusals);
        }
    }
}
