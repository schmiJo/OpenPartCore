//! A short randomized run in the test suite. `examples/fuzz.rs` runs the long one, with a report.

use coreshape_sketch::fuzz::*;
use coreshape_sketch::rng::Rng;
use coreshape_sketch::*;

#[test]
fn random_sketches_get_no_wrong_answers() {
    let mut wrong = Vec::new();
    for category in ALL_CATEGORIES {
        for seed in 0..120 {
            let out = check_case(&generate(category, seed));
            for w in out.wrong {
                wrong.push(format!("{} {seed}: {w}", category.name()));
            }
        }
    }
    assert!(wrong.is_empty(), "{} wrong:\n{}", wrong.len(), wrong.join("\n"));
}

#[test]
fn cases_replay_exactly() {
    for category in ALL_CATEGORIES {
        let (a, b) = (generate(category, 5), generate(category, 5));
        assert_eq!(a.start, b.start);
        assert_eq!(a.truth, b.truth);
    }
}

/// Random damage to a valid file must never panic, whether it loads or solves or not.
#[test]
fn damaged_sketches_never_panic() {
    let mut rng = Rng::new(808);
    let awkward = ["0", "-1", "1e308", "1e-308", "1e13", "null", "[]", "\"x\"", "true"];
    let mut loaded = 0;
    for round in 0..1500 {
        let case = generate(ALL_CATEGORIES[round % ALL_CATEGORIES.len()], round as u64);
        let text = case.start.to_json().to_text();
        let mut s = text.into_bytes();
        match rng.int(0, 2) {
            0 => {
                for _ in 0..rng.int(1, 4) {
                    let at = rng.int(0, s.len() as i64 - 1) as usize;
                    s[at] = rng.int(32, 126) as u8;
                }
            }
            1 => {
                let at = rng.int(0, s.len() as i64 - 2) as usize;
                let n = rng.int(1, 30) as usize;
                s.drain(at..(at + n).min(s.len()));
            }
            _ => {
                let t = String::from_utf8_lossy(&s).to_string();
                let spots: Vec<usize> = t
                    .char_indices()
                    .filter(|(i, c)| *i > 0 && (c.is_ascii_digit() || *c == '-') && matches!(t.as_bytes()[*i - 1], b':' | b'['))
                    .map(|(i, _)| i)
                    .collect();
                if spots.is_empty() {
                    continue;
                }
                let at = spots[rng.int(0, spots.len() as i64 - 1) as usize];
                let end = at + t[at..].find(|c: char| !(c.is_ascii_digit() || matches!(c, '.' | 'e' | '-' | '+'))).unwrap_or(0);
                let new = awkward[rng.int(0, awkward.len() as i64 - 1) as usize];
                s = format!("{}{}{}", &t[..at], new, &t[end..]).into_bytes();
            }
        }
        let damaged = String::from_utf8_lossy(&s);
        if let Ok(sketch) = Sketch::load_text(&damaged) {
            loaded += 1;
            if let Ok(solved) = sketch.solve() {
                let _ = solved.sketch.regions();
                let _ = solved.sketch.save_text();
            }
            let _ = sketch.analyze();
        }
    }
    assert!(loaded > 200, "only {loaded} damaged sketches loaded, so this tested little");
}
