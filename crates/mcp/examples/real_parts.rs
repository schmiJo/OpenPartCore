//! Runs every real part's script, each in a process of its own so that a hang or a crash costs only that part, and writes
//! what happened, STEP files and pictures to a folder. `scripts/check_real_parts.sh` runs this and then OpenCASCADE.
//!
//! real_parts <out folder> [a piece of a script's name]
//! real_parts --one <out folder> <script's name>

#[path = "../tests/real_parts/common.rs"]
#[allow(dead_code)]
mod common;

use coreshape_document::json::Value;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

const FOLDER: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/real_parts");
const JOBS: usize = 3;
const LIMIT_SECONDS: u64 = 600;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.get(1).map(String::as_str) == Some("--one") {
        if std::env::var("REAL_PARTS_LOUD").is_err() {
            std::panic::set_hook(Box::new(|_| {}));
        }
        let (out, stem) = (PathBuf::from(&args[2]), &args[3]);
        let all = common::scripts(Path::new(FOLDER));
        let (_, script) = all.iter().find(|(s, _)| s == stem).expect("no such script");
        let result = common::run(script, &out, stem, true);
        // How many other sets of dimensions to try, which takes longer. Threads and the like take long enough already.
        if let Some(tries) = std::env::var("REAL_PARTS_SCAN").ok().and_then(|n| n.parse().ok()) {
            if result.seconds < 30.0 {
                let rows = common::scan(script, &out, tries).into_iter().map(|(dims, state, why)| Value::obj(vec![("dims", Value::str(dims)), ("state", Value::str(state)), ("why", Value::str(why))])).collect();
                std::fs::write(out.join(format!("{stem}.scan.json")), Value::Arr(rows).to_pretty()).unwrap();
            }
        }
        std::fs::write(out.join(format!("{stem}.result.json")), result.to_json().to_pretty()).unwrap();
        return;
    }
    let out = PathBuf::from(args.get(1).expect("give a folder"));
    std::fs::create_dir_all(&out).unwrap();
    let want = args.get(2);
    let all = common::scripts(Path::new(FOLDER));
    let mut queue: Vec<String> = all.iter().map(|(s, _)| s.clone()).filter(|s| want.is_none_or(|w| s.contains(w.as_str()))).collect();
    queue.reverse();
    let me = std::env::current_exe().unwrap();
    let mut running: Vec<(String, std::process::Child, Instant)> = Vec::new();
    while !queue.is_empty() || !running.is_empty() {
        while running.len() < JOBS {
            let Some(stem) = queue.pop() else { break };
            let _ = std::fs::remove_file(out.join(format!("{stem}.result.json")));
            let child = Command::new(&me).arg("--one").arg(&out).arg(&stem).stdout(Stdio::null()).stderr(Stdio::null()).spawn().unwrap();
            running.push((stem, child, Instant::now()));
        }
        std::thread::sleep(Duration::from_millis(50));
        running.retain_mut(|(stem, child, began)| {
            let finished = child.try_wait().unwrap().is_some();
            let late = began.elapsed().as_secs() > LIMIT_SECONDS;
            if late && !finished {
                let _ = child.kill();
                let _ = child.wait();
            }
            if !finished && !late {
                return true;
            }
            let path = out.join(format!("{stem}.result.json"));
            if path.exists() {
                println!("{stem}: done in {:.1}s", began.elapsed().as_secs_f64());
            } else {
                let what = if late { format!("still going after {LIMIT_SECONDS}s") } else { "the process died".to_string() };
                println!("{stem}: {what}");
                let body = format!(
                    "{{\"name\":\"{stem}\",\"kind\":\"\",\"features\":0,\"volume\":null,\"area\":null,\"faces\":0,\"seconds\":{},\"whole\":false,\"steps\":[{{\"label\":\"the whole part\",\"state\":\"refused\",\"why\":\"{what}\",\"seconds\":0,\"notes\":[]}}],\"written\":[]}}",
                    began.elapsed().as_secs_f64()
                );
                std::fs::write(&path, body).unwrap();
            }
            false
        });
    }
}
