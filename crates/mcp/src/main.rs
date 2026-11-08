//! The server's entry point. Usage: coreshape-mcp [--root <folder>]
//!
//! Files are read and written only under the root, which is the current folder unless given.

use coreshape_mcp::Server;
use std::io::{BufRead, Write};
use std::path::PathBuf;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let mut root = std::env::current_dir().unwrap_or_else(|_| PathBuf::from("."));
    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--root" if i + 1 < args.len() => {
                root = PathBuf::from(&args[i + 1]);
                i += 2;
            }
            "--version" => {
                println!("coreshape-mcp {}", env!("CARGO_PKG_VERSION"));
                return;
            }
            other => {
                eprintln!("unknown argument {other}. Usage: coreshape-mcp [--root <folder>]");
                std::process::exit(2);
            }
        }
    }
    let mut server = Server::new(root);
    let stdin = std::io::stdin();
    let stdout = std::io::stdout();
    for line in stdin.lock().lines() {
        let Ok(line) = line else { break };
        if line.trim().is_empty() {
            continue;
        }
        if let Some(answer) = server.handle_line(&line) {
            let mut out = stdout.lock();
            if writeln!(out, "{answer}").is_err() || out.flush().is_err() {
                break;
            }
        }
    }
}
