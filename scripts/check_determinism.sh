#!/bin/sh
# Runs the determinism example twice in separate processes and compares every checksum.
set -e
bin=target/release/examples/determinism
cargo build --release -q -p coreshape-wasm --example determinism
a=$($bin); b=$($bin)
[ "$a" = "$b" ] && echo "identical across processes: $(echo "$a" | wc -l) models" || { echo "DIFFERENT"; exit 1; }
