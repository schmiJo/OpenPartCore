#!/bin/sh
# Models every real part through the tools an app uses, has OpenCASCADE read each result, and prints the table.
# Usage: scripts/check_real_parts.sh <scratch folder> [a piece of a part's name]
# Needs the neighbouring FreeCAD build that occt.sh uses. Without it, everything but OpenCASCADE's column still runs.
export PATH="$HOME/.cargo/bin:$PATH"
work="$1"
mkdir -p "$work"
here=$(cd "$(dirname "$0")" && pwd)
find "$work" -maxdepth 1 \( -name '*.step' -o -name '*.png' -o -name '*.result.json' -o -name '*.scan.json' -o -name 'occt.json' \) -delete
cargo run -q --release -p coreshape-mcp --example real_parts -- "$work" $2 >"$work/run.log" 2>&1 || cat "$work/run.log"
sh "$here/occt.sh" "$work" "$here/check_real_parts.py" 1200 || true
python3 -I "$here/real_parts_table.py" "$work"
