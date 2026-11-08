#!/bin/sh
# Exports the gallery and checks every STEP file in OpenCASCADE, using the FreeCAD
# build next door read-only: no bytecode is written and its settings stay in a
# throwaway folder. Usage: check_step.sh <scratch directory>
set -e
work="$1"
root="$(cd "$(dirname "$0")/.." && pwd)"
fc=/home/schmijo/Documents/Workspace/open_shape/FreeCAD
export CONDA_PREFIX="$fc/.pixi/envs/default"
(cd "$root" && cargo run --release -q -p coreshape-wasm --example gallery -- "$work/gallery" 2>/dev/null)
rm -f "$work/gallery/occt_report.txt"
PYTHONDONTWRITEBYTECODE=1 FREECAD_USER_HOME="$work/fchome" STEP_DIR="$work/gallery" PATH="$CONDA_PREFIX/bin:$PATH" \
  timeout 180 "$fc/build/release/bin/FreeCADCmd" "$root/scripts/check_step_occt.py" >/dev/null 2>&1 || true
cat "$work/gallery/occt_report.txt"
