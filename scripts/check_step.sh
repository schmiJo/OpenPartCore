#!/bin/sh
# Exports the gallery and checks every STEP file in OpenCASCADE, using the FreeCAD
# build next door read-only: no bytecode is written and its settings stay in a
# throwaway folder. Usage: check_step.sh <scratch directory>
set -e
work="$1"
root="$(cd "$(dirname "$0")/.." && pwd)"
fc=/home/schmijo/Documents/Workspace/open_shape/FreeCAD
mkdir -p "$work/fchome"
export CONDA_PREFIX="$fc/.pixi/envs/default"
# That project is under active work, and its binary has been renamed once already.
cmd="$fc/build/release/bin/OpenShapeCmd"
[ -x "$cmd" ] || cmd="$fc/build/release/bin/FreeCADCmd"
(cd "$root" && cargo run --release -q -p coreshape-wasm --example gallery -- "$work/gallery" 2>/dev/null)
rm -f "$work/gallery/occt_report.txt"
HOME="$work/fchome" XDG_DATA_HOME="$work/fchome/data" XDG_CONFIG_HOME="$work/fchome/config" XDG_CACHE_HOME="$work/fchome/cache" PYTHONDONTWRITEBYTECODE=1 FREECAD_USER_HOME="$work/fchome" STEP_DIR="$work/gallery" PATH="$CONDA_PREFIX/bin:$PATH" \
  timeout 180 "$cmd" "$root/scripts/check_step_occt.py" >/dev/null 2>&1 || true
cat "$work/gallery/occt_report.txt"
