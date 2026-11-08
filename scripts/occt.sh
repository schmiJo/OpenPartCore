#!/bin/sh
# Runs a Python script under OpenCASCADE, through the neighbouring FreeCAD build, without
# writing anything there. Usage: occt.sh <scratch directory> <script.py>. The script
# gets the scratch directory in the KR environment variable.
work="$1"; script="$2"
fc=/home/schmijo/Documents/Workspace/open_shape/FreeCAD
export CONDA_PREFIX="$fc/.pixi/envs/default"
cmd="$fc/build/release/bin/OpenShapeCmd"
[ -x "$cmd" ] || cmd="$fc/build/release/bin/FreeCADCmd"
PYTHONDONTWRITEBYTECODE=1 FREECAD_USER_HOME="$work/fchome" KR="$work" PATH="$CONDA_PREFIX/bin:$PATH" \
  timeout "${3:-200}" "$cmd" "$script" >/dev/null 2>&1
