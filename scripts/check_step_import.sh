#!/bin/sh
# Checks STEP import against OpenCASCADE, through the neighbouring FreeCAD build, which is only read.
#   1. It writes STEP files of many shapes, with their volumes and areas.
#   2. This kernel reads each one, and the numbers must match.
#   3. This kernel cuts a box out of each and writes it back, and OpenCASCADE must read that as valid with the same volume.
# Usage: check_step_import.sh <scratch directory>
set -e
work="$1"
root="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$work/in" "$work/out"
sh "$root/scripts/occt.sh" "$work" "$root/scripts/step_import/make.py" 300
(cd "$root" && cargo run --release -q -p coreshape --example step_import -- "$work"/in/*.step) > "$work/in.csv"
export STEP_OUT="$work/out"
(cd "$root" && cargo run --release -q -p coreshape --example step_import -- "$work"/in/*.step) > "$work/mine.csv"
sh "$root/scripts/occt.sh" "$work" "$root/scripts/step_import/check.py" 300
python3 "$root/scripts/step_import/report.py" "$work"
