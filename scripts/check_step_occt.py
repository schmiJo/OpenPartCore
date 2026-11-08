# Reads every STEP file in a directory with OpenCASCADE, through FreeCAD's Part
# module, and compares it with what the kernel says. Run under FreeCADCmd:
#   FreeCADCmd scripts/check_step_occt.py -- <directory>
# It only reads files. Set PYTHONDONTWRITEBYTECODE and FREECAD_USER_HOME so nothing
# gets written next to the FreeCAD installation or in the user's settings.
# OCCT prints its own progress to the same stream, so the report goes to a file too.
import csv, os, sys

directory = os.environ["STEP_DIR"]
import Part  # noqa: E402

rows = list(csv.DictReader(open(os.path.join(directory, "manifest.csv"))))
failures = 0
report = open(os.path.join(directory, "occt_report.txt"), "w")
def say(line):
    report.write(line + "\n")
    report.flush()
say(f"{'model':<20} {'valid':<6} {'V kernel':>14} {'V occt':>14} {'rel err':>9} {'A rel err':>9} {'solids':>6} {'faces':>9}")
for row in rows:
    path = os.path.join(directory, row["name"] + ".step")
    shape = Part.read(path)
    v, a = float(row["volume"]), float(row["area"])
    ok = shape.isValid()
    # Solids from a file are valid only if they are closed and correctly oriented.
    dv = abs(shape.Volume - v) / abs(v)
    da = abs(shape.Area - a) / abs(a)
    bad = (not ok) or dv > 1e-3 or da > 1e-3
    failures += bad
    say(f"{row['name']:<20} {str(ok):<6} {v:>14.4f} {shape.Volume:>14.4f} {dv:>9.2e} {da:>9.2e} {len(shape.Solids):>6} {len(shape.Faces):>4}/{row['faces']:<4}{'  <-- FAIL' if bad else ''}")
say(f"{len(rows) - failures} of {len(rows)} models passed")
report.close()
sys.exit(0)
