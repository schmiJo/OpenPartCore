# Runs under OpenCASCADE, by way of occt.sh: reads every STEP file in a folder, each on its own, and writes what it finds.
import glob, json, os
import Part

work = os.environ["KR"]
out = {}
for path in sorted(glob.glob(os.path.join(work, "*.step"))):
    name = os.path.basename(path)
    try:
        shape = Part.Shape()
        shape.read(path)
        out[name] = {"valid": shape.isValid(), "volume": sum(s.Volume for s in shape.Solids), "area": sum(s.Area for s in shape.Solids), "solids": len(shape.Solids)}
    except Exception as why:
        out[name] = {"error": str(why)}
    with open(os.path.join(work, "occt.json"), "w") as f:
        json.dump(out, f)
