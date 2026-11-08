# Reads back what this kernel wrote after cutting each imported part. Runs under FreeCADCmd.
import glob, os, Part

kr = os.environ["KR"]
report = open(kr + "/back.csv", "w")
for path in sorted(glob.glob(kr + "/out/*.step")):
    shape = Part.read(path)
    report.write("%s,%s,%.9f\n" % (os.path.basename(path)[:-5], shape.isValid(), shape.Volume))
report.close()
