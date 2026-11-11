#!/usr/bin/env python3
# One picture of all the real parts, to look through on a phone, from a folder check_real_parts.sh has filled.
# Usage: real_parts_sheet.py <folder> <out.png>. Needs Pillow. Nothing else in this repository does, and nothing depends on this.
import glob, json, os, sys
from PIL import Image, ImageDraw

folder, out = sys.argv[1], sys.argv[2]
COLUMNS, TILE_W, TILE_H, LABEL = 4, 300, 225, 34
stems = sorted(os.path.basename(p)[:-4] for p in glob.glob(os.path.join(folder, "*.png")) if "__change" not in p)
rows = (len(stems) + COLUMNS - 1) // COLUMNS
sheet = Image.new("RGB", (COLUMNS * TILE_W, rows * (TILE_H + LABEL)), (246, 246, 242))
draw = ImageDraw.Draw(sheet)
for k, stem in enumerate(stems):
    x, y = (k % COLUMNS) * TILE_W, (k // COLUMNS) * (TILE_H + LABEL)
    picture = Image.open(os.path.join(folder, stem + ".png")).convert("RGB").resize((TILE_W, TILE_H), Image.LANCZOS)
    sheet.paste(picture, (x, y + LABEL))
    result = json.load(open(os.path.join(folder, stem + ".result.json")))
    whole = result["whole"]
    draw.rectangle([x, y, x + TILE_W - 1, y + LABEL - 1], fill=(226, 240, 226) if whole else (246, 214, 210))
    draw.text((x + 6, y + 3), stem.split("_", 1)[0] + "  " + result["name"][:38], fill=(20, 20, 20))
    if not whole:
        first = next((s for s in result["steps"] if s["state"] != "ok"), None)
        draw.text((x + 6, y + 18), (first["label"] if first else "")[:44], fill=(150, 30, 20))
sheet.save(out)
print(f"{len(stems)} parts in {out}")
