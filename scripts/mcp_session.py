#!/usr/bin/env python3
"""Models a part through the MCP server exactly as an agent would, over its pipes, and saves what
it sees. It reads names back from the server and never computes them. Usage: mcp_session.py <folder>"""
import base64, json, pathlib, subprocess, sys

root = pathlib.Path(__file__).resolve().parent.parent
out = pathlib.Path(sys.argv[1]).resolve()
out.mkdir(parents=True, exist_ok=True)
binary = sys.argv[2] if len(sys.argv) > 2 else str(root / "target/release/coreshape-mcp")
server = subprocess.Popen([binary, "--root", str(out)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
next_id = 0


def rpc(method, params):
    global next_id
    next_id += 1
    server.stdin.write(json.dumps({"jsonrpc": "2.0", "id": next_id, "method": method, "params": params}) + "\n")
    server.stdin.flush()
    return json.loads(server.stdout.readline())


def tool(tool_name, /, **args):
    result = rpc("tools/call", {"name": tool_name, "arguments": args})["result"]
    texts = [c["text"] for c in result["content"] if c["type"] == "text"]
    if result.get("isError"):
        raise SystemExit(f"{tool_name} refused: {texts}")
    return result["content"], "\n".join(texts)


def solid(answer):
    return json.loads(answer[answer.index("{"):])


def look(picture, /, **args):
    content, text = tool("render", **args)
    image = next(c for c in content if c["type"] == "image")
    (out / f"{picture}.png").write_bytes(base64.b64decode(image["data"]))
    print(f"  saw {picture}: {json.loads(text)['visible_faces'][:6]}")


print(rpc("initialize", {"protocolVersion": "2025-06-18", "capabilities": {}, "clientInfo": {"name": "session", "version": "1"}})["result"]["serverInfo"])
print(len(rpc("tools/list", {})["result"]["tools"]), "tools")

for name, value in [("width", 60), ("depth", 40), ("height", 45), ("wall", 8), ("hole", 9)]:
    tool("parameter_set", name=name, value=value)

# An L-shaped section, from a solved sketch: it starts crooked, and the constraints straighten it.
lines = [([0, 1], [39, 0]), ([40, 1], [41, 9]), ([40, 8], [9, 9]), ([8, 8], [9, 44]), ([8, 45], [1, 46]), ([0, 45], [0, 0])]
entities = [{"id": i + 1, "type": "line", "start": a, "end": b} for i, (a, b) in enumerate(lines)]
constraints = [{"id": i + 1, "type": "coincident", "a": f"{i + 1}.end", "b": f"{(i + 1) % 6 + 1}.start"} for i in range(6)]
extra = [
    {"type": "coincident", "a": "1.start", "b": "origin"},
    {"type": "horizontal", "a": "1"}, {"type": "vertical", "a": "2"}, {"type": "horizontal", "a": "3"},
    {"type": "vertical", "a": "4"}, {"type": "horizontal", "a": "5"}, {"type": "vertical", "a": "6"},
    {"type": "distance", "a": "1", "value": "depth"}, {"type": "distance", "a": "2", "value": "wall"},
    {"type": "distance", "a": "5", "value": "wall"}, {"type": "distance", "a": "6", "value": "height"},
]
for k, c in enumerate(extra):
    constraints.append({"id": 7 + k, **c})
sketch = {"format": "coreshape-sketch", "version": 1, "entities": entities, "constraints": constraints}
_, text = tool("sketch_solve", sketch=sketch, parameters={"width": 60, "depth": 40, "height": 45, "wall": 8})
print("  sketch:", {k: v for k, v in json.loads(text)["analysis"].items() if k == "dof"}, "freedom left")

tool("feature_add", name="Section", feature={"type": "constrained_sketch", "plane": {"origin": [0, 0, 0], "z": [0, -1, 0], "x": [1, 0, 0]}, "sketch": sketch})
_, text = tool("feature_add", name="Body", feature={"type": "extrude", "sketch": 1, "start": 0, "length": "width"})
print("  after extrude:", solid(text[text.index("{"):])["solid"]["volume"])
look("1_extruded", labels=True, colors="faces")

# Ask what the inner corner is called, then round it.
edges = json.loads(tool("edges_list")[1])
inner = next(e for e in edges if set(e["name"].split("|")) == {"f2:side.3", "f2:side.4"})
print("  inner corner:", inner["name"], "length", inner["length"])
tool("feature_add", name="Inner round", feature={"type": "fillet", "edges": inner["select"], "radius": 6})

# Round the free corners of the base, found from the same list.
edges = json.loads(tool("edges_list")[1])
base = [e for e in edges if "f2:side.2" in e["name"] and e["curve"] == "line" and abs(e["direction"][2]) > 0.99]
print("  base corners:", len(base))
_, text = tool("feature_add", name="Corners", feature={"type": "fillet", "edges": {"any": [e["select"] for e in base]}, "radius": 5})
look("2_filleted")

# Drill the base and the wall. The part runs from y = 0 to y = -width, since the section's plane
# faces -y and the extrude follows it.
tool("feature_add", name="Base holes", feature={"type": "sketch", "regions": [
    {"outer": {"type": "circle", "cx": 26, "cy": "-width * 0.25", "r": "hole / 2"}},
    {"outer": {"type": "circle", "cx": 26, "cy": "-width * 0.75", "r": "hole / 2"}}]})
_, text = tool("feature_add", name="Drill base", feature={"type": "extrude", "sketch": 5, "start": -1, "length": "wall + 2", "mode": "remove"})
tool("feature_add", name="Wall hole", feature={"type": "sketch", "plane": {"origin": [-1, 0, 0], "z": [1, 0, 0], "x": [0, 1, 0]},
                                                "regions": [{"outer": {"type": "circle", "cx": "-width / 2", "cy": 28, "r": "hole / 2"}}]})
_, text = tool("feature_add", name="Drill wall", feature={"type": "extrude", "sketch": 7, "start": 0, "length": "wall + 2", "mode": "remove"})
report = solid(text[text.index("{"):])
print("  finished:", report["solid"])
look("3_finished", width=1000, height=720)
look("4_four_views", view="four", width=1100, height=900)
look("5_hole_rims", highlight_faces=["f6:side.0", "f6:region.1.side.0", "f8:side.0"], colors="plain", width=800, height=600)

# Change dimensions: every name should still find its edge.
_, text = tool("parameter_set", name="width", value=90)
print("  wider:", solid(text)["solid"]["volume"], "failed:", solid(text).get("failed"))
_, text = tool("parameter_set", name="wall", value=11)
print("  thicker:", solid(text)["solid"]["volume"], "failed:", solid(text).get("failed"))
look("6_changed", width=1000, height=720)

# A hole in the wrong place cuts nothing, and the server says so.
_, text = tool("feature_add", name="Miss", feature={"type": "solid", "shape": "cylinder", "radius": 3, "height": 5, "at": {"origin": [200, 0, 0]}, "mode": "remove"})
print("  a cut that misses:", solid(text[text.index("{"):]).get("notes"))
tool("feature_delete", id=json.loads(tool("features_list")[1])[-1]["id"])

# Ask for too much, and hear why.
_, text = tool("feature_edit", id=4, feature={"type": "fillet", "edges": {"any": [e["select"] for e in base]}, "radius": 60})
print("  too big a radius:", solid(text).get("failed"))
tool("feature_edit", id=4, feature={"type": "fillet", "edges": {"any": [e["select"] for e in base]}, "radius": 5})
tool("document_save", path="bracket.coreshape")
tool("export", format="step", path="bracket.step")
tool("export", format="stl", path="bracket.stl")
print("  files:", sorted(p.name for p in out.iterdir()))
server.stdin.close()
server.wait(timeout=5)
print("exit code", server.returncode)
