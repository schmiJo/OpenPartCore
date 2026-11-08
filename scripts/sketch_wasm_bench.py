#!/usr/bin/env python3
"""Runs the sketch solver's speed benchmark as WebAssembly in headless Chrome, in real time."""
import base64, pathlib, re, subprocess, sys, tempfile

root = pathlib.Path(__file__).resolve().parent.parent
target = sys.argv[1] if len(sys.argv) > 1 else str(root / "target")
subprocess.run(["cargo", "build", "--profile", "wasm", "-p", "coreshape-sketch", "--example", "wasm_bench",
                "--target", "wasm32-unknown-unknown", "--target-dir", target], check=True, cwd=root, capture_output=True)
wasm = pathlib.Path(target) / "wasm32-unknown-unknown/wasm/examples/wasm_bench.wasm"
data = base64.b64encode(wasm.read_bytes()).decode()
page = f"""<!doctype html><pre id=out>running</pre><script>
const bytes = Uint8Array.from(atob("{data}"), c => c.charCodeAt(0));
WebAssembly.instantiate(bytes, {{env: {{now: () => performance.now()}}}}).then(({{instance}}) => {{
  const x = instance.exports;
  const p = x.run();
  document.getElementById("out").textContent = new TextDecoder().decode(new Uint8Array(x.memory.buffer, p, x.len()));
}}).catch(e => {{ document.getElementById("out").textContent = "FAILED " + e; }});
</script>"""
with tempfile.TemporaryDirectory() as tmp:
    html = pathlib.Path(tmp) / "bench.html"
    html.write_text(page)
    result = subprocess.run(["google-chrome", "--headless=new", "--no-sandbox", "--disable-gpu", f"--user-data-dir={tmp}/profile",
                             "--timeout=90000", "--dump-dom", html.as_uri()], capture_output=True, text=True, timeout=240)
match = re.search(r'<pre id="out">(.*?)</pre>', result.stdout, re.S)
print(f"wasm module: {wasm.stat().st_size // 1024} KiB")
print(f"{'workload':<14} {'entities':>8} {'constraints':>11} {'free':>5} {'solve':>9} {'analyze':>9} {'begin':>9} {'move med':>9} {'move p95':>9} {'worst':>9}")
print(match.group(1) if match else result.stdout[-400:])
