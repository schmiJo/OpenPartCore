#!/usr/bin/env python3
"""Drives the built viewer in headless Chrome and reports what happened.

It uses the page's real controls: slider input events, clicks on feature rows, and
pointer events on the rollback bar. It needs dist/index.html from build_web.py.
"""
import http.server, json, pathlib, subprocess, sys, tempfile, threading

root = pathlib.Path(__file__).resolve().parent.parent
page = (root / "dist/index.html").read_text()

driver = r"""
<script>
(async () => {
  const log = [];
  const out = document.createElement("pre");
  out.id = "selftest";
  out.hidden = true;
  document.body.append(out);
  // The log is written after every check, so a stall still shows how far it got.
  const check = (name, ok, detail) => {
    log.push({name, ok: !!ok, detail: detail === undefined ? "" : String(detail)});
    out.textContent = JSON.stringify(log);
    fetch("/progress", { method: "POST", body: JSON.stringify(log) }).catch(() => {});
  };
  const frame = () => new Promise((r) => {
    let done = false;
    const finish = () => { if (!done) { done = true; r(); } };
    requestAnimationFrame(() => setTimeout(finish, 0));
    setTimeout(finish, 300);
  });
  try {
    while (!window.__viewer) { if (window.__fatal) throw new Error(window.__fatal); await new Promise((r) => setTimeout(r, 20)); }
    const { catalog, selectModel, settled } = window.__viewer;
    const settle = () => settled();
    check("runs the kernel where it should", window.__viewer.usesWorker === WANT_WORKER, `worker: ${window.__viewer.usesWorker}`);
    for (let i = 0; i < catalog.length; i++) {
      await selectModel(i);
      const b = window.__lastBuild;
      check(`${catalog[i].id} builds at defaults`, b.info.ok && b.info.failed === null && b.info.openEdges === 0, `${b.ms.toFixed(1)} ms`);
    }

    // How long does moving each model's first and last slider take?
    for (let i = 0; i < catalog.length; i++) {
      await selectModel(i);
      const m = catalog[i];
      const times = [];
      for (const p of [m.params[m.params.length - 1], m.params[0]]) {
        const original = p.value;
        p.value = original + (original + p.step <= p.max ? p.step : -p.step);
        await window.__viewer.rebuild(false);
        times.push(window.__lastBuild.ms.toFixed(0));
        p.value = original;
        await window.__viewer.rebuild(false);
      }
      check(`timing ${m.id}`, true, `last slider ${times[0]} ms, first slider ${times[1]} ms`);
    }
    // The same, as drafts: what each step of a drag costs.
    for (let i = 0; i < catalog.length; i++) {
      await selectModel(i);
      const m = catalog[i];
      const times = [];
      const values = m.params.map((p) => p.value);
      for (const at of [m.params.length - 1, 0]) {
        const p = m.params[at];
        const moved = values.slice();
        moved[at] = p.value + (p.value + p.step <= p.max ? p.step : -p.step);
        const r = await window.__viewer.kernel.build(i, moved, -1, true);
        times.push(r.ms.toFixed(0));
        await window.__viewer.kernel.build(i, values, -1, true);
      }
      check(`drag step ${m.id}`, true, `last slider ${times[0]} ms, first slider ${times[1]} ms`);
    }

    await selectModel(catalog.findIndex((m) => m.id === "bracket"));
    const startVolume = window.__lastBuild.info.volume;
    const count = window.__lastBuild.info.features.length;

    // A slider moves the model, and only rebuilds what depends on it.
    const hole = document.getElementById("p-bracket-rim");
    hole.value = "2";
    hole.dispatchEvent(new Event("input", {bubbles: true}));
    await settle();
    let b = window.__lastBuild;
    check("a slider that has stopped gets the full result", b.info.draft === false && b.info.volume > 0);
    check("slider rebuilds through the kernel", b.info.volume < startVolume, `${startVolume} -> ${b.info.volume}`);
    check("slider reuses unaffected features", b.info.rebuilt < count, `${b.info.rebuilt} of ${count} rebuilt`);

    // Tapping a feature rolls back to it.
    const rows = () => [...document.getElementById("featureRows").children];
    rows()[1].click();
    await settle();
    b = window.__lastBuild;
    check("tapping a feature rolls back", b.info.features.slice(2).every((f) => f.state === "rolledback"), JSON.stringify(b.info.features.map((f) => f.state)));
    check("rollback shows the plain extrusion", b.info.faces === 8, `${b.info.faces} faces`);

    // Dragging the bar with pointer events moves it without replacing it.
    const bar = document.getElementById("rollback");
    const before = bar;
    const rect = (el) => el.getBoundingClientRect();
    const target = rect(rows()[4]);
    const ev = (type, y) => new PointerEvent(type, {bubbles: true, pointerId: 7, clientX: rect(bar).left + 20, clientY: y, pointerType: "touch"});
    bar.setPointerCapture = () => {};
    bar.hasPointerCapture = () => true;
    bar.dispatchEvent(ev("pointerdown", rect(bar).top));
    bar.dispatchEvent(ev("pointermove", target.top + target.height));
    await settle();
    b = window.__lastBuild;
    check("dragging the bar rolls to that feature", b.info.features[4].state !== "rolledback" && b.info.features[5].state === "rolledback", JSON.stringify(b.info.features.map((f) => f.state)));
    check("the bar element survives a drag", document.getElementById("rollback") === before);

    // Back to the end.
    rows()[count - 1].click();
    await settle();
    b = window.__lastBuild;
    check("tapping the last feature rolls forward", b.info.features.every((f) => f.state !== "rolledback"));

    // While dragging, drafts arrive: a lighter mesh with no volume. Then the full result, with the same solid.
    await selectModel(catalog.findIndex((m) => m.id === "plate"));
    const fullTris = window.__lastBuild.info.triangles;
    const plateTop = document.getElementById("p-plate-top");
    window.__buildLog = [];
    for (let i = 0; i < 6; i++) {
      plateTop.value = String(1 + i * 0.25);
      plateTop.dispatchEvent(new Event("input", {bubbles: true}));
      await new Promise((r) => setTimeout(r, 40));
    }
    await settle();
    const builds = window.__buildLog;
    window.__buildLog = null;
    const drafts = builds.filter((x) => x.draft);
    check("a drag shows drafts", drafts.length > 0, `${drafts.length} of ${builds.length} builds`);
    check("a draft is lighter", drafts.every((x) => x.triangles < fullTris), `${drafts[0] ? drafts[0].triangles : "none"} of ${fullTris}`);
    const after = window.__lastBuild.info;
    check("the drag settles on the full result", after.draft === false && after.volume > 0 && builds[builds.length - 1].draft === false, `${after.triangles} triangles, volume ${after.volume}`);
    check("nothing is left waiting", window.__lastBuild.info.openEdges === 0);
    // The very last value given is what's shown, however fast they came.
    plateTop.value = "2";
    plateTop.dispatchEvent(new Event("input", {bubbles: true}));
    plateTop.value = "0.5";
    plateTop.dispatchEvent(new Event("input", {bubbles: true}));
    await settle();
    const small = window.__lastBuild.info.volume;
    plateTop.value = "2";
    plateTop.dispatchEvent(new Event("input", {bubbles: true}));
    await settle();
    check("the newest value wins", window.__lastBuild.info.volume < small, `${window.__lastBuild.info.volume} under ${small}`);

    // A refused case shows why, and the rest of the page stays alive.
    await selectModel(catalog.findIndex((m) => m.id === "stress"));
    const caseSlider = document.getElementById("p-stress-case");
    caseSlider.value = "13";
    caseSlider.dispatchEvent(new Event("input", {bubbles: true}));
    await settle();
    b = window.__lastBuild;
    const banner = document.getElementById("failure");
    check("the known limit is refused, not faked", b.info.failed === 1 && b.info.features[1].state === "failed", JSON.stringify(b.info.features[1].error));
    check("the refusal is shown to the user", !banner.hidden && banner.textContent.length > 20, banner.textContent);
    caseSlider.value = "0";
    caseSlider.dispatchEvent(new Event("input", {bubbles: true}));
    await settle();
    check("the next good case recovers", window.__lastBuild.info.failed === null && banner.hidden);

    // Every stress case other than the known limit must work.
    for (let c = 0; c < 13; c++) {
      caseSlider.value = String(c);
      caseSlider.dispatchEvent(new Event("input", {bubbles: true}));
      await settle();
      const r = window.__lastBuild;
      check(`stress: ${catalog[catalog.findIndex((m) => m.id === "stress")].params[0].choices[c]}`, r.info.failed === null && r.info.openEdges === 0, r.info.features.map((f) => f.error).join("") || `V=${r.info.volume.toFixed(2)} shells=${r.info.shells}`);
    }

    // Exports produce bytes in the right formats.
    const k = window.__viewer.kernel;
    const step = new TextDecoder().decode(await k.exportFile("step"));
    check("STEP export", step.startsWith("ISO-10303-21;") && step.trim().endsWith("END-ISO-10303-21;"), `${step.length} bytes`);
    const stl = await k.exportFile("stl");
    check("STL export", stl.length > 84 && (stl.length - 84) % 50 === 0, `${stl.length} bytes`);
    const saved = JSON.parse(new TextDecoder().decode(await k.exportFile("coreshape")));
    check("model file export", saved.format === "coreshape-document" && saved.features.length > 0, `${saved.features.length} features`);
    const zip = makeZip("a.step", new Uint8Array([1, 2, 3]));
    check("zip wrapper", zip[0] === 0x50 && zip[1] === 0x4b && zip.length === 30 + 6 + 3 + 46 + 6 + 22, `${zip.length} bytes`);
  } catch (e) {
    check("the self test ran to the end", false, e && e.stack ? e.stack : e);
  }
  check("the self test ran to the end", true);
  await fetch("/done", { method: "POST", body: JSON.stringify(log) });
})();
</script>
"""



def run_once(query, want_worker):
    """Runs the checks once, in real time, as a worker doesn't run under Chrome's virtual time. The page sends
    its log back to a small local server. A host may forbid workers, so the fallback gets the same checks."""
    html = (page + driver.replace("WANT_WORKER", "true" if want_worker else "false")).encode()
    finished = threading.Event()
    results = {}

    class Handler(http.server.BaseHTTPRequestHandler):
        def do_GET(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(html)

        def do_POST(self):
            body = self.rfile.read(int(self.headers["Content-Length"]))
            # Requests can overtake each other, so nothing may replace the final report.
            if self.path == "/done" or not finished.is_set():
                results["log"] = body
            self.send_response(204)
            self.end_headers()
            if self.path == "/done":
                finished.set()

        def log_message(self, *args):
            pass

    server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    with tempfile.TemporaryDirectory() as tmp:
        chrome = subprocess.Popen(
            ["google-chrome", "--headless=new", "--no-sandbox", "--use-angle=swiftshader", "--enable-unsafe-swiftshader",
             f"--user-data-dir={tmp}/profile", "--window-size=1100,1400", f"http://127.0.0.1:{server.server_port}/{query}"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        ended = finished.wait(timeout=180)
        chrome.terminate()
        chrome.wait(timeout=20)
    server.shutdown()
    if "log" not in results:
        print("The page never reported anything.")
        return False
    log = json.loads(results["log"])
    bad = [c for c in log if not c["ok"]]
    if not ended or not log or log[-1]["name"] != "the self test ran to the end":
        bad.append({"name": "the self test stopped early", "ok": False, "detail": ""})
    for c in log:
        print(("ok   " if c["ok"] else "FAIL ") + c["name"] + (f"  [{c['detail'][:150]}]" if c["detail"] else ""))
    print(f"{len(log) - len(bad)} of {len(log)} checks passed")
    return not bad


print("--- with the kernel in a worker")
first = run_once("", True)
print("--- with the kernel in the page, for hosts that don't allow workers")
second = run_once("?worker=0", False)
sys.exit(0 if first and second else 1)
