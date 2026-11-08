#!/usr/bin/env python3
"""Drives the built viewer in headless Chrome and reports what happened.

It uses the page's real controls: slider input events, clicks on feature rows, and
pointer events on the rollback bar. It needs dist/index.html from build_web.py.
"""
import json, pathlib, re, subprocess, sys, tempfile

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
  };
  const frame = () => new Promise((r) => {
    let done = false;
    const finish = () => { if (!done) { done = true; r(); } };
    requestAnimationFrame(() => setTimeout(finish, 0));
    setTimeout(finish, 300);
  });
  try {
    while (!window.__viewer) { if (window.__fatal) throw new Error(window.__fatal); await new Promise((r) => setTimeout(r, 20)); }
    const { catalog, selectModel } = window.__viewer;
    for (let i = 0; i < catalog.length; i++) {
      selectModel(i);
      const b = window.__lastBuild;
      check(`${catalog[i].id} builds at defaults`, b.info.ok && b.info.failed === null && b.info.openEdges === 0, `${b.ms.toFixed(1)} ms`);
    }

    // How long does moving each model's first and last slider take?
    for (let i = 0; i < catalog.length; i++) {
      selectModel(i);
      const m = catalog[i];
      const times = [];
      for (const p of [m.params[m.params.length - 1], m.params[0]]) {
        const original = p.value;
        p.value = original + (original + p.step <= p.max ? p.step : -p.step);
        window.__viewer.rebuild(false);
        times.push(window.__lastBuild.ms.toFixed(0));
        p.value = original;
        window.__viewer.rebuild(false);
      }
      check(`timing ${m.id}`, true, `last slider ${times[0]} ms, first slider ${times[1]} ms`);
    }

    selectModel(catalog.findIndex((m) => m.id === "bracket"));
    const startVolume = window.__lastBuild.info.volume;
    const count = window.__lastBuild.info.features.length;

    // A slider moves the model, and only rebuilds what depends on it.
    const hole = document.getElementById("p-bracket-rim");
    hole.value = "2";
    hole.dispatchEvent(new Event("input", {bubbles: true}));
    await frame(); await frame();
    let b = window.__lastBuild;
    check("slider rebuilds through the kernel", b.info.volume < startVolume, `${startVolume} -> ${b.info.volume}`);
    check("slider reuses unaffected features", b.info.rebuilt < count, `${b.info.rebuilt} of ${count} rebuilt`);

    // Tapping a feature rolls back to it.
    const rows = () => [...document.getElementById("featureRows").children];
    rows()[1].click();
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
    b = window.__lastBuild;
    check("dragging the bar rolls to that feature", b.info.features[4].state !== "rolledback" && b.info.features[5].state === "rolledback", JSON.stringify(b.info.features.map((f) => f.state)));
    check("the bar element survives a drag", document.getElementById("rollback") === before);

    // Back to the end.
    rows()[count - 1].click();
    b = window.__lastBuild;
    check("tapping the last feature rolls forward", b.info.features.every((f) => f.state !== "rolledback"));

    // A refused case shows why, and the rest of the page stays alive.
    selectModel(catalog.findIndex((m) => m.id === "stress"));
    const caseSlider = document.getElementById("p-stress-case");
    caseSlider.value = "13";
    caseSlider.dispatchEvent(new Event("input", {bubbles: true}));
    await frame(); await frame();
    b = window.__lastBuild;
    const banner = document.getElementById("failure");
    check("the known limit is refused, not faked", b.info.failed === 0, JSON.stringify(b.info.features[0].error));
    check("the refusal is shown to the user", !banner.hidden && banner.textContent.length > 20, banner.textContent);
    caseSlider.value = "0";
    caseSlider.dispatchEvent(new Event("input", {bubbles: true}));
    await frame(); await frame();
    check("the next good case recovers", window.__lastBuild.info.failed === null && banner.hidden);

    // Every stress case other than the known limit must work.
    for (let c = 0; c < 13; c++) {
      caseSlider.value = String(c);
      caseSlider.dispatchEvent(new Event("input", {bubbles: true}));
      await frame(); await frame();
      const r = window.__lastBuild;
      check(`stress: ${catalog[catalog.findIndex((m) => m.id === "stress")].params[0].choices[c]}`, r.info.failed === null && r.info.openEdges === 0, r.info.features[0].error || `V=${r.info.volume.toFixed(2)} shells=${r.info.shells}`);
    }

    // Exports produce bytes in the right formats.
    const k = window.__viewer.kernel;
    const step = new TextDecoder().decode(k.exportFile("step"));
    check("STEP export", step.startsWith("ISO-10303-21;") && step.trim().endsWith("END-ISO-10303-21;"), `${step.length} bytes`);
    const stl = k.exportFile("stl");
    check("STL export", stl.length > 84 && (stl.length - 84) % 50 === 0, `${stl.length} bytes`);
    const zip = makeZip("a.step", new Uint8Array([1, 2, 3]));
    check("zip wrapper", zip[0] === 0x50 && zip[1] === 0x4b && zip.length === 30 + 6 + 3 + 46 + 6 + 22, `${zip.length} bytes`);
  } catch (e) {
    check("the self test ran to the end", false, e && e.stack ? e.stack : e);
  }
  check("the self test ran to the end", true);
})();
</script>
"""

with tempfile.TemporaryDirectory() as tmp:
    test_page = pathlib.Path(tmp) / "selftest.html"
    test_page.write_text(page + driver)
    run = subprocess.run(
        ["google-chrome", "--headless=new", "--no-sandbox", "--use-angle=swiftshader", "--enable-unsafe-swiftshader",
         f"--user-data-dir={tmp}/profile", "--window-size=1100,1400", "--virtual-time-budget=30000",
         "--dump-dom", test_page.as_uri()],
        capture_output=True, text=True, timeout=180,
    )
match = re.search(r'<pre id="selftest"[^>]*>(.*?)</pre>', run.stdout, re.S)
if not match:
    print("The page never finished its self test.")
    print(run.stdout[-1500:])
    sys.exit(1)
log = json.loads(match.group(1).replace("&amp;", "&").replace("&lt;", "<").replace("&gt;", ">"))
bad = [c for c in log if not c["ok"]]
if not log or log[-1]["name"] != "the self test ran to the end":
    bad.append({"name": "the self test stopped early", "ok": False, "detail": ""})
for c in log:
    print(("ok   " if c["ok"] else "FAIL ") + c["name"] + (f"  [{c['detail'][:150]}]" if c["detail"] else ""))
print(f"{len(log) - len(bad)} of {len(log)} checks passed")
sys.exit(1 if bad else 0)
