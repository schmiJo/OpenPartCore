#!/usr/bin/env python3
"""Builds the browser viewer into dist/ as single self-contained HTML files.

dist/index.html     stand-alone page, for any static host or a local server
dist/artifact.html  page fragment for the artifact host, which adds its own skeleton

The kernel is embedded as base64, so each file has no external files to fetch.
"""
import base64, pathlib, subprocess, sys

root = pathlib.Path(__file__).resolve().parent.parent
subprocess.run(
    ["cargo", "build", "--profile", "wasm", "--target", "wasm32-unknown-unknown", "-p", "coreshape-wasm"],
    cwd=root, check=True,
)
wasm = (root / "target/wasm32-unknown-unknown/wasm/coreshape_wasm.wasm").read_bytes()
page = (root / "web/viewer.html").read_text()
b64 = base64.b64encode(wasm).decode()
dist = root / "dist"
dist.mkdir(exist_ok=True)

head = (
    '<!doctype html>\n<html lang="en">\n<meta charset="utf-8">\n'
    '<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">\n'
)
fill = lambda host: page.replace("__WASM_BASE64__", b64).replace("__HOST__", host)
(dist / "index.html").write_text(head + fill("standalone"))
(dist / "artifact.html").write_text(fill("artifact"))
print(f"wasm {len(wasm)/1024:.0f} KiB, page {(dist / 'index.html').stat().st_size/1024:.0f} KiB")
