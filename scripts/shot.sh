#!/bin/sh
# Screenshots the viewer with headless Chrome: shot.sh <out.png> [hash] [width] [height] [query]
# Uses a software WebGL renderer, so it needs no GPU.
out="$1"; hash="${2:-}"; w="${3:-1200}"; h="${4:-800}"; query="${5:-}"
root="$(cd "$(dirname "$0")/.." && pwd)"
python3 -m http.server 8173 --bind 127.0.0.1 --directory "$root/dist" >/dev/null 2>&1 &
server=$!
sleep 0.5
profile="$(mktemp -d)"
google-chrome --headless=new --no-sandbox --disable-gpu-sandbox --use-angle=swiftshader --enable-unsafe-swiftshader \
  --hide-scrollbars --force-device-scale-factor="${SCALE:-1}" --window-size="$w,$h" --user-data-dir="$profile" \
  --virtual-time-budget=4000 --screenshot="$out" "http://127.0.0.1:8173/index.html?$query#$hash" >/dev/null 2>&1
kill "$server" 2>/dev/null
rm -rf "$profile"
ls -la "$out"
