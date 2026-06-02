#!/usr/bin/env bash
# Compile the Arduino sketch into a browser-loadable firmware/ folder.
# Serve the repo with `./compile-firmware.sh --serve`, then open:
#   http://127.0.0.1:4173/servo_controller.html
set -euo pipefail

FQBN="${FQBN:-arduino:renesas_uno:unor4wifi}"
BUILD_DIR="${BUILD_DIR:-/tmp/adafruit-16-servo-build}"
OUT_DIR="${OUT_DIR:-firmware}"
PORT="${PORT:-4173}"
SERVE=0

usage() {
  cat <<'EOF'
Usage: ./compile-firmware.sh [--serve] [--port N]

Compiles adafruit_16_servo with arduino-cli, copies the .bin into ./firmware/,
and writes ./firmware/manifest.json for the browser OTA page's "Use compiled
bin" button.

Options:
  --serve     After compiling, serve the repo at http://127.0.0.1:4173/
  --port N    Port for --serve (default: 4173)
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --serve) SERVE=1; shift ;;
    --port) PORT="${2:?missing port}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

cd "$(dirname "$0")"

SKETCH_NAME="adafruit_16_servo"
if [[ -f "./${SKETCH_NAME}.ino" ]]; then
  SKETCH_DIR="$(pwd)"
elif [[ -f "./${SKETCH_NAME}/${SKETCH_NAME}.ino" ]]; then
  SKETCH_DIR="$(pwd)/${SKETCH_NAME}"
else
  echo "Could not find ${SKETCH_NAME}.ino" >&2
  exit 1
fi

BIN="$BUILD_DIR/${SKETCH_NAME}.ino.bin"
OUT_BIN="$OUT_DIR/${SKETCH_NAME}.ino.bin"
MANIFEST="$OUT_DIR/manifest.json"

echo "Compiling $SKETCH_NAME..."
arduino-cli compile --fqbn "$FQBN" --build-path "$BUILD_DIR" "$SKETCH_DIR"

if [[ ! -f "$BIN" ]]; then
  echo "Build artifact not found at $BIN" >&2
  exit 1
fi

mkdir -p "$OUT_DIR"
cp "$BIN" "$OUT_BIN"

FW_BUILD=$(grep -oE '#define[[:space:]]+FW_BUILD[[:space:]]+"[^"]*"' \
  "$SKETCH_DIR/${SKETCH_NAME}.ino" 2>/dev/null | grep -oE '"[^"]*"' | tr -d '"' || true)

python3 - "$OUT_BIN" "$MANIFEST" "$FQBN" "$FW_BUILD" <<'PY'
import hashlib
import json
import pathlib
import sys
from datetime import datetime, timezone

bin_path = pathlib.Path(sys.argv[1])
manifest_path = pathlib.Path(sys.argv[2])
fqbn = sys.argv[3]
fw_build = sys.argv[4]
data = bin_path.read_bytes()
manifest = {
    "filename": bin_path.name,
    "size": len(data),
    "sha256": hashlib.sha256(data).hexdigest(),
    "fwBuild": fw_build,
    "fqbn": fqbn,
    "builtAt": datetime.now(timezone.utc).isoformat(),
}
manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
PY

echo "Wrote $OUT_BIN"
echo "Wrote $MANIFEST"
echo "Firmware: ${FW_BUILD:-unknown} · $(wc -c < "$OUT_BIN" | tr -d ' ') bytes"

if (( SERVE )); then
  echo "Serving http://127.0.0.1:$PORT/servo_controller.html"
  python3 -m http.server "$PORT" --bind 127.0.0.1
fi
