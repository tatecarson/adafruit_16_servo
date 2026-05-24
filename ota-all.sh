#!/usr/bin/env bash
# OTA-flash every board listed in BOARDS in parallel.
# Edit the IPs below after the home setup so they match your three Arduinos.
#
# Credentials: OTA_PASSWORD must come from your environment, or — as a
# convenience for local development — be picked up from a gitignored Secrets.h
# in the same directory. The script never bakes in a default password.
set -euo pipefail

BOARDS=(
  192.168.8.138
  192.168.8.213
  192.168.8.198
)

FQBN="arduino:renesas_uno:unor4wifi"
BUILD_DIR="/tmp/adafruit-16-servo-build"
BIN="$BUILD_DIR/adafruit_16_servo.ino.bin"

cd "$(dirname "$0")"

# Pull OTA_PASSWORD from Secrets.h if not already in the environment.
if [[ -z "${OTA_PASSWORD:-}" && -f adafruit_16_servo/Secrets.h ]]; then
  OTA_PASSWORD=$(awk -F'"' '/^[[:space:]]*#define[[:space:]]+OTA_PASSWORD[[:space:]]/ {print $2; exit}' adafruit_16_servo/Secrets.h)
fi
: "${OTA_PASSWORD:?OTA_PASSWORD not set. Export it, or define it in Secrets.h.}"

# URL-encode the password for use in the query string. The custom /ota
# handler in Web.cpp uses `?p=<password>` rather than HTTP Basic auth.
PW_ENCODED=$(python3 -c "import urllib.parse,sys; print(urllib.parse.quote(sys.argv[1], safe=''))" "$OTA_PASSWORD")

# arduino-cli requires the sketch directory name to match the .ino base
# name. Look for the .ino in two likely places:
#   1. ./<SKETCH_NAME>.ino — pwd is already the sketch dir (e.g. after staging).
#   2. ./<SKETCH_NAME>/<SKETCH_NAME>.ino — pwd is the repo root, sketch is in a
#      subdir of the same name (normal layout for this repo; the old basename
#      check passed here but pointed arduino-cli at the wrong level — servo-v0f).
# If neither, stage into a temp dir whose name matches the sketch.
SKETCH_NAME="adafruit_16_servo"
SKETCH_DIR=""
if [[ -f "./${SKETCH_NAME}.ino" ]]; then
  SKETCH_DIR="$(pwd)"
elif [[ -f "./${SKETCH_NAME}/${SKETCH_NAME}.ino" ]]; then
  SKETCH_DIR="$(pwd)/${SKETCH_NAME}"
else
  # Worktree with a different basename, or pwd unrelated to the repo. Stage.
  STAGE="/tmp/ota-stage/${SKETCH_NAME}"
  rm -rf "$STAGE"
  mkdir -p "$STAGE"
  shopt -s nullglob
  cp -R adafruit_16_servo/*.ino adafruit_16_servo/*.h adafruit_16_servo/*.cpp "$STAGE"/
  shopt -u nullglob
  SKETCH_DIR="$STAGE"
fi

echo "Compiling..."
arduino-cli compile --fqbn "$FQBN" --build-path "$BUILD_DIR" "$SKETCH_DIR"

if [[ ! -f "$BIN" ]]; then
  echo "Build artifact not found at $BIN" >&2
  exit 1
fi

echo "Uploading to ${#BOARDS[@]} boards in parallel..."
pids=()
for ip in "${BOARDS[@]}"; do
  (
    # POST to port 80 /ota — the custom handler in Web.cpp. The
    # ArduinoOTA library's port-65280 /sketch endpoint returns 500 on
    # the UNO R4 WiFi (WiFiS3 stack interaction). Match what the
    # servo_controller.html upload uses.
    if curl -sS --fail \
        --connect-timeout 3 --max-time 60 \
        -X POST \
        -H "Content-Type: application/octet-stream" \
        --data-binary @"$BIN" \
        "http://$ip/ota?p=${PW_ENCODED}" > "/tmp/ota-$ip.log" 2>&1; then
      echo "  OK   $ip"
      exit 0
    else
      echo "  FAIL $ip (see /tmp/ota-$ip.log)"
      exit 1
    fi
  ) &
  pids+=($!)
done

failures=0
for pid in "${pids[@]}"; do
  if ! wait "$pid"; then
    failures=$((failures + 1))
  fi
done

if (( failures > 0 )); then
  echo "Done with $failures failure(s)." >&2
  exit 1
fi

echo "Done."
