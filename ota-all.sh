#!/usr/bin/env bash
# OTA-flash every board listed in BOARDS.
# Edit the IPs below after the home setup so they match your three Arduinos.
#
# Credentials: OTA_PASSWORD must come from your environment, or — as a
# convenience for local development — be picked up from a gitignored Secrets.h
# in the same directory. The script never bakes in a default password.
set -euo pipefail

DEFAULT_BOARDS=(
  192.168.8.138
  192.168.8.213
  192.168.8.198
)
BOARDS=()

FQBN="arduino:renesas_uno:unor4wifi"
BUILD_DIR="/tmp/adafruit-16-servo-build"
BIN="$BUILD_DIR/adafruit_16_servo.ino.bin"
CONNECT_TIMEOUT="${CONNECT_TIMEOUT:-3}"
# Set above the firmware's HARD_CAP_MS (120s, see otaReceive in
# adafruit_16_servo.ino) so the board's own receive loop terminates first and
# returns a clean HTTP 500 ("upload incomplete") instead of curl aborting
# mid-transfer and leaving a half-open socket on the board (servo-6lc).
UPLOAD_TIMEOUT="${UPLOAD_TIMEOUT:-150}"
PARALLEL=1

usage() {
  cat <<'EOF'
Usage: ./ota-all.sh [--serial|--parallel] [board-ip ...]

Uploads in parallel by default, matching the original script behavior. Each
board gets its own /tmp/ota-<ip>.log with curl timing diagnostics.

Options:
  --serial     Upload one board at a time for controlled diagnostics.
  --parallel   Upload to all boards concurrently (default).

If one or more board IPs are provided, upload only to those boards.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --parallel) PARALLEL=1; shift ;;
    --serial|--sequential) PARALLEL=0; shift ;;
    -h|--help) usage; exit 0 ;;
    -*) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
    *)
      # Accept either a bare IP/host or a full URL and normalize to host only,
      # so `./ota-all.sh http://192.168.8.198/` doesn't build a bogus curl URL
      # and a /tmp/ota-http://.../.log path that can't be created (servo-6lc).
      board="$1"
      board="${board#http://}"; board="${board#https://}"  # strip scheme
      board="${board%%/*}"                                  # strip path/trailing slash
      BOARDS+=("$board"); shift ;;
  esac
done
if [[ ${#BOARDS[@]} -eq 0 ]]; then
  BOARDS=("${DEFAULT_BOARDS[@]}")
fi

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

upload_one() {
  local ip="$1"
  # POST to port 80 /ota — the custom handler in Web.cpp. The
  # ArduinoOTA library's port-65280 /sketch endpoint returns 500 on
  # the UNO R4 WiFi (WiFiS3 stack interaction). Match what the
  # servo_controller.html upload uses.
  if curl -sS --fail \
      --connect-timeout "$CONNECT_TIMEOUT" --max-time "$UPLOAD_TIMEOUT" \
      --write-out "\nCURL http_code=%{http_code} time_connect=%{time_connect} time_starttransfer=%{time_starttransfer} time_total=%{time_total} size_upload=%{size_upload} speed_upload=%{speed_upload}\n" \
      -X POST \
      -H "Content-Type: application/octet-stream" \
      -H "Expect:" \
      --data-binary @"$BIN" \
      "http://$ip/ota?p=${PW_ENCODED}" > "/tmp/ota-$ip.log" 2>&1; then
    echo "  OK   $ip"
    return 0
  else
    echo "  FAIL $ip (see /tmp/ota-$ip.log)"
    return 1
  fi
}

failures=0
if (( PARALLEL )); then
  echo "Uploading to ${#BOARDS[@]} boards in parallel (${UPLOAD_TIMEOUT}s timeout)..."
  pids=()
  for ip in "${BOARDS[@]}"; do
    ( upload_one "$ip" ) &
    pids+=($!)
  done
  for pid in "${pids[@]}"; do
    if ! wait "$pid"; then
      failures=$((failures + 1))
    fi
  done
else
  echo "Uploading to ${#BOARDS[@]} boards sequentially (${UPLOAD_TIMEOUT}s timeout each)..."
  for ip in "${BOARDS[@]}"; do
    if ! upload_one "$ip"; then
      failures=$((failures + 1))
    fi
  done
fi

if (( failures > 0 )); then
  echo "Done with $failures failure(s)." >&2
  exit 1
fi

echo "Done."
