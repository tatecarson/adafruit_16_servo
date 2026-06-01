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
# Sequential by DEFAULT. Parallel OTA to the whole cluster is unreliable and
# dangerous: 3 simultaneous ~118 KB transfers share the Wi-Fi airtime, which
# drops each board's throughput ~8x (8 KB/s solo -> ~1 KB/s) AND crashes the
# WiFiS3 stacks (observed: all three boards reset/"connection reset by peer"
# and went dark, needing a power-cycle — servo-6lc). A single board flashes
# cleanly in ~15s at ~8 KB/s, so sequential (~15s x N boards) is both faster in
# practice and safe. Use --parallel only if you know what you're doing.
PARALLEL=0

usage() {
  cat <<'EOF'
Usage: ./ota-all.sh [--serial|--parallel] [board-ip ...]

Uploads one board at a time by default (sequential). Each board gets its own
/tmp/ota-<ip>.log with curl timing diagnostics.

Options:
  --serial     Upload one board at a time (default).
  --parallel   Upload to all boards concurrently. NOT RECOMMENDED — parallel
               OTA saturates the shared Wi-Fi and has crashed every board in
               the cluster (needs a power-cycle to recover). See servo-6lc.

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

# Firmware build string we're flashing, read from the source we just compiled.
# Used to disambiguate the false-FAIL case: a fast successful OTA resets the
# TCP connection (the board applies + reboots) before the HTTP 200 reaches
# curl, so curl reports failure even though it worked. After a curl failure we
# poll the board's /status.json and, if it came back on THIS exact build with a
# fresh uptime, report success instead (servo-6lc). Empty => can't verify, in
# which case a curl failure is treated as a real failure (old behavior).
EXPECTED_FW=$(grep -oE '#define[[:space:]]+FW_BUILD[[:space:]]+"[^"]*"' \
  "$SKETCH_DIR/${SKETCH_NAME}.ino" 2>/dev/null | grep -oE '"[^"]*"' | tr -d '"')
VERIFY_TIMEOUT="${VERIFY_TIMEOUT:-90}"   # seconds to wait for the board to reboot
VERIFY_MAX_UPTIME_MS="${VERIFY_MAX_UPTIME_MS:-60000}"  # "fresh reboot" ceiling
if [[ -n "$EXPECTED_FW" ]]; then
  echo "Expecting firmware build: $EXPECTED_FW (will verify reboots over /status.json)"
fi

# Returns 0 if the board at $1 came back on EXPECTED_FW with a fresh uptime,
# i.e. this OTA actually applied and rebooted. Polls for up to VERIFY_TIMEOUT.
verify_applied() {
  local ip="$1"
  [[ -n "$EXPECTED_FW" ]] || return 1
  local deadline=$(( SECONDS + VERIFY_TIMEOUT ))
  local json fw uptime
  while (( SECONDS < deadline )); do
    json=$(curl -s --max-time 5 "http://$ip/status.json" 2>/dev/null) || { sleep 2; continue; }
    fw=$(printf '%s' "$json" | grep -oE '"fw":"[^"]*"' | sed -E 's/.*:"(.*)"/\1/')
    uptime=$(printf '%s' "$json" | grep -oE '"uptimeMs":[0-9]+' | grep -oE '[0-9]+')
    if [[ "$fw" == "$EXPECTED_FW" && -n "$uptime" && "$uptime" -lt "$VERIFY_MAX_UPTIME_MS" ]]; then
      return 0
    fi
    sleep 2
  done
  return 1
}

# Number of attempts per board. The firmware fail-fast fix (servo-6lc) means a
# failed/aborted upload no longer wedges the board — it returns to serving
# within seconds — so a retry is safe and effective: a healthy transfer runs
# ~15s at ~8 KB/s, while a board in a transient degraded-throughput state often
# succeeds on the next attempt (this is the original "second run succeeded"
# behavior, now automated).
RETRIES="${RETRIES:-2}"
RETRY_DELAY="${RETRY_DELAY:-5}"

upload_one() {
  local ip="$1"
  local attempt
  for (( attempt=1; attempt<=RETRIES; attempt++ )); do
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
      if verify_applied "$ip"; then
        echo "  OK   $ip (attempt $attempt/$RETRIES; reboot verified on $EXPECTED_FW)"
        return 0
      fi
      echo "  WARN $ip upload returned HTTP 200, but reboot on $EXPECTED_FW was not verified"
      echo "       see /tmp/ota-$ip.log; board may be wedged post-apply"
      return 1
    fi
    # curl said it failed — but a fast successful OTA resets the connection
    # before the 200 OK lands here. Check whether the board actually rebooted
    # onto this build before calling it a failure (servo-6lc).
    if verify_applied "$ip"; then
      echo "  OK   $ip (curl lost the 200, but board rebooted on $EXPECTED_FW — verified)"
      return 0
    fi
    if (( attempt < RETRIES )); then
      echo "  retry $ip (attempt $attempt/$RETRIES failed; waiting ${RETRY_DELAY}s for board to recover)"
      sleep "$RETRY_DELAY"
    fi
  done
  if curl -s --max-time 5 "http://$ip/status.json" >/tmp/ota-status-"$ip".json 2>/dev/null; then
    echo "  FAIL $ip after $RETRIES attempts (status reachable but not fresh $EXPECTED_FW; see /tmp/ota-status-$ip.json and /tmp/ota-$ip.log)"
  else
    echo "  WEDGE $ip after $RETRIES attempts (no /status.json; likely post-OTA WiFi/server wedge; see /tmp/ota-$ip.log)"
  fi
  return 1
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
