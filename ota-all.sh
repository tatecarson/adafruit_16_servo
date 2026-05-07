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
if [[ -z "${OTA_PASSWORD:-}" && -f Secrets.h ]]; then
  OTA_PASSWORD=$(awk -F'"' '/^[[:space:]]*#define[[:space:]]+OTA_PASSWORD[[:space:]]/ {print $2; exit}' Secrets.h)
fi
: "${OTA_PASSWORD:?OTA_PASSWORD not set. Export it, or define it in Secrets.h.}"

USER_PASS="arduino:${OTA_PASSWORD}"

echo "Compiling..."
arduino-cli compile --fqbn "$FQBN" --build-path "$BUILD_DIR" .

if [[ ! -f "$BIN" ]]; then
  echo "Build artifact not found at $BIN" >&2
  exit 1
fi

echo "Uploading to ${#BOARDS[@]} boards in parallel..."
pids=()
for ip in "${BOARDS[@]}"; do
  (
    if curl -sS --fail \
        --connect-timeout 3 --max-time 30 \
        --user "$USER_PASS" \
        --data-binary @"$BIN" \
        "http://$ip:65280/sketch" > "/tmp/ota-$ip.log" 2>&1; then
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
