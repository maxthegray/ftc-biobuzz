#!/bin/sh
# Pull only the newest match's flight-recorder log(s) off the Control Hub.
#
# A match is two op-mode runs (Auto + TeleOp), so we grab the newest two
# .wpilog files instead of dragging all 30 over the link. USB-first: if the
# hub is already on adb (USB) we use it; otherwise we fall back to wifi.
#
# Usage: tools/pull-latest-logs.sh [HUB_IP] [HUB_PORT]
set -eu

HUB_IP="${1:-192.168.43.1}"
HUB_PORT="${2:-5555}"
LOG_DIR="/sdcard/FIRST/logs"
DEST="robot-logs"

if ! command -v adb >/dev/null 2>&1; then
    echo "error: adb not on PATH — install Android platform-tools" >&2
    exit 1
fi

reachable() {
    # A hub is reachable if any device is in the 'device' state.
    adb devices | awk 'NR>1 && $2=="device" {found=1} END {exit found?0:1}'
}

# USB-first: if nothing is already attached, try the wifi address.
if ! reachable; then
    adb connect "$HUB_IP:$HUB_PORT" >/dev/null 2>&1 || true
    if ! reachable; then
        echo "error: no Control Hub over USB or wifi — check the cable or join the hub's network" >&2
        exit 1
    fi
fi

# Newest two logs, by modification time, names only (no spaces in our filenames).
LOGS=$(adb shell "ls -t $LOG_DIR/*.wpilog 2>/dev/null" | tr -d '\r' | head -2)
if [ -z "$LOGS" ]; then
    echo "error: no .wpilog files in $LOG_DIR on the hub" >&2
    exit 1
fi

mkdir -p "$DEST"
for f in $LOGS; do
    adb pull "$f" "$DEST" >/dev/null
    echo "pulled $DEST/$(basename "$f")"
done
