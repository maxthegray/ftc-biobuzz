#!/bin/sh
# Makes sure adb can reach a Control Hub: USB first, then the hub's Wi-Fi
# address. If adb is stuck, restarts it once. Keeps trying for WAIT_SECONDS
# so a hub that is still booting gets picked up when it comes online.
#
# Usage: tools/hub-connect.sh [HUB_IP] [HUB_PORT] [WAIT_SECONDS]
set -eu

HUB_IP="${1:-192.168.43.1}"
HUB_PORT="${2:-5555}"
WAIT_SECONDS="${3:-60}"

if ! command -v adb >/dev/null 2>&1; then
    echo "error: adb not on PATH — install Android platform-tools" >&2
    exit 1
fi

reachable() {
    adb devices 2>/dev/null | awk 'NR>1 && $2=="device" {found=1} END {exit found?0:1}'
}

# adb connect can hang for over a minute when the address doesn't answer,
# so give each attempt 3 seconds.
try() {
    reachable && return 0
    adb connect "$HUB_IP:$HUB_PORT" >/dev/null 2>&1 &
    pid=$!
    ( sleep 3; kill "$pid" 2>/dev/null ) &
    wait "$pid" 2>/dev/null || true
    reachable
}

try && exit 0
adb kill-server >/dev/null 2>&1 || true
try && exit 0

echo "waiting for a Control Hub over USB or Wi-Fi (Ctrl-C to stop)..." >&2
deadline=$(($(date +%s) + WAIT_SECONDS))
while [ "$(date +%s)" -lt "$deadline" ]; do
    sleep 2
    if try; then
        echo "connected" >&2
        exit 0
    fi
done

echo "error: no Control Hub after ${WAIT_SECONDS}s — check the cable or join the hub's network" >&2
exit 1
