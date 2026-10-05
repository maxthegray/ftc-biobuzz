#!/bin/sh
# Makes sure adb can reach a Control Hub: USB first, then the hub's Wi-Fi
# address. If adb is stuck, restarts it and tries once more.
#
# Usage: tools/hub-connect.sh [HUB_IP] [HUB_PORT]
set -eu

HUB_IP="${1:-192.168.43.1}"
HUB_PORT="${2:-5555}"

if ! command -v adb >/dev/null 2>&1; then
    echo "error: adb not on PATH — install Android platform-tools" >&2
    exit 1
fi

reachable() {
    adb devices | awk 'NR>1 && $2=="device" {found=1} END {exit found?0:1}'
}

reachable && exit 0
adb connect "$HUB_IP:$HUB_PORT" >/dev/null 2>&1 || true
reachable && exit 0

adb kill-server >/dev/null 2>&1 || true
adb connect "$HUB_IP:$HUB_PORT" >/dev/null 2>&1 || true
reachable && exit 0

echo "error: no Control Hub over USB or Wi-Fi — check the cable or join the hub's network" >&2
exit 1
