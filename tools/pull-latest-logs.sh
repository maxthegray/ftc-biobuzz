#!/bin/sh
# Pull only the newest match's flight-recorder log(s) off the Control Hub.
#
# A match is two op-mode runs (Auto + TeleOp), so prefer the newest log whose
# op-mode name contains each prefix. If one side is missing, fill from the
# next-newest distinct log instead of dragging all 30 over the link.
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

# Names contain no spaces. Keep the remote modification-time order, select
# newest TeleOp + Auto when available, then fill any missing slot.
ALL_LOGS=$(adb shell "ls -t $LOG_DIR/*.wpilog 2>/dev/null" | tr -d '\r')
if [ -z "$ALL_LOGS" ]; then
    echo "error: no .wpilog files in $LOG_DIR on the hub" >&2
    exit 1
fi
LOGS=$(printf '%s\n' "$ALL_LOGS" | awk '
    {
        logs[count++] = $0
        lower = tolower($0)
        if (teleop == "" && lower ~ /teleop-/) teleop = $0
        if (auto == "" && lower ~ /auto-/) auto = $0
    }
    END {
        selected = 0
        if (teleop != "") {
            print teleop
            chosen[teleop] = 1
            selected++
        }
        if (auto != "" && !chosen[auto]) {
            print auto
            chosen[auto] = 1
            selected++
        }
        for (i = 0; i < count && selected < 2; i++) {
            if (!chosen[logs[i]]) {
                print logs[i]
                chosen[logs[i]] = 1
                selected++
            }
        }
    }
')

mkdir -p "$DEST"
for f in $LOGS; do
    adb pull "$f" "$DEST" >/dev/null
    filename=$(basename "$f")
    opmode=$(printf '%s\n' "$filename" | sed -E 's/-[0-9]{8}-[0-9]{6}.*[.]wpilog$//')
    echo "pulled $DEST/$filename (op-mode: $opmode)"
done
