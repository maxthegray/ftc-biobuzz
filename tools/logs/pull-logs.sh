#!/bin/sh
# Pull the newest N flight-recorder logs off the Control Hub into robot-logs/.
# The hub keeps at most 30, so the default pulls everything.
#
# Usage: tools/logs/pull-logs.sh [N] [HUB_IP] [HUB_PORT]
set -eu

N="${1:-30}"
HUB_IP="${2:-192.168.43.1}"
HUB_PORT="${3:-5555}"
LOG_DIR="/sdcard/FIRST/logs"
DEST="robot-logs"

case "$N" in
    ''|*[!0-9]*) echo "error: N must be a number, got '$N'" >&2; exit 1 ;;
esac

"$(dirname "$0")/../hub-connect.sh" "$HUB_IP" "$HUB_PORT"

# Names contain no spaces; ls -t lists newest first.
LOGS=$(adb shell "ls -t $LOG_DIR/*.wpilog 2>/dev/null" | tr -d '\r' | head -n "$N")
if [ -z "$LOGS" ]; then
    echo "error: no .wpilog files in $LOG_DIR on the hub" >&2
    exit 1
fi

mkdir -p "$DEST"
for f in $LOGS; do
    adb pull "$f" "$DEST" >/dev/null
    echo "pulled $DEST/$(basename "$f")"
done
