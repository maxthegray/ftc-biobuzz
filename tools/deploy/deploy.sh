#!/usr/bin/env bash
# Deploys to the Control Hub, choosing a full install or a Sloth hot reload
# from classify-deploy.sh, and records the deployed commit on success.
#
# Usage: tools/deploy/deploy.sh [GRADLE] [HUB_IP] [HUB_PORT]

set -euo pipefail
cd "$(git rev-parse --show-toplevel)"

GRADLE="${1:-./gradlew}"
HUB_IP="${2:-192.168.43.1}"
HUB_PORT="${3:-5555}"

report="$(bash tools/deploy/classify-deploy.sh)"
echo "$report"
echo

if grep -q '^REASON: no changes detected' <<<"$report"; then
  echo "Nothing to deploy."
  exit 0
fi

tools/hub-connect.sh "$HUB_IP" "$HUB_PORT"

if grep -q '^RECOMMENDATION: FULL' <<<"$report"; then
  "$GRADLE" :TeamCode:installDebug
else
  "$GRADLE" deploySloth
fi

git rev-parse HEAD > .last-deploy-sha
