#!/usr/bin/env bash
# Deploys to the Control Hub, choosing a full install or a Sloth hot reload
# from classify-deploy.sh, and records the deployed commit on success.
#
# Usage: tools/deploy/deploy.sh [GRADLE]

set -euo pipefail
cd "$(git rev-parse --show-toplevel)"

GRADLE="${1:-./gradlew}"

report="$(bash tools/deploy/classify-deploy.sh)"
echo "$report"
echo

if grep -q '^REASON: no changes detected' <<<"$report"; then
  echo "Nothing to deploy."
  exit 0
fi

if ! adb devices | awk 'NR>1 && $2=="device" {found=1} END {exit found?0:1}'; then
  echo "error: no Control Hub connected — plug in USB or run 'make connect'" >&2
  exit 1
fi

if grep -q '^RECOMMENDATION: FULL' <<<"$report"; then
  "$GRADLE" :TeamCode:installDebug
else
  "$GRADLE" deploySloth
fi

git rev-parse HEAD > .last-deploy-sha
