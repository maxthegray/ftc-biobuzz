#!/usr/bin/env bash
# Decides whether the pending robot deploy needs a full APK install or a
# Sloth hot reload, based on which files changed since the last deploy.
#
# Sloth hot-reloads ONLY classes under org.firstinspires.ftc.teamcode, and
# re-runs their static initialisers. A full install is required when a change
# can't be hot-reloaded:
#   - a @Pinned class (its code is loaded once, never re-initialised)
#   - any gradle / dependency / manifest change
#   - any source outside the teamcode package (e.g. the FtcRobotController
#     module) — Sloth won't touch it
#
# Output (stdout), consumed by SKILL.md:
#   RECOMMENDATION: HOT | FULL
#   REASON: <one line>
#   FILES:
#   <changed file>  -> <hot|FULL: why>
#
# This script DECIDES ONLY. It never runs gradle and never writes the marker;
# the skill procedure does that after a successful deploy.
# Portable to bash 3.2 (macOS default) — no mapfile/readarray.

set -euo pipefail
cd "$(git rev-parse --show-toplevel)"

MARKER=".claude/.last-deploy-sha"
TEAMCODE_PREFIX="TeamCode/src/main/kotlin/org/firstinspires/ftc/teamcode/"
TEAMCODE_PREFIX_JAVA="TeamCode/src/main/java/org/firstinspires/ftc/teamcode/"

# --- collect the candidate changed-file set ------------------------------
# tracked working-tree changes + untracked files + anything committed since
# the recorded marker. Deduped.
collect() {
  git diff --name-only HEAD
  git ls-files --others --exclude-standard
  if [[ -f "$MARKER" ]]; then
    local sha
    sha="$(cat "$MARKER")"
    if git cat-file -e "${sha}^{commit}" 2>/dev/null; then
      git diff --name-only "$sha" HEAD
    fi
  fi
}

files=()
while IFS= read -r line; do
  [[ -n "$line" ]] && files+=("$line")
done < <(collect | sort -u)

# No marker yet -> first deploy against this checkout. CLAUDE.md: the first
# deploy of a session is a full install. Honour that regardless of diff.
if [[ ! -f "$MARKER" ]]; then
  echo "RECOMMENDATION: FULL"
  echo "REASON: no deploy marker yet (treated as first deploy of the session) — full install."
  echo "FILES:"
  if [[ ${#files[@]} -eq 0 ]]; then echo "  (working tree clean)"; else printf '  %s\n' "${files[@]}"; fi
  exit 0
fi

if [[ ${#files[@]} -eq 0 ]]; then
  echo "RECOMMENDATION: HOT"
  echo "REASON: no changes detected since last deploy — nothing to push (hot reload is a safe no-op)."
  echo "FILES:"
  echo "  (none)"
  exit 0
fi

# --- classify ------------------------------------------------------------
full=0
details=()
reasons=()

for f in "${files[@]}"; do
  verdict="hot"
  case "$f" in
    *.gradle|*gradle.properties|settings.gradle|gradle/wrapper/*|*.dependencies.gradle)
      verdict="FULL: build/dependency change"; full=1; reasons+=("dependency/build change") ;;
    *AndroidManifest.xml)
      verdict="FULL: manifest change"; full=1; reasons+=("manifest change") ;;
    "$TEAMCODE_PREFIX"*|"$TEAMCODE_PREFIX_JAVA"*)
      # teamcode source — hot-reloadable UNLESS it's a @Pinned class.
      if [[ -f "$f" ]] && grep -q '@Pinned\|sinister\.loading\.Pinned' "$f"; then
        verdict="FULL: @Pinned class (code changes need full install)"
        full=1; reasons+=("@Pinned class changed")
      fi ;;
    *.kt|*.java)
      # source file outside the teamcode package — Sloth won't reload it.
      verdict="FULL: source outside teamcode package"; full=1; reasons+=("non-teamcode source change") ;;
    TeamCode/src/main/res/*)
      verdict="FULL: resource change"; full=1; reasons+=("resource change") ;;
    *)
      verdict="hot: not a compiled/hot-reload-relevant change" ;;
  esac
  details+=("$f  -> $verdict")
done

if [[ $full -eq 1 ]]; then
  uniq_reasons="$(printf '%s\n' "${reasons[@]}" | sort -u | paste -sd', ' -)"
  echo "RECOMMENDATION: FULL"
  echo "REASON: full install required — $uniq_reasons."
else
  echo "RECOMMENDATION: HOT"
  echo "REASON: only hot-reloadable teamcode changes — Sloth hot reload (~1s)."
fi
echo "FILES:"
printf '  %s\n' "${details[@]}"
