# Convenience targets for the day-to-day Control Hub workflow.
# Run `make` (or `make help`) to list everything.

HUB_IP   ?= 192.168.43.1
HUB_PORT ?= 5555
GRADLE   ?= ./gradlew

.DEFAULT_GOAL := help

help: ## Show this help
	@awk 'BEGIN {FS = ":.*?## "} /^[a-zA-Z_-]+:.*?## / {printf "  \033[36m%-18s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)

## --- Build ---------------------------------------------------------------

build: ## Compile + type-check, no install
	$(GRADLE) :TeamCode:assembleDebug

test: ## Run all host tests (TeamCode, log tools, MaxScope)
	$(GRADLE) :TeamCode:testDebugUnitTest
	python3 -m unittest tools/logs/test_analyze_wpilog.py tools/maxscope/test_log_viewer.py
	@if command -v node >/dev/null 2>&1; then node --test tools/maxscope/viewer/core.test.mjs; \
	else echo "node not found; skipping MaxScope JS tests"; fi

clean: ## gradle clean
	$(GRADLE) clean

## --- Deploy --------------------------------------------------------------

deploy: ## Deploy, picking full install or hot reload from what changed
	@tools/deploy/deploy.sh $(GRADLE)

install: ## Force a full APK install
	$(GRADLE) :TeamCode:installDebug
	@git rev-parse HEAD > .last-deploy-sha

hot: ## Force a Sloth hot reload (~1s, teamcode only)
	$(GRADLE) deploySloth

## --- ADB ----------------------------------------------------------------

connect: ## adb connect to Control Hub over WiFi ($(HUB_IP):$(HUB_PORT))
	adb connect $(HUB_IP):$(HUB_PORT)

reset-adb: ## Kill the adb server (use when it gets wedged)
	adb kill-server

logs: ## Stream robot logs (RobotCore / OpMode / System.err)
	adb logcat -s RobotCore:* OpMode:* System.err:*

## --- Flight logs ---------------------------------------------------------

pull-logs: ## Pull every log the hub keeps (newest 30) into ./robot-logs
	mkdir -p robot-logs
	adb pull /sdcard/FIRST/logs/. robot-logs/

pull-lab-records: ## Pull vision lab records into ./lab-records for review
	mkdir -p lab-records
	adb pull /sdcard/FIRST/lab-records/. lab-records/

analyze: ## Pull the newest match logs (if a hub is connected) and summarize
	@tools/logs/pull-latest-logs.sh $(HUB_IP) $(HUB_PORT) || echo "using logs already in robot-logs/" >&2
	@python3 tools/logs/analyze_wpilog.py

debug: ## Same as analyze, but emit a JSON diagnostic bundle
	@tools/logs/pull-latest-logs.sh $(HUB_IP) $(HUB_PORT) >&2 || echo "using logs already in robot-logs/" >&2
	@python3 tools/logs/analyze_wpilog.py --json

viewer: ## Open the offline MaxScope server at http://127.0.0.1:8008
	python3 tools/maxscope/log_viewer.py

.PHONY: help build test clean deploy install hot connect reset-adb logs pull-logs pull-lab-records analyze debug viewer
