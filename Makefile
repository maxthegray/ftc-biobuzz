# Convenience targets for the day-to-day Control Hub workflow.
# Run `make` (or `make help`) to list everything.

HUB_IP   ?= 192.168.43.1
HUB_PORT ?= 5555
GRADLE   ?= ./gradlew
HUB      := tools/hub-connect.sh $(HUB_IP) $(HUB_PORT)

# `make pull-logs 5` passes 5 as the count; `make pull-logs N=5` works too.
ifeq (pull-logs,$(firstword $(MAKECMDGOALS)))
  N ?= $(word 2,$(MAKECMDGOALS))
  $(eval $(word 2,$(MAKECMDGOALS)):;@:)
endif

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

## --- Deploy --------------------------------------------------------------

deploy: ## Deploy, picking full install or hot reload from what changed
	@tools/deploy/deploy.sh $(GRADLE) $(HUB_IP) $(HUB_PORT)

install: ## Force a full APK install
	@$(HUB)
	$(GRADLE) :TeamCode:installDebug
	@git rev-parse HEAD > .last-deploy-sha

hot: ## Force a Sloth hot reload (~1s, teamcode only)
	@$(HUB)
	$(GRADLE) deploySloth

## --- Robot ---------------------------------------------------------------

connect: ## Wait until a Control Hub is connected (USB or Wi-Fi)
	@$(HUB) && echo "Control Hub connected"

scrcpy: ## Mirror the Control Hub's screen
	@$(HUB)
	scrcpy

## --- Flight logs ---------------------------------------------------------

pull-logs: ## Pull the newest N logs into ./robot-logs (`make pull-logs 5`; default all 30)
	@tools/logs/pull-logs.sh "$(N)" $(HUB_IP) $(HUB_PORT)

viewer: ## Open MaxScope at http://127.0.0.1:8008 to browse and summarize logs
	python3 tools/maxscope/log_viewer.py

.PHONY: help build test deploy install hot connect scrcpy pull-logs viewer
