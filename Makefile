# Convenience targets for the day-to-day Control Hub workflow.
# Run `make` (or `make help`) to list everything.

HUB_IP   ?= 192.168.43.1
HUB_PORT ?= 5555
GRADLE   ?= ./gradlew

.DEFAULT_GOAL := help

help: ## Show this help
	@awk 'BEGIN {FS = ":.*?## "} /^[a-zA-Z_-]+:.*?## / {printf "  \033[36m%-12s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)

## --- Build ---------------------------------------------------------------

build: ## Compile + type-check, no install
	$(GRADLE) :TeamCode:assembleDebug

clean: ## gradle clean
	$(GRADLE) clean

## --- Deploy --------------------------------------------------------------

install: ## Full APK install (use after @Pinned/dep/manifest changes)
	$(GRADLE) :TeamCode:installDebug

hot: ## Sloth hot reload (~1s, teamcode only)
	$(GRADLE) deploySloth

## --- ADB ----------------------------------------------------------------

connect: ## adb connect to Control Hub over WiFi ($(HUB_IP):$(HUB_PORT))
	adb connect $(HUB_IP):$(HUB_PORT)

disconnect: ## adb disconnect all wireless devices
	adb disconnect

devices: ## List connected adb devices
	adb devices

reset-adb: ## Kill the adb server (use when it gets wedged)
	adb kill-server

logs: ## Stream robot logs (RobotCore / OpMode / System.err)
	adb logcat -s RobotCore:* OpMode:* System.err:*

logs-all: ## Stream full unfiltered logcat
	adb logcat

.PHONY: help build clean install hot connect disconnect devices reset-adb logs logs-all
