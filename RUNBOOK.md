# Competition Runbook

Symptom → where to look → likely cause. Ten minutes between matches; start
here, not in the code. `make analyze` prints the one-pager; AdvantageScope
(`make pull-logs`, open the newest `.wpilog`) has the full channels.

## Robot dead / op-mode stopped mid-match

- **Driver Station shows an exception** → next init shows `previous run
  crashed: …`; full context (running commands, recent events, stack trace)
  is in `/sdcard/FIRST/logs/lastcrash.txt`. In teleop this should be nearly
  impossible — command faults are contained — so suspect `periodic()` /
  `writeHardware()` / `onLoop()` code, which stays fail-fast by design.
- **No exception, robot just stopped responding** → `make analyze`: did the
  loop rate collapse (phase maxima show which phase), or did battery sag
  below ~9.5 V (brownout)?

## One mechanism stopped but the robot kept driving

That is fault containment working. Health telemetry shows
`command faults: N (last: …)`; the log's event timeline has the
`COMMAND FAULT` entry with the exception. Fix the command; the count tells
you whether it faulted once or every press.

## Auton drove to the wrong place

- **Wrong from the first path** → starting pose. Was the robot placed where
  `setStartingPose` claims? Was the right alliance locked on the selector
  (the `Auton` telemetry section shows LOCKED + alliance)?
- **Mirrored side only** → heading mirroring. Check every bare heading went
  through `Alliance.mirror(heading)` — run the routine through a
  `SimAutonRoutineTest`-style mirror test; it catches exactly this.
- **Drifted over the run** → `follow/translationalErrorIn` in the log. Error
  small but endpoint wrong = localization drift (check Pinpoint pods /
  `Localization Test` lap). Error large = following problem (battery sag?
  re-run Pedro tuners — note `useVoltageCompensation` shifts the calibration).
- **Jumped suddenly** → event timeline `pose correction applied: …`. A bad
  vision measurement got through the gates — tighten
  `LocalizerConfig.maxCorrection*` or lower `correctionBlend`.

## Teleop field-centric is the wrong way around

Heading reference is stale. Back+Y re-zeros heading (square the robot
first). If it happens after auton→teleop: check `restorePersistedPose`
returned true — the event timeline and `Health` section show it; a Robot
Controller restart older than 2 min ages the persisted pose out.

## Loop rate dropped

`make analyze` phase maxima: `writeHardware` spikes = Pedro/Pinpoint I²C
(check wiring, Pinpoint status in init Health); `telemetry` spikes = Panels
websocket (dashboard connected over a bad link?); `periodic` spikes = a
season subsystem doing I/O outside the bulk read. `overhead` consistently
high = something outside the loop (GC from per-tick allocation — check
recently added code).

## Pinpoint unhealthy

Init Health shows `Pinpoint status` every init tick. Anything but READY:
power-cycle the hub, check the I²C cable, re-run `PinpointDirect.recalibrate`
with the robot still. FAULT_* statuses are pod/IMU hardware faults.

## Battery discipline

Init Health warns `battery WARNING: LOW` below 12.0 V resting — swap before
the match, not after the auton sags. `make analyze` prints start/min/end of
every run; a healthy pack should not dip under ~10 V during normal play.
