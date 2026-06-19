# Sensors & I²C wiring

How to add I²C sensors (color, distance, ToF, the Pinpoint, extra encoders)
without re-introducing the battery-sag latency failure mode. This is a
**decision record + recipe**, written before any sensor exists on the robot.
Read it before you wire a sensor or touch `I2CBusThread` / `SRSHubSubsystem`.

Status as of writing: **zero sensors wired.** `SRSHubSubsystem`,
`SRSHubPinpointLocalizer`, and `I2CBusThread` exist but are referenced by no
op-mode. The drive uses the direct-Pinpoint path
(`Constants.createFollower(hardwareMap)`). Nothing here has run on hardware.

## The decision (do this unless measurement says otherwise)

1. **Pinpoint stays direct.** Its own Control-Hub I²C port, read inline inside
   `follower.update()` — the existing default. Do **not** move it behind the
   SRSHub and do **not** background it.
2. **All other I²C goes on one SRSHub**, read **inline** in
   `SRSHubSubsystem.periodic()`.
3. **Do not background the SRSHub read by default.** Inline is the default;
   `I2CBusThread` is a measured-need escape hatch, not the plan.

That's it. The rest of this doc is why, the hazards if you deviate, and the
measurement that's allowed to overturn any of the above.

## Why — the one fact that drives everything

The original latency bug (on a *prior* repo, not this one) was a flaky
per-sensor I²C read **retry-storming inside the loop** as the battery sagged.
There are two separable fixes, and only one of them is the actual cure:

- **Hardware concentration (the SRSHub) is the cure.** The SRSHub owns its
  three downstream I²C buses; the slow/flaky per-sensor traffic and any retries
  happen *there*, off the Control Hub. The Control Hub then does **one bounded
  register read** of already-decoded values (`SRSHub.update()` →
  `deviceClient.read(READ, updateLength)`). A flaky color sensor can no longer
  stretch the loop, because its retries never touch the CH link.
- **Backgrounding the read (a thread) is not a cure, and on this hardware it
  backfires.** The SRSHub plugs into a **Control Hub I²C port**, so its read
  travels the **same CH↔SBC Lynx serial link** as your motor writes and bulk
  encoder reads. A background poll doesn't remove that read's cost from the
  link — it just stops the main loop from *blocking* on it, at the price of
  (a) the poll contending with motor writes in the serial queue and (b) the
  follower running on a stale pose.

This exact experiment was tried and reverted on this codebase. The postmortem
lives in `pedroPathing/Constants.java` (the doc comment on
`createFollower`): backgrounding the Pinpoint read starved on link contention
(~12–25 ms/read) while the loop spun on stale data. `ARCHITECTURE.md`'s
threading section says the same. **The "sensors on their own I²C path" framing
is misleading**: the sensors are on the *SRSHub's* buses, but the SRSHub→CH
read is not on its own path — it shares the Lynx link.

So: concentrate in hardware (yes), offload in software (only if measured).

## If you ever thread it anyway — the hazards

Don't, until step 4 of the bring-up sequence fires. But if you do:

- **Tear hazard.** `SRSHub.update()` decodes *in place* into the device
  objects (`APDS9151`, `GoBildaPinpoint`, …), and the current handles in
  `SRSHubSubsystem.kt` read those live mutable fields. Reading them from
  another thread mid-decode tears. The fix is mandatory, not optional: the
  poll block must `update()` then **copy out into a fresh immutable snapshot
  data class**, publish that via `I2CBusThread.Ref`, and rewrite the handles
  to read `ref.get()`. Never republish a mutated shared object — that's the
  `I2CBusThread` contract (see its KDoc).
- **One volatile snapshot is sufficient.** Publishing one immutable object via
  one volatile write gives readers all-old-or-all-new — no torn multi-field
  read of pose x/y/heading. **No seqlock or double-buffer is warranted.**
- **Single bus owner for writes.** Write-side ops (Pinpoint `resetImu` /
  `setPose`) go through `hub.runCommand()`, which writes the *same*
  `deviceClient` as the read. If a thread owns the bus, route writes through a
  queue the thread drains **at the top of each poll cycle, before
  `update()`**, and coalesce duplicates (keep last). Don't expect to read back
  a queued `setPose` in the same tick.
- **Poll rate ≈ loop rate, maybe 1.5×. Not 100–150 Hz.** The combined
  transaction is several ms of bus time at `FAST_400K`; polling far above the
  loop rate just saturates the thread and maximally contends the Lynx link —
  re-creating the starvation above. Fresh-enough for the follower beats fast.
- **Staleness must trip the watchdog.** Feed `I2CBusThread.lastSuccessfulPollNs`
  age into the `LocalizerSubsystem` watchdog so "thread alive, data old" trips
  the same fault policy as a frozen pose.

## Integration gotchas (apply even with the inline default)

These bite whenever a custom localizer or the SRSHub is wired, threaded or not:

- **Registration order.** Drive subsystem **before** localizer subsystem
  (enforced by `registerAfter`). If the SRSHub feeds the localizer, the SRSHub
  must `init()` before the follower is built.
- **`FollowerBuilder` calls `localizer.resetIMU()` at construction time.**
  `SRSHubPinpointLocalizer.resetImu()` enqueues a hub command — so the SRSHub
  must already be initialised when you call
  `createFollower(hardwareMap, localizer)`, or `resetImu` must no-op until
  ready. (Flagged in `Constants.createFollower(hardwareMap, localizer)`.)
- **Pinpoint-on-SRSHub loses the device-status watchdog.**
  `LocalizerSubsystem.init()` reads the raw `GoBildaPinpointDriver.deviceStatus`
  straight from the hardware map for its ~1 Hz health check. Behind the SRSHub
  that read returns null; you'd have to route `PinpointHandle.deviceStatus`
  into the watchdog to get it back. Another reason the Pinpoint stays direct.
- **Failure domains.** Direct Pinpoint = its own port, its own failure domain.
  Everything-behind-one-SRSHub means one bad connector kills odometry *and*
  every aux sensor at once. Keep the control-critical sensor separate.

## Bring-up — the measurement that's allowed to change the decision

Do these in order, on hardware, each gated on the previous. The deciding
metric throughout is **loop time vs. battery voltage** (you already log both;
`make analyze` gives loop percentiles, and battery is a channel).

1. **Baseline the fear.** Run the current direct-Pinpoint path from a full pack
   down to ~11 V. If loop time stays flat as voltage sags, the failure mode
   you're architecting against **doesn't exist on this robot** — stop here and
   wire sensors the simple way.
2. **Add the SRSHub with aux sensors only**, Pinpoint still direct, SRSHub read
   inline. Instrument `hub.update()` duration and watch `crcMismatchCount()`
   under sag. This is the real win and the lowest-risk step.
3. **Only if step 1 showed sag-correlated latency:** measure whether moving the
   *Pinpoint* onto the SRSHub (still inline) flattens it — i.e. is the bounded
   SRSHub read more sag-stable than the direct Pinpoint? This is the only
   question that justifies Pinpoint-on-SRSHub.
4. **Only if the inline SRSHub read itself stretches the loop:** then evaluate
   `I2CBusThread`, and validate it specifically by confirming it does **not
   delay motor writes** (the documented failure mode) — not merely that
   loop-time-excluding-the-read dropped.

Snapshot immutability, the command queue, and poll rate only matter if step 4
fires. Don't build them on faith.

## Recipe — wiring an aux sensor when the season starts

Inline default, no threading. In the op-mode's `configure()`:

```kotlin
val srs = register(SRSHubSubsystem())          // reads inline in periodic()
val intakeColor = srs.color(bus = 1)           // register devices BEFORE init()
val frontDist   = srs.distance(bus = 2)
// ... then read from a subsystem's periodic():
//   val r = intakeColor.red; val mm = frontDist.distanceMm
```

A non-Pinpoint I²C device that genuinely must not block the loop *and* is on a
path independent of the CH Lynx link (rare — most aren't) can use
`I2CBusThread` directly; publish an immutable value, read `ref.get()` in
`periodic()`, `start()` in `init()`, `stop()` in `stop()`.

The Pinpoint needs no wiring — it's already the drive default. Only switch to
`createFollower(hardwareMap, SRSHubPinpointLocalizer(srs.pinpoint(...)))` if
step 3 above proved it's worth it.
