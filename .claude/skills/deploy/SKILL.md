---
name: deploy
description: Deploy this FTC robot code to the Control Hub, automatically choosing a full APK install vs a Sloth hot reload based on what changed since the last deploy. Use when the user wants to deploy, install, push code to the robot, or hot reload. Avoids the silent footgun of hot-reloading a @Pinned/dependency/manifest change (which doesn't take effect).
---

# Deploy to the robot

Pick the right deploy path and run it. The two paths must not be confused:

- **Full install** — `./gradlew :TeamCode:installDebug`. Full APK build +
  install. Required after changing anything Sloth can't hot-reload: a
  `@Pinned` class (there are none today; config objects are not pinned; a
  reload resets Panels edits to the code's values), any dependency/gradle change, the
  manifest, resources, or any source outside the
  `org.firstinspires.ftc.teamcode` package. Also the right call for the
  first deploy of a session.
- **Hot reload** — `./gradlew deploySloth`. Pushes only teamcode classes,
  ~1s. For ordinary iteration on subsystems, op-modes, and command logic.

The footgun this skill exists to prevent: hot-reloading a `@Pinned`,
dependency, or manifest change **silently does nothing** — the robot keeps
running old code. Always classify before deploying.

## Procedure

1. **Honour an explicit override.** If the user said "full install" / "force
   full" / "hot reload only", run `make install` or `make hot` and say you're
   overriding the automatic choice. Otherwise run:

   ```
   make deploy
   ```

   It runs `tools/deploy/classify-deploy.sh`, prints its
   `RECOMMENDATION: HOT|FULL`, `REASON` and deciding files, refuses to run
   without a connected device, deploys, and on success writes
   `.last-deploy-sha` so the next run diffs from here. `make install` also
   writes the marker; `make hot` does not, so a forced hot reload never hides
   a pending full install.

2. **If it reports no device,** tell the user to plug in USB or run
   `make connect` (adb connect to `192.168.43.1:5555`, the Control Hub
   default), then stop.

3. **Report** concisely: which path ran and why (quote the `REASON`; name the
   files if FULL was chosen because of pinned/dep/manifest changes), and
   success or failure. "Nothing to deploy" means no changes since the last
   deploy; say so.

The Load plugin auto-wires `removeSlothRemote` into `installDebug`, so a full
install correctly clears any staged hot-reload jar; the two paths won't fight.

## Notes

- Run everything from the repo root.
- Both agents use the same `.last-deploy-sha` marker. It is local, gitignored
  state. Older agent-specific markers are ignored; the first deploy after
  this migration therefore requires a full install.
- This skill does not tune or run op-modes — it only deploys. Selecting which
  op-mode to run happens on the Driver Station.
