# MaxScope

An offline WPILOG viewer for this robot. Python 3.10+ and a modern browser
are the only runtime requirements. There are no packages to install or
internet services to connect to.

From the repository root:

```sh
make viewer
```

Open <http://127.0.0.1:8008>. The newest run opens on its own. Choose another
from `robot-logs/`, including its subdirectories, or use **Open WPILOG** for
another file. Imported files are temporary copies removed when the server
stops; original logs are never modified. The viewer does not connect to the
robot: run `make pull-logs` as usual and new runs appear within a few seconds,
marked NEW. If you were looking at the newest run, the newly pulled one opens.

The run list is grouped by day and shows each run's time from op-mode start
to its last record. Runs that never left init, or stopped within 5 s of start,
are hidden behind **Show** (a run you open stays listed). Lengths come from a
header-only scan of every log in the background when the server starts, about
10 s for 300 MB.

Alternate directory or port:

```sh
python3 tools/maxscope/log_viewer.py --logs /path/to/logs --port 8010
```

## Coordinates

The field uses `pose = [x inches, y inches, heading radians]` directly:

- Origin at the audience-left corner.
- +X right along the audience wall; +Y away from the audience.
- Heading zero along +X; positive headings counterclockwise.
- Field size comes from this checkout's `RobotConfig.Field.LENGTH_INCHES`.

`Field/Robot`, WPILib structs, the metres conversion, centre-origin shift,
and `FIELD_VIEW_QUARTER_TURNS` are not used. Rendering only maps Cartesian
coordinates to canvas pixels. Existing AdvantageScope output remains
compatible. Field size is not historical metadata in today's logs, so older
logs with different field dimensions need a future metadata addition.

When the robot leaves the field, even partly (for example an op-mode that
never set a starting pose), the view grows by whole tiles so the whole robot
marker is always visible at every pose, rotated corners included, at most one
field length past each wall. Overhanging by under half an inch doesn't add a
tile. The view stays square. The field keeps its outline;
the area outside it is hatched. A pose past that limit is shown as an arrow on
the edge of the view. The view is fixed for the run, so it doesn't move while
scrubbing.

The **BIOBUZZ** checkbox on the field (on by default, saved) draws this
season's elements: red and blue alliance walls, LOADING ZONES (A5, F2),
GARDENS (A1, F6), the HIVE frame and both HIVES with their CELLS, the four
FLOWERS, and tile names A–F / 1–6. Positions come from the 2026-27 manual
(TU04) Section 9 and the Event Field Setup Guide, with the manual's ±1 in.
tolerance; each number and its figure is listed in `viewer/field-elements.js`.
The raised CELL of each HIVE is drawn solid: from `Hive/<RED|BLUE>/state` when
the log has it, otherwise the match setup (red audience, blue far), dashed.

Without it the grid is season-neutral. The robot is a 16 in marker (chassis outline,
wheel pods, heading line from the centre to the front), not a measured
footprint. The faint trail is the recorded run and
the darker trail is its elapsed portion, not a planned path. Drawing uses a
reduced trail; the coordinate readout always uses the full pose series.

## Controls

- Three tabs: **Field**, **Signals**, and **Events**. Field starts with the
  field on the left, Gamepads on the right and the run summary at the
  bottom; Events contains Commands and the event list.
- **+ View** adds a view to the current tab without removing it from other
  tabs, into the column with fewer views (away from the field on a tie). ×
  in a view heading hides it from that tab. Field and Events lay views out
  in two columns, Signals in one; the columns stack on narrow windows. The
  field's column is the wider one and an empty column gives its width to
  the rest. The field is drawn square to its column's width and its card
  wraps it exactly, so the column is only as tall as the field; it is
  smaller when other views share its column. Drag a view by the ⠿ handle in its
  heading: it moves as you drag, into any column or the free space under
  one, and lands where you see it. With the handle focused, Alt+↑/↓ moves it
  within its column and Alt+←/→ across. The layout is saved per tab. Tab contents and graph
  selections are saved in this browser; all views share playback time.
- Signals starts with one large graph. **+ Graph** adds another (up to eight);
  × beside a graph removes it. With several graphs, **Expand** temporarily
  fills the Signals view with one; **Show all graphs** restores the others.
  Existing saved graphs are retained when upgrading from the earlier viewer.
- **Dark** / **Light** in the header switches the theme. The choice is saved
  in this browser; until you pick one, the viewer follows the system setting.
- Every time shown counts from the op-mode `start` event, like the driver
  station; init time is negative and hovering the clock shows the log time.
  Under the clock, BIOBUZZ's TELEOP / FLOWER OWNERSHIP UNLOCKED / FINAL 20
  SECONDS and time left in the 2:00 driver period (AUTO and 0:30 for op-modes
  named `…Auto`), then time past the end. Start, FLOWER ownership unlocked
  at 1:00 left, the final 20 seconds, and the end of the period are marked
  under the slider and on every graph. Timing comes from `RobotConfig.Match`,
  with BIOBUZZ defaults if its source is unavailable. Match teleop rumbles
  both controllers twice at 1:00 left and three times at 0:20 left;
  autonomous and diagnostics have no match rumbles. A run is opened at its start.
- Scrub the shared time slider, click a graph, event, or command bar to seek.
- Click the field to jump to when the robot was there. If it passed that spot
  more than once, the note says how many times and each click goes to the
  next pass.
- Play at 0.25–4×. Space toggles playback when focus is outside a control.
- Left/right arrow steps 20 ms; Shift+arrow steps one second.
- Each graph has Overview, Drive, Timing, and Vision presets for its own
  signal layers. Applying a preset replaces that graph’s selection. Click a
  graph's **+ Add signal** button to browse expandable folders (for example,
  `Drive → motors → leftFront → power`) or search full channel paths.
  Arrays such as `pose` expand into named components. Escape closes the
  picker, and **Clear graph** empties that graph. Each graph supports up to
  eight overlaid signals with two unit scales, marked L/R in the legend.
  Signals with matching units share a scale. Checkboxes hide individual
  layers; × removes them. Missing channels stay labeled when switching logs.
  `velocity` also has a derived **Speed** component, √(vx² + vy²).
- Click a signal's name in the legend, or double-click its line, to highlight
  it: the other signals dim, and its max and min in view are marked on the
  graph and listed under its name. Click either to jump there. Click the name
  again, double-click away from the lines, or press Escape to clear.
- **+ Discrete** adds state lanes below a graph: strings, booleans, integer
  states, or individual gamepad buttons. Commands appears under the first
  graph by default. Each lane shares the graph's time axis, zoom, and cursor;
  click to seek and hover to inspect a state. Lane selections are saved.
- Field shows active commands above the replay. Its **Commands** checkbox
  toggles this display. Multiple active commands and suspended states are
  shown as recorded; this is only the command coverage available in the log.
- **Ball** (shown when the log has ball-tracking channels) draws where the
  robot thinks the tracked ball is: `BallAssist/tx`/`ty` (older logs), or the USB camera's
  `BallCamera/target/horizontalDeg`/`verticalDeg` (positive below the axis,
  flipped here). When the assist's ty isn't logged (aim only), the Limelight's
  own ty is used if it sees a target whose tx matches. The camera ray is
  projected onto the plane of a pollen's centre (2.8 in diameter) using the
  camera mount: `BallCamera/mount/…` from the log when `measured` is true,
  otherwise the Limelight's measured mount in `core.mjs` (lens 110.75 mm high,
  pitched 10° down, offset unknown). Within 48 in the ray is solid and ends at
  a ball dot, with the distance in the corner readout; accepted detections
  from every camera with a logged mount are outlined in range. Farther, or with no ty or mount, the ray
  is dashed and direction-only: past that range, one degree of ty moves the
  estimate by feet. Angles more than 250 ms old are hidden.
- **Camera** is a view of the camera frame itself: every candidate blob the
  detector published, at its pixel position and radius. The selected blob is
  filled and labelled with its angles, accepted blobs are outlined, and
  rejected ones are faded and labelled with the filter they failed. Add it
  once per camera from **+ View**; each card chooses which camera it shows,
  and the picker is hidden when a log has only one. Cameras are found by
  channel prefix (`<Subsystem>/candidates/…` or `<Subsystem>/target/…`), so a
  second camera subsystem appears without any viewer change. Logs recorded
  before a camera logged its candidates show nothing.
- The Commands view (Events tab) has a bar for each execution and the
  playback cursor; click a bar to seek. Instrumented logs use the recorded
  executions; older logs fall back to spans derived from the sampled active
  set, labelled "sampled", whose edges are only as exact as the sampling.
- Graphs follow playback: every graph shows the same fixed window (10 s by
  default) with the cursor two thirds across, so a short run and a long one
  read the same. **5 s / 10 s / 30 s / All** beside **+ Graph** pick the
  window, and the choice is saved; All is the whole run. Scrolling over a
  graph zooms; while following it changes the window length and keeps
  following. Dragging pans and detaches the graphs where you leave them;
  **Follow playback** (or F) puts them back on the cursor. While detached, a
  soft band under the field path marks the stretch the graphs show. Hovering a
  graph draws a crosshair on all of them and lists that graph's values at the
  pointer. The slider always spans the complete run.
- **Find problems** in the playback bar scans the run for moments worth a
  look. It is off when the viewer opens, so nothing is drawn until you ask.
  The rules use channels already logged: battery under 9.5 V (a fault under
  8 V; readings of 1 V or less are ignored as dead), loops over 50 ms
  (skipping the two startup loops), `<Subsystem>/connected` going false
  after it was true, `Localizer/faulted`, `Drive/odometryFallback`, pose
  jumps over 3 in between samples (not the reset as the op-mode stops),
  fault events, failed commands, lost command records, a cut-off log end,
  and traction (see **Traction**). Spans closer than a second merge
  (battery: five seconds, one per stretch of hard driving); the time listed
  is only the time actually past the threshold, with how many dips merged.
  While on, they
  show as marks under the slider, faint bands on every graph and ✕ marks on
  the field where the robot was, and the Problems view opens in the current
  tab; its chips filter by kind. It stays on as you switch runs. Click the
  button again, or close the Problems view, to turn it off. `[` and `]`
  step to the previous and next problem, turning it on first if needed.
  Rules live in `problems()` in `log_viewer.py`; a log without a channel
  just has no problems of that kind.
- **Overlay runs** in the header lays up to four other runs over the open
  one, lettered B–E (the open run is A). The picker lists runs of the same
  op-mode first; the **+** on a run in the sidebar overlays it directly and
  shows its letter once it is on. On the field each run has its own color:
  its path, the part played so far, and its robot at the cursor tagged with
  its letter. On graphs a line's color stays its signal's and each run draws
  its own dash pattern, shown on its chip; the legend, hover tooltip, pose
  readout and summary tiles give every run's value by letter. The overlay
  bar under the playback controls has a chip per run with show/hide, its own
  shift in seconds and remove, plus the alignment for all of them: op-mode
  `start` event (default), first movement (more than an inch), or log start.
  Each discrete lane gets a thin strip per shown overlay under the open run's
  strip, lettered in the margin: the same value has the same color in every
  strip, and the lane's header and hover list every run's value. Overlays
  stay when you open another run; opening one of them drops it from the
  overlays. Events, commands and gamepads show the open run only.
- Current command logs show instrumented commands only. Missing commands
  may be unlogged; lost-record counts and incomplete log tails are surfaced.
  Discrete command lanes show the recorded active set, including concurrent
  commands; they do not infer activity for unlogged commands. Event messages
  stay in Events rather than being drawn as continuously active states.
- Legacy logs show sampled active commands without lifecycle bars. A command
  finishing does not prove physical arrival. Suspensions are visible in the
  event list and active text; bars span the entire execution, including pauses.
- Selected scalar graphs, field, driver inputs, and events share one clock.
  Events combine recorder and command channels. Fault totals count records,
  not distinct root causes. Large event lists show up to 500 near the cursor.
- There is no camera-video replay, desired-path overlay or live tuning.
  Problems are fixed threshold rules, not a diagnosis of the cause.
- The parser supports the types this recorder emits; it does not decode
  arbitrary WPILib structs. Logs without raw `pose` still open but have no
  field playback.
- File limit: 256 MiB. Parsing happens in the local Python server, which
  keeps up to six decoded runs while their files total 64 MiB (decoded, a
  run takes about 8× its file size), so overlays of long teleop runs may be
  re-parsed when you add signals. Large files can take several seconds and use much
  more RAM than their on-disk size. Playback channels and selected graphs
  are transferred in full; this is not a streaming large-log database.
- Run dates come from the filename's hub clock, which may be wrong.

## Traction (wheels vs Pinpoint)

For logs with drive encoders (`Drive/motors/velocityTps`), the server compares
how fast the wheels say the robot moved with what the Pinpoint measured, in
the robot's frame. `traction.py` does the work; nothing is logged on the robot
for it.

- The wheel scales are fitted from each log: inches per tick forward, inches
  per tick strafing (mecanum rollers lose some), radians per tick turning. Each
  is the median ratio over samples where that axis moves, so hits and slips
  don't skew it. A log without enough strafing uses the forward scale; one
  without turning skips the turn term. The fit is in the run report as
  `traction`.
- Derived channels under `MaxScope/traction/`: `wheels` and `pinpoint`
  (`[forward, left, turn]`), `mismatchInPerS` (the difference, smoothed over
  5 samples; turn counts as the inches a wheel would travel),
  `thresholdInPerS`, and `wheelsMinusPinpointInPerS` (positive: the wheels
  outran the robot).
- **Find problems** flags a stretch where the mismatch stays over
  8 in/s + 25% of wheel speed for at least 0.1 s (stretches 0.5 s apart
  merge). **Wheels slipping**: the wheels went faster than the robot (spinning
  out, or pushing a wall or robot). **Pushed**: the robot moved without its
  wheels (hit or shoved). Wheels that stall against a wall agree with the
  Pinpoint and are not flagged.
- If wheels and Pinpoint disagree by more than 30% in ordinary driving (an
  encoder reversed or unplugged), the check reports **Traction check off**
  with the reason instead of flagging the whole run.
- The thresholds are first guesses, not yet tuned on a run with known hits.

## 3D field (experimental)

**3D ↗** on the field card opens `/3d` in its own tab: the run on a 3D model
of the BIOBUZZ field, for fun and for a feel of heights. The field, perimeter
walls, alliance areas, tape, HIVE frame, tilting HIVES (from
`Hive/<RED|BLUE>/state` or the match setup) and FLOWERS use the same numbers
as the 2D drawing (`field-elements.js`), as an approximate model. The robot is
a 16 in. box with its trail. Drag to orbit, right-drag to pan, scroll to zoom;
Audience, Red driver, Blue driver, Top and Chase set the camera.

It is kept apart from the viewer: only that page loads it and three.js
(vendored in `viewer/vendor/three`, see its README). The viewer's one link to
it is a `BroadcastChannel('maxscope')` message on every seek, so with **Sync
with MaxScope** on the 3D tab follows the run and time open in MaxScope; off,
it has its own run picker and playback.

## Development

`tools/maxscope/log_viewer.py` serves the UI and reuses `analyze_wpilog.py`. Browser
code is plain ES modules and Canvas; reload after edits. The server binds
only to `127.0.0.1`, exposes an allowlist of assets, and accepts same-origin
local requests. No cloud hosting or build step is needed.

```sh
python3 -m unittest tools/logs/test_analyze_wpilog.py tools/maxscope/test_log_viewer.py tools/maxscope/test_traction.py
node --test tools/maxscope/viewer/core.test.mjs
```

Node is needed only for the browser-math unit tests. Python integration tests
briefly bind a loopback HTTP server.
