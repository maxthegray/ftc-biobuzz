# MaxScope

An offline WPILOG viewer for this robot. Python 3.10+ and a modern browser
are the only runtime requirements. There are no packages to install or
internet services to connect to.

From the repository root:

```sh
make viewer
```

Open <http://127.0.0.1:8008>. Choose a run from `robot-logs/`, including its
subdirectories, or use **Open WPILOG** for another file. Imported files are
temporary copies removed when the server stops; original logs are never
modified. Use `make pull-logs` separately to retrieve new robot logs, then
refresh the run library. The viewer does not connect to the robot.

Alternate directory or port:

```sh
python3 tools/log_viewer.py --logs /path/to/logs --port 8010
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

When a run's poses leave the field (for example an op-mode that never set a
starting pose), the view grows by whole tiles to hold the whole path, at most
one field length past each wall. A pose less than a quarter tile past a wall
doesn't add a tile. The view stays square. The field keeps its outline;
the area outside it is hatched. A pose past that limit is shown as an arrow on
the edge of the view. The view is fixed for the run, so it doesn't move while
scrubbing.

The grid is season-neutral. The robot outline is a position/heading marker,
not a measured chassis footprint. The faint trail is the recorded run and
the darker trail is its elapsed portion, not a planned path. Drawing uses a
reduced trail; the coordinate readout always uses the full pose series.

## Controls

- Three tabs: **Field**, **Signals**, and **Events**. Field starts with
  Gamepads and the run summary at the bottom; Events contains Commands and
  the event list.
- **+ View** adds a view to the current tab without removing it from other
  tabs. × in a view heading hides it from that tab. Tab contents and graph
  selections are saved in this browser; all views share playback time.
- Signals starts with one large graph. **+ Graph** adds another (up to eight);
  × beside a graph removes it. With several graphs, **Expand** temporarily
  fills the Signals view with one; **Show all graphs** restores the others.
  Existing saved graphs are retained when upgrading from the earlier viewer.
- **Dark** / **Light** in the header switches the theme. The choice is saved
  in this browser; until you pick one, the viewer follows the system setting.
- Scrub the shared time slider, click a graph, event, or command bar to seek.
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
- **+ Discrete** adds state lanes below a graph: strings, booleans, integer
  states, or individual gamepad buttons. Commands appears under the first
  graph by default. Each lane shares the graph's time axis, zoom, and cursor;
  click to seek and hover to inspect a state. Lane selections are saved.
- Field shows active commands above the replay. Its **Commands** checkbox
  toggles this display. Multiple active commands and suspended states are
  shown as recorded; this is only the command coverage available in the log.
- **Ball** (shown when the log has ball-tracking channels) draws where the
  robot thinks the tracked ball is: `BallAssist/tx`/`ty`, or the USB camera's
  `BallCamera/target/horizontalDeg` when no assist target was logged. When
  the assist's ty isn't logged (aim only), the Limelight's own ty is used if
  its tx matches. For the Limelight, the camera ray is projected onto the
  plane of a pollen's centre (2.8 in diameter) using the measured mount in
  `core.mjs`: lens 110.75 mm high, pitched 10° down. Within 48 in the ray is
  solid and ends at a ball dot, with the distance in the corner readout.
  Farther, or with no ty or mount, the ray is dashed and direction-only:
  past that range, one degree of ty moves the estimate by feet. The camera
  offset from the robot centre isn't set yet, so the ray starts at the robot
  centre. Angles more than 250 ms old are hidden.
- Scroll over a graph to zoom all graphs around the pointer. Reset zoom
  restores the full run. The slider always spans the complete run.

Samples are held until the next recorded value; no future values or
interpolated poses are invented. Before a channel's first sample its value
is unavailable. Nonfinite numbers appear as gaps. Pose age is shown after
250 ms without a sample. Graph readouts use original samples while plotting
preserves each pixel bucket's first/last/min/max values. Timing is displayed
in milliseconds; pose heading remains radians.

## Coverage and limits

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
- There is no camera-video replay, desired-path overlay, live tuning, or
  automatic fault diagnosis in this draft.
- The parser supports the types this recorder emits; it does not decode
  arbitrary WPILib structs. Logs without raw `pose` still open but have no
  field playback.
- File limit: 256 MiB. Parsing happens in the local Python server, which
  caches one decoded run. Large files can take several seconds and use much
  more RAM than their on-disk size. Playback channels and selected graphs
  are transferred in full; this is not a streaming large-log database.
- Run dates come from the filename's hub clock, which may be wrong.

## Development

`tools/log_viewer.py` serves the UI and reuses `analyze_wpilog.py`. Browser
code is plain ES modules and Canvas; reload after edits. The server binds
only to `127.0.0.1`, exposes an allowlist of assets, and accepts same-origin
local requests. No cloud hosting or build step is needed.

```sh
python3 -m unittest tools/test_analyze_wpilog.py tools/test_log_viewer.py
node --test tools/viewer/core.test.mjs
```

Node is needed only for the browser-math unit tests. Python integration tests
briefly bind a loopback HTTP server.
