export const finite = v => typeof v === 'number' && Number.isFinite(v);

export function indexAt(series, time) {
  let lo = 0, hi = series.length;
  while (lo < hi) {
    const mid = (lo + hi) >>> 1;
    if (series[mid][0] <= time) lo = mid + 1;
    else hi = mid;
  }
  return lo - 1;
}

export function sampleAt(series = [], time) {
  const index = indexAt(series, time);
  return index < 0 ? null : series[index][1];
}

export function fieldPoint(pose, view, left, top, size) {
  // Only map Cartesian coordinates to screen pixels; no field rotation or unit conversion.
  return [left + (pose[0] - view.minX) / view.span * size, top + (1 - (pose[1] - view.minY) / view.span) * size];
}

// Half the side of the robot marker drawn on the field.
export const ROBOT_HALF_IN = 8;

// Square view over the field and the whole robot at every pose of the run, in whole field tiles.
// The robot counts by its drawn body at its heading, so a robot half over a wall still shows in
// full; overhanging by under `toleranceIn` doesn't add a tile. Each side grows by at most
// `maxExtraFields` field lengths so a pose glitch can't shrink the field to a dot.
export function fieldView(poses, length, maxExtraFields = 1, toleranceIn = 0.5) {
  const tile = length / 6, cap = 6 * maxExtraFields;
  let minX = 0, maxX = length, minY = 0, maxY = length;
  for (const [, pose] of poses) {
    if (!Array.isArray(pose) || !finite(pose[0]) || !finite(pose[1])) continue;
    const reach = finite(pose[2]) ? ROBOT_HALF_IN * (Math.abs(Math.cos(pose[2])) + Math.abs(Math.sin(pose[2]))) : ROBOT_HALF_IN * Math.SQRT2;
    minX = Math.min(minX, pose[0] - reach); maxX = Math.max(maxX, pose[0] + reach);
    minY = Math.min(minY, pose[1] - reach); maxY = Math.max(maxY, pose[1] + reach);
  }
  const tiles = (lo, hi) => [Math.max(-cap, Math.floor((lo + toleranceIn) / tile)), Math.min(6 + cap, Math.ceil((hi - toleranceIn) / tile))];
  const x = tiles(minX, maxX), y = tiles(minY, maxY);
  const count = Math.max(x[1] - x[0], y[1] - y[0]);
  const low = ([lo, hi]) => lo - Math.floor((count - (hi - lo)) / 2);
  return {minX: low(x) * tile, minY: low(y) * tile, span: count * tile, tile, tiles: count};
}

// Ball the robot is tracking, as camera angles in degrees: +tx right, +ty up. BallAssist is what
// the Limelight assist acted on; its ty is only logged while approaching, so the Limelight's own ty
// fills in when its tx shows it is the same target. BallCamera is the USB camera, whose vertical
// angle is positive below the axis and whose mount comes from its own log.
export const assistSource = {name: 'BallAssist', tx: 'BallAssist/tx', ty: 'BallAssist/ty', targetTy: 'BallAssist/targetTy',
  fallbackVisible: 'Limelight/target/visible', fallbackTx: 'Limelight/target/txDegrees', fallbackTy: 'Limelight/target/tyDegrees',
  mount: 'limelight'};

// Every camera subsystem in a log, by the channel prefix its own name gives it.
export function cameraSources(channels = []) {
  const names = new Set(channels.map(channel => channel.name ?? channel));
  const prefixes = new Set();
  for (const name of names) {
    const match = name.match(/^(.+)\/(?:candidates\/xPx|target\/horizontalDeg)$/);
    if (match) prefixes.add(match[1]);
  }
  return [...prefixes].sort();
}

const cameraSighting = camera => ({name: camera, camera, tx: `${camera}/target/horizontalDeg`,
  ty: `${camera}/target/verticalDeg`, tyPositiveDown: true, mount: 'logged'});

export function ballSources(cameras = []) {
  return [assistSource, ...cameras.map(cameraSighting)];
}

// Logs from before the camera mount was logged. Measured on the robot; offset from the centre unknown.
export const cameraMounts = {limelight: {heightIn: 110.75 / 25.4, pitchDownDeg: 10, forwardIn: 0, leftIn: 0, yawDeg: 0}};
export const POLLEN_DIAMETER_IN = 2.8;

export function ballSighting(playback, time, cameras = [], maxAgeSec = 0.25) {
  const fresh = name => {
    const series = playback[name] || [], index = indexAt(series, time);
    return index >= 0 && finite(series[index][1]) && time - series[index][0] <= maxAgeSec ? series[index][1] : null;
  };
  for (const source of ballSources(cameras)) {
    const tx = fresh(source.tx);
    if (tx === null) continue;
    let ty = fresh(source.ty);
    if (ty === null && source.fallbackTy && sampleAt(playback[source.fallbackVisible], time) === true && Math.abs((fresh(source.fallbackTx) ?? Infinity) - tx) < 0.5) ty = fresh(source.fallbackTy);
    if (ty !== null && source.tyPositiveDown) ty = -ty;
    const targetTy = source.targetTy ? sampleAt(playback[source.targetTy], time) : null;
    return {source: source.name, camera: source.camera ?? null, mount: source.mount, txDeg: tx, tyDeg: ty, targetTyDeg: finite(targetTy) ? targetTy : null};
  }
  return null;
}

export function loggedCameraMount(playback, time, camera) {
  const value = name => sampleAt(playback[`${camera}/mount/${name}`], time);
  if (value('measured') !== true) return null;
  const mount = {heightIn: value('heightIn'), pitchDownDeg: value('pitchDownDeg'),
    forwardIn: value('forwardIn') ?? 0, leftIn: value('leftIn') ?? 0, yawDeg: value('yawDeg') ?? 0};
  return Object.values(mount).every(finite) ? mount : null;
}

export function sightingMount(playback, time, sighting) {
  return sighting?.mount === 'logged' ? loggedCameraMount(playback, time, sighting.camera) : cameraMounts[sighting?.mount] ?? null;
}

// Every candidate the USB camera published for its frame at `time`, accepted first. Angles are
// converted to +up. Empty when the candidate sample is older than `maxAgeSec`.
export function cameraDetections(playback, time, camera, maxAgeSec = 0.25) {
  const xs = playback[`${camera}/candidates/xPx`];
  if (!xs?.length) return null;
  const value = name => sampleAt(playback[`${camera}/${name}`], time);
  const index = indexAt(xs, time), fresh = index >= 0 && time - xs[index][0] <= maxAgeSec;
  const column = name => fresh && Array.isArray(value(`candidates/${name}`)) ? value(`candidates/${name}`) : [];
  const x = fresh && Array.isArray(xs[index][1]) ? xs[index][1] : [];
  const y = column('yPx'), radius = column('radiusPx'), horizontal = column('horizontalDeg'), vertical = column('verticalDeg');
  const rejections = fresh && typeof value('candidates/rejections') === 'string' ? value('candidates/rejections').split(',') : [];
  const selectedIndex = fresh ? value('candidates/selectedIndex') : -1;
  const number = v => finite(v) ? v : null;
  return {
    widthPx: number(value('frame/widthPx')) || null, heightPx: number(value('frame/heightPx')) || null,
    status: value('frame/status') ?? null, fresh,
    detections: x.map((xPx, i) => ({xPx: number(xPx), yPx: number(y[i]), radiusPx: number(radius[i]),
      txDeg: number(horizontal[i]), tyDeg: finite(vertical[i]) ? -vertical[i] : null,
      rejection: rejections[i] && rejections[i] !== 'accepted' ? rejections[i] : null, selected: i === selectedIndex})),
  };
}

// Intersects the camera ray with the plane of the ball's centre. The ray starts at the camera
// (mount offset from the pose point) along the robot heading plus the mount yaw. Without a mount
// or ty only the bearing is known.
export function ballEstimate(pose, sighting, mount, ballDiameterIn = POLLEN_DIAMETER_IN) {
  const rad = Math.PI / 180, heading = pose[2];
  const forwardIn = mount?.forwardIn ?? 0, leftIn = mount?.leftIn ?? 0;
  const origin = [pose[0] + Math.cos(heading) * forwardIn - Math.sin(heading) * leftIn,
    pose[1] + Math.sin(heading) * forwardIn + Math.cos(heading) * leftIn];
  const pitch = (mount?.pitchDownDeg ?? 0) * rad, ranged = Boolean(mount) && sighting.tyDeg !== null;
  const right = Math.tan(sighting.txDeg * rad), up = ranged ? Math.tan(sighting.tyDeg * rad) : 0;
  const forward = ranged ? up * Math.sin(pitch) + Math.cos(pitch) : 1;
  const bearing = heading + (mount?.yawDeg ?? 0) * rad - Math.atan2(right, forward);
  const drop = Math.sin(pitch) - up * Math.cos(pitch), above = (mount?.heightIn ?? 0) - ballDiameterIn / 2;
  if (!ranged || drop <= 0 || above <= 0) return {origin, bearing, distanceIn: null, point: null};
  const distanceIn = Math.hypot(forward, right) * above / drop;
  return {origin, bearing, distanceIn, point: [origin[0] + Math.cos(bearing) * distanceIn, origin[1] + Math.sin(bearing) * distanceIn]};
}

export function scalarSeries(series, component = null, scale = 1) {
  return series.map(([t, value]) => {
    const scalar = component === null ? value
      : component === 'speed' ? (finite(value?.[0]) && finite(value?.[1]) ? Math.hypot(value[0], value[1]) : null)
      : value?.[component];
    return [t, typeof scalar === 'boolean' ? Number(scalar) : finite(scalar) ? scalar * scale : null];
  });
}

// The lowest and highest samples in [start, end], each as [time, value]; ties keep the earliest.
export function extremes(series, start, end) {
  let min = null, max = null;
  for (let i = Math.max(0, indexAt(series, start)); i < series.length && series[i][0] <= end; i++) {
    const point = series[i];
    if (point[0] < start || !finite(point[1])) continue;
    if (!min || point[1] < min[1]) min = point;
    if (!max || point[1] > max[1]) max = point;
  }
  return min && {min, max};
}

export function plotPoints(series, start, end, buckets = 600) {
  if (!series.length || end <= start) return [];
  const first = Math.max(0, indexAt(series, start));
  const last = Math.min(series.length - 1, indexAt(series, end) + 1);
  const result = [];
  let group = [], previousBucket = -1;
  const flush = () => {
    if (!group.length) return;
    let min = group[0], max = group[0];
    for (const point of group) {
      if (point[1] < min[1]) min = point;
      if (point[1] > max[1]) max = point;
    }
    const chosen = [...new Set([group[0], min, max, group.at(-1)])].sort((a, b) => a[0] - b[0]);
    result.push(...chosen);
    group = [];
  };
  for (let i = first; i <= last; i++) {
    const point = series[i];
    const bucket = Math.floor((point[0] - start) / (end - start) * buckets);
    if (!finite(point[1])) {
      flush();
      if (result.at(-1)?.[1] !== null) result.push([point[0], null]);
    } else {
      if (bucket !== previousBucket) flush();
      group.push(point);
    }
    previousBucket = bucket;
  }
  flush();
  return result;
}

export function channelOptions(channels) {
  const axes = ['Left stick X', 'Left stick Y', 'Right stick X', 'Right stick Y', 'Left trigger', 'Right trigger'];
  const wheels = ['leftFront', 'rightFront', 'leftRear', 'rightRear'];
  const labels = {pose: ['X', 'Y', 'Heading'], velocity: ['X velocity', 'Y velocity', 'Angular velocity'],
    'gamepad1/axes': axes, 'gamepad2/axes': axes, 'Drive/motors/power': wheels, 'Drive/motors/velocityTps': wheels,
    'MaxScope/traction/wheels': ['Forward', 'Left', 'Turn'], 'MaxScope/traction/pinpoint': ['Forward', 'Left', 'Turn']};
  // Speed is derived: the magnitude of the field-frame translational velocity.
  const derived = {velocity: {speed: 'Speed'}};
  return channels.flatMap(channel => {
    if (!['double', 'int64', 'boolean', 'double[]'].includes(channel.type)) return [];
    const components = channel.type === 'double[]' ? Array.from({length: channel.components}, (_, i) => i) : [null];
    if (channel.type === 'double[]' && channel.components >= 2) components.push(...Object.keys(derived[channel.name] ?? {}));
    return components.map(component => {
      const name = channel.name;
      let unit = '', scale = 1;
      if (name === 'pose') unit = component === 2 ? 'rad' : 'in';
      else if (name === 'velocity' || name.startsWith('MaxScope/traction/')) unit = component === 2 ? 'rad/s' : 'in/s';
      else if (name.startsWith('loop/') && name.endsWith('Nanos')) { unit = 'ms'; scale = 1e-6; }
      else if (name === 'battery' || name.endsWith('Volts') || name.endsWith('/volts')) unit = 'V';
      else if (name.endsWith('Amps')) unit = 'A';
      else if (/Rad$/.test(name)) unit = 'rad';
      else if (/In$/.test(name)) unit = 'in';
      else if (/Ms$/.test(name)) unit = 'ms';
      else if (/Fps$|Hz$/.test(name)) unit = 'Hz';
      else if (/Tps$/.test(name)) unit = 'ticks/s';
      const componentLabel = component === null ? null
        : String(derived[name]?.[component] ?? labels[name]?.[component] ?? component);
      const suffix = componentLabel === null ? '' : ` / ${componentLabel}`;
      return {key: `${name}::${component ?? ''}`, name, component, componentLabel, unit, scale, label: name + suffix};
    });
  });
}

export function channelTree(options) {
  const root = {children: new Map(), options: []};
  for (const option of options) {
    const parts = option.name.split('/');
    if (option.componentLabel !== null) parts.push(option.componentLabel);
    let current = root;
    for (const part of parts) {
      if (!current.children.has(part)) current.children.set(part, {name: part, children: new Map(), options: []});
      current = current.children.get(part);
    }
    current.options.push(option);
  }
  return root;
}

// A graph's left and right margins in CSS pixels. Unit scales alternate left, right, left, …;
// each scale past the first on a side sits further out and widens that margin.
export const OUTER_AXIS_WIDTH = 42;
export function plotMargins(axisCount) {
  return [45 + Math.max(0, Math.ceil(axisCount / 2) - 1) * OUTER_AXIS_WIDTH,
    45 + Math.max(0, Math.floor(axisCount / 2) - 1) * OUTER_AXIS_WIDTH];
}

export function plotBounds(layers, axis, end) {
  let min = Infinity, max = -Infinity;
  for (const layer of layers) {
    if (layer.axis !== axis || !layer.visible) continue;
    for (const [time, value] of layer.points) {
      if (finite(value) && time <= end) { min = Math.min(min, value); max = Math.max(max, value); }
    }
  }
  if (!finite(min)) { min = 0; max = 1; }
  const padding = (max - min) * 0.12 || Math.max(Math.abs(max) * 0.05, 0.1);
  // Padding never crosses zero: a loop time or a current that is never negative gets no
  // negative axis.
  return [min >= 0 && max > 0 ? Math.max(0, min - padding) : min - padding,
    max <= 0 && min < 0 ? Math.min(0, max + padding) : max + padding];
}

// When comparing, where each run is lined up: the op-mode start event, the first time the robot
// moves more than `thresholdIn` from where it began, or the log's first record. Falls back to 0.
export function alignmentAnchor(events = [], poses = [], mode, thresholdIn = 1) {
  if (mode === 'start') return events.find(event => event.text === 'start')?.tSec ?? 0;
  if (mode !== 'motion') return 0;
  let origin = null;
  for (const [time, pose] of poses) {
    if (!Array.isArray(pose) || !finite(pose[0]) || !finite(pose[1])) continue;
    if (!origin) origin = pose;
    else if (Math.hypot(pose[0] - origin[0], pose[1] - origin[1]) > thresholdIn) return time;
  }
  return 0;
}

export const shiftSeries = (series = [], offset) => offset ? series.map(([time, value]) => [time + offset, value]) : series;

// The next problem starting after `time`, or the last one starting before it. Problems are sorted.
export function adjacentProblem(problems, time, direction) {
  const epsilon = 1e-6;
  return (direction > 0 ? problems.find(item => item.startSec > time + epsilon)
    : problems.findLast(item => item.startSec < time - epsilon)) ?? null;
}

// Problems under the cursor: spans that cover it, and point problems within `nearSec` of it.
export function problemsAt(problems, time, nearSec = 0.1) {
  return problems.filter(item => item.endSec > item.startSec
    ? item.startSec <= time && time <= item.endSec
    : Math.abs(item.startSec - time) <= nearSec);
}

export const buttonNames = ['Cross', 'Circle', 'Square', 'Triangle', 'L1', 'R1', 'Up', 'Down', 'Left', 'Right', 'Options', 'Create', 'L3', 'R3', 'Touchpad'];

export function discreteOptions(channels) {
  const commandSource = ['commands/active', 'commands/running'].find(name => channels.some(channel => channel.name === name));
  const options = [{key: '@commands', name: 'Commands', source: commandSource, label: 'Commands', componentLabel: null, unit: 'state'}];
  for (const channel of channels) {
    if (!['string', 'boolean', 'int64'].includes(channel.type) || ['events', 'commands/events', 'commands/active', 'commands/running'].includes(channel.name)) continue;
    const base = {key: channel.name, name: channel.name, source: channel.name, label: channel.name, componentLabel: null, unit: channel.type === 'boolean' ? 'on/off' : 'state'};
    options.push(base);
    if (/^gamepad[12]\/buttons$/.test(channel.name)) {
      buttonNames.forEach((label, bit) => options.push({...base, key: `${channel.name}::${bit}`, label: `${channel.name} / ${label}`, componentLabel: label, bit, unit: 'on/off'}));
    }
  }
  return options;
}

export function discreteSeries(series, bit) {
  return series.map(([time, value]) => [time, bit !== undefined
    ? (finite(value) ? Boolean(value & (1 << bit)) : null)
    : (typeof value === 'string' || typeof value === 'boolean' || finite(value) ? value : null)]);
}

// Spans per command name from the sampled active set, for logs without lifecycle records. A name
// is active while it appears in consecutive samples; sampling makes the edges approximate.
export function activeCommandSpans(series = [], endSec) {
  const open = new Map(), spans = [];
  const names = value => new Set(typeof value === 'string' && value ? value.split('\n').map(line => line.replace(/^#\d+ /, '').replace(/ \(suspended\)$/, '')).filter(Boolean) : []);
  for (const [time, value] of series) {
    const active = names(value);
    for (const [name, start] of [...open]) {
      if (!active.has(name)) { spans.push({name, startSec: start, endSec: time, outcome: 'SAMPLED'}); open.delete(name); }
    }
    for (const name of active) if (!open.has(name)) open.set(name, time);
  }
  for (const [name, start] of open) spans.push({name, startSec: start, endSec, outcome: 'SAMPLED'});
  return spans.sort((a, b) => a.startSec - b.startSec);
}

export function discreteIntervals(series, end) {
  const changes = [];
  for (const [time, value] of series) {
    if (changes.at(-1)?.[0] === time) changes.pop();
    if (!changes.length || changes.at(-1)[1] !== value) changes.push([time, value]);
  }
  return changes.flatMap(([start, value], i) => {
    const stop = Math.min(changes[i + 1]?.[0] ?? end, end);
    return value !== null && start < stop ? [[start, {end: stop, value}]] : [];
  });
}

// BIOBUZZ milestones on the log clock, from the op-mode start event.
// Null when the op-mode never started.
export function matchPhases(match) {
  const start = match?.startSec;
  if (!finite(start)) return null;
  const phases = match.kind === 'auto'
    ? [['INIT', 0, start], ['AUTO', start, start + match.autoSec], ['AFTER AUTO', start + match.autoSec, Infinity]]
    : [['INIT', 0, start], ['TELEOP', start, start + match.flowerUnlockSec],
      ['FLOWER OWNERSHIP UNLOCKED', start + match.flowerUnlockSec, start + match.finalTwentySec],
      ['FINAL 20 SECONDS', start + match.finalTwentySec, start + match.driverSec],
      ['AFTER MATCH', start + match.driverSec, Infinity]];
  const end = start + (match.kind === 'auto' ? match.autoSec : match.driverSec);
  return {start, end, phases: phases.map(([name, from, to]) => ({name, from, to})),
    marks: match.kind === 'auto' ? [{tSec: start, label: 'start'}, {tSec: end, label: '0:30'}]
      : [{tSec: start, label: 'start'}, {tSec: start + match.flowerUnlockSec, label: 'FLOWER ownership unlocked'},
        {tSec: start + match.finalTwentySec, label: 'Final 20 seconds'}, {tSec: end, label: '2:00'}]};
}

export const minutes = seconds => `${Math.floor(seconds / 60)}:${String(Math.floor(seconds % 60)).padStart(2, '0')}`;

// The phase at `time` and what the driver station clock would read: time left in the period, or
// time past its end.
export function phaseAt(match, time) {
  const phase = match?.phases.find(item => time >= item.from && time < item.to);
  if (!phase) return null;
  if (phase.name === 'INIT') return {name: phase.name, clock: ''};
  if (phase.to === Infinity) return {name: phase.name, clock: `+${minutes(time - match.end)}`};
  return {name: phase.name, clock: `${minutes(Math.ceil(match.end - time - 1e-9))} left`};
}

// Each time the robot passed within `radiusIn` of `point`, as {startSec, endSec, atSec}, where
// atSec is that pass's closest sample.
export function posePasses(series, point, radiusIn) {
  const passes = [];
  let current = null;
  for (const [time, pose] of series) {
    const distance = Array.isArray(pose) && finite(pose[0]) && finite(pose[1]) ? Math.hypot(pose[0] - point[0], pose[1] - point[1]) : Infinity;
    if (distance <= radiusIn) {
      if (!current) { current = {startSec: time, endSec: time, atSec: time, distance}; passes.push(current); }
      current.endSec = time;
      if (distance < current.distance) { current.distance = distance; current.atSec = time; }
    } else current = null;
  }
  return passes.map(({startSec, endSec, atSec}) => ({startSec, endSec, atSec}));
}

// The pass to jump to from `time`: the first one starting after it, wrapping to the first.
export function nextPass(passes, time) {
  return passes.find(pass => pass.startSec > time + 1e-6) ?? passes[0] ?? null;
}

// The graph window while following playback: `length` seconds with the cursor `lead` of the way
// across, so more of the past shows than of what is coming, clamped to the run. A length longer
// than the run, including Infinity for "All", shows the whole run.
export function followWindow(time, length, end, lead = 2 / 3) {
  const runEnd = Math.max(end, 0.001), span = Math.min(length, runEnd);
  const start = Math.max(0, Math.min(runEnd - span, time - span * lead));
  return [start, start + span];
}
