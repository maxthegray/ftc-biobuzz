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

export function fieldPoint(pose, length, left, top, size) {
  // Only map Cartesian coordinates to screen pixels; no field rotation or unit conversion.
  return [left + pose[0] / length * size, top + (1 - pose[1] / length) * size];
}

export function scalarSeries(series, component = null, scale = 1) {
  return series.map(([t, value]) => {
    const scalar = component === null ? value : value?.[component];
    return [t, typeof scalar === 'boolean' ? Number(scalar) : finite(scalar) ? scalar * scale : null];
  });
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
  const labels = {pose: ['X', 'Y', 'Heading'], velocity: ['X velocity', 'Y velocity', 'Angular velocity'],
    'gamepad1/axes': axes, 'gamepad2/axes': axes};
  return channels.flatMap(channel => {
    if (!['double', 'int64', 'boolean', 'double[]'].includes(channel.type)) return [];
    const components = channel.type === 'double[]' ? Array.from({length: channel.components}, (_, i) => i) : [null];
    return components.map(component => {
      const name = channel.name;
      let unit = '', scale = 1;
      if (name === 'pose') unit = component === 2 ? 'rad' : 'in';
      else if (name === 'velocity') unit = component === 2 ? 'rad/s' : 'in/s';
      else if (name.startsWith('loop/') && name.endsWith('Nanos')) { unit = 'ms'; scale = 1e-6; }
      else if (name === 'battery') unit = 'V';
      else if (name.endsWith('Amps')) unit = 'A';
      else if (/Rad$/.test(name)) unit = 'rad';
      else if (/In$/.test(name)) unit = 'in';
      else if (/Ms$/.test(name)) unit = 'ms';
      else if (/Fps$|Hz$/.test(name)) unit = 'Hz';
      const componentLabel = component === null ? null : String(labels[name]?.[component] ?? component);
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
  return [min - padding, max + padding];
}

export const buttonNames = ['A', 'B', 'X', 'Y', 'LB', 'RB', 'Up', 'Down', 'Left', 'Right', 'Start', 'Back', 'LS', 'RS'];

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
