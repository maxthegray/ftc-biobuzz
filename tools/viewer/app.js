import {finite, indexAt, sampleAt, fieldPoint, fieldView, ballSources, ballSighting, ballEstimate, sightingMount, loggedCameraMount, cameraDetections, POLLEN_DIAMETER_IN, scalarSeries, plotPoints, plotBounds, channelOptions, channelTree, discreteOptions, discreteSeries, discreteIntervals, buttonNames} from './core.mjs';

const $ = id => document.getElementById(id);
const state = {logs: [], run: null, time: 0, playing: false, generation: 0, charts: [], options: [],
  series: new Map(), window: [0, 1], events: [], eventNodes: [], commandCursors: [], graphGeneration: 0, frame: null,
  activeTab: 'field', pickerChart: null, layout: null, focusedChart: null, panels: {field: ['field', 'camera', 'gamepads'], signals: ['signals'], events: ['commands', 'events']}};
const faultPattern = /FAULT|CRASH|FAIL|LOST|INCOMPLETE|DISABLED|OVERRUN/i;
const fmt = (n, digits = 2) => finite(n) ? n.toFixed(digits) : '—';
const node = (tag, className, text) => {
  const element = document.createElement(tag);
  if (className) element.className = className;
  if (text !== undefined) element.textContent = text;
  return element;
};

function notice(message = '', error = false) {
  $('notice').textContent = message;
  $('notice').hidden = !message;
  $('notice').classList.toggle('error', error);
}

async function api(path, options) {
  const response = await fetch(path, options);
  const result = await response.json();
  if (!response.ok) throw new Error(result.error || 'Could not read this log');
  return result;
}

function runLabel(name) {
  return name.replace(/-\d{8}-\d{6}.*\.wpilog$/, '').replace(/\.wpilog$/, '').replace(/([a-z])([A-Z])/g, '$1 $2');
}

function renderLibrary() {
  const search = $('search').value.toLowerCase();
  const runs = state.logs.filter(log => log.name.toLowerCase().includes(search));
  $('runs').replaceChildren();
  for (const log of runs) {
    const button = node('button', 'run');
    button.classList.toggle('selected', log.id === state.run?.id);
    button.title = log.name;
    button.append(node('strong', '', runLabel(log.name)));
    const match = log.name.match(/-(\d{4})(\d{2})(\d{2})-(\d{2})(\d{2})(\d{2})/);
    const date = match ? `${match[2]}/${match[3]} · ${match[4]}:${match[5]}:${match[6]}` : log.uploaded ? 'Imported' : 'Local file';
    button.append(node('small', '', `${date} · ${(log.bytes / 1048576).toFixed(1)} MiB`));
    button.onclick = () => openRun(log.id);
    $('runs').append(button);
  }
  if (!runs.length) $('runs').append(node('p', 'empty', state.logs.length ? 'No matching runs.' : 'No local logs yet. Open a WPILOG or run make pull-logs.'));
}

async function refresh() {
  try {
    const logs = await api('/api/logs');
    state.logs = [...state.logs.filter(log => log.uploaded), ...logs];
    renderLibrary();
  } catch (error) { notice(error.message, true); }
}

async function openRun(id) {
  const generation = ++state.generation;
  closePicker();
  stop();
  $('play').disabled = true;
  $('workspace').hidden = true;
  $('welcome').hidden = true;
  notice('Opening flight log… Large runs can take a few seconds.');
  try {
    const run = await api(`/api/log?id=${encodeURIComponent(id)}`);
    if (generation !== state.generation) return;
    state.run = run;
    state.time = 0;
    state.series = new Map(Object.entries(run.playback));
    state.window = [0, Math.max(run.endSec, 0.001)];
    state.options = channelOptions(run.channels);
    state.channelTree = channelTree(state.options);
    state.discreteOptions = discreteOptions(run.channels);
    state.events = [...(run.report.events || []), ...run.commandEvents].sort((a, b) => a.tSec - b.tSec);
    state.eventSeries = state.events.map(event => [event.tSec, event]);
    // The robot trail is a drawing aid. Cursor pose always uses the full native series.
    const poses = run.playback.pose || [];
    const stride = Math.max(1, Math.ceil(poses.length / 6000));
    state.trail = poses.filter((point, i) => i % stride === 0 || i === poses.length - 1);
    state.fieldView = fieldView(poses, run.fieldLengthIn);
    $('field-ball-option').hidden = !ballSources.some(source => run.playback[source.tx]?.length);
    $('run-title').textContent = runLabel(run.name);
    $('run-file').textContent = run.name;
    $('run-status').textContent = run.report.truncated ? 'Partial log' : 'No readable samples';
    $('run-status').hidden = !run.report.truncated && !run.report.empty;
    $('run-status').classList.toggle('warning', run.report.truncated || run.report.empty);
    $('field-size').textContent = `${run.fieldLengthIn} × ${run.fieldLengthIn} in`;
    $('scrub').max = run.endSec;
    $('duration').textContent = `${fmt(run.endSec)} s`;
    $('play').disabled = run.endSec <= 0;
    $('workspace').hidden = false;
    renderMetrics();
    renderCommands();
    renderEvents();
    renderGamepads();
    renderCharts();
    renderLibrary();
    seek(poses[0]?.[0] ?? 0);
    const warnings = [];
    if (run.report.truncated) warnings.push('This log has an incomplete ending. All complete records before it are available.');
    if (run.report.empty) warnings.push('This file contains no supported sample data. Choose another run.');
    if (!poses.length) warnings.push('No native pose channel was recorded; field playback is unavailable.');
    if (run.report.commandHistoryLostRecords) warnings.push(`${run.report.commandHistoryLostRecords} command records were lost.`);
    notice(warnings.join(' '));
    if (state.layout?.layers) {
      state.charts.forEach((chart, i) => { chart.keys = state.layout.layers[i] || []; });
      await loadCharts();
    } else await applyPreset(state.charts[0], 'Overview');
  } catch (error) {
    if (generation !== state.generation) return;
    notice(error.message, true);
    $('welcome').hidden = false;
  }
}

function renderMetrics() {
  const report = state.run.report;
  const faults = state.events.filter(event => /FAULT|CRASH|FAIL/i.test(event.text));
  const metrics = [
    ['RUN LENGTH', fmt(state.run.endSec, 1), 's', `${(report.recordCount || 0).toLocaleString()} records`],
    ['BATTERY MINIMUM', fmt(report.battery?.minV), 'V', `Start ${fmt(report.battery?.startV)} V`],
    ['LOOP P99', fmt(report.loop?.p99Ms), 'ms', `Peak ${fmt(report.loop?.maxMs)} ms`],
    ['FAULT RECORDS', String(faults.length), '', `${state.run.channels.length} recorded channels`],
  ];
  $('metrics').replaceChildren(...metrics.map(([label, value, unit, detail]) => {
    const item = node('div', 'metric');
    const number = node('strong', '', value);
    number.append(node('small', '', unit));
    item.append(node('div', 'label', label), number, node('div', 'detail', detail));
    return item;
  }));
}

function renderCommands() {
  const coverage = state.run.report.commandHistoryCoverage || 'none';
  const lost = state.run.report.commandHistoryLostRecords || 0;
  $('coverage').textContent = coverage === 'instrumented' ? 'INSTRUMENTED ONLY' : coverage === 'all scheduled' ? 'LEGACY SNAPSHOTS' : 'NOT RECORDED';
  $('coverage').classList.toggle('warning', lost > 0 || coverage === 'none');
  $('coverage-note').textContent = coverage === 'instrumented'
    ? `${lost ? `${lost} records lost. ` : ''}Unlogged commands are absent. FINISH means the command ended, not necessarily arrival.`
    : coverage === 'all scheduled' ? 'Legacy logs show sampled active commands; exact lifecycle bars are unavailable.'
    : 'This run has no command history. An empty timeline does not mean no commands ran.';
  $('commands').replaceChildren();
  state.commandCursors = [];
  const runs = state.run.report.commandExecutions || [];
  for (const run of runs) {
    const row = node('div', 'command');
    const label = node('div', 'command-label');
    label.append(node('span', '', `#${run.id} ${run.name}`), node('span', '', run.outcome));
    const track = node('div', 'command-track');
    const bar = node('button', `command-bar ${run.outcome.toLowerCase()}`);
    const length = Math.max(state.run.endSec, 0.001);
    bar.style.left = `${run.startSec / length * 100}%`;
    bar.style.width = `${Math.max(0, ((run.endSec ?? length) - run.startSec) / length * 100)}%`;
    bar.title = `${run.name} · ${fmt(run.startSec, 3)}–${fmt(run.endSec, 3)} s · ${run.outcome}${run.detail ? '\n' + run.detail : ''}`;
    bar.setAttribute('aria-label', bar.title);
    bar.onclick = () => seek(run.startSec);
    const cursor = node('span', 'command-cursor');
    state.commandCursors.push(cursor);
    track.append(bar, cursor);
    row.append(label, track);
    $('commands').append(row);
  }
  if (!runs.length) $('commands').append(node('p', 'empty', 'No traced executions in this run.'));
}

function renderEvents() {
  $('events').replaceChildren();
  state.eventNodes = [];
  const events = state.events.filter(event => $('event-filter').value !== 'faults' || faultPattern.test(event.text));
  // Render a bounded page; all events remain searchable through the time cursor.
  const current = indexAt(events.map(event => [event.tSec]), state.time);
  const start = Math.max(0, current - 100);
  const visible = events.slice(start, start + 500);
  if (events.length > 500) $('events').append(node('div', 'empty', `Showing ${start + 1}–${start + visible.length} of ${events.length} events around the cursor.`));
  for (const event of visible) {
    const button = node('button', `event${faultPattern.test(event.text) ? ' fault' : ''}`);
    button.append(node('time', '', `${fmt(event.tSec, 3)} s`), node('span', '', event.text));
    button.onclick = () => seek(event.tSec);
    $('events').append(button);
    state.eventNodes.push([event, button]);
  }
  if (!events.length) $('events').append(node('p', 'empty', 'No matching events in this run.'));
  state.eventPageTime = state.time;
}

function renderGamepads() {
  $('gamepads').replaceChildren();
  state.pads = [1, 2].map(number => {
    const pad = node('div', 'pad');
    const sticks = node('div', 'sticks');
    const dots = [0, 1].map(() => {
      const stick = node('div', 'stick'), dot = node('i');
      stick.append(dot); sticks.append(stick); return dot;
    });
    const data = node('div', 'pad-data');
    pad.append(node('div', 'pad-name', `GAMEPAD ${number}`), sticks, data);
    $('gamepads').append(pad);
    return {number, dots, data};
  });
}

function renderCharts() {
  $('charts').replaceChildren();
  state.charts = [];
  state.focusedChart = null;
  (state.layout?.layers || [[]]).forEach((keys, i) => createChart(keys, state.layout?.discrete?.[i] ?? (i === 0 ? ['@commands'] : [])));
  updateChartControls();
}

function createChart(keys = [], discreteKeys = []) {
  const root = node('div', 'chart'), top = node('div', 'chart-top');
  const select = node('button', 'channel-select', '+ Add signal');
  select.setAttribute('aria-haspopup', 'dialog');
  select.setAttribute('aria-controls', 'channel-picker');
  const addDiscrete = node('button', 'channel-select', '+ Discrete');
  addDiscrete.setAttribute('aria-haspopup', 'dialog');
  const lanes = node('div', 'discrete-lanes');
  const title = node('h4', 'graph-title');
  const actions = node('div', 'graph-actions');
  const preset = node('select', 'graph-preset');
  preset.append(new Option('Preset…', ''), ...['Overview', 'Drive', 'Timing', 'Vision'].map(name => new Option(name, name)));
  const focus = node('button', 'text-button graph-focus', 'Expand');
  const remove = node('button', 'text-button graph-remove', '×');
  const legend = node('div', 'chart-legend');
  const canvas = node('canvas');
  actions.append(preset, select, addDiscrete, focus, remove);
  top.append(title, actions);
  root.append(top, legend, canvas, lanes); $('charts').append(root);
  const chart = {root, title, select, preset, focus, remove, legend, canvas, keys: [...keys], discreteKeys: [...discreteKeys], lanes, addDiscrete, discrete: [], layers: [], axes: [], bounds: []};
  state.charts.push(chart);
  select.onclick = () => openPicker(chart);
  addDiscrete.onclick = () => openPicker(chart, 'discrete');
  preset.onchange = () => { if (preset.value) applyPreset(chart, preset.value); preset.value = ''; };
  focus.onclick = () => { state.focusedChart = state.focusedChart === chart ? null : chart; updateChartControls(); };
  remove.onclick = () => {
    if (state.charts.length <= 1) return;
    if (state.pickerChart === chart) closePicker();
    if (state.focusedChart === chart) state.focusedChart = null;
    state.charts = state.charts.filter(other => other !== chart);
    root.remove(); updateChartControls(); loadCharts();
  };
  wireTimeCanvas(canvas);
  return chart;
}

function wireTimeCanvas(canvas) {
  canvas.onclick = event => {
    const rect = canvas.getBoundingClientRect();
    const fraction = Math.max(0, Math.min(1, (event.clientX - rect.left - 45) / Math.max(1, rect.width - 90)));
    seek(state.window[0] + fraction * (state.window[1] - state.window[0]));
  };
  canvas.addEventListener('wheel', event => {
    event.preventDefault();
    const [start, end] = state.window;
    const rect = canvas.getBoundingClientRect();
    const fraction = Math.max(0, Math.min(1, (event.clientX - rect.left - 45) / Math.max(1, rect.width - 90)));
    const center = start + fraction * (end - start);
    const length = Math.min(Math.max(state.run.endSec, 0.001), Math.max(0.02, (end - start) * (event.deltaY > 0 ? 1.3 : 0.77)));
    const nextStart = Math.max(0, Math.min(state.run.endSec - length, center - fraction * length));
    state.window = [nextStart, nextStart + length];
    preparePlots(); drawCharts();
  }, {passive: false});
}

function updateChartControls() {
  $('charts').classList.toggle('single-graph', state.charts.length === 1 || Boolean(state.focusedChart));
  $('add-graph').disabled = state.charts.length >= 8;
  $('add-graph').title = state.charts.length >= 8 ? 'Up to 8 graphs' : 'Add an empty graph';
  state.charts.forEach((chart, i) => {
    chart.title.textContent = `Graph ${i + 1}`;
    chart.select.setAttribute('aria-label', `Add signal to graph ${i + 1}`);
    chart.addDiscrete.setAttribute('aria-label', `Add discrete field to graph ${i + 1}`);
    chart.preset.setAttribute('aria-label', `Preset for graph ${i + 1}`);
    chart.canvas.setAttribute('aria-label', `Signal graph ${i + 1}`);
    chart.remove.setAttribute('aria-label', `Remove graph ${i + 1}`);
    chart.remove.title = `Remove graph ${i + 1}`;
    chart.remove.hidden = state.charts.length === 1;
    chart.focus.hidden = state.charts.length === 1;
    chart.focus.textContent = state.focusedChart === chart ? 'Show all graphs' : 'Expand';
    chart.focus.setAttribute('aria-label', state.focusedChart === chart ? 'Show all graphs' : `Expand graph ${i + 1}`);
    chart.focus.setAttribute('aria-pressed', String(state.focusedChart === chart));
    chart.root.hidden = Boolean(state.focusedChart && state.focusedChart !== chart);
  });
  preparePlots(); drawCharts();
}

const themeKey = 'maxscope.theme';
// Past this the estimate moves by feet per degree of ty, so only the direction is drawn.
const BALL_TRUST_IN = 48;
let colors = null;
const cssColors = () => {
  const style = getComputedStyle(document.documentElement), read = name => style.getPropertyValue(`--${name}`).trim();
  const names = ['field', 'off-field', 'hatch', 'grid', 'field-label', 'wall', 'field-caption', 'trail-future', 'trail-past', 'lime', 'robot-line', 'robot-arrow', 'ball',
    'chart-grid', 'chart-label', 'chart-cursor', 'muted', 'lane', 'lane-off', 'lane-edge', 'lane-ink', 'layer-0'];
  return {...Object.fromEntries(names.map(name => [name, read(name)])),
    lanes: Array.from({length: 6}, (_, i) => read(`lane-${i}`)), layers: Array.from({length: 8}, (_, i) => read(`layer-${i}`))};
};

function discreteLabel(value, option) {
  if (value === null) return 'Not recorded';
  if (typeof value === 'boolean') return value ? 'On' : 'Off';
  if (/^gamepad[12]\/buttons$/.test(option?.source || '') && option.bit === undefined) {
    return buttonNames.filter((_, bit) => value & (1 << bit)).join(' + ') || 'None';
  }
  return String(value).replaceAll('\n', ' · ') || 'None';
}

function commandCoverage() {
  const report = state.run.report;
  return report.commandHistoryCoverage === 'instrumented'
    ? `Instrumented only${report.commandHistoryLostRecords ? ' · Records lost' : ''}`
    : report.commandHistoryCoverage === 'all scheduled' ? 'Legacy samples' : 'Not recorded';
}

function renderDiscrete(chart) {
  chart.lanes.replaceChildren();
  for (const lane of chart.discrete) {
    const row = node('div', 'discrete-lane');
    const header = node('div', 'discrete-heading');
    const label = lane.option?.label || `${lane.key} (not recorded)`;
    header.append(node('span', 'discrete-name', label));
    if (lane.key === '@commands') header.append(node('span', 'discrete-coverage', commandCoverage()));
    lane.value = node('span', 'discrete-value');
    const remove = node('button', 'layer-remove', '×');
    remove.setAttribute('aria-label', `Remove discrete ${label} from graph ${state.charts.indexOf(chart) + 1}`);
    remove.onclick = () => { chart.discreteKeys = chart.discreteKeys.filter(key => key !== lane.key); loadCharts(); };
    header.append(lane.value, remove);
    lane.canvas = node('canvas');
    lane.canvas.setAttribute('aria-label', `${label} discrete timeline`);
    wireTimeCanvas(lane.canvas);
    lane.canvas.onmousemove = event => {
      const rect = lane.canvas.getBoundingClientRect();
      const time = state.window[0] + Math.max(0, Math.min(1, (event.clientX - rect.left - 45) / Math.max(1, rect.width - 90))) * (state.window[1] - state.window[0]);
      lane.canvas.title = `${fmt(time, 3)} s · ${discreteLabel(sampleAt(lane.data, time), lane.option)}`;
    };
    row.append(header, lane.canvas); chart.lanes.append(row);
  }
}

function drawDiscrete(chart) {
  for (const lane of chart.discrete) {
    const {ctx, width, height} = context(lane.canvas);
    const [start, end] = state.window, left = 45, right = width - 45;
    const x = time => left + (time - start) / (end - start) * (right - left);
    const value = sampleAt(lane.data, state.time);
    lane.value.textContent = chart.loading ? 'Loading…' : discreteLabel(value, lane.option);
    lane.value.title = lane.value.textContent;
    ctx.fillStyle = colors.lane; ctx.fillRect(left, 5, right - left, height - 10);
    ctx.save(); ctx.beginPath(); ctx.rect(left, 0, right - left, height); ctx.clip();
    ctx.font = '10px ui-monospace, monospace'; ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
    const palette = colors.lanes;
    for (let i = Math.max(0, indexAt(lane.intervals, start)); i < lane.intervals.length; i++) {
      const [time, interval] = lane.intervals[i];
      if (time > end) break;
      if (interval.end <= start) continue;
      const a = x(Math.max(start, time)), b = x(Math.min(end, interval.end));
      const label = discreteLabel(interval.value, lane.option);
      let hash = 0;
      for (const char of label) hash = (hash * 31 + char.charCodeAt(0)) >>> 0;
      ctx.fillStyle = interval.value === false || interval.value === '' || interval.value === 0 ? colors['lane-off'] : palette[hash % palette.length];
      ctx.fillRect(a, 5, Math.max(1, b - a), height - 10);
      ctx.strokeStyle = colors['lane-edge']; ctx.beginPath(); ctx.moveTo(a, 5); ctx.lineTo(a, height - 5); ctx.stroke();
      if (b - a > 24) {
        ctx.save(); ctx.beginPath(); ctx.rect(a + 5, 5, b - a - 10, height - 10); ctx.clip();
        ctx.fillStyle = colors['lane-ink']; ctx.fillText(label, a + 7, height / 2); ctx.restore();
      }
    }
    if (state.time >= start && state.time <= end) {
      ctx.strokeStyle = colors['layer-0']; ctx.setLineDash([3, 3]); ctx.beginPath();
      ctx.moveTo(x(state.time), 0); ctx.lineTo(x(state.time), height); ctx.stroke(); ctx.setLineDash([]);
    }
    if (!lane.data.length) {
      ctx.fillStyle = colors['chart-label']; ctx.fillText(chart.loading ? 'Loading…' : 'No recorded samples', left + 8, height / 2);
    }
    ctx.restore();
  }
}

function renderLegend(chart) {
  chart.legend.replaceChildren();
  chart.layers.forEach(layer => {
    const item = node('div', 'legend-item');
    const label = node('label', 'layer-label');
    const check = node('input');
    check.type = 'checkbox'; check.checked = layer.visible;
    check.setAttribute('aria-label', `Show ${layer.option?.label || layer.key}`);
    check.onchange = () => { layer.visible = check.checked; preparePlots(); drawCharts(); };
    label.style.color = `var(--layer-${layer.slot})`;
    label.append(check, node('span', '', layer.option?.label || `${layer.key.split('::')[0]} (not recorded)`));
    layer.value = node('span', 'chart-value', '—');
    const axis = node('span', 'axis-label', layer.axis === 1 ? 'R' : 'L');
    axis.title = layer.axis === 1 ? 'Right axis' : 'Left axis';
    const remove = node('button', 'layer-remove', '×');
    remove.setAttribute('aria-label', `Remove ${layer.option?.label || layer.key}`);
    remove.onclick = () => { chart.keys = chart.keys.filter(key => key !== layer.key); loadCharts(); };
    item.append(label, axis, layer.value, remove);
    chart.legend.append(item);
  });
}

function selectTab(name, focus = false) {
  state.activeTab = name;
  for (const view of views.values()) $('view-storage').append(view);
  for (const key of state.panels[name]) $(`panel-${name}`).append(views.get(key));
  if (name === 'field') $('panel-field').append($('metrics'));
  updateViewControls();
  for (const tab of document.querySelectorAll('[role="tab"]')) {
    const selected = tab.dataset.view === name;
    tab.setAttribute('aria-selected', String(selected));
    tab.tabIndex = selected ? 0 : -1;
    $(tab.getAttribute('aria-controls')).hidden = !selected;
    if (selected && focus) tab.focus();
  }
  preparePlots();
  if (state.run) seek(state.time);
}

const viewNames = {field: 'Field', signals: 'Signals', commands: 'Commands', events: 'Events', gamepads: 'Gamepads', camera: 'Camera'};
const views = new Map();
const layoutKey = 'maxscope.layout.v2';

function saveLayout() {
  state.layout = {panels: state.panels, layers: state.charts.length ? state.charts.map(chart => [...chart.keys]) : state.layout?.layers,
    discrete: state.charts.length ? state.charts.map(chart => [...chart.discreteKeys]) : state.layout?.discrete,
    fieldCommands: $('field-commands-toggle').checked, fieldBall: $('field-ball-toggle').checked};
  try { localStorage.setItem(layoutKey, JSON.stringify(state.layout)); } catch { /* Storage may be disabled. */ }
}

function updateViewControls() {
  for (const option of $('add-view').options) option.disabled = state.panels[state.activeTab].includes(option.value);
  for (const panel of document.querySelectorAll('.view-panel')) {
    let empty = panel.querySelector('.empty-tab');
    if (!empty) { empty = node('p', 'empty empty-tab', 'Use + View to add a view.'); panel.append(empty); }
    empty.hidden = state.panels[panel.id.slice(6)].length > 0;
  }
}

function addView(key) {
  const keys = state.panels[state.activeTab];
  if (!views.has(key) || keys.includes(key)) return;
  keys.push(key);
  saveLayout(); selectTab(state.activeTab);
}

function wireTab(tab) {
  tab.onclick = () => selectTab(tab.dataset.view);
  tab.onkeydown = event => {
    const tabs = [...document.querySelectorAll('[role="tab"]')], index = tabs.indexOf(tab);
    let next;
    if (event.key === 'ArrowRight') next = (index + 1) % tabs.length;
    if (event.key === 'ArrowLeft') next = (index - 1 + tabs.length) % tabs.length;
    if (event.key === 'Home') next = 0;
    if (event.key === 'End') next = tabs.length - 1;
    if (next !== undefined) { event.preventDefault(); selectTab(tabs[next].dataset.view, true); }
  };
}

function initializeLayout() {
  const elements = {field: document.querySelector('.field-card'), signals: document.querySelector('.signals-card'),
    commands: $('commands').closest('.card'), events: $('events').closest('.card'), gamepads: document.querySelector('.inputs-card'),
    camera: document.querySelector('.camera-card')};
  for (const [key, view] of Object.entries(elements)) {
    view.dataset.panel = key; views.set(key, view);
    const remove = node('button', 'text-button hide-view', '×');
    remove.setAttribute('aria-label', `Hide ${viewNames[key]} from this tab`);
    remove.title = 'Hide from this tab';
    remove.onclick = () => {
      state.panels[state.activeTab] = state.panels[state.activeTab].filter(item => item !== key);
      saveLayout(); selectTab(state.activeTab);
    };
    view.querySelector('.card-heading').append(remove);
  }
  document.querySelectorAll('[role="tab"]').forEach(wireTab);
  try {
    const saved = JSON.parse(localStorage.getItem(layoutKey));
    $('field-commands-toggle').checked = saved?.fieldCommands !== false;
    $('field-ball-toggle').checked = saved?.fieldBall !== false;
    const previous = saved || JSON.parse(localStorage.getItem('maxscope.layout.v1'));
    if (saved?.panels) {
      for (const tab of Object.keys(state.panels)) {
        if (Array.isArray(saved.panels[tab])) state.panels[tab] = [...new Set(saved.panels[tab].filter(key => views.has(key)))];
      }
    }
    if (Array.isArray(previous?.layers) && previous.layers.length > 0) {
      state.layout = {discrete: Array.isArray(previous.discrete) ? previous.discrete.slice(0, 8).map(keys => Array.isArray(keys) ? [...new Set(keys.filter(key => typeof key === 'string'))].slice(0, 8) : []) : undefined, layers: previous.layers.slice(0, 8).map(keys => Array.isArray(keys) ? [...new Set(keys.filter(key => typeof key === 'string'))].slice(0, 8) : [])};
    }
  } catch { /* Ignore an unavailable or outdated saved layout. */ }
  selectTab('field');
}

function closePicker() {
  if ($('channel-picker').open) $('channel-picker').close();
  state.pickerChart = null;
}

function openPicker(chart, mode = 'continuous') {
  state.pickerChart = chart;
  state.pickerMode = mode;
  $('channel-search').value = '';
  $('picker-title').textContent = `Graph ${state.charts.indexOf(chart) + 1} ${mode === 'discrete' ? 'discrete field' : 'signal'}`;
  $('channel-clear').textContent = mode === 'discrete' ? 'Clear discrete fields' : 'Clear graph';
  $('channel-clear').disabled = !(mode === 'discrete' ? chart.discreteKeys : chart.keys).length;
  document.querySelector('.picker-note').textContent = mode === 'discrete' ? 'Up to 8 discrete lanes · Strings, buttons, and states' : 'Layer up to 8 signals · Two unit scales per graph';
  renderChannelTree();
  $('channel-picker').showModal();
  $('channel-search').focus();
}

function chooseChannel(option) {
  if (!state.pickerChart) return;
  const chart = state.pickerChart;
  const keyList = state.pickerMode === 'discrete' ? 'discreteKeys' : 'keys';
  if (option) {
    if (!chart[keyList].includes(option.key)) chart[keyList].push(option.key);
  } else chart[keyList] = [];
  closePicker();
  loadCharts();
}

function renderChannelTree() {
  const tree = $('channel-tree');
  tree.replaceChildren();
  const query = $('channel-search').value.trim().toLowerCase();
  const discrete = state.pickerMode === 'discrete';
  const options = discrete ? state.discreteOptions : state.options;
  const selectedKeys = (discrete ? state.pickerChart?.discreteKeys : state.pickerChart?.keys) || [];
  const selected = options.filter(option => selectedKeys.includes(option.key));
  const units = new Set(selected.map(option => option.unit));
  const leaf = (option, label) => {
    const button = node('button', 'channel-leaf');
    button.title = option.label;
    const included = selectedKeys.includes(option.key);
    button.setAttribute('aria-pressed', String(included));
    button.disabled = included || selectedKeys.length >= 8 || (!discrete && units.size >= 2 && !units.has(option.unit));
    if (button.disabled && !included) button.title = selectedKeys.length >= 8 ? (discrete ? 'Up to 8 discrete fields per graph' : 'Up to 8 signals per graph') : 'Each graph supports two unit scales; use another graph for this unit.';
    button.append(node('span', 'channel-name', label), node('span', 'channel-unit', option.unit));
    button.onclick = () => chooseChannel(option);
    return button;
  };
  if (query) {
    const matches = options.filter(option => query.split(/\s+/).every(word => option.label.toLowerCase().includes(word)));
    for (const option of matches) tree.append(leaf(option, option.label));
    if (!matches.length) tree.append(node('p', 'empty', 'No matching channels.'));
    $('channel-count').textContent = `${matches.length} matches`;
    return;
  }
  const render = (parent, target, path = []) => {
    const children = [...parent.children.values()].sort((a, b) => Number(b.children.size > 0) - Number(a.children.size > 0) || a.name.localeCompare(b.name));
    for (const child of children) {
      const childPath = [...path, child.name];
      if (child.children.size) {
        const folder = node('details', 'channel-folder');
        const summary = node('summary');
        const icon = node('span', 'folder-icon');
        icon.setAttribute('aria-hidden', 'true');
        summary.append(icon, node('span', '', child.name));
        const contents = node('div', 'folder-contents');
        const prefix = childPath.join('/');
        folder.open = selected.some(option => option.name === prefix || option.name.startsWith(prefix + '/'));
        for (const option of child.options) contents.append(leaf(option, 'Value'));
        render(child, contents, childPath);
        folder.append(summary, contents);
        target.append(folder);
      } else {
        for (const option of child.options) target.append(leaf(option, child.name));
      }
    }
  };
  render(discrete ? channelTree(options) : state.channelTree, tree);
  $('channel-count').textContent = `${options.length} ${discrete ? 'fields' : 'signals'}`;
}

async function applyPreset(chart, preset) {
  const candidates = {
    Overview: ['battery', 'loop/totalNanos'],
    Drive: ['velocity::0', 'velocity::1', 'velocity::2'],
    Timing: ['loop/totalNanos', 'loop/windowMaxTotalNanos', 'loop/recordNanos'],
    Vision: ['BallCamera/frame/ageMs', 'BallCamera/frame/processingMs', 'BallCamera/target/horizontalDeg'],
  }[preset];
  chart.keys = [];
  for (const key of candidates) {
    const option = state.options.find(option => option.key === key || option.name === key)
      || (preset === 'Vision' ? state.options.find(option => option.name.endsWith(key.slice(key.indexOf('/')))) : null);
    if (option && !chart.keys.includes(option.key)) chart.keys.push(option.key);
  }
  await loadCharts();
}

async function loadCharts() {
  if (!state.run) return;
  const generation = ++state.graphGeneration, runGeneration = state.generation;
  const definitions = state.charts.map(chart => chart.keys.map(key => ({key, option: state.options.find(option => option.key === key)})));
  const discreteDefinitions = state.charts.map(chart => chart.discreteKeys.map(key => ({key, option: state.discreteOptions.find(option => option.key === key)})));
  const requested = new Set([...definitions.flat().map(layer => layer.option?.name), ...discreteDefinitions.flat().map(lane => lane.option?.source)].filter(Boolean));
  const names = [...requested].filter(name => !state.series.has(name));
  state.charts.forEach((chart, i) => {
    const visibility = new Map(chart.layers.map(layer => [layer.key, layer.visible]));
    chart.axes = [...new Set(definitions[i].filter(layer => layer.option).map(layer => layer.option.unit))];
    chart.layers = definitions[i].map((layer, j) => ({...layer, slot: j % 8, visible: visibility.get(layer.key) ?? true,
      axis: Math.max(0, chart.axes.indexOf(layer.option?.unit)), data: [], points: []}));
    chart.loading = true;
    chart.discrete = discreteDefinitions[i].map(lane => ({...lane, data: [], intervals: []}));
    renderLegend(chart); renderDiscrete(chart);
  });
  saveLayout();
  drawCharts();
  try {
    for (let offset = 0; offset < names.length; offset += 8) {
      const query = new URLSearchParams({id: state.run.id});
      names.slice(offset, offset + 8).forEach(name => query.append('channel', name));
      const result = await api(`/api/series?${query}`);
      if (runGeneration !== state.generation || generation !== state.graphGeneration) return;
      for (const [name, series] of Object.entries(result)) state.series.set(name, series);
    }
    if (runGeneration !== state.generation || generation !== state.graphGeneration) return;
    const selected = requested;
    for (const key of state.series.keys()) {
      if (state.series.size <= 24) break;
      if (!selected.has(key)) state.series.delete(key);
    }
    state.charts.forEach(chart => {
      chart.loading = false;
      for (const layer of chart.layers) {
        const option = layer.option;
        layer.data = option ? scalarSeries(state.series.get(option.name) || [], option.component, option.scale) : [];
      }
      for (const lane of chart.discrete) {
        lane.data = discreteSeries(state.series.get(lane.option?.source) || [], lane.option?.bit);
        lane.intervals = discreteIntervals(lane.data, state.run.endSec);
      }
    });
    preparePlots(); drawCharts();
  } catch (error) {
    if (runGeneration === state.generation && generation === state.graphGeneration) {
      state.charts.forEach(chart => { chart.loading = false; });
      notice(error.message, true); drawCharts();
    }
  }
}

function preparePlots() {
  for (const chart of state.charts) {
    if (!chart.canvas.clientWidth) continue;
    for (const layer of chart.layers) layer.points = plotPoints(layer.data, ...state.window, Math.max(100, Math.floor(chart.canvas.clientWidth)));
    chart.bounds = chart.axes.map((unit, axis) => plotBounds(chart.layers, axis, state.window[1]));
  }
}

function context(canvas) {
  const ratio = window.devicePixelRatio || 1;
  const width = canvas.clientWidth, height = canvas.clientHeight;
  if (canvas.width !== Math.round(width * ratio) || canvas.height !== Math.round(height * ratio)) {
    canvas.width = Math.round(width * ratio); canvas.height = Math.round(height * ratio);
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  ctx.clearRect(0, 0, width, height);
  return {ctx, width, height};
}

function drawField() {
  if (!state.run || !$('field').clientWidth) return;
  const {ctx, width, height} = context($('field'));
  const length = state.run.fieldLengthIn, view = state.fieldView, size = Math.min(width - 78, height - 58);
  const left = (width - size) / 2 + 8, top = 17, cell = size / view.tiles;
  const [fieldLeft, fieldTop] = fieldPoint([0, length], view, left, top, size), fieldSize = length / view.span * size;
  const expanded = view.tiles > 6;
  ctx.fillStyle = expanded ? colors['off-field'] : colors.field; ctx.fillRect(left, top, size, size);
  if (expanded) {
    ctx.save(); ctx.beginPath(); ctx.rect(left, top, size, size); ctx.rect(fieldLeft, fieldTop, fieldSize, fieldSize); ctx.clip('evenodd');
    ctx.strokeStyle = colors.hatch; ctx.lineWidth = 1; ctx.beginPath();
    for (let d = 0; d <= size * 2; d += 8) { ctx.moveTo(left + d, top); ctx.lineTo(left + d - size, top + size); }
    ctx.stroke(); ctx.restore();
    ctx.fillStyle = colors.field; ctx.fillRect(fieldLeft, fieldTop, fieldSize, fieldSize);
  }
  ctx.lineWidth = 1; ctx.strokeStyle = colors.grid; ctx.font = '9px ui-monospace, monospace';
  const labelEvery = Math.ceil(view.tiles / 6);
  for (let i = 0; i <= view.tiles; i++) {
    const at = i * cell, tileX = Math.round(view.minX / view.tile) + i, tileY = Math.round(view.minY / view.tile) + view.tiles - i;
    ctx.beginPath(); ctx.moveTo(left + at, top); ctx.lineTo(left + at, top + size); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(left, top + at); ctx.lineTo(left + size, top + at); ctx.stroke();
    const label = index => fmt(index * view.tile, index === 0 ? 0 : 1);
    ctx.fillStyle = colors['field-label'];
    if (tileX % labelEvery === 0) { ctx.textAlign = 'center'; ctx.fillText(label(tileX), left + at, top + size + 16); }
    if (tileY % labelEvery === 0) { ctx.textAlign = 'right'; ctx.fillText(label(tileY), left - 8, top + at + 3); }
  }
  if (expanded) { ctx.strokeStyle = colors.wall; ctx.lineWidth = 1.5; ctx.strokeRect(fieldLeft, fieldTop, fieldSize, fieldSize); }
  ctx.fillStyle = colors['field-caption']; ctx.textAlign = 'center';
  ctx.fillText('AUDIENCE WALL · +X →', left + size / 2, height - 4);
  ctx.textAlign = 'left'; ctx.fillText('+Y ↑', 8, top + 6);
  ctx.save(); ctx.beginPath(); ctx.rect(left, top, size, size); ctx.clip();
  const trail = state.trail || [];
  const drawTrail = (past, color) => {
    ctx.strokeStyle = color; ctx.lineWidth = past ? 2 : 1.5; ctx.beginPath();
    let connected = false;
    for (const [t, pose] of trail) {
      if (!Array.isArray(pose) || !pose.slice(0, 2).every(finite) || (past && t > state.time)) { connected = false; continue; }
      const [x, y] = fieldPoint(pose, view, left, top, size);
      if (connected) ctx.lineTo(x, y); else ctx.moveTo(x, y);
      connected = true;
    }
    ctx.stroke();
  };
  drawTrail(false, colors['trail-future']); drawTrail(true, colors['trail-past']);
  const series = state.run.playback.pose || [];
  const pose = sampleAt(series, state.time);
  const valid = Array.isArray(pose) && pose.length >= 3 && pose.slice(0, 3).every(finite);
  $('pose-x').textContent = fmt(pose?.[0]); $('pose-y').textContent = fmt(pose?.[1]); $('pose-h').textContent = fmt(pose?.[2], 3);
  const index = indexAt(series, state.time);
  const age = index >= 0 ? state.time - series[index][0] : null;
  const outside = valid && (pose[0] < 0 || pose[1] < 0 || pose[0] > length || pose[1] > length);
  const [x, y] = valid ? fieldPoint(pose, view, left, top, size) : [];
  const slack = view.slack / view.span * size;
  const offView = valid && (x < left - slack || x > left + size + slack || y < top - slack || y > top + size + slack);
  $('pose-status').textContent = !valid ? 'No valid pose at this time' : offView ? 'Pose is beyond the view' : outside ? 'Pose is outside the field' : age > 0.25 ? `Last pose ${fmt(age)} s ago` : '';
  const ball = !$('field-ball-option').hidden && $('field-ball-toggle').checked && valid ? ballSighting(state.run.playback, state.time) : null;
  const angle = value => value === null ? '—' : `${value >= 0 ? '+' : ''}${fmt(value, 1)}°`;
  $('ball-status').hidden = $('field-ball-option').hidden || !$('field-ball-toggle').checked;
  const estimate = ball ? ballEstimate(pose, ball, sightingMount(state.run.playback, state.time, ball)) : null;
  const near = estimate?.distanceIn !== null && estimate?.distanceIn <= BALL_TRUST_IN;
  const range = !estimate || estimate.distanceIn === null ? '' : near ? ` · ~${fmt(estimate.distanceIn, 0)} in` : ' · far';
  $('ball-status').textContent = !ball ? 'No ball target' : `tx ${angle(ball.txDeg)} · ty ${angle(ball.tyDeg)}${ball.targetTyDeg === null ? '' : ` → ${angle(ball.targetTyDeg)}`}${range}`;
  const ballRadius = Math.max(4, POLLEN_DIAMETER_IN / 2 / view.span * size);
  if (ball && !offView && ball.source === 'BallCamera') {
    const mount = loggedCameraMount(state.run.playback, state.time);
    for (const detection of cameraDetections(state.run.playback, state.time)?.detections || []) {
      if (detection.selected || detection.rejection || !mount || detection.txDeg === null || detection.tyDeg === null) continue;
      const other = ballEstimate(pose, detection, mount);
      if (other.distanceIn === null || other.distanceIn > BALL_TRUST_IN) continue;
      const [otherX, otherY] = fieldPoint(other.point, view, left, top, size);
      ctx.strokeStyle = colors.ball; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.arc(otherX, otherY, ballRadius, 0, Math.PI * 2); ctx.stroke();
    }
  }
  if (estimate && !offView) {
    const end = near ? estimate.point : [estimate.origin[0] + Math.cos(estimate.bearing) * view.span * 2, estimate.origin[1] + Math.sin(estimate.bearing) * view.span * 2];
    const [startX, startY] = fieldPoint(estimate.origin, view, left, top, size);
    const [endX, endY] = fieldPoint(end, view, left, top, size);
    ctx.strokeStyle = colors.ball; ctx.lineWidth = 2; ctx.setLineDash(near ? [] : [6, 4]);
    ctx.beginPath(); ctx.moveTo(startX, startY); ctx.lineTo(endX, endY); ctx.stroke(); ctx.setLineDash([]);
    if (near) {
      ctx.fillStyle = colors.ball; ctx.beginPath();
      ctx.arc(endX, endY, ballRadius, 0, Math.PI * 2); ctx.fill();
    }
  }
  if (offView) {
    const edgeX = Math.min(left + size - 2, Math.max(left + 2, x)), edgeY = Math.min(top + size - 2, Math.max(top + 2, y));
    ctx.translate(edgeX, edgeY); ctx.rotate(Math.atan2(y - edgeY, x - edgeX));
    ctx.fillStyle = colors['robot-arrow']; ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(-12, -6); ctx.lineTo(-12, 6); ctx.closePath(); ctx.fill();
  } else if (valid) {
    const radius = Math.max(5, 8 * size / view.span);
    ctx.translate(x, y); ctx.rotate(-pose[2]);
    ctx.fillStyle = colors.lime; ctx.strokeStyle = colors['robot-line']; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.roundRect(-radius, -radius, radius * 2, radius * 2, 4); ctx.fill(); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(radius * 1.65, 0); ctx.lineTo(radius * 1.2, -3); ctx.moveTo(radius * 1.65, 0); ctx.lineTo(radius * 1.2, 3); ctx.stroke();
    ctx.beginPath(); ctx.arc(0, 0, 2, 0, Math.PI * 2); ctx.fillStyle = colors['robot-line']; ctx.fill();
  }
  ctx.restore();
}

function drawCamera() {
  if (!state.run || !$('camera').clientWidth) return;
  const {ctx, width, height} = context($('camera'));
  const frame = cameraDetections(state.run.playback, state.time);
  $('camera-status').textContent = !frame ? 'No camera detections in this log' : !frame.fresh ? 'No fresh camera frame' : '';
  const accepted = frame?.detections.filter(detection => !detection.rejection).length ?? 0;
  $('camera-summary').textContent = frame?.fresh ? `${frame.detections.length} candidates · ${accepted} accepted` : '';
  const frameWidth = frame?.widthPx ?? 640, frameHeight = frame?.heightPx ?? 480;
  $('camera-size').textContent = frame?.widthPx ? `${frameWidth} × ${frameHeight} px` : '';
  const scale = Math.min((width - 24) / frameWidth, (height - 16) / frameHeight);
  const left = (width - frameWidth * scale) / 2, top = 8;
  ctx.fillStyle = colors.field; ctx.fillRect(left, top, frameWidth * scale, frameHeight * scale);
  ctx.strokeStyle = colors.grid; ctx.lineWidth = 1; ctx.strokeRect(left, top, frameWidth * scale, frameHeight * scale);
  ctx.setLineDash([3, 4]); ctx.beginPath();
  ctx.moveTo(left + frameWidth * scale / 2, top); ctx.lineTo(left + frameWidth * scale / 2, top + frameHeight * scale);
  ctx.moveTo(left, top + frameHeight * scale / 2); ctx.lineTo(left + frameWidth * scale, top + frameHeight * scale / 2);
  ctx.stroke(); ctx.setLineDash([]);
  if (!frame) return;
  ctx.save(); ctx.beginPath(); ctx.rect(left, top, frameWidth * scale, frameHeight * scale); ctx.clip();
  ctx.font = '9px ui-monospace, monospace'; ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
  for (const detection of [...frame.detections].reverse()) {
    if (detection.xPx === null || detection.yPx === null) continue;
    const cx = left + detection.xPx * scale, cy = top + detection.yPx * scale, r = Math.max(3, (detection.radiusPx ?? 0) * scale);
    ctx.globalAlpha = detection.rejection ? 0.45 : 1;
    ctx.strokeStyle = detection.rejection ? colors.muted : colors.ball; ctx.lineWidth = detection.selected ? 2.5 : 1.5;
    ctx.setLineDash(detection.rejection ? [3, 3] : []);
    ctx.beginPath(); ctx.arc(cx, cy, r, 0, Math.PI * 2);
    if (detection.selected) { ctx.fillStyle = colors.ball; ctx.globalAlpha = 0.35; ctx.fill(); ctx.globalAlpha = 1; }
    ctx.stroke(); ctx.setLineDash([]);
    const label = detection.rejection ?? (detection.txDeg === null ? '' : `${fmt(detection.txDeg, 1)}°, ${fmt(detection.tyDeg, 1)}°`);
    if (label) { ctx.fillStyle = detection.rejection ? colors.muted : colors['lane-ink']; ctx.fillText(label, cx + r + 4, cy); }
  }
  ctx.restore(); ctx.globalAlpha = 1;
}

function drawCharts() {
  for (const chart of state.charts) {
    if (!chart.canvas.clientWidth) continue;
    const {ctx, width, height} = context(chart.canvas);
    const [start, end] = state.window;
    const left = 45, right = width - 45, top = 21, bottom = height - 22;
    const x = t => left + (t - start) / (end - start) * (right - left);
    ctx.font = '9px ui-monospace, monospace'; ctx.lineWidth = 1;
    for (let i = 0; i <= 2; i++) {
      const at = top + (bottom - top) * i / 2;
      ctx.strokeStyle = colors['chart-grid']; ctx.beginPath(); ctx.moveTo(left, at); ctx.lineTo(right, at); ctx.stroke();
    }
    chart.axes.forEach((unit, axis) => {
      const [min, max] = chart.bounds[axis] || [0, 1];
      ctx.fillStyle = colors['chart-label']; ctx.textAlign = axis === 0 ? 'right' : 'left';
      const at = axis === 0 ? left - 7 : right + 7;
      ctx.fillText(unit || 'value', at, 10);
      for (let i = 0; i <= 2; i++) ctx.fillText(fmt(max - (max - min) * i / 2, Math.abs(max) > 100 ? 0 : 1), at, top + (bottom - top) * i / 2 + 3);
    });
    ctx.fillStyle = colors.muted; ctx.textAlign = 'left'; ctx.fillText(`${fmt(start, 1)} s`, left, height - 4);
    ctx.textAlign = 'right'; ctx.fillText(`${fmt(end, 1)} s`, right, height - 4);
    ctx.save(); ctx.beginPath(); ctx.rect(left, top - 3, right - left, bottom - top + 6); ctx.clip();
    for (const layer of chart.layers) {
      const value = sampleAt(layer.data, state.time);
      layer.value.textContent = chart.loading ? '…' : `${fmt(value)}${layer.option?.unit ? ' ' + layer.option.unit : ''}`;
      const sampled = indexAt(layer.data, state.time);
      layer.value.title = sampled < 0 ? 'No sample at this time' : `Last sample at ${fmt(layer.data[sampled][0], 6)} s`;
      if (!layer.visible) continue;
      const [min, max] = chart.bounds[layer.axis] || [0, 1];
      const y = value => bottom - (value - min) / (max - min) * (bottom - top);
      ctx.strokeStyle = colors.layers[layer.slot]; ctx.lineWidth = 1.5; ctx.beginPath();
      let connected = false, previousValue = null;
      for (const [t, value] of layer.points) {
        if (!finite(value)) { connected = false; continue; }
        if (connected) { ctx.lineTo(x(t), y(previousValue)); ctx.lineTo(x(t), y(value)); }
        else ctx.moveTo(x(t), y(value));
        connected = true; previousValue = value;
      }
      ctx.stroke();
    }
    if (state.time >= start && state.time <= end) {
      ctx.strokeStyle = colors['chart-cursor']; ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(x(state.time), top); ctx.lineTo(x(state.time), bottom); ctx.stroke(); ctx.setLineDash([]);
    }
    ctx.restore();
    drawDiscrete(chart);
    if (!chart.layers.some(layer => layer.data.length)) {
      ctx.fillStyle = colors.muted; ctx.textAlign = 'center';
      ctx.fillText(chart.loading ? 'Loading samples…' : chart.keys.length ? 'No recorded samples' : 'Add signals to compare them', (left + right) / 2, (top + bottom) / 2);
    }
  }
}

function updateDetails() {
  const playback = state.run.playback;
  const active = sampleAt(playback['commands/active']?.length ? playback['commands/active'] : playback['commands/running'], state.time);
  const mode = sampleAt(playback.driveMode, state.time);
  $('field-activity').hidden = !$('field-commands-toggle').checked;
  $('field-command-coverage').textContent = commandCoverage();
  const commandText = active === null ? 'No command sample at this time' : active || 'No active logged commands';
  if ($('field-active-commands').dataset.value !== commandText) {
    $('field-active-commands').dataset.value = commandText;
    $('field-active-commands').replaceChildren(...commandText.split('\n').map(name => node('span', 'active-command-chip', name)));
  }
  $('active').textContent = `${mode || 'No drive mode recorded'}${active ? '\n' + active : ''}`;
  const progress = state.run.endSec ? state.time / state.run.endSec * 100 : 0;
  state.commandCursors.forEach(cursor => { cursor.style.left = `${progress}%`; });
  if (state.events.length > 500 && Math.abs(state.time - state.eventPageTime) > 5) renderEvents();
  const current = sampleAt(state.eventSeries, state.time);
  for (const [event, element] of state.eventNodes) element.classList.toggle('current', event === current);
  for (const pad of state.pads) {
    const axes = sampleAt(playback[`gamepad${pad.number}/axes`], state.time);
    const mask = sampleAt(playback[`gamepad${pad.number}/buttons`], state.time);
    pad.dots.forEach((dot, i) => {
      dot.style.left = `${50 + (finite(axes?.[i * 2]) ? axes[i * 2] : 0) * 38}%`;
      dot.style.top = `${50 - (finite(axes?.[i * 2 + 1]) ? axes[i * 2 + 1] : 0) * 38}%`;
      dot.style.opacity = axes ? '1' : '0';
    });
    const buttons = finite(mask) ? buttonNames.filter((_, i) => mask & (1 << i)).join(' ') || 'None' : 'Not recorded';
    pad.data.textContent = axes ? `LT ${fmt(axes[4])}  RT ${fmt(axes[5])}\nButtons: ${buttons}` : 'No input sample at this time';
  }
}

function seek(time) {
  if (!state.run) return;
  state.time = Math.max(0, Math.min(state.run.endSec, time));
  $('scrub').value = state.time;
  $('time').textContent = `${fmt(state.time, 3)} s`;
  drawField(); drawCamera(); drawCharts(); updateDetails();
}

function stop() {
  state.playing = false;
  cancelAnimationFrame(state.frame);
  $('play').textContent = '▶'; $('play').setAttribute('aria-label', 'Play');
}

$('play').onclick = () => {
  if (state.playing) { stop(); return; }
  if (state.time >= state.run.endSec) seek(0);
  state.playing = true;
  $('play').textContent = 'Ⅱ'; $('play').setAttribute('aria-label', 'Pause');
  let last = performance.now();
  const tick = now => {
    if (!state.playing) return;
    seek(state.time + (now - last) / 1000 * Number($('speed').value));
    last = now;
    if (state.time >= state.run.endSec) stop();
    else state.frame = requestAnimationFrame(tick);
  };
  state.frame = requestAnimationFrame(tick);
};
$('scrub').oninput = () => { stop(); seek(Number($('scrub').value)); };
$('reset-zoom').onclick = () => { state.window = [0, Math.max(state.run.endSec, 0.001)]; preparePlots(); drawCharts(); };
$('add-graph').onclick = () => {
  if (!state.run || state.charts.length >= 8) return;
  state.focusedChart = null;
  const chart = createChart();
  updateChartControls(); saveLayout();
  chart.select.focus();
};
$('event-filter').onchange = () => { renderEvents(); updateDetails(); };
$('field-commands-toggle').onchange = () => { saveLayout(); if (state.run) updateDetails(); };
$('field-ball-toggle').onchange = () => { saveLayout(); if (state.run) drawField(); };
$('refresh').onclick = refresh;
$('search').oninput = renderLibrary;
$('file').onchange = async event => {
  const file = event.target.files[0];
  if (!file) return;
  if (file.size > 256 * 1048576) { notice('This draft supports logs up to 256 MiB.', true); return; }
  notice('Opening local file…');
  try {
    const log = await api(`/api/upload?name=${encodeURIComponent(file.name)}`, {method: 'POST', body: file, headers: {'Content-Type': 'application/octet-stream'}});
    state.logs.unshift(log); renderLibrary(); await openRun(log.id);
  } catch (error) { notice(error.message, true); }
  event.target.value = '';
};
document.addEventListener('keydown', event => {
  if ($('channel-picker').open || ['INPUT', 'SELECT', 'BUTTON', 'TEXTAREA'].includes(event.target.tagName) || !state.run || $('workspace').hidden) return;
  if (event.code === 'Space') { event.preventDefault(); $('play').click(); }
  if (event.code === 'ArrowLeft' || event.code === 'ArrowRight') {
    event.preventDefault(); stop(); seek(state.time + (event.code === 'ArrowRight' ? 1 : -1) * (event.shiftKey ? 1 : 0.02));
  }
});
$('picker-close').onclick = closePicker;
$('channel-clear').onclick = () => chooseChannel(null);
$('channel-search').oninput = renderChannelTree;
$('channel-picker').addEventListener('close', () => { state.pickerChart = null; });
$('add-view').onchange = event => { addView(event.target.value); event.target.value = ''; };
function applyTheme(theme) {
  document.documentElement.dataset.theme = theme;
  colors = cssColors();
  const next = theme === 'dark' ? 'light' : 'dark';
  $('theme-toggle').textContent = theme === 'dark' ? '☀ Light' : '☾ Dark';
  $('theme-toggle').setAttribute('aria-label', `Switch to ${next} mode`);
  if (state.run && !$('workspace').hidden) { drawField(); drawCamera(); drawCharts(); }
}
$('theme-toggle').onclick = () => {
  const theme = document.documentElement.dataset.theme === 'dark' ? 'light' : 'dark';
  try { localStorage.setItem(themeKey, theme); } catch { /* Storage may be disabled. */ }
  applyTheme(theme);
};
matchMedia('(prefers-color-scheme: dark)').addEventListener('change', event => {
  let saved = null;
  try { saved = localStorage.getItem(themeKey); } catch { /* Storage may be disabled. */ }
  if (!saved) applyTheme(event.matches ? 'dark' : 'light');
});
applyTheme(document.documentElement.dataset.theme === 'dark' ? 'dark' : 'light');
initializeLayout();
new ResizeObserver(() => { if (state.run && !$('workspace').hidden) { preparePlots(); drawField(); drawCamera(); drawCharts(); } }).observe($('workspace'));
await refresh();
