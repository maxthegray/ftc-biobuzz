import {finite, indexAt, sampleAt, activeCommandSpans, fieldView, ballSources, cameraSources, channelOptions, channelTree, discreteOptions, buttonNames, alignmentAnchor, shiftSeries, adjacentProblem, problemsAt} from './core.mjs';
import {$, state, fmt, node, api, notice, commandCoverage, refreshColors} from './ui.js';
import {createCharts} from './charts.js';
import {createLayout} from './layout.js';
import {drawField, drawCamera, fillCameraSources} from './field-camera.js';

// Whole words, so "schedule default Drive" isn't a FAULT. Matches the server's problem rule.
const faultPattern = /\b(?:FAULT|CRASH|FAIL(?:ED|URE)?|LOST|INCOMPLETE|DISABLED|OVERRUN)\b/i;
const FIELD_TIMELINE_LANES = 5;
const themeKey = 'maxscope.theme';
const {closePicker, renderCharts, loadCharts, applyPreset, preparePlots, drawCharts} = createCharts({seek, saveLayout: () => saveLayout()});
const {initializeLayout, saveLayout, setProblemsView} = createLayout({seek, preparePlots, redraw});

function redraw() { drawField(); drawCamera(); drawCharts(); }

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
    button.append(node('small', '', `${runDate(log)} · ${(log.bytes / 1048576).toFixed(1)} MiB`));
    button.onclick = () => openRun(log.id);
    $('runs').append(button);
  }
  if (!runs.length) $('runs').append(node('p', 'empty', state.logs.length ? 'No matching runs.' : 'No local logs yet. Open a WPILOG or run make pull-logs.'));
  renderCompareOptions();
}

function runDate(log) {
  const match = log.name.match(/-(\d{4})(\d{2})(\d{2})-(\d{2})(\d{2})(\d{2})/);
  return match ? `${match[2]}/${match[3]} · ${match[4]}:${match[5]}:${match[6]}` : log.uploaded ? 'Imported' : 'Local file';
}

function renderCompareOptions() {
  const select = $('compare-run');
  select.replaceChildren(new Option('Compare with…', ''), ...state.logs
    .filter(log => log.id !== state.run?.id)
    .map(log => new Option(`${runLabel(log.name)} · ${runDate(log)}`, log.id, false, log.id === state.compare?.id)));
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
    state.problems = [];
    state.problemCounts = {};
    state.problemFilter = null;
    if (state.compare?.id === id) clearCompare(false);
    state.eventSeries = state.events.map(event => [event.tSec, event]);
    const poses = run.playback.pose || [];
    state.trail = downsample(poses);
    if (state.compare) alignCompare(); else updateFieldView();
    state.cameras = cameraSources(run.channels);
    $('field-ball-option').hidden = !ballSources(state.cameras).some(source => run.playback[source.tx]?.length);
    for (const view of state.cameraViews) fillCameraSources(view);
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
    renderProblems();
    if (state.problemsOn) loadProblems();
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
  const report = state.run.report, other = state.compare?.run.report;
  const faults = (events = []) => events.filter(event => faultPattern.test(event.text)).length;
  const versus = (value, unit, digits = 2) => other ? ` · B ${fmt(value, digits)}${unit ? ' ' + unit : ''}` : '';
  const metrics = [
    ['RUN LENGTH', fmt(state.run.endSec, 1), 's', `${(report.recordCount || 0).toLocaleString()} records${versus(state.compare?.run.endSec, 's', 1)}`],
    ['BATTERY MINIMUM', fmt(report.battery?.minV), 'V', `Start ${fmt(report.battery?.startV)} V${versus(other?.battery?.minV, 'V')}`],
    ['LOOP P99', fmt(report.loop?.p99Ms), 'ms', `Peak ${fmt(report.loop?.maxMs)} ms${versus(other?.loop?.p99Ms, 'ms')}`],
    ['FAULT EVENTS', String(faults(state.run.report.events)), '', `${state.run.channels.length} recorded channels${other ? ` · B ${faults(other.events)}` : ''}`],
  ];
  $('metrics').replaceChildren(...metrics.map(([label, value, unit, detail]) => {
    const item = node('div', 'metric');
    const number = node('strong', '', value);
    number.append(node('small', '', unit));
    item.append(node('div', 'label', label), number, node('div', 'detail', detail));
    return item;
  }));
}

async function loadProblems() {
  const generation = ++state.problemGeneration, id = state.run.id;
  state.problemsScanning = true;
  renderProblems();
  try {
    const result = await api(`/api/problems?id=${encodeURIComponent(id)}`);
    if (generation !== state.problemGeneration || !state.problemsOn || state.run?.id !== id) return;
    state.problems = result.problems;
    state.problemCounts = result.counts;
    state.problemsScanning = false;
    renderProblems();
    seek(state.time);
  } catch (error) {
    if (generation === state.problemGeneration) { state.problemsScanning = false; notice(error.message, true); renderProblems(); }
  }
}

function updateProblemsButton() {
  const count = state.problems.length;
  $('problems-toggle').textContent = !state.problemsOn ? 'Find problems' : state.problemsScanning ? 'Scanning…' : `${count} problem${count === 1 ? '' : 's'}`;
  $('problems-toggle').setAttribute('aria-pressed', String(state.problemsOn));
  $('problems-toggle').classList.toggle('fault', state.problemsOn && state.problems.some(item => item.severity === 'fault'));
  $('problems-toggle').title = state.problemsOn ? 'Hide problems' : 'Scan this run for problems · [ and ] step between them';
}

async function setProblems(on) {
  state.problemsOn = on;
  state.problemGeneration++;
  state.problems = [];
  state.problemCounts = {};
  state.problemFilter = null;
  state.problemsScanning = false;
  if (!state.run) return;
  setProblemsView(on);
  renderProblems();
  seek(state.time);
  if (on) await loadProblems();
}

function renderProblems() {
  updateProblemsButton();
  const problems = state.problems;
  const kinds = new Map();
  for (const item of problems) kinds.set(item.kind, (kinds.get(item.kind) || 0) + 1);
  const truncated = Object.entries(state.problemCounts).filter(([kind, count]) => count > (kinds.get(kind) || 0));
  if (state.problemFilter && !kinds.has(state.problemFilter)) state.problemFilter = null;
  $('problem-count').textContent = state.problemsScanning ? 'SCANNING' : problems.length ? `${problems.length} FLAGGED` : 'NONE FLAGGED';
  $('problem-count').classList.toggle('warning', problems.some(item => item.severity === 'fault'));
  $('problem-prev').disabled = $('problem-next').disabled = !problems.length;
  $('problem-kinds').replaceChildren(...[...kinds].map(([kind, count]) => {
    const chip = node('button', `problem-kind ${problems.find(item => item.kind === kind).severity}`, `${kind} · ${count}`);
    chip.setAttribute('aria-pressed', String(state.problemFilter === kind));
    chip.title = state.problemFilter === kind ? 'Show every problem' : `Show only ${kind.toLowerCase()}`;
    chip.onclick = () => { state.problemFilter = state.problemFilter === kind ? null : kind; renderProblems(); updateDetails(); };
    return chip;
  }));
  state.problemNodes = [];
  const shown = problems.filter(item => !state.problemFilter || item.kind === state.problemFilter);
  $('problems').replaceChildren(...shown.map(item => {
    const button = node('button', `problem ${item.severity}`);
    button.append(node('time', '', `${fmt(item.startSec, 2)} s`), node('strong', '', item.kind), node('span', '', item.detail));
    button.onclick = () => { stop(); seek(item.startSec); };
    state.problemNodes.push([item, button]);
    return button;
  }));
  if (!problems.length) $('problems').append(node('p', 'empty', state.problemsScanning ? 'Scanning this run…' : 'Nothing flagged in this run. Only the rules above are checked.'));
  for (const [kind, count] of truncated) $('problems').append(node('p', 'empty', `${kind}: showing the first ${kinds.get(kind) || 0} of ${count}.`));
  renderMarkers();
}

// Problem spans on the playback slider. The range thumb's centre can't reach the track ends, so
// the markers are inset by half a thumb to line up with it.
function renderMarkers() {
  const length = Math.max(state.run.endSec, 0.001);
  $('scrub-markers').replaceChildren(...state.problems.map(item => {
    const marker = node('span', `scrub-marker ${item.severity}`);
    marker.style.setProperty('--start', String(item.startSec / length));
    marker.style.setProperty('--span', String(Math.max(0, item.endSec - item.startSec) / length));
    marker.title = `${fmt(item.startSec, 2)} s · ${item.kind}${item.detail ? ' · ' + item.detail : ''}`;
    marker.onclick = () => { stop(); seek(item.startSec); };
    return marker;
  }));
}

async function stepProblem(direction) {
  if (!state.problemsOn) await setProblems(true);
  const target = adjacentProblem(state.problems.filter(item => !state.problemFilter || item.kind === state.problemFilter), state.time, direction);
  if (target) { stop(); seek(target.startSec); }
}

// The robot trail is a drawing aid. Cursor pose always uses the full native series.
function downsample(poses) {
  const stride = Math.max(1, Math.ceil(poses.length / 6000));
  return poses.filter((point, i) => i % stride === 0 || i === poses.length - 1);
}

function updateFieldView() {
  const poses = [...(state.run.playback.pose || []), ...(state.compare?.run.playback.pose || [])];
  state.fieldView = fieldView(poses, state.run.fieldLengthIn);
}

async function openCompare(id) {
  if (!id) { clearCompare(); return; }
  const generation = ++state.compareGeneration;
  notice('Opening the run to compare…');
  try {
    const run = await api(`/api/log?id=${encodeURIComponent(id)}`);
    if (generation !== state.compareGeneration || !state.run) return;
    state.compare = {id, run, raw: new Map(Object.entries(run.playback)), series: new Map(), offset: 0, trail: []};
    $('compare-name').textContent = `B: ${runLabel(run.name)} · ${runDate(run)}`;
    $('compare-options').hidden = false;
    $('compare-legend').hidden = false;
    alignCompare();
    renderMetrics();
    notice('');
    await loadCharts();
  } catch (error) {
    if (generation === state.compareGeneration) { notice(error.message, true); clearCompare(); }
  }
}

// B's samples are shifted onto this run's clock once per alignment, so every view reads them
// with the same lookups as this run's own.
function alignCompare() {
  const compare = state.compare;
  const mode = $('compare-align').value, manual = Number($('compare-shift').value) || 0;
  const anchor = (events, poses) => alignmentAnchor(events, poses, mode);
  compare.offset = anchor(state.events, state.run.playback.pose) - anchor(compare.run.report.events, compare.run.playback.pose) + manual;
  compare.series = new Map([...compare.raw].map(([name, series]) => [name, shiftSeries(series, compare.offset)]));
  compare.trail = downsample(compare.series.get('pose') || []);
  updateFieldView();
}

function clearCompare(redraw = true) {
  state.compareGeneration++;
  state.compare = null;
  $('compare-run').value = '';
  $('compare-options').hidden = true;
  $('compare-legend').hidden = true;
  $('compare-pose').hidden = true;
  if (redraw && state.run) { updateFieldView(); renderMetrics(); loadCharts(); seek(state.time); }
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
  const playback = state.run.playback;
  renderFieldTimeline(runs.length ? runs
    : activeCommandSpans(playback['commands/active']?.length ? playback['commands/active'] : playback['commands/running'], state.run.endSec));
}

// The same executions as the Commands view, one row per command name, so the field card shows the
// whole run at a glance under the commands active right now.
function renderFieldTimeline(runs) {
  const length = Math.max(state.run.endSec, 0.001);
  const byName = new Map();
  for (const run of runs) {
    if (!byName.has(run.name)) byName.set(run.name, []);
    byName.get(run.name).push(run);
  }
  const rows = [...byName].slice(0, FIELD_TIMELINE_LANES);
  $('field-timeline').hidden = !runs.length;
  $('field-timeline').replaceChildren(...rows.map(([name, items]) => {
    const row = node('div', 'timeline-row');
    const label = node('span', 'timeline-name', name);
    label.title = name;
    const lane = node('div', 'timeline-lane');
    for (const run of items) {
      const bar = node('button', `command-bar ${run.outcome.toLowerCase()}`);
      bar.style.left = `${run.startSec / length * 100}%`;
      bar.style.width = `${Math.max(0.4, ((run.endSec ?? length) - run.startSec) / length * 100)}%`;
      bar.title = `${run.name} · ${fmt(run.startSec, 3)}–${fmt(run.endSec, 3)} s · ${run.outcome === 'SAMPLED' ? 'from sampled active set' : run.outcome}`;
      bar.setAttribute('aria-label', bar.title);
      bar.onclick = () => seek(run.startSec);
      lane.append(bar);
    }
    const cursor = node('span', 'command-cursor');
    state.commandCursors.push(cursor);
    lane.append(cursor);
    row.append(label, lane);
    return row;
  }));
  if (byName.size > rows.length) $('field-timeline').append(node('div', 'timeline-note', `${byName.size - rows.length} more commands in the Commands view`));
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
  const flagged = new Set(problemsAt(state.problems, state.time));
  for (const [item, element] of state.problemNodes || []) element.classList.toggle('current', flagged.has(item));
  if (state.compare) {
    const pose = sampleAt(state.compare.series.get('pose'), state.time);
    const before = state.time < (state.compare.series.get('pose')?.[0]?.[0] ?? Infinity);
    $('compare-pose').hidden = false;
    $('compare-pose').textContent = before || !Array.isArray(pose) ? 'B has not started at this time'
      : `B  X ${fmt(pose[0])} in · Y ${fmt(pose[1])} in · θ ${fmt(pose[2], 3)} rad · B time ${fmt(state.time - state.compare.offset, 2)} s`;
  }
  for (const pad of state.pads) {
    const axes = sampleAt(playback[`gamepad${pad.number}/axes`], state.time);
    const mask = sampleAt(playback[`gamepad${pad.number}/buttons`], state.time);
    pad.dots.forEach((dot, i) => {
      dot.style.left = `${50 + (finite(axes?.[i * 2]) ? axes[i * 2] : 0) * 38}%`;
      dot.style.top = `${50 - (finite(axes?.[i * 2 + 1]) ? axes[i * 2 + 1] : 0) * 38}%`;
      dot.style.opacity = axes ? '1' : '0';
    });
    const buttons = finite(mask) ? buttonNames.filter((_, i) => mask & (1 << i)).join(' ') || 'None' : 'Not recorded';
    pad.data.textContent = axes ? `L2 ${fmt(axes[4])}  R2 ${fmt(axes[5])}\nButtons: ${buttons}` : 'No input sample at this time';
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
$('event-filter').onchange = () => { renderEvents(); updateDetails(); };
$('field-commands-toggle').onchange = () => { saveLayout(); if (state.run) updateDetails(); };
$('field-ball-toggle').onchange = () => { saveLayout(); if (state.run) drawField(); };
$('problems-toggle').onclick = () => setProblems(!state.problemsOn);
$('problem-prev').onclick = () => stepProblem(-1);
$('problem-next').onclick = () => stepProblem(1);
$('compare-run').onchange = event => openCompare(event.target.value);
$('compare-clear').onclick = () => clearCompare();
const realign = () => { if (!state.compare) return; alignCompare(); loadCharts(); seek(state.time); };
$('compare-align').onchange = realign;
$('compare-shift').oninput = realign;
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
  if (event.code === 'BracketLeft' || event.code === 'BracketRight') { event.preventDefault(); stepProblem(event.code === 'BracketRight' ? 1 : -1); }
  if (event.code === 'ArrowLeft' || event.code === 'ArrowRight') {
    event.preventDefault(); stop(); seek(state.time + (event.code === 'ArrowRight' ? 1 : -1) * (event.shiftKey ? 1 : 0.02));
  }
});
function applyTheme(theme) {
  document.documentElement.dataset.theme = theme;
  refreshColors();
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
// Closing the Problems view turns detection off, rather than leaving its marks with no list.
document.querySelector('.problems-card .hide-view').addEventListener('click', () => setProblems(false));
const redrawOnResize = new ResizeObserver(() => { if (state.run && !$('workspace').hidden) { preparePlots(); drawField(); drawCamera(); drawCharts(); } });
redrawOnResize.observe($('workspace'));
redrawOnResize.observe(document.querySelector('.field-wrap'));
await refresh();
