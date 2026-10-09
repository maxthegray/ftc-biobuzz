import {finite, indexAt, sampleAt, activeCommandSpans, fieldView, ballSources, cameraSources, channelOptions, channelTree, discreteOptions, buttonNames, alignmentAnchor, shiftSeries, adjacentProblem, problemsAt, matchPhases, phaseAt, minutes, posePasses, nextPass} from './core.mjs';
import {$, state, fmt, fmtTime, node, api, notice, commandCoverage, refreshColors, runLabel, runDay, runClock, runDate} from './ui.js';
import {createOverlays, shownOverlays, MAX_OVERLAYS} from './overlays.js';
import {createCharts} from './charts.js';
import {createLayout} from './layout.js';
import {drawField, drawCamera, fillCameraSources, fieldInches} from './field-camera.js';

// Whole words, so "schedule default Drive" isn't a FAULT. Matches the server's problem rule.
const faultPattern = /\b(?:FAULT|CRASH|FAIL(?:ED|URE)?|LOST|INCOMPLETE|DISABLED|OVERRUN)\b/i;
const themeKey = 'maxscope.theme';
const {closePicker, renderCharts, loadCharts, applyPreset, preparePlots, drawCharts, syncWindow, follow, updateWindowControls} = createCharts({seek, saveLayout: () => saveLayout(), onWindow: () => drawField()});
const {initializeLayout, saveLayout, setProblemsView} = createLayout({seek, preparePlots, redraw});
// Everything that shows overlays redraws when one is added, removed, hidden, shifted or realigned;
// `reload` also fetches their samples for the graphs.
const overlays = createOverlays({downsample: poses => downsample(poses), onChange: (reload = true) => {
  overlays.render();
  if (!state.run) return;
  updateFieldView();
  renderMetrics();
  renderLibrary();
  if (reload) loadCharts(); else { preparePlots(); drawCharts(); }
  seek(state.time);
}});

function redraw() { drawField(); drawCamera(); drawCharts(); }

// Runs that never left init, or stopped within seconds of start, are hidden unless asked for or open.
const listed = log => state.showShort || !log.short || log.id === state.run?.id;

function renderLibrary() {
  const search = $('search').value.toLowerCase();
  const matching = state.logs.filter(log => log.name.toLowerCase().includes(search));
  const runs = matching.filter(listed);
  const hidden = matching.filter(log => log.short).length;
  $('short-toggle').hidden = !hidden;
  $('short-toggle').textContent = state.showShort ? 'Hide short runs' : `${hidden} short run${hidden === 1 ? '' : 's'} hidden · Show`;
  $('runs').replaceChildren();
  let day = null;
  for (const log of runs) {
    const date = runDay(log);
    if (date !== day) { day = date; $('runs').append(node('div', 'run-day', date)); }
    const row = node('div', 'run-row');
    const button = node('button', 'run');
    button.classList.toggle('selected', log.id === state.run?.id);
    button.classList.toggle('short', log.short);
    button.title = log.name;
    const top = node('span', 'run-top');
    top.append(node('strong', '', runLabel(log.name)));
    if (state.newIds.has(log.id)) top.append(node('span', 'run-new', 'NEW'));
    top.append(node('span', 'run-length', !log.scanned ? '…' : log.drivenSec !== null ? minutes(log.drivenSec) : 'init only'));
    button.append(top, node('small', '', `${runClock(log)} · ${(log.bytes / 1048576).toFixed(1)} MiB`));
    button.onclick = () => openRun(log.id);
    row.append(button);
    if (log.id !== state.run?.id && state.run) {
      const overlay = state.overlays.find(item => item.id === log.id);
      const toggle = node('button', `run-overlay${overlay ? ' on' : ''}`, overlay?.letter ?? '+');
      if (overlay) toggle.style.setProperty('--run', `var(--overlay-${overlay.slot})`);
      toggle.title = overlay ? `Remove overlay ${overlay.letter}` : 'Overlay this run';
      toggle.setAttribute('aria-label', `${toggle.title}: ${runLabel(log.name)} ${runDate(log)}`);
      toggle.disabled = !overlay && state.overlays.length >= MAX_OVERLAYS;
      toggle.onclick = () => overlays.toggle(log.id);
      row.append(toggle);
    }
    $('runs').append(row);
  }
  if (!runs.length) $('runs').append(node('p', 'empty', state.logs.length ? 'No matching runs.' : 'No local logs yet. Open a WPILOG or run make pull-logs.'));
}

// The newest pulled run worth opening: scanned, and not a short one. Null while it is still scanning.
function newestRun() {
  const newest = state.logs.find(log => !log.uploaded && (!log.scanned || !log.short));
  return newest?.scanned ? newest : null;
}

// Polled every few seconds, so make pull-logs shows up without a refresh. The newest run opens
// on first load, and again whenever one is pulled while you are looking at the newest (following
// is decided when a run opens, so a pull that is still being scanned doesn't break it).
async function refresh() {
  try {
    const logs = await api('/api/logs');
    const signature = list => list.map(log => `${log.id}:${log.scanned}`).join();
    const changed = signature(logs) !== signature(state.logs.filter(log => !log.uploaded));
    if (state.knownIds) for (const log of logs) if (!state.knownIds.has(log.id)) state.newIds.add(log.id);
    state.knownIds = new Set([...(state.knownIds || []), ...logs.map(log => log.id)]);
    state.logs = [...state.logs.filter(log => log.uploaded), ...logs];
    if (changed) renderLibrary();
    const newest = newestRun();
    if (newest && !state.openedOnce) openRun(newest.id);
    else if (newest && state.following && newest.id !== state.run?.id) { await openRun(newest.id); notice(`Opened the newest run, ${runLabel(newest.name)} · ${runDate(newest)}.`); }
    return logs.some(log => !log.scanned);
  } catch (error) { notice(error.message, true); return false; }
}

// Polling pauses while the tab is hidden and catches up as soon as it is shown.
async function poll(first = false) {
  clearTimeout(state.pollTimer);
  const scanning = first || !document.hidden ? await refresh() : false;
  state.pollTimer = setTimeout(poll, scanning ? 1000 : 5000);
}
document.addEventListener('visibilitychange', () => { if (!document.hidden) poll(); });

async function openRun(id) {
  const generation = ++state.generation;
  state.openedOnce = true;
  state.following = id === newestRun()?.id;
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
    state.follow = true;
    state.match = matchPhases(run.match);
    state.hoverTime = null;
    state.options = channelOptions(run.channels);
    state.channelTree = channelTree(state.options);
    state.discreteOptions = discreteOptions(run.channels);
    state.events = [...(run.report.events || []), ...run.commandEvents].sort((a, b) => a.tSec - b.tSec);
    state.problems = [];
    state.problemCounts = {};
    state.problemFilter = null;

    state.eventSeries = state.events.map(event => [event.tSec, event]);
    const poses = run.playback.pose || [];
    state.trail = downsample(poses);
    overlays.reopen();
    updateFieldView();
    state.cameras = cameraSources(run.channels);
    $('field-ball-option').hidden = !ballSources(state.cameras).some(source => run.playback[source.tx]?.length);
    for (const view of state.cameraViews) fillCameraSources(view);
    $('run-title').textContent = runLabel(run.name);
    $('run-file').textContent = run.name;
    $('run-status').textContent = run.report.truncated ? 'Partial log' : 'No readable samples';
    $('run-status').hidden = !run.report.truncated && !run.report.empty;
    $('run-status').classList.toggle('warning', run.report.truncated || run.report.empty);
    $('field-size').textContent = `Field · ${run.fieldLengthIn}" × ${run.fieldLengthIn}"`;
    $('scrub').max = run.endSec;
    $('duration').textContent = fmtTime(run.endSec);
    $('duration').title = `Log ends at ${fmt(run.endSec)} s`;
    $('play').disabled = run.endSec <= 0;
    $('workspace').hidden = false;
    $('overlay-open').hidden = false;
    overlays.render();
    renderPhaseMarks();
    renderMetrics();
    renderProblems();
    if (state.problemsOn) loadProblems();
    renderCommands();
    renderEvents();
    renderGamepads();
    renderCharts();
    renderLibrary();
    // Op-mode start, or the first pose after it: the start event lands a loop before any sample.
    seek(Math.max(state.match?.start ?? 0, poses[0]?.[0] ?? 0));
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
  const faults = (events = []) => events.filter(event => faultPattern.test(event.text)).length;
  const metrics = [
    ['RUN LENGTH', fmt(state.run.endSec, 1), 's', `${(report.recordCount || 0).toLocaleString()} records`, run => fmt(run.endSec, 1)],
    ['BATTERY MINIMUM', fmt(report.battery?.minV), 'V', `Start ${fmt(report.battery?.startV)} V`, run => fmt(run.report.battery?.minV)],
    ['LOOP P99', fmt(report.loop?.p99Ms), 'ms', `Peak ${fmt(report.loop?.maxMs)} ms`, run => fmt(run.report.loop?.p99Ms)],
    ['FAULT EVENTS', String(faults(report.events)), '', `${state.run.channels.length} recorded channels`, run => String(faults(run.report.events))],
  ];
  $('metrics').replaceChildren(...metrics.map(([label, value, unit, detail, of]) => {
    const item = node('div', 'metric');
    const number = node('strong', '', value);
    number.append(node('small', '', unit));
    item.append(node('div', 'label', label), number, node('div', 'detail', detail));
    const others = shownOverlays();
    if (others.length) item.append(node('div', 'metric-overlays'));
    for (const overlay of others) {
      const tag = node('span', 'metric-overlay');
      tag.style.setProperty('--run', `var(--overlay-${overlay.slot})`);
      tag.append(node('span', 'overlay-swatch', overlay.letter), node('span', '', `${of(overlay.run)}${unit ? ' ' + unit : ''}`));
      item.lastChild.append(tag);
    }
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
    button.append(node('time', '', fmtTime(item.startSec)), node('strong', '', item.kind), node('span', '', item.detail));
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
    marker.title = `${fmtTime(item.startSec)} · ${item.kind}${item.detail ? ' · ' + item.detail : ''}`;
    marker.onclick = () => { stop(); seek(item.startSec); };
    return marker;
  }));
}

// Match boundaries and BIOBUZZ milestones under the slider.
function renderPhaseMarks() {
  const length = Math.max(state.run.endSec, 0.001);
  $('phase-marks').replaceChildren(...(state.match?.marks || []).filter(mark => mark.tSec <= state.run.endSec).map(mark => {
    const label = node('span', 'phase-mark', mark.label);
    label.style.setProperty('--at', String(mark.tSec / length));
    return label;
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
  const poses = [...(state.run.playback.pose || []), ...state.overlays.flatMap(overlay => overlay.run?.playback.pose || [])];
  state.fieldView = fieldView(poses, state.run.fieldLengthIn);
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
  // Legacy logs have no executions, so their bars are spans of the sampled active set.
  const playback = state.run.playback;
  const traced = state.run.report.commandExecutions || [];
  const runs = traced.length ? traced
    : activeCommandSpans(playback['commands/active']?.length ? playback['commands/active'] : playback['commands/running'], state.run.endSec);
  for (const run of runs) {
    const row = node('div', 'command');
    const label = node('div', 'command-label');
    label.append(node('span', '', run.id === undefined ? run.name : `#${run.id} ${run.name}`), node('span', '', run.outcome === 'SAMPLED' ? 'sampled' : run.outcome));
    const track = node('div', 'command-track');
    const bar = node('button', `command-bar ${run.outcome.toLowerCase()}`);
    const length = Math.max(state.run.endSec, 0.001);
    bar.style.left = `${run.startSec / length * 100}%`;
    bar.style.width = `${Math.max(0, ((run.endSec ?? length) - run.startSec) / length * 100)}%`;
    bar.title = `${run.name} · ${fmtTime(run.startSec, 3)}–${fmtTime(run.endSec, 3)} · ${run.outcome}${run.detail ? '\n' + run.detail : ''}`;
    bar.setAttribute('aria-label', bar.title);
    bar.onclick = () => seek(run.startSec);
    const cursor = node('span', 'command-cursor');
    state.commandCursors.push(cursor);
    track.append(bar, cursor);
    row.append(label, track);
    $('commands').append(row);
  }
  if (!runs.length) $('commands').append(node('p', 'empty', 'No command history in this run.'));
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
    button.append(node('time', '', fmtTime(event.tSec, 3)), node('span', '', event.text));
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
  $('field-activity').title = `Command coverage: ${commandCoverage()}`;
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
  overlays.renderPoses();
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
  field3d.postMessage({type: 'time', runId: state.run.id, time: state.time});
  if (syncWindow()) updateWindowControls();
  $('time').textContent = fmtTime(state.time);
  $('time').title = `Log time ${fmt(state.time, 3)} s`;
  const phase = phaseAt(state.match, state.time);
  $('phase').hidden = !phase;
  if (phase) { $('phase').textContent = phase.clock ? `${phase.name} · ${phase.clock}` : phase.name; $('phase').dataset.phase = phase.name; }
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
$('field-elements-toggle').onchange = () => { saveLayout(); if (state.run) drawField(); };
// The experimental 3D page lives in its own tab: it opens on this run and time, then follows
// every seek through the 'maxscope' channel. Nothing else here knows it exists.
const field3d = new BroadcastChannel('maxscope');
$('field-3d').onclick = () => { if (state.run) $('field-3d').href = `/3d#run=${encodeURIComponent(state.run.id)}&t=${state.time}`; };
$('problems-toggle').onclick = () => setProblems(!state.problemsOn);
$('problem-prev').onclick = () => stepProblem(-1);
$('problem-next').onclick = () => stepProblem(1);
$('refresh').onclick = refresh;
$('search').oninput = renderLibrary;
$('field-zoom-reset').onclick = () => follow();
$('short-toggle').onclick = () => { state.showShort = !state.showShort; renderLibrary(); };
$('field').addEventListener('click', event => {
  if (!state.run) return;
  const point = fieldInches(event);
  if (!point) return;
  const passes = posePasses(state.run.playback.pose || [], point.at, point.radiusIn);
  const target = nextPass(passes, state.time);
  clearTimeout(state.fieldHintTimer);
  $('field-hint').hidden = false;
  $('field-hint').textContent = !target ? 'The robot never came here'
    : passes.length === 1 ? `Here once, at ${fmtTime(target.atSec)}` : `Here ${passes.length} times · pass ${passes.indexOf(target) + 1}, at ${fmtTime(target.atSec)} · click again for the next`;
  state.fieldHintTimer = setTimeout(() => { $('field-hint').hidden = true; }, 4000);
  if (target) { stop(); seek(target.atSec); }
});
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
  if (event.code === 'KeyF' && !event.metaKey && !event.ctrlKey) { event.preventDefault(); follow(); }
  if (event.code === 'BracketLeft' || event.code === 'BracketRight') { event.preventDefault(); stepProblem(event.code === 'BracketRight' ? 1 : -1); }
  if (event.code === 'ArrowLeft' || event.code === 'ArrowRight') {
    event.preventDefault(); stop(); seek(state.time + (event.code === 'ArrowRight' ? 1 : -1) * (event.shiftKey ? 1 : 0.02));
  }
});
function applyTheme(theme) {
  document.documentElement.dataset.theme = theme;
  refreshColors();
  const next = theme === 'dark' ? 'light' : 'dark';
  $('theme-toggle').textContent = theme === 'dark' ? 'Light' : 'Dark';
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
await poll(true);
