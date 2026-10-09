import {alignmentAnchor, shiftSeries, sampleAt, minutes} from './core.mjs';
import {$, state, fmt, fmtTime, node, api, notice, runLabel, runDate} from './ui.js';

// Up to four other runs drawn over the open one, lettered B–E. On the field each run has its own
// color; on graphs the color stays the signal's and each run has its own dash pattern.
export const MAX_OVERLAYS = 4;
export const OVERLAY_DASHES = [[7, 4], [2, 3], [9, 3, 2, 3], [14, 5]];
const LETTERS = 'BCDE';

// Ready overlays the user hasn't hidden.
export const shownOverlays = () => state.overlays.filter(overlay => overlay.run && overlay.visible);

export function createOverlays({onChange, downsample}) {
  async function add(id) {
    if (!state.run || id === state.run.id || state.overlays.some(overlay => overlay.id === id) || state.overlays.length >= MAX_OVERLAYS) return;
    const used = new Set(state.overlays.map(overlay => overlay.slot));
    const slot = [0, 1, 2, 3].find(i => !used.has(i));
    const overlay = {id, slot, letter: LETTERS[slot], visible: true, shift: 0, run: null, raw: new Map(), series: new Map(), offset: 0, trail: [],
      log: state.logs.find(log => log.id === id)};
    state.overlays.push(overlay);
    render();
    try {
      overlay.run = await api(`/api/log?id=${encodeURIComponent(id)}`);
      if (!state.overlays.includes(overlay)) return;
      overlay.raw = new Map(Object.entries(overlay.run.playback));
      align(overlay);
      onChange();
    } catch (error) {
      notice(error.message, true);
      remove(id);
    }
  }

  function remove(id) {
    state.overlays = state.overlays.filter(overlay => overlay.id !== id);
    onChange();
  }

  function toggle(id) {
    if (state.overlays.some(overlay => overlay.id === id)) remove(id); else add(id);
  }

  function clear() {
    state.overlays = [];
    onChange();
  }

  // Each overlay's samples are shifted onto the open run's clock once per alignment, so every view
  // reads them with the same lookups as the open run's own.
  function align(overlay) {
    const mode = $('overlay-align').value;
    const anchor = (events, poses) => alignmentAnchor(events, poses, mode);
    overlay.offset = anchor(state.events, state.run.playback.pose) - anchor(overlay.run.report.events, overlay.run.playback.pose) + overlay.shift;
    overlay.series = new Map([...overlay.raw].map(([name, series]) => [name, shiftSeries(series, overlay.offset)]));
    overlay.trail = downsample(overlay.series.get('pose') || []);
  }

  // After the open run changes: it can't overlay itself, and every other overlay lines up again.
  function reopen() {
    state.overlays = state.overlays.filter(overlay => overlay.id !== state.run.id);
    for (const overlay of state.overlays) if (overlay.run) align(overlay);
  }

  function realign() {
    for (const overlay of state.overlays) if (overlay.run) align(overlay);
    onChange();
  }

  function render() {
    renderBar();
    if ($('overlay-picker').open) renderPicker();
    $('overlay-open').textContent = state.overlays.length ? `Overlays · ${state.overlays.length}` : 'Overlay runs';
  }

  // One chip per overlay under the playback bar: its color and dash, name, length, its own time
  // shift, show/hide and remove. The alignment applies to all of them.
  function renderBar() {
    $('overlay-bar').hidden = !state.overlays.length;
    $('overlay-chips').replaceChildren(...[...state.overlays].sort((a, b) => a.slot - b.slot).map(overlay => {
      const chip = node('div', `overlay-chip${overlay.visible ? '' : ' hidden-overlay'}`);
      chip.style.setProperty('--run', `var(--overlay-${overlay.slot})`);
      const swatch = node('span', 'overlay-swatch', overlay.letter);
      const dash = node('span', `overlay-dash dash-${overlay.slot}`);
      dash.title = 'Line style of this run on graphs';
      const log = overlay.log || overlay.run;
      const name = node('span', 'overlay-name', overlay.run ? `${runLabel(overlay.run.name)} · ${runDate(overlay.run)}` : log ? `${runLabel(log.name)} · loading…` : 'Loading…');
      name.title = overlay.run?.name ?? '';
      const length = node('span', 'overlay-length', log?.drivenSec ? minutes(log.drivenSec) : '');
      const shift = node('input', 'overlay-shift');
      shift.type = 'number'; shift.step = '0.1'; shift.value = String(overlay.shift);
      shift.title = `Shift ${overlay.letter} later, in seconds`;
      shift.setAttribute('aria-label', `Shift run ${overlay.letter} later, in seconds`);
      shift.oninput = () => { overlay.shift = Number(shift.value) || 0; if (overlay.run) { align(overlay); onChange(); } };
      const eye = node('button', 'overlay-eye', overlay.visible ? 'Hide' : 'Show');
      eye.setAttribute('aria-pressed', String(!overlay.visible));
      eye.onclick = () => { overlay.visible = !overlay.visible; onChange(false); };
      const close = node('button', 'overlay-remove', '×');
      close.title = `Remove run ${overlay.letter}`;
      close.setAttribute('aria-label', `Remove run ${overlay.letter}`);
      close.onclick = () => remove(overlay.id);
      chip.append(swatch, dash, name, length, node('span', 'overlay-shift-label', 'shift'), shift, eye, close);
      return chip;
    }));
  }

  // Runs of the open run's op-mode first: the runs worth lining up against each other.
  function renderPicker() {
    const query = $('overlay-search').value.trim().toLowerCase();
    const current = state.run ? runLabel(state.run.name) : '';
    const candidates = state.logs.filter(log => log.id !== state.run?.id && (!log.short || state.overlays.some(overlay => overlay.id === log.id))
      && (!query || log.name.toLowerCase().includes(query) || runLabel(log.name).toLowerCase().includes(query)));
    const same = candidates.filter(log => runLabel(log.name) === current), others = candidates.filter(log => runLabel(log.name) !== current);
    const full = state.overlays.length >= MAX_OVERLAYS;
    const row = log => {
      const overlay = state.overlays.find(item => item.id === log.id);
      const button = node('button', 'overlay-option');
      button.setAttribute('aria-pressed', String(Boolean(overlay)));
      button.disabled = !overlay && full;
      if (overlay) button.style.setProperty('--run', `var(--overlay-${overlay.slot})`);
      button.append(node('span', 'overlay-swatch', overlay?.letter ?? ''), node('span', 'overlay-option-name', runLabel(log.name)),
        node('span', 'overlay-option-date', runDate(log)), node('span', 'overlay-option-length', log.drivenSec ? minutes(log.drivenSec) : ''));
      button.title = button.disabled ? `Up to ${MAX_OVERLAYS} overlays` : overlay ? `Remove run ${overlay.letter}` : 'Overlay this run';
      button.onclick = () => toggle(log.id);
      return button;
    };
    const groups = [];
    if (same.length) groups.push(node('div', 'overlay-group', `Same op-mode · ${current}`), ...same.map(row));
    if (others.length) groups.push(node('div', 'overlay-group', 'Other runs'), ...others.map(row));
    $('overlay-list').replaceChildren(...groups);
    if (!groups.length) $('overlay-list').append(node('p', 'empty', query ? 'No matching runs.' : 'No other runs to overlay.'));
    $('overlay-count').textContent = `${state.overlays.length} of ${MAX_OVERLAYS}`;
    $('overlay-clear').disabled = !state.overlays.length;
  }

  // Each shown overlay's pose at the cursor, one line per run in its color.
  function renderPoses() {
    const lines = shownOverlays().map(overlay => {
      const poses = overlay.series.get('pose') || [];
      const pose = sampleAt(poses, state.time);
      const line = node('div', 'overlay-pose');
      line.style.setProperty('--run', `var(--overlay-${overlay.slot})`);
      line.append(node('span', 'overlay-swatch', overlay.letter));
      line.append(node('span', '', state.time < (poses[0]?.[0] ?? Infinity) || !Array.isArray(pose) ? 'not started yet'
        : `X ${fmt(pose[0])} · Y ${fmt(pose[1])} in · ${fmt(pose[2] * 180 / Math.PI, 1)}° · at ${fmtTime(state.time - overlay.offset, 2, overlay.run.match?.startSec)}`));
      return line;
    });
    $('overlay-poses').hidden = !lines.length;
    $('overlay-poses').replaceChildren(...lines);
  }

  $('overlay-open').onclick = () => { if (!state.run) return; $('overlay-search').value = ''; renderPicker(); $('overlay-picker').showModal(); $('overlay-search').focus(); };
  $('overlay-close').onclick = () => $('overlay-picker').close();
  $('overlay-search').oninput = renderPicker;
  $('overlay-clear').onclick = clear;
  $('overlay-clear-all').onclick = clear;
  $('overlay-align').onchange = realign;

  return {add, remove, toggle, clear, reopen, render, renderPoses};
}
