import {finite} from './core.mjs';

export const $ = id => document.getElementById(id);
export const state = {logs: [], run: null, time: 0, playing: false, generation: 0, charts: [], options: [],
  series: new Map(), window: [0, 1], events: [], eventNodes: [], commandCursors: [], graphGeneration: 0, frame: null,
  match: null, follow: true, windowLength: 10, hoverTime: null, hoverChart: null, showShort: false, knownIds: null, newIds: new Set(), openedOnce: false, problems: [], problemCounts: {}, problemsOn: false, problemGeneration: 0, problemFilter: null, problemNodes: [], overlays: [], cameras: [], cameraViews: [], dragKey: null, activeTab: 'field', pickerChart: null, layout: null, focusedChart: null, panels: {field: [['field'], ['gamepads']], signals: [['signals']], events: [['commands'], ['events']]}};
export const fmt = (n, digits = 2) => finite(n) ? n.toFixed(digits) : '—';
export const node = (tag, className, text) => {
  const element = document.createElement(tag);
  if (className) element.className = className;
  if (text !== undefined) element.textContent = text;
  return element;
};

// Times shown to people count from the op-mode start, like the driver station; init is negative.
// `start` is another run's start when labelling an overlay run's own clock.
export function fmtTime(time, digits = 2, start = state.run?.match?.startSec ?? 0) {
  const value = time - (start ?? 0);
  return finite(value) ? `${value < -0.5 * 10 ** -digits ? '−' : ''}${Math.abs(value).toFixed(digits)} s` : '—';
}

// Run names and dates from the log's filename: "BallFollowTeleOp-20280309-124145-…" reads as
// "Ball Follow Tele Op" on Mar 9 at 12:41:45.
export function runLabel(name) {
  return name.replace(/-\d{8}-\d{6}.*\.wpilog$/, '').replace(/\.wpilog$/, '').replace(/([a-z])([A-Z])/g, '$1 $2');
}

const stamp = log => log.name.match(/-(\d{4})(\d{2})(\d{2})-(\d{2})(\d{2})(\d{2})/);
const MONTHS = ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun', 'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec'];
// Dates come from the hub's clock, which may be wrong, so no weekday is guessed from them.
export function runDay(log) {
  const match = stamp(log);
  return match ? `${MONTHS[Number(match[2]) - 1] ?? match[2]} ${Number(match[3])}` : log.uploaded ? 'Imported' : 'Other';
}
export function runClock(log) {
  const match = stamp(log);
  return match ? `${match[4]}:${match[5]}:${match[6]}` : log.uploaded ? 'Imported' : 'Local file';
}
export function runDate(log) {
  return stamp(log) ? `${runDay(log)} · ${runClock(log)}` : runClock(log);
}

export function notice(message = '', error = false) {
  $('notice').textContent = message;
  $('notice').hidden = !message;
  $('notice').classList.toggle('error', error);
}

export async function api(path, options) {
  const response = await fetch(path, options);
  const result = await response.json();
  if (!response.ok) throw new Error(result.error || 'Could not read this log');
  return result;
}

export function commandCoverage() {
  const report = state.run.report;
  return report.commandHistoryCoverage === 'instrumented'
    ? `Instrumented only${report.commandHistoryLostRecords ? ' · Records lost' : ''}`
    : report.commandHistoryCoverage === 'all scheduled' ? 'Legacy samples' : 'Not recorded';
}

export let colors = null;
const cssColors = () => {
  const style = getComputedStyle(document.documentElement), read = name => style.getPropertyValue(`--${name}`).trim();
  const names = ['field', 'off-field', 'hatch', 'grid', 'field-label', 'wall', 'field-caption', 'trail-future', 'trail-past', 'lime', 'robot-line', 'robot-arrow', 'ball',
    'chart-grid', 'chart-label', 'chart-cursor', 'muted', 'lane', 'lane-off', 'lane-edge', 'lane-ink', 'layer-0', 'red', 'amber', 'trail-window', 'tooltip'];
  return {...Object.fromEntries(names.map(name => [name, read(name)])),
    lanes: Array.from({length: 6}, (_, i) => read(`lane-${i}`)), layers: Array.from({length: 8}, (_, i) => read(`layer-${i}`)),
    overlays: Array.from({length: 4}, (_, i) => read(`overlay-${i}`))};
};

export const problemColor = problem => colors[problem.severity === 'fault' ? 'red' : 'amber'];

export function refreshColors() { colors = cssColors(); }

export function context(canvas) {
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

