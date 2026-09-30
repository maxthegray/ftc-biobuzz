import {finite} from './core.mjs';

export const $ = id => document.getElementById(id);
export const state = {logs: [], run: null, time: 0, playing: false, generation: 0, charts: [], options: [],
  series: new Map(), window: [0, 1], events: [], eventNodes: [], commandCursors: [], graphGeneration: 0, frame: null,
  cameras: [], cameraViews: [], dragKey: null, activeTab: 'field', pickerChart: null, layout: null, focusedChart: null, panels: {field: [['field'], ['gamepads']], signals: [['signals']], events: [['commands'], ['events']]}};
export const fmt = (n, digits = 2) => finite(n) ? n.toFixed(digits) : '—';
export const node = (tag, className, text) => {
  const element = document.createElement(tag);
  if (className) element.className = className;
  if (text !== undefined) element.textContent = text;
  return element;
};

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
    'chart-grid', 'chart-label', 'chart-cursor', 'muted', 'lane', 'lane-off', 'lane-edge', 'lane-ink', 'layer-0'];
  return {...Object.fromEntries(names.map(name => [name, read(name)])),
    lanes: Array.from({length: 6}, (_, i) => read(`lane-${i}`)), layers: Array.from({length: 8}, (_, i) => read(`layer-${i}`))};
};

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

