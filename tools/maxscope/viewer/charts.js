import {finite, indexAt, sampleAt, scalarSeries, plotPoints, plotBounds, channelTree, discreteSeries, discreteIntervals, buttonNames, shiftSeries, followWindow} from './core.mjs';
import {$, state, fmt, fmtTime, node, api, notice, commandCoverage, context, colors, problemColor} from './ui.js';
import {OVERLAY_DASHES, shownOverlays} from './overlays.js';

export function createCharts({seek, saveLayout, onWindow}) {
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
    wireTimeCanvas(canvas, chart);
    return chart;
  }

  const timeAt = (canvas, clientX) => {
    const rect = canvas.getBoundingClientRect();
    const fraction = Math.max(0, Math.min(1, (clientX - rect.left - 45) / Math.max(1, rect.width - 90)));
    return state.window[0] + fraction * (state.window[1] - state.window[0]);
  };

  // Graphs follow playback through a fixed-length window, so a 20 s run and a 6 min run read the
  // same. Panning detaches them where you leave them; Follow (or F) puts them back on the cursor.
  const WINDOW_KEY = 'maxscope.graphWindow';
  const ZOOM_PER_PIXEL = 0.0015;
  try {
    const saved = localStorage.getItem(WINDOW_KEY);
    if (saved === 'all') state.windowLength = Infinity;
    else if (Number(saved) > 0) state.windowLength = Number(saved);
  } catch { /* Storage may be disabled. */ }

  function saveWindowLength() {
    try { localStorage.setItem(WINDOW_KEY, state.windowLength === Infinity ? 'all' : String(state.windowLength)); } catch { /* Storage may be disabled. */ }
  }

  // Moves a following window to the cursor; true when it moved. Called on every seek.
  function syncWindow() {
    if (!state.run || !state.follow) return false;
    const [start, end] = followWindow(state.time, state.windowLength, state.run.endSec);
    if (start === state.window[0] && end === state.window[1]) return false;
    state.window = [start, end];
    preparePlots();
    return true;
  }

  function follow() {
    state.follow = true;
    syncWindow(); updateWindowControls(); drawCharts(); onWindow();
  }

  function setWindowLength(length) {
    state.windowLength = length;
    saveWindowLength();
    follow();
  }

  // A window placed by hand: panning, or zooming while detached.
  function setWindow(start, length) {
    const begin = Math.max(0, Math.min(state.run.endSec - length, start));
    state.follow = false;
    state.window = [begin, begin + length];
    state.windowLength = length;
    saveWindowLength();
    preparePlots(); updateWindowControls(); drawCharts(); onWindow();
  }

  function updateWindowControls() {
    const full = !state.run || state.window[1] - state.window[0] >= state.run.endSec - 1e-6;
    for (const button of document.querySelectorAll('[data-window]')) {
      const length = button.dataset.window === 'all' ? Infinity : Number(button.dataset.window);
      button.setAttribute('aria-pressed', String(state.follow && length === state.windowLength));
    }
    $('follow').hidden = state.follow || full;
  }

  // Click seeks; dragging pans a zoomed graph. Hovering shows a crosshair on every graph and the
  // values under the pointer on this one.
  function wireTimeCanvas(canvas, chart = null) {
    canvas.onpointerdown = event => {
      if (event.button !== 0 || !state.run) return;
      const startX = event.clientX, [start, end] = state.window;
      const perPixel = (end - start) / Math.max(1, canvas.getBoundingClientRect().width - 90);
      let panning = false;
      canvas.setPointerCapture(event.pointerId);
      const move = moved => {
        const dx = moved.clientX - startX;
        if (!panning && Math.abs(dx) < 4) return;
        panning = true; canvas.classList.add('panning');
        state.hoverTime = null;
        setWindow(start - dx * perPixel, end - start);
      };
      const finish = released => {
        canvas.removeEventListener('pointermove', move);
        canvas.classList.remove('panning');
        if (!panning && released.type === 'pointerup') seek(timeAt(canvas, released.clientX));
      };
      canvas.addEventListener('pointermove', move);
      canvas.addEventListener('pointerup', finish, {once: true});
      canvas.addEventListener('pointercancel', finish, {once: true});
    };
    canvas.addEventListener('pointermove', event => {
      if (!state.run || canvas.classList.contains('panning')) return;
      state.hoverTime = timeAt(canvas, event.clientX);
      state.hoverChart = chart;
      state.hoverX = event.clientX - canvas.getBoundingClientRect().left;
      drawCharts();
    });
    canvas.addEventListener('pointerleave', () => { state.hoverTime = null; state.hoverChart = null; drawCharts(); });
    canvas.addEventListener('wheel', event => {
      event.preventDefault();
      const [start, end] = state.window;
      const rect = canvas.getBoundingClientRect();
      const fraction = Math.max(0, Math.min(1, (event.clientX - rect.left - 45) / Math.max(1, rect.width - 90)));
      const center = start + fraction * (end - start);
      // Proportional to how far the wheel or trackpad moved, so a trackpad's many small events don't
      // fly through zoom levels: one mouse notch (100 px) is about 16%. Line-mode deltas are ~16 px.
      const pixels = event.deltaY * (event.deltaMode === 1 ? 16 : event.deltaMode === 2 ? 400 : 1);
      const factor = Math.exp(Math.max(-1, Math.min(1, pixels * ZOOM_PER_PIXEL)));
      const length = Math.min(Math.max(state.run.endSec, 0.001), Math.max(0.02, (end - start) * factor));
      // Following, zoom changes the window length and keeps following; detached, it zooms around the pointer.
      if (state.follow) { state.windowLength = length; saveWindowLength(); follow(); }
      else setWindow(center - fraction * length, length);
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

  function discreteLabel(value, option) {
    if (value === null) return 'Not recorded';
    if (typeof value === 'boolean') return value ? 'On' : 'Off';
    if (/^gamepad[12]\/buttons$/.test(option?.source || '') && option.bit === undefined) {
      return buttonNames.filter((_, bit) => value & (1 << bit)).join(' + ') || 'None';
    }
    return String(value).replaceAll('\n', ' · ') || 'None';
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
      lane.overlayValues = node('span', 'chart-overlay-values discrete-overlay-values');
      const remove = node('button', 'layer-remove', '×');
      remove.setAttribute('aria-label', `Remove discrete ${label} from graph ${state.charts.indexOf(chart) + 1}`);
      remove.onclick = () => { chart.discreteKeys = chart.discreteKeys.filter(key => key !== lane.key); loadCharts(); };
      header.append(lane.value, lane.overlayValues, remove);
      lane.canvas = node('canvas');
      lane.canvas.setAttribute('aria-label', `${label} discrete timeline`);
      wireTimeCanvas(lane.canvas);
      lane.canvas.onmousemove = event => {
        const rect = lane.canvas.getBoundingClientRect();
        const time = state.window[0] + Math.max(0, Math.min(1, (event.clientX - rect.left - 45) / Math.max(1, rect.width - 90))) * (state.window[1] - state.window[0]);
        const others = shownStrips(lane).map(strip => ` · ${strip.overlay.letter}: ${discreteLabel(sampleAt(strip.data, time), lane.option)}`).join('');
        lane.canvas.title = `${fmtTime(time, 3)} · ${state.overlays.length ? 'A: ' : ''}${discreteLabel(sampleAt(lane.data, time), lane.option)}${others}`;
      };
      row.append(header, lane.canvas); chart.lanes.append(row);
    }
  }

  // Overlay strips are drawn under the open run's strip, one per shown overlay.
  const LANE_HEIGHT = 42, STRIP_HEIGHT = 14, STRIP_GAP = 4;
  const shownStrips = lane => (lane.strips || []).filter(strip => strip.overlay.visible);

  // A run of state blocks between `top` and `top + height`: the same value gets the same color in
  // every strip, off/false/empty is dimmed, and blocks wide enough carry their label.
  function drawStrip(ctx, x, intervals, option, top, height, labelled) {
    const [start, end] = state.window, palette = colors.lanes;
    for (let i = Math.max(0, indexAt(intervals, start)); i < intervals.length; i++) {
      const [time, interval] = intervals[i];
      if (time > end) break;
      if (interval.end <= start) continue;
      const a = x(Math.max(start, time)), b = x(Math.min(end, interval.end));
      const label = discreteLabel(interval.value, option);
      let hash = 0;
      for (const char of label) hash = (hash * 31 + char.charCodeAt(0)) >>> 0;
      ctx.fillStyle = interval.value === false || interval.value === '' || interval.value === 0 ? colors['lane-off'] : palette[hash % palette.length];
      ctx.fillRect(a, top, Math.max(1, b - a), height);
      ctx.strokeStyle = colors['lane-edge']; ctx.beginPath(); ctx.moveTo(a, top); ctx.lineTo(a, top + height); ctx.stroke();
      if (labelled && b - a > 24) {
        ctx.save(); ctx.beginPath(); ctx.rect(a + 5, top, b - a - 10, height); ctx.clip();
        ctx.fillStyle = colors['lane-ink']; ctx.fillText(label, a + 7, top + height / 2); ctx.restore();
      }
    }
  }

  function drawDiscrete(chart) {
    for (const lane of chart.discrete) {
      const strips = shownStrips(lane);
      const canvasHeight = `${LANE_HEIGHT + strips.length * (STRIP_HEIGHT + STRIP_GAP)}px`;
      if (lane.canvas.style.height !== canvasHeight) lane.canvas.style.height = canvasHeight;
      const {ctx, width, height} = context(lane.canvas);
      const [start, end] = state.window, left = 45, right = width - 45;
      const x = time => left + (time - start) / (end - start) * (right - left);
      lane.value.textContent = chart.loading ? 'Loading…' : discreteLabel(sampleAt(lane.data, state.time), lane.option);
      lane.value.title = lane.value.textContent;
      lane.overlayValues.replaceChildren(...(chart.loading ? [] : strips.map(strip => {
        const value = node('span', 'chart-overlay-value', discreteLabel(sampleAt(strip.data, state.time), lane.option));
        value.style.setProperty('--run', `var(--overlay-${strip.overlay.slot})`);
        value.dataset.letter = strip.overlay.letter;
        value.title = `Run ${strip.overlay.letter}: ${value.textContent}`;
        return value;
      })));
      ctx.font = '10px ui-monospace, monospace'; ctx.textBaseline = 'middle';
      // Run letters in the left margin, once there is more than one run to tell apart.
      if (strips.length) {
        ctx.textAlign = 'right';
        ctx.fillStyle = colors.lime; ctx.fillText('A', left - 8, LANE_HEIGHT / 2);
        strips.forEach((strip, i) => {
          ctx.fillStyle = colors.overlays[strip.overlay.slot];
          ctx.fillText(strip.overlay.letter, left - 8, LANE_HEIGHT + i * (STRIP_HEIGHT + STRIP_GAP) + STRIP_HEIGHT / 2);
        });
      }
      ctx.fillStyle = colors.lane; ctx.fillRect(left, 5, right - left, LANE_HEIGHT - 10);
      strips.forEach((strip, i) => { ctx.fillStyle = colors.lane; ctx.fillRect(left, LANE_HEIGHT + i * (STRIP_HEIGHT + STRIP_GAP), right - left, STRIP_HEIGHT); });
      ctx.save(); ctx.beginPath(); ctx.rect(left, 0, right - left, height); ctx.clip();
      ctx.textAlign = 'left';
      drawStrip(ctx, x, lane.intervals, lane.option, 5, LANE_HEIGHT - 10, true);
      strips.forEach((strip, i) => {
        const top = LANE_HEIGHT + i * (STRIP_HEIGHT + STRIP_GAP);
        drawStrip(ctx, x, strip.intervals, lane.option, top, STRIP_HEIGHT, false);
        ctx.fillStyle = colors.overlays[strip.overlay.slot]; ctx.fillRect(left, top, 2, STRIP_HEIGHT);
      });
      if (state.time >= start && state.time <= end) {
        ctx.strokeStyle = colors['layer-0']; ctx.setLineDash([3, 3]); ctx.beginPath();
        ctx.moveTo(x(state.time), 0); ctx.lineTo(x(state.time), height); ctx.stroke(); ctx.setLineDash([]);
      }
      if (state.hoverTime !== null && state.hoverTime >= start && state.hoverTime <= end) {
        ctx.strokeStyle = colors.muted; ctx.lineWidth = 1; ctx.beginPath();
        ctx.moveTo(x(state.hoverTime), 0); ctx.lineTo(x(state.hoverTime), height); ctx.stroke();
      }
      if (!lane.data.length) {
        ctx.fillStyle = colors['chart-label']; ctx.fillText(chart.loading ? 'Loading…' : 'No recorded samples', left + 8, LANE_HEIGHT / 2);
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
      layer.overlayValues = node('span', 'chart-overlay-values');
      const axis = node('span', 'axis-label', layer.axis === 1 ? 'R' : 'L');
      axis.title = layer.axis === 1 ? 'Right axis' : 'Left axis';
      const remove = node('button', 'layer-remove', '×');
      remove.setAttribute('aria-label', `Remove ${layer.option?.label || layer.key}`);
      remove.onclick = () => { chart.keys = chart.keys.filter(key => key !== layer.key); loadCharts(); };
      item.append(label, axis, layer.value, layer.overlayValues, remove);
      chart.legend.append(item);
    });
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
        axis: Math.max(0, chart.axes.indexOf(layer.option?.unit)), data: [], points: [], overlays: []}));
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
      // Each overlay's samples for the same channels, shifted onto this run's clock.
      const overlays = state.overlays.filter(overlay => overlay.run);
      for (const overlay of overlays) {
        const missing = [...requested].filter(name => !overlay.raw.has(name));
        for (let offset = 0; offset < missing.length; offset += 8) {
          const query = new URLSearchParams({id: overlay.id});
          missing.slice(offset, offset + 8).forEach(name => query.append('channel', name));
          const result = await api(`/api/series?${query}`);
          if (runGeneration !== state.generation || generation !== state.graphGeneration || !state.overlays.includes(overlay)) return;
          for (const [name, series] of Object.entries(result)) { overlay.raw.set(name, series); overlay.series.set(name, shiftSeries(series, overlay.offset)); }
        }
      }
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
          layer.overlays = option ? overlays.map(overlay => ({overlay, data: scalarSeries(overlay.series.get(option.name) || [], option.component, option.scale), points: []})) : [];
        }
        for (const lane of chart.discrete) {
          lane.data = discreteSeries(state.series.get(lane.option?.source) || [], lane.option?.bit);
          lane.intervals = discreteIntervals(lane.data, state.run.endSec);
          // Each overlay's own samples of the same field, on this run's clock and ending where its run
          // ends. Commands come from whichever command channel that run recorded.
          lane.strips = lane.option ? overlays.map(overlay => {
            const source = lane.key === '@commands'
              ? ['commands/active', 'commands/running'].find(name => overlay.run.channels.some(channel => channel.name === name))
              : lane.option.source;
            const data = discreteSeries(overlay.series.get(source) || [], lane.option.bit);
            return {overlay, data, intervals: discreteIntervals(data, overlay.run.endSec + overlay.offset)};
          }) : [];
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
      const buckets = Math.max(100, Math.floor(chart.canvas.clientWidth));
      for (const layer of chart.layers) {
        layer.points = plotPoints(layer.data, ...state.window, buckets);
        for (const trace of layer.overlays) trace.points = trace.overlay.visible ? plotPoints(trace.data, ...state.window, buckets) : [];
      }
      const scaled = [...chart.layers, ...chart.layers.flatMap(layer => layer.overlays.map(trace => ({...layer, points: trace.points})))];
      chart.bounds = chart.axes.map((unit, axis) => plotBounds(scaled, axis, state.window[1]));
    }
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
      ctx.fillStyle = colors.muted; ctx.textAlign = 'left'; ctx.fillText(fmtTime(start, 1), left, height - 4);
      ctx.textAlign = 'right'; ctx.fillText(fmtTime(end, 1), right, height - 4);
      ctx.save(); ctx.beginPath(); ctx.rect(left, top - 3, right - left, bottom - top + 6); ctx.clip();
      ctx.globalAlpha = 0.14;
      for (const item of state.problems) {
        if (item.endSec < start || item.startSec > end) continue;
        const a = x(Math.max(start, item.startSec)), b = x(Math.min(end, item.endSec));
        ctx.fillStyle = problemColor(item); ctx.fillRect(a - 1, top - 3, Math.max(2, b - a), bottom - top + 6);
      }
      ctx.globalAlpha = 1;
      ctx.font = '9px ui-monospace, monospace'; ctx.textAlign = 'left';
      for (const mark of state.match?.marks || []) {
        if (mark.tSec < start || mark.tSec > end) continue;
        ctx.strokeStyle = colors['chart-label']; ctx.globalAlpha = 0.6; ctx.setLineDash([2, 4]);
        ctx.beginPath(); ctx.moveTo(x(mark.tSec), top - 3); ctx.lineTo(x(mark.tSec), bottom + 3); ctx.stroke(); ctx.setLineDash([]);
        ctx.globalAlpha = 1; ctx.fillStyle = colors['chart-label']; ctx.fillText(mark.label, x(mark.tSec) + 4, top + 8);
      }
      const stroke = (points, y) => {
        ctx.beginPath();
        let connected = false, previousValue = null;
        for (const [t, value] of points) {
          if (!finite(value)) { connected = false; continue; }
          if (connected) { ctx.lineTo(x(t), y(previousValue)); ctx.lineTo(x(t), y(value)); }
          else ctx.moveTo(x(t), y(value));
          connected = true; previousValue = value;
        }
        ctx.stroke();
      };
      for (const layer of chart.layers) {
        const value = sampleAt(layer.data, state.time);
        layer.value.textContent = chart.loading ? '…' : `${fmt(value)}${layer.option?.unit ? ' ' + layer.option.unit : ''}`;
        layer.overlayValues.replaceChildren(...(chart.loading ? [] : layer.overlays.filter(trace => trace.overlay.visible && trace.data.length).map(trace => {
          const value = node('span', 'chart-overlay-value', fmt(sampleAt(trace.data, state.time)));
          value.style.setProperty('--run', `var(--overlay-${trace.overlay.slot})`);
          value.dataset.letter = trace.overlay.letter;
          value.title = `Run ${trace.overlay.letter}`;
          return value;
        })));
        const sampled = indexAt(layer.data, state.time);
        layer.value.title = sampled < 0 ? 'No sample at this time' : `Last sample at ${fmtTime(layer.data[sampled][0], 6)}`;
        if (!layer.visible) continue;
        const [min, max] = chart.bounds[layer.axis] || [0, 1];
        const y = value => bottom - (value - min) / (max - min) * (bottom - top);
        ctx.strokeStyle = colors.layers[layer.slot]; ctx.lineWidth = 1.5;
        // Overlay runs keep the signal's color and take their own dash, thinner and under the open run.
        ctx.lineWidth = 1.25; ctx.globalAlpha = 0.8;
        for (const trace of layer.overlays) {
          if (!trace.points.length) continue;
          ctx.setLineDash(OVERLAY_DASHES[trace.overlay.slot]); stroke(trace.points, y);
        }
        ctx.setLineDash([]); ctx.globalAlpha = 1; ctx.lineWidth = 1.5;
        stroke(layer.points, y);
      }
      if (state.time >= start && state.time <= end) {
        ctx.strokeStyle = colors['chart-cursor']; ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(x(state.time), top); ctx.lineTo(x(state.time), bottom); ctx.stroke(); ctx.setLineDash([]);
      }
      if (state.hoverTime !== null && state.hoverTime >= start && state.hoverTime <= end) {
        ctx.strokeStyle = colors.muted; ctx.lineWidth = 1; ctx.beginPath();
        ctx.moveTo(x(state.hoverTime), top - 3); ctx.lineTo(x(state.hoverTime), bottom + 3); ctx.stroke();
      }
      ctx.restore();
      if (state.hoverChart === chart && state.hoverTime !== null) drawTooltip(chart, ctx, width, top, bottom);
      drawDiscrete(chart);
      if (!chart.layers.some(layer => layer.data.length)) {
        ctx.fillStyle = colors.muted; ctx.textAlign = 'center';
        ctx.fillText(chart.loading ? 'Loading samples…' : chart.keys.length ? 'No recorded samples' : 'Add signals to compare them', (left + right) / 2, (top + bottom) / 2);
      }
    }
  }

  // Values at the hovered time, beside the pointer and flipped to its left near the right edge.
  function drawTooltip(chart, ctx, width, top, bottom) {
    const rows = [[fmtTime(state.hoverTime, 3), colors.muted]];
    for (const layer of chart.layers) {
      if (!layer.visible || !layer.data.length) continue;
      const unit = layer.option?.unit ? ' ' + layer.option.unit : '';
      const name = layer.option?.componentLabel ?? layer.option?.name.split('/').at(-1) ?? layer.key;
      let text = `${name}  ${fmt(sampleAt(layer.data, state.hoverTime))}${unit}`;
      rows.push([text, colors.layers[layer.slot]]);
      for (const trace of layer.overlays) {
        if (trace.overlay.visible && trace.data.length) rows.push([`  ${trace.overlay.letter}  ${fmt(sampleAt(trace.data, state.hoverTime))}${unit}`, colors.overlays[trace.overlay.slot]]);
      }
    }
    ctx.font = '10px ui-monospace, monospace';
    const boxWidth = Math.max(...rows.map(([text]) => ctx.measureText(text).width)) + 16, boxHeight = rows.length * 14 + 8;
    let left = state.hoverX + 12;
    if (left + boxWidth > width - 4) left = state.hoverX - 12 - boxWidth;
    const boxTop = Math.max(2, Math.min(bottom - boxHeight, top));
    ctx.fillStyle = colors.tooltip; ctx.strokeStyle = colors['chart-grid'];
    ctx.beginPath(); ctx.roundRect(left, boxTop, boxWidth, boxHeight, 5); ctx.fill(); ctx.stroke();
    ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
    rows.forEach(([text, color], i) => { ctx.fillStyle = color; ctx.fillText(text, left + 8, boxTop + 11 + i * 14); });
    ctx.textBaseline = 'alphabetic';
  }

  for (const button of document.querySelectorAll('[data-window]')) {
    button.onclick = () => setWindowLength(button.dataset.window === 'all' ? Infinity : Number(button.dataset.window));
  }
  $('follow').onclick = follow;
  $('add-graph').onclick = () => {
    if (!state.run || state.charts.length >= 8) return;
    state.focusedChart = null;
    const chart = createChart();
    updateChartControls(); saveLayout();
    chart.select.focus();
  };
  $('picker-close').onclick = closePicker;
  $('channel-clear').onclick = () => chooseChannel(null);
  $('channel-search').oninput = renderChannelTree;
  $('channel-picker').addEventListener('close', () => { state.pickerChart = null; });

  return {closePicker, renderCharts, loadCharts, applyPreset, preparePlots, drawCharts, syncWindow, follow, updateWindowControls};
}
