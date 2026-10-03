import {$, state, node} from './ui.js';
import {createCameraView, drawCamera} from './field-camera.js';

export function createLayout({seek, preparePlots, redraw}) {
  // Each tab lays its views out in columns (state.panels[tab] is an array of columns), so a tall
  // view on one side leaves the other side free for more views rather than an empty row.
  const TAB_COLUMNS = {field: 2, signals: 1, events: 2};
  const shownKeys = () => Object.values(state.panels).flat(2);

  function columnsOf(name) {
    const panel = $(`panel-${name}`);
    let container = panel.querySelector(':scope > .view-columns');
    if (!container) {
      container = node('div', 'view-columns');
      container.style.setProperty('--columns', TAB_COLUMNS[name]);
      for (let i = 0; i < TAB_COLUMNS[name]; i++) container.append(node('div', 'view-column'));
      panel.prepend(container);
    }
    return [...container.children];
  }

  function selectTab(name, focus = false) {
    state.activeTab = name;
    for (const view of views.values()) $('view-storage').append(view);
    const columns = columnsOf(name);
    state.panels[name].forEach((keys, i) => { for (const key of keys) columns[i].append(views.get(key)); });
    layoutColumns(name);
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
      fieldCommands: $('field-commands-toggle').checked, fieldBall: $('field-ball-toggle').checked,
      cameraViews: Object.fromEntries(state.cameraViews.map(view => [view.key, view.source]))};
    try { localStorage.setItem(layoutKey, JSON.stringify(state.layout)); } catch { /* Storage may be disabled. */ }
  }

  function updateViewControls() {
    // Camera views can be added more than once, one per camera; the rest are single.
    for (const option of $('add-view').options) option.disabled = option.value !== 'camera' && state.panels[state.activeTab].flat().includes(option.value);
    for (const panel of document.querySelectorAll('.view-panel')) {
      let empty = panel.querySelector('.empty-tab');
      if (!empty) { empty = node('p', 'empty empty-tab', 'Use + View to add a view.'); panel.append(empty); }
      empty.hidden = state.panels[panel.id.slice(6)].flat().length > 0;
    }
  }

  function addView(key) {
    const columns = state.panels[state.activeTab];
    // New views go to the column with the fewest views; on a tie, away from the field, which is
    // the view that fills its column.
    const shortest = columns.reduce((best, keys, i) =>
      keys.length < columns[best].length || (keys.length === columns[best].length && columns[best].includes('field')) ? i : best, 0);
    if (key === 'camera') {
      const used = state.cameraViews.map(view => Number(view.key.split(':')[1]));
      const next = `camera:${Math.max(0, ...used) + 1}`;
      const shown = new Set(shownKeys());
      const spare = state.cameraViews.find(view => !shown.has(view.key));
      createCameraViewIfNeeded(spare ? spare.key : next, state.cameras.find(camera => !usedCameras().includes(camera)));
      columns[shortest].push(spare ? spare.key : next);
    } else {
      if (!views.has(key) || columns.flat().includes(key)) return;
      columns[shortest].push(key);
    }
    saveLayout(); selectTab(state.activeTab);
    drawCamera();
  }

  const usedCameras = () => state.cameraViews.map(view => view.source).filter(Boolean);

  function createCameraViewIfNeeded(key, source) {
    const existing = state.cameraViews.find(view => view.key === key);
    if (existing) return existing;
    const view = createCameraView(key, source ?? state.cameras[0] ?? null, saveLayout);
    state.cameraViews.push(view);
    views.set(key, view.card);
    addDragHandle(key, view.card);
    addHideButton(key, view.card);
    return view;
  }

  // Views are reordered by dragging their handle, or with Alt+arrow while the handle has focus.
  // Pointer events rather than HTML5 drag: they work with touch and a pen too. The card itself
  // moves as the pointer goes, so the drop is what you already see.
  function addDragHandle(key, card) {
    card.dataset.viewKey = key;
    const handle = node('button', 'drag-handle', '⠿');
    handle.title = 'Drag to move · Alt+arrows to move by keyboard';
    handle.setAttribute('aria-label', `Move ${viewNames[key] ?? 'Camera'}`);
    handle.onpointerdown = event => {
      if (event.button !== 0) return;
      event.preventDefault();
      state.dragKey = key;
      card.classList.add('dragging');
      card.parentElement.parentElement.classList.add('dragging');
      layoutColumns(state.activeTab, true);
      // On window, so a drag that leaves the handle still tracks and always ends.
      const move = moved => { if (state.dragKey === key) placeDraggedCard(card, moved.clientX, moved.clientY); };
      const stop = () => {
        window.removeEventListener('pointermove', move);
        window.removeEventListener('pointerup', finish);
        window.removeEventListener('pointercancel', cancel);
        endDrag(card);
      };
      const finish = () => { stop(); commitColumns(); };
      const cancel = () => { stop(); selectTab(state.activeTab); };
      window.addEventListener('pointermove', move);
      window.addEventListener('pointerup', finish);
      window.addEventListener('pointercancel', cancel);
    };
    handle.onkeydown = event => {
      if (!event.altKey) return;
      const step = {ArrowUp: [0, -1], ArrowDown: [0, 1], ArrowLeft: [-1, 0], ArrowRight: [1, 0]}[event.key];
      if (step) { event.preventDefault(); moveView(key, ...step); }
    };
    card.querySelector('.card-heading').prepend(handle);
  }

  function endDrag(card) {
    state.dragKey = null;
    card.classList.remove('dragging');
    for (const marked of document.querySelectorAll('.view-columns.dragging, .view-column.drop-column')) marked.classList.remove('dragging', 'drop-column');
    layoutColumns(state.activeTab);
  }

  // Moves the dragged card in the DOM to where the pointer is. Within a column the card only jumps
  // when the pointer crosses the middle of a neighbour, so it never flickers back and forth.
  function placeDraggedCard(card, x, y) {
    const columns = columnsOf(state.activeTab);
    const target = columns.find(column => { const box = column.getBoundingClientRect(); return x >= box.left && x < box.right; })
      ?? columns.reduce((best, column) => {
        const box = column.getBoundingClientRect(), gap = Math.min(Math.abs(x - box.left), Math.abs(x - box.right));
        return gap < best.gap ? {column, gap} : best;
      }, {column: columns[0], gap: Infinity}).column;
    for (const column of columns) column.classList.toggle('drop-column', column === target);
    const middle = other => { const box = other.getBoundingClientRect(); return box.top + box.height / 2; };
    const others = [...target.children].filter(other => other !== card);
    if (card.parentElement === target) {
      const index = [...target.children].indexOf(card);
      const above = others.slice(0, index).find(other => y < middle(other));
      const below = others.slice(index).reverse().find(other => y > middle(other));
      if (above) target.insertBefore(card, above);
      else if (below) below.after(card);
    } else {
      const before = others.find(other => y < middle(other));
      if (before) target.insertBefore(card, before); else target.append(card);
    }
  }

  // Columns share the width by what they hold: one with the field gets more, an empty one none —
  // except while dragging, when every column stays open as a drop target.
  function layoutColumns(name, dragging = false) {
    const columns = columnsOf(name);
    const weights = columns.map(column => {
      const empty = !column.children.length;
      column.hidden = empty && !dragging;
      return empty ? 'minmax(0, 1fr)' : column.querySelector('.field-card') ? 'minmax(0, 3fr)' : 'minmax(0, 2fr)';
    });
    const template = columns.filter(column => !column.hidden).map(column => weights[columns.indexOf(column)]).join(' ');
    columns[0].parentElement.style.setProperty('--template', template);
  }

  function commitColumns() {
    state.panels[state.activeTab] = columnsOf(state.activeTab).map(column => [...column.children].map(card => card.dataset.viewKey));
    saveLayout(); updateViewControls();
    redraw();
  }

  function moveView(key, dColumn, dIndex) {
    const columns = state.panels[state.activeTab];
    const from = columns.findIndex(keys => keys.includes(key));
    if (from < 0) return;
    const index = columns[from].indexOf(key), to = from + dColumn;
    if (to < 0 || to >= columns.length) return;
    columns[from].splice(index, 1);
    const at = Math.max(0, Math.min(columns[to].length, index + dIndex));
    columns[to].splice(at, 0, key);
    saveLayout(); selectTab(state.activeTab);
    views.get(key)?.querySelector('.drag-handle')?.focus();
  }

  function addHideButton(key, card) {
    const remove = node('button', 'text-button hide-view', '×');
    remove.setAttribute('aria-label', `Hide ${viewNames[key] ?? 'Camera'} from this tab`);
    remove.title = 'Hide from this tab';
    remove.onclick = () => {
      state.panels[state.activeTab] = state.panels[state.activeTab].map(keys => keys.filter(item => item !== key));
      if (key.startsWith('camera:') && !shownKeys().includes(key)) {
        // A camera view shown nowhere is discarded; take its card out too, since selectTab only
        // sweeps registered views.
        state.cameraViews = state.cameraViews.filter(view => view.key !== key);
        views.delete(key);
        card.remove();
      }
      saveLayout(); selectTab(state.activeTab);
    };
    card.querySelector('.card-heading').append(remove);
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
      commands: $('commands').closest('.card'), events: $('events').closest('.card'), gamepads: document.querySelector('.inputs-card')};
    for (const [key, view] of Object.entries(elements)) {
      view.dataset.panel = key; views.set(key, view);
      addDragHandle(key, view);
      addHideButton(key, view);
    }
    document.querySelectorAll('[role="tab"]').forEach(wireTab);
    try {
      const saved = JSON.parse(localStorage.getItem(layoutKey));
      $('field-commands-toggle').checked = saved?.fieldCommands !== false;
      $('field-ball-toggle').checked = saved?.fieldBall !== false;
      const previous = saved || JSON.parse(localStorage.getItem('maxscope.layout.v1'));
      if (saved?.panels) {
        for (const keys of Object.values(saved.panels)) {
          for (const key of Array.isArray(keys) ? keys.flat() : []) {
            if (typeof key === 'string' && key.startsWith('camera:')) createCameraViewIfNeeded(key, saved.cameraViews?.[key]);
          }
        }
        for (const tab of Object.keys(state.panels)) {
          if (Array.isArray(saved.panels[tab])) state.panels[tab] = restoredColumns(saved.panels[tab], TAB_COLUMNS[tab]);
        }
      }
      if (Array.isArray(previous?.layers) && previous.layers.length > 0) {
        state.layout = {discrete: Array.isArray(previous.discrete) ? previous.discrete.slice(0, 8).map(keys => Array.isArray(keys) ? [...new Set(keys.filter(key => typeof key === 'string'))].slice(0, 8) : []) : undefined, layers: previous.layers.slice(0, 8).map(keys => Array.isArray(keys) ? [...new Set(keys.filter(key => typeof key === 'string'))].slice(0, 8) : [])};
      }
    } catch { /* Ignore an unavailable or outdated saved layout. */ }
    for (const key of new Set(shownKeys())) if (key.startsWith('camera:')) createCameraViewIfNeeded(key);
    selectTab('field');
  }

  // A saved tab is columns of keys, or a flat list from before columns, which was drawn row by row
  // across two columns. Unknown keys drop out; extra columns fold into the last one.
  function restoredColumns(saved, count) {
    const columns = Array.from({length: count}, () => []);
    const seen = new Set();
    const place = (key, column) => {
      if (typeof key === 'string' && views.has(key) && !seen.has(key)) { seen.add(key); columns[Math.min(column, count - 1)].push(key); }
    };
    if (saved.every(Array.isArray)) saved.forEach((keys, column) => keys.forEach(key => place(key, column)));
    else saved.forEach((key, i) => place(key, i % count));
    return columns;
  }

  $('add-view').onchange = event => { addView(event.target.value); event.target.value = ''; };

  return {initializeLayout, saveLayout};
}
