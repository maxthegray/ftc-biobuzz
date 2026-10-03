import {finite, indexAt, sampleAt, fieldPoint, ballSighting, ballEstimate, sightingMount, loggedCameraMount, cameraDetections, POLLEN_DIAMETER_IN} from './core.mjs';
import {$, state, fmt, node, context, colors} from './ui.js';

// Past this the estimate moves by feet per degree of ty, so only the direction is drawn.
const BALL_TRUST_IN = 48;

export function drawField() {
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
  const ball = !$('field-ball-option').hidden && $('field-ball-toggle').checked && valid ? ballSighting(state.run.playback, state.time, state.cameras) : null;
  const angle = value => value === null ? '—' : `${value >= 0 ? '+' : ''}${fmt(value, 1)}°`;
  $('ball-status').hidden = $('field-ball-option').hidden || !$('field-ball-toggle').checked;
  const estimate = ball ? ballEstimate(pose, ball, sightingMount(state.run.playback, state.time, ball)) : null;
  const near = estimate?.distanceIn !== null && estimate?.distanceIn <= BALL_TRUST_IN;
  const range = !estimate || estimate.distanceIn === null ? '' : near ? ` · ~${fmt(estimate.distanceIn, 0)} in` : ' · far';
  $('ball-status').textContent = !ball ? 'No ball target' : `tx ${angle(ball.txDeg)} · ty ${angle(ball.tyDeg)}${ball.targetTyDeg === null ? '' : ` → ${angle(ball.targetTyDeg)}`}${range}`;
  const ballRadius = Math.max(4, POLLEN_DIAMETER_IN / 2 / view.span * size);
  // Every camera's accepted detections, each through its own logged mount; the tracked one is
  // drawn below with its ray.
  if (ball && !offView) {
    for (const camera of state.cameras) {
      const mount = loggedCameraMount(state.run.playback, state.time, camera);
      if (!mount) continue;
      for (const detection of cameraDetections(state.run.playback, state.time, camera)?.detections || []) {
        if (detection.rejection || detection.txDeg === null || detection.tyDeg === null) continue;
        if (camera === ball.camera && detection.selected) continue;
        const other = ballEstimate(pose, detection, mount);
        if (other.distanceIn === null || other.distanceIn > BALL_TRUST_IN) continue;
        const [otherX, otherY] = fieldPoint(other.point, view, left, top, size);
        ctx.strokeStyle = colors.ball; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.arc(otherX, otherY, ballRadius, 0, Math.PI * 2); ctx.stroke();
      }
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

// Camera views are added from + View, one card per camera, each choosing which camera it shows.
export function createCameraView(key, source, onSourceChange) {
  const card = node('section', 'card camera-card');
  const heading = node('div', 'card-heading');
  const select = node('select');
  select.setAttribute('aria-label', 'Camera to show');
  const summary = node('span', 'muted');
  heading.append(node('h3', '', 'Camera'), select, summary);
  const wrap = node('div', 'camera-wrap');
  const canvas = node('canvas');
  canvas.setAttribute('aria-label', 'Camera detections in image pixels');
  const status = node('span', 'field-status');
  wrap.append(canvas, status);
  const size = node('span', '');
  const caption = node('div', 'field-caption');
  caption.append(node('span', '', 'Filled: selected · outlined: accepted · faded: rejected (reason)'), size);
  card.append(heading, wrap, caption);
  const view = {key, source, card, select, summary, canvas, status, size};
  select.onchange = () => { view.source = select.value; onSourceChange(); drawCamera(); };
  fillCameraSources(view);
  return view;
}

export function fillCameraSources(view) {
  const cameras = state.cameras.length ? state.cameras : view.source ? [view.source] : [];
  if (!view.source || (state.cameras.length && !state.cameras.includes(view.source))) view.source = cameras[0] ?? view.source;
  view.select.replaceChildren(...cameras.map(camera => {
    const option = node('option', '', camera);
    option.value = camera;
    option.selected = camera === view.source;
    return option;
  }));
  view.select.hidden = cameras.length < 2;
}

export function drawCamera() {
  for (const view of state.cameraViews) drawCameraView(view);
}

function drawCameraView(view) {
  if (!state.run || !view.canvas.clientWidth) return;
  const {ctx, width, height} = context(view.canvas);
  const frame = view.source ? cameraDetections(state.run.playback, state.time, view.source) : null;
  view.status.textContent = !frame ? 'No camera detections in this log' : !frame.fresh ? 'No fresh camera frame' : '';
  const accepted = frame?.detections.filter(detection => !detection.rejection).length ?? 0;
  view.summary.textContent = frame?.fresh ? `${frame.detections.length} candidates · ${accepted} accepted` : '';
  const frameWidth = frame?.widthPx ?? 640, frameHeight = frame?.heightPx ?? 480;
  view.size.textContent = frame?.widthPx ? `${frameWidth} × ${frameHeight} px` : '';
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

