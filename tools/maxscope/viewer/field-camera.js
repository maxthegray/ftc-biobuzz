import {finite, indexAt, sampleAt, fieldPoint, ROBOT_HALF_IN, ballSighting, ballEstimate, sightingMount, loggedCameraMount, cameraDetections, POLLEN_DIAMETER_IN} from './core.mjs';
import {$, state, fmt, fmtTime, node, context, colors, problemColor} from './ui.js';
import {shownOverlays} from './overlays.js';

// Past this the estimate moves by feet per degree of ty, so only the direction is drawn.
const BALL_TRUST_IN = 48;

export function drawField() {
  if (!state.run || !$('field').clientWidth) return;
  const {ctx, width, height} = context($('field'));
  const length = state.run.fieldLengthIn, view = state.fieldView, size = Math.min(width - 78, height - 58);
  const left = (width - size) / 2 + 8, top = 17, cell = size / view.tiles;
  state.fieldGeometry = {left, top, size};
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
  // Foam tile seams, then the walls over them.
  ctx.lineWidth = 2; ctx.strokeStyle = colors.grid; ctx.font = '9px ui-monospace, monospace';
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
  ctx.strokeStyle = colors.wall; ctx.lineWidth = 2.5; ctx.strokeRect(fieldLeft, fieldTop, fieldSize, fieldSize);
  ctx.fillStyle = colors['field-caption']; ctx.textAlign = 'center';
  ctx.fillText('AUDIENCE WALL · +X →', left + size / 2, height - 4);
  // Along the left edge, clear of the tick labels.
  ctx.save(); ctx.translate(10, top + size / 2); ctx.rotate(-Math.PI / 2); ctx.textAlign = 'center'; ctx.fillText('+Y →', 0, 0); ctx.restore();
  ctx.save(); ctx.beginPath(); ctx.rect(left, top, size, size); ctx.clip();
  const drawTrail = (trail, past, color) => {
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
  // Overlay runs under the open one, each in its own color: faint whole path, stronger played part.
  const overlays = shownOverlays();
  for (const overlay of overlays) {
    const color = colors.overlays[overlay.slot];
    ctx.globalAlpha = 0.18; drawTrail(overlay.trail, false, color);
    ctx.globalAlpha = 0.6; drawTrail(overlay.trail, true, color); ctx.globalAlpha = 1;
  }
  // When the graphs are detached on part of the run, a soft band under the path marks the stretch
  // they show, labelled in the corner; the trail's own played/unplayed colors stay on top. While
  // they follow playback there is no band: it would only trail the robot.
  const [windowStart, windowEnd] = state.window;
  const zoomed = !state.follow && windowEnd - windowStart < 0.9 * state.run.endSec;
  $('field-zoom').hidden = !zoomed;
  if (zoomed) {
    $('field-zoom-label').textContent = `Graphs show ${fmtTime(windowStart, 1)} to ${fmtTime(windowEnd, 1)}`;
    const series = state.run.playback.pose || [];
    const first = Math.max(0, indexAt(series, windowStart)), last = indexAt(series, windowEnd);
    const stride = Math.max(1, Math.ceil((last - first) / 3000));
    ctx.save(); ctx.globalAlpha = 0.22; ctx.lineWidth = 9; ctx.lineCap = 'round'; ctx.lineJoin = 'round';
    ctx.strokeStyle = colors['trail-window']; ctx.beginPath();
    let connected = false;
    for (let i = first; i <= last; i += stride) {
      const pose = series[i][1];
      if (!Array.isArray(pose) || !pose.slice(0, 2).every(finite)) { connected = false; continue; }
      const [px, py] = fieldPoint(pose, view, left, top, size);
      if (connected) ctx.lineTo(px, py); else ctx.moveTo(px, py);
      connected = true;
    }
    ctx.stroke(); ctx.restore();
  }
  drawTrail(state.trail || [], false, colors['trail-future']); drawTrail(state.trail || [], true, colors['trail-past']);

  const series = state.run.playback.pose || [];
  if (state.problemsOn) {
    ctx.lineWidth = 2;
    for (const item of state.problems) {
      const at = sampleAt(series, item.startSec);
      if (!Array.isArray(at) || !at.slice(0, 2).every(finite)) continue;
      const [px, py] = fieldPoint(at, view, left, top, size);
      ctx.strokeStyle = problemColor(item); ctx.beginPath();
      ctx.moveTo(px - 4, py - 4); ctx.lineTo(px + 4, py + 4); ctx.moveTo(px + 4, py - 4); ctx.lineTo(px - 4, py + 4); ctx.stroke();
    }
  }
  // Each overlay's robot at the cursor, tagged with its letter, once that run has started.
  const radius = Math.max(5, ROBOT_HALF_IN * size / view.span);
  for (const overlay of overlays) {
    const poses = overlay.series.get('pose') || [];
    const ghost = state.time >= (poses[0]?.[0] ?? Infinity) ? sampleAt(poses, state.time) : null;
    if (!Array.isArray(ghost) || !ghost.slice(0, 3).every(finite)) continue;
    const color = colors.overlays[overlay.slot];
    const [gx, gy] = fieldPoint(ghost, view, left, top, size);
    ctx.save(); ctx.translate(gx, gy); ctx.rotate(-ghost[2]);
    drawRobot(ctx, radius, color);
    ctx.restore();
    ctx.globalAlpha = 1; ctx.font = '600 10px Inter, system-ui, sans-serif'; ctx.textAlign = 'center';
    ctx.fillStyle = color; ctx.fillText(overlay.letter, gx, gy - radius * 1.5 - 3);
  }
  const pose = sampleAt(series, state.time);
  const valid = Array.isArray(pose) && pose.length >= 3 && pose.slice(0, 3).every(finite);
  $('pose-x').textContent = fmt(pose?.[0]); $('pose-y').textContent = fmt(pose?.[1]); $('pose-h').textContent = fmt(finite(pose?.[2]) ? pose[2] * 180 / Math.PI : null, 1);
  $('pose-h').title = `${fmt(pose?.[2], 3)} rad`;
  const index = indexAt(series, state.time);
  const age = index >= 0 ? state.time - series[index][0] : null;
  const outside = valid && (pose[0] < 0 || pose[1] < 0 || pose[0] > length || pose[1] > length);
  const [x, y] = valid ? fieldPoint(pose, view, left, top, size) : [];
  const offView = valid && (x < left || x > left + size || y < top || y > top + size);
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
  // The view holds the robot's body; its heading arrow may reach past it, so it isn't clipped.
  ctx.restore(); ctx.save();
  if (offView) {
    const edgeX = Math.min(left + size - 2, Math.max(left + 2, x)), edgeY = Math.min(top + size - 2, Math.max(top + 2, y));
    ctx.translate(edgeX, edgeY); ctx.rotate(Math.atan2(y - edgeY, x - edgeX));
    ctx.fillStyle = colors['robot-arrow']; ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(-12, -6); ctx.lineTo(-12, 6); ctx.closePath(); ctx.fill();
  } else if (valid) {
    const radius = Math.max(5, ROBOT_HALF_IN * size / view.span);
    ctx.translate(x, y); ctx.rotate(-pose[2]);
    drawRobot(ctx, radius, colors.lime);
    // With overlays on, the open run's robot is A, matching the overlay bar.
    if (overlays.length) {
      ctx.rotate(pose[2]); ctx.font = '600 10px Inter, system-ui, sans-serif'; ctx.textAlign = 'center';
      ctx.fillStyle = colors.lime; ctx.fillText('A', 0, -radius * 1.5 - 3);
    }
  }
  ctx.restore();
}

// The robot marker, already translated and rotated to the pose (+x is the front): a chassis
// outline, a wheel pod in each corner, and a heading line from the centre to the front edge.
function drawRobot(ctx, radius, color) {
  ctx.lineJoin = 'round';
  ctx.globalAlpha = 0.16; ctx.fillStyle = color; ctx.fillRect(-radius, -radius, radius * 2, radius * 2);
  ctx.globalAlpha = 1; ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.strokeRect(-radius, -radius, radius * 2, radius * 2);
  const length = radius * 0.62, width = radius * 0.34, inset = radius * 0.1;
  ctx.globalAlpha = 0.75; ctx.fillStyle = color;
  for (const x of [-1, 1]) for (const y of [-1, 1]) {
    ctx.fillRect(x * (radius - inset) - (x > 0 ? length : 0), y * (radius - inset) - (y > 0 ? width : 0), length, width);
  }
  ctx.globalAlpha = 1; ctx.lineWidth = 2; ctx.lineCap = 'round';
  ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(radius, 0); ctx.stroke();
  ctx.beginPath(); ctx.arc(0, 0, Math.max(2, radius * 0.12), 0, Math.PI * 2); ctx.fill();
  ctx.lineCap = 'butt';
}

// A click on the field canvas in field inches, with a pick radius of about 12 px, or null when it
// falls outside the drawn field view.
export function fieldInches(event) {
  const geometry = state.fieldGeometry, view = state.fieldView;
  if (!geometry || !view) return null;
  const rect = $('field').getBoundingClientRect();
  const x = (event.clientX - rect.left - geometry.left) / geometry.size, y = (event.clientY - rect.top - geometry.top) / geometry.size;
  if (x < 0 || x > 1 || y < 0 || y > 1) return null;
  return {at: [view.minX + x * view.span, view.minY + (1 - y) * view.span], radiusIn: Math.max(3, 12 / geometry.size * view.span)};
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

