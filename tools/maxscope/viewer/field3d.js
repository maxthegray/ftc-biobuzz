// MaxScope's experimental 3D field. A page of its own: the main viewer never loads it or three.js,
// and only tells it the run and time it is showing (BroadcastChannel 'maxscope').
//
// Pedro's frame throughout, z up: origin at the audience-left corner, +X along the audience wall,
// +Y away from the audience, inches.
import * as THREE from './vendor/three/three.module.js';
import {OrbitControls} from './vendor/three/OrbitControls.js';
import {finite, sampleAt, indexAt, matchPhases, phaseAt, ROBOT_HALF_IN} from './core.mjs';
import {BIOBUZZ, flowerPoints, hiveStates} from './field-elements.js';
import {runLabel, runDate} from './ui.js';

const $ = id => document.getElementById(id);
const page = {run: null, match: null, time: 0, playing: false, frame: null, view: 'audience', poses: [], generation: 0};
const css = name => getComputedStyle(document.documentElement).getPropertyValue(`--${name}`).trim();
const WALL_HEIGHT = 12, ROBOT_HEIGHT = 12, TRAIL_POINTS = 4000;

const renderer = new THREE.WebGLRenderer({antialias: true});
renderer.setPixelRatio(window.devicePixelRatio || 1);
renderer.shadowMap.enabled = true;
$('stage').append(renderer.domElement);
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(42, 1, 1, 3000);
camera.up.set(0, 0, 1);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.maxPolarAngle = Math.PI * 0.49;

const material = (color, options = {}) => new THREE.MeshStandardMaterial({color, roughness: 0.85, metalness: 0.05, ...options});
const box = (w, d, h, mat) => new THREE.Mesh(new THREE.BoxGeometry(w, d, h), mat);
const edges = (mesh, color) => mesh.add(new THREE.LineSegments(new THREE.EdgesGeometry(mesh.geometry), new THREE.LineBasicMaterial({color})));

// A round bar between two points.
function bar(a, b, radius, mat) {
  const start = new THREE.Vector3(...a), end = new THREE.Vector3(...b);
  const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, start.distanceTo(end), 10), mat);
  mesh.position.copy(start).add(end).multiplyScalar(0.5);
  mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), end.clone().sub(start).normalize());
  mesh.castShadow = true;
  return mesh;
}

// A flat strip of tape on the tiles.
function tape(x0, y0, x1, y1, color, opacity = 1) {
  const mesh = new THREE.Mesh(new THREE.PlaneGeometry(x1 - x0, y1 - y0), new THREE.MeshBasicMaterial({color, transparent: opacity < 1, opacity}));
  mesh.position.set((x0 + x1) / 2, (y0 + y1) / 2, 0.08);
  return mesh;
}

const world = {field: new THREE.Group(), elements: new THREE.Group(), lights: new THREE.Group(), hives: {}, robot: null, trail: null, past: null};
scene.add(world.field, world.elements, world.lights);

function buildScene(length) {
  world.field.clear(); world.elements.clear(); world.lights.clear();
  if (world.robot) scene.remove(world.robot);
  const tile = length / 6, centre = length / 2;
  const red = css('alliance-red'), blue = css('alliance-blue');
  scene.background = new THREE.Color(css('surface'));
  world.lights.add(new THREE.HemisphereLight(0xffffff, 0x404040, 1.6));
  const sun = new THREE.DirectionalLight(0xffffff, 1.6);
  sun.position.set(centre - 80, centre - 140, 220); sun.target.position.set(centre, centre, 0);
  sun.castShadow = true; sun.shadow.mapSize.set(2048, 2048);
  Object.assign(sun.shadow.camera, {left: -130, right: 130, top: 130, bottom: -130, near: 10, far: 500});
  world.lights.add(sun, sun.target);

  // Floor around the field, then the 36 foam TILES with their seams showing between them.
  const ground = new THREE.Mesh(new THREE.PlaneGeometry(length * 4, length * 4), material(css('off-field')));
  ground.position.set(centre, centre, -0.6); ground.receiveShadow = true;
  const seams = new THREE.Mesh(new THREE.PlaneGeometry(length, length), material(css('grid')));
  seams.position.set(centre, centre, -0.05); seams.receiveShadow = true;
  world.field.add(ground, seams);
  const foam = [material(css('field')), material(new THREE.Color(css('field')).offsetHSL(0, 0, 0.02))];
  for (let i = 0; i < 6; i++) for (let j = 0; j < 6; j++) {
    const tileMesh = new THREE.Mesh(new THREE.PlaneGeometry(tile - 0.4, tile - 0.4), foam[(i + j) % 2]);
    tileMesh.position.set((i + 0.5) * tile, (j + 0.5) * tile, 0); tileMesh.receiveShadow = true;
    world.field.add(tileMesh);
  }
  // Perimeter: clear walls with a rail on top; the alliance walls' rails in their colors.
  const clear = material(0xffffff, {transparent: true, opacity: 0.12, roughness: 0.2});
  for (const [x, y, w, d, rail] of [[centre, -0.5, length + 2, 1, css('wall')], [centre, length + 0.5, length + 2, 1, css('wall')],
    [-0.5, centre, 1, length, red], [length + 0.5, centre, 1, length, blue]]) {
    const wall = box(w, d, WALL_HEIGHT, clear); wall.position.set(x, y, WALL_HEIGHT / 2);
    const top = box(w + 0.4, d + 0.4, 1, material(rail)); top.position.set(x, y, WALL_HEIGHT + 0.5); top.castShadow = true;
    world.field.add(wall, top);
  }
  // ALLIANCE AREAS outside the side walls, between tile seams 1 and 5 (Event Field Setup Guide §8).
  world.field.add(tape(-54, tile, -1, 5 * tile, red, 0.18), tape(length + 1, tile, length + 54, 5 * tile, blue, 0.18));

  // LOADING ZONES and GARDENS: tape on three sides and a light fill; GARDENS a solid strip.
  const strip = 1.5, {loadingDepth: deep, gardenDepth} = BIOBUZZ;
  for (const [x0, x1, y0, y1, color, wallX] of [[0, deep, 4 * tile, 5 * tile, red, 0], [length - deep, length, tile, 2 * tile, blue, length]]) {
    world.elements.add(tape(x0, y0, x1, y1, color, 0.18));
    const inner = wallX === 0 ? x1 - strip : x0;
    world.elements.add(tape(x0, y1 - strip, x1, y1, color), tape(x0, y0, x1, y0 + strip, color), tape(inner, y0, inner + strip, y1, color));
  }
  world.elements.add(tape(0, 0, tile, gardenDepth, red), tape(length - tile, length - gardenDepth, length, length, blue));

  // HIVE frame: an A-frame at each end of its width, joined at the apex by the crossbar.
  const metal = material(css('frame'), {metalness: 0.6, roughness: 0.4});
  const {frameWidth, frameDepth, pivotHeight} = BIOBUZZ;
  for (const x of [centre - frameWidth / 2, centre + frameWidth / 2]) {
    world.elements.add(bar([x, centre - frameDepth / 2, 0], [x, centre, pivotHeight], 0.75, metal),
      bar([x, centre + frameDepth / 2, 0], [x, centre, pivotHeight], 0.75, metal),
      bar([x, centre - frameDepth / 2, 0.75], [x, centre + frameDepth / 2, 0.75], 0.75, metal));
  }
  world.elements.add(bar([centre - frameWidth / 2, centre, pivotHeight], [centre + frameWidth / 2, centre, pivotHeight], 0.9, metal));

  // HIVES: an arm through the pivot with a CELL at each end. A CELL is its Figure 9-11 outline,
  // 20 in. wide and 14 in. tall, extruded 12.04 in. along the arm.
  const outline = new THREE.Shape();
  const {cellWidth: w, cellHeight: h, cellSide: side, cellDepth: depth, cellGap: gap} = BIOBUZZ;
  outline.moveTo(-w / 2, 0); outline.lineTo(w / 2, 0); outline.lineTo(w / 2, side); outline.lineTo(0, h); outline.lineTo(-w / 2, side); outline.closePath();
  const cellGeometry = new THREE.ExtrudeGeometry(outline, {depth, bevelEnabled: false});
  cellGeometry.rotateX(Math.PI / 2); cellGeometry.translate(0, depth / 2, -h / 2);
  for (const [alliance, color, x] of [['RED', red, centre - BIOBUZZ.hiveSpacing / 2], ['BLUE', blue, centre + BIOBUZZ.hiveSpacing / 2]]) {
    const hive = new THREE.Group(); hive.position.set(x, centre, pivotHeight);
    const arm = new THREE.Group(); hive.add(arm);
    arm.add(bar([0, -gap / 2, 0], [0, gap / 2, 0], 0.8, metal));
    const cells = {};
    for (const [location, sign] of [['AUDIENCE', -1], ['FAR', 1]]) {
      const cell = new THREE.Mesh(cellGeometry, material(color, {transparent: true, opacity: 0.4, side: THREE.DoubleSide}));
      cell.position.y = sign * (gap / 2 + depth / 2); cell.castShadow = true;
      edges(cell, color);
      arm.add(cell); cells[location] = cell;
    }
    world.elements.add(hive);
    world.hives[alliance] = {arm, cells};
  }

  // FLOWERS: base ring on the tiles, four pipes, and the opening 21.5 in. up.
  const gold = material(css('flower'), {metalness: 0.3}), green = material(css('flower-core')), black = material(0x222222);
  for (const [fx, fy] of flowerPoints(length)) {
    const flower = new THREE.Group(); flower.position.set(fx, fy, 0);
    const base = new THREE.Mesh(new THREE.TorusGeometry(BIOBUZZ.flowerRing - 0.5, 0.5, 8, 24), black); base.position.z = 0.4;
    const top = new THREE.Mesh(new THREE.TorusGeometry(BIOBUZZ.flowerOpening + 0.4, 0.6, 8, 24), gold); top.position.z = BIOBUZZ.flowerHeight;
    flower.add(base, top);
    for (const [dx, dy] of [[1, 1], [1, -1], [-1, 1], [-1, -1]]) flower.add(bar([dx * 1.7, dy * 1.7, 4], [dx * 1.7, dy * 1.7, BIOBUZZ.flowerHeight], 0.35, green));
    world.elements.add(flower);
  }

  // The robot: a 16 in. chassis box with a pale front face and four wheels.
  const accent = css('lime'), size = ROBOT_HALF_IN * 2;
  const robot = new THREE.Group();
  const body = box(size, size, ROBOT_HEIGHT - 3, material(accent, {transparent: true, opacity: 0.85}));
  body.position.z = (ROBOT_HEIGHT - 3) / 2 + 2.5; body.castShadow = true; edges(body, 0x111111);
  const front = box(1, size - 2, ROBOT_HEIGHT - 5, material(0xffffff)); front.position.set(size / 2, 0, body.position.z);
  robot.add(body, front);
  for (const [dx, dy] of [[1, 1], [1, -1], [-1, 1], [-1, -1]]) {
    const wheel = new THREE.Mesh(new THREE.CylinderGeometry(2, 2, 1.6, 16), material(0x1b1b1b));
    wheel.position.set(dx * (size / 2 - 3), dy * (size / 2 + 0.2), 2); wheel.castShadow = true;
    robot.add(wheel);
  }
  robot.visible = false;
  scene.add(robot);
  world.robot = robot;
}

// The recorded path on the tiles: the whole run faint, the part played so far in the accent.
function buildTrail() {
  for (const line of [world.trail, world.past]) if (line) { scene.remove(line); line.geometry.dispose(); }
  const stride = Math.max(1, Math.ceil(page.poses.length / TRAIL_POINTS));
  page.trailTimes = [];
  const points = [];
  page.poses.forEach(([time, pose], i) => {
    if ((i % stride && i !== page.poses.length - 1) || !Array.isArray(pose) || !finite(pose[0]) || !finite(pose[1])) return;
    points.push(new THREE.Vector3(pose[0], pose[1], 0.3)); page.trailTimes.push([time]);
  });
  const geometry = new THREE.BufferGeometry().setFromPoints(points);
  world.trail = new THREE.Line(geometry, new THREE.LineBasicMaterial({color: css('trail-future')}));
  world.past = new THREE.Line(geometry.clone(), new THREE.LineBasicMaterial({color: css('trail-past')}));
  world.past.position.z = 0.05;
  scene.add(world.trail, world.past);
}

// Camera presets, all looking at the field from where a person would stand.
function setView(name, jump = true) {
  page.view = name;
  for (const button of document.querySelectorAll('[data-view]')) button.setAttribute('aria-pressed', String(button.dataset.view === name));
  controls.enabled = name !== 'chase';
  if (name === 'chase' || !jump) return;
  const length = page.run?.fieldLengthIn ?? 141.5, c = length / 2;
  const views = {audience: [[c, -length * 0.95, length * 0.8], [c, c * 0.95, 0]], red: [[-length * 0.95, c, length * 0.7], [c, c, 0]],
    blue: [[length * 1.95, c, length * 0.7], [c, c, 0]], top: [[c, c - 1, length * 2.1], [c, c, 0]]};
  const [position, target] = views[name];
  camera.position.set(...position); controls.target.set(...target); controls.update();
}

function updateScene() {
  const pose = sampleAt(page.poses, page.time);
  const valid = Array.isArray(pose) && pose.slice(0, 3).every(finite);
  world.robot.visible = valid;
  if (valid) { world.robot.position.set(pose[0], pose[1], 0); world.robot.rotation.z = pose[2]; }
  if (world.past) world.past.geometry.setDrawRange(0, Math.max(0, indexAt(page.trailTimes, page.time) + 1));
  const hives = hiveStates(page.run.playback, page.time);
  for (const [alliance, {arm, cells}] of Object.entries(world.hives)) {
    // Tipping the audience CELL up turns the arm's -Y end upwards.
    arm.rotation.x = (hives[alliance].state === 'AUDIENCE_RAISED' ? -1 : 1) * BIOBUZZ.hiveTilt;
    for (const [location, cell] of Object.entries(cells)) cell.material.opacity = hives[alliance].state === `${location}_RAISED` ? 0.6 : 0.3;
  }
  if (page.view === 'chase' && valid) {
    const back = new THREE.Vector3(Math.cos(pose[2]), Math.sin(pose[2]), 0);
    camera.position.lerp(new THREE.Vector3(pose[0], pose[1], 0).addScaledVector(back, -70).add(new THREE.Vector3(0, 0, 45)), 0.15);
    controls.target.lerp(new THREE.Vector3(pose[0], pose[1], 6).addScaledVector(back, 30), 0.2);
  }
  const start = page.run.match?.startSec ?? 0, t = page.time - start;
  const phase = phaseAt(page.match, page.time);
  $('time').textContent = `${t < 0 ? '−' : ''}${Math.abs(t).toFixed(2)} s${phase ? ` · ${phase.name}${phase.clock ? ' ' + phase.clock : ''}` : ''}`;
  $('pose').textContent = valid ? `X ${pose[0].toFixed(1)} · Y ${pose[1].toFixed(1)} in · ${(pose[2] * 180 / Math.PI).toFixed(0)}°` : 'No pose';
  $('scrub').value = page.time;
}

function setTime(time) {
  if (!page.run) return;
  page.time = Math.max(0, Math.min(page.run.endSec, time));
  updateScene();
}

async function load(id, time = null) {
  const generation = ++page.generation;
  $('field3d-note').hidden = false; $('field3d-note').textContent = 'Opening run…';
  try {
    const response = await fetch(`/api/log?id=${encodeURIComponent(id)}`);
    const run = await response.json();
    if (!response.ok) throw new Error(run.error || 'Could not read this log');
    if (generation !== page.generation) return;
    page.run = run; page.match = matchPhases(run.match); page.poses = run.playback.pose || [];
    buildScene(run.fieldLengthIn); buildTrail();
    world.elements.visible = $('elements').checked;
    $('run').value = id;
    $('scrub').max = run.endSec; $('play').disabled = run.endSec <= 0;
    $('field3d-note').hidden = Boolean(page.poses.length);
    $('field3d-note').textContent = page.poses.length ? '' : 'This run has no pose channel.';
    if (!camera.userData.placed) { setView(page.view); camera.userData.placed = true; }
    setTime(time ?? Math.max(run.match?.startSec ?? 0, page.poses[0]?.[0] ?? 0));
  } catch (error) {
    if (generation === page.generation) $('field3d-note').textContent = error.message;
  }
}

function stop() {
  page.playing = false; cancelAnimationFrame(page.frame);
  $('play').textContent = '▶'; $('play').setAttribute('aria-label', 'Play');
}

$('play').onclick = () => {
  if (page.playing) { stop(); return; }
  if (page.time >= page.run.endSec) setTime(0);
  page.playing = true; $('play').textContent = 'Ⅱ'; $('play').setAttribute('aria-label', 'Pause');
  let last = performance.now();
  const tick = now => {
    if (!page.playing) return;
    setTime(page.time + (now - last) / 1000 * Number($('speed').value)); last = now;
    if (page.time >= page.run.endSec) stop(); else page.frame = requestAnimationFrame(tick);
  };
  page.frame = requestAnimationFrame(tick);
};
$('scrub').oninput = () => { stop(); setTime(Number($('scrub').value)); };
$('run').onchange = () => { stop(); load($('run').value); };
$('elements').onchange = () => { world.elements.visible = $('elements').checked; };
for (const button of document.querySelectorAll('[data-view]')) button.onclick = () => setView(button.dataset.view);

// Following the MaxScope tab: it posts the run and time on every seek.
const channel = new BroadcastChannel('maxscope');
channel.onmessage = ({data}) => {
  if (!$('sync').checked || data?.type !== 'time') return;
  stop();
  if (data.runId !== page.run?.id) load(data.runId, data.time); else setTime(data.time);
};

new ResizeObserver(() => {
  const {clientWidth: width, clientHeight: height} = $('stage');
  renderer.setSize(width, height); camera.aspect = width / Math.max(1, height); camera.updateProjectionMatrix();
}).observe($('stage'));

renderer.setAnimationLoop(() => {
  if (page.run && page.view === 'chase') updateScene();
  controls.update();
  renderer.render(scene, camera);
});

// The run from the link MaxScope opened (#run=…&t=…), or the newest one.
const logs = await (await fetch('/api/logs')).json();
$('run').replaceChildren(...logs.map(log => new Option(`${runLabel(log.name)} · ${runDate(log)}`, log.id)));
const hash = new URLSearchParams(location.hash.slice(1));
const first = hash.get('run') || logs.find(log => !log.short)?.id || logs[0]?.id;
setView('audience');
if (first) await load(first, hash.has('t') ? Number(hash.get('t')) : null);
else $('field3d-note').textContent = 'No local logs yet.';
