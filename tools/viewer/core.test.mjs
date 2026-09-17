import test from 'node:test';
import assert from 'node:assert/strict';
import {fieldPoint, fieldView, ballSighting, ballEstimate, cameraDetections, loggedCameraMount, sampleAt, plotPoints, plotBounds, scalarSeries, channelOptions, channelTree, discreteOptions, discreteSeries, discreteIntervals} from './core.mjs';

test('Pedro field corners retain their axes with only screen Y inverted', () => {
  const view = {minX: 0, minY: 0, span: 141.5};
  assert.deepEqual(fieldPoint([0, 0, 0], view, 20, 10, 283), [20, 293]);
  assert.deepEqual(fieldPoint([141.5, 141.5, 0], view, 20, 10, 283), [303, 10]);
  const [x, y] = fieldPoint([8, 56, Math.PI / 2], view, 20, 10, 283);
  assert.ok(Math.abs(x - 36) < 1e-10 && Math.abs(y - 181) < 1e-10);
});

test('field view stays on the field when the path does', () => {
  const view = fieldView([[0, [8, 56, 0]], [1, [141.5, 0, 0]], [2, [NaN, 900, 0]]], 141.5);
  assert.deepEqual(view, {minX: 0, minY: 0, span: 141.5, tile: 141.5 / 6, tiles: 6, slack: 141.5 / 24});
  assert.deepEqual(fieldView([], 141.5).tiles, 6);
});

test('field view ignores a pose just past a wall', () => {
  assert.equal(fieldView([[0, [-5, 72, 0]], [1, [72, 149, 0]]], 144).tiles, 6);
  assert.equal(fieldView([[0, [-7, 72, 0]]], 144).minX, -24);
});

test('field view grows by whole tiles to hold an off-field path and stays square', () => {
  const tile = 144 / 6;
  const view = fieldView([[0, [0, 0, 0]], [1, [-30, 10, 0]], [2, [20, 160, 0]]], 144);
  assert.equal(view.minX, -tile);
  assert.equal(view.tiles, 7);
  assert.ok(view.minY <= 0 && view.minY + view.span >= 160);
  assert.equal(view.span, view.tiles * tile);
});

test('field view keeps the farthest real pose and caps growth at one field per side', () => {
  const tile = 144 / 6;
  assert.equal(fieldView([[0, [72, -103, 0]]], 144).minY, -5 * tile);
  const far = fieldView([[0, [-1000, 72, 0]], [1, [1000, 72, 0]]], 144);
  assert.deepEqual([far.minX, far.span], [-144, 432]);
});

test('cursor uses latest known state, never a future sample or interpolated command', () => {
  const series = [[1.000001, 'START'], [1.000002, 'FINISH'], [2, null]];
  assert.equal(sampleAt(series, 1), null);
  assert.equal(sampleAt(series, 1.000001), 'START');
  assert.equal(sampleAt(series, 1.000002), 'FINISH');
  assert.equal(sampleAt(series, 1.5), 'FINISH');
  assert.equal(sampleAt(series, 2.5), null);
});

test('plot reduction preserves short spikes, troughs, and invalid-data gaps', () => {
  const series = Array.from({length: 10000}, (_, i) => [i / 100, 0]);
  series[3333][1] = 100;
  series[3334][1] = -80;
  series[3335][1] = null;
  const points = plotPoints(series, 0, 100, 100);
  assert.ok(points.some(([t, v]) => t === 33.33 && v === 100));
  assert.ok(points.some(([t, v]) => t === 33.34 && v === -80));
  assert.ok(points.some(([t, v]) => t === 33.35 && v === null));
  assert.ok(points.length < 500);
});

test('plots retain neighboring samples at zoom boundaries', () => {
  assert.deepEqual(plotPoints([[0, 1], [1, 2], [2, 3], [3, 4]], 1.2, 1.8), [[1, 2], [2, 3]]);
});

test('overlays share bounds per unit while ignoring hidden layers and future values', () => {
  const layers = [
    {axis: 0, visible: true, points: [[0, 10], [1, 12], [3, 900]]},
    {axis: 0, visible: true, points: [[0, 8], [1, null]]},
    {axis: 0, visible: false, points: [[0, -1000]]},
    {axis: 1, visible: true, points: [[0, 100], [1, 200]]},
  ];
  assert.deepEqual(plotBounds(layers, 0, 2), [7.52, 12.48]);
  assert.deepEqual(plotBounds(layers, 1, 2), [88, 212]);
  const [min, max] = plotBounds([{axis: 0, visible: true, points: [[0, 0]]}], 0, 1);
  assert.ok(min < 0 && max > 0);
  assert.ok(plotBounds([], 0, 1).every(Number.isFinite));
});

test('native heading stays radians and loop graphs explicitly convert ns to ms', () => {
  const options = channelOptions([{name: 'pose', type: 'double[]', components: 3},
    {name: 'loop/totalNanos', type: 'int64'}, {name: 'events', type: 'string'}]);
  assert.equal(options.length, 4);
  assert.equal(options[2].unit, 'rad');
  assert.equal(options[2].scale, 1);
  assert.equal(options[3].unit, 'ms');
  assert.deepEqual(scalarSeries([[0, 12000000]], null, options[3].scale), [[0, 12]]);
  assert.deepEqual(scalarSeries([[0, [3, null, NaN]]], 1), [[0, null]]);
});

test('channel folders retain complete paths, array components, and prefix collisions', () => {
  const options = channelOptions([
    {name: 'Drive/motors/leftFront/power', type: 'double'},
    {name: 'Drive/motors/rightFront/power', type: 'double'},
    {name: 'Drive/motors', type: 'double'},
    {name: 'pose', type: 'double[]', components: 3},
    {name: 'battery', type: 'double'},
  ]);
  const tree = channelTree(options);
  const motors = tree.children.get('Drive').children.get('motors');
  assert.equal(motors.options[0].key, 'Drive/motors::');
  assert.equal(motors.children.get('leftFront').children.get('power').options[0].key, 'Drive/motors/leftFront/power::');
  assert.equal(motors.children.get('rightFront').children.get('power').options[0].key, 'Drive/motors/rightFront/power::');
  assert.equal(tree.children.get('pose').children.get('Heading').options[0].key, 'pose::2');
  assert.equal(tree.children.get('battery').options[0].unit, 'V');
});

test('discrete states preserve transitions, gaps, false, and simultaneous timestamp updates', () => {
  const series = [[1, 'intake'], [2, 'intake'], [2, 'shoot'], [3, 'shoot'], [4, null], [5, false], [6, '']];
  assert.deepEqual(discreteIntervals(series, 7), [
    [1, {end: 2, value: 'intake'}], [2, {end: 4, value: 'shoot'}],
    [5, {end: 6, value: false}], [6, {end: 7, value: ''}],
  ]);
  assert.equal(sampleAt(series, .5), null);
  assert.equal(sampleAt(series, 2), 'shoot');
  assert.deepEqual(discreteIntervals([[1, 'A'], [2, 'B'], [2, 'A'], [3, 'C']], 2.5), [[1, {end: 2.5, value: 'A'}]]);
  assert.deepEqual(discreteIntervals([[1.000001, 'A'], [1.000002, 'B']], 2)[0], [1.000001, {end: 1.000002, value: 'A'}]);
});

test('discrete picker includes command state and individual gamepad buttons, not held event messages', () => {
  const options = discreteOptions([
    {name: 'commands/active', type: 'string'}, {name: 'commands/running', type: 'string'},
    {name: 'commands/events', type: 'string'}, {name: 'events', type: 'string'},
    {name: 'driveMode', type: 'string'}, {name: 'gamepad1/buttons', type: 'int64'},
  ]);
  assert.equal(options[0].key, '@commands');
  assert.equal(options[0].source, 'commands/active');
  assert.equal(options.filter(option => option.name === 'events' || option.name === 'commands/events').length, 0);
  const button = options.find(option => option.key === 'gamepad1/buttons::1');
  assert.equal(button.componentLabel, 'B');
  assert.deepEqual(discreteSeries([[0, 0], [1, 3], [2, null]], button.bit), [[0, false], [1, true], [2, null]]);
  assert.deepEqual(discreteSeries([[0, ''], [1, false], [2, NaN]]), [[0, ''], [1, false], [2, null]]);
  assert.equal(discreteOptions([{name: 'commands/running', type: 'string'}])[0].source, 'commands/running');
  assert.equal(discreteOptions([])[0].source, undefined);
});

test('ball sighting prefers the assist target and ignores stale or missing angles', () => {
  const playback = {'BallAssist/tx': [[1, 10], [2, null]], 'BallAssist/ty': [[1, -3]], 'BallAssist/targetTy': [[0, 2]],
    'BallCamera/target/horizontalDeg': [[1.9, -5]]};
  assert.deepEqual(ballSighting(playback, 1.1), {source: 'BallAssist', mount: 'limelight', txDeg: 10, tyDeg: -3, targetTyDeg: 2});
  assert.equal(ballSighting(playback, 1.5), null);
  assert.deepEqual(ballSighting(playback, 2.0), {source: 'BallCamera', mount: 'logged', txDeg: -5, tyDeg: null, targetTyDeg: null});
  assert.equal(ballSighting({}, 1), null);
});

test('assist ty falls back to the Limelight only when it is the same target', () => {
  const playback = {'BallAssist/tx': [[1, 6.2]], 'Limelight/target/visible': [[1, true]], 'Limelight/target/txDegrees': [[1, 6.3]], 'Limelight/target/tyDegrees': [[1, -4]]};
  assert.equal(ballSighting(playback, 1).tyDeg, -4);
  assert.equal(ballSighting({...playback, 'Limelight/target/visible': [[1, false]]}, 1).tyDeg, null);
  playback['Limelight/target/txDegrees'] = [[1, -12]];
  assert.equal(ballSighting(playback, 1).tyDeg, null);
});

test('ball estimate projects the camera ray onto the ball-centre plane', () => {
  const mount = {heightIn: 4.36, pitchDownDeg: 10}, above = 4.36 - 1.4;
  const ahead = ballEstimate([0, 0, 0], {txDeg: 0, tyDeg: -10}, mount);
  assert.ok(Math.abs(ahead.distanceIn - above / Math.tan(20 * Math.PI / 180)) < 1e-9);
  assert.ok(Math.abs(ahead.point[0] - ahead.distanceIn) < 1e-9 && Math.abs(ahead.point[1]) < 1e-9);
  const right = ballEstimate([10, 20, Math.PI / 2], {txDeg: 20, tyDeg: -10}, mount);
  assert.ok(right.bearing < Math.PI / 2 && right.point[0] > 10 && right.point[1] > 20);
  assert.equal(ballEstimate([0, 0, 0], {txDeg: 0, tyDeg: 10}, mount).distanceIn, null);
  const noRange = ballEstimate([0, 0, 1], {txDeg: 45, tyDeg: null}, mount);
  assert.ok(Math.abs(noRange.bearing - (1 - Math.PI / 4)) < 1e-12 && noRange.point === null);
});

test('USB camera vertical angles are flipped to positive up', () => {
  const playback = {'BallCamera/target/horizontalDeg': [[1, 3]], 'BallCamera/target/verticalDeg': [[1, 8]]};
  assert.equal(ballSighting(playback, 1).tyDeg, -8);
});

test('ball estimate starts at the mounted camera and turns with its yaw', () => {
  const mount = {heightIn: 4.36, pitchDownDeg: 10, forwardIn: 8, leftIn: 2, yawDeg: 90};
  const estimate = ballEstimate([10, 10, 0], {txDeg: 0, tyDeg: -10}, mount);
  assert.deepEqual(estimate.origin, [18, 12]);
  assert.ok(Math.abs(estimate.bearing - Math.PI / 2) < 1e-12);
  assert.ok(Math.abs(estimate.point[0] - 18) < 1e-9 && estimate.point[1] > 12);
});

test('logged camera mount is used only once marked measured', () => {
  const playback = {'BallCamera/mount/measured': [[0, false]], 'BallCamera/mount/heightIn': [[0, 6]], 'BallCamera/mount/pitchDownDeg': [[0, 15]],
    'BallCamera/mount/forwardIn': [[0, 7]], 'BallCamera/mount/leftIn': [[0, 0]], 'BallCamera/mount/yawDeg': [[0, 0]]};
  assert.equal(loggedCameraMount(playback, 1), null);
  playback['BallCamera/mount/measured'] = [[0, true]];
  assert.deepEqual(loggedCameraMount(playback, 1), {heightIn: 6, pitchDownDeg: 15, forwardIn: 7, leftIn: 0, yawDeg: 0});
});

test('camera detections pair the logged columns and drop stale frames', () => {
  const playback = {
    'BallCamera/candidates/xPx': [[1, [400, 50]], [2, []]], 'BallCamera/candidates/yPx': [[1, [300, 60]], [2, []]],
    'BallCamera/candidates/radiusPx': [[1, [12, 1]], [2, []]], 'BallCamera/candidates/horizontalDeg': [[1, [5, null]], [2, []]],
    'BallCamera/candidates/verticalDeg': [[1, [4, null]], [2, []]], 'BallCamera/candidates/rejections': [[1, 'accepted,small'], [2, '']],
    'BallCamera/candidates/selectedIndex': [[1, 0], [2, -1]], 'BallCamera/frame/widthPx': [[0, 640]], 'BallCamera/frame/heightPx': [[0, 480]],
  };
  const frame = cameraDetections(playback, 1.1);
  assert.equal(frame.widthPx, 640);
  assert.deepEqual(frame.detections, [
    {xPx: 400, yPx: 300, radiusPx: 12, txDeg: 5, tyDeg: -4, rejection: null, selected: true},
    {xPx: 50, yPx: 60, radiusPx: 1, txDeg: null, tyDeg: null, rejection: 'small', selected: false},
  ]);
  assert.deepEqual(cameraDetections(playback, 2.1).detections, []);
  assert.deepEqual(cameraDetections(playback, 1.5).detections, []);
  assert.equal(cameraDetections({}, 1), null);
});

test('a projected sighting recovers the ball it came from, wherever the robot stands', () => {
  const mount = {heightIn: 6, pitchDownDeg: 15, forwardIn: 7, leftIn: 2, yawDeg: -5};
  const ball = [96, 72], above = mount.heightIn - 1.4, pitch = mount.pitchDownDeg * Math.PI / 180;
  const sightingFrom = pose => {
    const heading = pose[2], axis = heading + mount.yawDeg * Math.PI / 180;
    const camera = [pose[0] + Math.cos(heading) * mount.forwardIn - Math.sin(heading) * mount.leftIn,
      pose[1] + Math.sin(heading) * mount.forwardIn + Math.cos(heading) * mount.leftIn];
    const range = Math.hypot(ball[0] - camera[0], ball[1] - camera[1]);
    const alpha = Math.atan2(ball[1] - camera[1], ball[0] - camera[0]) - axis;
    const forward = range * Math.cos(alpha), right = -range * Math.sin(alpha);
    const denominator = forward * Math.cos(pitch) + above * Math.sin(pitch);
    return {txDeg: Math.atan(right / denominator) * 180 / Math.PI,
      tyDeg: -Math.atan((above * Math.cos(pitch) - forward * Math.sin(pitch)) / denominator) * 180 / Math.PI};
  };
  for (const pose of [[40, 60, 0], [66, 67.8, -0.162], [77.5, 71.25, 0.4], [96, 40, Math.PI / 2], [120, 90, Math.PI]]) {
    const {point} = ballEstimate(pose, sightingFrom(pose), mount);
    assert.ok(Math.hypot(point[0] - ball[0], point[1] - ball[1]) < 1e-6, `pose ${pose} gave ${point}`);
  }
});
