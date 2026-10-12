import test from 'node:test';
import assert from 'node:assert/strict';
import {activeCommandSpans, fieldPoint, fieldView, ballSighting, ballEstimate, cameraDetections, loggedCameraMount, cameraSources, sampleAt, plotPoints, plotBounds, extremes, scalarSeries, channelOptions, channelTree, discreteOptions, discreteSeries, discreteIntervals, alignmentAnchor, shiftSeries, adjacentProblem, problemsAt, matchPhases, phaseAt, posePasses, nextPass, followWindow} from './core.mjs';

test('Pedro field corners retain their axes with only screen Y inverted', () => {
  const view = {minX: 0, minY: 0, span: 141.5};
  assert.deepEqual(fieldPoint([0, 0, 0], view, 20, 10, 283), [20, 293]);
  assert.deepEqual(fieldPoint([141.5, 141.5, 0], view, 20, 10, 283), [303, 10]);
  const [x, y] = fieldPoint([8, 56, Math.PI / 2], view, 20, 10, 283);
  assert.ok(Math.abs(x - 36) < 1e-10 && Math.abs(y - 181) < 1e-10);
});

test('field view stays on the field when the whole robot does', () => {
  const view = fieldView([[0, [8, 56, 0]], [1, [133.5, 8, 0]], [2, [NaN, 900, 0]]], 141.5);
  assert.deepEqual(view, {minX: 0, minY: 0, span: 141.5, tile: 141.5 / 6, tiles: 6});
  assert.deepEqual(fieldView([], 141.5).tiles, 6);
});

test('field view grows to show a robot that is partly over a wall', () => {
  assert.equal(fieldView([[0, [8, 72, 0]]], 144).tiles, 6);
  assert.equal(fieldView([[0, [8.2, 72, 0]]], 144).minX, 0);
  assert.equal(fieldView([[0, [8, 72, Math.PI / 4]]], 144).minX, -24);
  assert.equal(fieldView([[0, [2, 72, 0]]], 144).minX, -24);
  assert.equal(fieldView([[0, [72, 140, NaN]]], 144).tiles, 7);
});

test('field view grows by whole tiles to hold an off-field path and stays square', () => {
  const tile = 144 / 6;
  const view = fieldView([[0, [10, 10, 0]], [1, [-30, 10, 0]], [2, [20, 160, 0]]], 144);
  assert.equal(view.minX, -2 * tile);
  assert.ok(view.minY <= 0 && view.minY + view.span >= 168);
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

test('extremes cover only the window, skip gaps, and keep the earliest tie', () => {
  const series = [[0, 99], [1, 2], [2, null], [3, 7], [4, -1], [5, 7], [6, -50]];
  assert.deepEqual(extremes(series, 0.5, 5.5), {min: [4, -1], max: [3, 7]});
  assert.equal(extremes(series, 2, 2.5), null);
  assert.equal(extremes([], 0, 1), null);
});

test('velocity gets a derived speed channel in in/s', () => {
  const options = channelOptions([{name: 'velocity', type: 'double[]', components: 3}]);
  const speed = options.find(option => option.key === 'velocity::speed');
  assert.equal(speed.label, 'velocity / Speed');
  assert.equal(speed.unit, 'in/s');
  assert.deepEqual(scalarSeries([[0, [3, -4, 1]], [1, [3, null, 0]]], speed.component), [[0, 5], [1, null]]);
});

test('drive wheel arrays are labelled by wheel, velocity in ticks/s', () => {
  const options = channelOptions([
    {name: 'Drive/motors/power', type: 'double[]', components: 4},
    {name: 'Drive/motors/velocityTps', type: 'double[]', components: 4},
  ]);
  assert.deepEqual(options.slice(0, 4).map(option => option.componentLabel), ['leftFront', 'rightFront', 'leftRear', 'rightRear']);
  assert.equal(options[0].unit, '');
  assert.equal(options[5].label, 'Drive/motors/velocityTps / rightFront');
  assert.equal(options[5].unit, 'ticks/s');
});

test('traction channels read in in/s, turn in rad/s', () => {
  const options = channelOptions([
    {name: 'MaxScope/traction/wheels', type: 'double[]', components: 3},
    {name: 'MaxScope/traction/mismatchInPerS', type: 'double'},
  ]);
  assert.deepEqual(options.slice(0, 3).map(option => [option.componentLabel, option.unit]),
    [['Forward', 'in/s'], ['Left', 'in/s'], ['Turn', 'rad/s']]);
  assert.equal(options.find(option => option.name === 'MaxScope/traction/mismatchInPerS').unit, 'in/s');
});

test('battery and hub channels read in volts and amps', () => {
  const options = channelOptions([
    {name: 'MaxScope/battery/restingVolts', type: 'double'},
    {name: 'MaxScope/battery/currentAmps', type: 'double'},
    {name: 'hubs/control/volts', type: 'double'},
    {name: 'hubs/control/currentAmps', type: 'double'},
  ]);
  assert.deepEqual(options.map(option => option.unit), ['V', 'A', 'V', 'A']);
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
  assert.equal(button.componentLabel, 'Circle');
  assert.deepEqual(discreteSeries([[0, 0], [1, 3], [2, null]], button.bit), [[0, false], [1, true], [2, null]]);
  assert.deepEqual(discreteSeries([[0, ''], [1, false], [2, NaN]]), [[0, ''], [1, false], [2, null]]);
  assert.equal(discreteOptions([{name: 'commands/running', type: 'string'}])[0].source, 'commands/running');
  assert.equal(discreteOptions([])[0].source, undefined);
});

test('ball sighting prefers the assist target and ignores stale or missing angles', () => {
  const playback = {'BallAssist/tx': [[1, 10], [2, null]], 'BallAssist/ty': [[1, -3]], 'BallAssist/targetTy': [[0, 2]],
    'BallCamera/target/horizontalDeg': [[1.9, -5]]};
  assert.deepEqual(ballSighting(playback, 1.1, ['BallCamera']), {source: 'BallAssist', camera: null, mount: 'limelight', txDeg: 10, tyDeg: -3, targetTyDeg: 2});
  assert.equal(ballSighting(playback, 1.5, ['BallCamera']), null);
  assert.deepEqual(ballSighting(playback, 2.0, ['BallCamera']), {source: 'BallCamera', camera: 'BallCamera', mount: 'logged', txDeg: -5, tyDeg: null, targetTyDeg: null});
  assert.equal(ballSighting({}, 1, ['BallCamera']), null);
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
  const playback = {'IntakeCam/target/horizontalDeg': [[1, 3]], 'IntakeCam/target/verticalDeg': [[1, 8]]};
  assert.equal(ballSighting(playback, 1, ['IntakeCam']).tyDeg, -8);
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
  assert.equal(loggedCameraMount(playback, 1, 'BallCamera'), null);
  playback['BallCamera/mount/measured'] = [[0, true]];
  assert.deepEqual(loggedCameraMount(playback, 1, 'BallCamera'), {heightIn: 6, pitchDownDeg: 15, forwardIn: 7, leftIn: 0, yawDeg: 0});
});

test('camera detections pair the logged columns and drop stale frames', () => {
  const playback = {
    'BallCamera/candidates/xPx': [[1, [400, 50]], [2, []]], 'BallCamera/candidates/yPx': [[1, [300, 60]], [2, []]],
    'BallCamera/candidates/radiusPx': [[1, [12, 1]], [2, []]], 'BallCamera/candidates/horizontalDeg': [[1, [5, null]], [2, []]],
    'BallCamera/candidates/verticalDeg': [[1, [4, null]], [2, []]], 'BallCamera/candidates/rejections': [[1, 'accepted,small'], [2, '']],
    'BallCamera/candidates/selectedIndex': [[1, 0], [2, -1]], 'BallCamera/frame/widthPx': [[0, 640]], 'BallCamera/frame/heightPx': [[0, 480]],
  };
  const frame = cameraDetections(playback, 1.1, 'BallCamera');
  assert.equal(frame.widthPx, 640);
  assert.deepEqual(frame.detections, [
    {xPx: 400, yPx: 300, radiusPx: 12, txDeg: 5, tyDeg: -4, rejection: null, selected: true},
    {xPx: 50, yPx: 60, radiusPx: 1, txDeg: null, tyDeg: null, rejection: 'small', selected: false},
  ]);
  assert.deepEqual(cameraDetections(playback, 2.1, 'BallCamera').detections, []);
  assert.deepEqual(cameraDetections(playback, 1.5, 'BallCamera').detections, []);
  assert.equal(cameraDetections({}, 1, 'BallCamera'), null);
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

test('cameras are discovered by channel prefix, whatever a subsystem is called', () => {
  assert.deepEqual(cameraSources([{name: 'BallCamera/candidates/xPx'}, {name: 'BallCamera/target/horizontalDeg'},
    {name: 'IntakeCam/target/horizontalDeg'}, {name: 'pose'}, {name: 'Limelight/target/txDegrees'}]), ['BallCamera', 'IntakeCam']);
  assert.deepEqual(cameraSources([]), []);
});

test('the sighting names the camera it came from so its own mount is used', () => {
  const playback = {'FrontCam/target/horizontalDeg': [[1, 2]], 'RearCam/target/horizontalDeg': [[1, -9]]};
  assert.equal(ballSighting(playback, 1, ['RearCam', 'FrontCam']).camera, 'RearCam');
  assert.equal(ballSighting(playback, 1, ['FrontCam']).camera, 'FrontCam');
});

test('sampled active sets become one span per command name', () => {
  const series = [[0, ''], [1, '#1 Drive teleop'], [2, '#1 Drive teleop\n#2 Ball aim (suspended)'], [3, '#2 Ball aim'], [4, '']];
  assert.deepEqual(activeCommandSpans(series, 6), [
    {name: 'Drive teleop', startSec: 1, endSec: 3, outcome: 'SAMPLED'},
    {name: 'Ball aim', startSec: 2, endSec: 4, outcome: 'SAMPLED'},
  ]);
  assert.deepEqual(activeCommandSpans([[0, '#1 Held']], 9), [{name: 'Held', startSec: 0, endSec: 9, outcome: 'SAMPLED'}]);
  assert.deepEqual(activeCommandSpans(), []);
});

test('graph padding never invents a negative loop time', () => {
  assert.deepEqual(plotBounds([{axis: 0, visible: true, points: [[0, 0], [1, 50]]}], 0, 1), [0, 56]);
  assert.deepEqual(plotBounds([{axis: 0, visible: true, points: [[0, -50], [1, 0]]}], 0, 1), [-56, 0]);
});

test('compared runs line up at op-mode start, first movement, or log start', () => {
  const events = [{tSec: 0.4, text: 'init X'}, {tSec: 1.2, text: 'start'}];
  const poses = [[0.5, [NaN, 0, 0]], [1, [10, 10, 0]], [2, [10.5, 10, 0]], [3, [12, 10, 0]]];
  assert.equal(alignmentAnchor(events, poses, 'start'), 1.2);
  assert.equal(alignmentAnchor([], poses, 'start'), 0);
  assert.equal(alignmentAnchor(events, poses, 'motion'), 3);
  assert.equal(alignmentAnchor(events, poses, 'log'), 0);
  assert.deepEqual(shiftSeries([[1, 'a'], [2, 'b']], 0.5), [[1.5, 'a'], [2.5, 'b']]);
});

test('problem stepping and the problems under the cursor', () => {
  const problems = [{startSec: 2, endSec: 4}, {startSec: 6, endSec: 6}, {startSec: 9, endSec: 9.5}];
  assert.equal(adjacentProblem(problems, 0, 1), problems[0]);
  assert.equal(adjacentProblem(problems, 2, 1), problems[1]);
  assert.equal(adjacentProblem(problems, 6, -1), problems[0]);
  assert.equal(adjacentProblem(problems, 2, -1), null);
  assert.deepEqual(problemsAt(problems, 3), [problems[0]]);
  assert.deepEqual(problemsAt(problems, 6.05), [problems[1]]);
  assert.deepEqual(problemsAt(problems, 5), []);
});

test('match clock follows BIOBUZZ milestones and counts down to the end of teleop', () => {
  const teleop = matchPhases({kind: 'teleop', startSec: 2, autoSec: 30, driverSec: 120, flowerUnlockSec: 60, finalTwentySec: 100});
  assert.deepEqual(teleop.marks.map(mark => [mark.tSec, mark.label]),
    [[2, 'start'], [62, 'FLOWER ownership unlocked'], [102, 'Final 20 seconds'], [122, '2:00']]);
  assert.deepEqual(phaseAt(teleop, 1), {name: 'INIT', clock: ''});
  assert.deepEqual(phaseAt(teleop, 2), {name: 'TELEOP', clock: '2:00 left'});
  assert.deepEqual(phaseAt(teleop, 2.5), {name: 'TELEOP', clock: '2:00 left'});
  assert.deepEqual(phaseAt(teleop, 61.999), {name: 'TELEOP', clock: '1:01 left'});
  assert.deepEqual(phaseAt(teleop, 62), {name: 'FLOWER OWNERSHIP UNLOCKED', clock: '1:00 left'});
  assert.deepEqual(phaseAt(teleop, 92), {name: 'FLOWER OWNERSHIP UNLOCKED', clock: '0:30 left'});
  assert.deepEqual(phaseAt(teleop, 101.999), {name: 'FLOWER OWNERSHIP UNLOCKED', clock: '0:21 left'});
  assert.deepEqual(phaseAt(teleop, 102), {name: 'FINAL 20 SECONDS', clock: '0:20 left'});
  assert.deepEqual(phaseAt(teleop, 121.999), {name: 'FINAL 20 SECONDS', clock: '0:01 left'});
  assert.deepEqual(phaseAt(teleop, 122), {name: 'AFTER MATCH', clock: '+0:00'});
  assert.deepEqual(phaseAt(teleop, 135), {name: 'AFTER MATCH', clock: '+0:13'});
});

test('autonomous and unstarted runs have no teleop milestones', () => {
  const auto = matchPhases({kind: 'auto', startSec: 1, autoSec: 30, driverSec: 120, flowerUnlockSec: 60, finalTwentySec: 100});
  assert.deepEqual(auto.marks.map(mark => [mark.tSec, mark.label]), [[1, 'start'], [31, '0:30']]);
  assert.deepEqual(phaseAt(auto, 1), {name: 'AUTO', clock: '0:30 left'});
  assert.deepEqual(phaseAt(auto, 20), {name: 'AUTO', clock: '0:11 left'});
  assert.deepEqual(phaseAt(auto, 31), {name: 'AFTER AUTO', clock: '+0:00'});
  assert.deepEqual(phaseAt(auto, 40), {name: 'AFTER AUTO', clock: '+0:09'});
  assert.equal(matchPhases({kind: 'teleop', startSec: null}), null);
  assert.equal(matchPhases(null), null);
});

test('clicking the field finds every pass near the point and cycles through them', () => {
  const series = [[0, [0, 0, 0]], [1, [10, 0, 0]], [2, [11, 0, 0]], [3, [30, 0, 0]], [4, [10.5, 0.2, 0]], [5, [NaN, 0, 0]], [6, [9, 0, 0]]];
  const passes = posePasses(series, [10.4, 0], 2);
  assert.deepEqual(passes, [{startSec: 1, endSec: 2, atSec: 1}, {startSec: 4, endSec: 4, atSec: 4}, {startSec: 6, endSec: 6, atSec: 6}]);
  assert.equal(nextPass(passes, 0), passes[0]);
  assert.equal(nextPass(passes, 1), passes[1]);
  assert.equal(nextPass(passes, 6), passes[0]);
  assert.equal(nextPass([], 0), null);
});

test('a following graph window keeps its length and the cursor two thirds across, within the run', () => {
  assert.deepEqual(followWindow(100, 9, 300), [94, 103]);
  assert.deepEqual(followWindow(2, 9, 300), [0, 9]);
  assert.deepEqual(followWindow(299, 9, 300), [291, 300]);
  assert.deepEqual(followWindow(5, 9, 6), [0, 6]);
  assert.deepEqual(followWindow(150, Infinity, 300), [0, 300]);
  assert.deepEqual(followWindow(0, 10, 0), [0, 0.001]);
});
