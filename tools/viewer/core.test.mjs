import test from 'node:test';
import assert from 'node:assert/strict';
import {fieldPoint, sampleAt, plotPoints, plotBounds, scalarSeries, channelOptions, channelTree, discreteOptions, discreteSeries, discreteIntervals} from './core.mjs';

test('Pedro field corners retain their axes with only screen Y inverted', () => {
  assert.deepEqual(fieldPoint([0, 0, 0], 141.5, 20, 10, 283), [20, 293]);
  assert.deepEqual(fieldPoint([141.5, 141.5, 0], 141.5, 20, 10, 283), [303, 10]);
  const [x, y] = fieldPoint([8, 56, Math.PI / 2], 141.5, 20, 10, 283);
  assert.ok(Math.abs(x - 36) < 1e-10 && Math.abs(y - 181) < 1e-10);
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
