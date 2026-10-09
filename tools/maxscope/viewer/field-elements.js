import {sampleAt} from './core.mjs';

// BIOBUZZ (2026-27) field elements in Pedro's frame: origin at the audience-left corner, +X along
// the audience wall, +Y away from the audience, inches. The red ALLIANCE AREA is on the left wall
// (x = 0), blue on the right. Tiles are columns A-F along +X and rows 1-6 along +Y.
//
// Source: 2026-2027 FIRST Tech Challenge Competition Manual, BIOBUZZ, TU04, Section 9 (figure
// numbers below), and the Event Field Setup Guide V1.0 §9-10. The manual's illustrations are
// nominal with a ±1 in. tolerance; where it gives no number, positions come from its drawings.
//  - LOADING ZONE (§9.3, Figure 9-3): 11 in. deep along the alliance wall, one tile wide; red on
//    A5, blue on F2 (Figures 9-2, 9-5).
//  - GARDEN (§9.3, Figure 9-3): one tile long, 2 in. deep (two 1 in. tapes) along the wall; red on
//    A1 along the audience wall, blue on F6 along the far wall.
//  - HIVE Structure (§9.6): centred on the FIELD. The frame is 49.46 in. wide (along X) and
//    38.95 in. deep at its base (along Y), Figure 9-8; HIVE pivots 25.5 in. apart centre to centre
//    (Figure 9-10), red on the audience's left (Figure 9-17).
//  - CELLS (§9.6.2, Figures 9-9 to 9-11): 20 in. wide openings, 12.04 in. deep, 18.84 in. apart
//    along the arm, the HIVE tilted 30° with one CELL up.
//  - FLOWERS (§9.7, Figure 9-12; Setup Guide §10.1): four, on the perimeter where wall panels
//    join, at tile seams: far wall at X = 2 tiles, audience wall X = 4 tiles, red wall Y = 2 tiles,
//    blue wall Y = 4 tiles (Figure 9-17). A 4.0 in. opening on a ~4.8 in. base ring.
const CELL_WIDTH = 20, CELL_DEPTH = 12.04, CELL_GAP = 18.84, HIVE_TILT = 30 * Math.PI / 180;
const HIVE_SPACING = 25.5, FRAME_WIDTH = 49.46, FRAME_DEPTH = 38.95;
const LOADING_DEPTH = 11, GARDEN_DEPTH = 2;
const FLOWER_RING = 2.4, FLOWER_OPENING = 2.0, FLOWER_FROM_WALL = 2.6;

// Which CELL of each HIVE is up: from `Hive/<RED|BLUE>/state` when the log has it, otherwise the
// match setup (Figure 10-2: red's audience CELL and blue's far CELL raised).
const SETUP = {RED: 'AUDIENCE_RAISED', BLUE: 'FAR_RAISED'};
export const HIVE_STATE_CHANNELS = ['Hive/RED/state', 'Hive/BLUE/state'];

export function hiveStates(playback, time) {
  return Object.fromEntries(['RED', 'BLUE'].map(alliance => {
    const logged = sampleAt(playback[`Hive/${alliance}/state`], time);
    return [alliance, {state: logged === 'AUDIENCE_RAISED' || logged === 'FAR_RAISED' ? logged : SETUP[alliance], logged: typeof logged === 'string'}];
  }));
}

// Draws the elements under the robot paths. `at` maps field inches to canvas pixels and `scale`
// is pixels per inch.
export function drawBiobuzz(ctx, {length, at, scale, colors, hives}) {
  const tile = length / 6, centre = length / 2;
  const red = colors['alliance-red'], blue = colors['alliance-blue'];
  const rect = (x0, y0, x1, y1) => {
    const [ax, ay] = at([x0, y1]), [bx, by] = at([x1, y0]);
    return [ax, ay, bx - ax, by - ay];
  };
  const line = (points, width, color, dash = []) => {
    ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash); ctx.beginPath();
    points.forEach((point, i) => { const [x, y] = at(point); if (i) ctx.lineTo(x, y); else ctx.moveTo(x, y); });
    ctx.stroke(); ctx.setLineDash([]);
  };
  const tape = Math.max(1.5, scale * 1.5);
  ctx.save();

  // Alliance walls.
  line([[0, 0], [0, length]], Math.max(3, scale * 2), red);
  line([[length, 0], [length, length]], Math.max(3, scale * 2), blue);

  // LOADING ZONES: tape on the three sides away from the wall, with a light fill.
  const zone = (x0, x1, y0, y1, color, wallX) => {
    ctx.globalAlpha = 0.14; ctx.fillStyle = color; ctx.fillRect(...rect(x0, y0, x1, y1)); ctx.globalAlpha = 1;
    const inner = wallX === 0 ? x1 : x0;
    line([[wallX, y1], [inner, y1], [inner, y0], [wallX, y0]], tape, color);
  };
  zone(0, LOADING_DEPTH, 4 * tile, 5 * tile, red, 0);
  zone(length - LOADING_DEPTH, length, tile, 2 * tile, blue, length);

  // GARDENS: a strip along the wall, the outside edge of the tape.
  ctx.fillStyle = red; ctx.fillRect(...rect(0, 0, tile, GARDEN_DEPTH));
  ctx.fillStyle = blue; ctx.fillRect(...rect(length - tile, length - GARDEN_DEPTH, length, length));

  // HIVE frame: the two A-frame feet across Y and the under-tile strips joining them.
  const fx0 = centre - FRAME_WIDTH / 2, fx1 = centre + FRAME_WIDTH / 2, fy0 = centre - FRAME_DEPTH / 2, fy1 = centre + FRAME_DEPTH / 2;
  line([[fx0, fy0], [fx1, fy0]], 1, colors['frame'], [3, 3]);
  line([[fx0, fy1], [fx1, fy1]], 1, colors['frame'], [3, 3]);
  line([[fx0, fy0], [fx0, fy1]], Math.max(2, scale * 1.2), colors['frame']);
  line([[fx1, fy0], [fx1, fy1]], Math.max(2, scale * 1.2), colors['frame']);
  line([[fx0, centre], [fx1, centre]], Math.max(1.5, scale * 0.8), colors['frame']);

  // HIVES: two CELLS each, seen from above with the arm tilted 30°: the raised CELL is drawn solid.
  const reach = (CELL_GAP / 2 + CELL_DEPTH / 2) * Math.cos(HIVE_TILT), depth = CELL_DEPTH * Math.cos(HIVE_TILT);
  for (const [alliance, color, x] of [['RED', red, centre - HIVE_SPACING / 2], ['BLUE', blue, centre + HIVE_SPACING / 2]]) {
    const raised = hives[alliance].state;
    for (const [side, y] of [['AUDIENCE', centre - reach], ['FAR', centre + reach]]) {
      const up = raised === `${side}_RAISED`;
      const box = rect(x - CELL_WIDTH / 2, y - depth / 2, x + CELL_WIDTH / 2, y + depth / 2);
      ctx.globalAlpha = up ? 0.45 : 0.14; ctx.fillStyle = color; ctx.fillRect(...box); ctx.globalAlpha = 1;
      ctx.strokeStyle = color; ctx.lineWidth = up ? 2 : 1.2; ctx.setLineDash(up || hives[alliance].logged ? [] : [4, 3]);
      ctx.strokeRect(...box); ctx.setLineDash([]);
    }
    line([[x, centre - reach + depth / 2], [x, centre + reach - depth / 2]], Math.max(1.5, scale * 0.9), color);
    const [px, py] = at([x, centre]);
    ctx.fillStyle = color; ctx.beginPath(); ctx.arc(px, py, Math.max(2.5, scale * 1.4), 0, Math.PI * 2); ctx.fill();
  }

  // FLOWERS: base ring and top opening, against the wall.
  const flowers = [[2 * tile, length - FLOWER_FROM_WALL], [4 * tile, FLOWER_FROM_WALL], [FLOWER_FROM_WALL, 2 * tile], [length - FLOWER_FROM_WALL, 4 * tile]];
  for (const point of flowers) {
    const [x, y] = at(point);
    ctx.fillStyle = colors['flower']; ctx.beginPath(); ctx.arc(x, y, Math.max(4, FLOWER_RING * scale), 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = colors['flower-core']; ctx.beginPath(); ctx.arc(x, y, Math.max(2, FLOWER_OPENING * scale), 0, Math.PI * 2); ctx.fill();
  }
  ctx.restore();
}

// Tile names just inside the field: A-F along the audience wall, 1-6 along the red wall.
export function drawTileNames(ctx, {length, at, colors}) {
  const tile = length / 6, inset = tile * 0.16;
  ctx.save();
  ctx.font = '600 9px Inter, system-ui, sans-serif'; ctx.fillStyle = colors['field-label']; ctx.globalAlpha = 0.8;
  ctx.textBaseline = 'middle'; ctx.textAlign = 'center';
  for (let i = 0; i < 6; i++) {
    ctx.fillText('ABCDEF'[i], ...at([(i + 0.5) * tile, inset]));
    ctx.fillText(String(i + 1), ...at([inset, (i + 0.5) * tile]));
  }
  ctx.restore();
}
