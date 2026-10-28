// Check every arm against both feet using the existing exact bone checker.
// Its two groups are named L/R. This adapter only relabels coordinate frames:
// every original compressed vertex/index chunk and every transform is retained.
import { gunzipSync, gzipSync } from 'node:zlib';
import { createBodyCollision } from '../body_collision.mjs';

const ARM = /^(hum|fore|ulna|hand|f[1-5][mabc])_[LR]$/;
const FOOT = /^(foot|toes)_[LR]$/;
export function createArmFootCollision(A, THREE, boneData) {
  const payload = gunzipSync(Buffer.from(boneData.data, 'base64'));
  let offset = 0;
  const chunks = boneData.parts.map(part => {
    const size = 6 * (part.n + part.m), chunk = payload.subarray(offset, offset + size);
    offset += size;
    return { part, chunk };
  });
  if (offset !== payload.length) throw new Error('Arm/foot collision payload length mismatch');
  const groups = [];
  for (const armSide of ['L', 'R']) for (const footSide of ['L', 'R']) {
    const parts = [], selected = [], frames = new Map();
    for (const { part, chunk } of chunks) {
      let alias = null;
      if (ARM.test(part.seg) && part.seg.endsWith('_' + armSide))
        alias = part.seg.replace(/_[LR]$/, '_L');
      else if (FOOT.test(part.seg) && part.seg.endsWith('_' + footSide))
        alias = part.seg.startsWith('foot') ? 'hum_R' : 'fore_R';
      if (!alias) continue;
      parts.push({ ...part, seg: alias }); selected.push(chunk); frames.set(alias, part.seg);
    }
    groups.push({ armSide, footSide, frames, collider: createBodyCollision(A, THREE,
      { parts, data: gzipSync(Buffer.concat(selected)).toString('base64') }) });
  }
  return {
    scope: 'arm-foot-bones', pairGroups: groups.length,
    inspect(W, { maxPairs = 6 } = {}) {
      const pairs = [];
      let candidatePairs = 0, triangleTests = 0;
      for (const group of groups) {
        const frames = Object.fromEntries([...group.frames].map(([alias, segment]) => [alias, W[segment]]));
        const result = group.collider.inspect(frames, { maxPairs, maxTriangleIntersections: 1 });
        candidatePairs += result.candidatePairs; triangleTests += result.triangleTests;
        pairs.push(...result.pairs.map(pair => ({ ...pair,
          a: group.frames.get(pair.a), b: group.frames.get(pair.b),
          armSide: group.armSide, footSide: group.footSide })));
      }
      return { colliding: pairs.length > 0, scope: 'arm-foot-bones', pairGroups: groups.length,
        pairs, candidatePairs, triangleTests };
    },
  };
}
