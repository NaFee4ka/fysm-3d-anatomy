// Shared by the browser and the offline trajectory checks. No mechanics are
// inferred from animation: these tracks contain geometric configurations only.
export function transitionSlerp(target, other, fraction) {
  if (fraction === 0) return target;
  const same = target.x === other.x && target.y === other.y && target.z === other.z && target.w === other.w;
  const antipodal = target.x === -other.x && target.y === -other.y && target.z === -other.z && target.w === -other.w;
  // THREE's near-identical interpolation can introduce a 1-ULP change even
  // when a rigid joint does not move. Preserve its already normalized value.
  return same || antipodal ? target : fraction === 1 ? target.copy(other) : target.slerp(other, fraction).normalize();
}
export function createTransitions(THREE, A, data) {
  const unavailable = { valid: false, poses: [], tracks: [], route: () => null, sample: () => null, sampleTrack: () => null };
  const object = o => !!o && typeof o === 'object' && !Array.isArray(o);
  const mainIds = new Set(['dog', 'plank', 'half', 'bow', 'fold', 'handstand']);
  const expectedIds = new Set([...mainIds, ...['struna', 'detali', 'centering'].flatMap(a =>
    Array.from({ length: 8 }, (_, i) => a + '-' + String(i + 1).padStart(2, '0')))]);
  if (!data || data.schemaVersion !== 1 || data.verified !== true || data.kind !== 'kinematic-demonstration'
      || !Array.isArray(data.segNames) || data.segNames.length !== A.SEG.length
      || data.segNames.some((s, i) => s !== A.SEG[i].name) || !Array.isArray(data.poses)
      || data.poses.some(p => !object(p) || typeof p.id !== 'string' || !expectedIds.has(p.id)
        || typeof p.name !== 'string' || !p.name.trim()
        || (p.kind === 'pose' ? !mainIds.has(p.id) : p.kind !== 'practice'
          || !['struna', 'detali', 'centering'].includes(p.algorithmId)
          || !Number.isInteger(p.stepIndex) || p.stepIndex < 0 || p.stepIndex > 7
          || p.id !== p.algorithmId + '-' + String(p.stepIndex + 1).padStart(2, '0')))
      || data.poses.length !== 30 || new Set(data.poses.map(p => p.id)).size !== 30
      || data.checks?.oppositeArms !== true || data.checks?.armsFeet !== true || data.checks?.floor !== true
      || data.checks?.criticalRegions !== true
      || data.checks?.elbowMotion !== true
      || !object(data.tracks) || !object(data.endpoints) || !object(data.sourceHashes)
      || !Object.keys(data.sourceHashes).length || Object.values(data.sourceHashes).some(h => typeof h !== 'string' || !/^[a-f0-9]{64}$/.test(h))) return unavailable;
  const own = (o, k) => !!o && Object.hasOwn(o, k);
  const finite = xs => Array.isArray(xs) && xs.every(Number.isFinite);
  const goodConf = f => object(f) && object(f.local) && Object.keys(f.local).length === data.segNames.length
    && finite(f.rootP) && f.rootP.length === 3 && f.rootP.every(n => Math.abs(n) <= 10)
    && finite(f.root) && f.root.length === 4 && Math.abs(Math.hypot(...f.root) - 1) < 1e-6
    && data.segNames.every(s => finite(f.local?.[s]) && f.local[s].length === 4
      && Math.abs(Math.hypot(...f.local[s]) - 1) < 1e-6);
  const goodFrame = f => goodConf(f) && Number.isFinite(f.t)
    && (f.floorLiftM === undefined || (Number.isFinite(f.floorLiftM) && f.floorLiftM >= 0 && f.floorLiftM <= .05));
  const sameConf = (a, b) => a.rootP.every((x, i) => Math.abs(x - b.rootP[i]) < 1e-8)
    && [a.root, ...data.segNames.map(s => a.local[s])].every((q, i) => {
      const r = i ? b.local[data.segNames[i - 1]] : b.root;
      return Math.min(Math.hypot(...q.map((x, j) => x - r[j])), Math.hypot(...q.map((x, j) => x + r[j]))) < 1e-8;
    });
  if (Object.values(data.endpoints).some(c => !goodFrame(c))) return unavailable;
  const tracks = Object.values(data.tracks), adjacency = new Map();
  for (const [key, track] of Object.entries(data.tracks)) {
    if (!track || track.id !== key || !own(data.endpoints, track.from) || !own(data.endpoints, track.to) || track.from === track.to
        || !Array.isArray(track.frames) || track.frames.length < 2 || !(track.duration > 0)
        || !Number.isFinite(track.duration) || !Array.isArray(track.phases)
        || !track.frames.every(goodFrame) || track.frames[0].t !== 0
        || Math.abs(track.frames.at(-1).t - track.duration) > 1e-8
        || track.frames.some((f, i, all) => i > 0 && f.t <= all[i - 1].t)
        || !sameConf(track.frames[0], data.endpoints[track.from]) || !sameConf(track.frames.at(-1), data.endpoints[track.to])
        || !track.phases.length || track.phases.some((p, i, all) => !p || !Number.isFinite(p.t0) || !Number.isFinite(p.t1)
          || p.t0 < 0 || p.t1 > track.duration + 1e-8 || p.t1 <= p.t0
          || typeof p.label !== 'string' || !p.label.trim()
          || (p.reverseLabel !== undefined && (typeof p.reverseLabel !== 'string' || !p.reverseLabel.trim()))
          || (i ? Math.abs(p.t0 - all[i - 1].t1) > 1e-8 : p.t0 !== 0))
        || Math.abs(track.phases.at(-1).t1 - track.duration) > 1e-8) return unavailable;
    for (const [from, to, reverse] of [[track.from, track.to, false], [track.to, track.from, true]]) {
      if (!adjacency.has(from)) adjacency.set(from, []);
      adjacency.get(from).push({ trackId: track.id, to, reverse, duration: track.duration });
    }
  }
  const ids = new Set(data.poses.map(p => p.id));
  if (data.poses.some(p => typeof p.id !== 'string' || !own(data.endpoints, p.id) || !adjacency.has(p.id))) return unavailable;
  function pathFor(from, to, edges) {
    let time = 0;
    const phases = edges.flatMap(edge => {
      const track = data.tracks[edge.trackId], offset = time; time += track.duration;
      return track.phases.map(p => ({
        t0: offset + (edge.reverse ? track.duration - p.t1 : p.t0),
        t1: offset + (edge.reverse ? track.duration - p.t0 : p.t1),
        label: edge.reverse ? (p.reverseLabel || p.label) : p.label,
      })).sort((a, b) => a.t0 - b.t0);
    });
    return { from, to, duration: time, edges, phases };
  }
  function route(from, to) {
    if (!ids.has(from) || !ids.has(to)) return null;
    if (from === to) return { from, to, duration: 0, edges: [], phases: [] };
    // Solve an unordered pair in one canonical direction. Equal-cost graph
    // branches must not choose different body paths when the user swaps ends.
    const reversed = from > to, origin = reversed ? to : from, destination = reversed ? from : to;
    const finish = edges => pathFor(from, to, reversed
      ? edges.toReversed().map(edge => ({ trackId: edge.trackId, reverse: !edge.reverse })) : edges);
    // Saved neighbouring flows take precedence over a faster illustrative
    // clock through another asana or the internal neutral position.
    const direct = (adjacency.get(origin) || []).filter(edge => edge.to === destination)
      .sort((a, b) => a.duration - b.duration || (a.trackId < b.trackId ? -1 : a.trackId > b.trackId ? 1 : 0));
    if (direct.length) return finish([{ trackId: direct[0].trackId, reverse: direct[0].reverse }]);
    const queue = [{ at: origin, cost: 0, edges: [] }], best = new Map([[origin, 0]]);
    while (queue.length) {
      queue.sort((a, b) => a.cost - b.cost);
      const current = queue.shift();
      if (current.cost !== best.get(current.at)) continue;
      if (current.at === destination) {
        return finish(current.edges);
      }
      for (const edge of adjacency.get(current.at) || []) {
        const cost = current.cost + edge.duration;
        if (cost >= (best.get(edge.to) ?? Infinity)) continue;
        best.set(edge.to, cost);
        queue.push({ at: edge.to, cost, edges: [...current.edges, { trackId: edge.trackId, reverse: edge.reverse }] });
      }
    }
    return null;
  }
  function configuration(frame) {
    return { local: Object.fromEntries(data.segNames.map(s => [s, new THREE.Quaternion(...frame.local[s]).normalize()])),
      rootQ: new THREE.Quaternion(...frame.root).normalize(), rootP: new THREE.Vector3(...frame.rootP) };
  }
  function sampleTrack(id, t) {
    const track = own(data.tracks, id) ? data.tracks[id] : null;
    if (!track || !Number.isFinite(t)) return null;
    t = Math.max(0, Math.min(track.duration, t));
    const frames = track.frames;
    if (t === 0) return configuration(frames[0]);
    if (t === track.duration) return configuration(frames.at(-1));
    let lo = 0, hi = frames.length - 1;
    while (hi - lo > 1) { const m = (lo + hi) >> 1; if (frames[m].t <= t) lo = m; else hi = m; }
    const a = frames[lo], b = frames[hi], u = (t - a.t) / (b.t - a.t), c = configuration(a);
    for (const s of data.segNames) transitionSlerp(c.local[s], new THREE.Quaternion(...b.local[s]).normalize(), u);
    transitionSlerp(c.rootQ, new THREE.Quaternion(...b.root).normalize(), u);
    if (!a.rootP.every((v, i) => v === b.rootP[i])) c.rootP.lerp(new THREE.Vector3(...b.rootP), u);
    // A separately audited local floor envelope. It is zero at saved frames,
    // so reversed routes and joined tracks share exactly the same endpoint.
    c.rootP.y += 4 * u * (1 - u) * (a.floorLiftM || 0);
    return c;
  }
  function sample(path, t) {
    if (!path || !ids.has(path.from) || !ids.has(path.to) || !Number.isFinite(t)) return null;
    if (!path.edges.length) return configuration(data.endpoints[path.from]);
    t = Math.max(0, Math.min(path.duration, t));
    for (let i = 0; i < path.edges.length; i++) {
      const edge = path.edges[i], track = own(data.tracks, edge.trackId) ? data.tracks[edge.trackId] : null;
      if (!track) return null;
      if (t <= track.duration || i === path.edges.length - 1)
        return sampleTrack(edge.trackId, edge.reverse ? track.duration - t : t);
      t -= track.duration;
    }
    return null;
  }
  // Connectivity is a release invariant, not an assumption about the hub.
  if (data.poses.some(a => data.poses.some(b => !route(a.id, b.id)))) return unavailable;
  return { valid: true, poses: data.poses, tracks: tracks.map(t => ({ id: t.id, from: t.from, to: t.to, duration: t.duration })),
    route, sample, sampleTrack, data };
}
