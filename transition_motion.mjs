// Model-specific elbow motion guard. It does not convert the model's ROM into
// a personal medical limit, and it does not silently clamp source wrist poses.
export function createElbowMotionGuard(A) {
  const joints = A.JOINTS.filter(j => j.type === 'elbow');
  const row = A.ROM.elbow.find(row => row[0] === 'flex'), extensionLimit = row?.[4], flexionLimit = row?.[2];
  if (joints.length !== 2 || !Number.isFinite(extensionLimit) || !Number.isFinite(flexionLimit)) throw new Error('Missing elbow motion model');
  const epsilonDeg = 1e-5;
  function inspect(conf) {
    const flexDeg = {}, violations = [];
    for (const joint of joints) {
      const value = A.analyzeJoint(joint, conf.local, {}).comps.find(c => c.key === 'flex')?.signed;
      flexDeg[joint.side] = value;
      if (!Number.isFinite(value) || value < -extensionLimit - epsilonDeg || value > flexionLimit + epsilonDeg)
        violations.push({ joint: joint.id, side: joint.side, flexDeg: value,
          minimumFlexDeg: -extensionLimit, maximumFlexDeg: flexionLimit });
    }
    return { valid: violations.length === 0, flexDeg, violations };
  }
  function audit(runtime, data) {
    let states = 0;
    const minimumFlexDeg = { L: Infinity, R: Infinity }, maximumFlexDeg = { L: -Infinity, R: -Infinity };
    for (const track of Object.values(data.tracks)) {
      const times = [];
      for (let i = 0; i < track.frames.length - 1; i++) for (let j = 0; j < 20; j++)
        times.push(track.frames[i].t + (track.frames[i + 1].t - track.frames[i].t) * j / 20);
      times.push(track.duration);
      for (const time of times) {
        const result = inspect(runtime.sampleTrack(track.id, time));
        if (!result.valid) throw new Error(`Unverified elbow motion ${track.id}:${time}: ${JSON.stringify(result.violations)}`);
        for (const side of ['L', 'R']) {
          minimumFlexDeg[side] = Math.min(minimumFlexDeg[side], result.flexDeg[side]);
          maximumFlexDeg[side] = Math.max(maximumFlexDeg[side], result.flexDeg[side]);
        }
        states++;
      }
    }
    return { verified: true, scope: 'bilateral-elbow-flexion', substeps: 20, states,
      modelExtensionLimitDeg: extensionLimit, modelFlexionLimitDeg: flexionLimit,
      angularToleranceDeg: epsilonDeg, minimumFlexDeg, maximumFlexDeg,
      limitation: 'This checks the model elbow angle only; it does not establish personal ROM, tissue loading, or dynamic balance.' };
  }
  return { inspect, audit, extensionLimit, flexionLimit };
}
