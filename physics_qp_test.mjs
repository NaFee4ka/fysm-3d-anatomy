// Analytical, small QPs: solution quality must cover inequalities and optimality,
// not just equilibrium. No anatomical data or generated physics is rewritten.
import assert from 'node:assert/strict';
import { qpSolve, qpSolveVerified } from './physics_core.mjs';

const near = (actual, expected, tolerance = 3e-6) => assert(Math.abs(actual - expected) < tolerance, `${actual} differs from ${expected}`);
const solved = result => {
  assert(result instanceof Float64Array);
  const q = result.quality;
  assert.equal(q.status, 'solved'); assert.equal(q.converged, true); assert.equal(q.finite, true);
  assert(q.primalResidual <= q.primalTolerance); assert(q.dualResidual <= q.dualTolerance);
  assert(q.equalityViolation <= q.primalTolerance); assert(q.inequalityViolation <= q.primalTolerance); assert(q.boundViolation <= q.primalTolerance);
};
let checks = 0;

// Unconstrained diagonal positive-definite problem: x* = -P^-1 q.
{
  const x = qpSolveVerified([2, 4], [], [], [], [], [-10, -10], [10, 10], { q: [-4, 8] });
  solved(x); near(x[0], 2); near(x[1], -2); checks++;
}

// A weighted equality-constrained minimum has x=.75, y=.25.
{
  const x = qpSolveVerified([1, 3], [[1, 1]], [1], [], [], [0, 0], [1, 1]);
  solved(x); near(x[0], .75); near(x[1], .25); checks++;
}

// Bounds are active, and diagnostics must describe the returned clamped vector.
{
  const x = qpSolveVerified([1, 1], [], [], [], [], [0, 0], [1, 1], { q: [-4, 2] });
  solved(x); near(x[0], 1); near(x[1], 0); assert.equal(x.quality.boundViolation, 0); checks++;
}

const P = [1, 1], eq = [[0, 1]], b = [0], ineq = [new Map([[0, 1]])], upper = [.2], lo = [0, 0], hi = [1, 1], q = [-10, 0];
// Regression: this feasible problem used to have eq residual=0 and no failure
// metadata even when the solver returned x=.6000017, violating x<=.2 by .4.
{
  const x = qpSolve(P, eq, b, ineq, upper, lo, hi, { q, iters: 50 });
  assert.equal(x[1], 0); assert.equal(x.quality.equalityViolation, 0);
  near(x.quality.inequalityViolation, Math.max(0, x[0] - .2), 1e-12);
  assert(x.quality.inequalityViolation > .3); assert.equal(x.quality.converged, false);
  assert.equal(x.quality.status, 'max_iterations'); assert.equal(x.quality.boundViolation, 0); checks++;
}

// A continuation retry solves the same objective/constraints. Default stable
// variable/row identities must work even when this generic caller passes none.
{
  const x = qpSolveVerified(P, eq, b, ineq, upper, lo, hi, { q, iters: 100 });
  solved(x); near(x[0], .2); near(x[1], 0);
  assert.equal(x.quality.attempts, 2); assert.equal(x.quality.attemptQuality.length, 2);
  assert.equal(x.quality.attemptQuality[0].converged, false);
  assert.equal(x.quality.totalIterations, x.quality.attemptQuality.reduce((sum, a) => sum + a.iterations, 0));
  assert(x.warm.x instanceof Map); assert(x.warm.y instanceof Map);
  const cold = qpSolveVerified(P, eq, b, ineq, upper, lo, hi, { q });
  near(x[0], cold[0], 1e-9); checks++;
}

// Early stopping is an explicit failure by default; a diagnostic caller must
// deliberately opt into unverified output and receives its residuals unchanged.
{
  assert.throws(() => qpSolveVerified(P, eq, b, ineq, upper, lo, hi, { q, iters: 1 }), error => {
    assert.equal(error.quality.converged, false); assert.equal(error.quality.attempts, 2);
    assert(error.solution instanceof Float64Array); assert.equal(error.solution.quality, error.quality);
    return true;
  });
  const x = qpSolveVerified(P, eq, b, ineq, upper, lo, hi, { q, iters: 1, retries: 0, allowUnconverged: true });
  assert.equal(x.quality.converged, false); assert.equal(x.quality.attempts, 1); assert(x.quality.inequalityViolation > .7); checks++;
}

// Infeasible equality and bounds never become "solved" merely by clamping.
{
  assert.throws(() => qpSolveVerified([1], [[1]], [2], [], [], [0], [1], { iters: 100 }), error => {
    assert.equal(error.quality.converged, false); near(error.quality.equalityViolation, 1, 1e-12);
    assert.equal(error.quality.boundViolation, 0); return true;
  });
  checks++;
}

// A finite but nonoptimal point can satisfy every constraint. Dual stationarity
// must prevent such a point from passing a loose primal-only stopping check.
{
  const x = qpSolve([1], [], [], [], [], [-100], [100], { q: [-10], iters: 1 });
  assert.equal(x.quality.equalityViolation, 0); assert.equal(x.quality.inequalityViolation, 0);
  assert.equal(x.quality.boundViolation, 0); assert(x.quality.dualResidual > x.quality.dualTolerance);
  assert.equal(x.quality.converged, false); checks++;
}

// Verify stationarity against the returned point and exported dual, rather than
// the unclamped ADMM iterate that preceded it.
{
  const x = qpSolveVerified([1], [], [], [], [], [0], [1], { q: [-10], iters: 1, retries: 0, allowUnconverged: true });
  assert.equal(x[0], 1);
  const boundDual = x.warm.y.get('bv0') || 0;
  near(x.quality.dualResidual, Math.abs(x[0] - 10 + boundDual), 1e-12); checks++;
}

// Warm/cold solutions of a nearby problem agree; keyed duals continue correctly.
{
  const first = qpSolveVerified(P, eq, b, ineq, upper, lo, hi, { q });
  const warm = qpSolveVerified(P, eq, b, ineq, upper, lo, hi, { q: [-9, 0], warm: first.warm });
  const cold = qpSolveVerified(P, eq, b, ineq, upper, lo, hi, { q: [-9, 0] });
  solved(warm); solved(cold); near(warm[0], cold[0]); near(warm[1], cold[1]); checks++;
}

// Absolute and relative terms are separately configurable and are reported in
// the original QP scale. A large unconstrained force does not receive a made-up
// universal tolerance, and zero relative tolerance is preserved.
{
  const x = qpSolveVerified([2], [], [], [], [], [-1e7], [1e7], { q: [-2e6], epsAbs: 1e-7, epsRel: 2e-8 });
  solved(x); near(x[0], 1e6, .05); near(x.quality.dualTolerance, 1e-7 + 2e-8 * 2e6, 1e-10);
  const absolute = qpSolveVerified([1], [], [], [], [], [-10], [10], { q: [-2], epsAbs: 1e-7, epsRel: 0 });
  solved(absolute); assert.equal(absolute.quality.primalTolerance, 1e-7); assert.equal(absolute.quality.dualTolerance, 1e-7); checks++;
}

// Nonfinite iterates have an explicit failure status and cannot be published.
{
  assert.throws(() => qpSolveVerified([NaN], [], [], [], [], [0], [1], { iters: 1 }), error => {
    assert.equal(error.quality.status, 'nonfinite'); assert.equal(error.quality.finite, false); assert.equal(error.quality.attempts, 1); return true;
  });
  checks++;
}
console.log(JSON.stringify({ suite: 'verified-qp', checks, pass: true }));
