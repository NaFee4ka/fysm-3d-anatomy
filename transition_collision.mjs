import { createWholeBodyCollision } from './whole_body_collision.mjs';

const scope = 'transition-critical-bone-pairs-v2';
const categories = Object.freeze(['all-arm-leg-pairs', 'limbs-axial-body-including-head',
  'all-leg-pairs', 'distal-fingers-own-palms', 'own-forearm-first-metacarpals',
  'scapulae-third-thoracic-vertebra']);

// Hard supplementary scope for the new animation. Existing exact opposite-arm
// and arm/foot checks still run independently with their unchanged policy.
// Adjacent segments use the same bounded joint-local policy as the whole-body
// checker. Their remote shafts never receive a whole-pair exemption.
export function createTransitionCollision(A, THREE, boneData, { partialGeometryForTests = false } = {}) {
  if (!Array.isArray(boneData?.parts)) throw new Error('Missing transition-collision bone manifest');
  const arm = s => /^(hum|fore|ulna|hand|f[1-5][mabc])_[LR]$/.test(s);
  const leg = s => /^(thigh|shank|foot|toes)_[LR]$/.test(s);
  const axial = s => /^(pelvis|lumA|lumB|thA|thB|thC|neckA|neckB|head)$/.test(s);
  const girdle = s => /^(clav|scap)_[LR]$/.test(s);
  const hand = s => /^hand_[LR]$/.test(s);
  const distalFinger = s => /^f[1-5][bc]_[LR]$/.test(s);
  // Part indices locate metadata in the supplied manifest; none are anatomical
  // selector constants. Reordering source parts preserves both new categories.
  const metadata = descriptor => boneData.parts[descriptor.partIndex];
  const forearm = p => /^fore_[LR]$/.test(p.seg) && metadata(p).latin === 'radius'
    || /^ulna_[LR]$/.test(p.seg) && metadata(p).latin === 'ulna';
  const firstMetacarpal = p => /^f1m_[LR]$/.test(p.seg) && metadata(p).latin === 'os metacarpale I';
  const scapula = p => /^scap_[LR]$/.test(p.seg) && metadata(p).latin === 'scapula';
  const thirdThoracicVertebra = p => axial(p.seg) && metadata(p).latin === 'vertebra thoracica Th3';
  const ownForearmThumb = (a, b) => (forearm(a) && firstMetacarpal(b) || firstMetacarpal(a) && forearm(b))
    && a.seg.slice(-1) === b.seg.slice(-1);
  const scapulaTh3 = (a, b) => scapula(a) && thirdThoracicVertebra(b) || thirdThoracicVertebra(a) && scapula(b);
  const descriptors = boneData.parts.map((part, partIndex) => ({ seg: part.seg, partIndex }));
  const forearms = descriptors.filter(forearm), thumbs = descriptors.filter(firstMetacarpal);
  const scapulae = descriptors.filter(scapula), th3 = descriptors.filter(thirdThoracicVertebra);
  const semanticPairCounts = Object.freeze({ ownForearmFirstMetacarpals: forearms.reduce((sum, a) =>
    sum + thumbs.filter(b => a.seg.slice(-1) === b.seg.slice(-1)).length, 0), scapulaTh3: scapulae.length * th3.length });
  // Production must not silently certify an empty semantic category if a
  // source label disappears. Only deliberately partial synthetic test meshes
  // opt out; actual geometry uses the default complete-inventory requirement.
  if (partialGeometryForTests !== true) {
    const onePerSegment = (parts, segments) => parts.length === segments.length
      && segments.every(seg => parts.filter(p => p.seg === seg).length === 1);
    if (!onePerSegment(forearms, ['fore_L', 'ulna_L', 'fore_R', 'ulna_R'])
      || !onePerSegment(thumbs, ['f1m_L', 'f1m_R']) || !onePerSegment(scapulae, ['scap_L', 'scap_R'])
      || th3.length !== 1 || semanticPairCounts.ownForearmFirstMetacarpals !== 4 || semanticPairCounts.scapulaTh3 !== 2)
      throw new Error('Transition critical semantic inventory must contain four own-forearm/thumb and two scapula/Th3 pairs');
  }
  const collider = createWholeBodyCollision(A, THREE, boneData);
  const filter = (a, b) => {
    // Neither pair gains a natural-articulation exception. The former bow path
    // folded the first metacarpal into radius/ulna; a half-pose release swept a
    // scapula through Th3. Both endpoints of those paths were individually clear.
    if (ownForearmThumb(a, b) || scapulaTh3(a, b)) return true;
    if ((arm(a.seg) && leg(b.seg)) || (leg(a.seg) && arm(b.seg))) return true;
    if (((arm(a.seg) || leg(a.seg)) && axial(b.seg)) || (axial(a.seg) && (arm(b.seg) || leg(b.seg)))) {
      return true;
    }
    if (leg(a.seg) && leg(b.seg)) {
      return true;
    }
    return (hand(a.seg) && distalFinger(b.seg) || distalFinger(a.seg) && hand(b.seg)) && a.seg.slice(-1) === b.seg.slice(-1);
  };
  // The lower-motion cache omits clavicles/scapulae. A scapular pair must only
  // enter the upper inspector, whose key includes the shoulder-girdle motion.
  const upper = s => arm(s) || girdle(s);
  const lowerFilter = (a, b) => filter(a, b) && !upper(a.seg) && !upper(b.seg);
  const upperFilter = (a, b) => filter(a, b) && (upper(a.seg) || upper(b.seg));
  return { inspect: (W, { maxPairs = 1 } = {}) => collider.inspect(W, { pairFilter: filter,
    pairFilterLabel: scope, maxPairs }),
  inspectLower: W => collider.inspect(W, { pairFilter: lowerFilter, pairFilterLabel: 'all-legs-with-legs-and-axial-body', maxPairs: 1 }),
  inspectUpper: W => collider.inspect(W, { pairFilter: upperFilter,
    pairFilterLabel: 'arms-with-legs-axial-palms-own-thumb-forearms-and-scapula-th3', maxPairs: 1 }),
  collider, scope, categories, semanticPairCounts };
}
