# Joint tissue surfaces

`joints.json` contains **reconstructed** cartilage, labra and menisci fitted to the
finished skeleton in `skull/bones_bp3d.json`. It does not contain atlas-segmented
cartilage. All 42 parts have `source: "reconstructed"`, empty `ids`, explicit
`boneRefs`, construction methods, region parameters and limitations.

The frozen bone geometry and the physics are not changed by this builder. The
`basis.sha256` records the exact packed skeleton used to reconstruct the tissues.
Source bones are BodyParts3D, © DBCLS, CC BY 4.0. Preserve that attribution when
distributing these derived surfaces.

## Coverage

Each side has 21 parts, covering six joint groups:

| Joint | Reconstructed parts per side |
|---|---|
| Shoulder | Medial humeral head cap, glenoid face, triangular glenoid labral lip |
| Hip | Medial femoral head cap, acetabular lunate face, acetabular labral lip |
| Knee | Distal femoral articular region, superior tibial plateau, posterior patella, medial and lateral open C-shaped meniscal wedges |
| Elbow | Distal humeral articular region, proximal radial head, proximal anterior ulnar notch |
| Wrist | Distal radial face, proximal scaphoid, lunate and triquetral faces |
| Ankle | Talar trochlea, tibial plafond, medial distal fibular facet |

`replaces` lists the exact existing Russian bucket names to suppress after the
new meshes load. Suppression is keyed by `seg + "|" + name`, preserving unrelated
bone and cartilage buckets. All old cartilage, meniscus and labrum buckets for
these six groups are covered. Spine, fingers/toes, sternoclavicular,
acromioclavicular and other tissues retain their previous representations.

## Geometry and format

The builder decodes the ready bones into rest-world metres (+X left, +Y up,
+Z anterior), welds coincident bone vertices in memory and clips continuous
anatomical region fields against the actual bone triangles. It adds an outward
thin shell, with a closed base and side wall. Thicknesses of 0.4–1.0 mm are
illustrative display choices, not measured cartilage thicknesses. Only edges
longer than 8 mm are subdivided, sharing edge vertices; subdivision does not
claim extra anatomical detail absent from the frozen source geometry.
Separate face fans touching only at a point receive separate vertex indices
before extrusion (39 such splits), so their thin walls cannot form a shared
edge with four or six incident faces. Their source positions are not moved.

Labra follow the outer closed contour of the reconstructed socket footprint,
resampled and mildly faired twice, with a triangular raised cross section.
They are reconstructed rims, not generic circular tori and not actual atlas
labra. Menisci use separate measured medial/lateral tibial plateau footprints,
open C-shaped wedges, tapered horns and closed caps. Their bases are projected
onto the finished tibial triangles; the inner space and horn openings remain
unfilled. Their dimensions and horn locations are reconstructions.

Positions use bbox quantization to unsigned 16-bit triples, coordinate-wise
delta encoding modulo 65536, then flattened unsigned 16-bit triangle index
deltas. All arrays are concatenated into one deterministic gzip/base64 blob.
`n` is vertex count and `m` is triangle count. The renderer subtracts the rest
segment joint once, then rigidly follows that segment's transform. It does not
deform these vertices each frame or change skeleton transforms.

## Build and checks

Only Python 3 and NumPy are required for this optional data rebuild. The normal
root `python3 build.py` uses the checked-in data and needs no NumPy installation.
No original BP3D archive, network call, trimesh or rtree is needed here.

```sh
python3 joints/build.py
python3 joints/check.py --rebuild --report joints/check-report.json
python3 joints/capsule_rims.py --check
node joint_surfaces_test.mjs --actual
python3 build.py
```

The data build writes the packed JSON atomically and records counts and per-part
mesh measurements in `coverage.json`. `check.py` checks the frozen bone hash,
all 12 sided joint groups, legacy replacements, explicit provenance, closed
manifold shells, opposing winding at shared edges, unit normals and
nondegenerate quantized triangles. Exact
closest-triangle queries verify cartilage vertices remain within their stated
offset from their own ready bone and menisci remain on the plateau. The optional
`--rebuild` compares a full in-memory rebuild with the checked-in data.

Current package: **42 parts, 21,044 vertices, 41,992 triangles, 377,054 bytes**.
All 42 quantized meshes are closed manifold shells, with no collapsed triangles
or zero vertex normals. Maximum cartilage distance from its own bone is
**1.00055 mm**, consistent with the chosen offset and 16-bit quantization.
The glenoid region also has a height bound and selects the component nearest
the ready skeleton's glenoid landmark. This excludes the nearby lower acromial
face, which otherwise also faces the humeral head. A broad paired-side
bbox/area/centroid check catches this region error while preserving the actual
left/right asymmetry of the fitted source bones.
The right glenoid footprint has about 0.343 times the left footprint area in
this frozen fit; mirrored centroids differ by 11.35 mm. The broad check permits
area ratios from 0.25 to 4, per-axis bbox ratios from 0.35 to 2.85 and centroid
differences below 24 mm. These are coarse implementation bounds rather than
anatomical normal ranges; the tighter socket height/region constraints reject
the specific acromial-component error.
The runtime suite verifies rigid following, selection, bounds and layer behavior
across the poses and cycle states. These checks establish implementation
consistency; they do not validate clinical tissue shapes or joint mechanics.

## Display capsule attachment rims

`capsule-rims.json` is a separate small display reconstruction: **12 capsules,
200 control points and 808 dense rim samples, 78,275 bytes**. It does not alter `joints.json` or the numerical
capsule strands. `capsule_rims.py` reads the existing strand identifiers and
order through `createAnatomy`, then projects the two display attachment rings
onto bounded epiphyseal regions of the appropriate finished bones, with a
0.6 mm outward offset. Shoulder/hip proximal rings follow the actual socket
footprint contour; cyclic arc-length sampling prevents independent nearest
projections from collapsing source angles onto one corner. The wrist distal
rim uses the proximal scaphoid/lunate/triquetrum union. The inherited open knee
sector and the source count/order are preserved.
Between-knot samples are also fitted to the same bounded bone regions, rather
than allowing a spline or chord to cut across a bone. The elbow's distal rim
and wrist's proximal rim use actual horizontal bone-plane sections with
ordered radial intersections; this prevents closest-point corner collapses.

Each record has `sourceIds`, `prox` and `dist` (`seg`, nested xyz `positions`,
`densePositions`, `fixedSampleCount`),
and a display `wrap` ellipsoid (`seg`, `c`, `r`) derived from the selected
bone-region dimensions. It is a display routing estimate, not a collision
surface. Mechanical anchors, wrapping, routed lengths, strain, forces and
physics remain based on the original strands. This geometry has reconstructed
attachment regions, not segmented or measured capsular insertions.

This optional rebuild additionally needs Node and the repository's `three`
package (`npm ci` first). No network or original atlas files are needed after
dependencies are installed:

```sh
python3 joints/capsule_rims.py
python3 joints/capsule_rims.py --check
```

`--check` creates the data in memory, compares it with the checked-in sidecar,
and writes nothing. It verifies the bone hash through the rebuild, every point
within 0.625 mm of its ready bone, the original identifiers/count/order, no
collapsed adjacent samples and no crossing edges in each rim's principal-plane
projection. With four intervals per source pair, the closed rims have 33 or 37
dense samples and the open knee rims have 29. Every fourth dense sample
matches its source display control point exactly; the closed seam repeats its
first sample. All 808 dense samples are within 0.625 mm of their own ready
bone. Renderer tests and browser review cover sampled paths and states; they
do not guarantee absence of capsule collisions or self-intersections in all
animated states.

## Source availability and limitations

`source-audit.json` records the official BodyParts3D 4.0 metadata/archive search,
file hashes, exact available objects and misleading name matches. Neither
metadata table supplies named articular cartilage, menisci, labra, capsules or
the main internal/collateral ligaments of these six limb joint groups. The
actual wrist flexor retinacula and long plantar ligaments are distinct tissues;
they are not substituted for missing cartilage. There are 23 genuine spinal
disks, including one geometrically inferred level; they are recorded in the
audit but are not added to this limb package.

The frozen rig has no physiological cartilage clearance. For example its rest
knee already has millimetres of femur/tibia overlap. This builder preserves that
skeleton and cannot create a clinical joint space. Cartilage follows its own
bone rigidly; menisci do not translate, compress or follow both opposing bones.
Labral compliance, fluid, synovium, contact, collision and pressure are not
simulated. Acetabular masks, glenoid contours and meniscal boundaries inherit
the resolution and fitting limitations of the ready bone meshes. The hip rim is
a complete illustrative lip; a separate transverse acetabular ligament is not
segmented. Source-independent labels and colors must not imply medical
validation or a segmented patient anatomy.
