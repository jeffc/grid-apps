import { base } from '../../../../geo/base.js';
import { codec } from '../../../core/codec.js';
import { newPoint } from '../../../../geo/point.js';
import { newPolygon } from '../../../../geo/polygon.js';
import { sliceConnect } from '../../../../geo/slicer.js';
import { newSlice } from '../../../core/slice.js';
import { Tool } from '../core/tool.js';
import { Slicer as topo_slicer } from './slicer-topo.js';
import { THREE } from '../../../../ext/three.js';
import { Topo } from './topo4.js';

const RAD2DEG = 180 / Math.PI;
const DEG2RAD = Math.PI / 180;

function scale(fn, factor = 1, base = 0) {
    return (value, msg) => {
        fn(base + value * factor, msg);
    }
}

export class FourAxis extends Topo {
    constructor() {
        super();
    }

    async slice(onupdate) {
        // Force CPU execution for four-axis because the accessibility/machinability (C-Space) mapping
        // requires raw, un-offset model contours to check for tool collisions across 360 degrees.
        // The WebGPU rasterizer (RasterPath) only outputs pre-offset, vertical toolpath profiles (CL)
        // and does not have a shader implementation for multi-angle C-space search.
        // TODO: Reevaluate at a later date if WebGPU support can be added for 4-axis C-Space mapping.
        this.gpu = false;
        return await super.slice(onupdate);
    }

    async generatePath(onupdate) {
        if (this.gpu) {
            return this.gpu_slices;
        }
        const { resolution, toolInstance, sliced: slices } = this;
        /**
         * ALGORITHM STEP 2: Accessibility Analysis (C-Space Mapping)
         * For each point on the slice contours, determine the 'feasible orientation interval'.
         * 1. Build a machinability map for each slice from original contours.
         * 2. For each resampled point, sample 360 degrees in 5-degree increments.
         * 3. Check for upward (machine +Z) collisions after rotation.
         */
        // Production resample distance (resolution * 2) for contouring.
        const resampleDist = resolution * 2;
        const resampledSlices = this.slices = [];
        console.log(`topo4 generate | slices=${slices.length} resampleDist=${resampleDist}`);

        const machinabilityMap = slices.map(slice => {
            const polys = (slice.tops ? slice.tops.map(t => t.poly) : slice.camLines) || [];
            return new Machinability(polys, resolution, toolInstance);
        });

        let sliceCount = 0;
        for (let slice of slices) {
            const isEvery100 = (sliceCount % 100 === 0);
            const mach = machinabilityMap[sliceCount++];

            const resampledSlice = newSlice(slice.z);
            resampledSlice.index = slice.index;
            const resampledPolys = [];
            // use slice.tops or slice.camLines
            const polys = (slice.tops ? slice.tops.map(t => t.poly) : slice.camLines) || [];
            let pIn = 0, pOut = 0;
            for (let poly of polys) {
                const points = poly.points || (poly.id ? undefined : poly.array);
                if (points) pIn += (points.length / (poly.points ? 1 : 3));

                // 2a. Resample the polygon contour points.
                // Resampling ensures uniform resolution along the contour for accessibility checks.
                const resampled = resamplePolygon(poly, resampleDist);
                if (resampled) {
                    // 2b. Compute surface outward normals for the resampled vertices.
                    const normals = computeNormals(resampled);
                    
                    for (let i = 0; i < resampled.points.length; i++) {
                        const p = resampled.points[i];
                        const n = normals[i];
                        
                        // 2c. Query the Machinability C-space map to find all valid rotation angles (MDRs) 
                        // for this point and normal. p.mdr will hold an array of collision-free angles.
                        p.mdr = mach.getMDRs(p, n);
                    }
                    
                    // Segment Decomposition
                    const candidates = generateSegmentCandidates(resampled.points);
                    let chosen = findMinimumCover(candidates, resampled.points.length);
                    if (!chosen) {
                        chosen = candidates; // Fallback
                    }
                    const resolved = resolveOverlaps(chosen, resampled.points.length);
                    
                    // Reconstruct path segments as separate open/closed polygons
                    const isContourOpen = resampled.isOpen ? resampled.isOpen() : (resampled.open || false);
                    const len = resampled.points.length;
                    
                    for (let seg of resolved) {
                        const runs = getContiguousRuns(seg.indices, len, isContourOpen);
                        for (let run of runs) {
                            if (run.length === 0) continue;
                            const segmentPoints = [];
                            for (let idx of run) {
                                const originalPt = resampled.points[idx];
                                const n = normals[idx];
                                
                                // Compute ideal target normal angle
                                let target = Math.atan2(-n.y, n.z) * RAD2DEG;
                                if (target < 0) target += 360;
                                
                                // Create a new point for this segment and assign angle
                                const newPt = newPoint(originalPt.x, originalPt.y, originalPt.z);
                                newPt.mdr = originalPt.mdr;
                                
                                let angle;
                                if (seg.mds && seg.mds.length > 0) {
                                    angle = getClosestAngleInMDS(target, seg.mds);
                                } else if (originalPt.mdr && originalPt.mdr.length > 0) {
                                    angle = getClosestAngleInMDS(target, originalPt.mdr);
                                } else {
                                    angle = 0;
                                }
                                newPt.a = angle;
                                segmentPoints.push(newPt);
                            }
                            
                            const isRunClosed = (!isContourOpen && run.length === len);
                            const segmentPoly = newPolygon(segmentPoints).setOpen(!isRunClosed);
                            resampledPolys.push(segmentPoly);
                            pOut += segmentPoints.length;
                        }
                    }
                }
            }
            if (resampledPolys.length > 0) {
                if (isEvery100) console.log(`  slice z=${slice.z.round(2)} index=${slice.index} pIn=${pIn} pOut=${pOut}`);
                resampledSlice.addTops(resampledPolys);
                resampledSlice.camLines = resampledPolys;
                if (isEvery100) {
                    const layers = resampledSlice.output();
                    const deepClone = (poly) => {
                        const np = newPolygon(poly.points.map(p => {
                            const pt = newPoint(p.x, p.y, p.z);
                            if (p.a !== undefined) pt.a = p.a;
                            if (p.mdr) pt.mdr = p.mdr;
                            return pt;
                        }));
                        np.setOpen(poly.isOpen ? poly.isOpen() : (poly.open || false));
                        return np;
                    };

                    const colors = [0x00ffff, 0xff00ff, 0xffcc00, 0xff6600, 0x9900ff];
                    for (let sIdx = 0; sIdx < Math.min(resampledPolys.length, 5); sIdx++) {
                        const poly = resampledPolys[sIdx];
                        const shifted = deepClone(poly).move({ z: -this.zoff, x: 0, y: 0 });
                        layers.setLayer(`fouraxis-segment-${sIdx + 1}`, { line: colors[sIdx] })
                            .addPoly(shifted, { thin: false });
                    }
                    if (resampledPolys.length > 5) {
                        const remainingPolys = resampledPolys.slice(5).map(poly => deepClone(poly).move({ z: -this.zoff, x: 0, y: 0 }));
                        layers.setLayer("fouraxis-segment-other", { line: 0x888888 })
                            .addPolys(remainingPolys, { thin: true });
                    }

                     const mdrPolys = [];
                     const chosenPolys = [];
                     let pCount = 0;
                     for (let p of resampledPolys.map(poly => poly.points).flat()) {
                         if (pCount++ % 10 !== 0) continue;
                         if (p.mdr) {
                             for (let angle of p.mdr) {
                                 const rad = angle * DEG2RAD;
                                 // Machine UP in part space for CCW rotation 'a' around X:
                                 // dY = -sin(a), dZ = cos(a) (CW rotation of tool ray in part space)
                                 const dy = -Math.sin(rad) * 0.3;
                                 const dz = Math.cos(rad) * 0.3;
                                 // Un-swap for visualization: p.x is original X, p.y is Y, p.z is original Z
                                 mdrPolys.push(newPolygon([
                                     newPoint(p.x, p.y, p.z - this.zoff),
                                     newPoint(p.x, p.y + dy, p.z + dz - this.zoff)
                                 ]).setOpen());
                             }
                         }
                         if (p.a !== undefined) {
                             const rad = p.a * DEG2RAD;
                             // Make the chosen ray slightly longer (e.g. 0.5) to stand out
                             const dy = -Math.sin(rad) * 0.5;
                             const dz = Math.cos(rad) * 0.5;
                             chosenPolys.push(newPolygon([
                                 newPoint(p.x, p.y, p.z - this.zoff),
                                 newPoint(p.x, p.y + dy, p.z + dz - this.zoff)
                             ]).setOpen());
                         }
                     }

                     if (mdrPolys.length > 0) {
                         layers.setLayer("fouraxis-machinable", { line: 0xff0000 })
                             .addPolys(mdrPolys, { thin: true });
                     }
                     if (chosenPolys.length > 0) {
                         layers.setLayer("fouraxis-chosen", { line: 0x0000ff })
                              .addPolys(chosenPolys, { thin: true });
                      }
                 }
                resampledSlices.push(resampledSlice);
            }
        }
        console.log(`topo4 generate | resampledSlices=${resampledSlices.length}`);

        return resampledSlices;
    }

    async sliceGPU(onupdate) {
        const slices = await super.sliceGPU(onupdate);
        for (let slice of slices) {
            slice.output()
                .setLayer("four-axis", { line: this.lineColor })
                .addPoly(slice.camLines[0].clone().applyRotations());
        }
        return slices;
    }

    async fourAxisWorker(onupdate) {
        const paths = await this.latheWorker(onupdate);
        for (let slice of paths) {
            const poly = slice.camLines[0];
            slice.output()
                .setLayer("four-axis", { line: this.lineColor })
                .addPoly(poly.clone().applyRotations().move({ z: -this.zoff, x: 0, y: 0 }));
        }
        return paths;
    }

    async fourAxisMinions(onupdate) {
        const paths = await this.latheMinions(onupdate);
        for (let slice of paths) {
            const poly = slice.camLines[0];
            slice.output()
                .setLayer("four-axis", { line: this.lineColor })
                .addPoly(poly.clone().applyRotations().move({ z: -this.zoff, x: 0, y: 0 }));
        }
        return paths;
    }
}

export function resamplePolygon(poly, dist) {
    if (!poly) return undefined;
    const points = poly.points || (poly.id ? undefined : poly.array); // handle different point formats
    if (!points || (points.length < 2 && !Array.isArray(points))) return undefined;

    // convert to proper point array if it's a flat float array from decoder
    let pts = points;
    if (points instanceof Float32Array || (Array.isArray(points) && typeof points[0] === 'number')) {
        pts = [];
        for (let i=0; i<points.length; i += 3) {
            pts.push(newPoint(points[i], points[i+1], points[i+2]));
        }
    }

    if (pts.length < 2) return undefined;

    const newPoints = [];
    const len = pts.length;
    const isOpen = poly.open || (poly.isOpen ? poly.isOpen() : false);

    for (let i = 0; i < len; i++) {
        const p1 = pts[i];
        const p2 = pts[(i + 1) % len];
        newPoints.push(p1);
        if (isOpen && i === len - 1) continue;
        const d = p1.distTo3D(p2);
        if (d > dist) {
            const steps = Math.floor(d / dist);
            for (let j = 1; j <= steps; j++) {
                const ratio = j / (steps + 1);
                newPoints.push(newPoint(
                    p1.x + (p2.x - p1.x) * ratio,
                    p1.y + (p2.y - p1.y) * ratio,
                    p1.z + (p2.z - p1.z) * ratio
                ));
            }
        }
    }
    return newPolygon(newPoints).setOpen(isOpen);
}

export class Machinability {
    /**
     * @param {Polygon[]} polys Slice contours
     * @param {number} resolution Machine resolution (mm)
     * @param {Tool} tool Kiri tool instance
     */
    constructor(polys, resolution, tool) {
        this.resolution = resolution;

        // 1. Analytical Tool Parameters
        // We extract the physical dimensions of the active tool to perform exact collision checks.
        const isBall = tool.isBallMill();
        const isTaper = tool.isTaperMill();
        const isTaperBall = tool.isTaperBall();
        const isDrill = tool.isDrill();

        let tipRad = tool.tipDiameter() / 2;
        if (isBall) {
            tipRad = tool.fluteDiameter() / 2;
        } else if (isTaperBall) {
            const { r } = tool.getBallTaperParams();
            tipRad = r;
        }

        this.tool = {
            isBall, isTaper, isTaperBall, isDrill,
            fluteRad: tool.fluteDiameter() / 2,
            tipRad,
            fluteLen: tool.fluteLength() || 100,
            taperAngle: tool.getTaperAngle(),
        };

        // For tapered ball mills, find the boundary parameters transitioning from spherical tip to conical flute
        if (isTaperBall) {
            const { r, b } = tool.getBallTaperParams();
            this.tool.ballR = r;
            this.tool.ballB = b;
        }

        // 2. Coordinate System Synchronization
        // Kiri:Moto processes 4-axis by slicing normal to the X-axis (the axis of rotation).
        // Standard coordinates for a resampled point 'p' are:
        // - p.x: Coordinate along the rotation axis (Model X). This is constant for a given slice.
        // - p.y: Model Y coordinate.
        // - p.z: Model Z coordinate (depth/height relative to the rotation center).
        // Since slicing swaps axes to make the slice plane normal to X:
        // All analysis and collision detection occurs in the 2D cross-section plane (Model Z, Model Y).
        this.polys = polys.map(p => {
            if (p.id) return p;
            const points = p.points || p.array;
            if (points instanceof Float32Array || (Array.isArray(points) && typeof points[0] === 'number')) {
                const pts = [];
                for (let i=0; i<points.length; i += 3) {
                    pts.push(newPoint(points[i], points[i+1], points[i+2]));
                }
                return newPolygon(pts).setOpen(p.open);
            }
            return p;
        });

        // 3. Segment Flattening
        // We flatten the contour polygons into individual segments represented in (Z, Y) coordinates
        // to perform fast analytical line-to-cutter collision tests.
        const segments = this.segments = [];
        for (let poly of this.polys) {
            poly.forEachSegment((p1, p2) => {
                segments.push({ p1, p2 });
            });
        }
    }

    /**
     * @param {Point} p Point in slice plane (YZ)
     * @param {Point} n Normal in slice plane (YZ)
     * @returns {number[]} Array of angles (0-355) that are machinable
     */
    getMDRs(p, n) {
        const mdrs = [];
        for (let a = 0; a < 360; a += 5) {
            if (this.isMachinable(p, n, a)) {
                mdrs.push(a);
            }
        }
        return mdrs;
    }

    /**
     * Computes tool radius at a given depth above the tip center.
     * @param {number} d Depth (mm)
     * @returns {number} Radius (mm)
     */
    getRadius(d) {
        const t = this.tool;
        if (d <= 0) return 0;
        if (t.isBall) {
            // Sphere: x^2 + (z-R)^2 = R^2 => x = sqrt(R^2 - (z-R)^2)
            if (d < t.fluteRad) return Math.sqrt(t.fluteRad * t.fluteRad - (d - t.fluteRad) * (d - t.fluteRad));
            return t.fluteRad;
        }
        if (t.isTaperBall) {
            if (d < t.ballB) return Math.sqrt(t.ballR * t.ballR - (d - t.ballR) * (d - t.ballR));
            const rad = t.ballR + (d - t.ballB) * Math.tan(t.taperAngle * Math.PI / 180);
            return Math.min(rad, t.fluteRad);
        }
        if (t.isTaper) {
            const rad = t.tipRad + d * Math.tan(t.taperAngle * Math.PI / 180);
            return Math.min(rad, t.fluteRad);
        }
        if (t.isDrill) {
            return Math.min(d * Math.tan(70 * Math.PI / 180), t.fluteRad);
        }
        return t.fluteRad; // Standard Endmill
    }

    /**
     * Analytical collision check for a given rotation angle.
     * Checks if a tool pointing down in machine space (Z direction) collides with the model
     * when the part is rotated by the given angle.
     *
     * @param {Point} p Contact Center (CC) in (z, y)
     * @param {Point} n Outward Normal in (z, y). n.z is the Z-component, n.y is the Y-component.
     * @param {number} angle Machine rotation (degrees). 0 is home, positive is CCW rotation around X.
     * @returns {boolean} True if orientation is collision-free (machinable)
     */
    isMachinable(p, n, angle) {
        // Convert the machine rotation angle to radians.
        const rad = (angle * Math.PI) / 180;
        const cos = Math.cos(rad);
        const sin = Math.sin(rad);

        // 1. Tool Ray Direction (d_l) in Part Space
        // In machine space, the tool spindle axis always points straight down (along machine -Z).
        // Conversely, the "upward" direction from the workpiece to the spindle is along machine +Z.
        // If the workpiece rotates CCW by `angle` (rad), the tool spindle appears to rotate CW 
        // by `-rad` relative to the part coordinate frame.
        // Under a CW rotation of `-rad` around the X-axis:
        // - dz_part = cos(-rad) = cos(rad)
        // - dy_part = sin(-rad) = -sin(rad)
        // Thus, the tool axis ray pointing upwards in part space is:
        const dz = cos;
        const dy = -sin;

        const R = this.tool.tipRad;

        // 2. Surface Feasibility Check (Back-face Culling)
        // The tool axis must point outward (away from the material) at the contact point.
        // Mathematically, the dot product between the tool ray (dz, dy) and the surface normal (n.z, n.y)
        // must be greater than or equal to 0 (within a small tolerance). If the dot product is negative,
        // the tool would be pointing "down into" the workpiece surface, indicating a back-face collision.
        const dot = dz * n.z + dy * n.y;
        if (dot < -0.001) return false;

        // 3. Cutter Location (CL) calculation
        // The "Cutter Location" (CL) is the coordinate of the center of the tool's tip sphere/circle.
        // Since the tool tip is tangent to the surface at the contact point (CC), we offset from the contact point
        // along the outward surface normal by the tool's tip radius:
        // CL = P + R * n
        const CL = { z: p.z + n.z * R, y: p.y + n.y * R };

        // 4. Lateral Axis (Perpendicular to the tool axis ray)
        // We define a lateral direction vector perpendicular to (dz, dy) in the (Z, Y) plane.
        // A vector perpendicular to (dz, dy) is (-dy, dz) = (sin, cos).
        const perpZ = -dy;
        const perpY = dz;

        const eps = 0.01;
        const chordEps = 0.1; // Tolerance to account for piecewise-linear discretization errors of the model
        const Rsq = (R - chordEps) * (R - chordEps);

        // 5. Check all segments for collision with the tool geometry
        for (let seg of this.segments) {
            // Segment endpoints in the slice plane (Model Z, Model Y)
            const s1 = { z: seg.p1.z, y: seg.p1.y };
            const s2 = { z: seg.p2.z, y: seg.p2.y };

            // 6. Ball Tip Volume Collision Test
            // Check if the segment intersects the spherical/circular tip of the tool.
            // We calculate the closest point on the line segment [s1, s2] to the tip center (CL).
            const dxs = s2.z - s1.z;
            const dys = s2.y - s1.y;
            const lensq_seg = dxs * dxs + dys * dys;
            let ts = lensq_seg > 0 ? ((CL.z - s1.z) * dxs + (CL.y - s1.y) * dys) / lensq_seg : 0;
            ts = Math.max(0, Math.min(1, ts));
            const closestZ = s1.z + ts * dxs;
            const closestY = s1.y + ts * dys;
            const distSq = (closestZ - CL.z) ** 2 + (closestY - CL.y) ** 2;
            
            // If the closest point on the segment is inside the tool tip radius, we have a collision.
            if (R > eps && distSq < Rsq) return false;

            // 7. Shaft/Flute Volume Collision Test
            // We project the segment endpoints into the tool's local 2D coordinate system.
            // Origin is at the tip center (CL), and the tool axis (spindle) points along (dz, dy).
            const v1 = { z: s1.z - CL.z, y: s1.y - CL.y };
            const v2 = { z: s2.z - CL.z, y: s2.y - CL.y };

            // sz: Depth along the tool axis (0 at the tip center, positive going up the shaft)
            // sx: Lateral distance perpendicular to the tool axis
            const sz1 = v1.z * dz + v1.y * dy;
            const sx1 = v1.z * perpZ + v1.y * perpY;
            const sz2 = v2.z * dz + v2.y * dy;
            const sx2 = v2.z * perpZ + v2.y * perpY;

            // Collision check function at a given depth/lateral position
            const checkPoint = (depth, lateral) => {
                if (depth < eps) return true; // Ignore sections of the model that are behind the tip center
                // A collision occurs if the lateral distance of the segment point to the tool axis
                // is less than the tool's profile radius at that depth.
                return Math.abs(lateral) > this.getRadius(depth + R) - eps;
            };

            // Check the segment endpoints and midpoint
            if (!checkPoint(sz1, sx1)) return false;
            if (!checkPoint(sz2, sx2)) return false;
            if (!checkPoint((sz1 + sz2) / 2, (sx1 + sx2) / 2)) return false;

            // Check the critical point where the segment crosses the tool axis (sx = 0)
            const dsx = sx2 - sx1;
            const dsz = sz2 - sz1;
            if (Math.abs(dsx) > 1e-9) {
                // Find parameter t where the lateral coordinate sx equals 0
                let t_ray = -sx1 / dsx;
                if (t_ray > 0 && t_ray < 1) {
                    const sz_at_t = sz1 + t_ray * dsz;
                    if (!checkPoint(sz_at_t, 0)) return false;
                }
            }
        }

        return true;
    }
}

export function computeNormals(poly) {
    const points = poly.points;
    const len = points.length;
    if (len < 2) return points.map(() => { return { z: 0, y: 1 } });

    // 1. Compute signed area in the (Z, Y) plane to determine the polygon's winding direction.
    // In our coordinate convention: p.z is Model Z, p.y is Model Y.
    // A positive area indicates clockwise (CW) winding, while negative is counter-clockwise (CCW).
    let area2 = 0;
    for (let i = 0; i < len; i++) {
        const p1 = points[i];
        const p2 = points[(i + 1) % len];
        area2 += (p2.z - p1.z) * (p2.y + p1.y);
    }
    const isCW = area2 > 0;

    const normals = [];
    const isOpen = poly.open || (poly.isOpen ? poly.isOpen() : false);

    // 2. Iterate through vertices and compute the outward normal at each vertex.
    // Vertex normals are calculated by averaging the normals of the two adjacent segments.
    for (let i = 0; i < len; i++) {
        const p1 = points[(i + len - 1) % len];
        const p2 = points[i];
        const p3 = points[(i + 1) % len];

        // Segment vectors in the transformed plane (p.z is Model Z, p.y is Model Y)
        let v1 = { z: p2.z - p1.z, y: p2.y - p1.y };
        let v2 = { z: p3.z - p2.z, y: p3.y - p2.y };

        // Handle boundaries for open paths
        if (isOpen) {
            if (i === 0) v1 = v2;
            if (i === len - 1) v2 = v1;
        }

        // Calculate segment normal vectors pointing outward.
        // For CW winding: normal is (-v.y, v.z)
        // For CCW winding: normal is (v.y, -v.z)
        let n1, n2;
        if (isCW) {
            n1 = { z: -v1.y, y: v1.z };
            n2 = { z: -v2.y, y: v2.z };
        } else {
            n1 = { z: v1.y, y: -v1.z };
            n2 = { z: v2.y, y: -v2.z };
        }

        const l1 = Math.sqrt(n1.z * n1.z + n1.y * n1.y);
        const l2 = Math.sqrt(n2.z * n2.z + n2.y * n2.y);

        // Average and normalize the two adjacent segment normals to get the vertex normal
        const n = {
            z: (n1.z / (l1 || 1) + n2.z / (l2 || 1)),
            y: (n1.y / (l1 || 1) + n2.y / (l2 || 1))
        };
        const ln = Math.sqrt(n.z * n.z + n.y * n.y);
        normals.push({ z: n.z / (ln || 1), y: n.y / (ln || 1) });
    }
    return normals;
}

export function getSectors(angles) {
    if (!angles || angles.length === 0) return [];
    
    // Sort angles in ascending order
    const sorted = [...angles].sort((a, b) => a - b);
    
    // Group angles into contiguous sectors (5-degree increments)
    const sectors = [];
    let currentSector = [sorted[0]];
    
    for (let i = 1; i < sorted.length; i++) {
        if (sorted[i] - sorted[i-1] === 5) {
            currentSector.push(sorted[i]);
        } else {
            sectors.push(currentSector);
            currentSector = [sorted[i]];
        }
    }
    sectors.push(currentSector);
    
    // Check if the first and last sectors wrap around and connect (355 to 0)
    if (sectors.length > 1) {
        const first = sectors[0];
        const last = sectors[sectors.length - 1];
        if (last[last.length - 1] === 355 && first[0] === 0) {
            // Merge last sector into first
            sectors[0] = [...last, ...first];
            sectors.pop();
        }
    }
    
    return sectors;
}

export function intersectSectors(secA, secB) {
    const setB = new Set(secB);
    const common = secA.filter(x => setB.has(x));
    if (common.length === 0) return null;
    
    // Group the intersection angles into contiguous sectors
    const commonSectors = getSectors(common);
    if (commonSectors.length === 0) return null;
    
    // If the intersection splits into multiple sectors, choose the largest one
    commonSectors.sort((a, b) => b.length - a.length);
    return commonSectors[0];
}

export function generateSegmentCandidates(points) {
    const len = points.length;
    const candidates = [];
    const coveredMDS = Array.from({ length: len }, () => new Set());
    
    // For each point, find all its MDS sectors
    const pointSectors = points.map(pt => getSectors(pt.mdr || []));
    
    // Iterate over all points and all their sectors to start candidate searches
    for (let i = 0; i < len; i++) {
        const sectors = pointSectors[i];
        if (sectors.length === 0) {
            // Point with no valid angles forms a 1-point fallback segment
            candidates.push({
                indices: [i],
                mds: []
            });
            continue;
        }
        
        for (let sIdx = 0; sIdx < sectors.length; sIdx++) {
            const sector = sectors[sIdx];
            const sectorKey = sector.join(',');
            
            // Skip if this (point, sector) has already been covered by an existing candidate
            if (coveredMDS[i].has(sectorKey)) continue;
            
            // Start a branching back-and-forth traverse
            const activeStates = [{
                indices: [i],
                mds: sector,
                forwardIdx: (i + 1) % len,
                backwardIdx: (i - 1 + len) % len,
                canExpandForward: true,
                canExpandBackward: true
            }];
            
            const completedCandidates = [];
            
            while (activeStates.length > 0) {
                const state = activeStates.shift();
                let expanded = false;
                
                // Try to expand forward
                if (state.canExpandForward && state.indices.length < len) {
                    const fIdx = state.forwardIdx;
                    if (!state.indices.includes(fIdx)) {
                        const fSectors = pointSectors[fIdx];
                        const overlaps = [];
                        
                        for (let fSec of fSectors) {
                            const intersect = intersectSectors(state.mds, fSec);
                            if (intersect) {
                                overlaps.push(intersect);
                            }
                        }
                        
                        if (overlaps.length > 0) {
                            // Branch the search for each overlapping sector option (over-segmentation)
                            for (let intersect of overlaps) {
                                activeStates.push({
                                    indices: [...state.indices, fIdx],
                                    mds: intersect,
                                    forwardIdx: (fIdx + 1) % len,
                                    backwardIdx: state.backwardIdx,
                                    canExpandForward: true,
                                    canExpandBackward: state.canExpandBackward
                                });
                            }
                            expanded = true;
                        }
                    }
                }
                
                // Try to expand backward
                if (!expanded && state.canExpandBackward && state.indices.length < len) {
                    const bIdx = state.backwardIdx;
                    if (!state.indices.includes(bIdx)) {
                        const bSectors = pointSectors[bIdx];
                        const overlaps = [];
                        
                        for (let bSec of bSectors) {
                            const intersect = intersectSectors(state.mds, bSec);
                            if (intersect) {
                                overlaps.push(intersect);
                            }
                        }
                        
                        if (overlaps.length > 0) {
                            // Branch the search for each overlapping sector option
                            for (let intersect of overlaps) {
                                activeStates.push({
                                    indices: [bIdx, ...state.indices],
                                    mds: intersect,
                                    forwardIdx: state.forwardIdx,
                                    backwardIdx: (bIdx - 1 + len) % len,
                                    canExpandForward: false, // already ran forward
                                    canExpandBackward: true
                                });
                            }
                            expanded = true;
                        }
                    }
                }
                
                if (!expanded) {
                    completedCandidates.push(state);
                }
            }
            
            // Log completed candidates and mark covered (point, sector) keys
            for (let cand of completedCandidates) {
                for (let idx of cand.indices) {
                    const pSectors = pointSectors[idx];
                    for (let pSec of pSectors) {
                        if (intersectSectors(cand.mds, pSec)) {
                            coveredMDS[idx].add(pSec.join(','));
                        }
                    }
                }
                
                candidates.push({
                    indices: cand.indices,
                    mds: cand.mds
                });
            }
        }
    }
    
    return candidates;
}

export function findMinimumCover(candidates, numPoints) {
    let bestCover = null;
    
    function search(currentCover, coveredSet) {
        if (coveredSet.size === numPoints) {
            if (!bestCover || currentCover.length < bestCover.length) {
                bestCover = [...currentCover];
            }
            return;
        }
        
        if (bestCover && currentCover.length >= bestCover.length) {
            return; // Pruning
        }
        
        // Find the first uncovered point on the circle
        let nextUncovered = -1;
        for (let i = 0; i < numPoints; i++) {
            if (!coveredSet.has(i)) {
                nextUncovered = i;
                break;
            }
        }
        
        if (nextUncovered === -1) return;
        
        // Find options covering this point
        const options = [];
        for (let cIdx = 0; cIdx < candidates.length; cIdx++) {
            const cand = candidates[cIdx];
            if (cand.indices.includes(nextUncovered)) {
                options.push(cand);
            }
        }
        
        // Sort options to prioritize those that cover the most uncovered points
        options.sort((a, b) => {
            const aUncovered = a.indices.filter(x => !coveredSet.has(x)).length;
            const bUncovered = b.indices.filter(x => !coveredSet.has(x)).length;
            return bUncovered - aUncovered;
        });
        
        for (let opt of options) {
            const nextCoveredSet = new Set(coveredSet);
            for (let idx of opt.indices) {
                nextCoveredSet.add(idx);
            }
            
            currentCover.push(opt);
            search(currentCover, nextCoveredSet);
            currentCover.pop();
        }
    }
    
    search([], new Set());
    return bestCover;
}

export function resolveOverlaps(chosenSegments, numPoints) {
    const assignments = new Array(numPoints).fill(-1);
    
    // Unique assignments
    for (let i = 0; i < numPoints; i++) {
        const coveringSegs = [];
        for (let sIdx = 0; sIdx < chosenSegments.length; sIdx++) {
            if (chosenSegments[sIdx].indices.includes(i)) {
                coveringSegs.push(sIdx);
            }
        }
        
        if (coveringSegs.length === 1) {
            assignments[i] = coveringSegs[0];
        }
    }
    
    // Resolve overlaps by assigning to the first covering segment
    for (let i = 0; i < numPoints; i++) {
        if (assignments[i] === -1) {
            const coveringSegs = [];
            for (let sIdx = 0; sIdx < chosenSegments.length; sIdx++) {
                if (chosenSegments[sIdx].indices.includes(i)) {
                    coveringSegs.push(sIdx);
                }
            }
            assignments[i] = coveringSegs[0];
        }
    }
    
    // Group back into segment definitions
    const resolved = [];
    for (let sIdx = 0; sIdx < chosenSegments.length; sIdx++) {
        const indices = [];
        for (let idx of chosenSegments[sIdx].indices) {
            if (assignments[idx] === sIdx) {
                indices.push(idx);
            }
        }
        if (indices.length > 0) {
            resolved.push({
                indices,
                mds: chosenSegments[sIdx].mds
            });
        }
    }
    
    return resolved;
}

export function getClosestAngleInMDS(targetAngle, mds) {
    if (!mds || mds.length === 0) return 0;
    let bestAngle = mds[0];
    let minDiff = Infinity;
    for (let angle of mds) {
        let diff = Math.abs(angle - targetAngle) % 360;
        if (diff > 180) diff = 360 - diff;
        if (diff < minDiff) {
            minDiff = diff;
            bestAngle = angle;
        }
    }
    return bestAngle;
}

export function getContiguousRuns(indices, len, isContourOpen) {
    if (indices.length === 0) return [];
    if (indices.length === 1) return [[indices[0]]];
    
    const runs = [];
    let currentRun = [indices[0]];
    
    for (let i = 1; i < indices.length; i++) {
        const prev = indices[i - 1];
        const curr = indices[i];
        
        let adjacent = false;
        if (isContourOpen) {
            adjacent = (curr === prev + 1);
        } else {
            adjacent = (curr === (prev + 1) % len);
        }
        
        if (adjacent) {
            currentRun.push(curr);
        } else {
            runs.push(currentRun);
            currentRun = [curr];
        }
    }
    runs.push(currentRun);
    
    // If closed contour and we have multiple runs, check if the last run connects to the first run
    if (!isContourOpen && runs.length > 1) {
        const firstRun = runs[0];
        const lastRun = runs[runs.length - 1];
        const firstVal = firstRun[0];
        const lastVal = lastRun[lastRun.length - 1];
        if ((lastVal + 1) % len === firstVal) {
            // Merge last run into first run: prepend lastRun elements to firstRun
            runs[0] = [...lastRun, ...firstRun];
            runs.pop();
        }
    }
    
    return runs;
}

export async function generate(opt) {
    return new FourAxis().generate(opt);
}
