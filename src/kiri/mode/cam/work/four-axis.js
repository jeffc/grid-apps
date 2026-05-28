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
        const resampleDist = resolution * 2; // N units
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

                const resampled = resamplePolygon(poly, resampleDist);
                if (resampled) {
                    resampledPolys.push(resampled);
                    const normals = computeNormals(resampled);
                    for (let i = 0; i < resampled.points.length; i++) {
                        const p = resampled.points[i];
                        const n = normals[i];
                        p.mdr = mach.getMDRs(p, n);
                    }
                    pOut += resampled.points.length;
                }
            }
            if (resampledPolys.length > 0) {
                if (isEvery100) console.log(`  slice z=${slice.z.round(2)} index=${slice.index} pIn=${pIn} pOut=${pOut}`);
                resampledSlice.addTops(resampledPolys);
                resampledSlice.camLines = resampledPolys;
                if (isEvery100) {
                    const layers = resampledSlice.output();
                    const shiftedContours = resampledPolys.map(poly => poly.clone().move({ z: -this.zoff, x: 0, y: 0 }));
                    layers.setLayer("fouraxis-contours", { line: 0x00ff00 })
                        .addPolys(shiftedContours, { thin: true });

                    const mdrPolys = [];
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
                    }
                    if (mdrPolys.length > 0) {
                        layers.setLayer("fouraxis-machinable", { line: 0xff0000 })
                            .addPolys(mdrPolys, { thin: true });
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
        // We extract the physical dimensions to perform exact collision tests.
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

        // For tapered ball mills, find the transition point from sphere to cone
        if (isTaperBall) {
            const { r, b } = tool.getBallTaperParams();
            this.tool.ballR = r;
            this.tool.ballB = b;
        }

        // 2. Coordinate System Synchronization
        // Kiri:Moto slices normal to the X axis by swapping X and Z before slicing.
        // Post-slice Point state:
        // p.x = Original Model X (the rotation axis, constant for the slice)
        // p.y = Original Model Y
        // p.z = Original Model Z + offset (the depth/height coordinate)
        // Our analysis happens in the (p.z, p.y) plane (Model Z, Model Y).
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
        // Store segments as (z, y) pairs for rapid analytical testing.
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
     *
     * @param {Point} p Contact Center (CC) in (z, y)
     * @param {Point} n Outward Normal in (z, y). n.z is the Z-component, n.y is the Y-component.
     * @param {number} angle Machine rotation (degrees). 0 is home, positive is CCW rotation around X.
     * @returns {boolean} True if orientation is collision-free
     */
    isMachinable(p, n, angle) {
        const rad = (angle * Math.PI) / 180;
        const cos = Math.cos(rad);
        const sin = Math.sin(rad);

        // 1. Tool Ray Direction (d_l) in Part Space
        // In the (Z, Y) slice plane, a machine "UP" ray (which is constant in machine space)
        // rotates with the part. For a part rotation of 'a' degrees CCW:
        // The ray direction in part space is (cos a, -sin a) due to CW relative rotation.
        // dZ_model = cos(a), dY_model = -sin(a)
        const dz = cos;
        const dy = -sin;

        const R = this.tool.tipRad;

        // 2. Surface Feasibility Check (Back-face Culling)
        // The tool axis must point "away" from the material.
        // We check the dot product between the tool ray (dz, dy) and the surface normal (n.z, n.y).
        const dot = dz * n.z + dy * n.y;
        if (dot < -0.001) return false;

        // 3. Cutter Location (CL) calculation
        // For ball-end mills, the "Cutter Location" is the center of the tip sphere.
        // The spherical tip is tangent to the part at the Contact Center (CC).
        // CL = CC + R * n
        const CL = { z: p.z + n.z * R, y: p.y + n.y * R };

        // 4. Lateral Axis (Perpendicular to tool ray)
        // A vector perpendicular to (dz, dy) is (-dy, dz).
        const perpZ = -dy;
        const perpY = dz;

        const eps = 0.01;
        const chordEps = 0.1; // Tolerance for model's piecewise-linear approximation
        const Rsq = (R - chordEps) * (R - chordEps);

        for (let seg of this.segments) {
            // Segment endpoints in the analysis plane (Model Z, Model Y)
            const s1 = { z: seg.p1.z, y: seg.p1.y };
            const s2 = { z: seg.p2.z, y: seg.p2.y };

            // 6. Ball Volume Collision Test
            // We check the distance from the tip center (CL) to the segment.
            const dxs = s2.z - s1.z;
            const dys = s2.y - s1.y;
            const lensq_seg = dxs * dxs + dys * dys;
            let ts = lensq_seg > 0 ? ((CL.z - s1.z) * dxs + (CL.y - s1.y) * dys) / lensq_seg : 0;
            ts = Math.max(0, Math.min(1, ts));
            const closestZ = s1.z + ts * dxs;
            const closestY = s1.y + ts * dys;
            const distSq = (closestZ - CL.z) ** 2 + (closestY - CL.y) ** 2;
            if (R > eps && distSq < Rsq) return false;

            // 7. Shaft Volume Collision Test
            // We project the segment into the tool's local 2D space (origin at CL, tool axis along Z).
            const v1 = { z: s1.z - CL.z, y: s1.y - CL.y };
            const v2 = { z: s2.z - CL.z, y: s2.y - CL.y };

            const sz1 = v1.z * dz + v1.y * dy;       // Depth along tool axis
            const sx1 = v1.z * perpZ + v1.y * perpY; // Lateral distance from tool axis
            const sz2 = v2.z * dz + v2.y * dy;
            const sx2 = v2.z * perpZ + v2.y * perpY;

            const checkPoint = (depth, lateral) => {
                if (depth < eps) return true; // Ignore parts of the model "behind" the tip center
                // A collision occurs if the model segment enters the tool's radius at that depth.
                return Math.abs(lateral) > this.getRadius(depth + R) - eps;
            };

            // Check segment endpoints and midpoint
            if (!checkPoint(sz1, sx1)) return false;
            if (!checkPoint(sz2, sx2)) return false;
            if (!checkPoint((sz1 + sz2) / 2, (sx1 + sx2) / 2)) return false;

            // Check the critical point (point on the segment closest to the tool axis)
            const dsx = sx2 - sx1;
            const dsz = sz2 - sz1;
            if (Math.abs(dsx) > 1e-9) {
                // Find parameter t where lateral distance sx crosses zero
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

    // Compute signed area in (Z, Y) plane to determine winding
    // In our swapped points: p.z = Model Z, p.y = Model Y
    let area2 = 0;
    for (let i = 0; i < len; i++) {
        const p1 = points[i];
        const p2 = points[(i + 1) % len];
        area2 += (p2.z - p1.z) * (p2.y + p1.y);
    }
    const isCW = area2 > 0;

    const normals = [];
    const isOpen = poly.open || (poly.isOpen ? poly.isOpen() : false);

    for (let i = 0; i < len; i++) {
        const p1 = points[(i + len - 1) % len];
        const p2 = points[i];
        const p3 = points[(i + 1) % len];

        // Segment vectors in transformed plane (p.z is Model Z, p.y is Model Y)
        let v1 = { z: p2.z - p1.z, y: p2.y - p1.y };
        let v2 = { z: p3.z - p2.z, y: p3.y - p2.y };

        if (isOpen) {
            if (i === 0) v1 = v2;
            if (i === len - 1) v2 = v1;
        }

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

        const n = {
            z: (n1.z / (l1 || 1) + n2.z / (l2 || 1)),
            y: (n1.y / (l1 || 1) + n2.y / (l2 || 1))
        };
        const ln = Math.sqrt(n.z * n.z + n.y * n.y);
        normals.push({ z: n.z / (ln || 1), y: n.y / (ln || 1) });
    }
    return normals;
}

export async function generate(opt) {
    return new FourAxis().generate(opt);
}
