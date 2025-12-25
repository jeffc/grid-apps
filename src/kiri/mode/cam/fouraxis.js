/** Copyright Stewart Allen <sa@grid.space> -- All Rights Reserved */

import { base } from '../../../geo/base.js';
import { newPoint } from '../../../geo/point.js';
import { newPolygon } from '../../../geo/polygon.js';

const RAD2DEG = 180 / Math.PI;
const DEG2RAD = Math.PI / 180;

export async function generateFourAxis(params) {
    const { sliced, tool, zoff, leave, linear, lineColor, onupdate } = params;

    console.log(`Four axis slicing: ${sliced.length} slices`);

    /* TODO - re-home this function */
    /**
     * Resample a closed contour so points are evenly spaced.
     *
     * @param {Polygon} poly - input contour (must be closed)
     * @param {number} spacing - desired spacing between samples (units of pts)
     * @param {boolean} includeOriginals - whether to also include the
     *     original points in the output.
     * @returns {Polygon} contour with resampled points (new points, does not mutate input)
     */
    let resampleClosedContour = (poly, spacing, includeOriginals = true) => {
    if (!poly || !poly.points) return newPolygon();
    const pts = poly.points;
    if (!Array.isArray(pts) || pts.length === 0) return [];

    if (spacing <= 0) throw new Error("spacing must be > 0");

    // helper: distance between two points
    const dist3 = (a, b) => Math.sqrt(Math.pow(b.x - a.x, 2) +  Math.pow(b.y - a.y, 2) + Math.pow(b.z - a.z, 2));

    // Build segment list
    const segs = [];
    const n = pts.length;
    if (n === 1) return [ { x: pts[0].x, y: pts[0].y, z: pts[0].z } ];

    for (let i = 0; i < n - 1; i++) {
        segs.push({ a: pts[i], b: pts[i + 1], len: dist3(pts[i], pts[i + 1]) });
    }
    segs.push({ a: pts[n - 1], b: pts[0], len: dist3(pts[n - 1], pts[0]) });

    const total = segs.reduce((s, x) => s + x.len, 0);
    if (total === 0) return [ { x: pts[0].x, y: pts[0].y, z: pts[0].z } ];

    // If spacing larger than length:
    if (spacing >= total) {
        // Return one representative point or optionally return original first point
        return [ { x: pts[0].x, y: pts[0].y, z: pts[0].z } ];
    }

    // Determine sample positions along length (distances from start)
    let sampleCount = Math.floor(total / spacing) + 1; // includes the 0 position

    let positions = new Array();;
    for (let i = 0; i < sampleCount; i++) {
        positions.push(i * spacing);
    }

    // if we're including original points, add them in too.
    if (includeOriginals) {
        let accum = 0;
        for (let seg of segs) {
        accum += seg.len;
        positions.push(accum);
        }
        positions = positions.sort((a,b) => a-b).filter((x, i, a) => a.indexOf(x) === i);
    }

    // Interpolate samples
    const result = [];
    let segIndex = 0;
    let segAccum = 0; // length before current segment
    for (let pos of positions) {
        // clamp for numeric round-off
        if (pos > total) pos = total;

        // Advance segIndex until pos is on current segment
        while (segIndex < segs.length && segAccum + segs[segIndex].len < pos - 1e-12) {
        segAccum += segs[segIndex].len;
        segIndex++;
        }
        // If we've gone beyond last seg (can happen due to rounding), clamp to last
        if (segIndex >= segs.length) segIndex = segs.length - 1;

        const seg = segs[segIndex];
        const segLen = seg.len || 1e-12;
        const t = Math.max(0, Math.min(1, (pos - segAccum) / segLen));
        // linear interpolation of x,y,z
        const x = seg.a.x + (seg.b.x - seg.a.x) * t;
        const y = seg.a.y + (seg.b.y - seg.a.y) * t;
        const z = seg.a.z + (seg.b.z - seg.a.z) * t;

        result.push({ x, y, z });
    }

    // check to make sure we didn't duplicate the first and last point
    const pFirst = result[0];
    const pLast = result[result.length - 1];
    if (pFirst.x == pLast.x && pFirst.y == pLast.y && pFirst.z == pLast.z) {
        result.pop();
    }

    let outPoly = newPolygon();
    outPoly.addPoints(result.map((p) => newPoint(p.x, p.y, p.z)));
    outPoly.setClosed();
    return outPoly;
    };

    /**
     * Converts an MDR (Machining Direction Range) which may have wrap-around ranges
     * (e.g., [350, 10]) into a set of non-wrapping ranges.
     * @param {number[][]} mdr - Array of [start, end] angle ranges.
     * @returns {number[][]} An array of normalized [start, end] ranges.
     */
    const normalizeMDR = (mdr) => {
        if (!mdr) return [];
        const normalized = [];
        for (const range of mdr) {
            const [start, end] = range;
            if (start <= end) {
                normalized.push(range);
            } else {
                normalized.push([start, 360]);
                normalized.push([0, end]);
            }
        }
        return normalized;
    };

    /**
     * Calculates the intersection of two non-wrapping angle ranges.
     * @param {number[]} r1 - First range [start, end].
     * @param {number[]} r2 - Second range [start, end].
     * @returns {number[]|null} The intersection range, or null if no intersection.
     */
    const intersectRanges = (r1, r2) => {
        const s = Math.max(r1[0], r2[0]);
        const e = Math.min(r1[1], r2[1]);
        if (s < e) {
            return [s, e];
        }
        return null;
    };

    /**
     * Calculates the intersection of two MDRs.
     * @param {number[][]} mdr1 - First MDR.
     * @param {number[][]} mdr2 - Second MDR.
     * @returns {number[][]} A new MDR representing the intersection.
     */
    const intersectMDRs = (mdr1, mdr2) => {
        const norm1 = normalizeMDR(mdr1);
        const norm2 = normalizeMDR(mdr2);
        const intersection = [];
        for (const r1 of norm1) {
            for (const r2 of norm2) {
                const res = intersectRanges(r1, r2);
                if (res) {
                    intersection.push(res);
                }
            }
        }
        return intersection;
    };

    /**
     * Calculates the union of two MDRs.
     * @param {number[][]} mdr1 - First MDR.
     * @param {number[][]} mdr2 - Second MDR.
     * @returns {number[][]} A new MDR representing the union.
     */
    const unionMDRs = (mdr1, mdr2) => {
        const norm1 = normalizeMDR(mdr1);
        const norm2 = normalizeMDR(mdr2);
        const all = [...norm1, ...norm2];
        if (all.length === 0) return [];
        all.sort((a, b) => a[0] - b[0]);

        const merged = [all[0]];
        for (let i = 1; i < all.length; i++) {
            const last = merged[merged.length - 1];
            const current = all[i];
            if (current[0] <= last[1]) {
                last[1] = Math.max(last[1], current[1]);
            } else {
                merged.push(current);
            }
        }

        if (merged.length > 1) {
            const first = merged[0];
            const last = merged[merged.length - 1];
            if (last[1] === 360 && first[0] === 0) {
                merged[0] = [last[0], first[1]];
                merged.pop();
            }
        }
        return merged;
    }

    /**
     * Calculates the total angular length of an MDR.
     * @param {number[][]} mdr - The MDR to measure.
     * @returns {number} The total length in degrees.
     */
    const totalLength = (mdr) => {
        if (!mdr) return 0;
        return mdr.reduce((sum, range) => sum + range[1] - range[0], 0);
    };

    /**
     * Calculates the similarity between two MDRs, defined as the ratio of
     * the length of their intersection to the length of their union.
     * @param {number[][]} mdr1 - First MDR.
     * @param {number[][]} mdr2 - Second MDR.
     * @returns {number} Similarity score between 0 and 1.
     */
    const calculateSimilarity = (mdr1, mdr2) => {
        const intersection = intersectMDRs(mdr1, mdr2);
        const union = unionMDRs(mdr1, mdr2);
        const intersectionLength = totalLength(intersection);
        const unionLength = totalLength(union);
        if (unionLength === 0) {
            return 1;
        }
        return intersectionLength / unionLength;
    };

    let sidx = 0; // TODO - remove (debugging)
    for (const slice of sliced) {
    console.log(`${sidx} / ${sliced.length}`); // TODO - replace with proper progress callback
    // The 'slice.tops' property contains an array of Polygon objects
    const contours = slice.tops;

    sidx++; // TODO - remove (debugging)

    // if this slice doesn't have any, keep going
    if (!contours || contours.length === 0) {
        continue;
    }

    // resample contours with a uniform spacing
    const resampledContours = contours.map((con) => {
        const c = con.poly;
        // TODO - change 5 to configurable resample distance
        return resampleClosedContour(c, 5);
    });

    // for machinability computations, we're going to need to rotate all of
    // the contours about the x axis by every integer number of degrees. do
    // that once.
    let contoursRotatedCache = [...Array(360).keys()].map((theta) => {
        return contours.map((con) => {
        let c = con.poly.clone(true);
        let cRot = c.rotateYZ(theta);
        return cRot;
        });
    });

    /**
     * Compute whether a point is machinable at the given angle.
     * @param {Point} p - The point to test
     * @param {Point} vnorm - The vertex normal vector at the given point
     * @param {Number} angle - The angle of rotation (CCW about the X axis)
     * @return true or false, whether the point is machinable
     */
    const isMachinable = (p, vnorm, angle) => {
        // find the location for the tip of the cutting tool
        // TODO - take into account the actual geometry of the tool
        let toolTip = p.clone().add(vnorm.clone().normalize().scale(0.1, 0.1, 0.1));
        toolTip.rotateYZ(angle * DEG2RAD);
        toolTip.swapXZ();
        let rayDirection = newPoint(0, 0, 1).swapXZ();

        for (let c of contoursRotatedCache[angle]) {
        for (let pidx = 0; pidx < c.points.length; pidx++) {
            let swappedp1 = c.points[pidx].clone().swapXZ();
            let swappedp2 = c.points[(pidx+1) % c.points.length].clone().swapXZ();
            let intersects = base.util.intersectRayLine(
            toolTip, { dx: rayDirection.x, dy: rayDirection.y }, swappedp1, swappedp2);
            if (intersects && intersects.dist > 1e-6) {
            return false;
            }
        }
        }
        return true;
    };

    slice.segments = [];
    for (const contour of resampledContours) {
        // skip degenerate/empty contours
        if (!contour.points) {
        continue;
        }
        const nPoints = contour.points.length;
        for (let i = 0; i < nPoints; i++) {
        // calculate machinable direction range (MDR) for each point
        const point = contour.points[i];
        const prevPoint = contour.points[ (i + nPoints - 1) % nPoints];
        const nextPoint = contour.points[ (i+1) % nPoints];

        // use the Point class to represent a 2d vector here
        const incomingEdge = point.sub(prevPoint);
        const outgoingEdge = nextPoint.sub(point);

        // We look at the previous and next points and calculate the angle
        // between the current point and each one. We search for valid
        // machinable directions between those two angles, since angles
        // beyond either will definitely not be machinable

        // remember that we've sliced along the X axis, so consider angles
        // as (y, z)
        const prevCheckAngle = (Math.round(Math.atan2(-(incomingEdge.z), -(incomingEdge.y)) * RAD2DEG) + 360) % 360;
        const nextCheckAngle = (Math.round(Math.atan2(outgoingEdge.z, outgoingEdge.y) * RAD2DEG) + 360) % 360;

        // compute the vertex normal vector. we rotate each vector by 90
        // degrees so that it becomes normal to the edge it was pointing
        // along, then average the two.
        //
        // Since the point class methods mutate in place, encapsulate the
        // logic into a small lambda and immediately call it to avoid
        // leaking lots of temporary variables into the scope.
        const incomingEdgeNormal = (() => { let p = incomingEdge.clone(); p.rotateYZ(-90*DEG2RAD); p.normalize(); return p; })();
        const outgoingEdgeNormal = (() => { let p = outgoingEdge.clone(); p.rotateYZ(-90*DEG2RAD); p.normalize(); return p; })();
        const vertexNormal = (() => { let p = incomingEdgeNormal.add(outgoingEdgeNormal); p.normalize(); return p; })();
        if(sidx % 200 == 0) {
            slice.output().setLayer("machinability-normals", {line: 0xFF00FF}).
            addPoly(newPolygon([point, point.add(vertexNormal)]));
        }

        let MDR = [];
        let firstValid = null;
        for (let angle = prevCheckAngle; angle != nextCheckAngle; angle = (angle + 1) % 360) {
            // check if the given angle is a machinable direction
            let vec = newPoint(0, Math.cos(angle*DEG2RAD), Math.sin(angle*DEG2RAD));
            if (isMachinable(point, vertexNormal, (360 - angle + 90) % 360)) {
            let MDR = [];
            let firstValid = null;
            let currentAngle = prevCheckAngle;
            while (true) {
                let angle = currentAngle;

                if (isMachinable(point, vertexNormal, (360 - angle + 90) % 360)) {
                if(sidx % 200 == 0 && i % 100 == 0) {
                    let vec = newPoint(0, Math.cos(angle*DEG2RAD), Math.sin(angle*DEG2RAD));
                    slice.output().setLayer("machinability", {line: lineColor}).
                        addPoly(newPolygon([point, point.add(vec)]));
                }
                if (firstValid === null) {
                    firstValid = angle;
                }
                } else {
                if (firstValid !== null) {
                    MDR.push([firstValid, (angle - 1 + 360) % 360]);
                    firstValid = null;
                }
                }

                if (angle === nextCheckAngle) {
                if (firstValid !== null) {
                    MDR.push([firstValid, (angle - 1 + 360) % 360]);
                }
                break;
                }
                currentAngle = (currentAngle + 1) % 360;
            }
            point.MDR = MDR;
            }

            // --- start of graph-cut implementation ---
            const points = contour.points;
            const numPoints = points.length;
            if (numPoints < 2) {
            continue;
            }

            // 1. Build graph edges with weights based on MDR similarity
            const edges = [];
            for (let i = 0; i < numPoints; i++) {
            const p1 = points[i];
            const p2 = points[(i + 1) % numPoints];
            const similarity = calculateSimilarity(p1.MDR, p2.MDR);
            const weight = 1 - similarity;
            edges.push({
                from: i,
                to: (i + 1) % numPoints,
                weight: weight
            });
            }

            // 2. Greedily cut edges with high weights (low similarity)
            const cutThreshold = 0.5; // TODO: make this configurable
            const adj = new Map();
            for (let i = 0; i < numPoints; i++) adj.set(i, []);

            for (const edge of edges) {
            if (edge.weight <= cutThreshold) {
                adj.get(edge.from).push(edge.to);
                adj.get(edge.to).push(edge.from);
            }
            }

            // 3. Find connected components which form the new segments
            const visited = new Array(numPoints).fill(false);
            const segments = [];
            for (let i = 0; i < numPoints; i++) {
            if (!visited[i]) {
                const component_indices = [];
                const q = [i];
                visited[i] = true;
                let head = 0;
                // Standard breadth-first search to find all nodes in the component
                while(head < q.length) {
                    const u = q[head++];
                    component_indices.push(u);
                    if (adj.has(u)) {
                        for (const v of adj.get(u)) {
                            if (!visited[v]) {
                                visited[v] = true;
                                q.push(v);
                            }
                        }
                    }
                }

                if (component_indices.length > 0) {
                    // The component indices need to be ordered to form a path
                    const ordered_segment_indices = [];
                    let start_node = -1;
                    if (component_indices.length === 1) {
                        start_node = component_indices[0];
                    } else {
                        // Find an endpoint of the path (a node with degree 1)
                        for(const node_idx of component_indices) {
                            if (adj.get(node_idx).length <= 1) {
                                start_node = node_idx;
                                break;
                            }
                        }
                    }
                    if (start_node === -1) {
                        // This case happens if the segment is a closed loop
                        start_node = component_indices[0];
                    }

                    // Traverse the path from the start node to order the points
                    const path_q = [start_node];
                    const path_visited = new Set([start_node]);
                    while(path_q.length > 0) {
                        const u = path_q.shift();
                        ordered_segment_indices.push(u);
                        if (adj.has(u)) {
                            for (const v of adj.get(u)) {
                                if (!path_visited.has(v)) {
                                    path_visited.add(v);
                                    path_q.push(v);
                                }
                            }
                        }
                    }
                    const segment_points = ordered_segment_indices.map(idx => points[idx]);
                    segments.push(newPolygon().addPoints(segment_points).setOpen());
                }
            }
            }
            slice.segments.push(...segments);
        }
        }
    }
    }
    return sliced;
}
