/**
 * @file
 *
 * This file contains the logic for 4-axis machining path generation.
 *
 * The core of this process is determining the "Machinable Direction Range" (MDR)
 * for each point on a contour. The MDR is the set of angles from which a tool
 * can access a point without colliding with other parts of the model on the same Z-slice.
 *
 * --- The Performance Challenge ---
 *
 * A naive approach to calculating the MDR involves, for each point, checking all
 * 360 possible tool approach angles. For each angle, a ray-intersection test
 * is performed against every other line segment on the slice to check for
 * collisions. This results in a high-complexity algorithm (roughly O(N^2)) that
 * is too slow for complex models.
 *
 * --- The Spatial Grid Solution ---
 *
 * To solve this, we invert the process and use a spatial acceleration data
 * structure (a 2D grid). The algorithm is as follows:
 *
 * 1. Loop through each of the 360 tool angles first.
 *
 * 2. For each angle:
 *    a. Rotate all contours on the slice by that angle.
 *    b. Project the rotated contours onto the YZ-plane, creating 2D segments.
 *    c. Build a 2D `SpatialGrid` and insert all of the 2D line segments into it.
 *
 * 3. With the grid for the current angle built, loop through each point that needs
 *    to be checked for machinability.
 *
 * 4. The `isMachinable` check now becomes much faster:
 *    a. The function receives the pre-built grid for the current angle.
 *    b. It projects the 3D `toolTip` point into a 2D `ray_origin`.
 *    c. Instead of checking against all segments, it performs a `queryRay()`
 *       on the grid. This query traverses the grid cells along the path of the
 *       tool's ray and returns only the 2D segments in those cells.
 *    d. The expensive line intersection test is then only performed on this
 *       small subset of candidate segments.
 *
 * This changes the complexity of the collision check from O(N) to O(log N) or
 * O(1) on average, resulting in a significant performance improvement.
 */

import { base } from '../../../geo/base.js';
import { newPoint } from '../../../geo/point.js';
import { newPolygon } from '../../../geo/polygon.js';

const RAD2DEG = 180 / Math.PI;
const DEG2RAD = Math.PI / 180;

/**
 * A purely 2D spatial grid for fast line segment lookups.
 * Expects all inputs (bounds, segments, points) to have {x, y} properties.
 */
class SpatialGrid {
    constructor(bounds, cellSize) {
        this.bounds = bounds;
        this.cellSize = cellSize > 0 ? cellSize : 1.0;
        this.grid = [];
        this.cols = Math.ceil((bounds.max.x - bounds.min.x) / this.cellSize) || 1;
        this.rows = Math.ceil((bounds.max.y - bounds.min.y) / this.cellSize) || 1;
        for (let i = 0; i < this.cols * this.rows; i++) {
            this.grid.push([]);
        }
    }

    _getCells(segment) {
        const [p1, p2] = segment;
        const b = {
            min: { x: Math.min(p1.x, p2.x), y: Math.min(p1.y, p2.y) },
            max: { x: Math.max(p1.x, p2.x), y: Math.max(p1.y, p2.y) }
        };

        const minX = Math.floor((b.min.x - this.bounds.min.x) / this.cellSize);
        const maxX = Math.floor((b.max.x - this.bounds.min.x) / this.cellSize);
        const minY = Math.floor((b.min.y - this.bounds.min.y) / this.cellSize);
        const maxY = Math.floor((b.max.y - this.bounds.min.y) / this.cellSize);

        const cells = [];
        for (let y = Math.max(0, minY); y <= Math.min(this.rows - 1, maxY); y++) {
            for (let x = Math.max(0, minX); x <= Math.min(this.cols - 1, maxX); x++) {
                cells.push(y * this.cols + x);
            }
        }
        return cells;
    }

    insert(segment) {
        this._getCells(segment).forEach(idx => {
            this.grid[idx].push(segment);
        });
    }

    queryRay(ray_origin) {
        // Assumes a horizontal ray in the +x direction, as is our use case.
        const candidates = new Set();
        const startX = Math.floor((ray_origin.x - this.bounds.min.x) / this.cellSize);
        const startY = Math.floor((ray_origin.y - this.bounds.min.y) / this.cellSize);

        if (startX >= this.cols || startY < 0 || startY >= this.rows) {
            return [];
        }

        // Traverse grid cells horizontally along the ray path
        for (let x = Math.max(0, startX); x < this.cols; x++) {
            const y = startY;
            const idx = y * this.cols + x;
            if (this.grid[idx]) {
                this.grid[idx].forEach(seg => candidates.add(seg));
            }
        }
        return [...candidates];
    }
}

/**
 * Given a point and a grid containing rotated geometry, checks if a tool can
 * access the point without colliding with other geometry on the same slice.
 */
function isMachinable(p, vnorm, angle, grid) {
    // TODO - consider actual tool geometry here
    let toolTip = p.clone().add(vnorm.clone().normalize().scale(0.1, 0.1, 0.1));
    toolTip.rotateYZ(angle * DEG2RAD);

    // project the 3D tool tip to a 2D ray origin for the grid query
    const ray_origin = { x: toolTip.z, y: toolTip.y };

    const candidates = grid.queryRay(ray_origin);

    if (candidates.length === 0) {
        return true;
    }

    const ray_direction = { dx: 1, dy: 0 }; // ray fires along the +Z axis in model space

    for (let seg of candidates) {
        // segment is already a 2D {x,y} pair
        let intersects = base.util.intersectRayLine(ray_origin, ray_direction, seg[0], seg[1]);

        if (self.debug_isMachinable && intersects) {
            console.log({
                msg: "intersection found",
                intersects,
                is_collision: intersects.dist > 1e-6
            });
        }

        // if ray hits a segment that is "in front" of the tool tip, count it
        if (intersects && intersects.dist > 1e-6) {
          return false;
        }
    }

  return true;
}

function resampleClosedContour(poly, spacing) {
    if (!poly || !poly.points) return newPolygon();
    const pts = poly.points;
    if (!Array.isArray(pts) || pts.length === 0) return newPolygon();
    if (spacing <= 0) throw new Error("spacing must be > 0");

    const dist3 = (a, b) => Math.sqrt(Math.pow(b.x - a.x, 2) + Math.pow(b.y - a.y, 2) + Math.pow(b.z - a.z, 2));

    const segs = [];
    const n = pts.length;
    if (n === 1) return newPolygon().addPoints([pts[0].clone()]);

    for (let i = 0; i < n; i++) {
        const p1 = pts[i];
        const p2 = pts[(i + 1) % n];
        segs.push({ a: p1, b: p2, len: dist3(p1, p2) });
    }

    const total = segs.reduce((s, x) => s + x.len, 0);
    if (total === 0) return newPolygon().addPoints([pts[0].clone()]);
    // The user prefers a higher resample distance during debugging, so I'm commenting out this line.
    // if (spacing >= total) return newPolygon().addPoints([pts[0].clone()]);

    const sampleCount = Math.floor(total / spacing);
    const result = [];
    let segIndex = 0;
    let segAccum = 0;
    for (let i = 0; i < sampleCount; i++) {
        const pos = i * spacing;
        while (segIndex < segs.length && segAccum + segs[segIndex].len < pos - 1e-12) {
            segAccum += segs[segIndex].len;
            segIndex++;
        }
        if (segIndex >= segs.length) segIndex = segs.length - 1;

        const seg = segs[segIndex];
        const segLen = seg.len || 1e-12;
        const t = Math.max(0, Math.min(1, (pos - segAccum) / segLen));
        result.push(newPoint(
            seg.a.x + (seg.b.x - seg.a.x) * t,
            seg.a.y + (seg.b.y - seg.a.y) * t,
            seg.a.z + (seg.b.z - seg.a.z) * t
        ));
    }

    return newPolygon().addPoints(result).setClosed();
}

function normalizeMDR(mdr) {
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

function intersectRanges(r1, r2) {
    const s = Math.max(r1[0], r2[0]);
    const e = Math.min(r1[1], r2[1]);
    return s < e ? [s, e] : null;
};

function intersectMDRs(mdr1, mdr2) {
    const norm1 = normalizeMDR(mdr1);
    const norm2 = normalizeMDR(mdr2);
    const intersection = [];
    for (const r1 of norm1) {
        for (const r2 of norm2) {
            const res = intersectRanges(r1, r2);
            if (res) intersection.push(res);
        }
    }
    return intersection;
};

function unionMDRs(mdr1, mdr2) {
    const all = [...normalizeMDR(mdr1), ...normalizeMDR(mdr2)];
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

function totalLength(mdr) {
    if (!mdr) return 0;
    return normalizeMDR(mdr).reduce((sum, range) => sum + range[1] - range[0], 0);
};

function calculateSimilarity(mdr1, mdr2) {
    const intersectionLength = totalLength(intersectMDRs(mdr1, mdr2));
    const unionLength = totalLength(unionMDRs(mdr1, mdr2));
    return unionLength === 0 ? 1 : intersectionLength / unionLength;
};

function convertBoolsToMDR(machinable) {
    const mdr = [];
    let start = -1;
    for (let i = 0; i < 361; i++) {
        const angle = i % 360;
        if (machinable[angle] && start === -1) {
            start = angle;
        } else if (!machinable[angle] && start !== -1) {
            mdr.push([start, angle === 0 ? 360 : angle]);
            start = -1;
        }
    }
    if (start !== -1) {
        mdr.push([start, 360]);
    }
    // merge ranges that cross the 0/360 boundary
    if (mdr.length > 1 && mdr[0][0] === 0 && mdr[mdr.length-1][1] === 360) {
        mdr[mdr.length - 1][1] = mdr[0][1];
        mdr.shift();
    }
    return mdr;
}

function extrapolateMachinability(sparseMachinable, step) {
    const fullMachinable = new Array(360).fill(false);
    for (let i = 0; i < 360; i += step) {
        if (sparseMachinable[i]) {
            fullMachinable[i] = true;
        }
    }

    for (let i = 0; i < 360; i += step) {
        const currentAngle = i;
        const nextAngle = (i + step) % 360;

        // If both current and next step are machinable, fill in between
        if (fullMachinable[currentAngle] && fullMachinable[nextAngle]) {
            for (let j = 1; j < step; j++) {
                fullMachinable[(currentAngle + j) % 360] = true;
            }
        }
    }
    return fullMachinable;
}

export async function generateFourAxis(params) {
    const { sliced, onupdate, lineColor } = params;

    console.log(`Four axis slicing: ${sliced.length} slices`);

    const angleStep = 5; // User-defined angle step

    let sidx = 0;
    for (const slice of sliced) {
        onupdate(sidx++ / sliced.length, `slice ${sidx}`);
        const contours = slice.tops;
        if (!contours || contours.length === 0) {
            continue;
        }

        // 1. Resample contours and pre-calculate normals
        const resampledContours = contours.map(con => resampleClosedContour(con.poly, 10));
        for (const contour of resampledContours) {
            const nPoints = contour.points.length;
            if (nPoints === 0) continue;
            for (let i = 0; i < nPoints; i++) {
                const point = contour.points[i];
                const prevPoint = contour.points[(i + nPoints - 1) % nPoints];
                const nextPoint = contour.points[(i + 1) % nPoints];

                const incomingEdge = point.clone().sub(prevPoint);
                const outgoingEdge = nextPoint.clone().sub(point);

                if (incomingEdge.magnitude() === 0 || outgoingEdge.magnitude() === 0) {
                    point._4axis = { machinable: new Array(360).fill(false), vnorm: newPoint(0,0,1) };
                    continue;
                }

                const incomingEdgeNormal = incomingEdge.clone().rotateYZ(-90 * DEG2RAD).normalize();
                const outgoingEdgeNormal = outgoingEdge.clone().rotateYZ(-90 * DEG2RAD).normalize();
                const vertexNormal = incomingEdgeNormal.add(outgoingEdgeNormal).normalize();

                if (isNaN(vertexNormal.x)) {
                    vertexNormal.copy(outgoingEdgeNormal);
                }

                point._4axis = { machinable: new Array(360).fill(false), vnorm: vertexNormal };

                if (sidx % 200 === 0) {
                    slice.output().setLayer("machinability-normals", {line: 0xFF00FF})
                        .addPoly(newPolygon([point, point.clone().add(vertexNormal)]));
                }
            }
        }

        // 2. Iterate by angle, building a spatial grid for each
        for (let angle = 0; angle < 360; angle += angleStep) { // Use angleStep here
            const bounds = { min: { x: Infinity, y: Infinity }, max: { x: -Infinity, y: -Infinity } };
            const rotatedPolys = resampledContours.map(poly => {
                const p = poly.clone(true).rotateYZ(angle);
                p.points.forEach(pt => {
                    bounds.min.x = Math.min(bounds.min.x, pt.z);
                    bounds.min.y = Math.min(bounds.min.y, pt.y);
                    bounds.max.x = Math.max(bounds.max.x, pt.z);
                    bounds.max.y = Math.max(bounds.max.y, pt.y);
                });
                return p;
            });

            // pad the bounds slightly to ensure the offset toolTip is included
            const padding = 5.0;
            bounds.min.x -= padding;
            bounds.min.y -= padding;
            bounds.max.x += padding;
            bounds.max.y += padding;

            const grid = new SpatialGrid(bounds, 2.0);
            rotatedPolys.forEach(poly => {
                const points = poly.points;
                for (let i = 0; i < points.length; i++) {
                    const p1 = points[i];
                    const p2 = points[(i + 1) % points.length];
                    // project 3D segment to 2D before insertion
                    const seg2d = [ { x: p1.z, y: p1.y }, { x: p2.z, y: p2.y } ];
                    grid.insert(seg2d);
                }
            });

            // 3. Check machinability for each point at this angle
            for (const contour of resampledContours) {
                for (const point of contour.points) {
                    const vnorm_rot = point._4axis.vnorm.clone().rotateYZ(angle * DEG2RAD);

                    // only check for collisions if the surface normal is generally facing the tool
                    if (/*vnorm_rot.z >= 0*/true) {
                        if (isMachinable(point, point._4axis.vnorm, angle, grid)) {
                            point._4axis.machinable[angle] = true; // Index by the tested rotation angle
                            if (sidx % 200 === 0) { // Removed point_idx filter based on user preference
                                const visualization_angle = (360 - angle + 90) % 360;
                                const vec = newPoint(0, Math.cos(visualization_angle * DEG2RAD), Math.sin(visualization_angle * DEG2RAD));
                                slice.output().setLayer("machinability", {line: lineColor})
                                    .addPoly(newPolygon([point, point.add(vec)]));
                            }
                        }
                    }
                }
            }
        }

      /* -- uncomment this once machinability is debugged
        // 4. Convert boolean arrays to MDR ranges
        for (const contour of resampledContours) {
            if (!contour.points) continue;
            for (const point of contour.points) {
                // Extrapolate machinability before converting to MDR ranges
                const fullMachinable = extrapolateMachinability(point._4axis.machinable, angleStep);
                point.MDR = convertBoolsToMDR(fullMachinable);
                delete point._4axis;
            }
        }

        // 5. Graph-cut segmentation (unchanged)
        slice.segments = [];
        for (const contour of resampledContours) {
            const points = contour.points;
            const numPoints = points.length;
            if (numPoints < 2) continue;

            const edges = [];
            for (let i = 0; i < numPoints; i++) {
                const p1 = points[i];
                const p2 = points[(i + 1) % numPoints];
                edges.push({
                    from: i,
                    to: (i + 1) % numPoints,
                    weight: 1 - calculateSimilarity(p1.MDR, p2.MDR)
                });
            }

            const cutThreshold = 0.5;
            const adj = new Map();
            for (let i = 0; i < numPoints; i++) adj.set(i, []);
            for (const edge of edges) {
                if (edge.weight <= cutThreshold) {
                    adj.get(edge.from).push(edge.to);
                    adj.get(edge.to).push(edge.from);
                }
            }

            const visited = new Array(numPoints).fill(false);
            const segments = [];
            for (let i = 0; i < numPoints; i++) {
                if (!visited[i]) {
                    const component_indices = [];
                    const q = [i];
                    visited[i] = true;
                    let head = 0;
                    while (head < q.length) {
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
                        const ordered_segment_indices = [];
                        let start_node = component_indices.find(node_idx => adj.get(node_idx).length <= 1) ?? component_indices[0];
                        const path_q = [start_node];
                        const path_visited = new Set([start_node]);
                        while (path_q.length > 0) {
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
    */
    }
    return sliced;
}
