/** Copyright Stewart Allen <sa@grid.space> -- All Rights Reserved */

import { CamOp } from '../core/op.js';
import { Tool } from '../core/tool.js';
import { newSlice } from '../../../core/slice.js';
import { newPoint } from '../../../../geo/point.js';
import { newPolygon } from '../../../../geo/polygon.js';
import { polygons as POLY } from '../../../../geo/polygons.js';

/**
 * Find the largest inscribed circle inside a set of polygons.
 * This is an approximation using a grid search followed by local hill climbing optimization.
 * Crucial for finding the optimal placement of the helical entry path.
 * 
 * @param {Polygon[]} polys - The input polygons representing the machinable area boundaries.
 * @param {number} toolDiam - Tool diameter to determine search grid bounds and thresholds.
 * @returns {object|null} Object containing { center: {x, y}, radius } of the largest inscribed circle.
 */
function findLargestInscribedCircle(polys, toolDiam) {
    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    
    // Compute bounding box for the entire set of polygons
    for (let poly of polys) {
        let bounds = poly.bounds;
        if (bounds.minx < minX) minX = bounds.minx;
        if (bounds.maxx > maxX) maxX = bounds.maxx;
        if (bounds.miny < minY) minY = bounds.miny;
        if (bounds.maxy > maxY) maxY = bounds.maxy;
    }
    if (minX === Infinity) return null;

    /**
     * Helper function to check if point (x, y) is inside the solid region
     * and calculate its distance to the closest boundary segment.
     */
    function getDistToBoundary(x, y) {
        let pt = newPoint(x, y, 0);
        let inside = false;
        
        // A point is inside if it is inside any outer polygon,
        // but NOT inside any inner hole of that polygon.
        for (let poly of polys) {
            if (pt.isInPolygon(poly)) {
                let inHole = false;
                if (poly.inner) {
                    for (let inner of poly.inner) {
                        if (pt.isInPolygon(inner)) {
                            inHole = true;
                            break;
                        }
                    }
                }
                if (!inHole) {
                    inside = true;
                    break;
                }
            }
        }
        if (!inside) return -1; // Outside the machinable area

        // Calculate minimum distance to all boundary segments
        let minDist = Infinity;
        for (let poly of polys) {
            let points = poly.points;
            let len = points.length;
            for (let i = 0; i < len; i++) {
                let p1 = points[i];
                let p2 = points[(i + 1) % len];
                let dist = pt.distToLine(p1, p2);
                if (dist < minDist) minDist = dist;
            }
            if (poly.inner) {
                for (let inner of poly.inner) {
                    let ipoints = inner.points;
                    let ilen = ipoints.length;
                    for (let i = 0; i < ilen; i++) {
                        let p1 = ipoints[i];
                        let p2 = ipoints[(i + 1) % ilen];
                        let dist = pt.distToLine(p1, p2);
                        if (dist < minDist) minDist = dist;
                    }
                }
            }
        }
        return minDist;
    }

    let bestPt = null;
    let maxDist = -Infinity;

    // Phase 1: Coarse Grid Search
    // We sample a 30x30 grid inside the bounding box to locate promising candidates
    let steps = 30;
    let dx = (maxX - minX) / steps;
    let dy = (maxY - minY) / steps;
    for (let i = 0; i <= steps; i++) {
        for (let j = 0; j <= steps; j++) {
            let x = minX + i * dx;
            let y = minY + j * dy;
            let dist = getDistToBoundary(x, y);
            if (dist > maxDist) {
                maxDist = dist;
                bestPt = { x, y };
            }
        }
    }

    // Phase 2: Fine Refinement via Hill Climbing
    // Start at the best grid point and query local directions, halving search radius on failure
    if (bestPt) {
        let radius = Math.max(dx, dy);
        while (radius > 0.01) {
            let bestLocalPt = bestPt;
            let directions = [
                {x: 0, y: 1}, {x: 0, y: -1}, {x: 1, y: 0}, {x: -1, y: 0},
                {x: 0.7, y: 0.7}, {x: -0.7, y: 0.7}, {x: 0.7, y: -0.7}, {x: -0.7, y: -0.7}
            ];
            let improved = false;
            for (let d of directions) {
                let nx = bestPt.x + d.x * radius;
                let ny = bestPt.y + d.y * radius;
                let dist = getDistToBoundary(nx, ny);
                if (dist > maxDist) {
                    maxDist = dist;
                    bestLocalPt = { x: nx, y: ny };
                    improved = true;
                }
            }
            if (!improved) {
                radius /= 2; // Shrink radius to search closer
            } else {
                bestPt = bestLocalPt; // Move center to the improved coordinate
            }
        }
    }

    return { center: bestPt, radius: maxDist };
}

/**
 * Generate a closed circle polygon with sampled points.
 * Used for establishing tool footprints and plunge areas for Clipper Offset.
 */
function createCirclePolygon(cx, cy, cz, radius) {
    let poly = newPolygon();
    let steps = Math.max(32, Math.min(128, Math.round(2 * Math.PI * radius / 1.0)));
    for (let i = 0; i < steps; i++) {
        let angle = (i / steps) * 2 * Math.PI;
        poly.add(cx + radius * Math.cos(angle), cy + radius * Math.sin(angle), cz);
    }
    return poly;
}

/**
 * Convert a linear toolpath centerline segment into D-shaped trochoidal loops.
 * Trochoidal loops prevent full tool engagement when slotting or clearing narrow channels.
 * 
 * @param {Point[]} pathPoints - Input centerline polyline coordinates.
 * @param {number} toolDiam - Tool diameter.
 * @param {number} rdoc - Radial Depth of Cut.
 * @returns {Point[]} Transformed trochoidal path points.
 */
function trochoidifyPath(pathPoints, toolDiam, rdoc) {
    let result = [];
    if (pathPoints.length < 2) return pathPoints;
    let step_size = 0.1 * toolDiam; // Forward step size per loop
    let R_loop = 0.5 * toolDiam;    // Radius of each D-loop

    for (let i = 0; i < pathPoints.length - 1; i++) {
        let p1 = pathPoints[i];
        let p2 = pathPoints[i+1];
        let dist = p1.distTo2D(p2);
        if (dist < 0.01) continue;

        // Compute direction vector V and perpendicular normal N
        let V = { x: (p2.x - p1.x) / dist, y: (p2.y - p1.y) / dist };
        let N = { x: -V.y, y: V.x };

        let current_dist = 0;
        while (current_dist < dist) {
            let S_k = {
                x: p1.x + current_dist * V.x,
                y: p1.y + current_dist * V.y
            };
            let next_dist = Math.min(dist, current_dist + step_size);
            let actual_step = next_dist - current_dist;

            // Generate D-loop: arc into material, ending at the next centerline step S_k+1
            for (let theta = 0; theta <= Math.PI; theta += Math.PI / 8) {
                let pt = newPoint(
                    S_k.x + (theta / Math.PI) * actual_step * V.x + R_loop * Math.sin(theta) * N.x,
                    S_k.y + (theta / Math.PI) * actual_step * V.y + R_loop * Math.sin(theta) * N.y,
                    p1.z
                );
                result.push(pt);
            }
            current_dist = next_dist;
        }
    }
    return result;
}

/**
 * Emit polygon points with active feedrate, utilizing Kiri:Moto's arc detector.
 * 
 * @param {object} ops - The preparation API options object.
 * @param {Polygon} poly - Polygon to emit.
 * @param {number} active_feedrate - Chip-thinning adjusted feedrate.
 * @param {boolean} arcing - Whether arc fitting is enabled in process settings.
 */
function emitPolygonPoints(ops, poly, active_feedrate, arcing) {
    let { camOut } = ops;
    let points = poly.points;

    // If arc fitting is enabled, pass points through Kiri:Moto's bi-arc detector
    if (arcing) {
        let p_arc = newPolygon(points).setOpenValue(poly.open).detectArcs({
            tolerance: ops.widget.settings?.process?.camArcTolerance || 0.02,
            arcRes: ops.widget.settings?.process?.camArcResolution || 5,
            minPoints: 5
        });
        points = p_arc.points;
    }

    let skip = 0;
    let type;
    let center;
    let lastP = points.peek();
    let lastOut;

    // Loop through points and emit G01 cuts or G02/G03 arcs
    for (let point of points) {
        lastOut = point.clone();
        if (type) {
            skip = point === lastP ? 0 : skip - 1;
            camOut(lastOut, skip ? -1 : type, { center, feed: active_feedrate });
            if (!skip) center = type = undefined;
            continue;
        } else if (point.arc) {
            let { arc } = point;
            skip = arc.skip;
            type = arc.clockwise ? 2 : 3;
            center = arc.center.clone().move({ x: -point.x, y: -point.y });
        }
        camOut(lastOut, 1, { feed: active_feedrate });
    }
}

/**
 * Helper function to find the slice closest to a target Z coordinate.
 * This is robust against floating point errors and flat-surface Z shifts.
 */
function findSliceTops(slices, targetZ) {
    let closestSlice = null;
    let minDist = Infinity;
    for (let slice of slices) {
        let dist = Math.abs(slice.z - targetZ);
        if (dist < minDist) {
            minDist = dist;
            closestSlice = slice;
        }
    }
    // Return the slice tops if the distance is within tolerance (e.g., 0.05mm)
    if (closestSlice && minDist < 0.05) {
        return closestSlice.tops || [];
    }
    return [];
}

/**
 * Helper to check if a point lies inside the cleared polygon set.
 */
function isPointInCleared(pt, clearedPolys) {
    if (!clearedPolys || clearedPolys.length === 0) return false;
    for (let poly of clearedPolys) {
        if (pt.isInPolygon(poly)) {
            let inHole = false;
            if (poly.inner) {
                for (let inner of poly.inner) {
                    if (pt.isInPolygon(inner)) {
                        inHole = true;
                        break;
                    }
                }
            }
            if (!inHole) {
                return true;
            }
        }
    }
    return false;
}

/**
 * Check if segment a1-a2 intersects segment b1-b2.
 */
function lineIntersectsLine(a1, a2, b1, b2) {
    let det = (a2.x - a1.x) * (b2.y - b1.y) - (b2.x - b1.x) * (a2.y - a1.y);
    if (Math.abs(det) < 1e-9) return false; // Parallel or collinear
    let lambda = ((b2.y - b1.y) * (b2.x - a1.x) + (b1.x - b2.x) * (b2.y - a1.y)) / det;
    let gamma = ((a1.y - a2.y) * (b2.x - a1.x) + (a2.x - a1.x) * (b2.y - a1.y)) / det;
    return (0 < lambda && lambda < 1) && (0 < gamma && gamma < 1);
}

/**
 * Check if segment p1-p2 intersects any boundary segment of a polygon.
 */
function lineIntersectsPoly(p1, p2, poly) {
    let points = poly.points;
    let len = points.length;
    for (let i = 0; i < len; i++) {
        let q1 = points[i];
        let q2 = points[(i + 1) % len];
        if (lineIntersectsLine(p1, p2, q1, q2)) return true;
    }
    if (poly.inner) {
        for (let inner of poly.inner) {
            let ipoints = inner.points;
            let ilen = ipoints.length;
            for (let i = 0; i < ilen; i++) {
                let q1 = ipoints[i];
                let q2 = ipoints[(i + 1) % ilen];
                if (lineIntersectsLine(p1, p2, q1, q2)) return true;
            }
        }
    }
    return false;
}

/**
 * Check if the entire straight line segment between p1 and p2 lies within cleared space.
 */
function isLineInCleared(p1, p2, clearedPolys) {
    if (!isPointInCleared(p1, clearedPolys)) return false;
    if (!isPointInCleared(p2, clearedPolys)) return false;
    let mid = newPoint((p1.x + p2.x) / 2, (p1.y + p2.y) / 2, p1.z);
    if (!isPointInCleared(mid, clearedPolys)) return false;
    for (let poly of clearedPolys) {
        if (lineIntersectsPoly(p1, p2, poly)) return false;
    }
    return true;
}

/**
 * Custom spiralize and prune function for a single region's peeling steps.
 */
function spiralizeAndPrune(all_loops, Tool_D, Target_RDOC, climb, Z_level, region) {
    if (!all_loops || all_loops.length === 0) return [];
    let Tool_R = Tool_D / 2;

    // Phase 1: Group all_loops into containment chains (nesting trees)
    let n = all_loops.length;
    let parents = new Array(n).fill(-1);
    let isLeaf = new Array(n).fill(true);
    let depths = new Array(n).fill(0);
    let containedBy = Array.from({ length: n }, () => []);

    for (let i = 0; i < n; i++) {
        let pi = all_loops[i].points[0];
        if (!pi) continue;
        for (let j = 0; j < n; j++) {
            if (i !== j) {
                if (pi.isInPolygon(all_loops[j])) {
                    containedBy[i].push(j);
                }
            }
        }
    }

    for (let i = 0; i < n; i++) {
        let containers = containedBy[i];
        if (containers.length > 0) {
            let bestParent = containers[0];
            for (let c of containers) {
                if (containedBy[c].length > containedBy[bestParent].length) {
                    bestParent = c;
                }
            }
            parents[i] = bestParent;
            isLeaf[bestParent] = false;
            depths[i] = containers.length;
        }
    }

    let childCount = new Array(n).fill(0);
    for (let i = 0; i < n; i++) {
        if (parents[i] !== -1) {
            childCount[parents[i]]++;
        }
    }

    let assigned = new Array(n).fill(false);
    let chains = [];

    let leafIndices = [];
    for (let i = 0; i < n; i++) {
        if (isLeaf[i]) {
            leafIndices.push(i);
        }
    }
    leafIndices.sort((a, b) => depths[b] - depths[a]);

    for (let leafIdx of leafIndices) {
        let chain = [];
        let curr = leafIdx;
        while (curr !== -1 && !assigned[curr]) {
            chain.push(all_loops[curr]);
            assigned[curr] = true;
            let next = parents[curr];
            if (next !== -1 && (childCount[next] > 1 || assigned[next])) {
                break;
            }
            curr = next;
        }
        if (chain.length > 0) {
            // Already ordered from innermost (leaf) to outermost (parent)
            chains.push(chain);
        }
    }

    // Capture any unassigned loops
    for (let i = 0; i < n; i++) {
        if (!assigned[i]) {
            chains.push([all_loops[i]]);
        }
    }

    // Process each chain to produce the final pruned spiral segments
    let final_paths = [];

    // Determine N for resampling. We make sure it preserves original region point count.
    let origPointsCount = region ? region.points.length : 0;
    if (region && region.inner) {
        for (let inner of region.inner) {
            origPointsCount = Math.max(origPointsCount, inner.points.length);
        }
    }

    for (let chain of chains) {
        if (chain.length === 1) {
            // Single loop: prune it against its engagement zone
            let single = chain[0];
            let cuts = single.cut(single.engagement_zone, true);
            if (cuts) final_paths.push(...cuts);
            continue;
        }

        let N = Math.max(...chain.map(l => l.points.length), 128, origPointsCount);

        // Resample loops in the chain
        let resampledLoops = chain.map(loop => {
            let points = loop.points;
            let cumulative = [0];
            let totalDist = 0;
            for (let i = 0; i < points.length; i++) {
                let p1 = points[i];
                let p2 = points[(i + 1) % points.length];
                totalDist += p1.distTo2D(p2);
                cumulative.push(totalDist);
            }

            if (totalDist === 0) {
                return Array.from({ length: N }, () => points[0].clone());
            }

            let resampled = [];
            for (let i = 0; i < N; i++) {
                let targetDist = (i / N) * totalDist;
                let idx = 0;
                while (idx < points.length && cumulative[idx + 1] < targetDist) {
                    idx++;
                }
                let p1 = points[idx];
                let p2 = points[(idx + 1) % points.length];
                let segLen = cumulative[idx + 1] - cumulative[idx];
                let t = segLen > 0 ? (targetDist - cumulative[idx]) / segLen : 0;
                resampled.push(newPoint(p1.x + (p2.x - p1.x) * t, p1.y + (p2.y - p1.y) * t, p1.z));
            }
            return resampled;
        });

        // Align start points
        for (let i = 1; i < resampledLoops.length; i++) {
            let prevStart = resampledLoops[i - 1][0];
            let currPoints = resampledLoops[i];
            let minDist = Infinity;
            let bestIdx = 0;
            for (let j = 0; j < currPoints.length; j++) {
                let dist = prevStart.distTo2D(currPoints[j]);
                if (dist < minDist) {
                    minDist = dist;
                    bestIdx = j;
                }
            }
            if (bestIdx > 0) {
                resampledLoops[i] = currPoints.slice(bestIdx).concat(currPoints.slice(0, bestIdx));
            }
        }

        // Interpolate spiral and check engagement
        let spiral_segments = [];
        let current_segment = null;

        // Helper to add a point to the current segment or start a new one
        function addPoint(pt, is_cutting) {
            if (is_cutting) {
                if (!current_segment) {
                    current_segment = newPolygon().setOpen();
                }
                current_segment.addPoints([pt]);
            } else {
                if (current_segment) {
                    if (current_segment.points.length >= 2) {
                        spiral_segments.push(current_segment);
                    }
                    current_segment = null;
                }
            }
        }

        // 1. Trace the first (innermost) loop in its entirety to clean the inner wall
        let L_first = resampledLoops[0];
        let zone_first = chain[0].engagement_zone;
        for (let j = 0; j < N; j++) {
            let pt = L_first[j].clone();
            addPoint(pt, isPointInCleared(pt, zone_first));
        }
        let pt_first_start = L_first[0].clone();
        addPoint(pt_first_start, isPointInCleared(pt_first_start, zone_first));

        // 2. Interpolate spiral between loops
        for (let i = 0; i < resampledLoops.length - 1; i++) {
            let L_curr = resampledLoops[i];
            let L_next = resampledLoops[i + 1];
            let zone = chain[i].engagement_zone;

            for (let j = 0; j < N; j++) {
                let t = j / N;
                let x = L_curr[j].x * (1 - t) + L_next[j].x * t;
                let y = L_curr[j].y * (1 - t) + L_next[j].y * t;
                let pt = newPoint(x, y, Z_level);
                addPoint(pt, isPointInCleared(pt, zone));
            }
        }

        // 3. Append the final (outermost) loop to clean the outer wall (ALWAYS cut, do not prune!)
        let L_last = resampledLoops[resampledLoops.length - 1];
        for (let j = 0; j < N; j++) {
            let pt = L_last[j].clone();
            addPoint(pt, true); // Always keep the outer wall pass for clean/precise boundary contours
        }
        let pt_last_start = L_last[0].clone();
        addPoint(pt_last_start, true);

        // Flush last segment
        if (current_segment && current_segment.points.length >= 2) {
            spiral_segments.push(current_segment);
        }

        final_paths.push(...spiral_segments);
    }

    return final_paths;
}

class OpAdaptive extends CamOp {
    constructor(state, op) {
        super(state, op);
    }

    /**
     * Phase 3: The 3D Slicing & Terracing Loop
     * Processes 3D widget geometry from top to bottom, generating adaptive pocketing/peeling paths.
     */
    async slice(progress) {
        let { op, state } = this;
        let { addSlices, color, settings, stock, widget } = state;

        // Parse Tool parameters
        let camTool = new Tool(settings, op.tool);
        let Tool_D = camTool.fluteDiameter();
        let Tool_R = Tool_D / 2;

        // Parse Slicing / Terracing parameters
        let Target_RDOC = Tool_D * op.step; // Stepover distance
        let Z_Major = op.down;              // Max step-down
        let Z_Minor = op.stepup;            // Terrace step-up

        // Validations to prevent infinite loops and invalid configurations
        if (Z_Major <= 0) {
            throw `invalid step down "${Z_Major}"`;
        }
        if (Target_RDOC <= 0) {
            throw `invalid stepover "${op.step}"`;
        }
        let leave = op.leave;              // Stock to leave
        let Ramp_Angle_Max = op.ramp;       // Helical ramp entry angle
        let tolerance = op.tolerance ?? 0.01; // Micro tolerance for boolean operations

        let inside = op.inside;
        let direction = op.direction;

        // Workarea limits
        let zTop = state.workarea.top_z;
        let zBottom = state.workarea.bottom_z;

        console.log("Adaptive slice start, zTop:", zTop, "zBottom:", zBottom, "Z_Major:", Z_Major, "Z_Minor:", Z_Minor);

        // Phase 3.1: Generate Z-Slices heights array
        let plan = [];
        let prev_z = zTop;
        let current_z = zTop - Z_Major;

        // Build Z level plan with major steps and minor step-ups
        while (current_z > zBottom + 0.0001) {
            let levelPlan = {
                major: current_z,
                minors: [],
                prev: prev_z
            };
            // Generate intermediate step-up heights moving upwards
            if (Z_Minor > 0) {
                let z_int = current_z + Z_Minor;
                while (z_int < prev_z - 0.0001) {
                    levelPlan.minors.push(z_int); // Sorted bottom-to-top
                    z_int += Z_Minor;
                }
            }
            plan.push(levelPlan);
            prev_z = current_z;
            current_z -= Z_Major;
        }

        // Include last bottom level if needed
        if (prev_z > zBottom + 0.0001) {
            let levelPlan = {
                major: zBottom,
                minors: [],
                prev: prev_z
            };
            if (Z_Minor > 0) {
                let z_int = zBottom + Z_Minor;
                while (z_int < prev_z - 0.0001) {
                    levelPlan.minors.push(z_int);
                    z_int += Z_Minor;
                }
            }
            plan.push(levelPlan);
        }

        // Gather all unique Z-heights to perform geometry slicing
        let zList = [];
        for (let level of plan) {
            zList.push(level.major);
            zList.push(...level.minors);
        }
        zList = [...new Set(zList)].sort((a, b) => b - a); // Unique descending array

        console.log("plan level count:", plan.length, "zList count:", zList.length, "zList:", zList);

        // Initialize Slicer and perform 3D model slicing at Z-heights
        let slicer = state.newSlicer({ zflatup: true });
        let slices = [];
        await slicer.slice(zList, {
            each: (slice) => {
                slices.push(slice);
            }
        });

        console.log("slices count:", slices.length);

        // Define stock / outer boundary polygon
        let stock_poly = newPolygon().centerRectangle(stock.center, stock.x, stock.y);
        let outer_boundary = inside ? state.shadow.base.clone(true) : [ stock_poly ];

        // Build accumulated part offsets from top to bottom to protect the part from undercuts and bed-level clearance
        let part_offsets_by_z = {};
        let accumulated_part_offset = [];
        // Ensure slices are processed top-down
        let sorted_slices = slices.slice().sort((a, b) => b.z - a.z);
        for (let slice of sorted_slices) {
            let Z_curr = slice.z;
            let part_polys = slice.tops || [];
            let current_part_offset = [];
            if (part_polys.length > 0) {
                current_part_offset = POLY.expand(part_polys, leave, Z_curr) || [];
            }
            let prev_part_projected = POLY.setZ(accumulated_part_offset.map(p => p.clone(true)), Z_curr);
            let part_offset = POLY.union([...prev_part_projected, ...current_part_offset], 0.01, true) || [];
            part_offsets_by_z[Z_curr] = part_offset;
            accumulated_part_offset = part_offset;
        }

        this.outSlices = [];
        let Cleared_Footprint_Lower = null; // Boolean union of cleared areas below

        // Process top-down through the level plan
        for (let level of plan) {
            let Z_curr = level.major;
            let major_paths = [];
            let all_cleared_footprints = [];

            // Phase 3.2: Major Step-Down (Bulk Removal)
            let outer_boundary_curr = POLY.setZ(outer_boundary.map(p => p.clone(true)), Z_curr);

            // Retrieve accumulated part offset (which includes all levels above to protect geometry)
            let part_offset = part_offsets_by_z[Z_curr] || [];
            // Machinable Area = Stock Boundary - Part Offset
            let Machinable_Area = [];
            POLY.subtract(outer_boundary_curr, part_offset, Machinable_Area, undefined, Z_curr, 0.01);
            console.log("Z_curr:", Z_curr, "part_offset count:", part_offset.length, "Machinable_Area count:", Machinable_Area.length);

            // Group Machinable Area into individual disconnected regions
            for (let region of Machinable_Area) {
                let all_loops = [];
                // Find largest inscribed circle in this region
                let circle = findLargestInscribedCircle([ region ], Tool_D);
                console.log("Z_curr:", Z_curr, "Region area:", region.area(), "circle found:", !!circle, "radius:", circle ? circle.radius : 0);
                let current_cleared = [];
                let helix_poly = null;

                if (circle && circle.radius >= Tool_R) {
                    // Helical Entry (The 0.9D Rule)
                    let Helix_Radius = 0.45 * Tool_D;
                    // If the region is narrow but can still fit the entry, scale down helix radius
                    if (circle.radius < Helix_Radius + Tool_R) {
                        Helix_Radius = Math.max(0.1 * Tool_D, circle.radius - Tool_R - 0.01);
                    }

                    // Calculate helical descent pitch based on max ramp angle
                    let Pitch_Z = 2 * Math.PI * Helix_Radius * Math.tan(Ramp_Angle_Max * Math.PI / 180);
                    let Z_start = level.prev;
                    let Z_end = Z_curr;
                    let Z_depth = Z_start - Z_end;

                    let turns = Math.ceil(Z_depth / Pitch_Z);
                    let actual_pitch = Z_depth / turns;
                    let helix_pts = [];
                    let segments_per_turn = 32;

                    console.log("Helix gen: Z_start:", Z_start, "Z_end:", Z_end, "Z_depth:", Z_depth, "Pitch_Z:", Pitch_Z, "turns:", turns);

                    // Generate coordinates for helical descent
                    for (let t = 0; t <= turns * segments_per_turn; t++) {
                        let angle = (t / segments_per_turn) * 2 * Math.PI;
                        let x = circle.center.x + Helix_Radius * Math.cos(angle);
                        let y = circle.center.y + Helix_Radius * Math.sin(angle);
                        let z = Z_start - (t / (turns * segments_per_turn)) * Z_depth;
                        helix_pts.push(newPoint(x, y, z));
                    }

                    helix_poly = newPolygon(helix_pts).setOpen();
                    helix_poly.isHelix = true; // Mark as helix for feedrate adjustments in prepare
                    
                    // Footprint cleared by the helix (using circle polygon for proper offset execution)
                    let footprint = createCirclePolygon(circle.center.x, circle.center.y, Z_curr, Helix_Radius + Tool_R);
                    current_cleared = [ footprint ];
                } else {
                    // Fallback to plunging if region is too narrow for helix
                    let startPoint = region.points[0];
                    let footprint = createCirclePolygon(startPoint.x, startPoint.y, Z_curr, Tool_R);
                    current_cleared = [ footprint ];
                }

                if (helix_poly) {
                    major_paths.push(helix_poly);
                }

                // Adaptive Peeling loop: offset cleared footprint outwards
                let peeling_iterations = 0;
                let peeling_steps = [];
                console.log("Entering peeling loop for Z_curr:", Z_curr, "region area:", region.area());
                while (true) {
                    peeling_iterations++;
                    if (peeling_iterations > 1000) {
                        console.log("peeling infinite loop guard triggered at Z_curr:", Z_curr);
                        break;
                    }

                    let uncut = [];
                    POLY.subtract([ region ], current_cleared, uncut, undefined, Z_curr, 0.01);
                    let uncutArea = uncut.reduce((acc, p) => acc + Math.abs(p.area()), 0);
                    if (uncut.length === 0 || uncutArea < 0.1) {
                        console.log("Peeling complete, uncut area is negligible:", uncutArea);
                        break;
                    }

                    // Offset cleared footprint by stepover Target_RDOC, preserving Z_curr
                    let expanded = POLY.expand(current_cleared, Target_RDOC, Z_curr) || [];
                    
                    // Clip to region boundary using double-subtraction intersection (A intersected with B = A - (A - B))
                    let expanded_outside = [];
                    POLY.subtract(expanded, [ region ], expanded_outside, undefined, Z_curr, 0.01);
                    let next_cleared = [];
                    POLY.subtract(expanded, expanded_outside, next_cleared, undefined, Z_curr, 0.01);

                    // Compute path centerline by offsetting inward by Tool_R, preserving Z_curr
                    let next_path = POLY.expand(next_cleared, -Tool_R, Z_curr) || [];
                    next_path = next_path.filter(p => p.points.length > 1);

                    if (next_path.length === 0) {
                        console.log("Peeling next_path is empty. Initiating narrow channel fallback. Uncut count:", uncut.length);
                        // Narrow channel slotting / trochoidal paths fallback
                        for (let uncut_poly of uncut) {
                            // If channel is narrower than 1.5 * Tool_D
                            let inward_check = POLY.expand([ uncut_poly ], -0.75 * Tool_D, Z_curr) || [];
                            if (inward_check.length === 0) {
                                let centerline = POLY.expand([ uncut_poly ], -0.5 * Tool_R, Z_curr) || [];
                                for (let cp of centerline) {
                                    if (cp.points.length >= 2) {
                                        let troch_pts = trochoidifyPath(cp.points, Tool_D, Target_RDOC);
                                        if (troch_pts.length > 1) {
                                            for (let pt of troch_pts) {
                                                pt.z = Z_curr;
                                            }
                                            major_paths.push(newPolygon(troch_pts).setOpen());
                                        }
                                    }
                                }
                            }
                        }
                        break;
                    }

                // Enforce winding direction on centerline loops
                    POLY.setWinding(next_path, direction === 'climb');

                    // Store loops with their engagement zones
                    let step_engagement_zone = POLY.expand(uncut, Tool_R, Z_curr) || [];
                    for (let path of next_path) {
                        path.engagement_zone = step_engagement_zone;
                        all_loops.push(path);
                    }

                    current_cleared = next_cleared;
                }
                
                // Append clean boundary to all_loops to ensure perfect boundary contours
                let clean_boundary = POLY.expand([ region ], -Tool_R, Z_curr) || [];
                POLY.setWinding(clean_boundary, direction === 'climb');
                let final_engagement_zone = POLY.expand([ region ], Tool_R, Z_curr) || [];
                for (let path of clean_boundary) {
                    path.engagement_zone = final_engagement_zone;
                    all_loops.push(path);
                }
                
                // Spiralize and prune the collected peeling steps, passing the region boundary to preserve resolution
                let spiral_paths = spiralizeAndPrune(all_loops, Tool_D, Target_RDOC, direction === 'climb', Z_curr, region);
                major_paths.push(...spiral_paths);
                console.log("Region finished, major_paths count:", major_paths.length, "helix_poly:", !!helix_poly, "peeling iterations:", peeling_iterations);

                all_cleared_footprints.push(...current_cleared);
            }

            // Save the total cleared footprint from this major Z level
            Cleared_Footprint_Lower = POLY.union(all_cleared_footprints, 0.01, true) || [];
            POLY.setZ(Cleared_Footprint_Lower, Z_curr);

            // Store major slice output
            let majorSlice = newSlice(Z_curr);
            majorSlice.camLines = major_paths;
            majorSlice.cleared = Cleared_Footprint_Lower; // Store cleared footprint
            majorSlice
                .output()
                .setLayer("Adaptive", { face: color, line: color })
                .addPolys(majorSlice.camLines);
            this.outSlices.push(majorSlice);

            // Phase 3.3: Minor Step-Up (Terracing)
            // Clean up rest-material on walls at intermediate Z heights
            for (let Z_minor of level.minors) {
                let minor_paths = [];
                let outer_boundary_minor = POLY.setZ(outer_boundary.map(p => p.clone(true)), Z_minor);
                let current_boundary_minor = part_offsets_by_z[Z_minor] || [];

                // Offset lower footprint inward slightly to prevent tool rubbing on already cut walls
                let lower_footprint_offset = POLY.expand(Cleared_Footprint_Lower, -tolerance, Z_minor) || [];

                // Terrace_Area = Stock_Boundary - Current_Boundary - Cleared_Footprint_Lower_Offset
                let diff1 = [];
                POLY.subtract(outer_boundary_minor, current_boundary_minor, diff1, undefined, Z_minor, 0.01);
                let Terrace_Area = [];
                POLY.subtract(diff1, lower_footprint_offset, Terrace_Area, undefined, Z_minor, 0.01);

                let all_cleared_terraces = [];

                for (let region of Terrace_Area) {
                    let all_loops = [];
                    // Linking: tool can plunge in air directly in pre-cleared void
                    // Peeling the terrace region outwards
                    let current_cleared = lower_footprint_offset;
                    let terrace_iterations = 0;
                    let peeling_steps = [];

                    while (true) {
                        terrace_iterations++;
                        if (terrace_iterations > 1000) {
                            console.log("terrace infinite loop guard triggered at Z_minor:", Z_minor);
                            break;
                        }

                        let uncut = [];
                        POLY.subtract([ region ], current_cleared, uncut, undefined, Z_minor, 0.01);
                        let uncutArea = uncut.reduce((acc, p) => acc + Math.abs(p.area()), 0);
                        if (uncut.length === 0 || uncutArea < 0.1) {
                            break;
                        }

                        let expanded = POLY.expand(current_cleared, Target_RDOC, Z_minor) || [];
                        
                        // Clip to region boundary using double-subtraction intersection (A intersected with B = A - (A - B))
                        let expanded_outside = [];
                        POLY.subtract(expanded, [ region ], expanded_outside, undefined, Z_minor, 0.01);
                        let next_cleared = [];
                        POLY.subtract(expanded, expanded_outside, next_cleared, undefined, Z_minor, 0.01);

                        let next_path = POLY.expand(next_cleared, -Tool_R, Z_minor) || [];
                        next_path = next_path.filter(p => p.points.length > 1);

                        if (next_path.length === 0) {
                            break; // Narrow slotting fallback not typically needed for step-ups
                        }

                        POLY.setWinding(next_path, direction === 'climb');

                        let step_engagement_zone = POLY.expand(uncut, Tool_R, Z_minor) || [];
                        for (let path of next_path) {
                            path.engagement_zone = step_engagement_zone;
                            all_loops.push(path);
                        }

                        current_cleared = next_cleared;
                    }

                    // Append clean boundary to all_loops to ensure perfect boundary contours
                    let clean_boundary = POLY.expand([ region ], -Tool_R, Z_minor) || [];
                    POLY.setWinding(clean_boundary, direction === 'climb');
                    let final_engagement_zone = POLY.expand([ region ], Tool_R, Z_minor) || [];
                    for (let path of clean_boundary) {
                        path.engagement_zone = final_engagement_zone;
                        all_loops.push(path);
                    }

                    let spiral_paths = spiralizeAndPrune(all_loops, Tool_D, Target_RDOC, direction === 'climb', Z_minor, region);
                    minor_paths.push(...spiral_paths);

                    all_cleared_terraces.push(...current_cleared);
                }

                // Update lower footprint for the next step-up level
                let unioned_terrace = POLY.union(all_cleared_terraces, 0.01, true) || [];
                POLY.setZ(unioned_terrace, Z_minor);
                Cleared_Footprint_Lower = POLY.union([...Cleared_Footprint_Lower, ...unioned_terrace], 0.01, true) || [];
                POLY.setZ(Cleared_Footprint_Lower, Z_minor);

                let minorSlice = newSlice(Z_minor);
                minorSlice.camLines = minor_paths;
                minorSlice.cleared = Cleared_Footprint_Lower; // Store accumulated cleared footprint
                minorSlice
                    .output()
                    .setLayer("Adaptive", { face: color, line: color })
                    .addPolys(minorSlice.camLines);
                this.outSlices.push(minorSlice);
            }
        }

        // Register slices to Kiri:Moto's visualization pipeline
        addSlices(this.outSlices);
    }

    /**
     * Phase 4 & 5: Micro-Lifts, Linking, and Arcs Optimization
     * Transmits tool paths to the G-code emission pipeline with retract overrides.
     */
    prepare(ops, progress) {
        let { op, state } = this;
        let { camOut, setNextIsMove, newLayer, zSafe } = ops;

        // Parse tool and configuration parameters
        let camTool = new Tool(state.settings, op.tool);
        let Tool_D = camTool.fluteDiameter();
        let Tool_R = Tool_D / 2;
        let Target_RDOC = Tool_D * op.step;

        let rate = op.rate;
        let plunge = op.plunge;
        let arcing = state.settings.process.camArcEnabled;

        // 2.2 Dynamic Feedrate Adjustment (Radial Chip Thinning)
        // Adjust feedrate upwards if radial depth of cut is less than tool radius
        let F_adj_multiplier = 1.0;
        if (Target_RDOC < Tool_R) {
            F_adj_multiplier = Tool_R / Math.sqrt((2 * Tool_R * Target_RDOC) - Math.pow(Target_RDOC, 2));
        }
        let active_feedrate = rate * F_adj_multiplier;

        let first = true;

        // Process all slices in order
        for (let slice of this.outSlices) {
            newLayer();

            for (let path of slice.camLines) {
                let pts = path.points;
                if (pts.length < 2) continue;
                let startPt = pts[0];

                if (first) {
                    // Phase 4.4: First operation plunge
                    // Perform rapid G00 to safe height above starting point, then feed (G01) down to depth
                    setNextIsMove();
                    camOut(newPoint(startPt.x, startPt.y, zSafe), 0);
                    camOut(startPt, 1, { feed: plunge });
                    first = false;
                } else {
                    let lastPt = ops.getLastPoint();
                    // If tool is not already at the starting coordinate
                    if (lastPt.distTo2D(startPt) > 0.01 || Math.abs(lastPt.z - startPt.z) > 0.01) {
                        let cleared = slice.cleared;
                        if (cleared && isLineInCleared(lastPt, startPt, cleared)) {
                            // Safe to rapid directly in cleared space at current Z-height (with minor lift of 0.1mm)
                            setNextIsMove();
                            camOut(newPoint(lastPt.x, lastPt.y, lastPt.z + 0.1), 0);
                            camOut(newPoint(startPt.x, startPt.y, startPt.z + 0.1), 0);
                            camOut(startPt, 1, { feed: plunge });
                        } else {
                            // Unsafe - retract to zSafe, rapid, and plunge
                            setNextIsMove();
                            camOut(newPoint(lastPt.x, lastPt.y, zSafe), 0); // Retract to zSafe
                            camOut(newPoint(startPt.x, startPt.y, zSafe), 0); // Rapid XY at zSafe
                            camOut(startPt, 1, { feed: plunge });             // Plunge
                        }
                    }
                }

                // Output the coordinate segments
                if (path.isHelix) {
                    // Helical entry path: feed along points at plunge rate (varying Z coordinate)
                    for (let pt of pts) {
                        camOut(pt, 1, { feed: plunge });
                    }
                } else {
                    // Cutting path: emit points with chip-thinning feedrate and optional bi-arc fitting
                    emitPolygonPoints(ops, path, active_feedrate, arcing);
                }
            }
        }
    }
}

export { OpAdaptive };
