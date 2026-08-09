/** Copyright Stewart Allen <sa@grid.space> -- All Rights Reserved */

import { newPoint } from '../../../../geo/point.js';
import { newPolygon } from '../../../../geo/polygon.js';
import { polygons as POLY } from '../../../../geo/polygons.js';

class MATNode {
    constructor(x, y, radius) {
        this.x = x;
        this.y = y;
        this.radius = radius;
        this.neighbors = new Set();
        this.chamberId = null;
        this.isGenerator = false;
        this.metaNode = null; // References the MetaNode this node belongs to
    }
}

class MetaNode {
    constructor(id, strategy, generator = null) {
        this.id = id;
        this.strategy = strategy; // 'chamber', 'entry', or 'corridor'
        this.generator = generator; // generator object or null
        this.nodes = []; // MATNodes belonging to this meta-node
        this.neighbors = new Set(); // Connected MetaNodes
        this.connections = []; // Connections: { neighbor: MetaNode, myEnd: boolean, itsEnd: boolean }
        this.D = null; // Resampled path D points
        this.C = null; // Oriented/bridged path C points
    }
}

/**
 * Helper to calculate 2D distance between two nodes
 */
function dist2D(u, v) {
    return Math.hypot(u.x - v.x, u.y - v.y);
}

/**
 * Finds the intersection of a 2D ray with a polygon.
 * Returns the closest intersection point along the ray in the forward direction (t > 0).
 * 
 * Math Derivation (Cramer's Rule):
 * Ray: R(t) = C + t * D, where C = center, D = (dx, dy)
 * Segment: S(u) = P0 + u * V, where V = P1 - P0 = (vx, vy), and u in [0, 1]
 * Solving C + t * D = P0 + u * V:
 *   t * dx - u * vx = P0.x - C.x
 *   t * dy - u * vy = P0.y - C.y
 * Matrix form:
 *   | dx  -vx | | t |   | P0.x - C.x |
 *   | dy  -vy | | u | = | P0.y - C.y |
 * Determinant:
 *   det = dx * (-vy) - (-vx) * dy = -dx * vy + dy * vx
 * Cramer's Rule solutions:
 *   t = ((C.x - P0.x) * vy - (C.y - P0.y) * vx) / det  <-- (Corrected sign error that originally caused 180-deg rotation)
 *   u = (dx * (P0.y - C.y) - dy * (P0.x - C.x)) / det
 */
function getRayIntersection(center, dx, dy, poly) {
    let pts = poly.points;
    let best_t = Infinity;
    let best_pt = null;

    for (let i = 0; i < pts.length; i++) {
        let p0 = pts[i];
        let p1 = pts[(i + 1) % pts.length];

        let vx = p1.x - p0.x;
        let vy = p1.y - p0.y;

        let det = -dx * vy + dy * vx;
        if (Math.abs(det) < 1e-9) continue; // Ray is parallel to the segment

        let t = ((center.x - p0.x) * vy - (center.y - p0.y) * vx) / det;
        let u = (dx * (p0.y - center.y) - dy * (p0.x - center.x)) / det;

        // Intersection must be in the forward direction of the ray (t > 0)
        // and lie within the bounds of the segment (0 <= u <= 1)
        if (t > 0 && u >= 0 && u <= 1) {
            if (t < best_t) {
                best_t = t;
                best_pt = { x: center.x + t * dx, y: center.y + t * dy };
            }
        }
    }
    return best_pt || (pts.length ? pts[0] : center); // fallback
}

/**
 * Returns true if three points A, B, C are in counter-clockwise order.
 * Uses the sign of the cross product of vectors AB and AC:
 * (C.y - A.y) * (B.x - A.x) - (B.y - A.y) * (C.x - A.x)
 */
function ccw(A, B, C) {
    return (C.y - A.y) * (B.x - A.x) > (B.y - A.y) * (C.x - A.x);
}

/**
 * Returns true if segment AB intersects segment CD.
 * Uses orientation-based intersection: segment AB and CD intersect if and only if
 * points A and B lie on opposite sides of line CD, and points C and D lie on
 * opposite sides of line AB.
 * 
 * This is division-free and completely avoids floating-point division-by-zero errors.
 */
function segmentsIntersect(A, B, C, D) {
    return ccw(A, C, D) !== ccw(B, C, D) && ccw(A, B, C) !== ccw(A, B, D);
}

/**
 * Returns true if the straight line segment between p1 and p2 intersects any boundary
 * or inner island wall of the pocket, meaning the direct line-of-sight is blocked.
 * Used to verify star-convexity and prevent merging chambers around non-convex bends/corners.
 */
function lineOfSightBlocked(p1, p2, polygons) {
    for (let poly of polygons) {
        let pts = poly.points;
        for (let i = 0; i < pts.length; i++) {
            let q1 = pts[i];
            let q2 = pts[(i + 1) % pts.length];
            if (segmentsIntersect(p1, p2, q1, q2)) {
                return true;
            }
        }
        if (poly.inner) {
            for (let inner of poly.inner) {
                let ipts = inner.points;
                for (let i = 0; i < ipts.length; i++) {
                    let q1 = ipts[i];
                    let q2 = ipts[(i + 1) % ipts.length];
                    if (segmentsIntersect(p1, p2, q1, q2)) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

/**
 * Finds the shortest path of MATNodes between startNode and endNode on the MAT graph
 * using Breadth-First Search (BFS).
 */
function findMATPath(startNode, endNode) {
    let visited = new Set([ startNode ]);
    let queue = [ [ startNode ] ];
    while (queue.length > 0) {
        let path = queue.shift();
        let last = path[path.length - 1];
        if (last === endNode) {
            return path;
        }
        for (let neighbor of last.neighbors) {
            if (!visited.has(neighbor)) {
                visited.add(neighbor);
                queue.push([ ...path, neighbor ]);
            }
        }
    }
    return null;
}

/**
 * Verifies that node radii along the path from the main center are monotonically decreasing.
 * This guarantees that the channel never expands (which would form "shoulders" / shadow zones
 * violating star-convexity) as we move farther away from the spiral center.
 * 
 * Allows a small tolerance (default 0.1mm) to account for minor noise in MAT digitization.
 */
function isPathMonotonic(path, tolerance = 0.1) {
    for (let i = 1; i < path.length; i++) {
        if (path[i].radius > path[i-1].radius + tolerance) {
            return false;
        }
    }
    return true;
}

/**
 * Local vertex snapping filter: snaps wobbly boundary vertices back to the pocket walls.
 * If a vertex is near a corner vertex of the wall, it snaps directly to the corner vertex
 * to preserve sharp geometry. Otherwise, it snaps to the closest wall segment.
 */
function snapPolygonToWalls(poly, wallPolygons, tolerance = 0.15) {
    let pts = poly.points;
    let newPts = [];

    for (let pt of pts) {
        let bestSegDist = Infinity;
        let bestSegProj = null;

        // 1. Find the closest projection point on the wall segments
        for (let wallPoly of wallPolygons) {
            let wallPts = wallPoly.points;
            for (let i = 0; i < wallPts.length; i++) {
                let a = wallPts[i];
                let b = wallPts[(i + 1) % wallPts.length];

                let vx = b.x - a.x;
                let vy = b.y - a.y;
                let len2 = vx * vx + vy * vy;

                let projX, projY, dist;
                if (len2 < 1e-9) {
                    projX = a.x;
                    projY = a.y;
                } else {
                    let t = ((pt.x - a.x) * vx + (pt.y - a.y) * vy) / len2;
                    t = Math.max(0, Math.min(1, t));
                    projX = a.x + t * vx;
                    projY = a.y + t * vy;
                }

                dist = Math.hypot(pt.x - projX, pt.y - projY);
                if (dist < bestSegDist) {
                    bestSegDist = dist;
                    bestSegProj = { x: projX, y: projY };
                }
            }

            // Handle inner loops (islands)
            if (wallPoly.inner) {
                for (let innerPoly of wallPoly.inner) {
                    let innerPts = innerPoly.points;
                    for (let i = 0; i < innerPts.length; i++) {
                        let a = innerPts[i];
                        let b = innerPts[(i + 1) % innerPts.length];

                        let vx = b.x - a.x;
                        let vy = b.y - a.y;
                        let len2 = vx * vx + vy * vy;

                        let projX, projY, dist;
                        if (len2 < 1e-9) {
                            projX = a.x;
                            projY = a.y;
                        } else {
                            let t = ((pt.x - a.x) * vx + (pt.y - a.y) * vy) / len2;
                            t = Math.max(0, Math.min(1, t));
                            projX = a.x + t * vx;
                            projY = a.y + t * vy;
                        }

                        dist = Math.hypot(pt.x - projX, pt.y - projY);
                        if (dist < bestSegDist) {
                            bestSegDist = dist;
                            bestSegProj = { x: projX, y: projY };
                        }
                    }
                }
            }
        }

        // 2. Find the closest corner vertex of the wall polygons
        let bestVertexDist = Infinity;
        let bestVertex = null;

        for (let wallPoly of wallPolygons) {
            let wallPts = wallPoly.points;
            for (let wp of wallPts) {
                let dist = Math.hypot(pt.x - wp.x, pt.y - wp.y);
                if (dist < bestVertexDist) {
                    bestVertexDist = dist;
                    bestVertex = wp;
                }
            }

            if (wallPoly.inner) {
                for (let innerPoly of wallPoly.inner) {
                    let innerPts = innerPoly.points;
                    for (let wp of innerPts) {
                        let dist = Math.hypot(pt.x - wp.x, pt.y - wp.y);
                        if (dist < bestVertexDist) {
                            bestVertexDist = dist;
                            bestVertex = wp;
                        }
                    }
                }
            }
        }

        // 3. Choose the snapping target: snap directly to the corner vertex if near a corner
        if (bestVertexDist < tolerance && (bestVertexDist < bestSegDist + 0.02 || bestVertexDist < 0.05)) {
            newPts.push(newPoint(bestVertex.x, bestVertex.y, pt.z));
        } else if (bestSegDist < tolerance && bestSegProj) {
            newPts.push(newPoint(bestSegProj.x, bestSegProj.y, pt.z));
        } else {
            newPts.push(pt.clone());
        }
    }

    return newPolygon().addPoints(newPts);
}

/**
 * Generates a continuous spiral path that morphs from the center generator to the chamber boundary.
 * Uses global interpolation between a circle and the target offset shapes to distribute the morph
 * smoothly across the entire spiral.
 */
function generateChamberSpiral(chamberBoundary, generator, targetStepover, ccw, z, toolRadius, options) {
    let loops = [ chamberBoundary.clone() ];
    let current = chamberBoundary;

    // Find the center point of the generator
    // We sort the nodes by radius descending to ensure we pick a real peak node as the center,
    // which prevents the center from falling outside non-convex (e.g. L-shaped) merged chambers.
    let sortedNodes = generator.nodes.slice().sort((a, b) => b.radius - a.radius);
    let genCenter = { x: sortedNodes[0].x, y: sortedNodes[0].y };

    // Calculate maximum safe helix radius (since matRadius is un-inflated distance to offset wall)
    let matRadius = sortedNodes[0].radius;
    let helixRadius = Math.max(0, Math.min(matRadius, 0.95 * toolRadius));

    // Compute the maximum straight-line distance from the peak center to the boundary of the shape
    // using the inscribed circles of the MAT nodes.
    let maxD = matRadius;
    for (let n of generator.nodes) {
        let d = Math.hypot(n.x - genCenter.x, n.y - genCenter.y);
        if (d + n.radius > maxD) {
            maxD = d + n.radius;
        }
    }

    // Determine the number of morphing turns based on target stepover
    let N = Math.ceil(maxD / targetStepover);

    let inner_circle;
    if (helixRadius > 0.05) {
        inner_circle = newPolygon().centerCircle(newPoint(genCenter.x, genCenter.y, z), helixRadius, 72);
    } else {
        inner_circle = newPolygon().centerCircle(newPoint(genCenter.x, genCenter.y, z), 0.01, 8);
    }

    let l_outer = chamberBoundary;
    let p_circle_len = inner_circle.perimeter();
    let p_outer_len = l_outer.perimeter();

    let path_pts = [];

    for (let i = 0; i <= N; i++) {
        let isCleanup = (i === N); // Final 360-degree cleanup loop

        // Estimate perimeter for the current turn using linear interpolation
        let t_mid = isCleanup ? 1.0 : (i + 0.5) / Math.max(1, N);
        let perimeter_i = (1 - t_mid) * p_circle_len + t_mid * p_outer_len;
        let numSteps = Math.max(72, Math.ceil(perimeter_i / 0.25));

        for (let j = 0; j < numSteps; j++) {
            let u = j / numSteps;
            let theta = ccw ? (u * 2 * Math.PI) : ((1 - u) * 2 * Math.PI);
            let dx = Math.cos(theta);
            let dy = Math.sin(theta);

            // Inner circle projection point at current angle
            let p_circle = getRayIntersection(genCenter, dx, dy, inner_circle);
            
            // Outer boundary projection point at current angle
            let p_outer = getRayIntersection(genCenter, dx, dy, l_outer);

            // Interpolation weight: morphs linearly from 0 (at loop 0 start) to 1 (at outer boundary)
            let t = isCleanup ? 1.0 : Math.min(1.0, (i + u) / Math.max(1, N));

            // Apply linear morphing to ensure constant, uniform stepover along the ray direction
            let px = (1 - t) * p_circle.x + t * p_outer.x;
            let py = (1 - t) * p_circle.y + t * p_outer.y;

            path_pts.push(newPoint(px, py, z));
        }
    }
    return newPolygon().addPoints(path_pts).setOpen();
}

/**
 * Generates D-shaped trochoidal cutting paths along a single corridor segment A -> B.
 * 
 * Each trochoidal loop consists of:
 * 1. A semi-circular cutting arc extending outward to the pocket boundary wall (feed rate).
 * 2. A retract transition (safe-Z lift-off, rapid move to the start of the next loop, and plunge).
 * 
 * Inputs:
 * - A, B: Endpoints of the segment (must have .x, .y, and .radius properties).
 * - targetStepover: The maximum forward stepover distance per loop.
 * - toolRadius: The radius of the cutting tool.
 * - ccw: Boolean indicating rotation direction (true = Counter-Clockwise, false = Clockwise).
 * - z: The active cutting depth.
 */
function generateTrochoidSegment(A, B, targetStepover, toolRadius, ccw, z) {
    let pts = [];
    
    // 1. Calculate segment length and unit direction vectors
    let L = Math.hypot(B.x - A.x, B.y - A.y);
    if (L < 1e-4) return []; // Skip zero-length segments

    let dx = (B.x - A.x) / L; // Unit vector along the segment (X component)
    let dy = (B.y - A.y) / L; // Unit vector along the segment (Y component)
    
    // Normal vector perpendicular to the segment, pointing to the side (used for lateral loop offset)
    let n = { x: -dy, y: dx }; 

    // 2. Subdivide the segment length into N equal loops
    // We divide L by targetStepover and round up to ensure loop stepover S is <= targetStepover,
    // which guarantees that the start point A and end point B are exactly aligned.
    let N = Math.ceil(L / targetStepover);
    let S = L / N; // Exact stepover distance per loop

    let prevArcEnd = null;
    // 3. Generate individual trochoidal loops
    for (let i = 0; i < N; i++) {
        let s0 = i * S;       // Start distance of current loop along segment
        let s1 = (i + 1) * S;   // End distance of current loop along segment

        // Interpolate the starting coordinates and local MAT radius for this loop
        let p0 = {
            x: A.x + s0 * dx,
            y: A.y + s0 * dy,
            radius: A.radius + (s0 / L) * (B.radius - A.radius)
        };
        
        // Interpolate the ending coordinates and local MAT radius for this loop
        let p1 = {
            x: A.x + s1 * dx,
            y: A.y + s1 * dy,
            radius: A.radius + (s1 / L) * (B.radius - A.radius)
        };

        // Calculate the lateral cut width W.
        // This is the maximum distance from the centerline to the pocket boundary wall (offset_polygons).
        // Since MAT nodes are un-inflated (addedRadius = 0), this is directly equal to p1.radius.
        let W = Math.max(0.05, p1.radius);

        // 4. Interpolate the semi-circular cutting arc
        // We generate 16 steps (17 coordinates) representing a half-circle sweep.
        // The sweep starts at the back, curves outward to the wall at width W, and terminates at the front.
        let arcPts = [];
        let steps = 16;
        for (let k = 0; k <= steps; k++) {
            let u = k / steps;
            // Map u [0..1] to theta [-PI/2..PI/2] depending on cutting rotation direction
            let theta = ccw ? (-Math.PI/2 + u * Math.PI) : (Math.PI/2 - u * Math.PI);

            // Compute coordinate by adding lateral offset (normal vector * sin) and forward offset (tangent vector * cos)
            let px = p0.x + W * n.x * Math.sin(theta) + S * dx * Math.cos(theta);
            let py = p0.y + W * n.y * Math.sin(theta) + S * dy * Math.cos(theta);
            arcPts.push(newPoint(px, py, z));
        }

        // Add the cutting arc points to the accumulator
        pts.push(...arcPts);

        // 5. Generate the safe-Z rapid retract transit back to the centerline
        // Compute the lateral width of the NEXT loop to find where the tool needs to land.
        let W_next;
        if (i < N - 1) {
            let s2 = (i + 2) * S;
            let r2 = A.radius + (Math.min(L, s2) / L) * (B.radius - A.radius);
            W_next = Math.max(0.05, r2);
        } else {
            W_next = Math.max(0.05, B.radius);
        }

        // Compute the landing coordinate for the start of the next cutting loop
        let nextStartPt = ccw ?
            { x: p1.x - W_next * n.x, y: p1.y - W_next * n.y } :
            { x: p1.x + W_next * n.x, y: p1.y + W_next * n.y };

        // Insert transit coordinates to execute a retract-rapid-plunge:
        // - Cut backwards to the end of the previous arc, if there was one
        // - Lift tool straight up to safe clearance (z + 0.1)
        // - Rapid travel perpendicular to the path to the start point of the arc we just cut
        // - Plunge back do the cut depth
        // - Cut to the start of the next arc
        let firstPt = arcPts[0];
        let lastPt = arcPts[arcPts.length - 1];
        if (prevArcEnd != null) {
            pts.push(newPoint(prevArcEnd.x, prevArcEnd.y, z));
            lastPt = prevArcEnd;
        }
        pts.push(newPoint(lastPt.x, lastPt.y, z + 0.1));
        pts.push(newPoint(firstPt.x, firstPt.y, z + 0.1));
        pts.push(newPoint(firstPt.x, firstPt.y, z));
        pts.push(newPoint(nextStartPt.x, nextStartPt.y, z));

        // update the previous arc end
        prevArcEnd = arcPts[steps-1];
    }

    return pts;
}

/**
 * Chains segments into continuous skeleton paths (greedy search)
 */
function chainSegments(segs) {
    let paths = [];
    let unvisited = new Set(segs);
    while (unvisited.size > 0) {
        let startSeg = unvisited.values().next().value;
        unvisited.delete(startSeg);
        let currentPath = [ startSeg.p0, startSeg.p1 ];
        let growing = true;
        while (growing) {
            growing = false;
            let endPt = currentPath[currentPath.length - 1];
            let startPt = currentPath[0];
            for (let s of unvisited) {
                if (Math.hypot(s.p0.x - endPt.x, s.p0.y - endPt.y) < 1e-4) {
                    currentPath.push(s.p1);
                    unvisited.delete(s);
                    growing = true;
                    break;
                }
                if (Math.hypot(s.p1.x - endPt.x, s.p1.y - endPt.y) < 1e-4) {
                    currentPath.push(s.p0);
                    unvisited.delete(s);
                    growing = true;
                    break;
                }
                if (Math.hypot(s.p0.x - startPt.x, s.p0.y - startPt.y) < 1e-4) {
                    currentPath.unshift(s.p1);
                    unvisited.delete(s);
                    growing = true;
                    break;
                }
                if (Math.hypot(s.p1.x - startPt.x, s.p1.y - startPt.y) < 1e-4) {
                    currentPath.unshift(s.p0);
                    unvisited.delete(s);
                    growing = true;
                    break;
                }
            }
        }
        paths.push(currentPath);
    }
    return paths;
}

/**
 * Builds a Medial Axis Transform (MAT) graph from flat segments returned by JSPoly.
 */
export function buildMATGraph(segments, toolRadius, scale = 1000) {
    const nodeMap = new Map();

    function getOrCreateNode(pt) {
        const key = `${pt.x|0},${pt.y|0}`;
        let node = nodeMap.get(key);
        if (!node) {
            node = new MATNode(pt.x / scale, pt.y / scale, (pt.radius / scale) + toolRadius);
            nodeMap.set(key, node);
        }
        return node;
    }

    for (const segment of segments) {
        const u = getOrCreateNode(segment.point0);
        const v = getOrCreateNode(segment.point1);

        if (u !== v) {
            u.neighbors.add(v);
            v.neighbors.add(u);
        }
    }

    return Array.from(nodeMap.values());
}

/**
 * ─────────────────────────────────────────────────────────────────────────────
 * ADAPTIVE PATH PLANNING: MAT GRAPH CLASSIFICATION ALGORITHM
 * ─────────────────────────────────────────────────────────────────────────────
 */
export function adaptiveClear(polygons, toolDiam, stepover, options) {
    let toolRadius = toolDiam / 2;
    let threshold = 1.25 * toolDiam;
    let gradientLimit = 0.5;
    let zSafe = options.zSafe ?? ((options.zTop ?? 0.0) + 2.0);

    let teaRad = (options.tea ?? 60) * Math.PI / 180;
    let targetStepover = toolDiam * Math.sin(teaRad / 2) * Math.sin(teaRad / 2);

    // The medial axis (MAT) segments are always computed on offset_polygons.
    // We keep addedRadius as 0 so that the chamber classification operates on un-inflated radii,
    // which prevents chambers from swallowing narrow slots/corridors.
    let addedRadius = 0;

    let offset_polygons = POLY.offset(polygons, [ -toolRadius ], { z: options.z });

    let mat_segments = [];
    let scale = 1000;

    for (let poly of offset_polygons) {
        try {
            let scaled = poly.clone(true).scale({ x: scale, y: scale, z: scale });
            let out = scaled.points.map(p => ({ x: p.x|0, y: p.y|0 }));
            let inr = (scaled.inner ?? []).map(p => p.points.map(p => ({ x: p.x|0, y: p.y|0 })));
            if (out.length > 2) {
                let ma = JSPoly.construct_medial_axis(out, inr);
                mat_segments.push(...ma);
            }
        } catch (e) {
            console.warn("MAT construction failed for a pocket in adaptiveClear", e);
        }
    }

    if (mat_segments.length === 0) {
        options.chamber_lines = [];
        options.corridor_lines = [];
        options.mat_lines = [];
        options.generators = [];
        options.chamber_areas = [];
        options.corridor_areas = [];
        options.meta_graph = [];
        options.meta_walks = [];
        return polygons;
    }

    let mat_graph = buildMATGraph(mat_segments, addedRadius, scale);

    // Heal numerical gaps in the MAT graph by connecting close endpoints safely.
    // We connect points if and only if they are within each others' radius (d <= u.radius && d <= v.radius),
    // which mathematically guarantees the connection lies entirely within the allowable pocket area
    // without crossing any walls or boundaries.
    let endpoints = mat_graph.filter(n => n.neighbors.size <= 1);
    for (let i = 0; i < endpoints.length; i++) {
        let u = endpoints[i];
        let best_v = null;
        let best_d = Infinity;
        for (let j = i + 1; j < endpoints.length; j++) {
            let v = endpoints[j];
            if (u.neighbors.has(v)) continue;
            let d = dist2D(u, v);
            if (d <= u.radius && d <= v.radius && d < best_d) {
                best_d = d;
                best_v = v;
            }
        }
        if (best_v) {
            u.neighbors.add(best_v);
            best_v.neighbors.add(u);
        }
    }

    let unvisitedNodes = new Set(mat_graph);
    let pocket_components = [];

    while (unvisitedNodes.size > 0) {
        let startNode = unvisitedNodes.values().next().value;
        let component = [];
        let queue = [ startNode ];
        let visited = new Set([ startNode ]);

        while (queue.length > 0) {
            let curr = queue.shift();
            component.push(curr);
            unvisitedNodes.delete(curr);

            for (let neighbor of curr.neighbors) {
                if (!visited.has(neighbor)) {
                    visited.add(neighbor);
                    queue.push(neighbor);
                }
            }
        }
        pocket_components.push(component);
    }

    let generators = [];
    let generatorIdCounter = 0;

    for (let pocket_component of pocket_components) {
        let localMaxNodes = [];
        for (let node of pocket_component) {
            if (node.radius < threshold) continue;
            let isMax = true;
            for (let n of node.neighbors) {
                if (n.radius > node.radius) {
                    isMax = false;
                    break;
                }
            }
            if (isMax) {
                localMaxNodes.push(node);
            }
        }

        let pocket_generators = [];
        let visitedMax = new Set();

        for (let startNode of localMaxNodes) {
            if (visitedMax.has(startNode)) continue;

            let genNodes = [];
            let queue = [ startNode ];
            visitedMax.add(startNode);

            while (queue.length > 0) {
                let node = queue.shift();
                genNodes.push(node);

                for (let n of node.neighbors) {
                    if (!visitedMax.has(n) && Math.abs(n.radius - node.radius) < 1e-5 && localMaxNodes.includes(n)) {
                        visitedMax.add(n);
                        queue.push(n);
                    }
                }
            }

            let genId = ++generatorIdCounter;
            for (let node of genNodes) {
                node.isGenerator = true;
                node.chamberId = genId;
            }

            pocket_generators.push({
                id: genId,
                nodes: genNodes,
                type: 'chamber'
            });
        }

        // Fallback: If this component has no standard chambers (no nodes above threshold),
        // we create a special topological 'entry' node.
        // To guarantee clean loop breaking and prevent leaving uncut regions, the entry node
        // contains strictly the one absolute maximum node.
        if (pocket_generators.length === 0 && pocket_component.length > 0) {
            let absMaxNode = pocket_component[0];
            for (let node of pocket_component) {
                if (node.radius > absMaxNode.radius) {
                    absMaxNode = node;
                } else if (node.radius === absMaxNode.radius) {
                    if (node.x > absMaxNode.x) {
                        absMaxNode = node;
                    } else if (node.x === absMaxNode.x && node.y > absMaxNode.y) {
                        absMaxNode = node;
                    }
                }
            }

            let genId = ++generatorIdCounter;
            let genNodes = [ absMaxNode ];
            absMaxNode.isGenerator = true;
            absMaxNode.chamberId = genId;

            pocket_generators.push({
                id: genId,
                nodes: genNodes,
                type: 'entry'
            });
        }

        generators.push(...pocket_generators);
    }

    // Initialize peak nodes for each chamber
    for (let g of generators.filter(g => g.type === 'chamber')) {
        let sorted = g.nodes.slice().sort((a, b) => b.radius - a.radius);
        g.peakNodes = [ sorted[0] ];
    }

    // 4.5 Merge adjacent chambers if they preserve star-convexity (line of sight and MAT path monotonicity)
    let mergedChamberIds = new Set();
    let mergePass = true;
    while (mergePass) {
        mergePass = false;
        let activeChambers = generators.filter(g => g.type === 'chamber' && !mergedChamberIds.has(g.id));
        let bestMerge = null;
        
        for (let i = 0; i < activeChambers.length; i++) {
            let g1 = activeChambers[i];
            
            // Run BFS from g1's nodes to find paths to other active chambers on the MAT graph
            let visited = new Set(g1.nodes);
            let queue = g1.nodes.map(n => ({ node: n, minR: n.radius }));
            
            while (queue.length > 0) {
                let { node, minR } = queue.shift();
                
                // If we reached a node belonging to a different active chamber
                if (node.chamberId !== null && node.chamberId !== g1.id) {
                    let g2 = activeChambers.find(g => g.id === node.chamberId);
                    if (g2 && !mergedChamberIds.has(g2.id)) {
                        // Gather peaks and find candidate center of prospective merged chamber
                        let allPeaks = [...g1.peakNodes, ...g2.peakNodes];
                        let mainCenter = allPeaks.slice().sort((a, b) => b.radius - a.radius)[0];
                        
                        // Star-Convexity Verification:
                        // 1. Line-of-Sight must be clear between center and all other peaks.
                        // 2. MAT path radii must be monotonically decreasing away from the center.
                        let isStarConvex = true;
                        for (let peak of allPeaks) {
                            if (peak !== mainCenter) {
                                if (lineOfSightBlocked(mainCenter, peak, offset_polygons)) {
                                    isStarConvex = false;
                                    break;
                                }
                                let path = findMATPath(mainCenter, peak);
                                if (!path || !isPathMonotonic(path, 0.1)) {
                                    isStarConvex = false;
                                    break;
                                }
                            }
                        }
                        
                        if (isStarConvex) {
                            // Track the merge candidate with the widest bottleneck neck
                            if (!bestMerge || minR > bestMerge.minR) {
                                bestMerge = { g1, g2, minR, allPeaks };
                            }
                        }
                    }
                    continue; // Stop expanding along this branch (different chamber reached)
                }
                
                for (let neighbor of node.neighbors) {
                    if (!visited.has(neighbor)) {
                        visited.add(neighbor);
                        queue.push({ node: neighbor, minR: Math.min(minR, neighbor.radius) });
                    }
                }
            }
        }
        
        // Execute the best merge from this iteration and mark transitive check
        if (bestMerge) {
            let { g1, g2, allPeaks } = bestMerge;
            for (let n of g2.nodes) {
                n.chamberId = g1.id;
                g1.nodes.push(n);
            }
            g1.peakNodes = allPeaks;
            mergedChamberIds.add(g2.id);
            mergePass = true; // Repeat to handle transitive merges
        }
    }
    
    // Filter out the merged generators from the global generators list
    generators = generators.filter(g => !mergedChamberIds.has(g.id));

    // 5. Run BFS Chamber Expansion (restricted to standard 'chamber' generators)
    for (let gen of generators.filter(g => g.type === 'chamber')) {
        let queue = [];
        for (let node of gen.nodes) {
            for (let neighbor of node.neighbors) {
                if (neighbor.chamberId === null) {
                    queue.push({ node: neighbor, fromNode: node });
                }
            }
        }

        while (queue.length > 0) {
            let { node, fromNode } = queue.shift();
            if (node.chamberId !== null) continue;

            let ds = dist2D(fromNode, node);
            let dR = Math.abs(fromNode.radius - node.radius);
            let G = ds > 1e-6 ? dR / ds : 0;

            if (G < gradientLimit && node.radius < threshold) {
                continue;
            }

            node.chamberId = gen.id;
            for (let neighbor of node.neighbors) {
                if (neighbor.chamberId === null) {
                    queue.push({ node: neighbor, fromNode: node });
                }
            }
        }
    }

    // 6. Categorize MAT segments based on the node classifications
    let chamber_lines = [];
    let corridor_lines = [];
    let mat_lines = [];
    let renderedEdges = new Set();

    for (let node of mat_graph) {
        for (let neighbor of node.neighbors) {
            let key = node.x < neighbor.x ?
                `${node.x},${node.y}:${neighbor.x},${neighbor.y}` :
                `${neighbor.x},${neighbor.y}:${node.x},${node.y}`;

            if (renderedEdges.has(key)) continue;
            renderedEdges.add(key);

            let p0 = { x: node.x, y: node.y, radius: node.radius };
            let p1 = { x: neighbor.x, y: neighbor.y, radius: neighbor.radius };
            let segment = { point0: p0, point1: p1 };

            mat_lines.push(segment);

            if (node.chamberId !== null && node.chamberId === neighbor.chamberId) {
                chamber_lines.push(segment);
            } else {
                corridor_lines.push(segment);
            }
        }
    }

    // 7. Generate area polygons (tapered capsules)
    let z = options.z;
    let chamber_capsules = [];
    let corridor_capsules = [];

    function makeSingleCapsule(p0, p1) {
        let dx = p1.x - p0.x;
        let dy = p1.y - p0.y;
        let len = Math.hypot(dx, dy);
        let r0 = p0.radius;
        let r1 = p1.radius;

        if (len < 1e-6) {
            return [ newPolygon().centerCircle(newPoint(p0.x, p0.y, z), r0, 12) ];
        }

        let nx = -dy / len;
        let ny = dx / len;

        let pts = [
            newPoint(p0.x + nx * r0, p0.y + ny * r0, z),
            newPoint(p1.x + nx * r1, p1.y + ny * r1, z),
            newPoint(p1.x - nx * r1, p1.y - ny * r1, z),
            newPoint(p0.x - nx * r0, p0.y - ny * r0, z)
        ];

        let poly = newPolygon().addPoints(pts);
        let c0 = newPolygon().centerCircle(newPoint(p0.x, p0.y, z), r0, 12);
        let c1 = newPolygon().centerCircle(newPoint(p1.x, p1.y, z), r1, 12);

        return POLY.union([ poly, c0, c1 ], 0.00001, true);
    }

    function makeTaperedCapsule(p0, p1) {
        let dx = p1.x - p0.x;
        let dy = p1.y - p0.y;
        let len = Math.hypot(dx, dy);

        let sub_capsules = [];
        let max_len = 1.0;
        let num_subs = Math.ceil(len / max_len);

        for (let i = 0; i < num_subs; i++) {
            let t0 = i / num_subs;
            let t1 = (i + 1) / num_subs;

            let sp0 = {
                x: p0.x + t0 * dx,
                y: p0.y + t0 * dy,
                radius: p0.radius + t0 * (p1.radius - p0.radius)
            };
            let sp1 = {
                x: p0.x + t1 * dx,
                y: p0.y + t1 * dy,
                radius: p0.radius + t1 * (p1.radius - p0.radius)
            };

            sub_capsules.push(...makeSingleCapsule(sp0, sp1));
        }
        return sub_capsules;
    }

    for (let seg of chamber_lines) {
        chamber_capsules.push(...makeTaperedCapsule(seg.point0, seg.point1));
    }
    for (let seg of corridor_lines) {
        corridor_capsules.push(...makeTaperedCapsule(seg.point0, seg.point1));
    }

    let raw_chamber_area = chamber_capsules.length ? POLY.union(chamber_capsules, 0.00001, true) : [];
    let raw_corridor_area = corridor_capsules.length ? POLY.union(corridor_capsules, 0.00001, true) : [];

    options.chamber_areas = raw_chamber_area.length ? POLY.trimTo(raw_chamber_area, polygons) : [];
    options.corridor_areas = raw_corridor_area.length ? POLY.trimTo(raw_corridor_area, polygons) : [];

    // 8. Build the Meta-Graph of chambers, entries, and corridors
    let metaNodes = [];
    let metaNodeIdCounter = 0;

    // A. Create Chamber and Entry MetaNodes
    for (let gen of generators) {
        let strategy = gen.type; // 'chamber' or 'entry'
        let meta = new MetaNode(++metaNodeIdCounter, strategy, gen);
        for (let node of mat_graph) {
            if (node.chamberId === gen.id) {
                node.metaNode = meta;
                meta.nodes.push(node);
            }
        }
        metaNodes.push(meta);
    }

    // B. Create Corridor MetaNodes.
    // To ensure every Corridor MetaNode contains strictly a single, non-branching chain
    // of nodes, we stop growing the corridor component whenever we encounter a branching
    // junction node (node.neighbors.size > 2). Junction nodes themselves are packaged
    // into their own separate size-1 Corridor MetaNodes.
    let visitedCorridor = new Set();
    for (let startNode of mat_graph) {
        if (startNode.chamberId !== null || visitedCorridor.has(startNode)) {
            continue;
        }

        let meta = new MetaNode(++metaNodeIdCounter, 'corridor', null);

        if (startNode.neighbors.size > 2) {
            // Junction node: put only this node in the meta-node and stop
            startNode.metaNode = meta;
            meta.nodes.push(startNode);
            visitedCorridor.add(startNode);
        } else {
            // Regular corridor node: grow component until we hit a junction or chamber
            let queue = [ startNode ];
            visitedCorridor.add(startNode);

            while (queue.length > 0) {
                let node = queue.shift();
                node.metaNode = meta;
                meta.nodes.push(node);

                for (let neighbor of node.neighbors) {
                    if (neighbor.chamberId === null && !visitedCorridor.has(neighbor)) {
                        if (neighbor.neighbors.size > 2) {
                            // Stop growing before entering the junction node
                            continue;
                        }
                        visitedCorridor.add(neighbor);
                        queue.push(neighbor);
                    }
                }
            }
        }
        metaNodes.push(meta);
    }

    /**
     * Orders the nodes inside a Corridor MetaNode in-place to form a clean linear chain
     * running from one end of the corridor to the other.
     *
     * A Corridor MetaNode initially contains an unordered bag of nodes grouped by a BFS traversal.
     * To generate a continuous linear toolpath, we order these nodes sequentially.
     * We do this by traversing the `.neighbors` of each node grouped within the MetaNode to collect
     * all segments (including original MAT edges and healed gap connections). These segments are
     * then chained together using a greedy matching algorithm to find the longest contiguous backbone path.
     */
    function orderMetaNodes(m) {
        if (m.strategy === 'chamber' || m.strategy === 'entry') {
            // Chambers/entries are already grouped/ordered appropriately by their generator
            return;
        } else {
            let segs = [];
            let seen = new Set(); // Tracks undirected edges to avoid duplicate segments

            // Traverse all nodes grouped in this corridor MetaNode
            for (let u of m.nodes) {
                // Check all graph neighbors of node 'u'
                for (let v of u.neighbors) {
                    // Only build segments if the neighbor 'v' is also grouped inside this same MetaNode
                    if (v.metaNode === m) {
                        /**
                         * DEDUPLICATION:
                         * Since the graph is undirected, the edge between u and v is traversed twice:
                         * once when processing u (u -> v), and once when processing v (v -> u).
                         * To prevent adding duplicate segment objects (which would cause chaining loops),
                         * we construct a coordinate-based unique key where the smaller coordinate pair is
                         * always listed first. This uniquely identifies the undirected edge.
                         */
                        let key = (u.x < v.x || (u.x === v.x && u.y < v.y)) ?
                            `${u.x},${u.y}-${v.x},${v.y}` : `${v.x},${v.y}-${u.x},${u.y}`;

                        // If this undirected edge has not been processed yet, record it
                        if (!seen.has(key)) {
                            seen.add(key);
                            segs.push({ p0: u, p1: v });
                        }
                    }
                }
            }

            // Chain the segments together sequentially using a greedy matching algorithm
            let paths = chainSegments(segs);
            if (paths.length > 0) {
                // If branches exist or numerical splits occurred, sort paths by length descending
                paths.sort((a, b) => b.length - a.length);
                // Assign the longest continuous chain as the ordered nodes list for this corridor
                m.nodes = paths[0];
            }
        }
    }

    // Order nodes in-place for all meta-nodes immediately after construction
    for (let m of metaNodes) {
        orderMetaNodes(m);
    }

    // Establish Connections (Meta-Edges) in the Meta-Graph
    let connectedEdges = new Set();
    for (let node of mat_graph) {
        for (let neighbor of node.neighbors) {
            if (node.metaNode && neighbor.metaNode && node.metaNode !== neighbor.metaNode) {
                let key = node.x < neighbor.x ?
                    `${node.x},${node.y}:${neighbor.x},${neighbor.y}` :
                    `${neighbor.x},${neighbor.y}:${node.x},${node.y}`;

                if (connectedEdges.has(key)) continue;
                connectedEdges.add(key);

                let mu = node.metaNode;
                let mv = neighbor.metaNode;

                let nodesU = mu.nodes;
                let myEnd = false; // false = begin, true = end
                if (nodesU.length > 1) {
                    let d0 = dist2D(node, nodesU[0]);
                    let dK = dist2D(node, nodesU[nodesU.length - 1]);
                    myEnd = dK < d0;
                }

                let nodesV = mv.nodes;
                let itsEnd = false; // false = begin, true = end
                if (nodesV.length > 1) {
                    let d0 = dist2D(neighbor, nodesV[0]);
                    let dK = dist2D(neighbor, nodesV[nodesV.length - 1]);
                    itsEnd = dK < d0;
                }

                // If both meta-nodes are chambers, insert an intermediate Corridor MetaNode to clear the neck between them
                if (mu.strategy === 'chamber' && mv.strategy === 'chamber') {
                    let m_corr = new MetaNode(++metaNodeIdCounter, 'corridor', null);
                    m_corr.nodes = [node, neighbor];
                    metaNodes.push(m_corr);

                    // Connect Chamber A <-> Corridor
                    mu.neighbors.add(m_corr);
                    m_corr.neighbors.add(mu);
                    mu.connections.push({
                        neighbor: m_corr,
                        myEnd: myEnd,
                        itsEnd: false
                    });
                    m_corr.connections.push({
                        neighbor: mu,
                        myEnd: false,
                        itsEnd: myEnd
                    });

                    // Connect Corridor <-> Chamber B
                    m_corr.neighbors.add(mv);
                    mv.neighbors.add(m_corr);
                    m_corr.connections.push({
                        neighbor: mv,
                        myEnd: true,
                        itsEnd: itsEnd
                    });
                    mv.connections.push({
                        neighbor: m_corr,
                        myEnd: itsEnd,
                        itsEnd: true
                    });
                } else {
                    // Standard connection between different meta-nodes
                    mu.neighbors.add(mv);
                    mv.neighbors.add(mu);

                    let exists = mu.connections.push !== undefined && mu.connections.some(c => c.neighbor === mv && c.myEnd === myEnd && c.itsEnd === itsEnd);
                    if (!exists) {
                        mu.connections.push({
                            neighbor: mv,
                            myEnd: myEnd,
                            itsEnd: itsEnd
                        });
                        mv.connections.push({
                            neighbor: mu,
                            myEnd: itsEnd,
                            itsEnd: myEnd
                        });
                    }
                }
            }
        }
    }


    // Calculate representative coordinates for all MetaNodes using actual physical nodes
    let metaCenters = new Map();
    for (let m of metaNodes) {
        if (m.strategy === 'chamber' || m.strategy === 'entry') {
            // Use the peak node (largest radius) of the generator
            let sortedNodes = m.generator.nodes.slice().sort((a, b) => b.radius - a.radius);
            metaCenters.set(m.id, { x: sortedNodes[0].x, y: sortedNodes[0].y });
        } else {
            // Use the middle physical node along the ordered corridor nodes chain
            let midNode = m.nodes[Math.floor(m.nodes.length / 2)];
            metaCenters.set(m.id, { x: midNode.x, y: midNode.y });
        }
    }

    // 9. Pick the starting generator for each disconnected pocket component
    let unvisitedMeta = new Set(metaNodes);
    let all_walks = [];
    let active_generators = [];

    while (unvisitedMeta.size > 0) {
        let startNode = unvisitedMeta.values().next().value;
        let component = [];
        let queue = [ startNode ];
        let visitedComp = new Set([ startNode ]);

        while (queue.length > 0) {
            let curr = queue.shift();
            component.push(curr);
            unvisitedMeta.delete(curr);

            for (let neighbor of curr.neighbors) {
                if (!visitedComp.has(neighbor)) {
                    visitedComp.add(neighbor);
                    queue.push(neighbor);
                }
            }
        }

        let componentGenerators = [];
        for (let m of component) {
            if ((m.strategy === 'chamber' || m.strategy === 'entry') && m.generator) {
                componentGenerators.push(m.generator);
            }
        }

        let leftmostGen = null;
        let minX = Infinity;

        for (let gen of componentGenerators) {
            let sorted = gen.nodes.slice().sort((a, b) => b.radius - a.radius);
            let peak = sorted[0];
            if (peak.x < minX) {
                minX = peak.x;
                leftmostGen = gen;
            }
        }

        if (leftmostGen) {
            active_generators.push(leftmostGen);

            let startMetaNode = component.find(m => (m.strategy === 'chamber' || m.strategy === 'entry') && m.generator.id === leftmostGen.id);
            if (startMetaNode) {
                let componentMetaNodes = component;
                let adjMeta = new Map();
                for (let m of componentMetaNodes) {
                    adjMeta.set(m, []);
                }
                for (let m of componentMetaNodes) {
                    for (let neighbor of m.neighbors) {
                        if (componentMetaNodes.includes(neighbor)) {
                            adjMeta.get(m).push(neighbor);
                        }
                    }
                }

                let walk = [];
                let uncutNodes = new Set(component.map(m => m.id));
                let activeStack = new Set();

                /*
                  === POCKET META-GRAPH PATH RESOLUTION ===

                  You have the following structures available here:
                  - component: Array of MetaNode objects belonging to this connected pocket component.
                  - startMetaNode: The starting MetaNode (an entry or chamber) from which clearing begins.
                  - adjMeta: A Map mapping MetaNode to an Array of neighbor MetaNodes.
                  - walk: Array of step objects representing the ordered traversal. You need to populate this.
                  - all_walks: Array where you push the completed walk for this component.

                  MetaNode structure:
                  class MetaNode {
                      id: number,                          // Unique ID
                      strategy: string,                    // 'chamber', 'entry', or 'corridor'
                      nodes: MATNode[],                    // Original MAT graph nodes belonging to this meta-node
                      neighbors: Set<MetaNode>,            // Neighboring MetaNodes
                      connections: Connection[],           // Array of connections to neighbor MetaNodes
                      spine: Point[]                       // Ordered spine skeleton points (workspace coordinates)
                  }

                  Connection structure:
                  {
                      neighbor: MetaNode,                  // The neighboring MetaNode
                      myEnd: boolean,                      // Which end of this node's spine connects (false = begin, true = end)
                      itsEnd: boolean                      // Which end of the neighbor's spine connects (false = begin, true = end)
                  }

                  Step structure to push to walk:
                  {
                      id: number,                          // ID of the MetaNode being visited
                      strategy: string,                    // 'chamber', 'entry', or 'corridor'
                      first: boolean,                      // true if this is the first (cutting) visit, false if backtracking
                      enterEnd: boolean | null,            // Which end the tool enters (false = begin, true = end)
                      exitEnd: boolean | null,             // Which end the tool exits (false = begin, true = end)

                      // Resolved properties used by the toolpath generator:
                      resolvedEnterEnd: boolean,
                      resolvedExitEnd: boolean,

                      // Physical transition coordinates:
                      x: number,                           // Theoretical graph entry coordinate on the spine (legacy compatibility)
                      y: number,                           // Theoretical graph entry coordinate on the spine (legacy compatibility)
                      start_x: number,                     // Actual physical touchdown coordinate of the generated toolpath (can drift from x/y due to entry ramps/offsets)
                      start_y: number,                     // Actual physical touchdown coordinate of the generated toolpath (can drift from x/y due to entry ramps/offsets)
                      end_x: number,                       // Actual physical exit coordinate of the generated toolpath
                      end_y: number                        // Actual physical exit coordinate of the generated toolpath
                  }
                */

                function dfsWalk(curr, enterEnd) {
                    let isUncut = uncutNodes.has(curr.id);
                    if (isUncut) {
                        uncutNodes.delete(curr.id);
                    }

                    activeStack.add(curr.id);

                    let step = {
                        id: curr.id,
                        strategy: curr.strategy,
                        first: isUncut,
                        enterEnd: enterEnd,
                        exitEnd: null
                    };
                    walk.push(step);
                    let lastStep = step;

                    let currentEnd = enterEnd === null ? true : !enterEnd;

                    let neighbors = (adjMeta.get(curr) || []).slice();
                    while (true) {
                        let groupA = [];
                        let groupB = [];

                        for (let nb of neighbors) {
                            if (activeStack.has(nb.id)) {
                                continue;
                            }
                            if (uncutNodes.has(nb.id)) {
                                groupA.push(nb);
                            } else {
                                let nbNeighbors = adjMeta.get(nb) || [];
                                let hasUncutNeighbor = nbNeighbors.some(n => uncutNodes.has(n.id) && !activeStack.has(n.id));
                                if (hasUncutNeighbor) {
                                    groupB.push(nb);
                                }
                            }
                        }

                        let next = null;
                        if (groupA.length > 0) {
                            next = groupA[0];
                        } else if (groupB.length > 0) {
                            next = groupB[0];
                        }

                        if (!next) {
                            break;
                        }

                        let conn = curr.connections.find(c => c.neighbor === next && (curr.strategy !== 'corridor' || curr.nodes.length <= 1 || c.myEnd === currentEnd));
                        if (conn) {
                            lastStep.exitEnd = conn.myEnd;

                            dfsWalk(next, conn.itsEnd);

                            let backtrackStep = {
                                id: curr.id,
                                strategy: curr.strategy,
                                first: false,
                                enterEnd: conn.myEnd,
                                exitEnd: null
                            };
                            walk.push(backtrackStep);
                            lastStep = backtrackStep;

                            currentEnd = conn.myEnd;
                        } else {
                            // If the connection layout constraint is not met, remove the neighbor
                            // from our local copy of neighbors to prevent looping infinitely.
                            console.warn(`[MAT Warning] dfsWalk: Connection layout constraint not met between MetaNode ${curr.id} and MetaNode ${next.id} (required end: ${currentEnd}). Skipping neighbor.`);
                            let idx = neighbors.indexOf(next);
                            if (idx >= 0) {
                                neighbors.splice(idx, 1);
                            }
                        }
                    }

                    if (lastStep.exitEnd === null) {
                        if (lastStep.first) {
                            lastStep.exitEnd = enterEnd === null ? true : !enterEnd;
                        } else {
                            lastStep.exitEnd = enterEnd === null ? false : enterEnd;
                        }
                    }

                    activeStack.delete(curr.id);
                }

                dfsWalk(startMetaNode, null);

                // Prune trailing backtrack steps at the end of the walk
                while (walk.length > 0 && walk[walk.length - 1].first === false) {
                    walk.pop();
                }

                for (let i = 0; i < walk.length; i++) {
                    let step = walk[i];
                    let m = metaNodes.find(node => node.id === step.id);
                    if (m) {
                        let spine = m.nodes;
                        if (spine.length > 0) {
                            let enterBool = step.enterEnd === true;
                            let exitBool = step.exitEnd === true;

                            step.resolvedEnterEnd = enterBool;
                            step.resolvedExitEnd = exitBool;

                            let enterPt = !enterBool ? spine[0] : spine[spine.length - 1];
                            let exitPt = !exitBool ? spine[0] : spine[spine.length - 1];

                            step.x = enterPt.x;
                            step.y = enterPt.y;
                            step.start_x = enterPt.x;
                            step.start_y = enterPt.y;
                            step.end_x = exitPt.x;
                            step.end_y = exitPt.y;
                        }

                        // Determine helicalEntryNeeded statically for each step
                        if (i === 0) {
                            step.helicalEntryNeeded = true;
                        } else if (step.first === false) {
                            step.helicalEntryNeeded = false;
                        } else if (step.strategy === 'chamber' || step.strategy === 'entry') {
                            step.helicalEntryNeeded = true;
                        } else if (step.strategy === 'corridor') {
                            let prevStep = walk[i - 1];
                            let prevM = metaNodes.find(node => node.id === prevStep.id);
                            step.helicalEntryNeeded = prevM ? !m.neighbors.has(prevM) : true;
                        } else {
                            step.helicalEntryNeeded = false;
                        }
                    }
                }

                all_walks.push(walk);

                // RICH DEBUG LOGS FOR META-NODES AND DFS WALKS
                console.log(`=== [MAT Debug] Disconnected Pocket Component Walks ===`);
                for (let m of component) {
                    let spine = m.nodes;
                    let startPt = spine[0];
                    let endPt = spine[spine.length - 1];
                    console.log(`[MAT MetaNode Debug] Node ID: ${m.id}, Strategy: ${m.strategy}`);
                    if (startPt && endPt) {
                        console.log(`  Spine Start: (${startPt.x.toFixed(4)}, ${startPt.y.toFixed(4)}), End: (${endPt.x.toFixed(4)}, ${endPt.y.toFixed(4)}), Nodes count: ${spine.length}`);
                    } else {
                        console.log(`  No spine nodes found!`);
                    }
                    let edgesInfo = m.connections.map(c => `to Node ${c.neighbor.id} (${c.myEnd ? 'end' : 'begin'} -> ${c.itsEnd ? 'end' : 'begin'})`).join(', ');
                    console.log(`  Neighbors: ${edgesInfo || 'none'}`);
                }

                console.log(`[MAT DFS Walk Debug] Chosen Start Generator: ${leftmostGen.id} (MetaNode ID: ${startMetaNode.id})`);
                for (let stepIndex = 0; stepIndex < walk.length; stepIndex++) {
                    let step = walk[stepIndex];
                    let enterStr = step.resolvedEnterEnd !== undefined ? (step.resolvedEnterEnd ? 'end' : 'begin') : 'N/A';
                    let exitStr = step.resolvedExitEnd !== undefined ? (step.resolvedExitEnd ? 'end' : 'begin') : 'N/A';
                    console.log(`  Step ${stepIndex}: Node ID: ${step.id}, Strategy: ${step.strategy}, First/Cut Visit: ${step.first}, Enter End: ${enterStr}, Exit End: ${exitStr}`);
                }
                console.log(`====================================================`);
            }
        }
    }

    options.chamber_lines = chamber_lines;
    options.corridor_lines = corridor_lines;
    options.mat_lines = mat_lines;

    options.meta_graph = metaNodes.map(m => {
        let center = metaCenters.get(m.id);
        return {
            id: m.id,
            strategy: m.strategy,
            x: center.x,
            y: center.y,
            generator: m.generator ? {
                type: m.generator.nodes.length > 1 ? 'line' : 'point',
                nodes: m.generator.nodes.map(n => ({ x: n.x, y: n.y, radius: n.radius }))
            } : null,
            nodes: m.nodes.map(n => ({ x: n.x, y: n.y, radius: n.radius })),
            neighbors: Array.from(m.neighbors).map(n => n.id)
        };
    });

    options.meta_walks = all_walks;

    options.generators = active_generators.map(g => {
        return {
            type: g.nodes.length > 1 ? 'line' : 'point',
            nodes: g.nodes.map(n => ({ x: n.x, y: n.y, radius: n.radius }))
        };
    });

    // 10. Generate the actual toolpaths for chambers and corridors
    let toolpaths = [];
    let ccw = (options.direction === 'climb');
    let clearedMetaNodes = new Set();
    let pts = [];

    /**
     * Helper to push points to the continuous toolpath, automatically inserting
     * safe Z-hops for transits/backtracking, and plunges for cutting passes.
     */
    function pushPoints(newPts, isCut, forceOrder = false) {
        if (newPts.length === 0) return [];

        let oriented = newPts;
        if (!isCut && !forceOrder && pts.length > 0) {
            oriented = newPts.slice();
            let lastPt = pts[pts.length - 1];
            let dStart = Math.hypot(oriented[0].x - lastPt.x, oriented[0].y - lastPt.y);
            let dEnd = Math.hypot(oriented[oriented.length - 1].x - lastPt.x, oriented[oriented.length - 1].y - lastPt.y);
            if (dEnd < dStart) {
                oriented.reverse();
            }
        }

        // CRITICAL NOTE ON JS ARRAYS: We avoid using the spread operator push shortcut (e.g. pts.push(...array))
        // because JavaScript engines have a strict argument count limit (typically ~65,535). High-resolution
        // morphing spirals and dense trochoids can easily exceed this limit, causing a "Maximum call stack size
        // exceeded" RangeError. Iterative pushing with a loop is fully stack-safe.
        if (pts.length === 0) {
            if (isCut) {
                for (let p of newPts) {
                    pts.push(p.clone());
                }
            } else {
                for (let p of newPts) {
                    pts.push(p.clone().setZ(z + 0.1));
                }
            }
            return oriented;
        }

        let lastPt = pts[pts.length - 1];
        if (isCut) {
            if (lastPt.z > z + 0.05) {
                pts.push(newPoint(newPts[0].x, newPts[0].y, z + 0.1));
                pts.push(newPoint(newPts[0].x, newPts[0].y, newPts[0].z));
            }
            for (let p of newPts) {
                pts.push(p.clone());
            }
        } else {
            if (lastPt.z < z + 0.05) {
                pts.push(newPoint(lastPt.x, lastPt.y, z + 0.1));
                lastPt = pts[pts.length - 1];
            }
            for (let p of oriented) {
                pts.push(p.clone().setZ(z + 0.1));
            }
        }
        return oriented;
    }

    for (let walk of all_walks) {
        if (walk.length === 0) continue;

        let metaSpines = new Map(); // Cache spines in their first traversed direction

        // Helper: Generate helical entry points centered at a point
        function makeHelicalEntry(centerPt, generatorNode) {
            let matRadius = generatorNode.radius;
            let helixRadius = Math.max(0, Math.min(0.5 * matRadius, 0.95 * toolRadius));
            let zTop = options.zTop ?? 0.0;
            let zStart = Math.min(zTop, z + (options.down ?? 2.0));
            let helixPts = [];

            if (helixRadius > 0.05) {
                let rampAngle = (options.entry_helix_angle ?? 3) * Math.PI / 180;
                let pitch = 2 * Math.PI * helixRadius * Math.tan(rampAngle);

                // Add exactly one full turn's worth of Z-height (pitch) to zStart and the descent height H.
                // This ensures the first turn is performed in empty air, so that the tool is already
                // in full helical cutting motion before it makes physical contact with the stock surface.
                zStart += pitch;
                let H = (zStart - pitch - z) + pitch; // equivalent to H_solid + pitch

                let numTurns = Math.max(1, H / pitch);
                let totalRad = numTurns * 2 * Math.PI;
                let numSteps = Math.ceil(72 * numTurns);

                // Shift the helix orbit center by -helixRadius along the X-axis.
                // Since the helix angle (theta) ends at 0, the final point is px = cx + helixRadius * cos(0) = centerPt.x.
                // This ensures the helix terminates exactly at the start of the cutting path.
                let cx = centerPt.x - helixRadius;
                let cy = centerPt.y;

                for (let j = 0; j <= numSteps; j++) {
                    let u = j / numSteps;
                    let theta = ccw ? (-totalRad * (1 - u)) : (totalRad * (1 - u));
                    let px = cx + helixRadius * Math.cos(theta);
                    let py = cy + helixRadius * Math.sin(theta);
                    let pz = zStart - H * u;
                    helixPts.push(newPoint(px, py, pz));
                }
            } else {
                // Fallback straight plunge starts at the material boundary
                helixPts.push(newPoint(centerPt.x, centerPt.y, zStart));
                helixPts.push(newPoint(centerPt.x, centerPt.y, z));
            }
            if (helixPts.length > 0) {
                // Annotate the first helix point at zStart as forceSpeed: 0 so the descent (pre-plunge) from zSafe is rapid G0
                helixPts[0].annotate({ forceSpeed: 0 });
            }
            return helixPts;
        }

        let i = 0;
        let lastToolPathPt = null;
        while (i < walk.length) {
            let step = walk[i];
            let m = metaNodes.find(node => node.id === step.id);
            if (!m) {
                i++;
                continue;
            }

            let oriented = null;

            if (step.first === false) {
                if (lastToolPathPt) {
                    // 1. Lift to z + 0.1 at current tool position
                    pts.push(newPoint(lastToolPathPt.x, lastToolPathPt.y, z + 0.1).annotate({ forceSpeed: 0 }));

                    let prevStep = walk[i - 1];
                    let prevM = prevStep ? metaNodes.find(node => node.id === prevStep.id) : null;
                    let nextStep = walk[i + 1];
                    let nextM = nextStep ? metaNodes.find(node => node.id === nextStep.id) : null;

                    if (m.strategy === 'corridor') {
                        let spine = m.nodes.slice();
                        if (step.resolvedEnterEnd) {
                            spine.reverse();
                        }
                        for (let n of spine) {
                            pts.push(newPoint(n.x, n.y, z + 0.1).annotate({ forceSpeed: 0 }));
                        }
                        lastToolPathPt = newPoint(spine[spine.length - 1].x, spine[spine.length - 1].y, z + 0.1);
                    } else if (m.strategy === 'chamber' || m.strategy === 'entry') {
                        // Find connecting node from prevM
                        let node_from = null;
                        for (let node of m.nodes) {
                            for (let neighbor of node.neighbors) {
                                if (prevM && prevM.nodes.includes(neighbor)) {
                                    node_from = node;
                                    break;
                                }
                            }
                            if (node_from) break;
                        }
                        if (!node_from) {
                            let enterBool = step.resolvedEnterEnd === true;
                            node_from = !enterBool ? m.nodes[0] : m.nodes[m.nodes.length - 1];
                        }

                        // Move to connection node
                        pts.push(newPoint(node_from.x, node_from.y, z + 0.1).annotate({ forceSpeed: 0 }));

                        // Move to center peak node
                        let centerNode = m.generator.nodes[0];
                        pts.push(newPoint(centerNode.x, centerNode.y, z + 0.1).annotate({ forceSpeed: 0 }));

                        // Find connecting node to nextM
                        let node_to = null;
                        for (let node of m.nodes) {
                            for (let neighbor of node.neighbors) {
                                if (nextM && nextM.nodes.includes(neighbor)) {
                                    node_to = node;
                                    break;
                                }
                            }
                            if (node_to) break;
                        }
                        if (!node_to) {
                            let exitBool = step.resolvedExitEnd === true;
                            node_to = !exitBool ? m.nodes[0] : m.nodes[m.nodes.length - 1];
                        }

                        // Move to exit connection node
                        pts.push(newPoint(node_to.x, node_to.y, z + 0.1).annotate({ forceSpeed: 0 }));
                        lastToolPathPt = newPoint(node_to.x, node_to.y, z + 0.1);
                    }
                }
            } else {
                // First-time visit
                if (m.strategy === 'chamber') {
                    let m_capsules = [];
                    let chamberNodes = mat_graph.filter(node => node.chamberId === m.generator.id);
                    if (chamberNodes.length === 1) {
                        m_capsules.push(newPolygon().centerCircle(newPoint(chamberNodes[0].x, chamberNodes[0].y, z), chamberNodes[0].radius, 12));
                    } else {
                        for (let node of chamberNodes) {
                            for (let neighbor of node.neighbors) {
                                if (neighbor.chamberId === m.generator.id) {
                                    m_capsules.push(...makeTaperedCapsule(node, neighbor));
                                }
                            }
                        }
                    }
                    let B_m = POLY.trimTo(POLY.union(m_capsules, 0.00001, true), offset_polygons);
                    if (B_m.length > 0) {
                        let snapped = snapPolygonToWalls(B_m[0], offset_polygons, 0.15);
                        if (B_m[0].inner) {
                            snapped.inner = B_m[0].inner.map(inr => snapPolygonToWalls(inr, offset_polygons, 0.15));
                        }
                        let spiral = generateChamberSpiral(snapped, m.generator, targetStepover, ccw, z, toolRadius, options);

                        let ptsArray = spiral.points;

                        // Chambers must always start at the center and cut outwards. They can never be reversed
                        // or transition directly from a previous cut; they require a dedicated helical entry plunge.
                        if (step.helicalEntryNeeded && ptsArray.length > 0) {
                            let entryPts = makeHelicalEntry(ptsArray[0], m.generator.nodes[0]);
                            let transition = [];
                            if (i > 0 && lastToolPathPt) {
                                let firstHelixPt = entryPts[0];
                                transition.push(newPoint(lastToolPathPt.x, lastToolPathPt.y, zSafe).annotate({ forceSpeed: 0 }));
                                transition.push(newPoint(firstHelixPt.x, firstHelixPt.y, zSafe).annotate({ forceSpeed: 0 }));
                                transition.push(newPoint(firstHelixPt.x, firstHelixPt.y, firstHelixPt.z).annotate({ forceSpeed: 0 }));
                            }
                            ptsArray = [ ...transition, ...entryPts, ...ptsArray ];
                        }

                        oriented = pushPoints(ptsArray, true);
                    }
                    metaSpines.set(m.id, m.nodes);
                } else if (m.strategy === 'entry') {
                    let centerPt = m.nodes[0];
                    let entryPts = [];
                    if (step.helicalEntryNeeded) {
                        entryPts = makeHelicalEntry(centerPt, m.generator.nodes[0]);
                        let transition = [];
                        if (i > 0 && lastToolPathPt) {
                            let firstHelixPt = entryPts[0];
                            transition.push(newPoint(lastToolPathPt.x, lastToolPathPt.y, zSafe).annotate({ forceSpeed: 0 }));
                            transition.push(newPoint(firstHelixPt.x, firstHelixPt.y, zSafe).annotate({ forceSpeed: 0 }));
                            transition.push(newPoint(firstHelixPt.x, firstHelixPt.y, firstHelixPt.z).annotate({ forceSpeed: 0 }));
                        }
                        entryPts = [ ...transition, ...entryPts ];
                    } else {
                        entryPts = [ newPoint(centerPt.x, centerPt.y, z) ];
                    }

                    oriented = pushPoints(entryPts, true);
                    metaSpines.set(m.id, m.nodes);
                } else if (m.strategy === 'corridor') {
                    // Start a new point list C
                    let C = [];

                    /**
                     * Helper to get oriented nodes with radius.
                     * This duplicates the ordered nodes sequence, reverses it if the DFS traversal
                     * indicates we entered from the 'end' of the segment, and maps each MATNode
                     * to a library Point object with the radius property copied directly.
                     */
                    function getOrientedSpine(stepNode, stepWalk) {
                        let orientedSpine = stepNode.nodes.slice();
                        if (stepWalk.resolvedEnterEnd) {
                            orientedSpine.reverse();
                        }
                        return orientedSpine.map(n => {
                            let p = newPoint(n.x, n.y, z);
                            p.radius = n.radius;
                            return p;
                        });
                    }

                    // Add points in the corridor to C
                    C.push(...getOrientedSpine(m, step));

                    /**
                     * PREPENDING GAP-BRIDGING LOGIC:
                     * To keep the toolpath continuous at feed rate (no Z-hops or rapids between adjacent segments),
                     * we look at the previous step in the DFS walk. If a previous step exists, we retrieve its
                     * corresponding MetaNode's ordered nodes list and find the exit point used at the end of that step.
                     */
                    if (i > 0) {
                        let prevStep = walk[i - 1];
                        let prevM = metaNodes.find(node => node.id === prevStep.id);
                        // Do not prepend the exit point of a chamber to the corridor spine, as it lies far inside the chamber
                        if (prevM && prevM.nodes.length > 0 && prevM.strategy !== 'chamber') {
                            let exitIndex = prevStep.resolvedExitEnd ? prevM.nodes.length - 1 : 0;
                            let n = prevM.nodes[exitIndex];
                            let prevPt = newPoint(n.x, n.y, z);
                            prevPt.radius = n.radius;
                            C.unshift(prevPt);
                        }
                    }

                    /**
                     * APPENDING GAP-BRIDGING LOGIC:
                     * If the next step in the walk is a first-time cutting visit (first === true), we look ahead.
                     * We retrieve its MetaNode's ordered nodes list and determine where the tool will enter that node.
                     */
                    if (i + 1 < walk.length) {
                        let nextStep = walk[i + 1];
                        if (nextStep.first === true) {
                            let nextM = metaNodes.find(node => node.id === nextStep.id);
                            // Do not append the entry point of a chamber to the corridor spine, as it lies far inside the chamber
                            if (nextM && nextM.nodes.length > 0 && nextM.strategy !== 'chamber') {
                                let enterIndex = nextStep.resolvedEnterEnd ? nextM.nodes.length - 1 : 0;
                                let n = nextM.nodes[enterIndex];
                                let nextPt = newPoint(n.x, n.y, z);
                                nextPt.radius = n.radius;
                                C.push(nextPt);
                            }
                        }
                    }

                    /**
                     * RESAMPLING / INTERPOLATION:
                     * Currently, the path D is set directly to the compiled path C without resampling.
                     *
                     * (Commented-out resampling loop preserved below for future activation if needed)
                     */
                    let D = [...C];
                    /*
                    let S = targetStepover;
                    let D = [];
                    for (let j = 0; j < C.length; j++) {
                        let p0 = C[j];
                        D.push(p0);
                        if (j < C.length - 1) {
                            let p1 = C[j+1];
                            let dx = p1.x - p0.x;
                            let dy = p1.y - p0.y;
                            let d = Math.hypot(dx, dy);
                            if (d > S) {
                                let steps = Math.floor(d / S);
                                for (let k = 1; k <= steps; k++) {
                                    let dist = k * S;
                                    if (dist >= d - 0.001) break;
                                    let t = dist / d;
                                    let interpPt = newPoint(p0.x + dx * t, p0.y + dy * t, p0.z);
                                    interpPt.radius = p0.radius + (p1.radius - p0.radius) * t;
                                    D.push(interpPt);
                                }
                            }
                        }
                    }
                    */

                    /**
                     * TROCHOIDAL TOOLPATH GENERATION:
                     * We iterate through the resampled polyline D, segment by segment. For each segment, we call
                     * generateTrochoidSegment, which computes a helical/trochoidal clearing pattern of loops.
                     * All generated trochoid points are collected into allTrochPts.
                     */
                    let allTrochPts = [];
                    for (let idx = 0; idx < D.length - 1; idx++) {
                        allTrochPts.push(...generateTrochoidSegment(D[idx], D[idx+1], targetStepover, toolRadius, ccw, z));
                    }

                    console.log(`[MAT Debug] Corridor Grouping (MetaNode ${m.id}):`);
                    console.log(`  C length = ${C.length}, D length = ${D.length}, allTrochPts length = ${allTrochPts.length}`);

                    // Check if transitioning from a chamber to this corridor.
                    // If so, we perform a safe rapid transition at z + 0.1 rather than a retract/plunge.
                    let transitionPts = [];
                    if (i > 0 && lastToolPathPt && allTrochPts.length > 0) {
                        let prevStep = walk[i - 1];
                        let prevM = metaNodes.find(node => node.id === prevStep.id);
                        if (prevM && prevM.strategy === 'chamber') {
                            let fromPt = newPoint(lastToolPathPt.x, lastToolPathPt.y, z + 0.1);
                            let toPt = newPoint(allTrochPts[0].x, allTrochPts[0].y, z + 0.1);

                            // 1. Lift the tool head up 0.1mm
                            transitionPts.push(newPoint(lastToolPathPt.x, lastToolPathPt.y, z + 0.1).annotate({ forceSpeed: 0 }));

                            // 2. Check if a rapid move from current point to connection node crosses the part boundary
                            let crossed = false;
                            for (let poly of offset_polygons) {
                                let ints = poly.intersections(fromPt, toPt) ?? [];
                                if (ints.length > 0) { crossed = true; break; }
                                if (poly.inner) {
                                    for (let inner of poly.inner) {
                                        let i_ints = inner.intersections(fromPt, toPt) ?? [];
                                        if (i_ints.length > 0) { crossed = true; break; }
                                    }
                                }
                                if (crossed) break;
                            }

                            if (crossed) {
                                // If it crosses, first rapid back to the main center (peak node) of the chamber
                                let centerNode = prevM.generator.nodes[0];
                                transitionPts.push(newPoint(centerNode.x, centerNode.y, z + 0.1).annotate({ forceSpeed: 0 }));
                            }

                            // Rapid directly to the connection node at z + 0.1
                            transitionPts.push(toPt.annotate({ forceSpeed: 0 }));

                            // 3. Drop the tool head back to depth z at the connection node
                            transitionPts.push(newPoint(allTrochPts[0].x, allTrochPts[0].y, z).annotate({ forceSpeed: 0 }));

                            // Prevent standard helical entry plunge since we have safely transitioned at depth
                            step.helicalEntryNeeded = false;
                        }
                    }

                    if (transitionPts.length > 0) {
                        allTrochPts = [ ...transitionPts, ...allTrochPts ];
                    }

                    /**
                     * SAFE HELICAL ENTRY FALLBACK:
                     * If a helical entry plunge has not been performed yet, we generate it starting from this
                     * corridor's initial trochoid coordinate. We prepend it to allTrochPts to ensure the tool
                     * ramps safely down into solid stock.
                     */
                    if (step.helicalEntryNeeded && allTrochPts.length > 0) {
                        let fakeNode = { x: allTrochPts[0].x, y: allTrochPts[0].y, radius: D[0].radius };
                        let entryPts = makeHelicalEntry(allTrochPts[0], fakeNode);
                        let transition = [];
                        if (i > 0 && lastToolPathPt) {
                            let firstHelixPt = entryPts[0];
                            transition.push(newPoint(lastToolPathPt.x, lastToolPathPt.y, zSafe).annotate({ forceSpeed: 0 }));
                            transition.push(newPoint(firstHelixPt.x, firstHelixPt.y, zSafe).annotate({ forceSpeed: 0 }));
                            transition.push(newPoint(firstHelixPt.x, firstHelixPt.y, firstHelixPt.z).annotate({ forceSpeed: 0 }));
                        }
                        allTrochPts = [ ...transition, ...entryPts, ...allTrochPts ];
                    }

                    // Push the final oriented trochoidal segment points to the active cutting toolpath accumulator
                    oriented = pushPoints(allTrochPts, true);
                    metaSpines.set(m.id, m.nodes);
                    m.D = D;
                    m.C = C;
                }
            }

            if (oriented && oriented.length > 0) {
                step.start_x = oriented[0].x;
                step.start_y = oriented[0].y;
                step.end_x = oriented[oriented.length - 1].x;
                step.end_y = oriented[oriented.length - 1].y;
            }

            // Update lastToolPathPt to the end of the active toolpath
            if (pts.length > 0) {
                lastToolPathPt = pts[pts.length - 1];
            } else if (oriented && oriented.length > 0) {
                lastToolPathPt = oriented[oriented.length - 1];
            }

            i++;
        }

        if (pts.length > 0) {
            let lastPt = pts[pts.length - 1];
            // Force G0 rapid travel for the retract move to zSafe
            pts.push(newPoint(lastPt.x, lastPt.y, zSafe).annotate({ forceSpeed: 0 }));
            toolpaths.push(newPolygon().addPoints(pts).setOpen());
            pts = [];
        }
    }

    console.log(`=== [MAT Debug] Walk Step Toolpath Endpoints ===`);
    for (let walk of all_walks) {
        for (let stepIndex = 0; stepIndex < walk.length; stepIndex++) {
            let step = walk[stepIndex];
            let startStr = (step.start_x !== undefined && step.start_y !== undefined) ?
                `(${step.start_x.toFixed(4)}, ${step.start_y.toFixed(4)})` : 'undefined';
            let endStr = (step.end_x !== undefined && step.end_y !== undefined) ?
                `(${step.end_x.toFixed(4)}, ${step.end_y.toFixed(4)})` : 'undefined';
            console.log(`  Step ${stepIndex}: Node ID: ${step.id}, Strategy: ${step.strategy}, First/Cut Visit: ${step.first}, Start: ${startStr}, End: ${endStr}`);
        }
    }
    console.log(`===============================================`);

    let serializedWalks = all_walks.map(walk => walk.map(step => ({
        id: step.id,
        strategy: step.strategy,
        first: step.first,
        resolvedEnterEnd: step.resolvedEnterEnd,
        resolvedExitEnd: step.resolvedExitEnd,
        start_x: step.start_x,
        start_y: step.start_y,
        end_x: step.end_x,
        end_y: step.end_y
    })));
    let serializedToolpaths = toolpaths.map(p => p.points.map(pt => ({ x: pt.x, y: pt.y, z: pt.z })));
    let serializedMeta = metaNodes.map(m => ({
        id: m.id,
        strategy: m.strategy,
        spine: m.nodes.map(pt => ({ x: pt.x, y: pt.y })),
        connections: m.connections.map(c => ({ neighbor: c.neighbor.id, myEnd: c.myEnd, itsEnd: c.itsEnd })),
        D: m.D ? m.D.map(pt => ({ x: pt.x, y: pt.y, radius: pt.radius })) : null,
        C: m.C ? m.C.map(pt => ({ x: pt.x, y: pt.y, radius: pt.radius })) : null
    }));
    let serializedMatGraph = mat_graph.map(node => ({
        x: node.x,
        y: node.y,
        radius: node.radius,
        metaNodeId: node.metaNode ? node.metaNode.id : null,
        neighbors: Array.from(node.neighbors).map(nb => ({ x: nb.x, y: nb.y }))
    }));

    console.log(`=== [MAT JSON DUMP START] ===`);
    console.log(JSON.stringify({
        walks: serializedWalks,
        toolpaths: serializedToolpaths,
        metaNodes: serializedMeta,
        matGraph: serializedMatGraph
    }));
    console.log(`=== [MAT JSON DUMP END] ===`);

    console.log(`=== [MAT Debug] Generated Toolpaths ===`);
    console.log(`  Total Toolpaths Count: ${toolpaths.length}`);
    for (let idx = 0; idx < toolpaths.length; idx++) {
        console.log(`    Path ${idx}: points count = ${toolpaths[idx].points.length}`);
    }
    console.log(`================────────────────=======`);

    if (toolpaths.length > 0) {
        return toolpaths;
    }

    return polygons;
}
