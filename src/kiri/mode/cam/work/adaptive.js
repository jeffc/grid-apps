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
        this.spine = []; // Precomputed ordered list of skeleton points (workspace coordinates)
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
 * Returns the closest intersection point along the ray (t > 0).
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
        if (Math.abs(det) < 1e-9) continue;

        let t = ((p0.x - center.x) * vy - (p0.y - center.y) * vx) / det;
        let u = (dx * (p0.y - center.y) - dy * (p0.x - center.x)) / det;

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
    let genCenter = { x: 0, y: 0 };
    for (let n of generator.nodes) {
        genCenter.x += n.x;
        genCenter.y += n.y;
    }
    genCenter.x /= generator.nodes.length;
    genCenter.y /= generator.nodes.length;

    // Calculate maximum safe helix radius
    let matRadius = generator.nodes[0].radius;
    let helixRadius = Math.max(0, Math.min(matRadius - toolRadius, 0.95 * toolRadius));
    
    while (true) {
        let next = POLY.offset([ current ], -targetStepover, { z: z });
        if (next.length === 0) {
            break;
        }
        let pPt = newPoint(genCenter.x, genCenter.y, z);
        let match = next.find(p => pPt.inPolygon(p));
        if (!match) {
            let largest = next[0];
            for (let p of next) {
                if (p.area() > Math.abs(largest.area())) largest = p;
            }
            match = largest;
        }
        current = match.clean(true, null, 0.02); // minor clean to keep offset curves smooth
        loops.push(current);
    }
    
    loops.reverse();

    // Duplicate the final loop (outer boundary) to act as a 360-degree cleanup pass
    // that starts and ends at angle 0 with perfect continuity, avoiding any gashes/jumps.
    loops.push(loops[loops.length - 1].clone());

    let path_pts = [];
    if (helixRadius > 0.05) {
        let minLoopArea = Math.PI * helixRadius * helixRadius;
        loops = loops.filter(p => p.area() > minLoopArea * 1.05);
        let helixCircle = newPolygon().centerCircle(newPoint(genCenter.x, genCenter.y, z), helixRadius, 72);
        loops.unshift(helixCircle);
    } else {
        let tinyCircle = newPolygon().centerCircle(newPoint(genCenter.x, genCenter.y, z), 0.01, 8);
        loops.unshift(tinyCircle);
    }
    
    let numSteps = 72; // 5-degree steps
    let N = loops.length - 2; // Total loops excluding the final cleanup pass loop
    
    for (let i = 0; i < loops.length - 1; i++) {
        let l0 = loops[i];
        let l1 = loops[i+1];
        
        let isCleanup = (i === loops.length - 2); // Final 360-degree cleanup loop
        
        for (let j = 0; j < numSteps; j++) {
            let u = j / numSteps;
            let theta = ccw ? (u * 2 * Math.PI) : ((1 - u) * 2 * Math.PI);
            let dx = Math.cos(theta);
            let dy = Math.sin(theta);
            
            let p0 = getRayIntersection(genCenter, dx, dy, l0);
            let p1 = getRayIntersection(genCenter, dx, dy, l1);
            
            // Local blended point between loop i and loop i+1
            let px_loc = (1 - u) * p0.x + u * p1.x;
            let py_loc = (1 - u) * p0.y + u * p1.y;
            
            // Circle projection point at current angle
            let p_circle = getRayIntersection(genCenter, dx, dy, loops[0]);
            
            // Global weight to morph from circle (loop 0) to actual offset shapes (loop N)
            // Goes from 0 (at loop 0 start) to 1 (at loop N start)
            let t = isCleanup ? 1.0 : Math.min(1.0, (i + u) / Math.max(1, N));
            
            // Apply global interpolation
            let px = (1 - t) * p_circle.x + t * px_loc;
            let py = (1 - t) * p_circle.y + t * py_loc;
            
            path_pts.push(newPoint(px, py, z));
        }
    }
    
    return newPolygon().addPoints(path_pts).setOpen();
}

/**
 * Generates D-shaped trochoidal cutting paths along a single corridor segment A -> B.
 * Resamples the segment into sub-segments of length no more than targetStepover,
 * ensuring the start (A) and end (B) points are exactly preserved.
 */
function generateTrochoidSegment(A, B, targetStepover, toolRadius, ccw, z) {
    let pts = [];
    let L = Math.hypot(B.x - A.x, B.y - A.y);
    if (L < 1e-4) return [];

    let dx = (B.x - A.x) / L;
    let dy = (B.y - A.y) / L;
    let n = { x: -dy, y: dx };

    let N = Math.ceil(L / targetStepover);
    let S = L / N;

    for (let i = 0; i < N; i++) {
        let s0 = i * S;
        let s1 = (i + 1) * S;

        let p0 = {
            x: A.x + s0 * dx,
            y: A.y + s0 * dy,
            radius: A.radius + (s0 / L) * (B.radius - A.radius)
        };
        let p1 = {
            x: A.x + s1 * dx,
            y: A.y + s1 * dy,
            radius: A.radius + (s1 / L) * (B.radius - A.radius)
        };

        let W = Math.max(0.05, p1.radius - toolRadius);

        let arcPts = [];
        let steps = 16;
        for (let k = 0; k <= steps; k++) {
            let u = k / steps;
            let theta = ccw ? (-Math.PI/2 + u * Math.PI) : (Math.PI/2 - u * Math.PI);
            
            let px = p0.x + W * n.x * Math.sin(theta) + S * dx * Math.cos(theta);
            let py = p0.y + W * n.y * Math.sin(theta) + S * dy * Math.cos(theta);
            arcPts.push(newPoint(px, py, z));
        }

        pts.push(...arcPts);

        let W_next;
        if (i < N - 1) {
            let s2 = (i + 2) * S;
            let r2 = A.radius + (Math.min(L, s2) / L) * (B.radius - A.radius);
            W_next = Math.max(0.05, r2 - toolRadius);
        } else {
            W_next = Math.max(0.05, B.radius - toolRadius);
        }

        let nextStartPt = ccw ?
            { x: p1.x - W_next * n.x, y: p1.y - W_next * n.y } :
            { x: p1.x + W_next * n.x, y: p1.y + W_next * n.y };

        let lastPt = arcPts[arcPts.length - 1];
        pts.push(newPoint(lastPt.x, lastPt.y, z + 0.1));
        pts.push(newPoint(nextStartPt.x, nextStartPt.y, z + 0.1));
        pts.push(newPoint(nextStartPt.x, nextStartPt.y, z));
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

    let teaRad = (options.tea ?? 60) * Math.PI / 180;
    let targetStepover = toolDiam * Math.sin(teaRad / 2) * Math.sin(teaRad / 2);
    
    let has_profile = polygons.some(p => p.inner && p.inner.length > 0);
    let addedRadius = has_profile ? toolRadius : 0;

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

    function getMetaSpine(m, z) {
        if (m.strategy === 'chamber' || m.strategy === 'entry') {
            return m.generator.nodes.map(n => newPoint(n.x, n.y, z));
        } else {
            let segs = [];
            for (let seg of corridor_lines) {
                let n0 = mat_graph.find(n => n.x === seg.point0.x && n.y === seg.point0.y);
                let n1 = mat_graph.find(n => n.x === seg.point1.x && n.y === seg.point1.y);
                if (n0 && n1 && n0.metaNode === m && n1.metaNode === m) {
                    segs.push({ p0: seg.point0, p1: seg.point1 });
                }
            }
            let paths = chainSegments(segs);
            if (paths.length > 0) {
                paths.sort((a, b) => b.length - a.length);
                return paths[0].map(pt => newPoint(pt.x, pt.y, z));
            }
            return m.nodes.map(n => newPoint(n.x, n.y, z));
        }
    }

    // Precompute spines for all meta-nodes immediately after construction
    for (let m of metaNodes) {
        m.spine = getMetaSpine(m, z);
    }

    // Establish Connections (Meta-Edges) in the Meta-Graph
    for (let node of mat_graph) {
        for (let neighbor of node.neighbors) {
            if (node.metaNode && neighbor.metaNode && node.metaNode !== neighbor.metaNode) {
                node.metaNode.neighbors.add(neighbor.metaNode);
                neighbor.metaNode.neighbors.add(node.metaNode);

                let mu = node.metaNode;
                let mv = neighbor.metaNode;

                let spineU = mu.spine;
                let myEnd = false; // false = begin, true = end
                if (spineU.length > 1) {
                    let d0 = dist2D(node, spineU[0]);
                    let dK = dist2D(node, spineU[spineU.length - 1]);
                    myEnd = dK < d0;
                }

                let spineV = mv.spine;
                let itsEnd = false; // false = begin, true = end
                if (spineV.length > 1) {
                    let d0 = dist2D(neighbor, spineV[0]);
                    let dK = dist2D(neighbor, spineV[spineV.length - 1]);
                    itsEnd = dK < d0;
                }

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


    // Calculate Centers for all MetaNodes
    let metaCenters = new Map();
    for (let m of metaNodes) {
        let cx = 0, cy = 0;
        if (m.strategy === 'chamber' || m.strategy === 'entry') {
            for (let n of m.generator.nodes) {
                cx += n.x;
                cy += n.y;
            }
            cx /= m.generator.nodes.length;
            cy /= m.generator.nodes.length;
        } else {
            for (let n of m.nodes) {
                cx += n.x;
                cy += n.y;
            }
            cx /= m.nodes.length;
            cy /= m.nodes.length;
        }
        metaCenters.set(m.id, { x: cx, y: cy });
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

        let largestGen = null;
        let largestSize = -1;

        for (let gen of componentGenerators) {
            let size = 0;
            if (gen.nodes.length > 1) {
                for (let i = 0; i < gen.nodes.length - 1; i++) {
                    size += dist2D(gen.nodes[i], gen.nodes[i+1]);
                }
            } else {
                size = 2 * gen.nodes[0].radius;
            }

            if (size > largestSize) {
                largestSize = size;
                largestGen = gen;
            }
        }

        if (largestGen) {
            active_generators.push(largestGen);
            
            let startMetaNode = component.find(m => (m.strategy === 'chamber' || m.strategy === 'entry') && m.generator.id === largestGen.id);
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

                    while (true) {
                        let neighbors = adjMeta.get(curr) || [];
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

                        let conn = curr.connections.find(c => c.neighbor === next);
                        if (conn) {
                            let lastCurrStep = walk.slice().reverse().find(s => s.id === curr.id);
                            if (lastCurrStep) {
                                lastCurrStep.exitEnd = conn.myEnd;
                            }

                            dfsWalk(next, conn.itsEnd);

                            walk.push({
                                id: curr.id,
                                strategy: curr.strategy,
                                first: false,
                                enterEnd: conn.myEnd,
                                exitEnd: null
                            });
                        }
                    }

                    activeStack.delete(curr.id);
                }

                dfsWalk(startMetaNode, null);
                
                // Resolve topological start and end endpoints for all walk steps
                for (let step of walk) {
                    let m = metaNodes.find(node => node.id === step.id);
                    if (m) {
                        let spine = m.spine;
                        if (spine.length > 0) {
                            let enter = step.enterEnd;
                            let exit = step.exitEnd;

                            if (enter === null && exit === null) {
                                enter = false;
                                exit = true;
                            } else if (enter === null) {
                                enter = !exit;
                            } else if (exit === null) {
                                exit = !enter;
                            }

                            step.resolvedEnterEnd = enter;
                            step.resolvedExitEnd = exit;

                            let enterPt = !enter ? spine[0] : spine[spine.length - 1];
                            let exitPt = !exit ? spine[0] : spine[spine.length - 1];

                            step.x = enterPt.x;
                            step.y = enterPt.y;
                            step.start_x = enterPt.x;
                            step.start_y = enterPt.y;
                            step.end_x = exitPt.x;
                            step.end_y = exitPt.y;
                        }
                    }
                }

                all_walks.push(walk);

                // RICH DEBUG LOGS FOR META-NODES AND DFS WALKS
                console.log(`=== [MAT Debug] Disconnected Pocket Component Walks ===`);
                for (let m of component) {
                    let spine = m.spine;
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
                
                console.log(`[MAT DFS Walk Debug] Chosen Start Generator: ${largestGen.id} (MetaNode ID: ${startMetaNode.id})`);
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
                    pts.push(newPoint(p.x, p.y, p.z));
                }
            } else {
                for (let p of newPts) {
                    pts.push(newPoint(p.x, p.y, z + 0.1));
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
                pts.push(newPoint(p.x, p.y, p.z));
            }
        } else {
            if (lastPt.z < z + 0.05) {
                pts.push(newPoint(lastPt.x, lastPt.y, z + 0.1));
                lastPt = pts[pts.length - 1];
            }
            for (let p of oriented) {
                pts.push(newPoint(p.x, p.y, z + 0.1));
            }
        }
        return oriented;
    }

    for (let walk of all_walks) {
        if (walk.length === 0) continue;
        
        let metaSpines = new Map(); // Cache spines in their first traversed direction
        let helicalEntryAdded = false; // Tracks if helical plunge has been generated for this component

        // Helper: Generate helical entry points centered at a point
        function makeHelicalEntry(centerPt, generatorNode) {
            let matRadius = generatorNode.radius;
            let helixRadius = Math.max(0, Math.min(matRadius - toolRadius, 0.95 * toolRadius));
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
                
                for (let j = 0; j <= numSteps; j++) {
                    let u = j / numSteps;
                    let theta = ccw ? (-totalRad * (1 - u)) : (totalRad * (1 - u));
                    let px = centerPt.x + helixRadius * Math.cos(theta);
                    let py = centerPt.y + helixRadius * Math.sin(theta);
                    let pz = zStart - H * u;
                    helixPts.push(newPoint(px, py, pz));
                }
            } else {
                // Fallback straight plunge starts at the material boundary
                helixPts.push(newPoint(centerPt.x, centerPt.y, zStart));
                helixPts.push(newPoint(centerPt.x, centerPt.y, z));
            }
            return helixPts;
        }

        for (let i = 0; i < walk.length; i++) {
            let step = walk[i];
            let m = metaNodes.find(node => node.id === step.id);
            if (!m) continue;

            let oriented = null;

            if (step.first === false) {
                // Do a z-hop and insert its spine points in the proper order
                let orientedSpine = m.spine.slice();
                if (step.resolvedEnterEnd) {
                    orientedSpine.reverse();
                }
                oriented = pushPoints(orientedSpine, false, true);
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
                        if (ptsArray.length > 0) {
                            let enterPt = !step.resolvedEnterEnd ? m.spine[0] : m.spine[m.spine.length - 1];
                            let dStart = Math.hypot(ptsArray[0].x - enterPt.x, ptsArray[0].y - enterPt.y);
                            let dEnd = Math.hypot(ptsArray[ptsArray.length - 1].x - enterPt.x, ptsArray[ptsArray.length - 1].y - enterPt.y);
                            if (dEnd < dStart) {
                                ptsArray.reverse();
                            }
                        }

                        if (!helicalEntryAdded && ptsArray.length > 0) {
                            let entryPts = makeHelicalEntry(ptsArray[0], m.generator.nodes[0]);
                            ptsArray = [ ...entryPts, ...ptsArray ];
                            helicalEntryAdded = true;
                        }

                        oriented = pushPoints(ptsArray, true);
                    }
                    metaSpines.set(m.id, m.spine);
                } else if (m.strategy === 'entry') {
                    let centerPt = m.spine[0];
                    let entryPts = makeHelicalEntry(centerPt, m.generator.nodes[0]);
                    helicalEntryAdded = true;

                    oriented = pushPoints(entryPts, true);
                    metaSpines.set(m.id, m.spine);
                } else if (m.strategy === 'corridor') {
                    // Start a new point list C
                    let C = [];

                    // Helper to get oriented spine points with radius
                    function getOrientedSpine(stepNode, stepWalk) {
                        let orientedSpine = stepNode.spine.slice();
                        if (stepWalk.resolvedEnterEnd) {
                            orientedSpine.reverse();
                        }
                        return orientedSpine.map(pt => {
                            let p = pt.clone();
                            let closest = stepNode.nodes[0];
                            let bestD = Infinity;
                            for (let n of stepNode.nodes) {
                                let d = Math.hypot(n.x - p.x, n.y - p.y);
                                if (d < bestD) {
                                    bestD = d;
                                    closest = n;
                                }
                            }
                            p.radius = closest.radius;
                            return p;
                        });
                    }

                    // Add points in the corridor to C
                    C.push(...getOrientedSpine(m, step));

                    // Keep iterating until we hit a "not first time" node or a non-corridor node
                    let nextIdx = i + 1;
                    while (nextIdx < walk.length) {
                        let nextStep = walk[nextIdx];
                        let nextM = metaNodes.find(node => node.id === nextStep.id);
                        if (nextM && nextM.strategy === 'corridor' && nextStep.first === true) {
                            C.push(...getOrientedSpine(nextM, nextStep));
                            i = nextIdx; // Advance outer loop index
                            nextIdx++;
                        } else {
                            break;
                        }
                    }

                    // Add the first point of the next meta-node to C
                    if (i + 1 < walk.length) {
                        let nextStep = walk[i + 1];
                        let nextM = metaNodes.find(node => node.id === nextStep.id);
                        if (nextM) {
                            let nextOriented = nextM.spine.slice();
                            if (nextStep.resolvedEnterEnd) {
                                nextOriented.reverse();
                            }
                            if (nextOriented.length > 0) {
                                let nextPt = nextOriented[0].clone();
                                let closest = (nextM.strategy === 'chamber' || nextM.strategy === 'entry') ?
                                    nextM.generator.nodes[0] : nextM.nodes[0];
                                if (nextM.strategy === 'corridor') {
                                    let bestD = Infinity;
                                    for (let n of nextM.nodes) {
                                        let d = Math.hypot(n.x - nextPt.x, n.y - nextPt.y);
                                        if (d < bestD) {
                                            bestD = d;
                                            closest = n;
                                        }
                                    }
                                }
                                nextPt.radius = closest.radius;
                                C.push(nextPt);
                            }
                        }
                    }

                    // Compute target stepover S and resample C to form D
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

                    // Use D as the input path for trochoid generation
                    let allTrochPts = [];
                    for (let idx = 0; idx < D.length - 1; idx++) {
                        allTrochPts.push(...generateTrochoidSegment(D[idx], D[idx+1], targetStepover, toolRadius, ccw, z));
                    }

                    if (!helicalEntryAdded && allTrochPts.length > 0) {
                        let fakeNode = { x: allTrochPts[0].x, y: allTrochPts[0].y, radius: m.nodes[0].radius };
                        let entryPts = makeHelicalEntry(allTrochPts[0], fakeNode);
                        allTrochPts = [ ...entryPts, ...allTrochPts ];
                        helicalEntryAdded = true;
                    }

                    oriented = pushPoints(allTrochPts, true);
                    metaSpines.set(m.id, m.spine);
                }
            }

            if (oriented && oriented.length > 0) {
                step.start_x = oriented[0].x;
                step.start_y = oriented[0].y;
                step.end_x = oriented[oriented.length - 1].x;
                step.end_y = oriented[oriented.length - 1].y;
            }
        }

        if (pts.length > 0) {
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

    if (toolpaths.length > 0) {
        return toolpaths;
    }

    return polygons;
}
