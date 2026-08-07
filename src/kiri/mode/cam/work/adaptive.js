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
        this.strategy = strategy; // 'chamber' or 'corridor'
        this.generator = generator; // generator object or null
        this.nodes = []; // MATNodes belonging to this meta-node
        this.neighbors = new Set(); // Connected MetaNodes
    }
}

/**
 * Helper to calculate 2D distance between two nodes
 */
function dist2D(u, v) {
    return Math.hypot(u.x - v.x, u.y - v.y);
}

/**
 * Builds a Medial Axis Transform (MAT) graph from flat segments returned by JSPoly.
 * The graph nodes operate in normal workspace units (unscaled) and include toolRadius.
 * 
 * @param {Object[]} segments - Flat segments array from JSPoly
 * @param {number} toolRadius - Tool radius to add back to node radii
 * @param {number} scale - Scaling factor used for integer precision
 * @returns {MATNode[]} List of unique graph nodes in workspace units
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
 * 
 * This algorithm segments the Medial Axis Transform (MAT) graph of a pocket
 * into "Chambers" (wide regions cleared via morphing spirals) and "Corridors"
 * (narrow slots cleared via trochoidal loops).
 * 
 * TRIGONOMETRIC BASIS OF THE 0.5 GRADIENT THRESHOLD:
 * The gradient G = |dR/ds| along a MAT branch represents the rate of change
 * of the inscribed circle radius R relative to the distance s along the bisector.
 * For any corner of angle alpha:
 * 
 *             /
 *            /  ) alpha/2
 *  ---------C─────────────── (Bisector / MAT)
 *            \  ) alpha/2
 *             \
 * 
 * The radius R (distance to the walls) is:
 *   R(s) = s * sin(alpha / 2)
 * The spatial gradient G = |dR/ds| is:
 *   G = sin(alpha / 2)
 * 
 * Setting G = 0.5 gives:
 *   sin(alpha / 2) = 0.5  =>  alpha / 2 = 30°  =>  alpha = 60°
 * 
 * Therefore:
 *   - G >= 0.5 corresponds to corners with angle alpha >= 60° (e.g., 90° square
 *     corners have G ≈ 0.707). These are open corners that a tool can safely
 *     expand into using a morphing spiral.
 *   - G < 0.5 corresponds to corners with angle alpha < 60° (acute angles,
 *     tapers, or slots where G = 0). These narrow slots pinch the cutter,
 *     requiring trochoidal slotting to prevent excessive tool engagement.
 * 
 * ─────────────────────────────────────────────────────────────────────────────
 */
export function adaptiveClear(polygons, toolDiam, stepover, options) {
    let toolRadius = toolDiam / 2;
    let threshold = 1.25 * toolDiam;
    let gradientLimit = 0.5; // Corresponds exactly to a 60-degree corner angle
    
    // Check if we are clearing a pocket around a part profile (has holes)
    // or just clearing empty stock above the part (no holes).
    let has_profile = polygons.some(p => p.inner && p.inner.length > 0);
    let addedRadius = has_profile ? toolRadius : 0;

    // Pre-process: offset polygons inward by tool radius to collapse narrow areas
    let offset_polygons = POLY.offset(polygons, [ -toolRadius ], { z: options.z });
    
    let mat_segments = [];
    let scale = 1000;

    // 1. Gather all raw MAT segments
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

    // 2. Build the un-split topological graph in workspace units
    let mat_graph = buildMATGraph(mat_segments, addedRadius, scale);

    // Group mat_graph into connected components of nodes (each represents a disjoint pocket)
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

    // 3. Scan each disjoint pocket component to identify generators
    let generators = [];
    let generatorIdCounter = 0;

    for (let pocket_component of pocket_components) {
        // A. Find local maximum nodes inside this component
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

        // B. Group adjacent max nodes into Chamber Generators (Points or Lines)
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
                nodes: genNodes
            });
        }

        // Fallback: If this specific disjoint pocket is a narrow slot/corridor (no nodes above threshold),
        // select the node with the absolute maximum radius within this component as a fallback Point Generator.
        if (pocket_generators.length === 0 && pocket_component.length > 0) {
            let absMaxNode = pocket_component[0];
            for (let node of pocket_component) {
                if (node.radius > absMaxNode.radius) {
                    absMaxNode = node;
                } else if (node.radius === absMaxNode.radius) {
                    // Deterministic tie-breaker
                    if (node.x > absMaxNode.x) {
                        absMaxNode = node;
                    } else if (node.x === absMaxNode.x && node.y > absMaxNode.y) {
                        absMaxNode = node;
                    }
                }
            }

            let genId = ++generatorIdCounter;
            absMaxNode.isGenerator = true;
            absMaxNode.chamberId = genId;

            pocket_generators.push({
                id: genId,
                nodes: [ absMaxNode ]
            });
        }

        generators.push(...pocket_generators);
    }

    // 5. Run BFS Chamber Expansion (classify nodes into chambers)
    for (let gen of generators) {
        let queue = [];
        // Seed queue with all nodes in this generator
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

            // Calculate spatial gradient along this MAT segment
            let ds = dist2D(fromNode, node);
            let dR = Math.abs(fromNode.radius - node.radius);
            let G = ds > 1e-6 ? dR / ds : 0;

            // Stop expansion if we hit a narrow slot (low gradient + narrow radius)
            if (G < gradientLimit && node.radius < threshold) {
                continue;
            }

            // Otherwise, group this node into the chamber and queue its neighbors
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

    // Track unique edges to prevent double rendering
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

            // An edge is chamber if both connected nodes belong to the same chamber
            if (node.chamberId !== null && node.chamberId === neighbor.chamberId) {
                chamber_lines.push(segment);
            } else {
                corridor_lines.push(segment);
            }
        }
    }

    // 7. Generate area polygons (tapered capsules) for chambers and corridors
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

    // Subdivides a long MAT segment into smaller sub-segments of max length (e.g. 1.0mm)
    // to prevent trapezoidal gaps when offsetting at 45 degrees to the axes.
    function makeTaperedCapsule(p0, p1) {
        let dx = p1.x - p0.x;
        let dy = p1.y - p0.y;
        let len = Math.hypot(dx, dy);
        
        let sub_capsules = [];
        let max_len = 1.0; // 1mm maximum length for high-precision envelope tracking
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

    // Union all capsules and intersect (clip) them against the original pocket boundary
    // to prevent visual overflow/spillage into part walls or outside the stock boundaries.
    let raw_chamber_area = chamber_capsules.length ? POLY.union(chamber_capsules, 0.00001, true) : [];
    let raw_corridor_area = corridor_capsules.length ? POLY.union(corridor_capsules, 0.00001, true) : [];

    options.chamber_areas = raw_chamber_area.length ? POLY.trimTo(raw_chamber_area, polygons) : [];
    options.corridor_areas = raw_corridor_area.length ? POLY.trimTo(raw_corridor_area, polygons) : [];

    // 8. Build the Meta-Graph of chambers and corridors
    let metaNodes = [];
    let metaNodeIdCounter = 0;

    // A. Create Chamber MetaNodes
    for (let gen of generators) {
        let meta = new MetaNode(++metaNodeIdCounter, 'chamber', gen);
        // Find all MATNodes in this chamber
        for (let node of mat_graph) {
            if (node.chamberId === gen.id) {
                node.metaNode = meta;
                meta.nodes.push(node);
            }
        }
        metaNodes.push(meta);
    }

    // B. Create Corridor MetaNodes (identify connected components of corridor nodes)
    let visitedCorridor = new Set();
    for (let startNode of mat_graph) {
        if (startNode.chamberId !== null || visitedCorridor.has(startNode)) {
            continue;
        }

        let meta = new MetaNode(++metaNodeIdCounter, 'corridor', null);
        let queue = [ startNode ];
        visitedCorridor.add(startNode);

        while (queue.length > 0) {
            let node = queue.shift();
            node.metaNode = meta;
            meta.nodes.push(node);

            for (let neighbor of node.neighbors) {
                if (neighbor.chamberId === null && !visitedCorridor.has(neighbor)) {
                    visitedCorridor.add(neighbor);
                    queue.push(neighbor);
                }
            }
        }
        metaNodes.push(meta);
    }

    // C. Establish Edges in the Meta-Graph
    for (let node of mat_graph) {
        for (let neighbor of node.neighbors) {
            if (node.metaNode && neighbor.metaNode && node.metaNode !== neighbor.metaNode) {
                node.metaNode.neighbors.add(neighbor.metaNode);
                neighbor.metaNode.neighbors.add(node.metaNode);
            }
        }
    }

    // D. Calculate Centers for all MetaNodes
    let metaCenters = new Map();
    for (let m of metaNodes) {
        let cx = 0, cy = 0;
        if (m.strategy === 'chamber') {
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
        // Trace a single connected component of the meta-graph
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

        // Find all standard generators within this component
        let componentGenerators = [];
        for (let m of component) {
            if (m.strategy === 'chamber' && m.generator) {
                componentGenerators.push(m.generator);
            }
        }

        // Pick the largest generator within this connected component
        let largestGen = null;
        let largestSize = -1;

        for (let gen of componentGenerators) {
            let size = 0;
            if (gen.nodes.length > 1) {
                // Line Generator path length
                for (let i = 0; i < gen.nodes.length - 1; i++) {
                    size += dist2D(gen.nodes[i], gen.nodes[i+1]);
                }
            } else {
                // Point Generator diameter
                size = 2 * gen.nodes[0].radius;
            }

            if (size > largestSize) {
                largestSize = size;
                largestGen = gen;
            }
        }

        if (largestGen) {
            active_generators.push(largestGen);
            
            // Run DFS on this component starting from the largest generator's MetaNode
            let startMetaNode = component.find(m => m.strategy === 'chamber' && m.generator.id === largestGen.id);
            if (startMetaNode) {
                let walk = [];
                let visitedDFS = new Set();

                function dfs(curr) {
                    visitedDFS.add(curr);
                    let center = metaCenters.get(curr.id);
                    walk.push({
                        id: curr.id,
                        strategy: curr.strategy,
                        first: true,
                        x: center.x,
                        y: center.y
                    });

                    for (let neighbor of curr.neighbors) {
                        if (!visitedDFS.has(neighbor)) {
                            dfs(neighbor);
                            // Add the backtrack move
                            walk.push({
                                id: curr.id,
                                strategy: curr.strategy,
                                first: false,
                                x: center.x,
                                y: center.y
                            });
                        }
                    }
                }

                dfs(startMetaNode);
                all_walks.push(walk);
            }
        }
    }

    options.chamber_lines = chamber_lines;
    options.corridor_lines = corridor_lines;
    options.mat_lines = mat_lines;

    // Package meta-graph for clean options serialization
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
    
    // Only package active start generators (one per disjoint pocket) to be rendered
    options.generators = active_generators.map(g => {
        return {
            type: g.nodes.length > 1 ? 'line' : 'point',
            nodes: g.nodes.map(n => ({ x: n.x, y: n.y, radius: n.radius }))
        };
    });

    // Return the original polygons to prevent the slice loop from breaking
    return polygons;
}
