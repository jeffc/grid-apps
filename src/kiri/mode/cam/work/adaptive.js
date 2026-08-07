/** Copyright Stewart Allen <sa@grid.space> -- All Rights Reserved */

import { polygons as POLY } from '../../../../geo/polygons.js';

class MATNode {
    constructor(x, y, radius) {
        this.x = x;
        this.y = y;
        this.radius = radius;
        this.neighbors = new Set();
    }
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
 * Adaptive clearing algorithm.
 * 
 * @param {Polygon[]} polygons - Polygons to clear (raw pocket areas)
 * @param {number} toolDiam - Tool diameter
 * @param {number} stepover - Step size (over)
 * @param {Object} options - Additional options (e.g. z, direction)
 * @returns {Polygon[]} Generated toolpath polygons/lines (offset tool center workspace)
 */
export function adaptiveClear(polygons, toolDiam, stepover, options) {
    let toolRadius = toolDiam / 2;
    let threshold = 1.25 * toolDiam;
    
    // Check if we are clearing a pocket around a part profile (has holes)
    // or just clearing empty stock above the part (no holes).
    let has_profile = polygons.some(p => p.inner && p.inner.length > 0);
    let addedRadius = has_profile ? toolRadius : 0;

    // Pre-process: offset polygons inward by tool radius to collapse narrow areas
    let offset_polygons = POLY.offset(polygons, [ -toolRadius ], { z: options.z });
    
    let mat_segments = [];
    let max_nodes = [];
    let scale = 1000;

    for (let poly of offset_polygons) {
        try {
            let scaled = poly.clone(true).scale({ x: scale, y: scale, z: scale });
            let out = scaled.points.map(p => ({ x: p.x|0, y: p.y|0 }));
            let inr = (scaled.inner ?? []).map(p => p.points.map(p => ({ x: p.x|0, y: p.y|0 })));
            if (out.length > 2) {
                let ma = JSPoly.construct_medial_axis(out, inr);

                // Pre-split segments crossing the wide/narrow threshold
                let pocket_split = [];
                for (let seg of ma) {
                    let r0 = (seg.point0.radius / scale) + addedRadius;
                    let r1 = (seg.point1.radius / scale) + addedRadius;

                    let p0_wide = r0 > threshold;
                    let p1_wide = r1 > threshold;

                    if (p0_wide === p1_wide) {
                        pocket_split.push(seg);
                    } else {
                        // Split the segment at the threshold boundary point
                        let t = (threshold - r0) / (r1 - r0);
                        let px = seg.point0.x + t * (seg.point1.x - seg.point0.x);
                        let py = seg.point0.y + t * (seg.point1.y - seg.point0.y);
                        let pr = (threshold - addedRadius) * scale;

                        let split_pt = { x: px, y: py, radius: pr };
                        pocket_split.push({ point0: seg.point0, point1: split_pt });
                        pocket_split.push({ point0: split_pt, point1: seg.point1 });
                    }
                }

                mat_segments.push(...pocket_split);

                // Build a separate graph for this disjoint pocket
                let pocket_graph = buildMATGraph(pocket_split, addedRadius, scale);

                // Find the maximum node for this disjoint pocket
                let max_node = null;
                for (let node of pocket_graph) {
                    if (!max_node) {
                        max_node = node;
                        continue;
                    }
                    if (node.radius > max_node.radius) {
                        max_node = node;
                    } else if (node.radius === max_node.radius) {
                        // Deterministic tie-breaker based on coordinates
                        if (node.x > max_node.x) {
                            max_node = node;
                        } else if (node.x === max_node.x && node.y > max_node.y) {
                            max_node = node;
                        }
                    }
                }

                if (max_node) {
                    max_nodes.push(max_node);
                }
            }
        } catch (e) {
            console.warn("MAT construction failed for a pocket in adaptiveClear", e);
        }
    }

    // Classify segments in workspace units as wide or narrow
    let mat_wide_lines = [];
    let mat_narrow_lines = [];

    for (let seg of mat_segments) {
        let r0 = (seg.point0.radius / scale) + addedRadius;
        let r1 = (seg.point1.radius / scale) + addedRadius;

        let p0 = { x: seg.point0.x / scale, y: seg.point0.y / scale, radius: r0 };
        let p1 = { x: seg.point1.x / scale, y: seg.point1.y / scale, radius: r1 };

        // Note: Using Math.max() here is mathematically correct ONLY because we pre-split
        // all segments crossing the threshold boundary. This guarantees that every segment
        // lies entirely on one side of the threshold, meaning no segment straddles both.
        // Thus, if even one endpoint is strictly greater than the threshold, the entire
        // segment is classified as wide.
        let wide = Math.max(r0, r1) > threshold;
        let out_seg = { point0: p0, point1: p1 };

        if (wide) {
            mat_wide_lines.push(out_seg);
        } else {
            mat_narrow_lines.push(out_seg);
        }
    }

    // Build the combined graph in workspace units
    let mat_graph = buildMATGraph(mat_segments, addedRadius, scale);

    // Pass the computed results back via options object for visualization/processing
    options.mat_wide_lines = mat_wide_lines;
    options.mat_narrow_lines = mat_narrow_lines;
    options.mat_graph = mat_graph;
    options.max_nodes = max_nodes;

    // For now, return the original polygons to prevent the slice loop from breaking
    // when offset_polygons collapses, ensuring all layers are processed.
    return polygons;
}
