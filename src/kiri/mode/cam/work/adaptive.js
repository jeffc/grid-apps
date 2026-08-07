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
                mat_segments.push(...ma);

                // Build a separate graph for this disjoint pocket
                let pocket_graph = buildMATGraph(ma, toolRadius, scale);

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

    // Save segments in workspace units (unscaled) and add tool radius back to radii
    let mat_lines = mat_segments.map(seg => ({
        point0: {
            x: seg.point0.x / scale,
            y: seg.point0.y / scale,
            radius: (seg.point0.radius / scale) + toolRadius
        },
        point1: {
            x: seg.point1.x / scale,
            y: seg.point1.y / scale,
            radius: (seg.point1.radius / scale) + toolRadius
        }
    }));

    // Pass the computed results back via options object for visualization/processing
    options.mat_lines = mat_lines;
    options.max_nodes = max_nodes;

    // For now, return the original polygons to prevent the slice loop from breaking
    // when offset_polygons collapses, ensuring all layers are processed.
    return polygons;
}
