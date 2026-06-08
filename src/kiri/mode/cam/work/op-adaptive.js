/** Copyright Stewart Allen <sa@grid.space> -- All Rights Reserved */

import { CamOp } from '../core/op.js';
import { newSlice } from '../../../core/slice.js';
import { newPoint } from '../../../../geo/point.js';
import { newPolygon } from '../../../../geo/polygon.js';
import { polygons as POLY } from '../../../../geo/polygons.js';
import { util as base_util } from '../../../../geo/base.js';

class OpAdaptive extends CamOp {
    constructor(state, op) {
        super(state, op);
    }

    async slice(progress) {
        let { op, state } = this;
        let { addSlices, color, shadowAt, shadow, stock, tool, workarea } = state;
        let { down, all, inside, omitthru } = op;

        let zTop = workarea.top_z;
        let zBottom = workarea.bottom_z;

        if (down <= 0) {
            throw `invalid step down "${down}"`;
        }

        let D = tool.fluteDiameter();
        let toolRadius = D / 2;
        let epsilon = 0.01;
        let leave_xy = op.leave ?? 0;
        let helixMin = op.helixMin ?? 0.8;
        let helixMax = op.helixMax ?? 0.9;

        let zs = base_util.lerp(zTop, zBottom, down);
        if (!zs.length) return;

        let total = zs.length;
        let index = 0;

        for (let z of zs) {
            let slice = newSlice(z);
            let layers = slice.output();

            // Get shadow (part contour) at current Z level
            let partShadow = await shadowAt(z + 0.01);
            let hasContours = partShadow && partShadow.length > 0;
            if (hasContours) {
                partShadow = partShadow.clone(true);
                if (omitthru) {
                    for (let poly of partShadow) {
                        poly.inner = undefined;
                    }
                }
            }

            // Compute outer boundary
            let outerBoundary;
            if (all) {
                // Clear stock: clear the entire stock boundary
                outerBoundary = [ newPolygon().centerRectangle(stock.center, stock.x, stock.y) ];
            } else {
                // Clear stock disabled: clear within part shadow footprint
                let baseShadow = shadow.base.clone(true);
                // If omitthru is enabled OR there are no contours at this level, ignore/remove holes
                if (omitthru || !hasContours) {
                    for (let poly of baseShadow) {
                        poly.inner = undefined;
                    }
                }
                
                if (inside) {
                    // Inside only: do not expand part shadow
                    outerBoundary = baseShadow;
                } else {
                    // Expand part shadow by tool_radius + epsilon
                    let expanded = POLY.expand(baseShadow, toolRadius + epsilon);
                    if (expanded) {
                        outerBoundary = POLY.flatten(expanded);
                    } else {
                        outerBoundary = baseShadow;
                    }
                }
            }

            // Subtract original part contour from outer boundary to get "to-be-machined" region
            let cutRegion = [];
            if (hasContours) {
                POLY.subtract(outerBoundary, partShadow, cutRegion, undefined, undefined, 0);
            } else {
                cutRegion = outerBoundary;
            }

            // Align Z coordinates of the machined region to the current level
            POLY.setZ(cutRegion, z);

            // Compute helical entry points and path circles for each closed polygon in cutRegion
            let helicalCircles = [];
            let plungePoints = [];
            let H_r_max = (D * helixMax) / 2;
            let H_r_min = (D * helixMin) / 2;
            let R_req_max = H_r_max + toolRadius + leave_xy;
            let R_req_min = H_r_min + toolRadius + leave_xy;

            for (let poly of cutRegion) {
                if (poly.open) continue;

                let res = findPoleOfInaccessibility(poly, z);
                let entryPoint = res.point;
                let bestDist = res.dist;

                if (bestDist >= R_req_max) {
                    // Fits the maximum helix
                    let circle = newPolygon().centerCircle(entryPoint, H_r_max, 20);
                    POLY.setZ([circle], z);
                    helicalCircles.push(circle);
                } else if (bestDist >= R_req_min) {
                    // Fits a smaller helix (respecting helixMin bound)
                    let fitRadius = Math.max(H_r_min, bestDist - toolRadius - leave_xy - epsilon);
                    let circle = newPolygon().centerCircle(entryPoint, fitRadius, 20);
                    POLY.setZ([circle], z);
                    helicalCircles.push(circle);
                } else if (bestDist >= toolRadius + leave_xy) {
                    // Helix doesn't fit, draw a tiny circle indicating a plunge point
                    let circle = newPolygon().centerCircle(entryPoint, toolRadius / 4, 8);
                    POLY.setZ([circle], z);
                    plungePoints.push(circle);
                }
            }

            // Draw visual layers for preview
            layers
                .setLayer(op.rename ?? "adaptive", { line: color }, false)
                .addPolys(cutRegion);

            if (helicalCircles.length > 0) {
                // Helical entry circles drawn in Red (0xff0000)
                layers
                    .setLayer("helical-entry", { line: 0xff0000 }, false)
                    .addPolys(helicalCircles);
            }

            if (plungePoints.length > 0) {
                // Plunge points drawn in Cyan (0x00ffff)
                layers
                    .setLayer("plunge-entry", { line: 0x00ffff }, false)
                    .addPolys(plungePoints);
            }

            // Store the cutRegion on the slice (required for future G-code prepare phase)
            slice.camLines = cutRegion;

            // Add slice to widget slices
            addSlices(slice);

            index++;
            progress(index / total, "slicing adaptive");
        }
    }

    prepare(ops, progress) {
        // stub: does nothing for now
    }
}

function findPoleOfInaccessibility(poly, z) {
    let bounds = poly.bounds;
    let minX = bounds.minx, maxX = bounds.maxx;
    let minY = bounds.miny, maxY = bounds.maxy;
    
    // Initial 16x16 grid search
    let bestPt = null;
    let bestDist = -Infinity;
    
    let gridX = 16;
    let gridY = 16;
    let dx = (maxX - minX) / gridX;
    let dy = (maxY - minY) / gridY;
    
    for (let i = 0; i <= gridX; i++) {
        for (let j = 0; j <= gridY; j++) {
            let x = minX + i * dx;
            let y = minY + j * dy;
            let pt = newPoint(x, y, z);
            if (pt.isInPolygon(poly)) {
                let dist = getDistToWalls(pt, poly);
                if (dist > bestDist) {
                    bestDist = dist;
                    bestPt = pt;
                }
            }
        }
    }
    
    if (!bestPt) {
        // Fallback to bounding box center if no grid point is inside
        let cx = (minX + maxX) / 2;
        let cy = (minY + maxY) / 2;
        let pt = newPoint(cx, cy, z);
        return { point: pt, dist: 0 };
    }
    
    // 3 passes of local refinement
    let rx = dx;
    let ry = dy;
    for (let pass = 0; pass < 3; pass++) {
        rx /= 2;
        ry /= 2;
        let centerPt = bestPt;
        for (let i = -2; i <= 2; i++) {
            for (let j = -2; j <= 2; j++) {
                if (i === 0 && j === 0) continue;
                let x = centerPt.x + i * rx;
                let y = centerPt.y + j * ry;
                let pt = newPoint(x, y, z);
                if (pt.isInPolygon(poly)) {
                    let dist = getDistToWalls(pt, poly);
                    if (dist > bestDist) {
                        bestDist = dist;
                        bestPt = pt;
                    }
                }
            }
        }
    }
    
    return { point: bestPt, dist: bestDist };
}

function getDistToWalls(pt, poly) {
    let dist = pt.distToPolySegments(poly);
    if (poly.inner) {
        for (let hole of poly.inner) {
            dist = Math.min(dist, pt.distToPolySegments(hole));
        }
    }
    return dist;
}

export { OpAdaptive };
