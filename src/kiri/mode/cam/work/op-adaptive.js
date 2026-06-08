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
        let { down, omitthru } = op;
        
        // Fetch the boundary treatment option, defaulting to "clear margin"
        let bounds = op.bounds ?? "clear margin";

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
        let rampAngle = op.rampAngle ?? 2;

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

            // Compute outer boundary based on boundary treatment settings
            let outerBoundary;
            if (bounds === "clear stock") {
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
                
                if (bounds === "inside only") {
                    // Inside only: do not expand part shadow
                    outerBoundary = baseShadow;
                } else {
                    // "clear margin" or "allow entry in stock": Expand part shadow by tool_radius + epsilon
                    let expanded = POLY.expand(baseShadow, toolRadius + epsilon);
                    if (expanded) {
                        outerBoundary = POLY.flatten(expanded);
                    } else {
                        outerBoundary = baseShadow;
                    }

                    // Trim outer boundary to stock size if it exceeds physical stock limits
                    let stockPoly = newPolygon().centerRectangle(stock.center, stock.x, stock.y);
                    let trimmed = POLY.trimTo(outerBoundary, [ stockPoly ]);
                    if (trimmed && trimmed.length > 0) {
                        outerBoundary = trimmed;
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

            // Compute helical/ramp/plunge entry points and path shapes for each closed polygon in cutRegion
            let helicalCircles = [];
            let rampContours = [];
            let plungePoints = [];
            let H_r_max = (D * helixMax) / 2;
            let H_r_min = (D * helixMin) / 2;
            let R_req_max = H_r_max + toolRadius + leave_xy;
            let R_req_min = H_r_min + toolRadius + leave_xy;
            let stockPoly = newPolygon().centerRectangle(stock.center, stock.x, stock.y);

            for (let poly of cutRegion) {
                if (poly.open) continue;

                let entryPoint = null;
                let bestDist = -Infinity;

                // For "allow entry in stock", search for a safe entry point in the stock first
                if (bounds === "allow entry in stock") {
                    // Localize search region around the pocket by expanding by 3 * tool diameter
                    let expanded = POLY.expand([poly], D * 3);
                    if (expanded) {
                        // Subtract partShadow to make sure we don't enter inside part walls
                        let allowed = [];
                        POLY.subtract(expanded, partShadow, allowed, undefined, undefined, 0);
                        
                        // Clip the search zone to the physical stock rectangle
                        let clipped = POLY.trimTo(allowed, [stockPoly]);
                        if (clipped && clipped.length > 0) {
                            // Find specific polygon containing the pocket's first point
                            let firstPt = poly.first();
                            let searchPoly = null;
                            for (let cp of clipped) {
                                if (firstPt.isInPolygon(cp)) {
                                    searchPoly = cp;
                                    break;
                                }
                            }
                            if (!searchPoly) {
                                searchPoly = clipped[0];
                            }
                            
                            // Query the stock-entry finder for a point closest to part shadow with non-clipping transition path
                            let stockRes = findStockEntryPoint(
                                poly,
                                searchPoly,
                                partShadow,
                                stockPoly,
                                toolRadius,
                                leave_xy,
                                H_r_min,
                                H_r_max,
                                z
                            );
                            if (stockRes) {
                                entryPoint = stockRes.point;
                                bestDist = stockRes.dist;
                            }
                        }
                    }
                }

                // Fallback: search strictly inside the pocket if no stock entry is found or not allowed
                if (!entryPoint) {
                    let res = findPoleOfInaccessibility(poly, z);
                    entryPoint = res.point;
                    bestDist = res.dist;
                }

                // Classify and generate visual entry indicators
                if (bestDist >= R_req_max) {
                    // Fits the maximum helix diameter
                    let circle = newPolygon().centerCircle(entryPoint, H_r_max, 20);
                    POLY.setZ([circle], z);
                    helicalCircles.push(circle);
                } else if (bestDist >= R_req_min) {
                    // Fits a smaller helix diameter down to minimum limit
                    let fitRadius = Math.max(H_r_min, bestDist - toolRadius - leave_xy - epsilon);
                    let circle = newPolygon().centerCircle(entryPoint, fitRadius, 20);
                    POLY.setZ([circle], z);
                    helicalCircles.push(circle);
                } else if (bestDist >= toolRadius + leave_xy) {
                    // Helix doesn't fit, but the tool fits: do a contour ramp along innermost offset loop
                    let stepover = toolRadius * 0.8;
                    let currentLoops = [ poly.clone(true) ];
                    let innermost = [];
                    
                    // Offset progressively inward until the loop collapses to find the center-most path
                    while (currentLoops.length > 0) {
                        let nextLoops = POLY.offset(currentLoops, -stepover, { flat: true, z });
                        if (!nextLoops || nextLoops.length === 0) {
                            innermost = currentLoops;
                            break;
                        }
                        currentLoops = nextLoops;
                    }
                    
                    if (innermost.length > 0) {
                        POLY.setZ(innermost, z);
                        rampContours.push(...innermost);
                    } else {
                        // Fallback to plunge point if innermost loop cannot be computed
                        let circle = newPolygon().centerCircle(entryPoint, toolRadius / 4, 8);
                        POLY.setZ([circle], z);
                        plungePoints.push(circle);
                    }
                } else {
                    // Pocket is too narrow: plunge entry point only
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

            if (rampContours.length > 0) {
                // Ramp entry contours drawn in Yellow (0xffff00)
                layers
                    .setLayer("ramp-entry", { line: 0xffff00 }, false)
                    .addPolys(rampContours);
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

/**
 * Finds the Pole of Inaccessibility inside a polygon (the point furthest from all walls/boundaries).
 * Uses a coarse 16x16 grid search followed by 3 passes of local refinement.
 */
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

/**
 * Calculates the distance from a point to the walls (including inner holes) of a polygon.
 */
function getDistToWalls(pt, poly) {
    let dist = pt.distToPolySegments(poly);
    if (poly.inner) {
        for (let hole of poly.inner) {
            dist = Math.min(dist, pt.distToPolySegments(hole));
        }
    }
    return dist;
}

/**
 * Find the closest point on the boundary of a polygon (including holes) to a given point.
 * Used to construct the transition path from the stock entry point to the cut pocket.
 */
function getClosestPointOnPoly(pt, poly) {
    let closestPt = null;
    let minDist = Infinity;
    
    // Check all segments of a list of points (outer ring or hole)
    function checkPoints(points) {
        for (let i = 0; i < points.length; i++) {
            let p1 = points[i];
            let p2 = points[(i + 1) % points.length];
            
            let closest = closestPointOnSegment(pt, p1, p2);
            let dist = pt.distTo2D(closest);
            if (dist < minDist) {
                minDist = dist;
                closestPt = closest;
            }
        }
    }
    
    // Check the outer boundary
    checkPoints(poly.points);
    // Check any holes/inner loops
    if (poly.inner) {
        for (let hole of poly.inner) {
            checkPoints(hole.points);
        }
    }
    
    return closestPt;
}

/**
 * Compute the closest point on a 2D line segment AB to a point P.
 */
function closestPointOnSegment(p, a, b) {
    let ab_x = b.x - a.x;
    let ab_y = b.y - a.y;
    let ap_x = p.x - a.x;
    let ap_y = p.y - a.y;
    let abLenSq = ab_x * ab_x + ab_y * ab_y;
    if (abLenSq < 1e-9) return a.clone();
    
    let t = (ap_x * ab_x + ap_y * ab_y) / abLenSq;
    t = Math.max(0, Math.min(1, t));
    return newPoint(a.x + t * ab_x, a.y + t * ab_y, p.z);
}

/**
 * Check if the straight line segment between p1 and p2 intersects/clips any segment of the part geometry.
 */
function pathIntersectsPart(p1, p2, partShadow) {
    // Determine if three points are in counter-clockwise order
    function ccw(a, b, c) {
        return (c.y - a.y) * (b.x - a.x) > (b.y - a.y) * (c.x - a.x);
    }
    // Check if line segment AB intersects line segment CD
    function intersects(a, b, c, d) {
        return ccw(a, c, d) !== ccw(b, c, d) && ccw(a, b, c) !== ccw(a, b, d);
    }
    
    if (!partShadow) return false;
    // Inspect each polygon of the part shadow
    for (let poly of partShadow) {
        let pts = poly.points;
        for (let i = 0; i < pts.length; i++) {
            let next = pts[(i + 1) % pts.length];
            if (intersects(p1, p2, pts[i], next)) {
                return true; // Intersection with outer part boundary
            }
        }
        // Inspect each hole of the part shadow
        if (poly.inner) {
            for (let hole of poly.inner) {
                let hpts = hole.points;
                for (let i = 0; i < hpts.length; i++) {
                    let next = hpts[(i + 1) % hpts.length];
                    if (intersects(p1, p2, hpts[i], next)) {
                        return true; // Intersection with internal hole boundary
                    }
                }
            }
        }
    }
    return false;
}

/**
 * Calculate the minimum distance from a point to any segment of the part shadow (including internal holes).
 */
function getDistToPartShadow(pt, partShadow) {
    let dist = Infinity;
    if (!partShadow) return dist;
    for (let poly of partShadow) {
        dist = Math.min(dist, pt.distToPolySegments(poly));
        if (poly.inner) {
            for (let hole of poly.inner) {
                dist = Math.min(dist, pt.distToPolySegments(hole));
            }
        }
    }
    return dist;
}

/**
 * Perform a grid search followed by local refinement to find a safe entry point in the stock area.
 * Prioritizes points that are as close to the part boundary as possible while satisfying clearance/no-clip requirements.
 */
function findStockEntryPoint(poly, searchPoly, partShadow, stockPoly, toolRadius, leave_xy, H_r_min, H_r_max, z) {
    let R_req_min = H_r_min + toolRadius + leave_xy;
    
    let bounds = searchPoly.bounds;
    let minX = bounds.minx, maxX = bounds.maxx;
    let minY = bounds.miny, maxY = bounds.maxy;
    
    let bestPt = null;
    let bestDist = -Infinity;
    let bestPartDist = Infinity; // We want to minimize distance to part
    
    let gridX = 16;
    let gridY = 16;
    let dx = (maxX - minX) / gridX;
    let dy = (maxY - minY) / gridY;
    
    // Evaluate if a candidate point is safe and score its distance to the part boundary
    function evaluateCandidate(pt) {
        if (!pt.isInPolygon(searchPoly)) return;
        
        let distToPart = getDistToPartShadow(pt, partShadow);
        let distToStock = pt.distToPolySegments(stockPoly);
        
        // Safety check: must fit at least the minimum helix size safely away from part and stock edges
        if (distToPart < R_req_min || distToStock < R_req_min) return;
        
        // Safety check: path from candidate entry point to pocket must not intersect any part boundary
        let Q = getClosestPointOnPoly(pt, poly);
        if (Q && pathIntersectsPart(pt, Q, partShadow)) return;
        
        // Update best point if this valid candidate is closer to the part boundary
        if (distToPart < bestPartDist) {
            bestPartDist = distToPart;
            bestPt = pt;
            // bestDist is the clearance distance for helix sizing
            bestDist = Math.min(distToPart, distToStock);
        }
    }
    
    // Pass 1: Grid Search
    for (let i = 0; i <= gridX; i++) {
        for (let j = 0; j <= gridY; j++) {
            let x = minX + i * dx;
            let y = minY + j * dy;
            let pt = newPoint(x, y, z);
            evaluateCandidate(pt);
        }
    }
    
    // Pass 2: Local Refinement (3 rounds of local subdivision)
    if (bestPt) {
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
                    evaluateCandidate(pt);
                }
            }
        }
        return { point: bestPt, dist: bestDist };
    }
    
    return null;
}

export { OpAdaptive };
