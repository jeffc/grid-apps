/** Copyright Stewart Allen <sa@grid.space> -- All Rights Reserved */

import { CamOp } from '../core/op.js';
import { newSlice } from '../../../core/slice.js';
import { newPoint } from '../../../../geo/point.js';
import { newPolygon } from '../../../../geo/polygon.js';
import { polygons as POLY } from '../../../../geo/polygons.js';
import { util as base_util } from '../../../../geo/base.js';
import { paths } from '../../../../geo/paths.js';

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
                    // "clear margin" or "allow entry in stock": Expand part shadow by 1.5x the tool diameter
                    // to provide enough physical width for multiple concentric passes and trochoidal peeling.
                    let expanded = POLY.expand(baseShadow, D * 1.5);
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
            let allSegments = [];
            let H_r_max = (D * helixMax) / 2;
            let H_r_min = (D * helixMin) / 2;
            let R_req_max = H_r_max + toolRadius + leave_xy;
            let R_req_min = H_r_min + toolRadius + leave_xy;
            let stockPoly = newPolygon().centerRectangle(stock.center, stock.x, stock.y);
            
            // Calculate stepover distance
            let toolOver = D * (op.step ?? 0.4);

            for (let poly of cutRegion) {
                if (poly.open) continue;

                // --- ENTRY POINT SELECTION & SIZING ---
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

                // --- PATH GENERATION ---
                // Generate nested offset loops for this pocket
                let nestedLevels = [];
                POLY.offset([ poly.clone(true) ], -toolOver, {
                    count: 999,
                    outs: nestedLevels,
                    flat: false,
                    z
                });
                
                // Build the levels list starting with the original pocket boundary
                let levels = [ [ poly.clone(true) ] ];
                levels.push(...nestedLevels);
                
                // Generate linked spiral and peeling paths
                let pocketSegments = generateLinkedToolpath(
                    levels, toolRadius, toolOver, z,
                    helicalCircles, rampContours, plungePoints, bounds, stock
                );
                allSegments.push(...pocketSegments);
            }

            // Draw visual layers for preview
            // If there are no segments, output a single empty slice with the outline
            if (allSegments.length === 0) {
                let slice = newSlice(z);
                let layers = slice.output();
                
                // Pocket boundary drawn as a faint/dark outline
                layers
                    .setLayer(op.rename ?? "adaptive", { line: 0x555555 }, false)
                    .addPolys(cutRegion);
                    
                addSlices(slice);
            } else {
                let isFirst = true;
                for (let seg of allSegments) {
                    if (seg.polys.length === 0) continue;
                    
                    let slice = newSlice(z);
                    let layers = slice.output();
                    
                    // Pocket boundary drawn as a faint/dark outline on every slice segment
                    layers
                        .setLayer(op.rename ?? "adaptive", { line: 0x555555 }, false)
                        .addPolys(cutRegion);

                    // Add entry indicators only to the first slice segment of this Z height
                    if (isFirst) {
                        if (helicalCircles.length > 0) {
                            layers
                                .setLayer("helical-entry", { line: 0xff0000 }, false)
                                .addPolys(helicalCircles);
                        }
                        if (rampContours.length > 0) {
                            layers
                                .setLayer("ramp-entry", { line: 0xffff00 }, false)
                                .addPolys(rampContours);
                        }
                        if (plungePoints.length > 0) {
                            layers
                                .setLayer("plunge-entry", { line: 0x00ffff }, false)
                                .addPolys(plungePoints);
                        }
                        isFirst = false;
                    }

                    // Add the actual path of this segment to the appropriate layer
                    if (seg.type === 'peel') {
                        // Trochoidal peeling loops drawn in Orange (0xffa500)
                        layers
                            .setLayer("adaptive-peel", { line: 0xffa500 }, false)
                            .addPolys(seg.polys);
                    } else {
                        // Spiral morphing toolpath drawn in Green (0x00ff00)
                        layers
                            .setLayer("adaptive-spiral", { line: 0x00ff00 }, false)
                            .addPolys(seg.polys);
                    }

                    // Store the actual clearing segment lines as camLines on this slice
                    slice.camLines = seg.polys;

                    // Add slice to widget slices
                    addSlices(slice);
                }
            }

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

/**
 * Recursively flattens nested polygons and their inner holes into a flat array
 * of simple polygons without holes. This allows the pairing logic to treat
 * holes (islands) as simple loops that can be morphed/peeled outward to their
 * corresponding parent holes on the next level.
 */
function flattenToSimpleLoops(polys) {
    let flat = [];
    for (let poly of polys) {
        let cloned = poly.clone(true);
        // Clear inner so it's treated as a single simple ring when querying closest points
        cloned.inner = undefined;
        flat.push(cloned);
        if (poly.inner) {
            flat.push(...flattenToSimpleLoops(poly.inner));
        }
    }
    return flat;
}

/**
 * Generates linked spiral morphing and trochoidal peeling segments between concentric offset loops.
 * Walks from the innermost loops outward.
 */
function generateLinkedToolpath(levels, toolRadius, toolOver, z, helicalCircles, rampContours, plungePoints, bounds, stock) {
    let N = levels.length;
    if (N === 0) return [];
    
    // Flatten each concentric level to handle holes/islands as individual simple loops
    let flatLevels = levels.map(level => flattenToSimpleLoops(level));

    // Initialize cleared region (empty air or entry hole locations)
    let cleared = [];
    for (let poly of helicalCircles) {
        cleared.push(poly.clone(true));
    }
    for (let poly of rampContours) {
        let exp = POLY.expand([poly], toolRadius);
        if (exp) cleared.push(...POLY.flatten(exp));
    }
    for (let poly of plungePoints) {
        cleared.push(poly.clone(true));
    }

    // Helper to add cut path to cleared region and periodically simplify geometry
    function updateCleared(seg) {
        let clearedPolys = getSegmentClearedArea(seg, toolRadius, z);
        if (clearedPolys.length > 0) {
            cleared.push(...POLY.flatten(clearedPolys));
        }
        if (cleared.length > 5) {
            cleared = POLY.union(cleared, 0.01, true);
        }
    }

    // --- PASS 1: SECTIONING & CLASSIFICATION ---
    let classifiedSegments = [];

    // Process concentric offset levels inside-out
    for (let d = N - 1; d >= 0; d--) {
        let currentPolys = flatLevels[d];
        
        if (d === N - 1) {
            for (let poly of currentPolys) {
                let loopSegs = classifyInnermostSegments(poly, toolRadius, toolOver, z, cleared);
                for (let seg of loopSegs) {
                    classifiedSegments.push(seg);
                    updateCleared(seg);
                }
            }
        } else {
            let innerPolys = flatLevels[d + 1];
            
            for (let innerPoly of innerPolys) {
                let outerPoly = null;
                let firstPt = innerPoly.first();
                let minDistance = Infinity;
                
                for (let op of currentPolys) {
                    let cp = getClosestPointOnPoly(firstPt, op);
                    if (cp) {
                        let dist = firstPt.distTo2D(cp);
                        if (dist < minDistance) {
                            minDistance = dist;
                            outerPoly = op;
                        }
                    }
                }
                
                if (!outerPoly && currentPolys.length > 0) {
                    outerPoly = currentPolys[0]; // fallback
                }
                
                if (outerPoly) {
                    let loopSegs = classifyBetweenLoopsSegments(innerPoly, outerPoly, toolRadius, toolOver, z, cleared);
                    for (let seg of loopSegs) {
                        classifiedSegments.push(seg);
                        updateCleared(seg);
                    }
                }
            }
        }
    }

    // --- PASS 2: GEOMETRY GENERATION ---
    let resultSegments = [];
    for (let seg of classifiedSegments) {
        let segmentPolys = [];
        if (seg.isInnermost) {
            if (seg.type === "spiral") {
                let segPts = [...seg.pts];
                if (seg.isClosed) {
                    segPts.push(segPts[0]);
                }
                if (segPts.length > 1) {
                    let p = newPolygon().setOpen().addPoints(segPts);
                    POLY.setZ([p], z);
                    segmentPolys.push(p);
                }
            } else {
                // Trochoidal slotting
                let segPts = [...seg.pts];
                if (seg.isClosed) {
                    segPts.push(segPts[0]);
                }
                let stepSize = toolOver * 0.35;
                let resampled = resamplePoints(segPts, stepSize);
                let R_path = toolRadius * 0.5;

                for (let rp of resampled) {
                    let circle = newPolygon();
                    let steps = 16;
                    for (let k = 0; k <= steps; k++) {
                        let angle = (k / steps) * Math.PI * 2;
                        circle.add(rp.x + R_path * Math.cos(angle), rp.y + R_path * Math.sin(angle), z);
                    }
                    POLY.setZ([circle], z);
                    segmentPolys.push(circle);
                }
            }
        } else {
            // Between loops
            if (seg.type === "spiral") {
                let segPts = [...seg.pts];
                let segClosest = [...seg.closest];
                let segIndices = [...seg.indices];

                if (seg.isClosed) {
                    segPts.push(segPts[0]);
                    segClosest.push(segClosest[0]);
                    segIndices.push(seg.n);
                }

                // Unwrap indices
                let unwrapped = [];
                let lastVal = segIndices[0];
                unwrapped.push(lastVal);
                let offsetVal = 0;
                for (let k = 1; k < segIndices.length; k++) {
                    let val = segIndices[k];
                    if (val < lastVal) {
                        offsetVal += seg.n;
                    }
                    unwrapped.push(val + offsetVal);
                    lastVal = val;
                }

                let spiralPts = [];
                for (let k = 0; k < segPts.length; k++) {
                    let innerPt = segPts[k];
                    let outerPt = segClosest[k];
                    let t = unwrapped[k] / seg.n;
                    spiralPts.push(lerpPoint(innerPt, outerPt, t));
                }
                if (spiralPts.length > 1) {
                    let p = newPolygon().setOpen().addPoints(spiralPts);
                    POLY.setZ([p], z);
                    segmentPolys.push(p);
                }
            } else {
                // Trochoidal peeling
                let segPts = [...seg.pts];
                if (seg.isClosed) {
                    segPts.push(segPts[0]);
                }

                let stepSize = toolOver * 0.35;
                let resampled = resamplePoints(segPts, stepSize);
                for (let rp of resampled) {
                    let cp = getClosestPointOnPoly(rp, seg.outerPoly);
                    let d = rp.distTo2D(cp);
                    let R_path = Math.max(toolRadius / 2, d - toolRadius);
                    
                    let circle = newPolygon();
                    let steps = 16;
                    for (let k = 0; k <= steps; k++) {
                        let angle = (k / steps) * Math.PI * 2;
                        circle.add(rp.x + R_path * Math.cos(angle), rp.y + R_path * Math.sin(angle), z);
                    }
                    POLY.setZ([circle], z);
                    segmentPolys.push(circle);
                }
            }
        }

        if (segmentPolys.length > 0) {
            resultSegments.push({ type: seg.type, polys: segmentPolys });
        }
    }

    return resultSegments;
}

/**
 * Classifies segment points for the innermost guide loop.
 */
function classifyInnermostSegments(poly, toolRadius, toolOver, z, cleared) {
    let pts = poly.points;
    let n = pts.length;
    if (n < 2) return [];

    let dists = [];
    for (let i = 0; i < n; i++) {
        dists.push(getDistToCleared(pts[i], cleared));
    }

    let threshold = toolOver * 1.35;
    let segments = [];
    let currentType = dists[0] > threshold ? "peel" : "spiral";
    let currentPts = [ pts[0] ];
    let currentIndices = [ 0 ];

    for (let i = 1; i < n; i++) {
        let type = dists[i] > threshold ? "peel" : "spiral";
        if (type === currentType) {
            currentPts.push(pts[i]);
            currentIndices.push(i);
        } else {
            segments.push({ type: currentType, pts: currentPts, indices: currentIndices });
            currentType = type;
            currentPts = [ pts[i] ];
            currentIndices = [ i ];
        }
    }
    segments.push({ type: currentType, pts: currentPts, indices: currentIndices });

    if (segments.length > 1 && segments[0].type === segments[segments.length - 1].type) {
        let last = segments.pop();
        segments[0].pts = last.pts.concat(segments[0].pts);
        segments[0].indices = last.indices.concat(segments[0].indices);
    }

    let isClosedLoop = segments.length === 1 && poly.open === false;
    for (let seg of segments) {
        seg.isInnermost = true;
        seg.isClosed = isClosedLoop;
        seg.n = n;
    }
    return segments;
}

/**
 * Classifies segment points between an inner loop and an outer loop.
 */
function classifyBetweenLoopsSegments(innerPoly, outerPoly, toolRadius, toolOver, z, cleared) {
    let pts = innerPoly.points;
    let n = pts.length;
    if (n < 2) return [];

    let dists = [];
    let closestPts = [];
    for (let i = 0; i < n; i++) {
        let pt = pts[i];
        let cp = getClosestPointOnPoly(pt, outerPoly);
        dists.push(getDistToCleared(cp, cleared));
        closestPts.push(cp);
    }

    let threshold = toolOver * 1.35;
    let segments = [];
    let currentType = dists[0] > threshold ? "peel" : "spiral";
    let currentPts = [ pts[0] ];
    let currentClosest = [ closestPts[0] ];
    let currentIndices = [ 0 ];

    for (let i = 1; i < n; i++) {
        let type = dists[i] > threshold ? "peel" : "spiral";
        if (type === currentType) {
            currentPts.push(pts[i]);
            currentClosest.push(closestPts[i]);
            currentIndices.push(i);
        } else {
            segments.push({ type: currentType, pts: currentPts, closest: currentClosest, indices: currentIndices });
            currentType = type;
            currentPts = [ pts[i] ];
            currentClosest = [ closestPts[i] ];
            currentIndices = [ i ];
        }
    }
    segments.push({ type: currentType, pts: currentPts, closest: currentClosest, indices: currentIndices });

    if (segments.length > 1 && segments[0].type === segments[segments.length - 1].type) {
        let last = segments.pop();
        segments[0].pts = last.pts.concat(segments[0].pts);
        segments[0].closest = last.closest.concat(segments[0].closest);
        segments[0].indices = last.indices.concat(segments[0].indices);
    }

    let isClosedLoop = segments.length === 1 && innerPoly.open === false;
    for (let seg of segments) {
        seg.isInnermost = false;
        seg.isClosed = isClosedLoop;
        seg.n = n;
        seg.outerPoly = outerPoly;
    }
    return segments;
}

/**
 * Computes the cleared area polygon for a classified segment.
 */
function getSegmentClearedArea(seg, toolRadius, z) {
    if (seg.isInnermost) {
        let segPts = [...seg.pts];
        let radius = seg.type === "peel" ? toolRadius * 1.5 : toolRadius;
        if (seg.isClosed) {
            let poly = newPolygon().addPoints(segPts);
            let outer = POLY.expand([poly], radius);
            return outer ? POLY.flatten(outer) : [];
        } else {
            let p = newPolygon().setOpen().addPoints(segPts);
            return expandToolpath(p, radius, z);
        }
    } else {
        if (seg.type === "spiral") {
            let segPts = [...seg.pts];
            let segClosest = [...seg.closest];
            let segIndices = [...seg.indices];

            if (seg.isClosed) {
                segPts.push(segPts[0]);
                segClosest.push(segClosest[0]);
                segIndices.push(seg.n);
            }

            // Unwrap indices
            let unwrapped = [];
            let lastVal = segIndices[0];
            unwrapped.push(lastVal);
            let offsetVal = 0;
            for (let k = 1; k < segIndices.length; k++) {
                let val = segIndices[k];
                if (val < lastVal) {
                    offsetVal += seg.n;
                }
                unwrapped.push(val + offsetVal);
                lastVal = val;
            }

            let spiralPts = [];
            for (let k = 0; k < segPts.length; k++) {
                let innerPt = segPts[k];
                let outerPt = segClosest[k];
                let t = unwrapped[k] / seg.n;
                spiralPts.push(lerpPoint(innerPt, outerPt, t));
            }

            if (seg.isClosed) {
                let poly = newPolygon().addPoints(spiralPts);
                let outer = POLY.expand([poly], toolRadius);
                return outer ? POLY.flatten(outer) : [];
            } else {
                let p = newPolygon().setOpen().addPoints(spiralPts);
                return expandToolpath(p, toolRadius, z);
            }
        } else {
            // peel segment
            if (seg.isClosed) {
                let outerPoly = newPolygon().addPoints(seg.closest);
                let innerPoly = newPolygon().addPoints(seg.pts);
                outerPoly.inner = [ innerPoly ];

                let expanded = POLY.expand([outerPoly], toolRadius);
                return expanded ? POLY.flatten(expanded) : [];
            } else {
                let pts = [];
                pts.push(...seg.pts);
                for (let i = seg.closest.length - 1; i >= 0; i--) {
                    pts.push(seg.closest[i]);
                }
                let poly = newPolygon().addPoints(pts);
                let expanded = POLY.expand([poly], toolRadius);
                return expanded ? POLY.flatten(expanded) : [];
            }
        }
    }
}

/**
 * Calculates the shortest distance from a point to any polygon boundary/interior in the cleared region.
 */
function getDistToCleared(pt, cleared) {
    let minDist = Infinity;
    for (let poly of cleared) {
        if (pt.isInPolygon(poly)) {
            return 0;
        }
        let dist = pt.distToPolySegments(poly);
        if (poly.inner) {
            for (let hole of poly.inner) {
                dist = Math.min(dist, pt.distToPolySegments(hole));
            }
        }
        minDist = Math.min(minDist, dist);
    }
    return minDist;
}

/**
 * Generates a closed polygon representing the cleared area of a tool path.
 * Supports both open line paths (spirals) and closed loop paths (trochoids/boundaries).
 */
function expandToolpath(poly, toolRadius, z) {
    if (poly.open) {
        let res = paths.pointsToPath(poly.points, toolRadius, true);
        if (res && res.left && res.right && res.left.length && res.right.length) {
            let pts = [];
            pts.push(...res.left.map(p => newPoint(p.x, p.y, z)));
            for (let i = res.right.length - 1; i >= 0; i--) {
                let p = res.right[i];
                pts.push(newPoint(p.x, p.y, z));
            }
            let closedPoly = newPolygon().addPoints(pts);
            return [ closedPoly ];
        }
        return [];
    } else {
        let outer = POLY.expand([poly], toolRadius);
        let inner = POLY.expand([poly], -toolRadius);
        let ring = [];
        if (outer) {
            if (inner && inner.length > 0) {
                POLY.subtract(outer, inner, ring, undefined, undefined, 0);
            } else {
                ring = outer;
            }
        }
        return ring;
    }
}

/**
 * Resamples a sequence of vertices to have a constant step size.
 * Prevents vertex-density spikes from generating uneven trochoids.
 */
function resamplePoints(pts, stepSize) {
    if (pts.length < 2) return pts;
    let resampled = [];
    let current = pts[0];
    resampled.push(current);
    
    let i = 1;
    let distanceAccum = 0;
    while (i < pts.length) {
        let next = pts[i];
        let d = current.distTo2D(next);
        if (distanceAccum + d >= stepSize) {
            let needed = stepSize - distanceAccum;
            let t = needed / d;
            let np = newPoint(
                current.x + (next.x - current.x) * t,
                current.y + (next.y - current.y) * t,
                current.z
            );
            resampled.push(np);
            current = np;
            distanceAccum = 0;
        } else {
            distanceAccum += d;
            current = next;
            i++;
        }
    }
    return resampled;
}

/**
 * Linearly interpolates between two 3D coordinates.
 */
function lerpPoint(p1, p2, t) {
    return newPoint(p1.x + (p2.x - p1.x) * t, p1.y + (p2.y - p1.y) * t, p1.z);
}

export { OpAdaptive };
