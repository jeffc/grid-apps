/** Copyright Stewart Allen <sa@grid.space> -- All Rights Reserved */

/**
 * HIGH-EFFICIENCY 3D ADAPTIVE CLEARING ALGORITHM
 * 
 * OVERVIEW:
 * This module implements a high-efficiency adaptive roughing (volumetric clearing) toolpath generator
 * designed to maintain a strict control over the Tool Engagement Angle (TEA). Controlling the TEA prevents
 * tool overload/breakage in sharp corners and slots, while maintaining optimal material removal rates.
 * The pocket is cleared from the inside out, starting with a safe entry descent (helical, ramp, or plunge),
 * proceeding through an Archimedean spiral outward to the largest inscribed circle, and then branching
 * outward depth-first into pocket corners using expanding circular arcs linked by Z-hop travels.
 * 
 * ALGORITHMIC WORKFLOW:
 * 
 * 1. Safe Entry Point Detection:
 *    - Finds the furthest point from all pocket walls (Pole of Inaccessibility / Medial Axis Center)
 *      to serve as the entry descent location.
 *    - Slices innermost pocket boundaries and generates helical descent passes using the "0.9D centerline rule"
 *      (where D is tool diameter) to ensure the center pillar is fully swept by the tool face.
 * 
 * 2. Smooth Archimedean Spiral Clearing:
 *    - From the bottom of the helical entry, the tool winds outward along a smooth CCW Archimedean spiral:
 *          r(theta) = H_r + (pitch / 2*pi) * theta
 *      where H_r is the helical radius and pitch is the tool stepover (toolOver).
 *    - A 360-degree circular closing pass at the maximum radius (R_largest) finishes the spiral, sealing the shape.
 * 
 * 3. Depth-First Branch Arc Milling:
 *    - Beyond the maximum inscribed circle, the tool clears the pocket by stepping outward in concentric circles.
 *    - The circle at each step is trimmed against the inward-offset pocket boundary ("limitPoly").
 *    - This trims the circles into safe partial arcs inside the pocket.
 *    - Starting arcs at R_0 define individual "branches" (channels/corners of the pocket).
 *    - Each branch is tracked outward depth-first: we step out by `toolOver` and find the closest matching arc
 *      in the next concentric level (within Pi/3 radians). We continue until no matching arc exists or the arc chord
 *      distance falls below the tool diameter (signifying the slot has narrowed below the slotting limit).
 * 
 * 4. Micro Z-Hop Linking & Wall Contour Milling:
 *    - Within a branch, arcs are linked sequentially.
 *    - To maintain climb-milling (CCW), after completing arc `i` to `pEnd_i`, the tool:
 *      a. Mills back along the right wall contour from `pEnd_i` to the previous arc's end `pEnd_{i-1}` (if i > 0).
 *      b. Performs a vertical Z-hop retract (lifts by 0.5mm, rapid-travels to the start of the current arc `pStart_i`,
 *         and plunges back to depth).
 *      c. Mills along the left wall contour from `pStart_i` to the start of the next arc `pStart_{i+1}` (if i < k - 1).
 *    - At the end of the branch, a Z-hop is performed to the tip, and a smaller finishing pass traces the outer tip
 *      boundary contour from `pEnd_last` to `pStart_last`.
 * 
 * 5. Colinear Wall Alignment (Segment Projection):
 *    - To ensure wall contour cuts are perfectly colinear and parallel to the pocket sides (rather than angled due
 *      to discretized circle endpoint spacing), the endpoints `pStart` and `pEnd` are projected onto the closest
 *      polygon segments of the boundary.
 *    - The tool moves from the raw arc endpoint to its projected wall coordinate, cuts exactly along the projected
 *      wall segments, and exits back to the next raw arc endpoint.
 * 
 * 6. Safe Plunging & Horizontal Feed between Branches:
 *    - After finishing a branch, the tool retracts to z + 0.5.
 *    - Before entering the next branch, the tool moves at z + 0.5 to a plunge point (P_plunge) along the maximum
 *      cleared circle (R_largest) and plunges vertically into empty air.
 *    - The tool then feeds horizontally at cutting depth from P_plunge to the first arc's start point, completely
 *      preventing plunging into uncut stock.
 */

import '../../../../ext/jspoly.js';
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
        
        this.sliceOut = [];

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
                    helicalCircles, rampContours, plungePoints, bounds, stock,
                    leave_xy, op.clearFirst ?? true
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
                this.sliceOut.push(slice);
            } else {
                for (let seg of allSegments) {
                    if (seg.polys.length === 0) continue;
                    
                    let slice = newSlice(z);
                    let layers = slice.output();
                    
                    // Pocket boundary drawn as a faint/dark outline on every slice segment
                    layers
                        .setLayer(op.rename ?? "adaptive", { line: 0x555555 }, false)
                        .addPolys(cutRegion);

                    // Add entry indicators associated with this specific segment
                    if (seg.helicalCircles && seg.helicalCircles.length > 0) {
                        layers
                            .setLayer("helical-entry", { line: 0xff0000 }, false)
                            .addPolys(seg.helicalCircles);
                        slice.helicalCircles = seg.helicalCircles.map(c => c.clone(true));
                    }
                    if (seg.rampContours && seg.rampContours.length > 0) {
                        layers
                            .setLayer("ramp-entry", { line: 0xffff00 }, false)
                            .addPolys(seg.rampContours);
                        slice.rampContours = seg.rampContours.map(c => c.clone(true));
                    }
                    if (seg.plungePoints && seg.plungePoints.length > 0) {
                        layers
                            .setLayer("plunge-entry", { line: 0x00ffff }, false)
                            .addPolys(seg.plungePoints);
                        slice.plungePoints = seg.plungePoints.map(p => p.clone(true));
                    }

                    // Add the actual path of this segment to the appropriate layer
                    if (seg.type === 'peel') {
                        // Trochoidal peeling loops drawn in Orange (0xffa500)
                        layers
                            .setLayer("adaptive-peel", { line: 0xffa500 }, false)
                            .addPolys(seg.polys);
                        slice.camLines = seg.polys;
                    } else if (seg.type === 'centerline') {
                        // Slot centerline drawn in Magenta (0xff00ff)
                        layers
                            .setLayer("adaptive-centerline", { line: 0xff00ff }, false)
                            .addPolys(seg.polys);
                        // Do not set slice.camLines so it is not emitted to G-code
                    } else {
                        // Spiral morphing toolpath drawn in Green (0x00ff00)
                        layers
                            .setLayer("adaptive-spiral", { line: 0x00ff00 }, false)
                            .addPolys(seg.polys);
                        slice.camLines = seg.polys;
                    }

                    // Add slice to widget slices
                    addSlices(slice);
                    this.sliceOut.push(slice);
                }
            }

            index++;
            progress(index / total, "slicing adaptive");
        }
    }

    prepare(ops, progress) {
        let { op, sliceOut, state } = this;
        let { polyEmit, camOut, newLayer, setTool } = ops;

        if (!sliceOut || sliceOut.length === 0) return;

        // Set the tool parameters: toolId, rate, and plunge rate
        setTool(op.tool, op.rate, op.plunge || op.rate);

        let tool = state.tool;
        let D = tool.fluteDiameter();
        let toolRadius = D / 2;

        let total = sliceOut.length;
        let index = 0;

        for (let slice of sliceOut) {
            let z = slice.z;
            let zStart = z + op.down;
            if (state.stock && typeof state.stock.z === 'number') {
                zStart = Math.min(state.stock.z, zStart);
            } else if (state.workarea && typeof state.workarea.top_z === 'number') {
                zStart = Math.min(state.workarea.top_z, zStart);
            }

            // Keep track of which entry paths were merged
            let mergedEntryIndices = {
                helical: new Set(),
                ramp: new Set(),
                plunge: new Set()
            };

            // Pre-generate all entry paths to see if they can be merged with any camLines
            let helicalPaths = [];
            if (slice.helicalCircles && slice.helicalCircles.length > 0) {
                for (let circle of slice.helicalCircles) {
                    let center = circle.bounds.center();
                    let radius = circle.points[0].distTo2D(center);
                    let helicalPath = generateHelicalPath(center, radius, zStart, z, op.rampAngle || 2, false);
                    helicalPaths.push(helicalPath);
                }
            }

            let rampPaths = [];
            if (slice.rampContours && slice.rampContours.length > 0) {
                for (let contour of slice.rampContours) {
                    let rampPath = generateRampPath(contour, zStart, z, op.rampAngle || 2);
                    rampPaths.push(rampPath);
                }
            }

            let plungePaths = [];
            if (slice.plungePoints && slice.plungePoints.length > 0) {
                for (let circle of slice.plungePoints) {
                    let pt = circle.bounds.center();
                    let plungePath = newPolygon().setOpen().addPoints([
                        newPoint(pt.x, pt.y, zStart),
                        newPoint(pt.x, pt.y, z)
                    ]);
                    plungePaths.push(plungePath);
                }
            }

            // Try to merge each entry path with the closest path in slice.camLines
            if (slice.camLines && slice.camLines.length > 0) {
                // Helical entries
                for (let i = 0; i < helicalPaths.length; i++) {
                    let path = helicalPaths[i];
                    let endPt = path.last();
                    let bestCamIdx = -1;
                    let minDist = Infinity;
                    for (let j = 0; j < slice.camLines.length; j++) {
                        let camPoly = slice.camLines[j];
                        if (camPoly.points && camPoly.points.length > 0) {
                            let startPt = camPoly.first();
                            let dist = endPt.distTo2D(startPt);
                            if (dist < minDist) {
                                minDist = dist;
                                bestCamIdx = j;
                            }
                        }
                    }
                    if (bestCamIdx !== -1 && minDist < toolRadius * 4) {
                        let camPoly = slice.camLines[bestCamIdx];
                        let mergedPts = [...path.points, ...camPoly.points];
                        let mergedPoly = newPolygon().setOpen().addPoints(mergedPts);
                        slice.camLines[bestCamIdx] = mergedPoly;
                        mergedEntryIndices.helical.add(i);
                    }
                }

                // Ramp entries
                for (let i = 0; i < rampPaths.length; i++) {
                    let path = rampPaths[i];
                    let endPt = path.last();
                    let bestCamIdx = -1;
                    let minDist = Infinity;
                    for (let j = 0; j < slice.camLines.length; j++) {
                        let camPoly = slice.camLines[j];
                        if (camPoly.points && camPoly.points.length > 0) {
                            let startPt = camPoly.first();
                            let dist = endPt.distTo2D(startPt);
                            if (dist < minDist) {
                                minDist = dist;
                                bestCamIdx = j;
                            }
                        }
                    }
                    if (bestCamIdx !== -1 && minDist < toolRadius * 4) {
                        let camPoly = slice.camLines[bestCamIdx];
                        let mergedPts = [...path.points, ...camPoly.points];
                        let mergedPoly = newPolygon().setOpen().addPoints(mergedPts);
                        slice.camLines[bestCamIdx] = mergedPoly;
                        mergedEntryIndices.ramp.add(i);
                    }
                }

                // Plunge entries
                for (let i = 0; i < plungePaths.length; i++) {
                    let path = plungePaths[i];
                    let endPt = path.last();
                    let bestCamIdx = -1;
                    let minDist = Infinity;
                    for (let j = 0; j < slice.camLines.length; j++) {
                        let camPoly = slice.camLines[j];
                        if (camPoly.points && camPoly.points.length > 0) {
                            let startPt = camPoly.first();
                            let dist = endPt.distTo2D(startPt);
                            if (dist < minDist) {
                                minDist = dist;
                                bestCamIdx = j;
                            }
                        }
                    }
                    if (bestCamIdx !== -1 && minDist < toolRadius * 4) {
                        let camPoly = slice.camLines[bestCamIdx];
                        let mergedPts = [...path.points, ...camPoly.points];
                        let mergedPoly = newPolygon().setOpen().addPoints(mergedPts);
                        slice.camLines[bestCamIdx] = mergedPoly;
                        mergedEntryIndices.plunge.add(i);
                    }
                }
            }

            // Emit non-merged entry paths first
            if (slice.helicalCircles && slice.helicalCircles.length > 0) {
                for (let i = 0; i < slice.helicalCircles.length; i++) {
                    if (!mergedEntryIndices.helical.has(i)) {
                        polyEmit(helicalPaths[i]);
                        newLayer();
                    }
                }
            }

            if (slice.rampContours && slice.rampContours.length > 0) {
                for (let i = 0; i < slice.rampContours.length; i++) {
                    if (!mergedEntryIndices.ramp.has(i)) {
                        polyEmit(rampPaths[i]);
                        newLayer();
                    }
                }
            }

            if (slice.plungePoints && slice.plungePoints.length > 0) {
                for (let i = 0; i < slice.plungePoints.length; i++) {
                    if (!mergedEntryIndices.plunge.has(i)) {
                        let circle = slice.plungePoints[i];
                        let pt = circle.bounds.center();
                        camOut(newPoint(pt.x, pt.y, zStart), 0);
                        camOut(newPoint(pt.x, pt.y, z), 1);
                        newLayer();
                    }
                }
            }

            // Emit main pocket clearing toolpath (including merged entries)
            if (slice.camLines && slice.camLines.length > 0) {
                for (let poly of slice.camLines) {
                    if (poly.open) {
                        polyEmit(poly);
                    } else {
                        // closed loops: start from the point closest to print point
                        polyEmit(poly, -999);
                    }
                }
                newLayer();
            }

            index++;
            progress(index / total, "preparing adaptive");
        }
    }
}

/**
 * Generates a CCW helical spiral descent path as a series of linear segments.
 */
function generateHelicalPath(center, radius, zStart, zEnd, rampAngle, clockwise) {
    let theta_rad = (rampAngle || 2) * Math.PI / 180;
    let C = 2 * Math.PI * radius;
    let pitch = C * Math.tan(theta_rad);
    
    let total_drop = zStart - zEnd;
    let turns = Math.ceil(total_drop / pitch);
    if (turns <= 0) turns = 1;
    let drop_per_turn = total_drop / turns;
    
    let points = [];
    let numSegs = 24; // segments per turn
    
    let startAngle = 0;
    let cwMultiplier = clockwise ? -1 : 1;
    
    let currentZ = zStart;
    let p_start = center.clone().setZ(zStart).add(newPoint(Math.cos(startAngle) * radius, Math.sin(startAngle) * radius, 0));
    points.push(p_start);
    
    for (let t = 0; t < turns; t++) {
        let nextZ = zStart - (t + 1) * drop_per_turn;
        for (let i = 1; i <= numSegs; i++) {
            let angle = startAngle + (i / numSegs) * 2 * Math.PI * cwMultiplier;
            let z = currentZ - (i / numSegs) * drop_per_turn;
            let pt = center.clone().setZ(z).add(newPoint(Math.cos(angle) * radius, Math.sin(angle) * radius, 0));
            points.push(pt);
        }
        currentZ = nextZ;
        startAngle = startAngle + 2 * Math.PI * cwMultiplier;
    }
    
    // One flat cleanup circle at the bottom depth
    for (let i = 1; i <= numSegs; i++) {
        let angle = startAngle + (i / numSegs) * 2 * Math.PI * cwMultiplier;
        let pt = center.clone().setZ(zEnd).add(newPoint(Math.cos(angle) * radius, Math.sin(angle) * radius, 0));
        points.push(pt);
    }
    
    let poly = newPolygon().setOpen().addPoints(points);
    return poly;
}

/**
 * Generates a descending contour ramp path by traversing and interpolating Z values along a polygon contour.
 */
function generateRampPath(poly, zStart, zEnd, rampAngle) {
    let pts = poly.points;
    let n = pts.length;
    if (n < 2) return poly;
    
    let L = 0;
    for (let i = 0; i < n; i++) {
        let p1 = pts[i];
        let p2 = pts[(i + 1) % n];
        L += p1.distTo2D(p2);
    }
    
    let theta_rad = (rampAngle || 2) * Math.PI / 180;
    let pitch = L * Math.tan(theta_rad);
    
    let total_drop = zStart - zEnd;
    let turns = Math.ceil(total_drop / pitch);
    if (turns <= 0) turns = 1;
    
    let points = [];
    let z_step_per_segment = (total_drop / turns) / n;
    
    for (let t = 0; t < turns; t++) {
        for (let i = 0; i < n; i++) {
            let pt = pts[i];
            let z = zStart - ((t * n) + i) * z_step_per_segment;
            points.push(newPoint(pt.x, pt.y, Math.max(zEnd, z)));
        }
    }
    
    // Cleanup pass at the bottom Z depth
    for (let i = 0; i < n; i++) {
        let pt = pts[i];
        points.push(newPoint(pt.x, pt.y, zEnd));
    }
    // Close the cleanup loop
    let pt = pts[0];
    points.push(newPoint(pt.x, pt.y, zEnd));
    
    let polyOut = newPolygon().setOpen().addPoints(points);
    return polyOut;
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

function projectToSegment(P, A, B) {
    let dx = B.x - A.x;
    let dy = B.y - A.y;
    let lenSq = dx * dx + dy * dy;
    if (lenSq === 0) {
        return { point: A, t: 0, dist: P.distTo2D(A) };
    }
    let t = ((P.x - A.x) * dx + (P.y - A.y) * dy) / lenSq;
    t = Math.max(0, Math.min(1, t));
    let proj = newPoint(A.x + t * dx, A.y + t * dy, P.z);
    return { point: proj, t: t, dist: P.distTo2D(proj) };
}

/**
 * Traces the shortest path along the segments of a polygon boundary connecting two points.
 */
function getShortestContourSegment(lp, pStart, pEnd) {
    let pts = lp.points;
    let n = pts.length;
    if (n === 0) return [pStart, pEnd];
    
    // Find closest segment for pStart
    let idxSegStart = -1;
    let minDStart = Infinity;
    let pStartProj = null;
    let tStart = 0;
    for (let i = 0; i < n; i++) {
        let A = pts[i];
        let B = pts[(i + 1) % n];
        let proj = projectToSegment(pStart, A, B);
        if (proj.dist < minDStart) {
            minDStart = proj.dist;
            idxSegStart = i;
            pStartProj = proj.point;
            tStart = proj.t;
        }
    }
    
    // Find closest segment for pEnd
    let idxSegEnd = -1;
    let minDEnd = Infinity;
    let pEndProj = null;
    let tEnd = 0;
    for (let i = 0; i < n; i++) {
        let A = pts[i];
        let B = pts[(i + 1) % n];
        let proj = projectToSegment(pEnd, A, B);
        if (proj.dist < minDEnd) {
            minDEnd = proj.dist;
            idxSegEnd = i;
            pEndProj = proj.point;
            tEnd = proj.t;
        }
    }
    
    if (idxSegStart === -1 || idxSegEnd === -1) {
        return [pStart, pEnd];
    }
    
    // Path 1: Forward path from pStartProj to pEndProj
    let pathFwd = [];
    if (idxSegStart === idxSegEnd) {
        if (tStart <= tEnd) {
            pathFwd.push(pStartProj, pEndProj);
        } else {
            pathFwd.push(pStartProj);
            pathFwd.push(pts[(idxSegStart + 1) % n]);
            let curr = (idxSegStart + 1) % n;
            while (curr !== idxSegStart) {
                let next = (curr + 1) % n;
                pathFwd.push(pts[next]);
                curr = next;
            }
            pathFwd.push(pEndProj);
        }
    } else {
        pathFwd.push(pStartProj);
        let curr = idxSegStart;
        while (curr !== idxSegEnd) {
            let next = (curr + 1) % n;
            pathFwd.push(pts[next]);
            curr = next;
        }
        pathFwd.push(pEndProj);
    }
    
    // Calculate Forward path distance
    let distFwd = 0;
    for (let i = 0; i < pathFwd.length - 1; i++) {
        distFwd += pathFwd[i].distTo2D(pathFwd[i+1]);
    }
    
    // Path 2: Backward path from pStartProj to pEndProj
    let pathBwd = [];
    if (idxSegStart === idxSegEnd) {
        if (tStart >= tEnd) {
            pathBwd.push(pStartProj, pEndProj);
        } else {
            pathBwd.push(pStartProj);
            pathBwd.push(pts[idxSegStart]);
            let curr = idxSegStart;
            while (curr !== (idxSegStart + 1) % n) {
                let prev = (curr - 1 + n) % n;
                pathBwd.push(pts[prev]);
                curr = prev;
            }
            pathBwd.push(pEndProj);
        }
    } else {
        pathBwd.push(pStartProj);
        let curr = idxSegStart;
        while (curr !== idxSegEnd) {
            pathBwd.push(pts[curr]);
            curr = (curr - 1 + n) % n;
        }
        pathBwd.push(pEndProj);
    }
    
    // Calculate Backward path distance
    let distBwd = 0;
    for (let i = 0; i < pathBwd.length - 1; i++) {
        distBwd += pathBwd[i].distTo2D(pathBwd[i+1]);
    }
    
    let bestPath = distFwd < distBwd ? pathFwd : pathBwd;
    
    // Build the final segment: pStart -> bestPath -> pEnd
    let result = [ pStart ];
    if (pStart.distTo2D(pStartProj) > 0.01) {
        result.push(pStartProj);
    }
    for (let i = 0; i < bestPath.length; i++) {
        let pt = bestPath[i];
        if (i > 0 && i < bestPath.length - 1) {
            if (pt.distTo2D(pStart) > 0.05 && pt.distTo2D(pEnd) > 0.05) {
                result.push(newPoint(pt.x, pt.y, pStart.z));
            }
        }
    }
    if (pEnd.distTo2D(pEndProj) > 0.01) {
        result.push(pEndProj);
    }
    result.push(pEnd);
    return result;
}

function buildMATGraph(pocketRoot) {
    let out = pocketRoot.points.map(p => ({ x: p.x * 1000 | 0, y: p.y * 1000 | 0 }));
    let inr = (pocketRoot.inner ?? []).map(hole => hole.points.map(p => ({ x: p.x * 1000 | 0, y: p.y * 1000 | 0 })));
    
    let rawMa = [];
    try {
        rawMa = JSPoly.construct_medial_axis(out, inr);
    } catch (e) {
        console.error("Medial axis construction failed:", e);
        return { vertices: new Map(), edges: [] };
    }
    
    let vertices = new Map();
    let edges = [];
    let key = (x, y) => `${x.toFixed(4)},${y.toFixed(4)}`;
    
    function getOrCreateVertex(p) {
        let x = p.x / 1000;
        let y = p.y / 1000;
        let radius = p.radius / 1000;
        let k = key(x, y);
        if (!vertices.has(k)) {
            vertices.set(k, {
                key: k,
                x: x,
                y: y,
                radius: radius,
                neighbors: []
            });
        }
        return vertices.get(k);
    }
    
    for (let seg of rawMa) {
        let v0 = getOrCreateVertex(seg.point0);
        let v1 = getOrCreateVertex(seg.point1);
        
        if (v0.key === v1.key) continue;
        
        if (!v0.neighbors.includes(v1)) {
            v0.neighbors.push(v1);
        }
        if (!v1.neighbors.includes(v0)) {
            v1.neighbors.push(v0);
        }
        
        edges.push({ v0, v1, radius: (v0.radius + v1.radius) / 2 });
    }
    
    return { vertices, edges };
}

function extractSlotChains(maGraph, toolRadius, toolOver, leave_xy) {
    let slotThreshold = toolRadius + toolOver + leave_xy;
    let edges = maGraph.edges;
    let slotEdges = edges.filter(e => {
        let minRadius = toolRadius + leave_xy - 0.05;
        return e.radius < slotThreshold && e.radius >= minRadius;
    });
    
    let slotAdj = new Map();
    for (let e of slotEdges) {
        if (!slotAdj.has(e.v0.key)) slotAdj.set(e.v0.key, []);
        if (!slotAdj.has(e.v1.key)) slotAdj.set(e.v1.key, []);
        slotAdj.get(e.v0.key).push(e.v1);
        slotAdj.get(e.v1.key).push(e.v0);
    }
    
    let visited = new Set();
    let slotChains = [];
    
    for (let e of slotEdges) {
        let edgeKey = e.v0.key + "->" + e.v1.key;
        let revEdgeKey = e.v1.key + "->" + e.v0.key;
        if (visited.has(edgeKey) || visited.has(revEdgeKey)) continue;
        
        let chain = [e.v0, e.v1];
        visited.add(edgeKey);
        
        let curr = e.v1;
        let prev = e.v0;
        while (true) {
            let neighbors = slotAdj.get(curr.key) || [];
            let next = null;
            for (let n of neighbors) {
                let key1 = curr.key + "->" + n.key;
                let key2 = n.key + "->" + curr.key;
                if (n.key !== prev.key && !visited.has(key1) && !visited.has(key2)) {
                    next = n;
                    break;
                }
            }
            if (!next) break;
            chain.push(next);
            visited.add(curr.key + "->" + next.key);
            prev = curr;
            curr = next;
        }
        
        curr = e.v0;
        prev = e.v1;
        while (true) {
            let neighbors = slotAdj.get(curr.key) || [];
            let next = null;
            for (let n of neighbors) {
                let key1 = curr.key + "->" + n.key;
                let key2 = n.key + "->" + curr.key;
                if (n.key !== prev.key && !visited.has(key1) && !visited.has(key2)) {
                    next = n;
                    break;
                }
            }
            if (!next) break;
            chain.unshift(next);
            visited.add(next.key + "->" + curr.key);
            prev = curr;
            curr = next;
        }
        
        slotChains.push(chain);
    }
    
    return slotChains;
}

function resampleChain(chain, stepSize) {
    if (chain.length < 2) return [];
    let resampled = [];
    let current = chain[0];
    resampled.push({ x: current.x, y: current.y, radius: current.radius });
    
    let i = 1;
    let distanceAccum = 0;
    while (i < chain.length) {
        let next = chain[i];
        let d = Math.hypot(next.x - current.x, next.y - current.y);
        if (distanceAccum + d >= stepSize) {
            let needed = stepSize - distanceAccum;
            let t = needed / d;
            let np = {
                x: current.x + (next.x - current.x) * t,
                y: current.y + (next.y - current.y) * t,
                radius: current.radius + (next.radius - current.radius) * t
            };
            resampled.push(np);
            current = np;
            distanceAccum = 0;
        } else {
            distanceAccum += d;
            current = next;
            i++;
        }
    }
    let last = chain[chain.length - 1];
    let lastR = resampled[resampled.length - 1];
    if (Math.hypot(last.x - lastR.x, last.y - lastR.y) > 0.01) {
        resampled.push({ x: last.x, y: last.y, radius: last.radius });
    }
    return resampled;
}

function isPointCleared(rp, cleared, toolRadius) {
    let pt = newPoint(rp.x, rp.y);
    return pt.isInPolygon(cleared);
}

function generateSlotToolpaths(slotChains, toolRadius, toolOver, z, leave_xy, cleared, classifiedSegments) {
    let slotSegments = [];
    let stepSize = toolOver * 0.35;
    
    for (let chain of slotChains) {
        let resampled = resampleChain(chain, stepSize);
        let m = resampled.length;
        if (m < 2) continue;
        
        let tangents = [];
        let normals = [];
        for (let j = 0; j < m; j++) {
            let rp = resampled[j];
            let Tx, Ty;
            if (j < m - 1) {
                let next = resampled[j+1];
                let dx = next.x - rp.x;
                let dy = next.y - rp.y;
                let len = Math.hypot(dx, dy);
                Tx = dx / (len || 1);
                Ty = dy / (len || 1);
            } else if (j > 0) {
                let prev = resampled[j-1];
                let dx = rp.x - prev.x;
                let dy = rp.y - prev.y;
                let len = Math.hypot(dx, dy);
                Tx = dx / (len || 1);
                Ty = dy / (len || 1);
            } else {
                Tx = 1;
                Ty = 0;
            }
            tangents.push({ x: Tx, y: Ty });
            normals.push({ x: -Ty, y: Tx });
        }
        
        let trochoidPath = [];
        for (let j = 0; j < m; j++) {
            let rp = resampled[j];
            
            if (isPointCleared(rp, cleared, toolRadius)) {
                continue;
            }
            
            let T = tangents[j];
            let N = normals[j];
            let distToWall = rp.radius;
            let R_path = Math.max(toolRadius / 2, distToWall - toolRadius - leave_xy);
            
            let steps = 12;
            let loopPts = [];
            for (let k_step = 0; k_step <= steps; k_step++) {
                let phi = -Math.PI / 2 + (k_step / steps) * Math.PI;
                let x = rp.x + R_path * Math.cos(phi) * T.x + R_path * Math.sin(phi) * N.x;
                let y = rp.y + R_path * Math.cos(phi) * T.y + R_path * Math.sin(phi) * N.y;
                loopPts.push(newPoint(x, y, z));
            }
            
            if (trochoidPath.length === 0) {
                trochoidPath.push(...loopPts);
            } else {
                let prevEnd = trochoidPath[trochoidPath.length - 1];
                let currStart = loopPts[0];
                trochoidPath.push(newPoint(prevEnd.x, prevEnd.y, z + 0.5));
                trochoidPath.push(newPoint(currStart.x, currStart.y, z + 0.5));
                trochoidPath.push(newPoint(currStart.x, currStart.y, z));
                trochoidPath.push(...loopPts.slice(1));
            }
        }
        
        if (trochoidPath.length > 0) {
            slotSegments.push({
                type: "peel",
                pts: trochoidPath
            });
            classifiedSegments.push({
                type: "centerline",
                pts: resampled.map(p => newPoint(p.x, p.y, z)),
                isInnermost: true,
                isClosed: false,
                n: resampled.length
            });
        }
    }
    
    return slotSegments;
}

function sortSegments(segments, startPt, clearFirst) {
    let ordered = [];
    let currentPt = startPt;
    let remaining = [...segments];
    
    while (remaining.length > 0) {
        let bestIdx = -1;
        let minDist = Infinity;
        
        let hasCoresLeft = clearFirst && remaining.some(s => s.type === "spiral");
        
        for (let i = 0; i < remaining.length; i++) {
            let seg = remaining[i];
            if (hasCoresLeft && seg.type !== "spiral") continue;
            
            let firstPt = seg.pts[0];
            let dist = currentPt.distTo2D(firstPt);
            if (dist < minDist) {
                minDist = dist;
                bestIdx = i;
            }
        }
        
        if (bestIdx === -1) {
            bestIdx = 0;
        }
        
        let chosen = remaining[bestIdx];
        ordered.push(chosen);
        currentPt = chosen.pts[chosen.pts.length - 1];
        remaining.splice(bestIdx, 1);
    }
    
    return ordered;
}

function generateLinkedToolpath(levels, toolRadius, toolOver, z, helicalCircles, rampContours, plungePoints, bounds, stock, leave_xy, clearFirst) {
    let N = levels.length;
    if (N === 0) return [];
    
    let flatLevels = levels.map(level => flattenToSimpleLoops(level));

    let cleared = [];
    for (let poly of helicalCircles) {
        let exp = POLY.expand([poly], toolRadius);
        if (exp) cleared.push(...POLY.flatten(exp));
    }
    for (let poly of rampContours) {
        let exp = POLY.expand([poly], toolRadius);
        if (exp) cleared.push(...POLY.flatten(exp));
    }
    for (let poly of plungePoints) {
        let exp = POLY.expand([poly], toolRadius);
        if (exp) cleared.push(...POLY.flatten(exp));
    }

    let coreSegments = [];
    let classifiedSegments = [];

    let innermostPolys = flatLevels[N - 1];
    for (let poly of innermostPolys) {
        let entryPt = findEntryForPoly(poly, helicalCircles, rampContours, plungePoints);
        if (!entryPt) entryPt = poly.first();
        
        if (entryPt) {
            let pocketRoot = null;
            let firstPt = poly.first();
            for (let root of levels[0]) {
                if (firstPt.isInPolygon(root)) {
                    pocketRoot = root;
                    break;
                }
            }
            if (!pocketRoot && levels[0].length > 0) pocketRoot = levels[0][0];

            let pocketLoops = [];
            if (pocketRoot) {
                pocketLoops.push(pocketRoot);
                if (pocketRoot.inner) pocketLoops.push(...flattenToSimpleLoops(pocketRoot.inner));
            }

            let limitPoly = null;
            if (pocketRoot) limitPoly = POLY.expand([pocketRoot], -(toolRadius + leave_xy));

            let R_max = Infinity;
            for (let loop of pocketLoops) {
                let dist = entryPt.distToPolySegments(loop);
                if (dist < R_max) R_max = dist;
            }
            let R_largest = R_max - toolRadius - leave_xy;

            let H_r = 0.01;
            for (let c of helicalCircles) {
                let cp = c.bounds.center();
                if (cp.distTo2D(entryPt) < 0.1) {
                    H_r = c.points[0].distTo2D(cp);
                    break;
                }
            }

            let combinedPts = [];
            if (R_largest > H_r) {
                let pitch = toolOver;
                let r0 = H_r;
                let r1 = R_largest;
                let totalAngle = ((r1 - r0) / pitch) * 2 * Math.PI;
                let steps = Math.ceil(totalAngle / (Math.PI / 16));
                if (steps < 8) steps = 8;
                
                let spiralPts = [];
                for (let step = 0; step <= steps; step++) {
                    let theta = (step / steps) * totalAngle;
                    let r = r0 + (pitch / (2 * Math.PI)) * theta;
                    let x = entryPt.x + r * Math.cos(theta);
                    let y = entryPt.y + r * Math.sin(theta);
                    spiralPts.push(newPoint(x, y, z));
                }
                combinedPts.push(...spiralPts);

                let closingSteps = 64;
                let lastPt = spiralPts[spiralPts.length - 1];
                let startAngle = Math.atan2(lastPt.y - entryPt.y, lastPt.x - entryPt.x);
                let closingPts = [];
                for (let step = 1; step <= closingSteps; step++) {
                    let theta = startAngle + (step / closingSteps) * 2 * Math.PI;
                    let x = entryPt.x + r1 * Math.cos(theta);
                    let y = entryPt.y + r1 * Math.sin(theta);
                    closingPts.push(newPoint(x, y, z));
                }
                combinedPts.push(...closingPts);
            }

            let R_0 = (R_largest > H_r ? R_largest : H_r) + toolOver;
            let toolDiameter = toolRadius * 2;
            let initialArcs = [];
            let numPoints = 128;
            let circlePts = [];
            for (let step = 0; step < numPoints; step++) {
                let theta = (step / numPoints) * 2 * Math.PI;
                let x = entryPt.x + R_0 * Math.cos(theta);
                let y = entryPt.y + R_0 * Math.sin(theta);
                circlePts.push(newPoint(x, y, z));
            }
            let ptSafe = circlePts.map(pt => !isPointCleared(pt, cleared, 0));
            let inArc = false;
            let currentArc = [];
            for (let step = 0; step < numPoints; step++) {
                if (ptSafe[step]) {
                    if (!inArc) { inArc = true; currentArc = [ circlePts[step] ]; } 
                    else { currentArc.push(circlePts[step]); }
                } else if (inArc) { inArc = false; initialArcs.push(currentArc); currentArc = []; }
            }
            if (inArc && currentArc.length > 0) {
                if (initialArcs.length > 0 && ptSafe[0]) initialArcs[0] = currentArc.concat(initialArcs[0]);
                else initialArcs.push(currentArc);
            }

            let branches = [];
            for (let startArc of initialArcs) {
                if (startArc.length <= 1) continue;
                let isFullCircle = (startArc.length >= numPoints - 5);
                let chordLen = startArc[0].distTo2D(startArc[startArc.length - 1]);
                if (!isFullCircle && chordLen <= toolDiameter) continue;
                let branch = { arcs: [ startArc ], theta_center: Math.atan2(startArc[Math.floor(startArc.length / 2)].y - entryPt.y, startArc[Math.floor(startArc.length / 2)].x - entryPt.x) };
                branches.push(branch);
            }

            for (let branch of branches) {
                let k = branch.arcs.length;
                let firstArc = branch.arcs[0];
                let startPt = firstArc[0];
                let theta_start = Math.atan2(startPt.y - entryPt.y, startPt.x - entryPt.x);
                let R_safe = R_largest > H_r ? R_largest : H_r;
                let plungePt = newPoint(entryPt.x + R_safe * Math.cos(theta_start), entryPt.y + R_safe * Math.sin(theta_start), z);
                
                if (combinedPts.length > 0) {
                    let lastPt = combinedPts[combinedPts.length - 1];
                    combinedPts.push(newPoint(lastPt.x, lastPt.y, z + 0.5), newPoint(plungePt.x, plungePt.y, z + 0.5), newPoint(plungePt.x, plungePt.y, z));
                } else {
                    combinedPts.push(newPoint(plungePt.x, plungePt.y, z + 0.5), newPoint(plungePt.x, plungePt.y, z));
                }
                combinedPts.push(startPt);
                for (let i = 0; i < k; i++) {
                    let arc = branch.arcs[i];
                    combinedPts.push(...(i > 0 ? arc.slice(1) : arc));
                    if (i > 0) {
                        let bestLp = null, minDistLp = Infinity;
                        if (limitPoly) {
                            for (let lp of limitPoly) {
                                let dist = arc[arc.length-1].distToPolySegments(lp);
                                if (dist < minDistLp) { minDistLp = dist; bestLp = lp; }
                            }
                        }
                        if (bestLp) combinedPts.push(...getShortestContourSegment(bestLp, arc[arc.length-1], branch.arcs[i-1][branch.arcs[i-1].length-1]).slice(1));
                    }
                    if (i < k - 1) {
                        let lastPt = combinedPts[combinedPts.length - 1];
                        combinedPts.push(newPoint(lastPt.x, lastPt.y, z + 0.5), newPoint(arc[0].x, arc[0].y, z + 0.5), newPoint(arc[0].x, arc[0].y, z));
                        let bestLp = null, minDistLp = Infinity;
                        if (limitPoly) {
                            for (let lp of limitPoly) {
                                let dist = arc[0].distToPolySegments(lp);
                                if (dist < minDistLp) { minDistLp = dist; bestLp = lp; }
                            }
                        }
                        if (bestLp) combinedPts.push(...getShortestContourSegment(bestLp, arc[0], branch.arcs[i+1][0]).slice(1));
                    }
                }
            }

            if (combinedPts.length > 1) {
                coreSegments.push({ type: "spiral", pts: combinedPts, helicalCircles: helicalCircles.filter(c => c.bounds.center().distTo2D(entryPt) < 0.1), rampContours: rampContours.filter(c => c.first().distTo2D(entryPt) < 0.1), plungePoints: plungePoints.filter(c => c.bounds.center().distTo2D(entryPt) < 0.1) });
                let clearedCore = expandToolpath(newPolygon().setOpen().addPoints(combinedPts), toolRadius, z);
                if (clearedCore.length > 0) cleared.push(...POLY.flatten(clearedCore));
            }
        }
    }

    let pocketRoot = levels[0][0];
    let slotSegments = [];
    if (pocketRoot) {
        let maGraph = buildMATGraph(pocketRoot);
        slotSegments = generateSlotToolpaths(extractSlotChains(maGraph, toolRadius, toolOver, leave_xy), toolRadius, toolOver, z, leave_xy, cleared, classifiedSegments);
        for (let seg of slotSegments) {
            let exp = expandToolpath(newPolygon().setOpen().addPoints(seg.pts), toolRadius, z);
            if (exp) cleared.push(...POLY.flatten(exp));
        }
    }

    let allToLink = [...coreSegments, ...slotSegments];
    let startPt = helicalCircles[0]?.bounds.center() || plungePoints[0]?.bounds.center() || newPoint(0, 0, z);
    let sorted = sortSegments(allToLink, startPt, clearFirst);
    let resultSegments = [];
    for (let seg of sorted) {
        let p = newPolygon().setOpen().addPoints(seg.pts);
        p.z = z;
        resultSegments.push({ type: seg.type, polys: [ p ], helicalCircles: seg.helicalCircles, rampContours: seg.rampContours, plungePoints: seg.plungePoints });
    }
    for (let seg of classifiedSegments) {
        if (seg.type === "centerline") {
            let p = newPolygon().setOpen().addPoints(seg.pts);
            p.z = z;
            resultSegments.push({ type: seg.type, polys: [ p ] });
        }
    }
    return resultSegments;
}

function expandToolpath(poly, toolRadius, z) {
    if (!poly || !poly.points || poly.points.length === 0) return [];
    if (poly.points.length === 1) return [ newPolygon().centerCircle(poly.points[0], toolRadius, 20) ];
    if (poly.open) {
        let res = paths.pointsToPath(poly.points, toolRadius, true);
        if (res && res.left && res.right && res.left.length && res.right.length) {
            let pts = [...res.left.map(p => newPoint(p.x, p.y, z)), ...res.right.reverse().map(p => newPoint(p.x, p.y, z))];
            return [ newPolygon().addPoints(pts) ];
        }
        return [];
    } else {
        let outer = POLY.expand([poly], toolRadius);
        let inner = POLY.expand([poly], -toolRadius);
        return inner && inner.length > 0 ? POLY.subtract(outer, inner, [], undefined, undefined, 0) : outer;
    }
}

function resamplePoints(pts, stepSize) {
    if (pts.length < 2) return pts;
    let resampled = [];
    let current = pts[0];
    resampled.push(current);
    let i = 1, distanceAccum = 0;
    while (i < pts.length) {
        let next = pts[i], d = current.distTo2D(next);
        if (distanceAccum + d >= stepSize) {
            let t = (stepSize - distanceAccum) / d;
            current = newPoint(current.x + (next.x - current.x) * t, current.y + (next.y - current.y) * t, current.z);
            resampled.push(current);
            distanceAccum = 0;
        } else { distanceAccum += d; current = next; i++; }
    }
    return resampled;
}

function lerpPoint(p1, p2, t) {
    return newPoint(p1.x + (p2.x - p1.x) * t, p1.y + (p2.y - p1.y) * t, p1.z);
}

function findEntryForPoly(poly, helicalCircles, rampContours, plungePoints) {
    if (!poly || !poly.points || poly.points.length === 0) return null;
    let bestPt = null, minDist = Infinity, firstPt = poly.first();
    for (let c of [...helicalCircles, ...plungePoints]) {
        let dist = firstPt.distTo2D(c.bounds.center());
        if (dist < minDist) { minDist = dist; bestPt = c.bounds.center(); }
    }
    for (let c of rampContours) {
        let dist = firstPt.distTo2D(c.first());
        if (dist < minDist) { minDist = dist; bestPt = c.first(); }
    }
    return bestPt;
}

function rotatePolyToPoint(poly, targetPt) {
    let pts = poly.points, n = pts.length;
    if (n < 2) return;
    let closestIndex = 0, minDist = Infinity;
    for (let i = 0; i < n; i++) {
        let dist = pts[i].distTo2D(targetPt);
        if (dist < minDist) { minDist = dist; closestIndex = i; }
    }
    if (closestIndex > 0) poly.points = [...pts.slice(closestIndex), ...pts.slice(0, closestIndex)];
}

export { OpAdaptive };
