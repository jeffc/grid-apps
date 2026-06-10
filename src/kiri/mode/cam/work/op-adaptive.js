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
                    leave_xy
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
                            slice.helicalCircles = helicalCircles.map(c => c.clone(true));
                        }
                        if (rampContours.length > 0) {
                            layers
                                .setLayer("ramp-entry", { line: 0xffff00 }, false)
                                .addPolys(rampContours);
                            slice.rampContours = rampContours.map(c => c.clone(true));
                        }
                        if (plungePoints.length > 0) {
                            layers
                                .setLayer("plunge-entry", { line: 0x00ffff }, false)
                                .addPolys(plungePoints);
                            slice.plungePoints = plungePoints.map(p => p.clone(true));
                        }
                        isFirst = false;
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

/**
 * Traces a narrow slot centerline from a starting arc and generates a CCW micro-trochoidal toolpath along it.
 */
function processSlotBranch(startArc, entryPt, R_0, numPoints, toolRadius, toolOver, z, limitPoly, pocketRoot, isPointSafe) {
    let branch = {
        arcs: [ startArc ],
        theta_center: 0
    };
    let midPt = startArc[Math.floor(startArc.length / 2)];
    branch.theta_center = Math.atan2(midPt.y - entryPt.y, midPt.x - entryPt.x);
    
    let slotCenterPts = [ midPt ];
    let currPt = midPt;
    // Initialize direction pointing from entryPt to midPt
    let dx = midPt.x - entryPt.x;
    let dy = midPt.y - entryPt.y;
    let len = Math.hypot(dx, dy);
    let dirX = dx / (len || 1);
    let dirY = dy / (len || 1);
    
    let slotLoopCount = 0;
    while (slotLoopCount < 1000) {
        let bestPt = null;
        let bestScore = -Infinity;
        
        let numCheckPoints = 32;
        let ds = toolOver;
        for (let i = 0; i < numCheckPoints; i++) {
            let theta = (i / numCheckPoints) * 2 * Math.PI;
            let vx = Math.cos(theta);
            let vy = Math.sin(theta);
            
            // Dot product with current direction (allow up to ~100 degree turns)
            let dot = vx * dirX + vy * dirY;
            if (dot <= -0.2) continue;
            
            let testPt = newPoint(currPt.x + ds * vx, currPt.y + ds * vy, z);
            if (isPointSafe(testPt)) {
                let dist = getDistToWalls(testPt, pocketRoot);
                let score = dot + dist * 0.5;
                if (score > bestScore) {
                    bestScore = score;
                    bestPt = testPt;
                }
            }
        }
        
        if (bestPt) {
            // Check if we are looping back to start (for closed rings)
            if (slotLoopCount > 3 && bestPt.distTo2D(midPt) < toolOver * 1.5) {
                slotCenterPts.push(midPt);
                break;
            }
            slotCenterPts.push(bestPt);
            // Update direction vector
            let ndx = bestPt.x - currPt.x;
            let ndy = bestPt.y - currPt.y;
            let nlen = Math.hypot(ndx, ndy);
            dirX = ndx / (nlen || 1);
            dirY = ndy / (nlen || 1);
            currPt = bestPt;
        } else {
            break;
        }
        slotLoopCount++;
    }
    
    if (slotCenterPts.length > 0) {
        let stepSize = toolOver * 0.35;
        let resampled = resamplePoints(slotCenterPts, stepSize);
        let m = resampled.length;
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
            normals.push({ x: -Ty, y: Tx }); // N is CCW of T (pointing to left wall)
        }
        
        let trochoidPath = [];
        for (let j = 0; j < m; j++) {
            let rp = resampled[j];
            let T = tangents[j];
            let N = normals[j];
            let cp = getClosestPointOnPoly(rp, pocketRoot);
            let distToWall = cp ? rp.distTo2D(cp) : toolRadius;
            let R_path = Math.max(toolRadius / 2, distToWall - toolRadius);
            
            // Generate CCW cutting arc starting from the right wall (-N),
            // swinging forward (+T), and ending at the left wall (+N).
            let steps = 12;
            let loopPts = [];
            for (let k_step = 0; k_step <= steps; k_step++) {
                let phi = -Math.PI / 2 + (k_step / steps) * Math.PI;
                let x = rp.x + R_path * Math.cos(phi) * T.x + R_path * Math.sin(phi) * N.x;
                let y = rp.y + R_path * Math.cos(phi) * T.y + R_path * Math.sin(phi) * N.y;
                loopPts.push(newPoint(x, y, z));
            }
            
            if (j === 0) {
                trochoidPath.push(...loopPts);
            } else {
                let prevEnd = trochoidPath[trochoidPath.length - 1];
                let currStart = loopPts[0];
                // Vertical Z-hop retract
                trochoidPath.push(newPoint(prevEnd.x, prevEnd.y, z + 0.5));
                trochoidPath.push(newPoint(currStart.x, currStart.y, z + 0.5));
                trochoidPath.push(newPoint(currStart.x, currStart.y, z));
                trochoidPath.push(...loopPts.slice(1));
            }
        }
        if (trochoidPath.length > 0) {
            branch.trochoidPath = trochoidPath;
            branch.slotCenterline = resampled;
        }
    }
    return branch;
}

/**
 * Generates linked spiral morphing and trochoidal peeling segments between concentric offset loops.
 * Walks from the innermost loops outward.
 */
function generateLinkedToolpath(levels, toolRadius, toolOver, z, helicalCircles, rampContours, plungePoints, bounds, stock, leave_xy) {
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

    // --- PASS 1: SECTIONING & CLASSIFICATION ---
    let classifiedSegments = [];

    // Process only the innermost loops (at d = N - 1)
    let innermostPolys = flatLevels[N - 1];
    for (let poly of innermostPolys) {
        let entryPt = findEntryForPoly(poly, helicalCircles, rampContours, plungePoints);
        if (!entryPt) {
            entryPt = poly.first();
        }
        
        if (entryPt) {
            // 1. Calculate pocket root, pocket loops, and limit polygon
            let pocketRoot = null;
            let firstPt = poly.first();
            for (let root of levels[0]) {
                if (firstPt.isInPolygon(root)) {
                    pocketRoot = root;
                    break;
                }
            }
            if (!pocketRoot && levels[0].length > 0) {
                pocketRoot = levels[0][0];
            }

            let pocketLoops = [];
            if (pocketRoot) {
                pocketLoops.push(pocketRoot);
                if (pocketRoot.inner) {
                    pocketLoops.push(...flattenToSimpleLoops(pocketRoot.inner));
                }
            }

            let limitPoly = null;
            if (pocketRoot) {
                limitPoly = POLY.expand([pocketRoot], -(toolRadius + leave_xy));
            }

            // Helper to check if point is inside limitPoly
            let isPointSafe = function(pt) {
                if (limitPoly && limitPoly.length > 0 && pt.isInPolygon(limitPoly)) {
                    return true;
                }
                // Fallback for thin/collapsed slots: check if point is inside the pocket boundary
                // and at least toolRadius + leave_xy - 0.05 distance from the walls.
                return pt.isInPolygon(pocketRoot) && getDistToWalls(pt, pocketRoot) >= (toolRadius + leave_xy - 0.05);
            };

            // 2. Find closest radius to pocket boundaries (including islands)
            let R_max = Infinity;
            for (let loop of pocketLoops) {
                let dist = entryPt.distToPolySegments(loop);
                if (dist < R_max) R_max = dist;
            }
            let R_largest = R_max - toolRadius - leave_xy;

            // 3. Find centerline circle radius of the helical entry
            let H_r = 0.01;
            for (let c of helicalCircles) {
                let cp = c.bounds.center();
                if (cp.distTo2D(entryPt) < 0.1) {
                    H_r = c.points[0].distTo2D(cp);
                    break;
                }
            }

            let combinedPts = [];

            // 4. Generate Archimedean Spiral if space allows
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

                // 4b. Add perfectly circular closing pass at the maximum radius
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

                // Update cleared area in Pass 1 with the spiral and closing pass
                let spiralPoly = newPolygon().setOpen().addPoints(spiralPts.concat(closingPts));
                let clearedSpiral = expandToolpath(spiralPoly, toolRadius, z);
                if (clearedSpiral.length > 0) {
                    cleared.push(...POLY.flatten(clearedSpiral));
                }
            }

            // 5. Generate expanding circle arcs depth-first
            let R_0 = (R_largest > H_r ? R_largest : H_r) + toolOver;
            let toolDiameter = toolRadius * 2;
            
            // Generate initial circles to find starting branches
            let initialArcs = [];
            let numPoints = 128;
            let circlePts = [];
            for (let step = 0; step < numPoints; step++) {
                let theta = (step / numPoints) * 2 * Math.PI;
                let x = entryPt.x + R_0 * Math.cos(theta);
                let y = entryPt.y + R_0 * Math.sin(theta);
                circlePts.push(newPoint(x, y, z));
            }

            let ptSafe = circlePts.map(pt => isPointSafe(pt));
            let inArc = false;
            let currentArc = [];
            
            for (let step = 0; step < numPoints; step++) {
                if (ptSafe[step]) {
                    if (!inArc) {
                        inArc = true;
                        currentArc = [ circlePts[step] ];
                    } else {
                        currentArc.push(circlePts[step]);
                    }
                } else {
                    if (inArc) {
                        inArc = false;
                        initialArcs.push(currentArc);
                        currentArc = [];
                    }
                }
            }
            if (inArc && currentArc.length > 0) {
                if (initialArcs.length > 0 && ptSafe[0]) {
                    initialArcs[0] = currentArc.concat(initialArcs[0]);
                } else {
                    initialArcs.push(currentArc);
                }
            }

            // For each starting arc, clear it depth-first
            let branches = [];
            for (let startArc of initialArcs) {
                if (startArc.length <= 1) continue;

                // Check starting arc chord length (bypass if it's a full circle)
                let isFullCircle = (startArc.length >= numPoints - 5);
                let chordLen = startArc[0].distTo2D(startArc[startArc.length - 1]);
                
                if (!isFullCircle && chordLen <= toolDiameter) {
                    // This is a narrow slot branch right at R_0. Process it as a slot branch immediately!
                    let slotBranch = processSlotBranch(startArc, entryPt, R_0, numPoints, toolRadius, toolOver, z, limitPoly, pocketRoot, isPointSafe);
                    branches.push(slotBranch);
                    continue;
                }

                let branch = {
                    arcs: [ startArc ],
                    theta_center: 0
                };
                
                // Compute initial center angle
                let midPt = startArc[Math.floor(startArc.length / 2)];
                branch.theta_center = Math.atan2(midPt.y - entryPt.y, midPt.x - entryPt.x);

                // Offset further (depth-first)
                let current_R = R_0;
                let loopCount = 0;
                while (loopCount < 1000) {
                    current_R += toolOver;
                    
                    // Generate circle points at current_R
                    let cPts = [];
                    for (let step = 0; step < numPoints; step++) {
                        let theta = (step / numPoints) * 2 * Math.PI;
                        let x = entryPt.x + current_R * Math.cos(theta);
                        let y = entryPt.y + current_R * Math.sin(theta);
                        cPts.push(newPoint(x, y, z));
                    }
                    
                    let cPtSafe = cPts.map(pt => isPointSafe(pt));
                    let cArcs = [];
                    let cInArc = false;
                    let cCurrentArc = [];
                    
                    for (let step = 0; step < numPoints; step++) {
                        if (cPtSafe[step]) {
                            if (!cInArc) {
                                cInArc = true;
                                cCurrentArc = [ cPts[step] ];
                            } else {
                                cCurrentArc.push(cPts[step]);
                            }
                        } else {
                            if (cInArc) {
                                cInArc = false;
                                cArcs.push(cCurrentArc);
                                cCurrentArc = [];
                            }
                        }
                    }
                    if (cInArc && cCurrentArc.length > 0) {
                        if (cArcs.length > 0 && cPtSafe[0]) {
                            cArcs[0] = cCurrentArc.concat(cArcs[0]);
                        } else {
                            cArcs.push(cCurrentArc);
                        }
                    }
                    
                    if (cArcs.length === 0) break;

                    // Find the safe arc closest to branch.theta_center
                    let bestArc = null;
                    let minDiff = Infinity;
                    for (let arc of cArcs) {
                        if (arc.length <= 1) continue;
                        let arcMidPt = arc[Math.floor(arc.length / 2)];
                        let arcTheta = Math.atan2(arcMidPt.y - entryPt.y, arcMidPt.x - entryPt.x);
                        let diff = Math.abs(branch.theta_center - arcTheta);
                        if (diff > Math.PI) diff = 2 * Math.PI - diff;
                        if (diff < minDiff) {
                            minDiff = diff;
                            bestArc = arc;
                        }
                    }

                    // If a matching arc is found within Pi/3 radians and its chord length is > toolDiameter (or it is a full circle)
                    if (bestArc && minDiff < Math.PI / 3) {
                        let isBestFull = (bestArc.length >= numPoints - 5);
                        let chordLen = bestArc[0].distTo2D(bestArc[bestArc.length - 1]);
                        if (isBestFull || chordLen > toolDiameter) {
                            branch.arcs.push(bestArc);
                            let arcMidPt = bestArc[Math.floor(bestArc.length / 2)];
                            branch.theta_center = Math.atan2(arcMidPt.y - entryPt.y, arcMidPt.x - entryPt.x);
                        } else {
                            // The slot has narrowed below toolDiameter. Let's trace it and generate slotting path!
                            let slotBranch = processSlotBranch(bestArc, entryPt, current_R, numPoints, toolRadius, toolOver, z, limitPoly, pocketRoot, isPointSafe);
                            if (slotBranch.trochoidPath) {
                                branch.trochoidPath = slotBranch.trochoidPath;
                            }
                            break;
                        }
                    } else {
                        break;
                    }
                    
                    loopCount++;
                }
                
                branches.push(branch);
            }

            // 6. Update cleared area in Pass 1 and build combinedPts with retracts, boundary milling, & finishing passes
            //
            // We clear the pocket branch-by-branch depth-first. For each branch, we plunge into a safe,
            // pre-cleared zone along the maximum inscribed circle (R_safe), feed to the start of the first
            // arc, and then traverse the expanding concentric arcs outward.
            //
            // To maintain climb milling (counter-clockwise tool movement) and clean pocket walls:
            // - After completing arc i to pEnd_i, we mill backward along the right wall contour to pEnd_{i-1}.
            // - We perform a vertical Z-hop (retract to z+0.5, rapid-travel to pStart_i, plunge back to z).
            // - We mill forward along the left wall contour from pStart_i to pStart_{i+1} (start of next arc).
            // - At the tip (outermost arc), we Z-hop back to pEnd_last and perform a finishing pass around the tip.
            for (let branch of branches) {
                let k = branch.arcs.length;
                if (k === 0) continue;
                
                // First arc of the branch
                let firstArc = branch.arcs[0];
                let startPt = firstArc[0];
                
                // Calculate the entry plunge point (plungePt):
                // To avoid plunging vertically into uncut stock, we place the plunge point at the boundary of
                // the already cleared Archimedean spiral / inscribed circle (R_safe) along the angle pointing
                // towards the start of this branch's first arc.
                let theta_start = Math.atan2(startPt.y - entryPt.y, startPt.x - entryPt.x);
                let R_safe = R_largest > H_r ? R_largest : H_r;
                let px = entryPt.x + R_safe * Math.cos(theta_start);
                let py = entryPt.y + R_safe * Math.sin(theta_start);
                let plungePt = newPoint(px, py, z);
                
                // Position/Feed Sequence:
                // 1. If we have finished a previous branch, retract vertically to z + 0.5 (Z-hop).
                // 2. Rapid-travel horizontally at z + 0.5 to the new plungePt.
                // 3. Plunge vertically down to the cutting depth z.
                // 4. Feed horizontally at cutting depth z to startPt of the first arc.
                if (combinedPts.length > 0) {
                    let lastPt = combinedPts[combinedPts.length - 1];
                    // Retract vertically to Z-hop travel clearance height (z + 0.5)
                    combinedPts.push(newPoint(lastPt.x, lastPt.y, z + 0.5));
                    // Rapid-travel horizontally at clearance height to the safe plunge coordinates
                    combinedPts.push(newPoint(plungePt.x, plungePt.y, z + 0.5));
                    // Plunge vertically to the current layer's cutting depth z
                    combinedPts.push(newPoint(plungePt.x, plungePt.y, z));
                } else {
                    // Initial plunge sequence if this is the very first path segment
                    combinedPts.push(newPoint(plungePt.x, plungePt.y, z + 0.5));
                    combinedPts.push(newPoint(plungePt.x, plungePt.y, z));
                }
                
                // Feed horizontally from the safe entry plunge point to the start of the first clearing arc
                combinedPts.push(startPt);
                
                // Process all concentric arcs in the branch outward
                for (let i = 0; i < k; i++) {
                    let arc = branch.arcs[i];
                    
                    // Mill the arc to pEnd_i.
                    // For the first arc (i = 0), we already pushed startPt, so we push all points.
                    // For subsequent arcs, we exclude the start point (arc.slice(1)) to prevent duplicate points.
                    if (i > 0) {
                        combinedPts.push(...arc.slice(1));
                    } else {
                        combinedPts.push(...arc);
                    }
                    
                    // Update the cleared area tracker in Pass 1 to include the area swept by this arc.
                    // This ensures any subsequent path planning recognizes this region as empty space.
                    let arcPoly = newPolygon().setOpen().addPoints(arc);
                    let clearedArc = expandToolpath(arcPoly, toolRadius, z);
                    if (clearedArc.length > 0) {
                        cleared.push(...POLY.flatten(clearedArc));
                    }
                    
                    // --- RIGHT WALL CONTOUR MILLING (if not the first arc) ---
                    // After milling the arc to its right-hand endpoint (pEnd_i), we mill back along the
                    // right pocket boundary to the end of the previous arc (pEnd_prev).
                    // Moving from outer to inner along the right wall maintains climb-milling (material is to the right).
                    if (i > 0) {
                        let pEnd_i = arc[arc.length - 1];
                        let prevArc = branch.arcs[i - 1];
                        let pEnd_prev = prevArc[prevArc.length - 1];
                        
                        // Find the closest contour loop of the limit polygon to project onto
                        let bestLp = null;
                        let minDistLp = Infinity;
                        if (limitPoly) {
                            for (let lp of limitPoly) {
                                let dist = pEnd_i.distToPolySegments(lp);
                                if (dist < minDistLp) {
                                    minDistLp = dist;
                                    bestLp = lp;
                                }
                            }
                        }
                        
                        if (bestLp) {
                            // projectToSegment and getShortestContourSegment project raw endpoints onto the physical boundary,
                            // then trace the exact wall contours between them. This forces the toolpath to be perfectly parallel
                            // and colinear with straight pocket walls, avoiding angular drift.
                            let rightWallPts = getShortestContourSegment(bestLp, pEnd_i, pEnd_prev);
                            combinedPts.push(...rightWallPts.slice(1)); // exclude start to avoid duplicate
                            
                            // Update cleared area tracker
                            let rightWallPoly = newPolygon().setOpen().addPoints(rightWallPts);
                            let clearedRight = expandToolpath(rightWallPoly, toolRadius, z);
                            if (clearedRight.length > 0) {
                                cleared.push(...POLY.flatten(clearedRight));
                            }
                        }
                    }
                    
                    // --- Z-HOP & LEFT WALL CONTOUR MILLING (if not the last arc) ---
                    // If we have more arcs to clear in this branch, we need to transition from the right wall
                    // back to the start of the next arc (which begins on the left wall).
                    // Rather than dragging the tool across the pocket bottom, we perform a vertical Z-hop retract,
                    // rapid-travel, plunge, and then mill along the left wall contour to the next start (pStart_next).
                    if (i < k - 1) {
                        let lastPt = combinedPts[combinedPts.length - 1];
                        let pStart_curr = arc[0];
                        let nextArc = branch.arcs[i + 1];
                        let pStart_next = nextArc[0];
                        
                        // Vertical Z-hop retract sequence:
                        // 1. Lift vertically by 0.5mm above the current cut depth z.
                        // 2. Rapid-travel horizontally at clearance height to the start of the current arc.
                        // 3. Plunge vertically back to cutting depth z.
                        combinedPts.push(newPoint(lastPt.x, lastPt.y, z + 0.5));
                        combinedPts.push(newPoint(pStart_curr.x, pStart_curr.y, z + 0.5));
                        combinedPts.push(newPoint(pStart_curr.x, pStart_curr.y, z));
                        
                        // Mill the left wall boundary contour from pStart_curr to pStart_next.
                        // Moving from inner to outer along the left wall maintains climb-milling (material is to the left).
                        let bestLp = null;
                        let minDistLp = Infinity;
                        if (limitPoly) {
                            for (let lp of limitPoly) {
                                let dist = pStart_curr.distToPolySegments(lp);
                                if (dist < minDistLp) {
                                    minDistLp = dist;
                                    bestLp = lp;
                                }
                            }
                        }
                        
                        if (bestLp) {
                            // Trace the left wall boundary using segment-projected parallel paths
                            let leftWallPts = getShortestContourSegment(bestLp, pStart_curr, pStart_next);
                            combinedPts.push(...leftWallPts.slice(1)); // exclude start to avoid duplicate
                            
                            // Update cleared area tracker
                            let leftWallPoly = newPolygon().setOpen().addPoints(leftWallPts);
                            let clearedLeft = expandToolpath(leftWallPoly, toolRadius, z);
                            if (clearedLeft.length > 0) {
                                cleared.push(...POLY.flatten(clearedLeft));
                            }
                        }
                    }
                }
                
                if (branch.trochoidPath) {
                    // Z-hop from the end of the last circular arc to the start of the trochoidal slotting path
                    let lastPt = combinedPts[combinedPts.length - 1];
                    let tStartPt = branch.trochoidPath[0];
                    combinedPts.push(newPoint(lastPt.x, lastPt.y, z + 0.5));
                    combinedPts.push(newPoint(tStartPt.x, tStartPt.y, z + 0.5));
                    combinedPts.push(newPoint(tStartPt.x, tStartPt.y, z));
                    
                    // Append the trochoidal path (exclude the first point since we plunged to it)
                    combinedPts.push(...branch.trochoidPath.slice(1));
                    
                    // Update cleared area in Pass 1 with the swept area of the slot
                    let trochoidPoly = newPolygon().setOpen().addPoints(branch.trochoidPath);
                    let clearedTrochoid = expandToolpath(trochoidPoly, toolRadius, z);
                    if (clearedTrochoid.length > 0) {
                        cleared.push(...POLY.flatten(clearedTrochoid));
                    }
                } else {
                    // --- OUTMOST ARC TIP FINISHING PASS ---
                    // After completing the final arc (lastArc) and milling back along the right wall contour, the tool
                    // is left at the end of the previous arc (pEnd_{last-1}). We perform a Z-hop back to the tip's end (pEnd_last),
                    // and then mill along the outer boundary tip contour to the tip's start (pStart_last).
                    // This clears the remaining boundary stock at the outermost tip of the channel.
                    let lastArc = branch.arcs[k - 1];
                    let pStart_last = lastArc[0];
                    let pEnd_last = lastArc[lastArc.length - 1];
                    
                    let bestLp = null;
                    let minDistLp = Infinity;
                    if (limitPoly) {
                        for (let lp of limitPoly) {
                            let dist = pEnd_last.distToPolySegments(lp);
                            if (dist < minDistLp) {
                                minDistLp = dist;
                                bestLp = lp;
                            }
                        }
                    }
                    
                    if (bestLp) {
                        // Z-hop from the inner position (pEnd_{last-1}) back to the outermost tip endpoint (pEnd_last).
                        // (Skip this Z-hop if the branch only has a single arc, k = 1, as the tool is already at the tip).
                        if (k > 1) {
                            let lastPt = combinedPts[combinedPts.length - 1];
                            combinedPts.push(newPoint(lastPt.x, lastPt.y, z + 0.5));
                            combinedPts.push(newPoint(pEnd_last.x, pEnd_last.y, z + 0.5));
                            combinedPts.push(newPoint(pEnd_last.x, pEnd_last.y, z));
                        }
                        
                        // Trace the outer boundary around the tip from pEnd_last to pStart_last
                        let tipPts = getShortestContourSegment(bestLp, pEnd_last, pStart_last);
                        combinedPts.push(...tipPts.slice(1));
                        
                        // Update cleared area with tip pass
                        let tipPoly = newPolygon().setOpen().addPoints(tipPts);
                        let clearedTip = expandToolpath(tipPoly, toolRadius, z);
                        if (clearedTip.length > 0) {
                            cleared.push(...POLY.flatten(clearedTip));
                        }
                    }
                }
            }

            // Pack the entire linked sequence of points as a single toolpath segment
            if (combinedPts.length > 1) {
                classifiedSegments.push({
                    type: "spiral",
                    pts: combinedPts,
                    isInnermost: true,
                    isClosed: false,
                    n: combinedPts.length
                });
            }

            for (let branch of branches) {
                if (branch.slotCenterline && branch.slotCenterline.length > 1) {
                    classifiedSegments.push({
                        type: "centerline",
                        pts: branch.slotCenterline,
                        isInnermost: true,
                        isClosed: false,
                        n: branch.slotCenterline.length
                    });
                }
            }
        }
    }

    // --- PASS 2: GEOMETRY GENERATION ---
    // Convert the raw sequences of point coordinates into Polygon objects for CAM output.
    //
    // CRITICAL DETAIL:
    // We instantiate the newPolygon() as open and copy all points. Then we set 'p.z = z'
    // on the polygon container. We MUST NOT call 'POLY.setZ([p], z)', because that helper
    // iterates through all internal points and sets their 'pt.z = z'. Doing so would overwrite
    // and flatten all the z+0.5 Z-hop retract coordinates back to the cutting depth z, wiping
    // out vertical moves. Setting 'p.z = z' preserves individual point z-coordinates.
    let resultSegments = [];
    for (let seg of classifiedSegments) {
        let segmentPolys = [];
        let segPts = [...seg.pts];
        if (segPts.length > 1) {
            let p = newPolygon().setOpen().addPoints(segPts);
            p.z = z;
            segmentPolys.push(p);
        }
        if (segmentPolys.length > 0) {
            resultSegments.push({
                type: seg.type,
                polys: segmentPolys
            });
        }
    }
    return resultSegments;
}

/**
 * Generates a closed polygon representing the cleared area of a tool path.
 * Supports both open line paths (spirals) and closed loop paths (trochoids/boundaries).
 */
function expandToolpath(poly, toolRadius, z) {
    if (!poly || !poly.points || poly.points.length === 0) {
        return [];
    }
    if (poly.points.length === 1) {
        let pt = poly.points[0];
        let circle = newPolygon().centerCircle(pt, toolRadius, 20);
        return [ circle ];
    }
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

/**
 * Finds the entry point closest to the first point of the given loop polygon.
 */
function findEntryForPoly(poly, helicalCircles, rampContours, plungePoints) {
    if (!poly || !poly.points || poly.points.length === 0) return null;
    let bestPt = null;
    let minDist = Infinity;
    let firstPt = poly.first();
    
    // Check helical circles
    for (let c of helicalCircles) {
        let cp = c.bounds.center();
        let dist = firstPt.distTo2D(cp);
        if (dist < minDist) {
            minDist = dist;
            bestPt = cp;
        }
    }
    // Check ramp contours
    for (let c of rampContours) {
        let cp = c.first();
        let dist = firstPt.distTo2D(cp);
        if (dist < minDist) {
            minDist = dist;
            bestPt = cp;
        }
    }
    // Check plunge points
    for (let c of plungePoints) {
        let cp = c.bounds.center();
        let dist = firstPt.distTo2D(cp);
        if (dist < minDist) {
            minDist = dist;
            bestPt = cp;
        }
    }
    
    return bestPt;
}

/**
 * Rotates the vertices of a polygon closed loop so that the vertex closest to a target point is at index 0.
 */
function rotatePolyToPoint(poly, targetPt) {
    let pts = poly.points;
    let n = pts.length;
    if (n < 2) return;
    
    let closestIndex = 0;
    let minDist = Infinity;
    for (let i = 0; i < n; i++) {
        let dist = pts[i].distTo2D(targetPt);
        if (dist < minDist) {
            minDist = dist;
            closestIndex = i;
        }
    }
    
    if (closestIndex > 0) {
        poly.points = [...pts.slice(closestIndex), ...pts.slice(0, closestIndex)];
    }
}

export { OpAdaptive };
