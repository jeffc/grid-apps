/** Copyright Stewart Allen <sa@grid.space> -- All Rights Reserved */

import { CamOp } from '../core/op.js';
import { Tool } from '../core/tool.js';
import { generate as Topo } from './topo3.js';
import { newPoint } from '../../../../geo/point.js';
import { newPolygon } from '../../../../geo/polygon.js';
import { newSlice } from '../../../core/slice.js';
import { tip2tipEmit } from '../../../../geo/paths.js';

function createFilter(op, origin, axis) {
    // console.log({ origin, axis });
    let ok = () => true;
    let filter = slices => slices;
    let filterString = op.filter?.map(l => l.trim()).join('\n');
    if (filterString) {
        try {
            const obj = eval(`( ${filterString} )`);
            let box = obj?.box;
            let slice_fn = obj?.slices;
            let index = 0;
            const accept = [];
            filter = function (slices) {
                for (let slice of slices.filter(s => s.camLines)) {
                    if (slice_fn && slice_fn(slice, index++)) {
                        accept.push(slice);
                    } else if (box) {
                        // slice.z = x when axis = y
                        // slice.z = y when axis = x
                        let { x, y, z } = box;
                        x = x ?? [ -Infinity, Infinity ];
                        y = y ?? [ -Infinity, Infinity ];
                        z = z ?? [ -Infinity, Infinity ];
                        let ok = false;
                        if (axis === 'x') {
                            let sy = slice.z + origin.y;
                            if (sy >= y[0] && sy <= y[1]) {
                                ok = true;
                                for (let p of slice.camLines) {
                                    p.points = p.points.filter(p =>
                                        p.x - origin.x >= x[0] && p.x - origin.x <= x[1] &&
                                        p.z - origin.z >= z[0] && p.z - origin.z <= z[1]
                                    );
                                }
                            }
                        } else if (axis === 'y') {
                            let sx = slice.z - origin.x;
                            if (sx >= x[0] && sx <= x[1]) {
                                ok = true;
                                for (let p of slice.camLines) {
                                    p.points = p.points.filter(p =>
                                        p.y + origin.y >= y[0] && p.y + origin.y <= y[1] &&
                                        p.z - origin.z >= z[0] && p.z - origin.z <= z[1]
                                    );
                                }
                            }
                        } else if (axis === 'radial') {
                            ok = true;
                            for (let p of slice.camLines) {
                                p.points = p.points.filter(p =>
                                    p.x - origin.x >= x[0] && p.x - origin.x <= x[1] &&
                                    p.y - origin.y >= y[0] && p.y - origin.y <= y[1] &&
                                    p.z - origin.z >= z[0] && p.z - origin.z <= z[1]
                                );
                            }
                        }
                        if (ok) {
                            slice.camLines = slice.camLines.map(p => {
                                if (p.points.length > 1) {
                                    return newPolygon(p.points).setOpen(true);
                                } else {
                                    return undefined;
                                }
                            }).filter(p => p);
                            accept.push(slice);
                        }
                    }
                }
                return accept;
            };
        } catch (e) {
            console.log('filter parse error', e, op.filter);
        }
    }
    return filter;
}

class OpContour extends CamOp {
    constructor(state, op) {
        super(state, op);
    }

    async slice(progress) {
        let { op, state } = this;
        let { color, addSlices, settings, updateToolDiams } = state;

        let conTool = new Tool(settings, op.tool);
        let filter = createFilter(op, settings.origin, op.axis.toLowerCase());
        let toolDiam = this.toolDiam = conTool.fluteDiameter();
        this.toolStep = conTool.getStepSize(op.step);

        updateToolDiams(toolDiam);

        // we need topo for safe travel moves when roughing and outlining
        // not generated when drilling-only. then all z moves use bounds max.
        // also generates x and y contouring when selected
        let topo = this.topo = await Topo({
            // onupdate: (update, msg) => {
            onupdate: (index, total, msg) => {
                progress(index / total, msg);
            },
            ondone: (slices, topo) => {
                // If 'Omit Through' is enabled, post-process the generated toolpath slices
                // to cleanly snap coordinates crossing any through-hole onto the hole perimeter.
                if (op.omitthru && state.shadow && state.shadow.holes && state.shadow.holes.length) {
                    slices = cleanupContourSlices(slices, state.shadow.holes, topo, op);
                }
                slices = filter(slices);
                this.sliceOut = slices;
                addSlices(slices);
                for (let slice of slices) {
                    slice.output()
                        .setLayer(`contour ${op.axis}`, { face: color, line: color })
                        .addPolys(slice.camLines);
                }
            },
            contour: op,
            state
        });

        if (this.debug && topo.coastline) {
            console.log('coastline', topo.coastline);
            const dbs = newSlice(-1);
            const dbo = dbs.output();
            dbo.setLayer("coastline", { line: 0x0000dd }).addPolys(topo.coastline);
            addSlices([ dbs ])
        }

        // computed if set to 0
        this.tolerance = topo.tolerance;
    }

    prepare(ops, progress) {
        let { op, sliceOut, state, toolStep, topo } = this;
        let { settings } = state;
        let { process } = settings;

        let { polyEmit, setContouring, setTolerance, setTool, setTravelBoundary } = ops;
        let { widget, newLayer, zmax } = ops;

        let bounds = widget.getBoundingBox();
        let depthData = [];

        setTool(op.tool, op.rate, process.camFastFeedZ);
        setContouring(true, toolStep * 1.5, topo.coastline);
        if (state.shadow && state.shadow.base) {
            setTravelBoundary(state.shadow.base);
        }
        setTolerance(this.tolerance);

        let printPoint = newPoint(bounds.min.x, bounds.min.y, zmax);

        const isRadial = op.axis.toLowerCase() === 'radial';

        if (isRadial) {
            // RADIAL FINISHING TOOLPATH EMISSION:
            // Radial axis mode finishes the surface concentric-loop by concentric-loop or turns of a spiral.
            // Unlike linear X/Y parallel finishing passes where all segments are accumulated together
            // and ordered globally, in Radial Concentric mode we emit loops slice-by-slice (from innermost
            // out) and run tip-to-tip path ordering per loop to minimize travel and prevent collisions.
            for (let slice of sliceOut) {
                // ignore debug slices
                if (!slice.camLines) {
                    continue;
                }
                let polys = [];
                slice.camLines.forEach((poly) => {
                    poly = poly.clone(true).annotate({ slice: slice.index + 1 });
                    polys.push({ first: poly.first(), last: poly.last(), poly: poly });
                });
                // Find optimized path routing (tip2tip) on the loops within this slice
                printPoint = tip2tipEmit(polys, printPoint, (el, point) => {
                    let poly = el.poly;
                    if (poly.isClosed()) {
                        // Closed concentric loops are set to CounterClockwise for standard milling direction
                        poly.setCounterClockwise();
                        polyEmit(poly, -999);
                    } else {
                        // Open concentric arcs: reverse traversal direction if the end point is closer
                        if (poly.last() === point) {
                            poly.reverse();
                        }
                        polyEmit(poly);
                    }
                    newLayer();
                });
            }
        } else {
            // STANDARD LINEAR X/Y FINISHING EMISSION:
            // Accumulate all segments across all slices first, then run a single global tip-to-tip path optimizer.
            for (let slice of sliceOut) {
                // ignore debug slices
                if (!slice.camLines) {
                    continue;
                }
                let polys = [], poly;
                slice.camLines.forEach((poly) => {
                    poly = poly.clone(true).annotate({ slice: slice.index + 1 });
                    polys.push({ first: poly.first(), last: poly.last(), poly: poly });
                });
                depthData.appendAll(polys);
            }

            tip2tipEmit(depthData, printPoint, (el, point) => {
                let poly = el.poly;
                if (poly.isClosed()) {
                    poly.setCounterClockwise();
                    polyEmit(poly, -999);
                } else {
                    if (poly.last() === point) {
                        poly.reverse();
                    }
                    polyEmit(poly);
                }
                newLayer();
            });
        }
    }
}

// Finds the closest point on segment AB to point P. Used to project points onto boundaries.
function closestPointOnSegment(p, a, b) {
    let abx = b.x - a.x;
    let aby = b.y - a.y;
    let apx = p.x - a.x;
    let apy = p.y - a.y;
    let ab2 = abx * abx + aby * aby;
    if (ab2 === 0) return a;
    let t = (apx * abx + apy * aby) / ab2;
    t = Math.max(0, Math.min(1, t));
    return newPoint(a.x + t * abx, a.y + t * aby);
}

// Finds the closest point on the entire polygon perimeter to pt.
function closestPointOnPolygon(pt, poly) {
    let points = poly.points;
    let len = points.length;
    let minD = Infinity;
    let closest = null;
    for (let i = 0; i < len; i++) {
        let p1 = points[i];
        let p2 = points[(i + 1) % len];
        let cp = closestPointOnSegment(pt, p1, p2);
        let d = pt.distTo2D(cp);
        if (d < minD) {
            minD = d;
            closest = cp;
        }
    }
    return closest;
}

// SLICE POST-PROCESSING SNAP-TO-HOLE (OMIT THROUGH option):
// Post-processes contour slice lines to snap segments that cross through-holes onto the hole boundary.
// When 'Omit Through' is active, the toolpath should not run inside the holes. For segments crossing
// the holes, we project points inside the hole onto the nearest point on the hole's perimeter,
// and sample the correct Z height at that edge. This creates clean, continuous contours wrapping
// around the through-holes rather than leaving jagged/broken segments.
function cleanupContourSlices(slices, holes, topo, op) {
    let holeBoxes = [];
    // Compute bounding boxes for each through-hole to accelerate polygon containment tests
    for (let hole of holes) {
        let min_x = Infinity, max_x = -Infinity, min_y = Infinity, max_y = -Infinity;
        for (let p of hole.points) {
            if (p.x < min_x) min_x = p.x;
            if (p.x > max_x) max_x = p.x;
            if (p.y < min_y) min_y = p.y;
            if (p.y > max_y) max_y = p.y;
        }
        holeBoxes.push({ min_x, max_x, min_y, max_y, hole });
    }

    for (let slice of slices) {
        if (!slice.camLines) continue;
        let newPolys = [];
        for (let poly of slice.camLines) {
            let newPoints = [];
            let inHolePolygon = null;
            let inHoleStart = null;
            let inHoleLast = null;

            // Helper to emit the snapped boundary segment once we exit the through-hole area
            const emitHole = () => {
                if (inHolePolygon) {
                    if (inHoleStart) {
                        newPoints.push(inHoleStart);
                    }
                    if (inHoleLast && inHoleLast !== inHoleStart) {
                        newPoints.push(inHoleLast);
                    }
                    inHolePolygon = null;
                    inHoleStart = null;
                    inHoleLast = null;
                }
            };

            for (let pt of poly.points) {
                let currentHole = null;
                for (let hb of holeBoxes) {
                    if (pt.x >= hb.min_x && pt.x <= hb.max_x && pt.y >= hb.min_y && pt.y <= hb.max_y) {
                        if (pt.isInPolygon(hb.hole)) {
                            currentHole = hb.hole;
                            break;
                        }
                    }
                }

                if (currentHole) {
                    // Point is inside a through-hole: snap it to the closest point on the hole perimeter
                    let edgePt = closestPointOnPolygon(pt, currentHole);
                    // Sample Z height at the perimeter edge so the tool remains at the correct part height
                    let z = (topo && topo.toolAtXY ? topo.toolAtXY(edgePt.x, edgePt.y) : pt.z) + (op.leave || 0);
                    let projectedPt = newPoint(edgePt.x, edgePt.y, z);

                    if (inHolePolygon === currentHole) {
                        inHoleLast = projectedPt;
                    } else {
                        emitHole();
                        inHolePolygon = currentHole;
                        inHoleStart = projectedPt;
                        inHoleLast = projectedPt;
                    }
                } else {
                    // Point is on solid part: emit normally
                    emitHole();
                    newPoints.push(pt);
                }
            }
            emitHole();

            if (newPoints.length > 1) {
                poly.points = newPoints;
                newPolys.push(poly);
            }
        }
        slice.camLines = newPolys;
    }
    return slices;
}

export { OpContour };