/** Copyright Stewart Allen <sa@grid.space> -- All Rights Reserved */

// todo: surface offset pattern
// todo: trace dogbones, merge overlap

import { CamOp } from './op.js';
import { Tool } from './tool.js';
import { newSlice } from '../../core/slice.js';
import { newPoint } from '../../../geo/point.js';
import { newPolygon } from '../../../geo/polygon.js';
import { polygons as POLY } from '../../../geo/polygons.js';
import { util as base_util } from '../../../geo/base.js';
import { CAM } from './driver-be.js';

const DEG2RAD = Math.PI / 180;
const clib = self.ClipperLib;
const ctyp = clib.ClipType;
const ptyp = clib.PolyType;
const cfil = clib.PolyFillType;
const ts_eps = 0.01;

/**
 * OpArea is a CAM operation that handles different area-based toolpath generations
 * like clearing, tracing, and surface milling. It takes a set of selected
 * areas (polygons), and depending on the mode, generates toolpaths.
 *
 * It's a versatile operation that forms the basis for several user-facing CAM
 * operations like "pocketing", "roughing", and "contouring".
 */
class OpArea extends CamOp {
    constructor(state, op) {
        super(state, op);
    }

    /**
     * The slice method is the core of the OpArea operation. It takes the user's
     * selections and settings and generates the toolpaths as a series of "slices".
     *
     * @param {function} progress - Function to report progress
     */
    async slice(progress) {
        let { op, state } = this;
        let { direction, down, expand, flats, flatOff, follow } = op;
        let { mode, outline, over, rename, smooth, tool } = op;
        let { addSlices, color, cutTabs, settings } = state;
        let { shadowAt, setToolDiam, tabs, widget, workarea } = state;

        let areaTool = new Tool(settings, tool);
        let smoothVal = (smooth ?? 0) / 10;
        let toolDiam = areaTool.fluteDiameter();
        let toolOver = areaTool.getStepSize(over);
        let zTop = workarea.top_z;
        let zBottom = workarea.bottom_z;
        let shadowBase = state.shadow.base;
        let thruHoles = state.shadow.holes;
        let roundSharps = settings.process.camRoundCorners;

        // also updates tab offsets
        setToolDiam(toolDiam);

        // --- Area and Surface Polygon Selection ---
        // selected area polygons: surfaces and edges
        let { devel, edgeangle } = settings.controller;
        let polys = [];
        let stack = [];
        let surfaces = this.surfaces = [];
        let areas = this.areas = [ stack ];
        let aminz = Infinity;

        function newArea() {
            if (stack.length) {
                stack = [];
                areas.push(stack);
            }
        }

        function newLayer(z) {
            stack.push(newSlice(z));
            return stack.peek();
        }

        // gather area selections (from user clicks)
        for (let arr of (op.areas[widget.id] ?? [])) {
            let poly = newPolygon().fromArray(arr);
            aminz = Math.min(aminz, poly.minZ());
            polys.push(poly);
        }

        // connect open poly edge segments into closed loops (when possible)
        // surface and edge selections produce open polygons by default
        polys = POLY.nest(POLY.reconnect(polys, false));

        // gather surface selections (from face selections)
        let vert = widget.getGeoVertices({ unroll: true, translate: true }).map(v => v.round(4));
        let faces = CAM.surface_find(widget, (op.surfaces[widget.id] ?? []), (follow ?? edgeangle ?? 5) * DEG2RAD);
        let fpoly = [];
        for (let face of faces) {
            let i = face * 9;
            fpoly.push(newPolygon()
                .add(vert[i++], vert[i++], aminz = Math.min(aminz, vert[i++]))
                .add(vert[i++], vert[i++], aminz = Math.min(aminz, vert[i++]))
                .add(vert[i++], vert[i++], aminz = Math.min(aminz, vert[i++]))
            );
        }
        // remove invalid edges (eg. when vertical walls are the only selection)
        fpoly = fpoly.filter(p => p.area() > 0.001);

        // add in unioned surface areas
        polys.push(...POLY.setZ(POLY.union(fpoly, 0.00001, true), aminz));

        // --- Polygon Manipulation ---
        // smoothing for jaggies usually caused by vertical walls
        if (smoothVal) {
            polys = polys.map(poly => POLY.offset(POLY.offset([ poly ], smoothVal), -smoothVal)).flat();
            POLY.setZ(polys, aminz);
        }

        // expand selections (flattens z variable polys)
        if (Math.abs(expand) > 0) {
            let nupolys = polys.filter(p => p.open); // set aside open
            for (let p of polys.filter(p => !p.open)) {
                let expanded = POLY.expand([ p ], expand);
                if (expanded) {
                    POLY.setZ(expanded, p.minZ());
                    nupolys.push(...expanded.flat());
                }
            }
            // polys = nupolys;
            // re-merge after expansion in case it produces overlap
            polys = POLY.union(nupolys, 0.00001, true);
        }

        // --- Toolpath Generation ---
        // process each area separately
        let proc = 0;
        let pinc = 1 / polys.length;
        for (let area of polys) {
            let bounds = area.getBounds3D();
            // tool shadow offset, used for travel boundaries
            let ts_off = toolDiam / 2 - ts_eps + (op.leave_xy ?? 0);
            let offopt = {
                arc: 250,
                join: roundSharps ? ClipperLib.JoinType.jtRound : undefined,
                clean: true,
                simple: false,
                cleanDist: 10,
                minArea: 0.01,
            };

            if (outline) {
                // remove inner voids when processing outline only
                area.inner = undefined;
            }

            // for debugging, output the selected area
            newLayer().output()
                .setLayer("area", { line: 0xff8800 }, false)
                .addPolys([ area ]);

            newArea();

            // 'clear' mode: Pocketing operation
            if (mode === 'clear') {
                let zMov = flatOff ?? 0;
                let zs = flats ?
                    flats.filter(z => z <= zTop && z >= zBottom).map(v => v + zMov) :
                    down ? base_util.lerp(zTop, zBottom, down) : [ bounds.min.z ];
                let zroc = 0;
                let zinc = 1 / zs.length;
                let lzo;

                outer: for (;;)
                for (let z of zs) {
                    let slice = newLayer(z);
                    let layers = slice.output();
                    let shadow = await shadowAt(z);
                    // tool_shadow is used to create travel boundaries
                    let tool_shadow = [
                        ...POLY.offset(shadow, [  ts_off ], { count: 1, z, ...offopt }),
                        ...POLY.offset(shadow, [ -ts_off ], { count: 1, z, ...offopt }),
                    ];
                    // for roughing/outline backward compatability
                    if (op.omitthru) {
                        shadow = omitMatching(shadow, thruHoles);
                    }
                    // progressive offset of polygons inside area clipped to the shadow
                    let outs = [];
                    let clip = [];
                    let firstOff = -(toolDiam / 2 + (op.leave_xy ?? 0));
                    POLY.subtract([ area ], shadow, clip, undefined, undefined, 0);
                    POLY.offset(clip, [ firstOff, -toolOver ], {
                        count: op.steps ?? 999, outs, flat: true, z: z - zMov, ...offopt
                    });
                    // if we see no offsets, re-check the mesh bottom Z then exit
                    if (outs.length === 0) {
                        if (bounds && lzo > bounds.min.z) {
                            // try a bottom layer matching bottom of selection
                            zs = [ bounds.min.z ];
                            bounds = undefined;
                            continue outer;
                        }
                        // terminate z descent when no further output possible
                        break outer;
                    }
                    // support legacy outline features
                    if (op.omitouter) {
                        outs = omitOuter(outs);
                    } else if (op.omitinner) {
                        outs = omitInner(outs);
                    }
                    // cut tabs when present
                    if (tabs.length) outs = cutTabs(tabs, outs);
                    // for roughing backward compatability
                    if (op.leave_z) {
                        for (let out of outs)
                            for (let p of out.points)
                                p.z += op.leave_z;
                    }
                    // add tabs to travel boundaries
                    if (tabs.length) {
                        let tab_shadows = tabs.filter(t => t.top >= z).map(t => t.poly);
                        if (tab_shadows) tool_shadow.push(...tab_shadows);
                    }
                    POLY.setWinding(outs, direction === 'climb');
                    // store travel boundary that triggers up and over moves
                    slice.tool_shadow = [ area, ...shadow, ...tool_shadow ];
                    slice.camLines = outs;
                    zroc += zinc;
                    lzo = z;
                    progress(proc + (pinc * zroc), 'clear');
                    // for debugging, output shadow polygons
                    if (devel) layers
                        .setLayer("base", { line: 0xff0000 }, false)
                        .addPolys(shadowBase)
                        .setLayer("shadow", { line: 0x00ff00 }, false)
                        .addPolys(shadow)
                        .setLayer("tool shadow", { line: 0x44ff88 }, false)
                        .addPolys(tool_shadow);
                    layers
                        .setLayer(rename ?? "clear", { line: color }, false)
                        .addPolys(outs);
                    // of the last output still cuts, we need an escape
                    if (z === zs.peek()) {
                        break outer;
                    }
                }
                proc += pinc;
                progress(proc, 'clear');
            } else if (mode === 'adaptive') {
              console.log({
                msg: "adaptive roughing (on-the-fly)",
                zTop, zBottom, down, toolOver, area
              });

              // major Z steps
              const zs = (down ? base_util.lerp(zTop, zBottom, down) : [ bounds.min.z ]).reverse();

              let cleared_area = [];
              // 1. Compute and store the object shadow at each z step.
              let shadows = [];
              for (let i = 0; i < zs.length; i++) {
                const shadow = await shadowAt(zs[i]);
                shadows.push(
                  shadow.map((poly) => {
                    return poly;
                  }));
              }
                
              // 2. Starting at the top and working down, set each layer's
              //    machinable area to the intersection of the computed
              //    machinable area and the machinable area for the slice above.
              //    This represents eliminating any area covered by an
              //    overhang.

              // 3. Compute Z steps based on the step down distance. Then,
              //    consider the height range from the step down to the top.
              zs.forEach( (z, zidx) => {
                //      a) Starting at the bottom of the range, generate a
                //         toolpath that clears the machinable area.
                // get shadow at current Z
                const shadow_now = shadows[zidx];

                // machinable area is the selected area minus the part shadow
                const machinable_now = POLY.subtract([area], shadow_now, [], null, z);
                console.log(`adaptive: z=${z}, machinable_now=${machinable_now.length} polys`);

                // determine new area to clear at this z level
                const to_clear = POLY.subtract(machinable_now, cleared_area, [], null, z);
                console.log(`adaptive: z=${z}, to_clear=${to_clear.length} polys`);

                const to_clear_now = [];
                if (to_clear.length > 0) {
                  const slice = newLayer(z);
                  const layers = slice.output();
                  const firstOff = -(toolDiam / 2 + (op.leave_xy ?? 0));
                  POLY.offset(to_clear, [ firstOff, -toolOver ], {
                    count: op.steps ?? 999, outs: to_clear_now, flat: true, z, ...offopt
                  });
                  console.log(`adaptive: z=${z}, generated ${to_clear_now.length} toolpath polys`);

                  if (to_clear_now.length > 0) {
                    slice.camLines = to_clear_now;
                    // use shadow_now for collision avoidance at this z level
                    slice.tool_shadow = [ area, ...shadow_now ];
                    POLY.setWinding(to_clear_now, direction === 'climb');
                    layers.setLayer("adaptive", { line: color }, false).addPolys(to_clear_now);
                  }
                }

                //      b) Look at the machinable area of the next z-slice up, and
                //         subtract the machinable area from the slice we just
                //         cleared. If there's any area left, generate a toolpath
                //         to clear it. If not, move on to the next step up until
                //         we hit the top of the range.
                // (This is implicitly handled because we're iterating bottom-up, and 'cleared_area' accumulates)
                //      c) step down to the next major step (based on the step
                //         down distance) and repeat (a) and (b) upwards until we
                //         hit the layer we already machined.
                // (The 'zs' loop handles the major steps, and 'cleared_area' handles accumulated machining)
                cleared_area = POLY.union([...cleared_area, ...to_clear_now], 0.001, true, { z });
                console.log(`adaptive: z=${z}, cleared_area=${cleared_area.length} polys`);
              });
              console.log('adaptive: processing complete');


            } else if (mode === 'trace') {
                // 'trace' mode: Follows a line or path
                let { tr_over, tr_type  } = op;
                let zs = down ? base_util.lerp(zTop, op.thru ? zBottom : Math.max(zBottom, area.minZ()), down) : [ bounds.min.z ];
                let zroc = 0;
                let zinc = 1 / zs.length;
                for (let z of zs) {
                    let slice = newLayer(z);
                    let layers = slice.output();
                    let shadow = await shadowAt(z);
                    let outs = [];
                    if (tr_type === 'none') {
                        // 'none' trace type just uses the selected area as the toolpath
                        // todo: move this out of the zs loop and only setZ when needed
                        area = area.clone(true);
                        outs = [ zs.length > 1 || op.thru ? area.setZ(z) : clampZ(area, zTop, zBottom) ];
                    } else {
                        // 'inside' or 'outside' trace type offsets from the selected area
                        // drape is legacy outline
                        let offit = op.drape ? shadow : [ area ];
                        if (op.omitthru && op.drape) {
                            offit = omitMatching(offit, thruHoles);
                        }
                        // todo: move this out of the zs loop
                        let stepping = tr_type === 'inside' ?
                            ( tr_over ? -tr_over : [ -toolDiam / 2, -toolOver ] ) :
                            ( tr_over ? tr_over : [ toolDiam / 2, toolOver ] );
                        POLY.offset(offit, stepping, {
                            count: op.steps ?? 1, outs, flat: true, z, minArea: 0, open: true
                        });
                    }
                    if (outs.length === 0 && !op.drape) {
                        // terminate z descent when no further output possible
                        break;
                    }
                    // add dogbones when specified
                    if (op.dogbones) outs.forEach(out => out.addDogbones(toolDiam / 5, op.revbones));
                    // support legacy outline features
                    if (op.omitouter) {
                        outs = omitOuter(outs);
                    } else if (op.omitinner) {
                        outs = omitInner(outs);
                    }
                    // cut tabs when present
                    if (tabs.length) outs = cutTabs(tabs, outs);
                    slice.camLines = outs;
                    POLY.setWinding(outs, direction === 'climb');
                    // store travel boundary that triggers up and over moves
                    let tool_shadow = slice.tool_shadow = shadow.clone(true);
                    if (area.isOpen()) {
                        tool_shadow.push(...outs[0].clone().setZ(z).offset_open(toolDiam / 2, 'round'));
                    } else {
                        tool_shadow.push(
                            area,
                            ...POLY.offset(shadow, [  ts_off ], { count: 1, z, ...offopt }),
                            ...POLY.offset(shadow, [ -ts_off ], { count: 1, z, ...offopt }),
                        );
                    }
                    // add tabs to travel boundaries
                    if (tabs) {
                        let tab_shadows = tabs.filter(t => t.top >= z).map(t => t.poly);
                        if (tab_shadows) slice.tool_shadow.push(...tab_shadows);
                    }
                    zroc += zinc;
                    progress(proc + (pinc * zroc), 'trace');
                    if (devel) layers
                        .setLayer("base", { line: 0xff0000 }, false)
                        .addPolys(shadowBase)
                        .setLayer("shadow", { line: 0x00ff00 }, false)
                        .addPolys(shadow)
                        .setLayer("tool shadow", { line: 0x44ff88 }, false)
                        .addPolys(tool_shadow);
                    layers
                        .setLayer(rename ?? "trace", { line: color }, false)
                        .addPolys(outs);
                }
                proc += pinc;
                progress(proc, 'trace');
            } else
            if (mode === 'surface') {
                // 'surface' mode: Generates toolpaths that follow the 3D surface of the model.
                let { sr_type, sr_angle, tolerance } = op;

                let resolution = tolerance || 0.05;
                let raster = await self.get_raster_gpu({ mode: "tracing", resolution });
                let surface = [];
                let paths = [];

                // prepare paths based on surface type
                if (sr_type === 'linear') {
                    // 'linear' surface type generates parallel scan lines across the area
                    // scan the area bounding box with rays at defined angle
                    let scan = scanBoxAtAngle(bounds, sr_angle * DEG2RAD, toolOver);
                    let lines = scan.map(line => {
                        let { a, b } = line;
                        return [ newPoint(a.x, a.y, 0).toClipper(), newPoint(b.x, b.y, 0).toClipper() ]
                    });
                    // use clipper to clip lines to the area polygon
                    let clip = new clib.Clipper();
                    let ctre = new clib.PolyTree();
                    clip.AddPaths(lines, ptyp.ptSubject, false);
                    clip.AddPaths(POLY.toClipper([ area ]), ptyp.ptClip, true);
                    if (clip.Execute(ctyp.ctIntersection, ctre, cfil.pftNonZero, cfil.pftEvenOdd)) {
                        for (let node of ctre.m_AllPolys) {
                            paths.push(POLY.fromClipperNode(node, 0));
                        }
                    }
                    // convert resulting poly lines to raster float32 array groups
                    paths = paths.map(poly => poly.points.map(p => [ p.x, p.y ]).flat().toFloat32());
                } else
                if (sr_type === 'offset') {
                    // 'offset' surface type generates paths by progressively offsetting from the perimeter
                    // progressive inset from perimeter
                    POLY.offset([ area ], [ -toolDiam / 2, -toolOver ], {
                        count: 999, outs: paths, flat: true, z: 0, minArea: 0
                    });
                    paths.forEach(poly => poly.isClosed() && poly.push(poly.first()));
                    paths = paths.map(poly => poly.points.map(p => [ p.x, p.y ]).flat().toFloat32());
                }

                // prepare tool mesh points for GPU rastering
                let toolBounds = new THREE.Box3()
                    .expandByPoint({ x: -toolDiam/2, y: -toolDiam/2, z: 0 })
                    .expandByPoint({ x: toolDiam/2, y: toolDiam/2, z: 0 });
                let toolPos = areaTool.generateProfile(resolution).profile.slice();
                for (let i=0; i<toolPos.length; i+= 3) {
                    toolBounds.expandByPoint({ x: toolPos[i], y: toolPos[i+1], z: toolPos[i+2] });
                }
                let toolData = { positions: toolPos, bounds: toolBounds };

                // prepare terrain and raster paths over terrain using WebGPU
                let vertices = widget.getGeoVertices({ unroll: true, translate: true });
                let wbounds = bounds.clone().expandByVector({ x: toolDiam/2, y: toolDiam/2, z: 0 });
                wbounds.min.z = zBottom;
                wbounds.max.z = zTop;
                await raster.loadTool({
                    sparseData: toolData
                });
                await raster.loadTerrain({
                    triangles: vertices,
                    boundsOverride: wbounds
                });
                if (paths.length === 0) {
                    // skip raster if no output generated
                    continue;
                }
                let output = await raster.generateToolpaths({
                    paths,
                    step: toolOver / 2,
                    zFloor: zBottom - 1,
                    onProgress(pct) { console.log({ pct }); onupdate(pct/100, 100) }
                });
                raster.terminate();

                // convert terrain raster output back to open polylines
                for (let path of output.paths) {
                    path = newPolygon().fromArray([1, ...path]);
                    if (op.refine) path.refine(op.refine);
                    surface.push(path);
                    let slice = newLayer();
                    slice.camLines = [ path ];
                    slice.output()
                        .setLayer(rename ?? "linear", { line: color }, false)
                        .addPolys([ path ]);
                }

                // output this surface
                surfaces.push(surface);
            }
        }
        // filter out empty slices
        this.areas = areas = areas.map(area => {
            return area.filter(slice => slice.camLines && slice.camLines.length);
        }).filter(a => a.length);

        // only render slices containing ares to mill
        addSlices(areas.flat().filter(s => s.camLines && s.camLines.length));
    }

    /**
     * The prepare method is responsible for converting the generated slices into
     * a sequence of tool movements (G-code like instructions).
     *
     * @param {object} ops - A collection of output functions (e.g., pocket, polyEmit)
     * @param {function} progress - Function to report progress
     */
    prepare(ops, progress) {
        let { op, state, areas, surfaces } = this;
        let { newLayer, pocket, polyEmit, printPoint, tip2tipEmit } = ops;
        let { setContouring, setNextIsMove } = ops;
        let { process } = state.settings;

        // process surface paths first if they exist
        if (surfaces.length) {
            setContouring(true);
            for (let surface of surfaces) {
                let array = surface.map(poly => { return {
                    el: poly,
                    first: poly.first(),
                    last: poly.last()
                } });
                // emit toolpaths from tip-to-tip for efficiency
                tip2tipEmit(array, printPoint, (next, point) => {
                    setNextIsMove();
                    if (next.last === point) next.el.reverse();
                    printPoint = polyEmit(next.el);
                    newLayer();
                });
            }
            setContouring(false);
            // skip area processing when surface processing is done
            return;
        }

        // process areas as pockets, finding the closest one to the current tool position
        while (areas?.length) {
            let min = {
                dist: Infinity,
                area: undefined
            };

            for (let area of areas.filter(p => !p.used)) {
                // skip devel / debug only areas
                let topPolys = area[0].camLines;
                if (!topPolys) continue;
                // select poly with largest area as representative for the area
                let poly = topPolys.slice().sort((a,b) => b.area() - a.area())[0];
                if (!poly) continue;
                // compute move distance to top poly for efficient routing
                let find = poly.findClosestPointTo(printPoint);
                if (find.distance < min.dist) {
                    min.area = area;
                    min.dist = find.distance;
                }
            }

            // if we have a next-closest top poly, pocket that area
            if (min.area) {
                min.area.used = true;
                pocket({
                    cutdir: op.ov_conv, // conventional or climb milling
                    depthFirst: process.camDepthFirst && !op.drape, // depth-first cutting
                    easeDown: op.down && process.easeDown ? op.down : 0, // ease down into cuts
                    progress: (n,m) => progress(n/m, "area"),
                    slices: min.area.filter(slice => slice.camLines) // slices to process
                });
            } else {
                // no more areas to process
                break;
            }
        }
    }
}

/**
 * Omits outer polygons, returning only inner polygons.
 * @param {Polygon[]} polys
 * @returns {Polygon[]}
 */
function omitOuter(polys) {
    let inner = [];
    for (let poly of polys) {
        if (poly.inner) inner.push(...poly.inner);
    }
    return inner;
}

/**
 * Omits inner polygons from a set of polygons.
 * @param {Polygon[]} polys
 * @returns {Polygon[]}
 */
function omitInner(polys) {
    for (let poly of polys) {
        poly.inner = undefined;
    }
    return polys;
}

/**
 * Omits polygons from a target set that are equivalent to polygons in a matches set.
 * @param {Polygon[]} target
 * @param {Polygon[]} matches
 * @returns {Polygon[]}
 */
function omitMatching(target, matches) {
    target = target.clone(true);
    for (let poly of target.filter(p => p.inner)) {
        poly.inner = poly.inner.filter(inner => {
            for (let ho of matches) {
                if (inner.isEquivalent(ho)) {
                    return false;
                }
            }
            return true;
        });
    }
    return target;
}

/**
 * Clamps the Z values of a polygon's points between a min and max.
 * @param {Polygon} poly
 * @param {number} max
 * @param {number} min
 * @returns {Polygon}
 */
function clampZ(poly, max, min) {
    for (let p of poly.points) {
        if (p.z < min) p.z = min;
        else if (p.z > max) p.z = max;
    }
    if (poly.inner) {
        for (let p of poly.inner) {
            clampZ(p, max, min);
        }
    }
    return poly;
}

/**
 * Generates scan lines (rays) across a bounding box at a specified angle and step.
 * @param {THREE.Box2} box2
 * @param {number} angle - in radians
 * @param {number} step - spacing between rays
 * @returns {object[]} array of {a, b} vectors representing lines
 */
function scanBoxAtAngle(box2, angle, step) {
    const cx = (box2.min.x + box2.max.x) * 0.5;
    const cy = (box2.min.y + box2.max.y) * 0.5;
    const w = box2.max.x - box2.min.x;
    const h = box2.max.y - box2.min.y;

    // ray direction
    const dx = Math.cos(angle);
    const dy = Math.sin(angle);

    // normal between rays (perpendicular to ray dir)
    const nx = -dy;
    const ny = dx;

    // extent of the box along the normal
    const extentN = Math.abs(nx) * w + Math.abs(ny) * h;

    // length of each ray across the box (along ray dir)
    const extentD = Math.abs(dx) * w + Math.abs(dy) * h;

    // how many rays to cover the box; +1 so edges are covered
    const count = Math.max(1, Math.ceil(extentN / step) + 1);

    const halfSpan = step * (count - 1) * 0.5;
    const halfD = extentD * 0.5;
    const rays = [];

    for (let i = 0; i < count; i++) {
        // offset along normal
        const o = -halfSpan + i * step;
        const ox = cx + nx * o;
        const oy = cy + ny * o;

        // segment endpoints for this ray inside (or slightly outside) the box
        const ax = ox - dx * halfD;
        const ay = oy - dy * halfD;
        const bx = ox + dx * halfD;
        const by = oy + dy * halfD;

        rays.push({
            a: new THREE.Vector2(ax, ay),
            b: new THREE.Vector2(bx, by),
        });
    }

    return rays;
}

export { OpArea };
