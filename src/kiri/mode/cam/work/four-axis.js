/** Copyright Stewart Allen <sa@grid.space> -- All Rights Reserved */

import { base } from '../../../../geo/base.js';
import { codec } from '../../../core/codec.js';
import { newPoint } from '../../../../geo/point.js';
import { newPolygon } from '../../../../geo/polygon.js';
import { sliceConnect } from '../../../../geo/slicer.js';
import { newSlice } from '../../../core/slice.js';
import { Tool } from '../core/tool.js';
import { Slicer as topo_slicer } from './slicer-topo.js';

const RAD2DEG = 180 / Math.PI;
const DEG2RAD = Math.PI / 180;

function scale(fn, factor = 1, base = 0) {
    return (value, msg) => {
        fn(base + value * factor, msg);
    }
}

export class Topo {
    constructor() { }

    async generate(opt = {}) {
        let { state, op, onupdate, ondone } = opt;
        let { widget, settings, tabs, color } = state;
        let { controller, process } = settings;
        let { webGPU } = controller;

        /**
         * ALGORITHM STEP 1: Slicing Normal to the Rotation Axis
         * The model is oriented along the machine's rotation axis (X).
         * Intersect the model with a series of planes perpendicular to X.
         */
        let axis = op.axis.toLowerCase(),
            tool = new Tool(settings, op.tool),
            bounds = widget.getBoundingBox().clone(),
            density = parseInt(controller.animesh || 100) * 2500,
            { min, max } = bounds,
            span = {
                x: max.x - min.x,
                y: max.y - min.y
            },
            contour = {
                x: axis === "x",
                y: axis === "y"
            },
            tolerance = op.tolerance,
            resolution = (tolerance ? tolerance : 1 / Math.sqrt(density / (span.x * span.y))).round(5),
            step = this.step = (tool.traceOffset() * 2) * op.step,
            angle = this.angle = op.angle || 1,
            down = this.down = op.down,
            expand = this.expand = op.expand;

        if (tool.isTaperMill() && step === 0) {
            step = this.step = op.step * tool.unitScale();
        }

        if (tolerance === 0) {
            console.log(widget.id, 'topo4 auto tolerance', resolution.round(4));
        }

        this.zBottom = state.zBottom ?? 0;
        this.resolution = resolution;
        this.vertices = widget.getGeoVertices({ unroll: true, translate: true }).slice();
        this.tabverts = widget.getTabVertices();
        this.tool = tool.generateProfile(resolution).profile;
        this.maxo = tool.profileDim.maxo * resolution;
        this.diam = tool.fluteDiameter();
        this.unit = tool.unitScale();
        this.units = controller.units === 'in' ? 25.4 : 1
        this.zoff = widget.track.top || 0;
        this.leave = op.leave || 0;
        this.linear = false;
        this.lineColor = color;//controller.dark ? 0xffff00 : 0x555500;
        this.offStart = op.offStart ?? 0;
        this.offEnd = op.offEnd ?? 0;
        this.bounds  = bounds;
        this.gpu = webGPU ?? false;

        onupdate(0, "four-axis");

        const parts = webGPU ? [ 0.8, 0.2 ] : [ 0.25, 0.75 ];
        const range = this.range = { min: Infinity, max: -Infinity };
        const slices = this.sliced = await this.slice(scale(onupdate, parts[0], 0));

        for (let slice of slices) {
            range.min = Math.min(range.min, slice.z);
            range.max = Math.max(range.max, slice.z);
        }

        // ALGORITHM STEP 2: Accessibility Analysis (C-Space Mapping)
        // For each point on the slice contours, determine the 'feasible orientation interval'.
        // 1. Build a machinability map for each slice from original contours.
        // 2. For each resampled point, sample 360 degrees in 5-degree increments.
        // 3. Check for upward (machine +Z) collisions after rotation.
        const resampleDist = resolution * 2; // N units
        const resampledSlices = this.slices = [];
        console.log(`topo4 generate | slices=${slices.length} resampleDist=${resampleDist}`);

        const machinabilityMap = slices.map(slice => {
            const polys = (slice.tops ? slice.tops.map(t => t.poly) : slice.camLines) || [];
            return new Machinability(polys, resolution, this.diam);
        });

        let sliceCount = 0;
        for (let slice of slices) {
            const isEvery100 = (sliceCount % 100 === 0);
            const mach = machinabilityMap[sliceCount++];

            const resampledSlice = newSlice(slice.z);
            resampledSlice.index = slice.index;
            const resampledPolys = [];
            // use slice.tops or slice.camLines
            const polys = (slice.tops ? slice.tops.map(t => t.poly) : slice.camLines) || [];
            let pIn = 0, pOut = 0;
            for (let poly of polys) {
                const points = poly.points || (poly.id ? undefined : poly.array);
                if (points) pIn += (points.length / (poly.points ? 1 : 3));

                const resampled = resamplePolygon(poly, resampleDist);
                if (resampled) {
                    resampledPolys.push(resampled);
                    for (let p of resampled.points) {
                        p.mdr = mach.getMDRs(p);
                    }
                    pOut += resampled.points.length;
                }
            }
            if (resampledPolys.length > 0) {
                if (isEvery100) console.log(`  slice z=${slice.z.round(2)} index=${slice.index} pIn=${pIn} pOut=${pOut}`);
                resampledSlice.addTops(resampledPolys);
                resampledSlice.camLines = resampledPolys;
                if (isEvery100) {
                    const layers = resampledSlice.output();
                    layers.setLayer("fouraxis-contours", { line: 0x00ff00 })
                        .addPolys(resampledPolys, { thin: true });

                    const mdrPolys = [];
                    let pCount = 0;
                    for (let p of resampledPolys.map(poly => poly.points).flat()) {
                        if (pCount++ % 10 !== 0) continue;
                        if (p.mdr) {
                            for (let angle of p.mdr) {
                                const rad = angle * DEG2RAD;
                                // Machine UP in part space for CCW rotation 'a' around X:
                                // dY = -sin(a), dZ = cos(a)
                                const dy = -Math.sin(rad);
                                const dz = Math.cos(rad);
                                mdrPolys.push(newPolygon([
                                    p,
                                    newPoint(p.x, p.y + dy, p.z + dz)
                                ]).setOpen());
                            }
                        }
                    }
                    if (mdrPolys.length > 0) {
                        layers.setLayer("fouraxis-machinable", { line: 0xff0000 })
                            .addPolys(mdrPolys, { thin: true });
                    }
                }
                resampledSlices.push(resampledSlice);
            }
        }
        console.log(`topo4 generate | resampledSlices=${resampledSlices.length}`);

        onupdate(1, "four-axis");
        ondone(resampledSlices);

        return this;
    }

    async slice(onupdate) {
        const { vertices, resolution, tabverts, zoff, offStart, offEnd, units } = this;
        const { minions } = self.kiri_worker;
        const range = this.range = { min: Infinity, max: -Infinity };

        // swap XZ in shared array
        for (let i = 0, l = vertices.length; i < l; i += 3) {
            const x = vertices[i];
            const z = vertices[i + 2] + zoff;
            vertices[i] = z;
            vertices[i + 2] = x;
            range.min = Math.min(range.min, x);
            range.max = Math.max(range.max, x);
        }

        // add tool diameter to slice min/max range to fully carve part
        range.min += offStart * units;
        range.max -= offEnd * units;
        range.max += resolution * 2;

        // merge in tab vertices here so they don't affect slice range / dimensions
        for (let i = 0, l = tabverts.length; i < l; i += 3) {
            const x = tabverts[i];
            const z = tabverts[i + 2] + zoff;
            tabverts[i] = z;
            tabverts[i + 2] = x;
        }

        // re-create shared vertex array for workers
        const v2 = new Float32Array(vertices.length + tabverts.length);
        v2.set(vertices);
        if (tabverts.length > 0) {
            v2.set(tabverts, vertices.length);
        }
        this.vertices = v2.toShared();

        // rp.rasterizeMesh(vertices, resolution).then(rpo => console.log({ rpo }));
        if (this.gpu) {
            return await this.sliceGPU(onupdate);
        }

        const zSpan = range.max - range.min;
        const shards = Math.ceil(Math.min(25, vertices.length / 27000));
        const totalSteps = Math.ceil(zSpan / resolution);
        const stardSteps = Math.ceil(totalSteps / shards);
        const stepWidth = stardSteps * resolution;

        console.log(`topo4 slice | range=${range.min.round(2)}-${range.max.round(2)} shards=${shards} totalSteps=${totalSteps} stardSteps=${stardSteps}`);
        let slices = this.slices = [];
        for (let i=0; i<shards; i++) {
            let s;
            slices.push(s = {
                min: range.min + i * stepWidth,
                max: Math.min(range.max, range.min + (i+1) * stepWidth),
                index: i * stardSteps
            });
            // console.log(`  shard ${i} min=${s.min.round(2)} max=${s.max.round(2)} index=${s.index}`);
        }

        // console.log({
        //     shards,
        //     step: stepWidth,
        //     range,
        //     resolution,
        //     slices: slices.slice(),
        //     minions
        // });

        if (minions?.running > 1) {
            return await this.sliceMinions(onupdate);
        } else {
            return await this.sliceWorker(onupdate);
        }
    }

    async sliceGPU(onupdate) {
        const { angle, diam, leave, linear, offStart, offEnd, resolution, tool, units, vertices, zBottom } = this;
        const { down, expand } = this;

        // invert tool Z offset for gpu code
        let toolBounds = new THREE.Box3()
            .expandByPoint({ x: -this.diam/2, y: -diam/2, z: 0 })
            .expandByPoint({ x: this.diam/2, y: diam/2, z: 0 });
        let toolPos = tool.slice();
        let minz = Infinity;
        for (let i=0; i<toolPos.length; i += 3) {
            // toolPos[i+2] = -toolPos[i+2];
            toolBounds.expandByPoint({ x: 0, y: 0, z: toolPos[i+2] });
            minz = Math.min(minz,toolPos[i+2]);
        }
        let toolData = { positions: toolPos, bounds: toolBounds };

        // console.time('swap XZ vertices');
        // swap XZ vertices for gpu code
        for (let i=0; i<vertices.length; i+= 3) {
            let tmp = vertices[i+2];
            vertices[i+2] = vertices[i+0];
            vertices[i+1] = -vertices[i+1];
            vertices[i+0] = tmp;
        }
        // console.timeEnd('swap XZ vertices');

        let gpu = await self.get_raster_gpu({
            mode: "radial",
            resolution,
            rotationStep: angle,
            radialRotationOffset: 90
        });
        let xStep = Math.max(1, Math.round(this.step / resolution));
        let boundsOverride = this.bounds.clone();
        boundsOverride.min.x += offStart * units;
        boundsOverride.max.x -= offEnd * units;
        await gpu.loadTool({
            sparseData: toolData
        });
        await gpu.loadTerrain({
            triangles: vertices,
            boundsOverride,
            onProgress: (pct) => onupdate(pct / 1000)
        });
        let output = await gpu.generateToolpaths({
            xStep,
            yStep: 1,
            zFloor: zBottom,
            onProgress: (i,j) => onupdate(0.1 + i / j)
        });
        gpu.terminate();

        let { numStrips, strips } = output;
        let degPerRow = 360 / numStrips;
        let slices = this.gpu_slices = [];
        let xmult = resolution * xStep;
        let xoff = boundsOverride.min.x;
        let rows = [];
        for (let i=0; i<numStrips; i++) {
            let points = Array.from(strips[i].pathData).map((v,j) => newPoint(j * xmult + xoff, 0, v + leave).setA(-i * degPerRow));
            rows.push(points);
        }
        if (linear) {
            for (let i=0; i<rows.length; i++) {
                let slice = newSlice(i);
                slice.index = i;
                slice.camLines = [ newPolygon(rows[i]).setOpen() ];
                if (!down && i % 2 === 1) slice.camLines[0].reverse();
                slices.push(slice);
            }
        } else {
            let { pointsPerLine } = strips[0];
            for (let i=0; i<pointsPerLine; i++) {
                let slice = newSlice(i);
                let points = rows.map(row => row[i]);
                points.push(points[0].clone().setA(-360));
                if (!down && i % 2 === 1) points.reverse();
                slice.index = i;
                slice.camLines = [ newPolygon(points).setOpen() ];
                slices.push(slice);
            }
        }
        let maxZ = 0;
        if (down) {
            for (let r of rows)
            for (let p of r) {
                maxZ = Math.max(maxZ, p.z);
            }
            maxZ += expand;
        }
        if (down) {
            // step "up" points on trace until they max out
            // then reverse the stack to cut top down
            let reverse = 0;
            for (let slice of slices) {
                let z = slice.z;
                let poly = slice.camLines[0].clone();
                let pMinZ = poly.minZ();
                let steps = Math.floor((maxZ - pMinZ) / down);
                let stack = [ poly.clone() ];
                for (let i=0; i<steps; i++) {
                    for (let p of poly.points) {
                        p.z = Math.min(p.z + down, maxZ);
                    }
                    stack.push(poly.clone());
                }
                stack.reverse();
                slice.stack = stack.map(poly => {
                    if (reverse++ % 2 === 1) poly.reverse();
                    let slice = newSlice(z);
                    slice.camLines = [ poly ];
                    return slice;
                });
                // first move of new stack is to maxZ (problematic)
                // let first = stack[0].points;
                // first.splice(0,0,first[0].clone().setZ(maxZ));
                // last move of stack is to maxZ
                let last = stack.peek();
                last.push(last.last().clone().setZ(maxZ));
            }
            slices = this.gpu_slices = slices.map(slice => slice.stack).flat();
            slices.forEach((slice,index) => slice.index = index);
        }
        for (let slice of slices) {
            slice.output()
                .setLayer("four-axis", { line: this.lineColor })
                .addPoly(slice.camLines[0].clone().applyRotations());
        }
        // console.log({ webGPU: output, slices });
        return slices;
    }

    async sliceWorker(onupdate) {
        const { vertices, slices, resolution } = this;

        let output = [];
        let complete = 0;
        for (let slice of slices) {
            const recs = new topo_slicer(slice.index)
                .setFromArray(vertices, slice)
                .slice(resolution)
                .map(rec => {
                    const slice = newSlice(rec.z);
                    slice.index = rec.index;
                    const polys = sliceConnect(rec.lines);
                    for (let line of rec.lines) {
                        const { p1, p2 } = line;
                        if (!p1.swapped) { p1.swapXZ(); p1.swapped = true }
                        if (!p2.swapped) { p2.swapXZ(); p2.swapped = true }
                    }
                    for (let poly of polys) {
                        for (let p of poly.points) {
                            if (!p.swapped) { p.swapXZ(); p.swapped = true }
                        }
                    }
                    slice.addTops(polys);

                    const points = codec.encodePointArray(rec.lines.map(l => [l.p1, l.p2]).flat());
                    const shared = new Float32Array(new SharedArrayBuffer(points.length * 4));
                    shared.set(points);
                    slice.shared = shared;

                    return slice;
                });
            output.appendAll(recs);
            onupdate(++complete / slices.length);
        }

        return output;
    }

    async sliceMinions(onupdate) {
        const { queue, putCache, clearCache } = this;
        const { vertices, slices, resolution } = this;
        putCache("vertices", vertices);

        let complete = 0;
        let promises = slices.map(slice => {
            return queue("topo4_slice", {
                resolution,
                slice
            }).then(data => {
                onupdate(++complete / slices.length);
                return data;
            });
        });

        // merge boxes for all rasters for contouring clipping
        const output = codec.decode(await Promise.all(promises))
            .map(rec => rec.recs)
            .flat()
            .map(rec => newSlice(rec.z)
                .addTops(rec.polys)
                .setFields({ shared: rec.shared }))
            .sort((a, b) => a.z - b.z);

        clearCache();
        return output;
    }

    async fourAxis(onupdate) {
        /**
         * ALGORITHM STEP 2: Accessibility Analysis (C-Space Mapping)
         * For each point on the slice contours, determine the 'feasible orientation interval'.
         * Map the part geometry into tool Configuration Space to identify collision-free
         * rotation angles (\theta) where the tool can contact the target without gouging.
         *
         * ALGORITHM STEP 3: Continuous Path Optimization
         * Construct a graph where nodes are feasible orientations at each point.
         * Use Dynamic Programming or a shortest-path algorithm to select orientations
         * that minimize angular jumps, ensuring kinematic continuity and smoothness.
         */
        const { minions } = self.kiri_worker;
        if (this.gpu) {
            return await this.fourAxisGPU(onupdate);
        } else if (minions?.running > 1) {
            return await this.fourAxisMinions(onupdate);
        } else {
            return await this.fourAxisWorker(onupdate);
        }
    }

    async fourAxisGPU(onupdate) {
        return this.gpu_slices;
    }

    fourAxisPath(slices, tool) {
        const { resolution, step, zBottom, maxo } = this;

        const tlen = tool.length;
        const slen = slices.length;
        const heights = [];

        // console.log({ tlen, slen, sinc, slices: slices.map(s => s.z) });
        // cull slice lines to only the ones in range (~5x faster)
        const oslices = [];
        for (let slice of slices) {
            const lines = slice.lines;
            const plen = lines.length;
            const rec = { z: slice.z, lines: [] };
            for (let i = 0; i < plen;) {
                ++i; // skip x which should match slice.z
                let py0 = lines[i++];
                const pz0 = lines[i++];
                ++i; // skip x which should match slice.z
                let py1 = lines[i++];
                const pz1 = lines[i++];
                if ((py0 < -maxo && py1 < -maxo) || (py0 > maxo && py1 > maxo)) {
                    continue;
                }
                rec.lines.push(0, py0, pz0, 0, py1, pz1);
            }
            oslices.push(rec);
        }

        // iterate over all slices (real x = z)
        // find max real z using z ray intersect from tool point to slice lines + offset
        let lz = 0;
        for (let sz = 0; ; sz += step) {
            let si = Math.ceil(sz / resolution);
            if (si >= oslices.length) break;
            const rx = oslices[si].z;
            let mz = -Infinity;
            // iterate over tool offsets
            for (let ti = 0; ti < tlen;) {
                // tool offset in grid units from present x (si)
                const xo = tool[ti++]; // x grid offset (slice)
                const yo = tool[ti++] * resolution; // y grid offset (mult rez to get real y)
                const zo = tool[ti++]; // real z delta offset
                // get slice index corresponding with offset
                const ts = si + xo;
                // outside of slice array, skip
                if (ts < 0 || ts >= slen - 1) {
                    continue;
                }
                const slice = oslices[ts];
                const lines = slice.lines;
                const plen = lines.length;
                for (let i = 0; i < plen;) {
                    ++i; // skip x which should match slice.z
                    let py0 = lines[i++];
                    const pz0 = lines[i++];
                    ++i; // skip x which should match slice.z
                    let py1 = lines[i++];
                    const pz1 = lines[i++];
                    if ((py0 <= yo && py1 >= yo) || (py1 <= yo && py0 >= yo)) {
                        const dz = pz1 - pz0;
                        const dy = Math.abs(py1 - py0);
                        if (dy === 0) continue;
                        const fr = Math.abs(yo - py0) / dy;
                        const lz = pz0 + dz * fr + zo;
                        // check z height
                        mz = Math.max(mz, lz);
                    }
                }
                if (mz === -Infinity && xo === 0 && yo === 0) {
                    // tool tip is off the model
                    // continue;
                }
            }
            if (mz === -Infinity) {
                mz = zBottom;
            } else if (mz < zBottom) {
                mz = zBottom;
            }
            heights.push(rx, 0, lz = mz);
        }

        return heights;
    }

    async fourAxisWorker(onupdate) {
        const { sliced, tool, zoff, leave, linear } = this;

        const rota = this.angle * DEG2RAD;
        const steps = (Math.PI * 2) / rota;
        const axis = new THREE.Vector3(1, 0, 0);
        const mrot = new THREE.Matrix4().makeRotationAxis(axis, rota);
        const slices = sliced.map(s => { return { z: s.z, lines: s.shared } });
        const paths = [];
        const recs = [];

        let angle = 0;
        let count = 0;
        // for each step angle, find Z spine heights, produce record
        while (count++ < steps) {
            const heights = this.fourAxisPath(slices, tool, paths);
            recs.push({ angle, heights, degrees: angle * RAD2DEG });
            angle -= rota;
            for (let lines of slices.map(s => s.lines)) {
                rotatePoints(lines, mrot);
            }
            onupdate(count / steps);
        }

        count = linear ? recs.length : recs[0].heights.length / 3;
        // count = recs[0].heights.length / 3;
        while (count-- > 0) {
            let slice = newSlice(count);
            slice.camLines = [newPolygon().setOpen()];
            paths.push(slice);
        }

        if (linear) {
            recs.forEach((rec, i) => {
                const { degrees, heights } = rec;
                [...heights].group(3).forEach((a) => {
                    paths[i].camLines[0].push(newPoint(a[0], a[1], a[2] + leave).setA(degrees));
                });
                if (i % 2 === 1) {
                    paths[i].camLines[0].reverse();
                }
            });
        } else {
            for (let rec of recs) {
                const { degrees, heights } = rec;
                [...heights].group(3).forEach((a, i) => {
                    // progress each path 360 degrees to prevent A rolling backwards
                    paths[i].camLines[0].push(newPoint(a[0], a[1], a[2] + leave).setA(degrees + i * -360));
                });
            }
        }

        for (let slice of paths) {
            const poly = slice.camLines[0];
            if (!poly.length) {
                console.log('empty', slice);
                continue;
            }
            // repeat first point 360 degrees progressed
            const repeat = poly.points[0];
            slice.camLines[0].push(repeat.clone().setA(repeat.a - 360));
            slice.output()
                .setLayer("four-axis", { line: this.lineColor })
                .addPoly(poly.clone().applyRotations().move({ z: -zoff, x: 0, y: 0 }));
        }

        // console.log({ tool, slices, paths });

        return paths;
    }

    async fourAxisMinions(onupdate) {
        const { sliced, tool, zoff, leave, maxo, zBottom, step, resolution, linear } = this;
        const { putCache, clearCache, queue } = this;

        const rota = this.angle * DEG2RAD;
        const steps = (Math.PI * 2) / rota;
        const slices = sliced.map(s => { return { z: s.z, lines: s.shared } });
        const paths = [];
        const recs = [];

        putCache("four-axis", {
            maxo,
            tool,
            step,
            slices,
            zBottom,
            resolution,
        });

        let done = 0;
        let tangle = 0;
        let count = 0;
        let promises = [];
        // for each step angle, find Z spine heights, produce record
        while (count++ < steps) {
            const angle = tangle;
            let p = new Promise(resolve => {
                queue("topo4_four-axis", { angle }).then(data => {
                    // console.log({ angle, data });
                    recs.push({ angle, heights: data.heights, degrees: angle * RAD2DEG });
                    onupdate(++done / steps);
                    resolve();
                });
            });
            // await p;
            promises.push(p);
            tangle -= rota;
        }

        await Promise.all(promises);
        recs.sort((a, b) => { return b.angle - a.angle });

        count = linear ? recs.length : recs[0].heights.length / 3;
        while (count-- > 0) {
            let slice = newSlice(count);
            slice.camLines = [newPolygon().setOpen()];
            paths.push(slice);
        }

        if (linear) {
            recs.forEach((rec, i) => {
                const { degrees, heights } = rec;
                [...heights].group(3).forEach((a) => {
                    paths[i].camLines[0].push(newPoint(a[0], a[1], a[2] + leave).setA(degrees));
                });
                if (i % 2 === 1) {
                    paths[i].camLines[0].reverse();
                }
            });
        } else {
            for (let rec of recs) {
                const { degrees, heights } = rec;
                [...heights].group(3).forEach((a, i) => {
                    // progress each path 360 degrees to prevent A rolling backwards
                    paths[i].camLines[0].push(newPoint(a[0], a[1], a[2] + leave).setA(degrees + i * -360));
                });
            }
        }

        for (let slice of paths) {
            const poly = slice.camLines[0];
            if (!poly.length) {
                console.log('empty', slice);
                continue;
            }
            if (!linear) {
                // repeat first point 360 degrees progressed
                const repeat = poly.points[0];
                slice.camLines[0].push(repeat.clone().setA(repeat.a - 360));
            }
            slice.output()
                .setLayer("four-axis", { line: this.lineColor })
                .addPoly(poly.clone().applyRotations().move({ z: -zoff, x: 0, y: 0 }));
        }

        // console.log({ tool, slices, paths });
        clearCache();

        return paths;
    }

    putCache(key, data) {
        const { dispatch } = self.kiri_worker;
        dispatch.putCache({ key, data }, { done: data => { } });
    }

    clearCache() {
        const { dispatch } = self.kiri_worker;
        dispatch.clearCache({}, { done: data => { } });
    }

    queue(cmd, params) {
        const { minions } = self.kiri_worker;
        return new Promise(resolve => {
            minions.queue({ cmd, ...params }, resolve);
        });
    }
}

export function rotatePoints(lines, rot) {
    new THREE.BufferAttribute(lines, 3).applyMatrix4(rot);
}

export async function generate(opt) {
    return new Topo().generate(opt);
};

function resamplePolygon(poly, dist) {
    if (!poly) return undefined;
    const points = poly.points || (poly.id ? undefined : poly.array); // handle different point formats
    if (!points || (points.length < 2 && !Array.isArray(points))) return undefined;

    // convert to proper point array if it's a flat float array from decoder
    let pts = points;
    if (points instanceof Float32Array || (Array.isArray(points) && typeof points[0] === 'number')) {
        pts = [];
        for (let i=0; i<points.length; i += 3) {
            pts.push(newPoint(points[i], points[i+1], points[i+2]));
        }
    }

    if (pts.length < 2) return undefined;

    const newPoints = [];
    const len = pts.length;
    const isOpen = poly.open || (poly.isOpen ? poly.isOpen() : false);

    for (let i = 0; i < len; i++) {
        const p1 = pts[i];
        const p2 = pts[(i + 1) % len];
        newPoints.push(p1);
        if (isOpen && i === len - 1) continue;
        const d = p1.distTo3D(p2);
        if (d > dist) {
            const steps = Math.floor(d / dist);
            for (let j = 1; j <= steps; j++) {
                const ratio = j / (steps + 1);
                newPoints.push(newPoint(
                    p1.x + (p2.x - p1.x) * ratio,
                    p1.y + (p2.y - p1.y) * ratio,
                    p1.z + (p2.z - p1.z) * ratio
                ));
            }
        }
    }
    return newPolygon(newPoints).setOpen(isOpen);
}

class Machinability {
    constructor(polys, resolution, toolWidth = 0) {
        this.toolWidth = toolWidth;
        this.polys = polys.map(p => {
            if (p.id) return p;
            // handle decoded plain objects
            const points = p.points || p.array;
            if (points instanceof Float32Array || (Array.isArray(points) && typeof points[0] === 'number')) {
                const pts = [];
                for (let i=0; i<points.length; i += 3) {
                    pts.push(newPoint(points[i], points[i+1], points[i+2]));
                }
                return newPolygon(pts).setOpen(p.open);
            }
            return p;
        });

        // The slicer plane for a fixed X position is the (Y, Z) plane.
        // In our points: p.x = X, p.y = Y, p.z = Z.
        // So we use (p.z, p.y) as our 2D plane for intersection tests.
        const segments = this.segments = [];
        const bounds = new THREE.Box2();
        for (let poly of this.polys) {
            poly.forEachSegment((p1, p2) => {
                const seg = {
                    p1, p2,
                    // Use Z as 'x' and Y as 'y' for the spatial grid
                    minX: Math.min(p1.z, p2.z),
                    maxX: Math.max(p1.z, p2.z),
                    minY: Math.min(p1.y, p2.y),
                    maxY: Math.max(p1.y, p2.y)
                };
                segments.push(seg);
                bounds.expandByPoint({ x: seg.minX, y: seg.minY });
                bounds.expandByPoint({ x: seg.maxX, y: seg.maxY });
            });
        }

        // build simple spatial grid
        const gridRes = resolution * 20;
        const grid = this.grid = {};
        this.gridRes = gridRes;
        this.bounds = bounds;

        for (let seg of segments) {
            const x1 = Math.floor(seg.minX / gridRes);
            const x2 = Math.floor(seg.maxX / gridRes);
            const y1 = Math.floor(seg.minY / gridRes);
            const y2 = Math.floor(seg.maxY / gridRes);
            for (let x = x1; x <= x2; x++) {
                for (let y = y1; y <= y2; y++) {
                    const key = `${x},${y}`;
                    const cell = grid[key] = grid[key] || [];
                    cell.push(seg);
                }
            }
        }
    }

    /**
     * @param {Point} p Point in slice plane (YZ)
     * @returns {number[]} Array of angles (0-355) that are machinable
     */
    getMDRs(p) {
        const mdrs = [];
        for (let a = 0; a < 360; a += 5) {
            if (this.isMachinable(p, a)) {
                mdrs.push(a);
            }
        }
        return mdrs;
    }

    /**
     * @param {Point} origin
     * @param {number} dz
     * @param {number} dy
     * @param {number} startOff
     * @returns {boolean}
     */
    castRay(origin, dz, dy, startOff = 0) {
        const far = { x: origin.x + dz * 1000, y: origin.y + dy * 1000 };
        const rayMinX = Math.min(origin.x, far.x);
        const rayMaxX = Math.max(origin.x, far.x);
        const rayMinY = Math.min(origin.y, far.y);
        const rayMaxY = Math.max(origin.y, far.y);

        const keys = base.key;
        const gridRes = this.gridRes;
        const bounds = this.bounds;

        const checked = new Set();
        const distToEdge = 1000;
        const steps = Math.ceil(distToEdge / (gridRes / 5));
        
        for (let i = 0; i <= steps; i++) {
            const dist = i * gridRes / 5;
            const tx = origin.x + dz * dist;
            const ty = origin.y + dy * dist;
            
            if (tx < bounds.min.x && dz < 0) break;
            if (tx > bounds.max.x && dz > 0) break;
            if (ty < bounds.min.y && dy < 0) break;
            if (ty > bounds.max.y && dy > 0) break;

            const gx = Math.floor(tx / gridRes);
            const gy = Math.floor(ty / gridRes);
            const key = `${gx},${gy}`;
            
            if (checked.has(key)) continue;
            checked.add(key);
            
            const cell = this.grid[key];
            if (cell) {
                for (let seg of cell) {
                    if (rayMaxX < seg.minX || rayMinX > seg.maxX ||
                        rayMaxY < seg.minY || rayMinY > seg.maxY) {
                        continue;
                    }

                    const s1 = { x: seg.p1.z, y: seg.p1.y };
                    const s2 = { x: seg.p2.z, y: seg.p2.y };
                    const int = base.util.intersect(origin, far, s1, s2, keys.SEGINT);
                    if (int && (int.dist * 1000) > (0.001 + startOff)) {
                        return false;
                    }
                }
            }
        }

        return true;
    }

    /**
     * @param {Point} p Point in slice plane (YZ)
     * @param {number} angle Rotation angle in degrees
     * @returns {boolean} True if machinable
     */
    isMachinable(p, angle) {
        const rad = (angle * Math.PI) / 180;
        const cos = Math.cos(rad);
        const sin = Math.sin(rad);

        // Ray direction in part space that corresponds to machine +Z (up)
        // Part rotates around X axis. Machine UP at angle 0 is Model +Z.
        // In the (Y, Z) plane, machine UP at angle 'a' (CCW) is:
        // dZ = cos(a), dY = -sin(a)
        const dz = cos;
        const dy = -sin;

        // Tool width offset (perpendicular to ray)
        const perpZ = -dy; // perpendicular Z
        const perpY = dz;  // perpendicular Y
        const halfW = (this.toolWidth || 0) / 2;

        if (halfW > 0) {
            const o1 = { x: p.z + perpZ * halfW, y: p.y + perpY * halfW };
            const o2 = { x: p.z - perpZ * halfW, y: p.y - perpY * halfW };
            // For tool edges, we use a larger startOff because we expect the
            // tool tip itself to be in contact with the model, but the edges
            // might "brush" against the surface.
            if (!this.castRay(o1, dz, dy, 0.1)) return false;
            if (!this.castRay(o2, dz, dy, 0.1)) return false;
        }

        // Center ray
        return this.castRay({ x: p.z, y: p.y }, dz, dy, 0);
    }
}
