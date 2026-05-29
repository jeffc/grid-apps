import './setup-global-three.js';
import { newPoint } from './src/geo/point.js';
import { newPolygon } from './src/geo/polygon.js';
import { sliceConnect } from './src/geo/slicer.js';
import { Slicer as topo_slicer } from './src/kiri/mode/cam/work/slicer-topo.js';
import { Machinability, computeNormals, resamplePolygon } from './src/kiri/mode/cam/work/four-axis.js';

// 1. Construct vertices of a 25mm cube (12 triangles)
const size = 12.5; // half-size
const rawVertices = [
    // Front face (X = size)
    size, -size, -size,   size, size, -size,    size, size, size,
    size, -size, -size,   size, size, size,     size, -size, size,
    // Back face (X = -size)
    -size, size, -size,   -size, -size, -size,  -size, -size, size,
    -size, size, -size,   -size, -size, size,   -size, size, size,
    // Right face (Y = size)
    size, size, -size,    -size, size, -size,   -size, size, size,
    size, size, -size,    -size, size, size,    size, size, size,
    // Left face (Y = -size)
    -size, -size, -size,  size, -size, -size,   size, -size, size,
    -size, -size, -size,  size, -size, size,    -size, -size, size,
    // Top face (Z = size)
    size, -size, size,    size, size, size,     -size, size, size,
    size, -size, size,    -size, size, size,    -size, -size, size,
    // Bottom face (Z = -size)
    size, size, -size,    size, -size, -size,   -size, -size, -size,
    size, size, -size,    -size, -size, -size,  -size, size, -size
];

const vertices = new Float32Array(rawVertices);
const zoff = 12.5; // track top / z offset

// 2. Perform Swap XZ and add zoff (simulating Topo.slice)
// Swap XZ in shared array:
// vertices[i] = z + zoff
// vertices[i+2] = x
for (let i = 0; i < vertices.length; i += 3) {
    const x = vertices[i];
    const z = vertices[i + 2] + zoff;
    vertices[i] = z;
    vertices[i + 2] = x;
}

// 3. Slice the mesh at the middle slice.
// We want to slice at the original X = 0 slice.
// Since original X is now stored in vertices[i+2] (which is the new Z coordinate),
// the slicer slices along the new Z coordinate.
// So we slice at Z = 0 (which corresponds to original X = 0).
console.log("Running Topographical Slicer...");
const slicer = new topo_slicer(0);
slicer.points = [];
for (let i = 0; i < vertices.length; i += 3) {
    slicer.points.push(newPoint(vertices[i], vertices[i+1], vertices[i+2]));
}
slicer.min = -25.0; // range to slice
slicer.max = 25.0;

// slice at z = 0
const sliceResult = slicer.sliceZ(0);
console.log(`Slicer generated ${sliceResult.length} lines.`);

// 4. Connect lines to form polygons and swap XZ back (simulating Topo.sliceWorker)
const polys = sliceConnect(sliceResult);
console.log(`Connected lines into ${polys.length} polygons.`);

for (let poly of polys) {
    for (let p of poly.points) {
        if (!p.swapped) {
            p.swapXZ();
            p.swapped = true;
        }
    }
}

// Log contour vertices
polys.forEach((p, idx) => {
    console.log(`Polygon ${idx} vertices (post-swapXZ):`);
    console.log(p.points.map(pt => `(x=${pt.x.toFixed(2)}, y=${pt.y.toFixed(2)}, z=${pt.z.toFixed(2)})`));
});

// 5. Run resample and compute normals (simulating FourAxis.generatePath)
const resampleDist = 0.5;
const resampledPolys = [];
for (let poly of polys) {
    const resampled = resamplePolygon(poly, resampleDist);
    if (resampled) {
        resampledPolys.push(resampled);
    }
}

console.log(`Resampled into ${resampledPolys.length} polygons.`);
const firstResampled = resampledPolys[0];
const normals = computeNormals(firstResampled);

// 6. Run Machinability Analysis (using real code)
const mockTool = {
    isBallMill: () => false,
    isTaperMill: () => false,
    isTaperBall: () => false,
    isDrill: () => false,
    fluteDiameter: () => 3.0,
    tipDiameter: () => 3.0,
    fluteLength: () => 15.0,
    getTaperAngle: () => 0
};

const resolution = 0.01;
const mach = new Machinability(polys, resolution, mockTool);

console.log("\nRunning Machinability Analysis on resampled points:");
let checkedCount = 0;
for (let i = 0; i < firstResampled.points.length; i++) {
    const p = firstResampled.points[i];
    const n = normals[i];
    
    // Check points closest to the axes
    const isYNearZero = Math.abs(p.y) < 0.3;
    const isZNearAxis = Math.abs(p.z - zoff) < 0.3;
    
    if (isYNearZero || isZNearAxis) {
        checkedCount++;
        const mdrs = mach.getMDRs(p, n);
        console.log(`Point: (y=${p.y.toFixed(2)}, z=${p.z.toFixed(2)}) | Normal: (dy=${n.y.toFixed(4)}, dz=${n.z.toFixed(4)}) | Machinable angles count: ${mdrs.length}`);
    }
}

console.log(`\nChecked ${checkedCount} key points.`);
process.exit(0);
