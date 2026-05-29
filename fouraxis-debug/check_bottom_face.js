import './setup-global-three.js';
import { newPoint } from './src/geo/point.js';
import { newPolygon } from './src/geo/polygon.js';
import { Machinability, computeNormals, resamplePolygon } from './src/kiri/mode/cam/work/four-axis.js';

const zoff = 12.5;
const v1 = newPoint(0, -12.5, -12.5 + zoff);
const v2 = newPoint(0, 12.5, -12.5 + zoff);
const v3 = newPoint(0, 12.5, 12.5 + zoff);
const v4 = newPoint(0, -12.5, 12.5 + zoff);

const poly = newPolygon([v1, v2, v3, v4]);
const resampleDist = 0.5;
const resampled = resamplePolygon(poly, resampleDist);
const normals = computeNormals(resampled);

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

// We want to test different resolutions (e.g. 0.1 vs 0.01)
for (let res of [0.1, 0.05, 0.01]) {
    const mach = new Machinability([poly], res, mockTool);
    console.log(`\nResolution: ${res}`);
    console.log("Index | Y Coord | Z Coord (Original) | Normal Y | Normal Z | Angles Count");
    console.log("-------------------------------------------------------------------------");
    for (let i = 0; i < 20; i++) {
        const p = resampled.points[i];
        const n = normals[i];
        const mdrs = mach.getMDRs(p, n);
        console.log(`${i.toString().padStart(5)} | ${p.y.toFixed(3).padStart(7)} | ${(p.z - zoff).toFixed(3).padStart(20)} | ${n.y.toFixed(3).padStart(8)} | ${n.z.toFixed(3).padStart(8)} | ${mdrs.length.toString().padStart(12)}`);
    }
}

process.exit(0);
