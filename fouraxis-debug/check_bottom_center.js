import './setup-global-three.js';
import { newPoint } from './src/geo/point.js';
import { newPolygon } from './src/geo/polygon.js';
import { Machinability, computeNormals, resamplePolygon } from './src/kiri/mode/cam/work/four-axis.js';

const zoff = 0; // zoff is 0 in the user's UI run
const v1 = newPoint(0, -12.5, -12.5);
const v2 = newPoint(0, 12.5, -12.5);
const v3 = newPoint(0, 12.5, 12.5);
const v4 = newPoint(0, -12.5, 12.5);

const poly = newPolygon([v1, v2, v3, v4]);
const resampleDist = 0.035; // matching user's point spacing
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

const resolution = 0.0175; // matching user's resolution
const mach = new Machinability([poly], resolution, mockTool);

console.log("Index | Y Coord | Z Coord | Angles Count | Angles List (subset)");
console.log("-----------------------------------------------------------------");

for (let i = 0; i < resampled.points.length; i++) {
    const p = resampled.points[i];
    const n = normals[i];
    
    // Print every 20th point, or points near Y = 0 (which is the center of the bottom face)
    const isNearCenter = Math.abs(p.y) < 0.2;
    const isEvery20 = i % 20 === 0;
    
    // We only care about points on the bottom face (z = -12.5)
    if (p.z === -12.5 && (isEvery20 || isNearCenter)) {
        const mdrs = mach.getMDRs(p, n);
        console.log(`${i.toString().padStart(5)} | ${p.y.toFixed(3).padStart(7)} | ${p.z.toFixed(3).padStart(7)} | ${mdrs.length.toString().padStart(12)} | [${mdrs.slice(0, 8).join(", ")}...]`);
    }
}

process.exit(0);
