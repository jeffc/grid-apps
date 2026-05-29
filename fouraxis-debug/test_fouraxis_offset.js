import './setup-global-three.js';
import { newPoint } from './src/geo/point.js';
import { newPolygon } from './src/geo/polygon.js';
import { Machinability, computeNormals, resamplePolygon } from './src/kiri/mode/cam/work/four-axis.js';

// Construct 25mm square, but shifted in Z by +12.5mm (so Z is from 0 to 25)
const zoff = 12.5;
const v1 = newPoint(0, -12.5, -12.5 + zoff); // (x, y, z)
const v2 = newPoint(0, 12.5, -12.5 + zoff);
const v3 = newPoint(0, 12.5, 12.5 + zoff);
const v4 = newPoint(0, -12.5, 12.5 + zoff);

const poly = newPolygon([v1, v2, v3, v4]);

console.log("Constructed offset square polygon vertices:");
console.log(poly.points.map(p => `(y=${p.y.toFixed(2)}, z=${p.z.toFixed(2)})`));

const resampleDist = 0.5; // mm
const resampled = resamplePolygon(poly, resampleDist);
console.log(`Resampled polygon into ${resampled.points.length} points.`);

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

const resolution = 0.1;
const mach = new Machinability([poly], resolution, mockTool);

console.log("\nRunning Machinability Analysis on key points:");

let checkedCount = 0;
for (let i = 0; i < resampled.points.length; i++) {
    const p = resampled.points[i];
    const n = normals[i];
    
    // Check points closest to the axes
    const isYNearZero = Math.abs(p.y) < 0.3;
    // Note: Z coordinate is now shifted, so the axis is at Z = zoff
    const isZNearAxis = Math.abs(p.z - zoff) < 0.3;
    
    if (isYNearZero || isZNearAxis) {
        checkedCount++;
        const mdrs = mach.getMDRs(p, n);
        console.log(`Point: (y=${p.y.toFixed(2)}, z=${p.z.toFixed(2)}) | Normal: (dy=${n.y.toFixed(4)}, dz=${n.z.toFixed(4)}) | Machinable angles count: ${mdrs.length}`);
        if (mdrs.length > 0) {
            console.log(`  Angles (subset): [${mdrs.slice(0, 10).join(", ")}${mdrs.length > 10 ? ", ..." : ""}]`);
        } else {
            console.log(`  ❌ NO MACHINABLE ANGLES FOUND`);
        }
    }
}

console.log(`\nChecked ${checkedCount} key points on the axes.`);
process.exit(0);
