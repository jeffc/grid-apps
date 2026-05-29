import './setup-global-three.js';
import { newPoint } from './src/geo/point.js';
import { newPolygon } from './src/geo/polygon.js';
import { Machinability } from './src/kiri/mode/cam/work/four-axis.js';

// User's coordinates and normal
const p = newPoint(0, -12.500, -12.500);
const n = { y: 0.000, z: -1.000 };

// Segments from the cube
const seg1 = {
    p1: newPoint(0, 12.50, -12.50),
    p2: newPoint(0, 12.50, 12.50)
};

const angle = 270;
const rad = (angle * Math.PI) / 180;
const cos = Math.cos(rad);
const sin = Math.sin(rad);
const dz = cos;
const dy = -sin;

// Test standard flat tools of various diameters: 3.175 (1/8"), 6.35 (1/4")
for (let dia of [3.0, 3.175, 6.35]) {
    const R = dia / 2;
    const CL = { z: p.z + n.z * R, y: p.y + n.y * R };
    const perpZ = -dy;
    const perpY = dz;
    const eps = 0.01;
    
    console.log(`\nTool Diameter: ${dia} (R = ${R})`);
    console.log(`CL: y=${CL.y}, z=${CL.z}`);
    
    const s1 = { z: seg1.p1.z, y: seg1.p1.y };
    const s2 = { z: seg1.p2.z, y: seg1.p2.y };
    
    const v1 = { z: s1.z - CL.z, y: s1.y - CL.y };
    const v2 = { z: s2.z - CL.z, y: s2.y - CL.y };
    const sz1 = v1.z * dz + v1.y * dy;
    const sx1 = v1.z * perpZ + v1.y * perpY;
    const sz2 = v2.z * dz + v2.y * dy;
    const sx2 = v2.z * perpZ + v2.y * perpY;
    
    console.log(`s1 (Endpoint 1): sz=${sz1.toFixed(6)}, sx=${sx1.toFixed(6)}`);
    console.log(`s2 (Endpoint 2): sz=${sz2.toFixed(6)}, sx=${sx2.toFixed(6)}`);
    
    const checkPoint = (depth, lateral) => {
        if (depth < eps) return true;
        const radAtDepth = R; // Flat tool has constant radius R
        const val = Math.abs(lateral) > radAtDepth - eps;
        console.log(`  checkPoint(depth=${depth.toFixed(6)}, lateral=${lateral.toFixed(6)}): radAtDepth=${radAtDepth}, limit=${(radAtDepth - eps).toFixed(6)}, ok=${val}`);
        return val;
    };
    
    const ok1 = checkPoint(sz1, sx1);
    const ok2 = checkPoint(sz2, sx2);
    const okMid = checkPoint((sz1 + sz2) / 2, (sx1 + sx2) / 2);
    
    console.log(`Collision checks: ok1=${ok1}, ok2=${ok2}, okMid=${okMid}`);
}

process.exit(0);
