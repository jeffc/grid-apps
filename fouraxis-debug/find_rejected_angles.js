import './setup-global-three.js';
import { newPoint } from './src/geo/point.js';
import { newPolygon } from './src/geo/polygon.js';
import { Machinability, computeNormals, resamplePolygon } from './src/kiri/mode/cam/work/four-axis.js';

const zoff = 0;
const v1 = newPoint(0, -12.5, -12.5);
const v2 = newPoint(0, 12.5, -12.5);
const v3 = newPoint(0, 12.5, 12.5);
const v4 = newPoint(0, -12.5, 12.5);

const poly4 = newPolygon([v1, v2, v3, v4]);

// Generate a highly resampled polygon (simulating a high-triangle mesh slice)
const polyDense = resamplePolygon(poly4, 0.05);

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

const resolution = 0.05;

function runAnalysis(poly, name) {
    const mach = new Machinability([poly], resolution, mockTool);
    
    // We want to test point y = -12.5, z = -12.5 (the corner)
    const p = newPoint(0, -12.500, -12.500);
    const n = { y: -0.7071, z: -0.7071 }; // Diagonal normal at corner
    
    console.log(`\n=== Analysis for ${name} (${mach.segments.length} segments) ===`);
    
    const accepted = [];
    const rejected = [];
    
    for (let a = 0; a < 360; a += 5) {
        const rad = (a * Math.PI) / 180;
        const cos = Math.cos(rad);
        const sin = Math.sin(rad);
        const dz = cos;
        const dy = -sin;
        
        // 1. Back-face culling
        const dot = dz * n.z + dy * n.y;
        if (dot < -0.001) {
            rejected.push({ angle: a, reason: "Back-face culling", val: dot });
            continue;
        }
        
        // 2. Collision check
        let collided = false;
        const R = mach.tool.tipRad;
        const CL = { z: p.z + n.z * R, y: p.y + n.y * R };
        const perpZ = -dy;
        const perpY = dz;
        const eps = 0.01;
        const chordEps = 0.1;
        const Rsq = (R - chordEps) * (R - chordEps);
        const skipDistSq = (resolution * 10) ** 2;
        
        for (let idx = 0; idx < mach.segments.length; idx++) {
            const seg = mach.segments[idx];
            const s1 = { z: seg.p1.z, y: seg.p1.y };
            const s2 = { z: seg.p2.z, y: seg.p2.y };
            
            const d1sq = (s1.z - p.z)**2 + (s1.y - p.y)**2;
            const d2sq = (s2.z - p.z)**2 + (s2.y - p.y)**2;
            // if (d1sq < skipDistSq || d2sq < skipDistSq) continue;
            
            // Sphere collision
            const dxs = s2.z - s1.z;
            const dys = s2.y - s1.y;
            const lensq_seg = dxs * dxs + dys * dys;
            let ts = lensq_seg > 0 ? ((CL.z - s1.z) * dxs + (CL.y - s1.y) * dys) / lensq_seg : 0;
            ts = Math.max(0, Math.min(1, ts));
            const closestZ = s1.z + ts * dxs;
            const closestY = s1.y + ts * dys;
            const distSq = (closestZ - CL.z) ** 2 + (closestY - CL.y) ** 2;
            if (R > eps && distSq < Rsq) {
                rejected.push({ angle: a, reason: `Sphere collision with seg ${idx}`, seg });
                collided = true;
                break;
            }
            
            // Shaft collision
            const v1 = { z: s1.z - CL.z, y: s1.y - CL.y };
            const v2 = { z: s2.z - CL.z, y: s2.y - CL.y };
            const sz1 = v1.z * dz + v1.y * dy;
            const sx1 = v1.z * perpZ + v1.y * perpY;
            const sz2 = v2.z * dz + v2.y * dy;
            const sx2 = v2.z * perpZ + v2.y * perpY;
            
            const checkPoint = (depth, lateral) => {
                if (depth < eps) return true;
                return Math.abs(lateral) > mach.getRadius(depth + R) - eps;
            };
            
            if (!checkPoint(sz1, sx1) || !checkPoint(sz2, sx2) || !checkPoint((sz1 + sz2) / 2, (sx1 + sx2) / 2)) {
                rejected.push({ angle: a, reason: `Shaft endpoint collision with seg ${idx}`, seg });
                collided = true;
                break;
            }
            
            const dsx = sx2 - sx1;
            const dsz = sz2 - sz1;
            if (Math.abs(dsx) > 1e-9) {
                let t_ray = -sx1 / dsx;
                if (t_ray > 0 && t_ray < 1) {
                    const sz_at_t = sz1 + t_ray * dsz;
                    if (!checkPoint(sz_at_t, 0)) {
                        rejected.push({ angle: a, reason: `Shaft ray-crossing collision with seg ${idx}`, seg });
                        collided = true;
                        break;
                    }
                }
            }
        }
        
        if (!collided) {
            accepted.push(a);
        }
    }
    
    console.log(`Accepted count: ${accepted.length}`);
    console.log("Accepted angles:", accepted.join(", "));
    console.log("First few rejections:");
    rejected.slice(0, 10).forEach(r => {
        if (r.reason === "Back-face culling") {
            console.log(`  Angle ${r.angle}: ${r.reason} (dot=${r.val.toFixed(4)})`);
        } else {
            console.log(`  Angle ${r.angle}: ${r.reason} | Seg: (${r.seg.p1.y.toFixed(3)}, ${r.seg.p1.z.toFixed(3)}) -> (${r.seg.p2.y.toFixed(3)}, ${r.seg.p2.z.toFixed(3)})`);
        }
    });
}

runAnalysis(poly4, "Simple 4-segment polygon");
runAnalysis(polyDense, "Dense resampled polygon");

process.exit(0);
