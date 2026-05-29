import './setup-global-three.js';
import * as fs from 'fs';
import { newPoint } from './src/geo/point.js';
import { newPolygon } from './src/geo/polygon.js';
import { Machinability, computeNormals, resamplePolygon } from './src/kiri/mode/cam/work/four-axis.js';

// Construct 25mm square centered on (0, 0)
const v1 = newPoint(0, -12.5, -12.5);
const v2 = newPoint(0, 12.5, -12.5);
const v3 = newPoint(0, 12.5, 12.5);
const v4 = newPoint(0, -12.5, 12.5);
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

const resolution = 0.1;
const mach = new Machinability([poly], resolution, mockTool);

// Gather data
const pointsData = [];
for (let i = 0; i < resampled.points.length; i++) {
    const p = resampled.points[i];
    const n = normals[i];
    const mdrs = mach.getMDRs(p, n);
    pointsData.push({
        id: i,
        y: p.y,
        z: p.z,
        ny: n.y,
        nz: n.z,
        angles: mdrs
    });
}

const segmentsData = mach.segments.map(seg => ({
    p1: { y: seg.p1.y, z: seg.p1.z },
    p2: { y: seg.p2.y, z: seg.p2.z }
}));

const htmlContent = `
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>4-Axis Machinability Analysis Report</title>
    <style>
        :root {
            --bg-color: #0b0f19;
            --card-bg: #111827;
            --text-color: #e5e7eb;
            --text-muted: #9ca3af;
            --primary: #3b82f6;
            --primary-glow: rgba(59, 130, 246, 0.5);
            --success: #10b981;
            --danger: #ef4444;
            --warning: #f59e0b;
            --border-color: #1f2937;
        }

        * {
            box-sizing: border-box;
            margin: 0;
            padding: 0;
        }

        body {
            font-family: 'Outfit', 'Inter', -apple-system, sans-serif;
            background-color: var(--bg-color);
            color: var(--text-color);
            line-height: 1.6;
            padding: 2rem;
        }

        header {
            max-width: 1200px;
            margin: 0 auto 2rem auto;
            border-bottom: 1px solid var(--border-color);
            padding-bottom: 1.5rem;
        }

        h1 {
            font-size: 2.5rem;
            font-weight: 700;
            background: linear-gradient(135deg, #60a5fa, #3b82f6);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            margin-bottom: 0.5rem;
        }

        .subtitle {
            color: var(--text-muted);
            font-size: 1.1rem;
        }

        .container {
            max-width: 1200px;
            margin: 0 auto;
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 2rem;
        }

        @media (max-width: 1024px) {
            .container {
                grid-template-columns: 1fr;
            }
        }

        .card {
            background-color: var(--card-bg);
            border: 1px solid var(--border-color);
            border-radius: 12px;
            padding: 1.5rem;
            box-shadow: 0 4px 20px rgba(0, 0, 0, 0.3);
            display: flex;
            flex-direction: column;
            gap: 1.5rem;
        }

        .card-title {
            font-size: 1.25rem;
            font-weight: 600;
            color: #ffffff;
            border-left: 4px solid var(--primary);
            padding-left: 0.75rem;
        }

        /* Visualization Area */
        .vis-container {
            position: relative;
            background-color: #030712;
            border: 1px solid var(--border-color);
            border-radius: 8px;
            aspect-ratio: 1;
            width: 100%;
            display: flex;
            justify-content: center;
            align-items: center;
            overflow: hidden;
        }

        canvas {
            display: block;
            width: 100%;
            height: 100%;
        }

        .controls {
            display: flex;
            flex-direction: column;
            gap: 1rem;
        }

        .control-group {
            display: flex;
            flex-direction: column;
            gap: 0.5rem;
        }

        label {
            font-size: 0.875rem;
            font-weight: 500;
            color: var(--text-muted);
        }

        .slider-container {
            display: flex;
            align-items: center;
            gap: 1rem;
        }

        input[type="range"] {
            flex-grow: 1;
            height: 6px;
            background: var(--border-color);
            border-radius: 3px;
            outline: none;
            -webkit-appearance: none;
        }

        input[type="range"]::-webkit-slider-thumb {
            -webkit-appearance: none;
            width: 18px;
            height: 18px;
            border-radius: 50%;
            background: var(--primary);
            cursor: pointer;
            box-shadow: 0 0 10px var(--primary-glow);
        }

        .angle-val {
            font-family: monospace;
            font-size: 1.1rem;
            font-weight: bold;
            color: var(--primary);
            min-width: 50px;
            text-align: right;
        }

        /* Table Area */
        .table-container {
            max-height: 400px;
            overflow-y: auto;
            border: 1px solid var(--border-color);
            border-radius: 8px;
        }

        table {
            width: 100%;
            border-collapse: collapse;
            font-size: 0.875rem;
            text-align: left;
        }

        th, td {
            padding: 0.75rem 1rem;
            border-bottom: 1px solid var(--border-color);
        }

        th {
            background-color: #1f2937;
            color: #ffffff;
            font-weight: 600;
            position: sticky;
            top: 0;
        }

        tr:hover {
            background-color: rgba(255, 255, 255, 0.02);
            cursor: pointer;
        }

        tr.selected {
            background-color: rgba(59, 130, 246, 0.15);
            border-left: 2px solid var(--primary);
        }

        .badge {
            display: inline-block;
            padding: 0.125rem 0.5rem;
            border-radius: 4px;
            font-size: 0.75rem;
            font-weight: 600;
        }

        .badge-success {
            background-color: rgba(16, 185, 129, 0.15);
            color: var(--success);
            border: 1px solid rgba(16, 185, 129, 0.3);
        }

        .badge-danger {
            background-color: rgba(239, 68, 68, 0.15);
            color: var(--danger);
            border: 1px solid rgba(239, 68, 68, 0.3);
        }

        /* Explanations */
        .info-box {
            background-color: rgba(59, 130, 246, 0.05);
            border-left: 4px solid var(--primary);
            padding: 1rem;
            border-radius: 4px;
            font-size: 0.95rem;
        }

        .info-box p {
            margin-bottom: 0.5rem;
        }

        .info-box p:last-child {
            margin-bottom: 0;
        }

        .highlight {
            color: #ffffff;
            font-weight: 600;
        }

        .angles-arc-legend {
            display: flex;
            gap: 1.5rem;
            font-size: 0.85rem;
            margin-top: 0.5rem;
        }

        .legend-item {
            display: flex;
            align-items: center;
            gap: 0.5rem;
        }

        .legend-color {
            width: 12px;
            height: 12px;
            border-radius: 3px;
        }
    </style>
</head>
<body>
    <header>
        <h1>4-Axis Machinability Report</h1>
        <p class="subtitle">Unit Test Analysis of 25mm Square centered on Rotation Axis</p>
    </header>

    <div class="container">
        <!-- Column 1: Interactive Diagram & Controls -->
        <div class="card">
            <div class="card-title">Interactive Analysis Diagram</div>
            <div class="vis-container">
                <canvas id="visCanvas"></canvas>
            </div>
            
            <div class="controls">
                <div class="control-group">
                    <label for="angleSlider">Rotary Axis Angle (A-axis):</label>
                    <div class="slider-container">
                        <input type="range" id="angleSlider" min="0" max="355" step="5" value="0">
                        <span class="angle-val" id="angleLabel">0°</span>
                    </div>
                </div>

                <div class="angles-arc-legend">
                    <div class="legend-item">
                        <div class="legend-color" style="background-color: #10b981;"></div>
                        <span>Machinable Angles</span>
                    </div>
                    <div class="legend-item">
                        <div class="legend-color" style="background-color: #ef4444;"></div>
                        <span>Non-Machinable (Collision / Culled)</span>
                    </div>
                    <div class="legend-item">
                        <div class="legend-color" style="background-color: #f59e0b; width: 20px; height: 3px; border-radius: 0;"></div>
                        <span>Outward Surface Normal</span>
                    </div>
                </div>

                <div class="info-box">
                    <p><span class="highlight">Hover over points</span> on the square to inspect their coordinates, computed normal vectors, and the angular machinability intervals.</p>
                    <p><span class="highlight">Drag the slider</span> to rotate the tool around the part in simulation (CW relative tool rotation corresponding to CCW part rotation).</p>
                </div>
            </div>
        </div>

        <!-- Column 2: Data Table & Explanations -->
        <div class="card">
            <div class="card-title">Analysis Metrics & Swaps Explanation</div>
            
            <div class="info-box" style="background-color: rgba(245, 158, 11, 0.03); border-left-color: var(--warning);">
                <p><span class="highlight" style="color: var(--warning);">Why did this mismatch the UI?</span></p>
                <p>The UI rendering pipeline was drawing vectors by swapping coordinates incorrectly, which caused the machinability rays to display rotated at 90 degrees or not show up at Y=0 and Z=0.</p>
                <p>Specifically, the <code>computeNormals()</code> function returned coordinates where <code>n.x</code> represented Y-components and <code>n.y</code> represented Z-components. However, the collision check in <code>isMachinable()</code> read <code>n.x</code> as the Z-component, resulting in total culling or false collisions at the axes. Restoring the clean <code>z/y</code> mapping resolved this completely.</p>
            </div>

            <div class="card-title" style="margin-top: 1rem;">Resampled Points Output</div>
            <div class="table-container">
                <table>
                    <thead>
                        <tr>
                            <th>Point (Y, Z)</th>
                            <th>Normal (dY, dZ)</th>
                            <th>Machinable Angles</th>
                        </tr>
                    </thead>
                    <tbody id="pointsTableBody">
                        <!-- Filled by JS -->
                    </tbody>
                </table>
            </div>
        </div>
    </div>

    <script>
        const points = ${JSON.stringify(pointsData)};
        const segments = ${JSON.stringify(segmentsData)};
        const toolParams = {
            tipRad: 1.5,
            fluteRad: 1.5,
            fluteLen: 15.0
        };

        const canvas = document.getElementById('visCanvas');
        const ctx = canvas.getContext('2d');
        const angleSlider = document.getElementById('angleSlider');
        const angleLabel = document.getElementById('angleLabel');
        const tableBody = document.getElementById('pointsTableBody');

        let selectedPointId = null;
        let scale = 12; // pixels per mm
        let rotationAngle = 0;

        // Resize canvas to container
        function resizeCanvas() {
            const rect = canvas.parentElement.getBoundingClientRect();
            canvas.width = rect.width * window.devicePixelRatio;
            canvas.height = rect.width * window.devicePixelRatio;
            ctx.scale(window.devicePixelRatio, window.devicePixelRatio);
            draw();
        }

        window.addEventListener('resize', resizeCanvas);
        setTimeout(resizeCanvas, 100);

        // Populate Table
        points.forEach(p => {
            const tr = document.createElement('tr');
            tr.dataset.id = p.id;
            
            // Only highlight key axis points by default or make them easy to identify
            const isAxis = Math.abs(p.y) < 0.3 || Math.abs(p.z) < 0.3;
            if (isAxis) {
                tr.style.fontWeight = 'bold';
            }

            tr.innerHTML = \`
                <td>(\${p.y.toFixed(2)}, \${p.z.toFixed(2)})</td>
                <td>(\${p.ny.toFixed(2)}, \${p.nz.toFixed(2)})</td>
                <td>
                    <span class="badge \${p.angles.length > 0 ? 'badge-success' : 'badge-danger'}">
                        \${p.angles.length} angles (\${Math.round(p.angles.length * 5)}°)
                    </span>
                </td>
            \`;

            tr.addEventListener('click', () => {
                document.querySelectorAll('tr').forEach(r => r.classList.remove('selected'));
                tr.classList.add('selected');
                selectedPointId = p.id;
                draw();
            });

            tableBody.appendChild(tr);
        });

        // Mouse interaction for canvas
        canvas.addEventListener('mousemove', (e) => {
            const rect = canvas.getBoundingClientRect();
            const mouseX = e.clientX - rect.left - rect.width / 2;
            const mouseY = e.clientY - rect.top - rect.height / 2;

            // Convert to model space
            // Canvas space: Y is down, X is right
            // Model space: Y is vertical (up), Z is horizontal (right)
            // So: model Z = mouseX / scale, model Y = -mouseY / scale
            const mz = mouseX / scale;
            const my = -mouseY / scale;

            // Find closest point
            let closest = null;
            let minDist = Infinity;
            points.forEach(p => {
                const dist = Math.sqrt((p.y - my)**2 + (p.z - mz)**2);
                if (dist < minDist) {
                    minDist = dist;
                    closest = p;
                }
            });

            if (minDist < 1.5) { // hover radius in mm
                selectedPointId = closest.id;
                // Sync table selection
                document.querySelectorAll('tr').forEach(r => r.classList.remove('selected'));
                const tr = tableBody.children[closest.id];
                if (tr) {
                    tr.classList.add('selected');
                    tr.scrollIntoView({ block: 'nearest', behavior: 'smooth' });
                }
                draw();
            }
        });

        angleSlider.addEventListener('input', (e) => {
            rotationAngle = parseInt(e.target.value);
            angleLabel.textContent = rotationAngle + '°';
            draw();
        });

        function draw() {
            const w = canvas.width / window.devicePixelRatio;
            const h = canvas.height / window.devicePixelRatio;
            
            ctx.clearRect(0, 0, w, h);
            ctx.save();
            ctx.translate(w / 2, h / 2); // Center origin (0, 0)
            
            // Draw grid
            ctx.strokeStyle = '#111827';
            ctx.lineWidth = 1;
            for (let i = -20; i <= 20; i += 5) {
                ctx.beginPath();
                ctx.moveTo(i * scale, -20 * scale);
                ctx.lineTo(i * scale, 20 * scale);
                ctx.stroke();

                ctx.beginPath();
                ctx.moveTo(-20 * scale, i * scale);
                ctx.lineTo(20 * scale, i * scale);
                ctx.stroke();
            }

            // Draw center axes
            ctx.strokeStyle = '#1f2937';
            ctx.lineWidth = 1.5;
            ctx.beginPath();
            ctx.moveTo(-w/2, 0); ctx.lineTo(w/2, 0);
            ctx.moveTo(0, -h/2); ctx.lineTo(0, h/2);
            ctx.stroke();

            // Label axes (Z horizontal, Y vertical in our analysis plane)
            ctx.fillStyle = '#4b5563';
            ctx.font = '10px monospace';
            ctx.fillText('+Z (Model Z)', w/2 - 80, -5);
            ctx.fillText('+Y (Model Y)', 5, -h/2 + 20);

            // Draw 25mm Square Contour
            ctx.strokeStyle = '#ffffff';
            ctx.lineWidth = 2.5;
            ctx.beginPath();
            // Vertices: Z horizontal (X in canvas), Y vertical (Y in canvas, inverted)
            ctx.moveTo(-12.5 * scale, 12.5 * scale); // bottom-left (Z=-12.5, Y=-12.5) -> canvas X=-12.5, Y=12.5
            ctx.lineTo(-12.5 * scale, -12.5 * scale); // top-left (Z=-12.5, Y=12.5) -> canvas X=-12.5, Y=-12.5
            ctx.lineTo(12.5 * scale, -12.5 * scale);  // top-right (Z=12.5, Y=12.5) -> canvas X=12.5, Y=-12.5
            ctx.lineTo(12.5 * scale, 12.5 * scale);   // bottom-right (Z=12.5, Y=-12.5) -> canvas X=12.5, Y=12.5
            ctx.closePath();
            ctx.stroke();

            // Draw Resampled points
            points.forEach(p => {
                ctx.fillStyle = '#374151';
                ctx.beginPath();
                ctx.arc(p.z * scale, -p.y * scale, 2, 0, Math.PI * 2);
                ctx.fill();
            });

            // Draw Selected Point Details & Angles Arc
            const selP = points.find(p => p.id === selectedPointId);
            if (selP) {
                // Highlight point
                ctx.fillStyle = varColor('--primary');
                ctx.beginPath();
                ctx.arc(selP.z * scale, -selP.y * scale, 5, 0, Math.PI * 2);
                ctx.fill();

                // Draw Normal Vector
                ctx.strokeStyle = '#f59e0b';
                ctx.lineWidth = 2;
                ctx.beginPath();
                ctx.moveTo(selP.z * scale, -selP.y * scale);
                ctx.lineTo((selP.z + selP.nz * 3) * scale, -(selP.y + selP.ny * 3) * scale);
                ctx.stroke();

                // Draw Machinable angles arc around the selected point
                const rad = 25; // radius of visual arc
                for (let a = 0; a < 360; a += 5) {
                    const angleRad = a * Math.PI / 180;
                    // ray direction
                    const rz = Math.cos(angleRad);
                    const ry = -Math.sin(angleRad); // CW Tool relative rotation is (cos a, -sin a)
                    
                    const isOk = selP.angles.includes(a);
                    ctx.strokeStyle = isOk ? 'rgba(16, 185, 129, 0.4)' : 'rgba(239, 68, 68, 0.15)';
                    ctx.lineWidth = isOk ? 1.5 : 1;
                    ctx.beginPath();
                    ctx.moveTo(selP.z * scale, -selP.y * scale);
                    ctx.lineTo((selP.z + rz * 1.5) * scale, -(selP.y + ry * 1.5) * scale);
                    ctx.stroke();
                }
            }

            // Draw Tool at current slider rotation angle
            // Rotation angle is rotation of part CCW, which is rotation of tool CW around origin
            const rotRad = rotationAngle * Math.PI / 180;
            // Tool axis vector pointing from tool tip outwards
            const tdz = Math.cos(rotRad);
            const tdy = -Math.sin(rotRad);
            
            // To position the tool tangent to the square at this rotation angle:
            // Find the contact center (CC) point on the contour that is closest in direction of (tdz, tdy).
            // We search resampled points for the one with maximum dot product with the tool axis (tdz, tdy)
            let maxDot = -Infinity;
            let ccPoint = null;
            let ccNormal = null;
            points.forEach(p => {
                // Find normal corresponding to this point
                const n = p;
                const dot = tdz * n.nz + tdy * n.ny;
                if (dot > maxDot) {
                    maxDot = dot;
                    ccPoint = p;
                    ccNormal = n;
                }
            });

            if (ccPoint) {
                // Tool center (Cutter Location CL) is CC + R * normal
                const R = toolParams.tipRad;
                const clz = ccPoint.z + ccNormal.nz * R;
                const cly = ccPoint.y + ccNormal.ny * R;

                // Draw Tool Tip sphere (or cylinder base)
                ctx.fillStyle = 'rgba(59, 130, 246, 0.3)';
                ctx.strokeStyle = 'rgba(59, 130, 246, 0.8)';
                ctx.lineWidth = 1.5;
                ctx.beginPath();
                ctx.arc(clz * scale, -cly * scale, R * scale, 0, Math.PI * 2);
                ctx.fill();
                ctx.stroke();

                // Draw Tool Shaft (extending along tdz, tdy)
                const shaftLen = toolParams.fluteLen;
                const sx = clz + tdz * shaftLen;
                const sy = cly + tdy * shaftLen;

                // Draw shaft line
                ctx.strokeStyle = 'rgba(59, 130, 246, 0.6)';
                ctx.lineWidth = toolParams.fluteRad * 2 * scale;
                ctx.lineCap = 'butt';
                ctx.beginPath();
                ctx.moveTo(clz * scale, -cly * scale);
                ctx.lineTo(sx * scale, -sy * scale);
                ctx.stroke();

                // Draw Tool center point
                ctx.fillStyle = '#ffffff';
                ctx.beginPath();
                ctx.arc(clz * scale, -cly * scale, 2, 0, Math.PI * 2);
                ctx.fill();
            }

            ctx.restore();
        }

        function varColor(name) {
            return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
        }
    </script>
</body>
</html>
`;

const reportPath = '/home/jeff/.gemini/antigravity-cli/brain/f19cdb98-449b-47fc-a9ea-9cf9870d4a64/machinability_report.html';
fs.writeFileSync(reportPath, htmlContent);
console.log(`HTML Report generated successfully at: ${reportPath}`);
process.exit(0);
