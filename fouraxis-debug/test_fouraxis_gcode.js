import './setup-global-three.js';
import { newPoint } from './src/geo/point.js';
import { newPolygon } from './src/geo/polygon.js';
import { newPrint } from './src/kiri/core/print.js';
import { prepare_one } from './src/kiri/mode/cam/work/prepare.js';
import { cam_export } from './src/kiri/mode/cam/work/export.js';
import { OpFourAxis } from './src/kiri/mode/cam/work/op-four-axis.js';

// 1. Setup mock settings
const settings = {
    device: {
        bedDepth: 200,
        bedWidth: 200,
        gcodeSpace: true,
        spindleMax: 10000,
        gcodeChange: ["M6 T{tool}"],
        gcodeSpindle: ["M3 S{speed}"],
        gcodeDwell: ["G4 P{time}"],
        gcodeResetA: ["G92.4 A0 R0"],
        gcodeFExt: "gcode",
        gcodeStrip: false,
        gcodePre: [],
        gcodePost: []
    },
    process: {
        camFastFeed: 1200,
        camFastFeedZ: 300,
        camZTop: 35,
        camStockX: 50,
        camStockY: 50,
        camStockZ: 50,
        camStockIndexed: true,
        camStockOffset: false,
        camForceZMax: false,
        camFullEngage: 1.0,
        camInnerFirst: true,
        camOriginCenter: true,
        camZClearance: 5,
        camToolInit: true,
        camArcEnabled: false, // disabled for 4-axis by default anyway
        camEaseDown: false,
        camEaseAngle: 10,
        camDepthFirst: false
    },
    stock: {
        center: { x: 0, y: 0 }
    },
    filter: { CAM: "CAM" },
    mode: "CAM",
    controller: {
        alignTop: true,
        devel: true
    }
};

// 2. Setup mock widget and four-axis operation
const state = {
    settings
};

const opInstance = new OpFourAxis(state, {
    type: 'four-axis',
    tool: 1,
    axis: 'X',
    rate: 1000,
    plunge: 500,
    step: 1
});

// Configure tool 1 (mock tool)
settings.tools = [
    {
        id: 1,
        number: 1,
        name: "mock-tool",
        type: "endmill",
        metric: true,
        flute_diam: 3.0,
        flute_len: 15.0,
        taper_angle: 0
    }
];

// Setup slice data for OpFourAxis:
// We simulate 2 slices (different Z heights), each containing a closed square contour.
// Each point of the contour has a different A-axis angle assigned.
const slice1 = {
    z: 10,
    camLines: [
        newPolygon([
            newPoint(10, 10, 10).setA(0),
            newPoint(10, -10, 10).setA(90),
            newPoint(-10, -10, 10).setA(180),
            newPoint(-10, 10, 10).setA(270)
        ])
    ]
};

const slice2 = {
    z: 5,
    camLines: [
        newPolygon([
            newPoint(10, 10, 5).setA(0),
            newPoint(10, -10, 5).setA(90),
            newPoint(-10, -10, 5).setA(180),
            newPoint(-10, 10, 5).setA(270)
        ])
    ]
};

opInstance.slices = [slice1, slice2];

const mockWidget = {
    id: "mock-widget",
    isSynth: () => false,
    getBoundingBox: () => {
        return {
            dim: { x: 25, y: 25, z: 25 },
            max: { x: 12.5, y: 12.5, z: 12.5 },
            min: { x: -12.5, y: -12.5, z: -12.5 }
        };
    },
    track: {
        ignore: false,
        pos: { x: 0, y: 0, z: 0 },
        top: 12.5
    },
    meta: {
        disabled: false
    },
    camops: [
        opInstance
    ]
};

// 3. Run prepare_one
const print = newPrint(settings, [mockWidget]);
await print.ready();
print.output = [];

console.log("Running prepare_one...");
await prepare_one(mockWidget, settings, print, null, (prog, msg) => {
    // console.log(`  prep progress: ${prog.toFixed(2)} - ${msg}`);
});

console.log(`Prepared output contains ${print.output.length} layers.`);

// 4. Export to G-code
console.log("\nGenerating G-code...");
const gcodeLines = [];
cam_export(print, (gcode) => {
    if (typeof gcode === 'string') {
        gcodeLines.push(...gcode.split('\n'));
    }
});

// 5. Inspect G-code results
console.log(`Generated ${gcodeLines.length} lines of G-code.\n`);
console.log("=== FIRST 40 LINES OF G-CODE ===");
console.log(gcodeLines.slice(0, 40).join('\n'));
console.log("===============================\n");

console.log("=== LAST 15 LINES OF G-CODE ===");
console.log(gcodeLines.slice(-15).join('\n'));
console.log("===============================\n");

// 6. Assert/Verify requirements
console.log("=== Verification Checks ===");

// Check 1: Do we have A-axis commands?
const aAxisLines = gcodeLines.filter(line => line.includes(' A'));
console.log(`Check 1: Found ${aAxisLines.length} lines with 'A' commands.`);
if (aAxisLines.length > 0) {
    console.log("  ✅ SUCCESS: G-code contains A-axis rotation commands!");
} else {
    console.log("  ❌ FAILURE: No A-axis rotation commands found in G-code.");
}

// Check 2: Are closed contours properly closed?
// We check if the G-code segments at each Z level form closed loops in X/Y space.
const segmentsByZ = {};
let currentX = 0, currentY = 0, currentZ = 0, currentA = 0;
for (let line of gcodeLines) {
    line = line.split(';')[0].trim();
    if (!line) continue;
    
    let parts = line.split(/\s+/);
    let cmd = parts[0];
    if (cmd !== 'G0' && cmd !== 'G1') continue;
    
    let nextX = currentX;
    let nextY = currentY;
    let nextZ = currentZ;
    let nextA = currentA;
    
    for (let i = 1; i < parts.length; i++) {
        let arg = parts[i];
        if (arg.startsWith('X')) nextX = parseFloat(arg.slice(1));
        else if (arg.startsWith('Y')) nextY = parseFloat(arg.slice(1));
        else if (arg.startsWith('Z')) nextZ = parseFloat(arg.slice(1));
        else if (arg.startsWith('A')) nextA = parseFloat(arg.slice(1));
    }
    
    if (cmd === 'G1' && (nextX !== currentX || nextY !== currentY || nextA !== currentA)) {
        if (!segmentsByZ[nextZ]) {
            segmentsByZ[nextZ] = [];
        }
        segmentsByZ[nextZ].push({
            x1: currentX, y1: currentY,
            x2: nextX, y2: nextY
        });
    }
    
    currentX = nextX;
    currentY = nextY;
    currentZ = nextZ;
    currentA = nextA;
}

let closedCorrectly = true;
const zLevels = Object.keys(segmentsByZ);
if (zLevels.length === 0) {
    closedCorrectly = false;
} else {
    for (let z of zLevels) {
        const segs = segmentsByZ[z];
        if (segs.length === 0) {
            closedCorrectly = false;
            break;
        }
        const startX = segs[0].x1;
        const startY = segs[0].y1;
        const endX = segs[segs.length - 1].x2;
        const endY = segs[segs.length - 1].y2;
        
        // Tolerance for floating point differences
        const error = Math.hypot(startX - endX, startY - endY);
        if (error > 0.001) {
            console.log(`  Z=${z} loop not closed: start=(${startX}, ${startY}), end=(${endX}, ${endY}), error=${error}`);
            closedCorrectly = false;
        }
    }
}

console.log(`Check 2: Loop closure verification.`);
if (closedCorrectly) {
    console.log("  ✅ SUCCESS: Closed contours are closed (loops return to starting point)!");
} else {
    console.log("  ❌ FAILURE: Contours are not being closed properly.");
}

// Check 3: Do transitions between slices go up to safe height?
// We expect a G0 move to Z coordinate matching the safe Z height (zSafe).
// Let's calculate zSafe:
// stockZ = 50 / 2 = 25 (since camStockIndexed is true).
// zSafe = max(camZTop, Math.hypot(50, 50)/2 + 5) = max(35, 70.7/2 + 5) = max(35, 35.35 + 5 = 40.35).
// Let's check if the G-code contains a 'Z40.35' command.
const safeZLines = gcodeLines.filter(line => line.includes('Z40.35'));
console.log(`Check 3: Safe Z transition height check.`);
if (safeZLines.length > 0) {
    console.log(`  ✅ SUCCESS: Found ${safeZLines.length} safe transition moves at Z40.35!`);
} else {
    console.log("  ❌ FAILURE: Could not find transition moves to safe height Z40.35 in G-code.");
}

process.exit(0);
