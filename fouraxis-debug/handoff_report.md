# 4-Axis CAM Path Segment Decomposition & Integration: Handoff Document

This document describes the design, implementation, verification, and current status of the **4-Axis CAM Path Segment Decomposition** feature in Kiri:Moto. It serves as a comprehensive handoff guide for future developers or AI agents.

---

## 1. Feature Overview & Objectives

In standard 3-axis CNC machining, the cutter operates vertically. In 4-axis index/rotary machining, the workpiece rotates around the A-axis (parallel to the X-axis) to orient surfaces towards the tool. 

The core goals of this project:
1. **Accessibility Analysis (C-Space Mapping)**: Identify all collision-free angles (MDRs) for any given point on a slice contour using a stationary vertical tool relative to a rotating workpiece (modeled as clockwise relative rotation in the part's space).
2. **Path Segment Decomposition**:
   - Instead of choosing rotary angles independently for each point (which causes severe tool oscillation/chatter), group points into a minimum number of contiguous segments.
   - Each segment must have a shared, collision-free angular range (the **Maximum Disjoint Sector** or **MDS**).
   - Resolve overlaps to keep segments disjoint while covering the entire slice contour.
3. **Safe Transition Retracts**: Emit segments as separate open/closed polygons, allowing the print engine to generate vertical retraction and repositioning moves (Safe Z) when rotating the A-axis between segments.
4. **Visualization**: Provide clear, color-coded preview layers to show the segmented paths and the chosen A-axis tool tilt rays.

---

## 2. Key Architecture & File Structure

The 4-axis logic is integrated across several core files:

* **[four-axis.js](file:///home/jeff/code/grid-apps/src/kiri/mode/cam/work/four-axis.js)** (Core Logic)
  - Extends `Topo` to compute 4-axis toolpaths.
  - Implements the C-space mapping and the segment decomposition pipeline (`getSectors`, `intersectSectors`, `generateSegmentCandidates`, `findMinimumCover`, `resolveOverlaps`, `getContiguousRuns`).
  - Sets up the colorized preview layers and handles coordinate transformations for rendering.
* **[op-four-axis.js](file:///home/jeff/code/grid-apps/src/kiri/mode/cam/work/op-four-axis.js)** (Operations Layer)
  - Defines the CAM operation structure for 4-axis.
  - Feeds the generated CAM lines into the print engine using `emitTraces`.
* **[prepare.js](file:///home/jeff/code/grid-apps/src/kiri/mode/cam/work/prepare.js)** (G-code Preparation)
  - Emits the actual point streams for G-code output.
  - Configured to bypass ease-down and arcing for 4-axis/lathe paths to prevent interpolation artifacts.
* **[print.js](file:///home/jeff/code/grid-apps/src/kiri/core/print.js)** (G-code Generator)
  - Filters duplicate points and outputs G-code command sequences.
* **[fouraxis-debug/](file:///home/jeff/code/grid-apps/fouraxis-debug/)** (Ignored Debug & Test Directory)
  - Contains test suites, mock slicing models, mathematical sanity check scripts, and the original Eurographics paper.

---

## 3. Algorithm Implementation Details

### A. C-Space/Machinability Analysis
- Points on the slice contours are resampled using a distance `resampleDist` (production default: `resolution * 2`).
- For each resampled point, a surface outward normal is calculated.
- The `Machinability` class samples rotations from $0^\circ$ to $360^\circ$ in 5-degree steps. It simulates a vertical tool moving down to contact the point and checks if any other point on the slice collides with the tool.
- Valid, collision-free angles are stored in `p.mdr` as a list of discrete integers.

### B. Segment Decomposition (Eurographics Section 4.3)
1. **MDS Extraction (`getSectors`)**: Groups discrete valid angles into contiguous circular sectors (e.g., $[350^\circ, 20^\circ]$ is recognized as a single wrapping sector).
2. **Branching Search (`generateSegmentCandidates`)**:
   - Traverses the contour in both directions to extend segments as far as possible while maintaining a non-empty intersection of the MDS.
   - If a point offers multiple disjoint sectors that both intersect the running MDS, the search **branches** to explore both candidates (over-segmentation).
3. **Minimum Cover Solver (`findMinimumCover`)**:
   - Formulates selection as a circular interval cover problem.
   - Uses backtracking with pruning to find the absolute minimum number of segments to cover all points.
4. **Overlap Resolver (`resolveOverlaps`)**:
   - Uniquely assigns points to segments at overlap margins to maintain contiguity.
5. **Angle Assignment**:
   - For each segment, a single optimal angle is selected within its MDS that is closest to the ideal surface normal of the contact point.

---

## 4. Key Bugs Resolved

1. **Coordinate Axes Swap**:
   - *Problem*: `computeNormals()` returned normals with Y in `.x` and Z in `.y`. `isMachinable()` expected Y in `.y` and Z in `.x`.
   - *Fix*: Refactored both to use explicit `.y` and `.z` fields.
2. **Ball-Nose Tip Radius**:
   - *Problem*: Flat and ball-nose endmills default to `taper_tip = 0`, leading `Machinability` to assume `tipRad = 0` for ball-nose mills. This caused incorrect collision offsets.
   - *Fix*: Explicitly resolved the tip radius as `fluteDiameter() / 2` for ball-nose tools.
3. **Local Surface Skipping**:
   - *Problem*: The code originally skipped collision checks within a small radius of the contact point. This caused the tool to tilt directly through vertical walls at sharp corners (falsely reporting high accessibility).
   - *Fix*: Removed the local skipping check. The geometry checks are now numerically self-protecting.
4. **WebGPU Lathe Shader Bypass**:
   - *Problem*: When WebGPU was enabled, 4-axis operations defaulted to the GPU Lathe shader, bypassing the CPU 4-axis accessibility pipeline entirely.
   - *Fix*: Overrode `slice()` in `FourAxis` to force `this.gpu = false`, ensuring a graceful fallback to CPU-based slicing for 4-axis work.
5. **G-Code Point Filtering & Segmentation Jumps**:
   - *Problem*: A typo in `print.js` (`point.a == z`) caused pure A-axis rotation commands to be filtered out as duplicates. Also, `setContouring(true)` bypassed loop closures and safe Z retracts between segmented contours.
   - *Fix*: Corrected the comparison to `point.a == a`, switched `op-four-axis.js` to `emitTraces`, and bypassed easing-down and arcing for rotary modes in `prepare.js`.
6. **Visualization Mutation Bug**:
   - *Problem*: `Polygon.clone()` is a shallow copy of point references. Rotating/moving the clones for preview mutated the core toolpath coordinates in-place.
   - *Fix*: Implemented a custom `deepClone` utility in the rendering block to ensure visualization transforms do not modify underlying paths.

---

## 5. Preview & Visualization Layers

The preview layer outputs are configured inside [four-axis.js](file:///home/jeff/code/grid-apps/src/kiri/mode/cam/work/four-axis.js):
- **Segment Outlines**: The segmented contours are rendered in distinct colors:
  - Segment 1: Cyan (`0x00ffff`)
  - Segment 2: Magenta (`0x0ff00f` / `0xff00ff`)
  - Segment 3: Yellow (`0xffff00`)
  - Segment 4: Orange (`0xffa500`)
  - Segment 5: Purple (`0x800080`)
  - Overflow segments: Grey (`0x888888`)
- **Chosen Angle Rays**: Added the `"fouraxis-chosen"` layer (rendered in Blue `0x0000ff` as 0.5mm vectors) showing the selected tool orientation for every 10th point.
- **Machinability Range**: The red ray cloud (`0xff0000`) showing the bounds of collision-free rotation angles remains active.

---

## 6. How to Run and Verify

The debug directory `fouraxis-debug/` contains test files that run outside of the web browser using Node.js:

* **Verify G-Code and Segmentation**:
  ```bash
  node fouraxis-debug/test_fouraxis_gcode.js
  ```
  This script slices a mock square workpiece, runs the segment decomposition pipeline, generates the G-code point streams, and verifies:
  - Presence of A-axis rotation commands.
  - Closed loops closure in X/Y/A space.
  - Safe transition retraction height (Z clearances).

* **Rebuilding the Web Application**:
  After any modifications, rebuild and bundle:
  ```bash
  npm run webpack-src && npm run bundle:dev
  ```

---

## 7. Next Steps & Recommendations

When resuming work, consider the following optimization areas:
1. **Laplacian Smoothing**:
   - Currently, Laplacian smoothing is disabled to allow easy verification of segments.
   - *Recommendation*: Implement smoothing across segment boundaries to blend A-axis transitions, reducing acceleration spikes on the physical machine.
2. **Adaptive Resampling**:
   - Currently, `resampleDist` is fixed to `resolution * 2` for high resolution.
   - *Recommendation*: Implement adaptive resampling (denser on tight corners, sparser on flat edges) to reduce the number of C-space collision queries and speed up calculation.
3. **MDS Selection Optimization**:
   - When mapping chosen angles, the closest angle in the MDS to the surface normal is chosen point-by-point.
   - *Recommendation*: Perform a joint optimization (e.g. minimizing jerk/acceleration) to select a single smooth angular sweep per segment rather than independent point mappings.
