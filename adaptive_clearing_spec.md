# High-Efficiency 3D Adaptive Clearing: Algorithmic Implementation Specification

**Target Audience:** Autonomous Coding Agent / Implementation Developer
**Objective:** Implement a 2.5D/3D volumetric roughing (adaptive clearing) toolpath generator. The core directive is to maintain a STRICT maximum Tool Engagement Angle (TEA) to prevent tool breakage, while efficiently removing bulk material using a step-down/step-up (terracing) strategy.

---

## 1. Global Inputs & Parameters
Your implementation must accept the following parameters before execution:
* `Tool_D`: Tool diameter.
* `Tool_R`: Tool radius (`Tool_D / 2`).
* `Target_RDOC`: Target Radial Depth of Cut ($a_e$).
* `Z_Major`: Maximum step-down distance (bulk removal depth).
* `Z_Minor`: Step-up distance (terrace height for angled walls).
* `Stock_to_Leave`: Radial offset left on the final part walls for the finishing pass.
* `Ramp_Angle_Max`: Maximum allowable angle for helical entry (typically 2° - 3°).
* `Micro_Tolerance`: A microscopic offset (e.g., 0.01mm) used in boolean operations to prevent exact-edge overlapping.

---

## 2. Mathematical Constants & Formulas
Implement these as helper functions:

**2.1 Tool Engagement Angle (TEA) Calculation:**
The target engagement angle $	heta$ (in radians) must remain constant.
`Theta = acos(1 - (2 * Target_RDOC / Tool_D))`

**2.2 Dynamic Feedrate Adjustment (Chip Thinning):**
When the tool is cutting at an RDOC less than its radius, radial chip thinning occurs. Adjust the base feedrate (`Base_F`) using this multiplier:
`F_adj_multiplier = Tool_R / sqrt((2 * Tool_R * Target_RDOC) - (Target_RDOC ^ 2))`
`Active_Feedrate = Base_F * F_adj_multiplier`

---

## 3. The 3D Slicing & Terracing Loop (Main Architecture)
The algorithm processes the 3D geometry from top to bottom.

### Phase 3.1: Generate Z-Slices
1. Determine the total Z-depth of the part.
2. Generate an array of Z-heights using `Z_Major` increments. 
3. Between each `Z_Major` level, generate intermediate Z-heights moving *upwards* using `Z_Minor` increments to handle terracing.
4. Slice the 3D model at each calculated Z-height to extract a set of 2D target polygons (the boundaries of the part at that slice).

### Phase 3.2: The Major Step-Down (Bulk Removal)
For every `Z_Major` slice:
1.  **Define Target Area:** Offset the raw 2D part slice outwards by `Stock_to_Leave`. Subtract this from the stock boundary to determine the `Machinable_Area`.
2.  **Helical Entry (The 0.9D Rule):**
    * Find the largest inscribed circle within the `Machinable_Area` using a Medial Axis Transform (MAT) or distance transform.
    * Generate a helical toolpath centered in this circle.
    * **Crucial Math:** The diameter of the helix centerline must be `Tool_D * 0.90`. This ensures the radius of the helix is `0.45 * Tool_D`. Because the tool radius is `0.5 * Tool_D`, the tool overlaps the center axis by `0.05 * Tool_D`, preventing an uncut central pillar.
    * Calculate the Z-pitch of the helix so the descent angle does not exceed `Ramp_Angle_Max`.
3.  **Adaptive Peeling (Morphing Spirals):**
    * Begin cutting outward from the bottom of the entry helix.
    * Generate concentric paths (spirals) that morph into the shape of the `Machinable_Area` boundary.
    * Enforce the `Target_RDOC`. If the distance between the current path and the boundary requires a larger cut, insert a trochoidal loop.
4.  **Trochoidal Slotting (Narrow Channels):**
    * If the tool must enter a channel narrower than `1.5 * Tool_D`, STOP forward linear motion.
    * Generate D-shaped loops. The tool moves forward by a micro-step, arcs into the material to achieve `Target_RDOC`, and arcs back through the cleared channel.
5.  **Store Footprint:** Save the boolean union of all cleared areas at this `Z_Major` level as `Cleared_Footprint_Lower`.

### Phase 3.3: The Minor Step-Up (Terracing)
After completing a `Z_Major` level, process the `Z_Minor` slices immediately above it, moving from bottom to top:
1.  **Boolean Subtraction for Terraces:**
    * Take the 2D part slice at the current `Z_Minor` height. Offset it outwards by `Stock_to_Leave`. This is the `Current_Boundary`.
    * Take the `Cleared_Footprint_Lower` from the step below. Offset this footprint INWARD by `Micro_Tolerance` (0.01mm) to prevent tool rubbing against previously cut walls.
    * **The Operation:** `Terrace_Area = Stock_Boundary - Current_Boundary - Cleared_Footprint_Lower_Offset`.
2.  **Linking (No Helix Required):**
    * Because the volume directly below the `Terrace_Area` was cleared by the major step-down, the tool does NOT need to helix.
    * Command a Rapid (G00) plunge directly into the pre-cleared void (inside `Cleared_Footprint_Lower`).
    * Command a horizontal feed into the `Terrace_Area` boundary.
3.  **Peel the Terrace:** Execute the same adaptive peeling logic (Phase 3.2, Step 3) on the `Terrace_Area`.
4.  **Update Footprint:** Union the cleared `Terrace_Area` into the `Cleared_Footprint_Lower` for the next step-up calculation.

---

## 4. Micro-Lifts and Linking (Intra-Level)
Because adaptive paths only cut in one direction (climb milling), the tool must frequently reposition across already-cleared space to start the next loop.
1.  When a cutting pass terminates, do NOT drag the tool back across the floor.
2.  Generate a **Chordal Retract**: lift the tool in the Z-axis by 0.5mm.
3.  Generate a rapid back-feed (G00 or max feedrate G01) to the start coordinate of the next cutting pass.
4.  Plunge (G01) the 0.5mm back to the cutting depth and resume.

---

## 5. Post-Processing & Geometry Optimization
Raw offset booleans will generate toolpaths with hundreds of thousands of microscopic linear segments (G01), which will data-starve older CNC controllers and cause stuttering.
1.  **Bi-Arc Fitting:** Before emitting G-Code, pass all generated toolpath coordinates through a bi-arc fitting algorithm.
2.  Convert any sequence of points that fit a curve (within a specified machining tolerance, e.g., 0.02mm) into standard G02 (Clockwise) or G03 (Counter-Clockwise) arcs.
3.  Only emit G01 for strictly linear segments.
