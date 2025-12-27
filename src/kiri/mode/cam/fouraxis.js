/**
 * @file
 *
 * This file contains the logic for 4-axis machining path generation.
 *
 * The core of this process is determining the "Machinable Direction Range" (MDR)
 * for each point on a contour. The MDR is the set of angles from which a tool
 * can access a point without colliding with other parts of the model on the same Z-slice.
 *
 * --- The Performance Challenge ---
 *
 * A naive approach to calculating the MDR involves, for each point, checking all
 * 360 possible tool approach angles. For each angle, a ray-intersection test
 * is performed against every other line segment on the slice to check for
 * collisions. This results in a high-complexity algorithm (roughly O(N^2)) that
 * is too slow for complex models.
 *
 * --- The Spatial Grid Solution ---
 *
 * To solve this, we invert the process and use a spatial acceleration data
 * structure (a 2D grid). The algorithm is as follows:
 *
 * 1. Loop through each of the 360 tool angles first.
 *
 * 2. For each angle:
 *    a. Rotate all contours on the slice by that angle.
 *    b. Project the rotated contours onto the YZ-plane, creating 2D segments.
 *    c. Build a 2D `SpatialGrid` and insert all of the 2D line segments into it.
 *
 * 3. With the grid for the current angle built, loop through each point that needs
 *    to be checked for machinability.
 *
 * 4. The `isMachinable` check now becomes much faster:
 *    a. The function receives the pre-built grid for the current angle.
 *    b. It projects the 3D `toolTip` point into a 2D `ray_origin`.
 *    c. Instead of checking against all segments, it performs a `queryRay()`
 *       on the grid. This query traverses the grid cells along the path of the
 *       tool's ray and returns only the 2D segments in those cells.
 *    d. The expensive line intersection test is then only performed on this
 *       small subset of candidate segments.
 *
 * This changes the complexity of the collision check from O(N) to O(log N) or
 * O(1) on average, resulting in a significant performance improvement.
 */

import { base } from "../../../geo/base.js";
import { newPoint } from "../../../geo/point.js";
import { newPolygon } from "../../../geo/polygon.js";
import { SpatialGrid } from "../../../geo/spatial-grid.js";

const RAD2DEG = 180 / Math.PI;
const DEG2RAD = Math.PI / 180;

/**
 * Given a point and a grid containing rotated geometry, checks if a tool can
 * access the point without colliding with other geometry on the same slice.
 */
function isMachinable(p, vnorm, angle, grid) {
  // TODO - consider actual tool geometry here
  let toolTip = p.clone().add(vnorm.clone().normalize().scale(0.1, 0.1, 0.1));
  toolTip.rotateYZ(angle * DEG2RAD);

  // project the 3D tool tip to a 2D ray origin for the grid query
  const ray_origin = { x: toolTip.z, y: toolTip.y };

  const candidates = grid.queryRay(ray_origin);

  if (candidates.length === 0) {
    return true;
  }

  const ray_direction = { dx: 1, dy: 0 }; // ray fires along the +Z axis in model space

  for (let seg of candidates) {
    // segment is already a 2D {x,y} pair
    let intersects = base.util.intersectRayLine(
      ray_origin,
      ray_direction,
      seg[0],
      seg[1]
    );

    if (self.debug_isMachinable && intersects) {
      console.log({
        msg: "intersection found",
        intersects,
        is_collision: intersects.dist > 1e-6,
      });
    }

    // if ray hits a segment that is "in front" of the tool tip, count it
    if (intersects && intersects.dist > 1e-6) {
      return false;
    }
  }

  return true;
}

function resampleClosedContour(poly, spacing) {
  if (!poly || !poly.points) return newPolygon();
  const pts = poly.points;
  if (!Array.isArray(pts) || pts.length === 0) return newPolygon();
  if (spacing <= 0) throw new Error("spacing must be > 0");

  const dist3 = (a, b) =>
    Math.sqrt(
      Math.pow(b.x - a.x, 2) + Math.pow(b.y - a.y, 2) + Math.pow(b.z - a.z, 2)
    );

  const result = [];
  let accumulatedLength = 0;
  const epsilon = 1e-9;

  for (let i = 0; i < pts.length; i++) {
    const p1 = pts[i];
    const p2 = pts[(i + 1) % pts.length];
    const segmentLength = dist3(p1, p2);

    // Add the start point of the segment, avoiding duplicates
    if (result.length === 0 || dist3(result[result.length - 1], p1) > epsilon) {
      result.push(p1);
    }

    if (segmentLength > epsilon) {
      const numSamples = Math.floor(segmentLength / spacing);

      for (let j = 1; j <= numSamples; j++) {
        const sampleDist = j * spacing;
        if (sampleDist < segmentLength - epsilon) {
          const t = sampleDist / segmentLength;
          const newPt = newPoint(
            p1.x + (p2.x - p1.x) * t,
            p1.y + (p2.y - p1.y) * t,
            p1.z + (p2.z - p1.z) * t
          );
          result.push(newPt);
        }
      }
    }
    accumulatedLength += segmentLength;
  }

  return newPolygon().addPoints(result).setClosed();
}

function convertBoolsToMDR(machinable) {
  const mdr = [];
  let start = -1;

  // iterate from 0 to 360 to properly handle sectors that end at 359
  for (let i = 0; i <= 360; i++) {
    const angle = i % 360;
    const isMachinable = machinable[angle];

    if (isMachinable && start === -1) {
      // start of a new machinable sector
      start = i;
    } else if (!isMachinable && start !== -1) {
      // end of a sector
      const sectorEnd = (i + 360 - 1) % 360;
      // differentiate zero-length and 360-length sectors
      if (start != sectorEnd || i == 360) {
        mdr.push([start, sectorEnd]);
      }
      start = -1;
    }
  }

  // if the loop finishes while in a sector, it's either a full 360 range or wraps around
  if (start !== -1) {
    mdr.push([start, 360]);
  }

  // merge ranges that cross the 0/360 boundary
  if (mdr.length > 1) {
    const first = mdr[0];
    const last = mdr[mdr.length - 1];
    if (first[0] === 0 && last[1] >= 360) {
      // merge [350, 360] and [0, 10] into [350, 10]
      last[1] = first[1];
      mdr.shift();
    }
  }

  return mdr;
}

function extrapolateMachinability(sparseMachinable, step) {
  const fullMachinable = new Array(360).fill(false);
  for (let i = 0; i < 360; i += step) {
    if (sparseMachinable[i]) {
      fullMachinable[i] = true;
    }
  }

  for (let i = 0; i < 360; i += step) {
    const currentAngle = i;
    const nextAngle = (i + step) % 360;

    // If both current and next step are machinable, fill in between
    if (fullMachinable[currentAngle] && fullMachinable[nextAngle]) {
      for (let j = 1; j < step; j++) {
        fullMachinable[(currentAngle + j) % 360] = true;
      }
    }
  }
  return fullMachinable;
}

export async function generateFourAxis(params) {
  const { sliced, onupdate, lineColor } = params;

  console.log(`Four axis slicing: ${sliced.length} slices`);

  const angleStep = 5; // User-defined angle step

  let sidx = 0;
  for (const slice of sliced) {
    onupdate(sidx++ / sliced.length, `slice ${sidx}`);
    let contours = slice.tops;
    if (!contours) {
      continue;
    }
    contours = contours.filter((c) => c.poly.points.length > 0);
    if (contours.length === 0) {
      continue;
    }

    // 1. Resample contours and pre-calculate normals
    const resampledContours = contours.map((con) =>
      resampleClosedContour(con.poly, 5)
    );
    for (const contour of resampledContours) {
      const nPoints = contour.points.length;
      if (nPoints === 0) continue;
      for (let i = 0; i < nPoints; i++) {
        const point = contour.points[i];
        const prevPoint = contour.points[(i + nPoints - 1) % nPoints];
        const nextPoint = contour.points[(i + 1) % nPoints];

        const incomingEdge = point.clone().sub(prevPoint);
        const outgoingEdge = nextPoint.clone().sub(point);

        if (incomingEdge.magnitude() === 0 || outgoingEdge.magnitude() === 0) {
          point._4axis = {
            machinable: new Array(360).fill(false),
            vnorm: newPoint(0, 0, 1),
          };
          continue;
        }

        const incomingEdgeNormal = incomingEdge
          .clone()
          .rotateYZ(-90 * DEG2RAD)
          .normalize();
        const outgoingEdgeNormal = outgoingEdge
          .clone()
          .rotateYZ(-90 * DEG2RAD)
          .normalize();
        const vertexNormal = incomingEdgeNormal
          .add(outgoingEdgeNormal)
          .normalize();

        if (isNaN(vertexNormal.x)) {
          vertexNormal.copy(outgoingEdgeNormal);
        }

        point._4axis = {
          machinable: new Array(360).fill(false),
          vnorm: vertexNormal,
        };

        if (sidx % 200 === 0) {
          slice
            .output()
            .setLayer("machinability-normals", { line: 0xff00ff })
            .addPoly(newPolygon([point, point.clone().add(vertexNormal)]));
        }
      }
    }

    // 2. Iterate by angle, building a spatial grid for each
    for (let angle = 0; angle < 360; angle += angleStep) {
      // Use angleStep here
      const bounds = {
        min: { x: Infinity, y: Infinity },
        max: { x: -Infinity, y: -Infinity },
      };
      const rotatedPolys = resampledContours.map((poly) => {
        const p = poly.clone(true).rotateYZ(angle);
        p.points.forEach((pt) => {
          bounds.min.x = Math.min(bounds.min.x, pt.z);
          bounds.min.y = Math.min(bounds.min.y, pt.y);
          bounds.max.x = Math.max(bounds.max.x, pt.z);
          bounds.max.y = Math.max(bounds.max.y, pt.y);
        });
        return p;
      });

      // pad the bounds slightly to ensure the offset toolTip is included
      const padding = 5.0;
      bounds.min.x -= padding;
      bounds.min.y -= padding;
      bounds.max.x += padding;
      bounds.max.y += padding;

      const grid = new SpatialGrid(bounds, 2.0);
      rotatedPolys.forEach((poly) => {
        const points = poly.points;
        for (let i = 0; i < points.length; i++) {
          const p1 = points[i];
          const p2 = points[(i + 1) % points.length];
          // project 3D segment to 2D before insertion
          const seg2d = [
            { x: p1.z, y: p1.y },
            { x: p2.z, y: p2.y },
          ];
          grid.insert(seg2d);
        }
      });

      // 3. Check machinability for each point at this angle
      for (const contour of resampledContours) {
        for (const point of contour.points) {
          const vnorm_rot = point._4axis.vnorm
            .clone()
            .rotateYZ(angle * DEG2RAD);

          // only check for collisions if the surface normal is generally facing the tool
          if (/*vnorm_rot.z >= 0*/ true) {
            if (isMachinable(point, point._4axis.vnorm, angle, grid)) {
              point._4axis.machinable[angle] = true; // Index by the tested rotation angle
              if (sidx % 200 === 0) {
                // Removed point_idx filter based on user preference
                const visualization_angle = (360 - angle + 90) % 360;
                const vec = newPoint(
                  0,
                  Math.cos(visualization_angle * DEG2RAD),
                  Math.sin(visualization_angle * DEG2RAD)
                );
                slice
                  .output()
                  .setLayer("machinability", { line: lineColor })
                  .addPoly(newPolygon([point, point.add(vec)]));
              }
            }
          }
        }
      }
    }

    // 4. Convert boolean arrays to MDR ranges
    for (const contour of resampledContours) {
      if (!contour.points) continue;
      for (const point of contour.points) {
        // Extrapolate machinability before converting to MDR ranges
        const fullMachinable = extrapolateMachinability(
          point._4axis.machinable,
          angleStep
        );
        point.MDR = convertBoolsToMDR(fullMachinable);
        delete point._4axis.machinable;
      }
    }

    // 5. Assemble points into continuously-machinable paths.
    // This implements a two-stage approach based on the "FourAxis" paper.
    // First, an "over-segmentation" step generates all possible maximal paths.
    // Second, a greedy selection process picks the best paths from that set.

    const largestSector = (sectors) => {
      if (!sectors || sectors.length === 0) {
        return null;
      }
      let out = sectors[0];
      for (let i = 1; i < sectors.length; i++) {
        const s = sectors[i];
        if (s[1] - s[0] > out[1] - out[0]) {
          out = s;
        }
      }
      return out;
    };

    // compute if two sectors overlap by rotating both so that sector1 starts at
    // zero
    const sectorsOverlap = (s1, s2) => {
      let [a, b] = s1;
      let [c, d] = s2;

      let bb = (b - a + 360) % 360;
      let cc = (c - a + 360) % 360;
      let dd = (d - a + 360) % 360;

      // return true if s2 starts before s1 ends, OR if s2 wraps past the zero
      // mark (where s1 starts)
      return cc <= bb || cc > dd;
    };

    const nextSector = (currentSector, mdr) => {
      if (!mdr || mdr.length === 0) {
        return null;
      }
      const intersecting = mdr.filter((s) => sectorsOverlap(currentSector, s));
      return largestSector(intersecting);
    };

    let paths = [];
    for (const contour of resampledContours) {
      if (!contour.points || contour.points.length === 0) {
        continue;
      }

      // --- Stage 1: Over-segmentation ---
      // Generate all possible maximal paths from every reachable starting point.

      const potential_paths = [];
      contour.points.forEach((startPoint, startIndex) => {
        if (!startPoint.MDR || startPoint.MDR.length === 0) {
          return;
        }

        const startSector = largestSector(startPoint.MDR);
        if (!startSector) {
          return;
        }

        // extend forward
        const forwardPath = [{ point: startPoint, sector: startSector }];
        let lastSectorFwd = startSector;
        let currentIndex = startIndex;
        while (true) {
          currentIndex = (currentIndex + 1) % contour.points.length;
          const nextPoint = contour.points[currentIndex];
          if (nextPoint === startPoint) break; // completed a full loop

          const extendingSector = nextSector(lastSectorFwd, nextPoint.MDR);
          if (extendingSector) {
            lastSectorFwd = extendingSector;
            forwardPath.push({ point: nextPoint, sector: lastSectorFwd });
          } else {
            break;
          }
        }

        // extend backward
        const backwardPath = [];
        let lastSectorBwd = startSector;
        currentIndex = startIndex;
        while (true) {
          currentIndex =
            (currentIndex - 1 + contour.points.length) % contour.points.length;
          const nextPoint = contour.points[currentIndex];
          if (
            nextPoint === startPoint ||
            nextPoint === forwardPath.peek()?.point
          )
            break;

          const extendingSector = nextSector(lastSectorBwd, nextPoint.MDR);
          if (extendingSector) {
            lastSectorBwd = extendingSector;
            backwardPath.push({ point: nextPoint, sector: lastSectorBwd });
          } else {
            break;
          }
        }

        potential_paths.push([...backwardPath.reverse(), ...forwardPath]);
      });

      // --- Stage 2: Improved Greedy Selection ---
      // Iteratively select the best path from the potential paths.

      // Add a temporary 'used' flag to each point for this stage
      contour.points.forEach((p) => {
        p._used = false;
      });

      const final_paths = [];
      while (true) {
        // Sort potential paths by the number of unused points they contain
        potential_paths.sort((a, b) => {
          const a_unused = a.filter((node) => !node.point._used).length;
          const b_unused = b.filter((node) => !node.point._used).length;
          return b_unused - a_unused;
        });

        const best_path = potential_paths[0];

        // If no more usable paths can be found, we're done
        if (
          !best_path ||
          best_path.filter((node) => !node.point._used).length === 0
        ) {
          break;
        }

        // Add the best path to our final list
        const path_to_add = best_path.filter((node) => !node.point._used);
        final_paths.push(path_to_add);

        // Mark points in the chosen path as used
        for (const node of path_to_add) {
          node.point._used = true;
        }
      }
      paths.appendAll(final_paths);

      // Clean up temporary flags
      contour.points.forEach((p) => {
        delete p._used;
      });
    }

    paths = [
      [
        { point: newPoint(0, -10, -10), sector: [90, 270] },
        { point: newPoint(0, 0, -10), sector: [90, 270] },
        { point: newPoint(0, 10, -10), sector: [90, 270] },

        { point: newPoint(0, 10, 0), sector: [0, 180] },
        { point: newPoint(0, 10, 10), sector: [0, 180] },
      ],
    ];

    // do a pass through the paths and assign angles. prefer only changing
    // angles when necessary, and prefer machining normal to the path

    // compute the path normal at each point. since paths are wound
    // counterclockwise, the normal is the average of the incoming and outgoing
    // edge angles relative to the Y axis, each rotated by 90 degrees.
    paths.forEach((path) => {
      for (let i = 0; i < path.length; i++) {
        let pt = path[i].point;
        if (!pt._path) {
          pt._path = {};
        }

        if (path.length == 1) {
          pt._path.normAngle =
            (Math.atan2(pt._4axis.vnorm.z, pt._4axis.vnorm.y) * RAD2DEG + 360) %
            360;
        } else if (i == 0) {
          const nextpt = path[i + 1].point;
          pt._path.normAngle =
            (Math.atan2(nextpt.z - pt.z, nextpt.y - pt.y) * RAD2DEG +
              360 -
              90) %
            360;
        } else if (i == path.length - 1) {
          const prevpt = path[i - 1].point;
          pt._path.normAngle =
            (Math.atan2(pt.z - prevpt.z, pt.y - prevpt.y) * RAD2DEG +
              360 -
              90) %
            360;
        } else {
          const nextpt = path[i + 1].point;
          const prevpt = path[i - 1].point;
          pt._path.normAngle =
            (((Math.atan2(nextpt.z - pt.z, nextpt.y - pt.y) * RAD2DEG +
              360 -
              90) %
              360) +
              ((Math.atan2(pt.z - prevpt.z, pt.y - prevpt.y) * RAD2DEG +
                360 -
                90) %
                360)) /
            2;
        }
        if (sidx % 200 === 0) {
          let normVec = newPoint(0, 1, 0).rotateYZ(
            pt._path.normAngle * DEG2RAD
          );
          slice
            .output()
            .setLayer("path-normals", { line: 0xffffff })
            .addPoly(newPolygon([pt, pt.clone().add(normVec)]));
        }
      }
    });

    if (sidx == 200) {
      paths.map((path) => {
        console.log(
          "Path: " +
            path
              .map((pp) => {
                let p = pp.point;
                let a = p._path === undefined ? "undef" : p._path.normAngle;
                return `(${p.y}, ${p.z}, ${a}})`;
              })
              .reduce((a, b) => a + " " + b, "")
        );
      });

      // helper function. given a point on the contour and the global rotation,
      // computes where the point ends up after that rotation.
      let rotatedPoint = (pt, angle_deg) => {
        return pt.clone().rotateYZ(angle_deg * DEG2RAD);
      };

      let chooseAngle = (p) => {
        let na = (p.point._path.normAngle - 90 + 360) % 360;
        let s = p.sector;

        // if the path normal at this point is within the machinable sector, use
        // that angle
        if (na >= s[0] && na <= s[1]) {
          return na;
        }

        // otherwise use whichever end of the machinable range is closer to the
        // normal
        const diffStart = (s[0] - na + 360) % 360;
        const diffEnd = (s[1] - na + 360) % 360;
        if (diffStart <= diffEnd) {
          return s[0];
        } else {
          return s[1];
        }
      };

      // add extra points to handle angle changes
      let camPaths = paths.map((path) => {
        let outPath = [];
        let prevAngle = chooseAngle(path[0]);
        let prevPoint = path[0].point;
        for (let i = 0; i < path.length; i++) {
          let p = path[i];
          let angle = chooseAngle(p);
          if (angle != prevAngle) {
            console.log(
              `inserting point to move between ${prevAngle} and ${angle} at ${[prevPoint.y, prevPoint.z]}`
            );
            // TODO - does this violate machinability constraints?
            outPath.push(rotatedPoint(p.point, prevAngle).setA(-prevAngle));
          }
          prevAngle = angle;
          prevPoint = p.point;
          console.log(
            `including point at ${angle} at ${[p.point.y, p.point.z]}`
          );
          outPath.push(rotatedPoint(p.point, angle).setA(-angle));
        }
        console.log(outPath);
        debugger;
        return outPath;
      });

      if (sidx == 200) {
        camPaths.map((path) => {
          console.log(
            "Augmented path: " +
              path
                .map((p) => {
                  return `(${Math.round(p.y, 2)}, ${Math.round(p.z, 2)}, ${p.a}})`;
                })
                .reduce((a, b) => a + " " + b, "")
          );
        });
        slice.camLines = camPaths.map((path) => {
          return newPolygon().addPoints(path).setOpen();
        });
      }
      //   console.log(`${[p.point.y, p.point.z, angle]} -> ${[path_point.y, path_point.z]}`);
    }
    if (sidx == 200) {
      slice
        .output()
        .setLayer("machinability-paths", {
          line: [0xff0000, 0x00ff00, 0x0000ff][Math.floor(Math.random() * 3)],
        })
        .addPoly(newPolygon().addPoints(slice.camLines).setOpen());
    }
  }
  return sliced;
}
