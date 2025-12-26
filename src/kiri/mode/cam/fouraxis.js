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

import { base } from '../../../geo/base.js';
import { newPoint } from '../../../geo/point.js';
import { newPolygon } from '../../../geo/polygon.js';

const RAD2DEG = 180 / Math.PI;
const DEG2RAD = Math.PI / 180;

/**
 * A purely 2D spatial grid for fast line segment lookups.
 * Expects all inputs (bounds, segments, points) to have {x, y} properties.
 */
class SpatialGrid {
  constructor(bounds, cellSize) {
    this.bounds = bounds;
    this.cellSize = cellSize > 0 ? cellSize : 1.0;
    this.grid = [];
    this.cols = Math.ceil((bounds.max.x - bounds.min.x) / this.cellSize) || 1;
    this.rows = Math.ceil((bounds.max.y - bounds.min.y) / this.cellSize) || 1;
    for (let i = 0; i < this.cols * this.rows; i++) {
      this.grid.push([]);
    }
  }

  _getCells(segment) {
    const [p1, p2] = segment;
    const b = {
      min: { x: Math.min(p1.x, p2.x), y: Math.min(p1.y, p2.y) },
      max: { x: Math.max(p1.x, p2.x), y: Math.max(p1.y, p2.y) }
    };

    const minX = Math.floor((b.min.x - this.bounds.min.x) / this.cellSize);
    const maxX = Math.floor((b.max.x - this.bounds.min.x) / this.cellSize);
    const minY = Math.floor((b.min.y - this.bounds.min.y) / this.cellSize);
    const maxY = Math.floor((b.max.y - this.bounds.min.y) / this.cellSize);

    const cells = [];
    for (let y = Math.max(0, minY); y <= Math.min(this.rows - 1, maxY); y++) {
      for (let x = Math.max(0, minX); x <= Math.min(this.cols - 1, maxX); x++) {
        cells.push(y * this.cols + x);
      }
    }
    return cells;
  }

  insert(segment) {
    this._getCells(segment).forEach(idx => {
      this.grid[idx].push(segment);
    });
  }

  queryRay(ray_origin) {
    // Assumes a horizontal ray in the +x direction, as is our use case.
    const candidates = new Set();
    const startX = Math.floor((ray_origin.x - this.bounds.min.x) / this.cellSize);
    const startY = Math.floor((ray_origin.y - this.bounds.min.y) / this.cellSize);

    if (startX >= this.cols || startY < 0 || startY >= this.rows) {
      return [];
    }

    // Traverse grid cells horizontally along the ray path
    for (let x = Math.max(0, startX); x < this.cols; x++) {
      const y = startY;
      const idx = y * this.cols + x;
      if (this.grid[idx]) {
        this.grid[idx].forEach(seg => candidates.add(seg));
      }
    }
    return [...candidates];
  }
}

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
    let intersects = base.util.intersectRayLine(ray_origin, ray_direction, seg[0], seg[1]);

    if (self.debug_isMachinable && intersects) {
      console.log({
        msg: "intersection found",
        intersects,
        is_collision: intersects.dist > 1e-6
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

    const dist3 = (a, b) => Math.sqrt(Math.pow(b.x - a.x, 2) + Math.pow(b.y - a.y, 2) + Math.pow(b.z - a.z, 2));

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
      mdr.push([start, i]);
      start = -1;
    }
  }

  // if the loop finishes while in a sector, it's either a full 360 range or wraps around
  if (start !== -1 && mdr.length === 0) {
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
    const resampledContours = contours.map(con => resampleClosedContour(con.poly, 10));
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
          point._4axis = { machinable: new Array(360).fill(false), vnorm: newPoint(0,0,1) };
          continue;
        }

        const incomingEdgeNormal = incomingEdge.clone().rotateYZ(-90 * DEG2RAD).normalize();
        const outgoingEdgeNormal = outgoingEdge.clone().rotateYZ(-90 * DEG2RAD).normalize();
        const vertexNormal = incomingEdgeNormal.add(outgoingEdgeNormal).normalize();

        if (isNaN(vertexNormal.x)) {
          vertexNormal.copy(outgoingEdgeNormal);
        }

        point._4axis = { machinable: new Array(360).fill(false), vnorm: vertexNormal };

        if (sidx % 200 === 0) {
          slice.output().setLayer("machinability-normals", {line: 0xFF00FF})
            .addPoly(newPolygon([point, point.clone().add(vertexNormal)]));
        }
      }
    }

    // 2. Iterate by angle, building a spatial grid for each
    for (let angle = 0; angle < 360; angle += angleStep) { // Use angleStep here
      const bounds = { min: { x: Infinity, y: Infinity }, max: { x: -Infinity, y: -Infinity } };
      const rotatedPolys = resampledContours.map(poly => {
        const p = poly.clone(true).rotateYZ(angle);
        p.points.forEach(pt => {
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
      rotatedPolys.forEach(poly => {
        const points = poly.points;
        for (let i = 0; i < points.length; i++) {
          const p1 = points[i];
          const p2 = points[(i + 1) % points.length];
          // project 3D segment to 2D before insertion
          const seg2d = [ { x: p1.z, y: p1.y }, { x: p2.z, y: p2.y } ];
          grid.insert(seg2d);
        }
      });

      // 3. Check machinability for each point at this angle
      for (const contour of resampledContours) {
        for (const point of contour.points) {
          const vnorm_rot = point._4axis.vnorm.clone().rotateYZ(angle * DEG2RAD);

          // only check for collisions if the surface normal is generally facing the tool
          if (/*vnorm_rot.z >= 0*/true) {
            if (isMachinable(point, point._4axis.vnorm, angle, grid)) {
              point._4axis.machinable[angle] = true; // Index by the tested rotation angle
              if (sidx % 200 === 0) { // Removed point_idx filter based on user preference
                const visualization_angle = (360 - angle + 90) % 360;
                const vec = newPoint(0, Math.cos(visualization_angle * DEG2RAD), Math.sin(visualization_angle * DEG2RAD));
                slice.output().setLayer("machinability", {line: lineColor})
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
        const fullMachinable = extrapolateMachinability(point._4axis.machinable, angleStep);
        point.MDR = convertBoolsToMDR(fullMachinable);
        delete point._4axis;
      }
    }

    // 5. Assemble points into continuously-machinable paths. We do this using
    // the greedy method described in the paper.

    // helper functions for MDR combination
    let sectorsIntersect = (s1, s2) => (s1[0] <= s2[1] && s2[0] <= s1[1]);
    let largestSector = (sectors) => {
      if (sectors.length == 0) {
        return null;
      }
      let out = sectors[0];
      for (let s of sectors) {
        if ((s[1] - s[0]) > (out[1] - out[0])) {
          out = s;
        }
      }
      return out;
    };
    let nextSector = (currentSector, MDR) => {
      return largestSector(MDR.filter((s) => sectorsIntersect(currentSector, s)));
    };

    let paths = [];
    for (const contour of resampledContours) {
      if (!contour.points) continue;

      let reachablePoints = 0;
      contour.points.forEach((p) => {
        if (p._4axis === undefined) {
          p._4axis = {};
        }
        p._4axis.pathLabel = null;
        p._4axis.reachable = (p.MDR.length > 0);
        if (p._4axis.reachable) reachablePoints++;
      });
      let includedPoints = 0;
      while(includedPoints < reachablePoints) {
        // pick a random reachable and unassigned point 
        let ptIndex = Math.floor(Math.random() * contour.points.length);
        while(!contour.points[ptIndex]._4axis.reachable || contour.points[ptIndex]._4axis.pathLabel != null) {
          ptIndex++;
          ptIndex %= contour.points.length;
        };

        let pt = contour.points[ptIndex];
        // In order to get from point n to point n+1, they need to have at least
        // some overlap in their MDR. This corresponds to the reality of "there
        // has to be a machining angle that we can use to move between n and
        // n+1."
        //
        // Once we move on to the next point and consider the one after that,
        // we're limited to the MDS(s) that overlapped with the first point. In
        // other words, once we're at point n+1, we can move to any machinable
        // angle allowed for that point as long as we don't cross out of the
        // machinable direction sector that we're in.
        //
        // The fully correct way to do this would be to recursively branch over
        // all possible sector overlaps and pick the longest path, but per the
        // paper we're just going to pick the biggest sector of our starting
        // point and use that.

        // pick the largest sector
        let sector = largestSector(pt.MDR);
        let forwardPath = [{point: pt, sector: sector}];
        let backwardPath = [];

        // do the forward pathing
        let i = ptIndex;
        pt = contour.points[(i + 1) % contour.points.length];
        while (pt._4axis.reachable &&
               pt._4axis.pathLabel == null && 
               nextSector(sector, pt.MDR) != null) {

          sector = nextSector(sector, pt.MDR);
          forwardPath.push({point: pt, sector: sector});
          pt._4axis.reachable = true;
          pt._4axis.pathLabel = paths.length;
          i++;
          pt = contour.points[(i + 1) % contour.points.length];
        }

        // do backward pathing
        i = ptIndex;
        pt = contour.points[(i - 1 + contour.points.length) % contour.points.length];
        sector = largestSector(pt.MDR);
        while (pt._4axis.reachable &&
               pt._4axis.pathLabel == null && 
               nextSector(sector, pt.MDR) != null) {

          backwardPath.push({point: pt, sector: sector});
          pt._4axis.reachable = true;
          pt._4axis.pathLabel = paths.length;
          sector = nextSector(sector, pt.MDR);
          i--;
          pt = contour.points[(i - 1 + contour.points.length) % contour.points.length];
        }

        backwardPath.reverse();
        let fullPath = [...backwardPath, ...forwardPath];
        includedPoints += fullPath.length;
        paths.push(fullPath);

      }
    }
    if (sidx == 200) {
      paths.map((path) => {
        console.log("Path: " + (path.map((pp) => {
          let p = pp.point;
          return `(${p.y}, ${p.z}, ${(pp.sector[0]+pp.sector[1])/2})`;
        })).reduce((a,b) => (a + " " + b), ""));
      });
    slice.camLines = paths.map((path) => {
      let pts = path.map((p) => { 
        let angle = (p.sector[0] + p.sector[1])/2;
        return p.point.clone().rotateYZ(angle*DEG2RAD).setA(angle);
      });
      return newPolygon().addPoints(pts).setOpen();
    });
    }
    if (sidx == 200) {
      slice.output().setLayer("machinability-paths", {line: [0xFF0000, 0x00FF00, 0x0000FF][Math.floor(Math.random()*3)]})
        .addPoly(newPolygon().addPoints(slice.camLines).setOpen());
    }
  }
  return sliced;
}
