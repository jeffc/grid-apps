// implements simultaneous four-axis machining based on
// https://haisenzhao.github.io/FourAxis/files/four-axis.pdf

import { base } from "../../../geo/base.js";
import { newPoint } from "../../../geo/point.js";
import { newPolygon } from "../../../geo/polygon.js";
import { SpatialGrid } from "../../../geo/spatial-grid.js";
import { printPoint, printPolygon } from "../../../geo/print-geom.js";

const RAD2DEG = 180 / Math.PI;
const DEG2RAD = Math.PI / 180;

// translates (x, y, z) contours in a polygon to (y, z, x) contours so that all processing
// can happen in the XY plane.
// performs transformation in-place to save on memory, and returns the polygon
// for chaining
function rotateXAxisSliced(poly) {
  poly.points = poly.points.map((pt) => {
    if (pt._fouraxis && pt._fouraxis.plane == "XY") {
      console.warn(
        "Asked to rotate points that are already in the XY plane; there's probably a bug"
      );
    }

    let [x, y, z] = [pt.x, pt.y, pt.z];
    pt.x = y;
    pt.y = z;
    pt.z = x;

    // set a flag on the points
    if (!pt._fouraxis) {
      pt._fouraxis = {};
    }
    pt._fouraxis.plane = "XY";
    return pt;
  });
  return poly;
}

// inverts the process from rotateXAxisSliced
function rotateZAxisSliced(poly) {
  poly.points = poly.points.map((pt) => {
    if (!pt._fouraxis || pt._fouraxis.plane != "XY") {
      console.warn(
        "Asked to un-rotate points from the XY plane, but there's no evidence they were ever rotated. There's probably a bug"
      );
    }

    let [x, y, z] = [pt.x, pt.y, pt.z];
    pt.x = y;
    pt.y = z;
    pt.z = x;

    // set a flag on the points
    if (!pt._fouraxis) {
      pt._fouraxis = {};
    }
    pt._fouraxis.plane = "YZ";
    return pt;
  });
  return poly;
}

// Resample a contour (set of points) into segments no longer than the given
// length. Still include all of the original points to make sure that we don't
// lose any details.
//
// ASSUMES POINTS ARE ALL ON THE SAME Z LEVEL AND ONLY LOOKS AT X AND Y COORDS
// We do this for efficiency
function resampleContour(points, spacing, closed = true) {
  // helper to resample a given segment
  // DOES NOT include p2 in the segment
  let resampleSegment = (p1, p2) => {
    let segOut = [p1];
    const totalDist = base.util.dist2D(p1, p2);

    // if the total distance between the two points is less than the specified
    // spacing, just return the original segment
    if (totalDist <= spacing) {
      return segOut;
    }

    // otherwise, do the interpolation
    const dx = p2.x - p1.x;
    const dy = p2.y - p1.y;
    const interpSteps = Math.floor(totalDist / spacing);

    // track our segment x and y so we can do additions rather than
    // multiplications
    let px = p1.x,
      py = p1.y;
    for (let s = 0; s < interpSteps; s++) {
      px += dx / interpSteps;
      py += dy / interpSteps;
      // make segments copies of p1 to preserve any attached info (including the
      // z coordinate!)
      segOut.push(p1.clone().setX(px).setY(py));
    }
    return segOut;
  };

  let out = [];
  for (let i = 0; i < points.length - 1; i++) {
    out.push(...resampleSegment(points[i], points[i + 1]));
  }

  if (closed) {
    out.push(...resampleSegment(points[points.length - 1], points[0]));
  } else {
    out.push(points[points.length - 1]);
  }

  return out;
}

// root function that performs the four-axis toolpath generation
export async function generateFourAxis(params) {
  const { sliced, onupdate, lineColor } = params;

  console.log(`Four axis slicing: ${sliced.length} slices`);

  const angleStep = 5; // User-defined angle step TODO - make this a parameter

  let slice_index = 0;
  for (const slice of sliced) {
    // update the progress bar
    onupdate(slice_index++ / sliced.length, `slice ${slice_index}`);

    // get the contours from the slicer
    let contours = slice.tops.map((t) => t.poly);

    // if there aren't any contours, there's no work to be done here
    if (!contours) {
      continue;
    }

    // if none of the contours have any polygons, there's no work to be done
    // here.
    contours = contours.filter((c) => c.points.length > 0);
    if (contours.length === 0) {
      continue;
    }

    // First, we rotate all of the contours from the YZ plane into the XY plane.
    // Many of the geometry libraries assume 2D points on the XY plane, so we just
    // transform them once at the beginning and un-transform them after the
    // computation is complete.
    contours = contours.map((poly) => rotateXAxisSliced(poly));

    // next, resample all of the contours into small segments.
    let resampledContours = contours.map((poly) => {
      let p = poly.clone();
      p.points = resampleContour(p.points, 5);
      return p;
    });

    if (slice_index == Math.floor(sliced.length / 2)) {
      console.log(slice_index);
      console.log(contours.map((p) => printPolygon(p)).join("\n"));
      console.log(resampledContours.map((p) => printPolygon(p)).join("\n"));
    }
  }

  return sliced;
}
