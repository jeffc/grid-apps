// implements simultaneous four-axis machining based on
// https://haisenzhao.github.io/FourAxis/files/four-axis.pdf

import { base } from "../../../geo/base.js";
import { newPoint } from "../../../geo/point.js";
import { newPolygon } from "../../../geo/polygon.js";
import { SpatialGrid, fromSegments } from "../../../geo/spatial-grid.js";
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
    pt.x = z;
    pt.y = x;
    pt.z = y;

    // set a flag on the points
    if (!pt._fouraxis) {
      pt._fouraxis = {};
    }
    pt._fouraxis.plane = "YZ";
    return pt;
  });
  return poly;
}

// Calculate the normal vectors and "flatness" values at each point around a 2D
// contour. Assume counterclockwise winding and a closed polygon.
//
// returns updated points for chaining
function assignNormalsAndFlatness(points) {
  if (!points || points.length < 2) {
    console.log("Asked to assign normals to zero or one points; aborting");
    return points;
  }

  for (let i = 0; i < points.length; i++) {
    const pt = points[i];
    if (!pt._fouraxis || !pt._fouraxis.plane == "XY") {
      console.log(
        "Asked to find normal for point that isn't in the XY plane. Probably a bug."
      );
    }

    const prevPoint = points[(i - 1 + points.length) % points.length];
    const nextPoint = points[(i + 1 + points.length) % points.length];

    // compute the normals of the incoming and outgoing edge by taking the
    // difference between each and the current point (assuming CCW winding),
    // then rotating CLOCKWISE 90 degrees. Avoid the potentially costly sin()
    // and cos() calls by recognizing that rotating (a, b) by 90 degrees
    // clockwise gives (b, -a)
    const incomingEdgeNormal = newPoint(
      pt.y - prevPoint.y,
      -(pt.x - prevPoint.x)
    ).normalize();
    const outgoingEdgeNormal = newPoint(
      nextPoint.y - pt.y,
      -(nextPoint.x - pt.x)
    ).normalize();

    // compute the vertex normal by adding the two edge normals and normalizing
    // the result.
    pt._fouraxis.vertex_normal = incomingEdgeNormal
      .add(outgoingEdgeNormal)
      .normalize();
    pt._fouraxis.vertex_normal_angle =
      (Math.atan2(pt._fouraxis.vertex_normal.y, pt._fouraxis.vertex_normal.x) *
        RAD2DEG +
        360) %
      360;

    pt._INVALID = false;
    if (
      isNaN(pt._fouraxis.vertex_normal.x) ||
      isNaN(pt._fouraxis.vertex_normal.y)
    ) {
      pt._INVALID = true;
    }
    // compute the "flatness" as the absolute value of the dot product of the
    // incoming and outgoing normal vectors.
    const dotProduct = (a, b) => a.x * b.x + a.y * b.y;
    pt._fouraxis.flatness = Math.abs(
      dotProduct(incomingEdgeNormal, outgoingEdgeNormal)
    );
  }

  return points.filter((p) => p._INVALID === false);
}

// Compute whether a point is machinable given the grid and tool information
function isMachinable(point, normal, angle, grid, tool_offset = 0.1) {
  // a point is machinable if the ray cast from the tooltip straight up (+Z)
  // never hits any other geometry. Rather than rotating the whole grid each
  // time, we instead rotate the Z vector to point where Z would be in that
  // coordinate space.
  const tooltip = newPoint(
    point.x + normal.x * tool_offset,
    point.y + normal.y * tool_offset
  );

  if (isNaN(tooltip.x) || isNaN(tooltip.y)) {
    debugger;
  }
  const upAxis = newPoint(0, 1).rotate(-angle * DEG2RAD);
  const collision = grid.rayCast(tooltip, upAxis);
  if (point.debug) {
    console.log({
      p: printPoint(point),
      n: printPoint(normal),
      a: angle,
      coll: collision ? printPoint(collision) : null,
    });
  }
  return collision === null;
}

// Resample a contour (set of points) into segments no longer than the given
// length. Still include all of the original points to make sure that we don't
// lose any details.
//
// ASSUMES POINTS ARE ALL ON THE SAME Z LEVEL AND ONLY LOOKS AT X AND Y COORDS
// We do this for efficiency
function resampleContour(points, spacing, closed = true) {
  const epsilon = 1e-6;

  // helper to resample a given segment
  // DOES NOT include p2 in the segment
  let resampleSegment = (p1, p2) => {
    const totalDist = base.util.dist2D(p1, p2);
    if (totalDist <= spacing) {
      return [p1];
    }

    const interpSteps = Math.floor(totalDist / spacing);
    // if the spacing almost cleanly divides the total distance, don't
    // include the last step because it's too close to the endpoint
    if (Math.abs(totalDist - interpSteps * spacing) < epsilon) {
      // do nothing special, we'll just skip the last point
    }

    let segOut = [p1];
    const dx = (p2.x - p1.x) / totalDist;
    const dy = (p2.y - p1.y) / totalDist;

    for (let i = 1; i < interpSteps; i++) {
      const dist = i * spacing;
      segOut.push(
        newPoint(p1.x + dx * dist, p1.y + dy * dist, p1.z).annotate({
          _fouraxis: structuredClone(p1._fouraxis),
        })
      );
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

    // if none of the contours have any polygons with area, there's no work to
    // be done here.
    contours = contours.filter((c) => c.points.length > 2);
    if (contours.length === 0) {
      continue;
    }

    // First, we rotate all of the contours from the YZ plane into the XY plane.
    // Many of the geometry libraries assume 2D points on the XY plane, so we just
    // transform them once at the beginning and un-transform them after the
    // computation is complete.
    contours = contours.map((poly) => rotateXAxisSliced(poly.clone(true)));

    contours.forEach((poly) => poly.setCounterClockwise());

    // next, resample all of the contours into small segments.
    let resampledContours = contours
      .map((poly) => {
        let p = poly.clone(true, [], ["_fouraxis"]);
        p.points = resampleContour(p.points, 0.5);
        return p;
      })
      .filter((poly) => poly.points.length > 2);

    // Compute and add in the normal vectors for each contour
    resampledContours.forEach((poly) => {
      poly.points = assignNormalsAndFlatness(poly.points);
    });

    if (slice_index % 10 == 0) {
      resampledContours.forEach((poly) => {
        let pol = newPolygon(poly.points.map((p) => p.clone(["_fouraxis"])));
        slice
          .output()
          .setLayer("contours", { line: 0xff0000 })
          .addPoly(rotateZAxisSliced(pol));
      });
    }

    if (slice_index % 10 == 0) {
      resampledContours.forEach((poly) =>
        poly.points.forEach((p, i) => {
          const viz_p = newPoint(p.z, p.x, p.y);
          slice
            .output()
            .setLayer("machinability-normals", { line: 0xff00ff })
            .addPoly(
              newPolygon([
                viz_p,
                newPoint(
                  viz_p.x,
                  viz_p.y + (i == 0 ? 3 : 1) * p._fouraxis.vertex_normal.x,
                  viz_p.z + (i == 0 ? 3 : 1) * p._fouraxis.vertex_normal.y
                ),
              ])
            );
        })
      );
    }

    // assign some storage to each point on the contours to store machinability
    // ranges
    resampledContours.forEach((poly) => {
      poly.points.forEach((p) => {
        p._fouraxis.machinability = {
          MDR: [],
          current_range_start: null,
        };
      });
    });

    // generate the segment collision grid based on the original input
    // contours, since the extra resolution of the resampled contours doesn't
    // gain us anything
    let segments = contours
      .map((c) =>
        // assume that contours are closed
        c.points.map((p, i, pts) => [
          pts[i],
          pts[(i + 1 + pts.length) % pts.length],
        ])
      )
      .flat();
    let grid = fromSegments(segments, 2.0, 10.0);

    // Iterate by angle to set machinability for each point
    resampledContours.forEach((poly) => {
      poly.points.forEach((p, i) => {
        if (slice_index === 10 && i == 119) {
          console.log(`DEBUGGING POINT ${i}: (${[p.x, p.y]})`);
          p.debug = true;
        }
      });
      poly.points.forEach((p) => {
        const machinable = (p._fouraxis.machinable = []);
        for (let angle = 0; angle < 360; angle += angleStep) {
          machinable[angle] = isMachinable(
            p,
            p._fouraxis.vertex_normal,
            angle,
            grid
          );
        }
        if (p.debug) {
          console.log(p._fouraxis.machinable.map((m) => (m ? 1 : 0)).join(""));
        }

        let inRange = false;
        let rangeStart = 0;
        for (let angle = 0; angle < 360; angle += angleStep) {
          if (machinable[angle] && !inRange) {
            inRange = true;
            rangeStart = angle;
          } else if (!machinable[angle] && inRange) {
            inRange = false;
            p._fouraxis.machinability.MDR.push([rangeStart, angle - angleStep]);
          }
        }
        if (inRange) {
          p._fouraxis.machinability.MDR.push([rangeStart, 360 - angleStep]);
        }

        // stitch together ranges that wrap around 0/360
        if (p._fouraxis.machinability.MDR.length > 1) {
          const first = p._fouraxis.machinability.MDR[0];
          const last = p._fouraxis.machinability.MDR.peek();
          if (first[0] === 0 && last[1] === 360 - angleStep) {
            last[1] = first[1];
            p._fouraxis.machinability.MDR.shift();
          }
        }
      });
    });

    if (slice_index % 10 == 0) {
      let visualizeMachinabilityAngle = (p, angle) => {
        const viz_p = newPoint(p.z, p.x, p.y);
        slice
          .output()
          .setLayer(`machinability`, { line: 0xffffff })
          .addPoly(
            newPolygon([
              viz_p,
              newPoint(
                viz_p.x,
                viz_p.y + Math.cos((90 - angle) * DEG2RAD),
                viz_p.z + Math.sin((90 - angle) * DEG2RAD)
              ),
            ])
          );
      };

      resampledContours.forEach((poly) =>
        poly.points.forEach((p, i) => {
          p._fouraxis.machinability.MDR.forEach((mdr) => {
            // in order to handle wrap-around / zero-crossings, offset the MDR
            // so that our loop iterator always remains positive
            let viz_offset = (360 - mdr[0] + 360) % 360;
            let viz_limit = (mdr[1] + viz_offset) % 360;
            for (let a = 0; a < viz_limit; a++)
              if ((a - viz_offset) % angleStep == 0) {
                visualizeMachinabilityAngle(p, (a - viz_offset + 360) % 360);
              }
          });
        })
      );
    }
  }
  return sliced;
}
