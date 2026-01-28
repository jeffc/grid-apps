"use strict";

import { base } from "../../../geo/base.js";
import { newPoint } from "../../../geo/point.js";

export const RAD2DEG = 180 / Math.PI;
export const DEG2RAD = Math.PI / 180;

// Represents a path segment
export class Segment {
  // start and end are Point objects
  constructor(start, end) {
    this.start = start;
    this.end = end;
    this.chosenAngle = null;
    this.chosenMDS = null;

    // compute the MDR that's the intersection of the two endpoint MDRs
    this.MDR = intersectMDRs(
      this.start._fouraxis.machinability.MDR,
      this.end._fouraxis.machinability.MDR
    );
  }
}

// Represents a path segment
export class MDR {
  // start and end are Point objects
  constructor(start, end) {
    this.start = start;
    this.end = end;

    // compute the MDR that's the intersection of the two endpoint MDRs
    this.MDR = intersectMDRs(
      this.start._fouraxis.machinability.MDR,
      this.end._fouraxis.machinability.MDR
    );
  }
}

export function intersectMDRs(mdr1, mdr2) {
  const events = [];

  const processMDR = (mdr) => {
    for (const seg of mdr) {
      if (seg.start <= seg.stop) {
        events.push({ type: "start", val: seg.start });
        events.push({ type: "end", val: seg.stop });
      } else {
        // Wrap-around case: split into two segments
        events.push({ type: "start", val: seg.start });
        events.push({ type: "end", val: 360 });
        events.push({ type: "start", val: 0 });
        events.push({ type: "end", val: seg.stop });
      }
    }
  };

  processMDR(mdr1);
  processMDR(mdr2);

  // Sort events by value. If values are equal, process 'start' before 'end'
  // to correctly handle adjacent segments.
  events.sort((a, b) => a.val - b.val || (a.type === "start" ? -1 : 1));

  const result = [];
  let count = 0;
  let intersectionStart = null;

  for (const event of events) {
    if (event.type === "start") {
      if (count === 1) {
        // Transitioning from 1 to 2 active segments means an intersection starts
        intersectionStart = event.val;
      }
      count++;
    } else {
      // event.type === 'end'
      if (count === 2) {
        // Transitioning from 2 to 1 active segments means an intersection ends
        if (intersectionStart !== null && intersectionStart < event.val) {
          result.push([intersectionStart, event.val]);
        }
        intersectionStart = null;
      }
      count--;
    }
  }

  // Post-processing to merge segments that may have been split at 0/360
  if (result.length > 1) {
    const first = result[0];
    const last = result[result.length - 1];
    if (first[0] === 0 && last[1] === 360) {
      const newStart = last[0];
      const newEnd = first[1];
      // Remove the two segments that will be merged
      result.shift();
      result.pop();
      // Add the new merged segment
      result.push([newStart, newEnd]);
    }
  }

  return result.map((s) => {
    return { start: s[0], stop: s[1], segmentLabel: null };
  });
}

// translates (x, y, z) contours in a polygon to (y, z, x) contours so that all processing
// can happen in the XY plane.
// performs transformation in-place to save on memory, and returns the polygon
// for chaining
export function rotateXAxisSliced(poly) {
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
export function rotateZAxisSliced(poly) {
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

// Calculate the normal vectors at each point around a 2D
// contour. Assume counterclockwise winding and a closed polygon.
//
// returns updated points for chaining
export function assignNormals(points) {
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
  }

  return points.filter((p) => p._INVALID === false);
}

// Resample a contour (set of points) into segments no longer than the given
// length. Still include all of the original points to make sure that we don't
// lose any details.
//
// ASSUMES POINTS ARE ALL ON THE SAME Z LEVEL AND ONLY LOOKS AT X AND Y COORDS
// We do this for efficiency
export function resampleContour(points, spacing, closed = true) {
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

// Helper function: compute if two sectors overlap by rotating both so that
// sector1 starts at zero
export function sectorsOverlap(s1, s2) {
  let [a, b] = [s1.start, s1.stop];
  let [c, d] = [s2.start, s2.stop];

  let bb = (b - a + 360) % 360;
  let cc = (c - a + 360) % 360;
  let dd = (d - a + 360) % 360;

  // return true if s2 starts before s1 ends, OR if s2 wraps past the zero
  // mark (where s1 starts)
  return cc <= bb || cc > dd;
}

// helper, determines sector size given start and stop in degrees
export function sectorSize(start, stop) {
  return (stop - start + 360) % 360;
}
