"use strict";

import { base } from "../../../geo/base.js";
import { newPoint } from "../../../geo/point.js";
import { newPolygon } from "../../../geo/polygon.js";
import {
  DEG2RAD,
  intersectMDRs,
  sectorsOverlap,
  sectorSize,
} from "./fouraxis-util.js";

// Compute whether a point is machinable given the grid and tool information
export function isMachinable(
  point,
  normal,
  angle,
  grid,
  toolObj,
  tool_offset = 0.1
) {
  // a point is machinable if the ray cast from the tooltip straight up (+Z)
  // never hits any other geometry. Rather than rotating the whole grid each
  // time, we instead rotate the Z vector to point where Z would be in that
  // coordinate space.
  const tooltip = newPoint(
    point.x + normal.x * tool_offset,
    point.y + normal.y * tool_offset
  );

  // TODO - add more tool geom logic here
  const toolDiam = toolObj.shaftDiameter();
  const toolRadiusOffset = newPoint(-normal.y, normal.x).scale(
    toolDiam / 2,
    toolDiam / 2,
    1
  );
  const toolEdge1 = newPoint(
    tooltip.x + toolRadiusOffset.x,
    tooltip.y + toolRadiusOffset.y
  );
  const toolEdge2 = newPoint(
    tooltip.x - toolRadiusOffset.x,
    tooltip.y - toolRadiusOffset.y
  );

  if (isNaN(tooltip.x) || isNaN(tooltip.y)) {
    debugger;
  }
  const upAxis = newPoint(0, 1).rotate(-angle * DEG2RAD);
  return !grid.rayCast(toolEdge1, upAxis) && !grid.rayCast(toolEdge2, upAxis);
}

// function to compute machinability range (MDR) and assign to each point in a
// contour
export function assignMDRs(poly, grid, angleStep, toolObj) {
  // assign some storage to each point on the contours to store machinability
  // ranges
  poly.points.forEach((p) => {
    p._fouraxis.machinability = {
      MDR: [],
      current_range_start: null,
    };
  });

  // now compute
  poly.points.forEach((p) => {
    const machinable = (p._fouraxis.machinable = []);
    for (let angle = 0; angle < 360; angle += angleStep) {
      machinable[angle] = isMachinable(
        p,
        p._fouraxis.vertex_normal,
        angle,
        grid,
        toolObj
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
        p._fouraxis.machinability.MDR.push({
          start: rangeStart,
          stop: angle - angleStep,
          segmentLabel: null,
        });
      }
    }
    if (inRange) {
      p._fouraxis.machinability.MDR.push({
        start: rangeStart,
        stop: 360 - angleStep,
        segmentLabel: null,
      });
    }

    // stitch together ranges that wrap around 0/360
    if (p._fouraxis.machinability.MDR.length > 1) {
      const first = p._fouraxis.machinability.MDR[0];
      const last = p._fouraxis.machinability.MDR.peek();
      if (first.start === 0 && last.stop === 360 - angleStep) {
        last.stop = first.stop;
        p._fouraxis.machinability.MDR.shift();
      }
    }
  });
  return poly; // for chaining, if necessary
}

// perform an exhaustive back-and-forth analysis to assign possible segment
// labels to each point (one label per MDS)
export function assignMDSLabels(segs) {
  // segment labels are unique per contour
  let nextLabel = 0;

  for (let si = 0; si < segs.length; si++) {
    let seg = segs[si];
    for (let mdsi = 0; mdsi < seg.MDR.length; mdsi++) {
      let mds = seg.MDR[mdsi];
      if (mds.segmentLabel != null) {
        continue;
      }

      // get a fresh label
      nextLabel++;
      mds.segmentLabel = nextLabel;
      let segsInPath = [seg];

      // traverse forward and assign the segment label to points which have
      // overlapping MDSs. If we encounter a point that doesn't overlap, or one
      // that already has a label (which shouldn't ever happen, unless we loop
      // back around...) stop
      // assigning.

      let traverseidx = si;
      let currentMDR = [mds];
      while (true) {
        traverseidx = (traverseidx + 1) % segs.length;
        let nextSeg = segs[traverseidx];
        let overlappingMDR = nextSeg.MDR.filter((m) =>
          currentMDR.some((s) => sectorsOverlap(s, m))
        );
        if (overlappingMDR.length == 0) {
          break;
        }

        if (overlappingMDR.some((m) => m.segmentLabel !== null)) {
          debugger; // this shouldn't happen
          break;
        }

        overlappingMDR.forEach((m) => {
          m.segmentLabel = nextLabel;
        });

        currentMDR = overlappingMDR;
        segsInPath.push(nextSeg);
      }
      traverseidx = si;
      currentMDR = [mds];
      // reverse the path so that we can push() the backwards traversal on the
      // end
      segsInPath.reverse();
      while (true) {
        traverseidx = (traverseidx - 1 + segs.length) % segs.length;
        let nextSeg = segs[traverseidx];
        let overlappingMDR = nextSeg.MDR.filter((m) =>
          currentMDR.some((s) => sectorsOverlap(s, m))
        );
        if (overlappingMDR.length == 0) {
          break;
        }

        if (overlappingMDR.some((m) => m.segmentLabel !== null)) {
          debugger; // this shouldn't happen
          break;
        }

        overlappingMDR.forEach((m) => {
          m.segmentLabel = nextLabel;
        });

        currentMDR = overlappingMDR;
        segsInPath.push(nextSeg);
      }
      // reverse the path again so we get back to a CCW winding order
      segsInPath.reverse();
    }
  }
  return segs;
}

// Assign individual path labels to segments
export function assignPaths(segs) {
  segs.forEach((s) => {
    // TODO - properly implement graph-cut-based assignment

    // choose the path label with the largest MDS
    const chosenMDS = s.MDR.reduce(
      (mds1, mds2) =>
        sectorSize(mds1.start, mds1.stop) > sectorSize(mds2.start, mds2.stop)
          ? mds1
          : mds2,
      { start: 0, stop: 0, segmentLabel: null }
    );
    s.segmentLabel = chosenMDS.segmentLabel;
    s.chosenMDS = chosenMDS;
  });

  const paths = {};
  const visited = new Array(segs.length).fill(false);

  for (let i = 0; i < segs.length; i++) {
    if (visited[i]) {
      continue;
    }
    const s = segs[i];

    const currentLabel = s.segmentLabel;
    if (currentLabel === null) {
      visited[i] = true;
      continue;
    }

    // We've found a point on a new, unvisited path.
    // First, find the absolute start of this segment by traversing backwards.
    let startIdx = i;
    while (
      segs[(startIdx - 1 + segs.length) % segs.length].segmentLabel ===
      currentLabel
    ) {
      startIdx = (startIdx - 1 + segs.length) % segs.length;
      if (startIdx === i) {
        // Full circle path, break to avoid infinite loop
        break;
      }
    }

    // Now, we are at the start of the segment. Traverse forward and collect points.
    const pathSegs = [];
    let currentIdx = startIdx;
    while (true) {
      pathSegs.push(segs[currentIdx]);
      visited[currentIdx] = true;

      const nextIdx = (currentIdx + 1) % segs.length;
      if (segs[nextIdx].segmentLabel !== currentLabel) {
        // The segment has ended.
        break;
      }
      currentIdx = nextIdx;
      if (currentIdx === startIdx) {
        // We've completed a full circle.
        break;
      }
    }
    paths[currentLabel] = pathSegs;
  }

  return { segments: segs, paths: paths };
}

export function sanityCheckPoint(p, grid, toolObj) {
  if (
    p &&
    !isMachinable(
      p,
      p._fouraxis.vertex_normal,
      p._fouraxis.chosenAngle,
      grid,
      toolObj
    )
  ) {
    console.log("POINT NOT MACHINABLE AT CHOSEN ANGLE");
    debugger;
  }
}

export function sanityCheckToolpath(tp, grid, toolObj) {
  tp.forEach((p) => {
    sanityCheckPoint(p, grid, toolObj);
  });
}

// find a safe path between two nodes, if one exists
export const findConnectingPath = (n1, n2, grid, angleStep, toolObj) => {
  if (!n1 || !n2) {
    return null;
  }
  let p1 = n1.segment.end;
  let p2 = n2.segment.start;

  let displacement = newPoint(p2.x - p1.x, p2.y - p1.y);
  let distance = displacement.magnitude();
  let direction = displacement.clone().normalize();

  // interpolate at least three points between the two
  let nPoints = Math.round(Math.max(distance / 0.2, 3));
  let samplePoints = [];
  for (let i = 0; i < nPoints; i++) {
    const newP = newPoint(
      p1.x + (i * displacement.x) / nPoints,
      p1.y + (i * displacement.y) / nPoints,
      p1.z
    );
    newP._fouraxis = {
      // assign a normal along our path
      vertex_normal: newPoint(-direction.y, direction.x),
    };
    samplePoints.push(newP);
  }
  // assign possible MDRs to our points
  const candidatePath = newPolygon(samplePoints).setOpen();
  assignMDRs(candidatePath, grid, angleStep, toolObj);

  // now force the MDRs of our start and end points to only contain the
  // chosen MDS for those points
  samplePoints[0]._fouraxis.machinability.MDR = [p1._fouraxis.chosenMDS];
  samplePoints.last()._fouraxis.machinability.MDR = [p2._fouraxis.chosenMDS];

  // now do a traversal along the path. We can't re-use our existing
  // functions because we're looking to find one specific continuous path,
  // if it exists.
  let currentMDS = p1._fouraxis.chosenMDS;
  for (let i = 1; i < samplePoints.length; i++) {
    const p = samplePoints[i];
    const chosenMDS = p._fouraxis.machinability.MDR.filter((mds) => {
      return sectorsOverlap(currentMDS, mds);
    }).reduce((mds1, mds2) => {
      if (!mds1) return mds2;
      if (!mds2) return mds1;

      return sectorSize(mds1.start, mds1.stop) >
        sectorSize(mds2.start, mds2.stop)
        ? mds1
        : mds2;
    }, null);

    // if we got to the point where we can't find a continuous MDS, give up
    // and say there's no path
    if (!chosenMDS) {
      return null;
    }
    p._fouraxis.chosenMDS = chosenMDS;
    currentMDS = chosenMDS;
  }
  return samplePoints.slice(1, samplePoints.length - 1);
};
