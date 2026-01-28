"use strict";

import { base } from "../../../geo/base.js";
import { newPoint } from "../../../geo/point.js";
import { newPolygon } from "../../../geo/polygon.js";
import {
  DEG2RAD,
  intersectMDRs,
  sectorsOverlap,
  sectorSize,
  Segment,
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

  // initialize segmentLabels array for each MDS
  for (const seg of segs) {
    for (const mds of seg.MDR) {
      mds.segmentLabels = [];
      mds.segmentLabel = null; // also clear old property just in case
    }
  }

  for (let si = 0; si < segs.length; si++) {
    let seg = segs[si];
    for (let mdsi = 0; mdsi < seg.MDR.length; mdsi++) {
      let mds = seg.MDR[mdsi];

      // For each MDS, we start a new "path search" and assign a new label.
      // This allows an MDS to be part of multiple potential paths.
      const currentLabel = nextLabel++;

      const propagate = (start_si, start_mds) => {
        let q = [{si: start_si, mds: start_mds}];
        if (start_mds.segmentLabels.includes(currentLabel)) {
            return;
        }
        start_mds.segmentLabels.push(currentLabel);

        let head = 0;
        while(head < q.length) {
            const { si, mds } = q[head++];

            // check neighbors (forward and backward)
            for (const offset of [-1, 1]) {
                const ni = (si + offset + segs.length) % segs.length;
                const nextSeg = segs[ni];
                for (const next_mds of nextSeg.MDR) {
                    if (sectorsOverlap(mds, next_mds)) {
                        if (!next_mds.segmentLabels.includes(currentLabel)) {
                            next_mds.segmentLabels.push(currentLabel);
                            q.push({si: ni, mds: next_mds});
                        }
                    }
                }
            }
        }
      };

      propagate(si, mds);
    }
  }
  return segs;
}

// given a set of segments with segmentLabels populated, select one for each segment
export function selectPaths(segs) {
    // TODO: implement graph-cut or other algorithm to select the best path
    // from the candidates in segmentLabels.
    // For now, use a simple heuristic: for each segment, find the label that
    // appears most frequently in its MDSs, and assign that.
    segs.forEach(seg => {
        const allLabels = seg.MDR.flatMap(mds => mds.segmentLabels);
        if (allLabels.length > 0) {
            const counts = allLabels.reduce((acc, label) => {
                acc[label] = (acc[label] || 0) + 1;
                return acc;
            }, {});
            const bestLabel = Object.keys(counts).reduce((a, b) => counts[a] > counts[b] ? a : b);
            seg.segmentLabel = parseInt(bestLabel, 10);
        } else {
            seg.segmentLabel = null;
        }

        // also choose the best MDS for this segment, which will be needed later
        const chosenMDS = seg.MDR.reduce(
            (mds1, mds2) =>
              sectorSize(mds1.start, mds1.stop) > sectorSize(mds2.start, mds2.stop)
                ? mds1
                : mds2,
            { start: 0, stop: 0 }
          );
        seg.chosenMDS = chosenMDS;
    });
    return segs;
}

// Assign individual path labels to segments
export function assignPaths(segs) {
  // This function now just groups segments into paths based on the
  // segmentLabel that was assigned in a previous step (e.g. selectPaths)
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

// find a safe path between two nodes, if one exists
export const findConnectingPath = (n1, n2, grid, angleStep, toolObj) => {
  if (!n1 || !n2 || !n1.segment || !n2.segment) {
    return null;
  }

  const s1 = n1.segment;
  const s2 = n2.segment;
  const p1 = s1.end;
  const p2 = s2.start;

  // if segments are already connected, return an empty path (zero-cost)
  if (p1.x === p2.x && p1.y === p2.y && p1.z === p2.z) {
    return [];
  }

  // find intersection of machinable angles for start and end segments
  const commonMDS = intersectMDRs(s1.chosenMDS ? [s1.chosenMDS] : s1.MDR, s2.chosenMDS ? [s2.chosenMDS] : s2.MDR);

  if (commonMDS.length === 0) {
    return null;
  }

  const displacement = newPoint(p2.x - p1.x, p2.y - p1.y);
  const distance = displacement.magnitude();
  const direction = displacement.clone().normalize();
  const normal = newPoint(-direction.y, direction.x);

  // create a set of points to check for collisions along the path
  const nPoints = Math.round(Math.max(distance / 0.2, 3));
  const samplePoints = [];
  for (let i = 1; i < nPoints - 1; i++) {
    samplePoints.push(newPoint(
      p1.x + (i * displacement.x) / nPoints,
      p1.y + (i * displacement.y) / nPoints,
      p1.z
    ));
  }

  // for each commonly machinable angle range, check if the path is clear
  for (const mds of commonMDS) {
    // check a few points within the shared angle range
    for (let ang = mds.start; ang <= mds.stop; ang += angleStep) {
      let pathIsClear = true;
      for (const p of samplePoints) {
        if (!isMachinable(p, normal, ang, grid, toolObj)) {
          pathIsClear = false;
          break;
        }
      }
      // if we found a clear path, create segments and return them
      if (pathIsClear) {
        const connectingSegments = [];
        const allPoints = [p1, ...samplePoints, p2];

        // Give all points the machinability info they need for the Segment constructor.
        allPoints.forEach(p => {
            if (!p._fouraxis) p._fouraxis = {};
            if (!p._fouraxis.machinability) p._fouraxis.machinability = {};
            p._fouraxis.machinability.MDR = [mds];
        });

        for (let i = 0; i < allPoints.length - 1; i++) {
            const pa = allPoints[i];
            const pb = allPoints[i+1];
            const seg = new Segment(pa, pb); // Now this should work.
            seg.chosenMDS = mds;
            seg.chosenAngle = ang;
            connectingSegments.push(seg);
        }
        return connectingSegments;
      }
    }
  }

  return null;
};
