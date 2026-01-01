// implements simultaneous four-axis machining based on
// https://haisenzhao.github.io/FourAxis/files/four-axis.pdf

// REMAINING TASKS
//  - Better 2D tool geometry handling
//  - 3D tool geometry handling
//  - Graph-cut segment decomposition
//  - Better toolpath-to-toolpath pathfinding
//  - CAM generation improvements
//  - General cleanup

import { base } from "../../../geo/base.js";
import { newPoint } from "../../../geo/point.js";
import { newPolygon } from "../../../geo/polygon.js";
import { SpatialGrid, fromSegments } from "../../../geo/spatial-grid.js";
import { Graph } from "../../../geo/graph.js";
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
function isMachinable(point, normal, angle, grid, toolObj, tool_offset = 0.1) {
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
  return (
    !grid.rayCast(toolEdge1, upAxis) &&
    !grid.rayCast(toolEdge2, upAxis)
  );
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

// function to compute machinability range (MDR) and assign to each point in a
// contour
function assignMDRs(poly, grid, angleStep, toolObj) {
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
// Helper function: compute if two sectors overlap by rotating both so that
// sector1 starts at zero
function sectorsOverlap(s1, s2) {
  let [a, b] = [s1.start, s1.stop];
  let [c, d] = [s2.start, s2.stop];

  let bb = (b - a + 360) % 360;
  let cc = (c - a + 360) % 360;
  let dd = (d - a + 360) % 360;

  // return true if s2 starts before s1 ends, OR if s2 wraps past the zero
  // mark (where s1 starts)
  return cc <= bb || cc > dd;
}

// perform an exhaustive back-and-forth analysis to assign possible segment
// labels to each point (one label per MDS)
function assignMDSLabels(poly) {
  // segment labels are unique per contour
  let nextLabel = 0;

  const pts = poly.points;
  for (let pi = 0; pi < poly.points.length; pi++) {
    let pt = pts[pi];
    for (let mdsi = 0; mdsi < pt._fouraxis.machinability.MDR.length; mdsi++) {
      let mds = pt._fouraxis.machinability.MDR[mdsi];
      if (mds.segmentLabel != null) {
        continue;
      }

      // get a fresh label
      nextLabel++;
      mds.segmentLabel = nextLabel;
      let pointsOnPath = [pt];

      // traverse forward and assign the segment label to points which have
      // overlapping MDSs. If we encounter a point that doesn't overlap or that
      // has two disjoint overlaps, or one that already has a label (which
      // shouldn't ever happen...) stop assigning.
      let traverseidx = pi;
      let currentMDS = mds;
      while (true) {
        traverseidx = (traverseidx + 1) % pts.length;
        let nextPt = pts[traverseidx];
        let overlappingMDSs = nextPt._fouraxis.machinability.MDR.filter((m) =>
          sectorsOverlap(currentMDS, m)
        );
        if (overlappingMDSs.length != 1) {
          break;
        }
        let nextMDS = overlappingMDSs[0];
        if (nextMDS.segmentLabel !== null) {
          break;
        }
        // If another MDS on this point is already part of the current path,
        // then this is an invalid merge of two disjoint paths.
        if (
          nextPt._fouraxis.machinability.MDR.some(
            (m) => m !== nextMDS && m.segmentLabel === nextLabel
          )
        ) {
          break;
        }
        nextMDS.segmentLabel = nextLabel;
        currentMDS = nextMDS;
        pointsOnPath.push(nextPt);
      }
      traverseidx = pi;
      currentMDS = mds;
      // reverse the path so that we can push() the backwards traversal on the
      // end
      pointsOnPath.reverse();
      while (true) {
        traverseidx = (traverseidx - 1 + pts.length) % pts.length;
        let nextPt = pts[traverseidx];
        let overlappingMDSs = nextPt._fouraxis.machinability.MDR.filter((m) =>
          sectorsOverlap(currentMDS, m)
        );
        if (overlappingMDSs.length != 1) {
          break;
        }
        let nextMDS = overlappingMDSs[0];
        if (nextMDS.segmentLabel !== null) {
          break;
        }
        // If another MDS on this point is already part of the current path,
        // then this is an invalid merge of two disjoint paths.
        if (
          nextPt._fouraxis.machinability.MDR.some(
            (m) => m !== nextMDS && m.segmentLabel === nextLabel
          )
        ) {
          break;
        }
        nextMDS.segmentLabel = nextLabel;
        currentMDS = nextMDS;
        pointsOnPath.push(nextPt);
      }
      // reverse the path again so we get back to a CCW winding order
      pointsOnPath.reverse();
    }
  }
  return poly;
}

// helper, determines sector size given start and stop in degrees
function sectorSize(start, stop) {
  return (stop - start + 360) % 360;
}

// Assign individual path labels to points in a polygon
function assignPaths(poly) {
  const pts = poly.points;
  pts.forEach((p) => {
    // TODO - properly implement graph-cut-based assignment

    // choose the path label with the largest MDS
    const chosenMDS = p._fouraxis.machinability.MDR.reduce(
      (mds1, mds2) =>
        sectorSize(mds1.start, mds1.stop) > sectorSize(mds2.start, mds2.stop)
          ? mds1
          : mds2,
      { start: 0, stop: 0, segmentLabel: null }
    );
    p._fouraxis.segmentLabel = chosenMDS.segmentLabel;
    p._fouraxis.chosenMDS = chosenMDS;
  });

  const paths = {};
  const visited = new Array(pts.length).fill(false);

  for (let i = 0; i < pts.length; i++) {
    if (visited[i]) {
      continue;
    }

    const currentLabel = pts[i]._fouraxis.segmentLabel;
    if (currentLabel === null) {
      visited[i] = true;
      continue;
    }

    // We've found a point on a new, unvisited path.
    // First, find the absolute start of this segment by traversing backwards.
    let startIdx = i;
    while (
      pts[(startIdx - 1 + pts.length) % pts.length]._fouraxis.segmentLabel ===
      currentLabel
    ) {
      startIdx = (startIdx - 1 + pts.length) % pts.length;
      if (startIdx === i) {
        // Full circle path, break to avoid infinite loop
        break;
      }
    }

    // Now, we are at the start of the segment. Traverse forward and collect points.
    const pathPoints = [];
    let currentIdx = startIdx;
    while (true) {
      pathPoints.push(pts[currentIdx]);
      visited[currentIdx] = true;

      const nextIdx = (currentIdx + 1) % pts.length;
      if (pts[nextIdx]._fouraxis.segmentLabel !== currentLabel) {
        // The segment has ended.
        break;
      }
      currentIdx = nextIdx;
      if (currentIdx === startIdx) {
        // We've completed a full circle.
        break;
      }
    }
    paths[currentLabel] = pathPoints;
  }

  if (!poly._fouraxis) {
    poly._fouraxis = {};
  }
  poly._fouraxis.paths = paths;

  return poly;
}

function sanityCheckPoint(p, grid, toolObj) {
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

function sanityCheckToolpath(tp, grid, toolObj) {
  tp.forEach((p) => {
    sanityCheckPoint(p, grid, toolObj);
  });
}

// root function that performs the four-axis toolpath generation
export async function generateFourAxis(params) {
  const { sliced, onupdate, toolObj, lineColor } = params;

  console.log(`Four axis slicing: ${sliced.length} slices`);

  const angleStep = 5; // User-defined angle step TODO - make this a parameter

  let slice_index = 0;
  let totalRotations = 0;
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

    // add the start point to the end of the polygon's path, since what we
    // really care about are path segments (not their endpoints)
    contours.forEach((poly) => poly.points.push(poly.points[0]));

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

    // Set machinability for each point
    resampledContours.forEach((poly) =>
      assignMDRs(poly, grid, angleStep, toolObj)
    );

    // assign segment candidate labels
    resampledContours.forEach((poly) => assignMDSLabels(poly));

    // group points into paths by segment label
    resampledContours.forEach((poly) => assignPaths(poly));

    // get all of the paths in the current slice and compute an ordering for
    // them
    let allPaths = resampledContours
      .map((poly) =>
        Object.entries(poly._fouraxis.paths).map(([pathidx, path]) => {
          return {
            name: `${poly.id}-${pathidx}`,
            points: path,
            start: path[0],
            end: path[path.length - 1],
          };
        })
      )
      .flat();

    // compute segment connectivity graph. The graph is logically undirected,
    // but we represent it using a directed graph so that we can annotate
    // directed edges with their path data.
    let toolpathGraph = new Graph();
    // generate the list of nodes for the graph
    const toolpathPairs = allPaths.map((path) => [
      {
        name: `${path.name}-start`,
        path: path,
        start: true,
        point: path.start,
      },
      {
        name: `${path.name}-end`,
        path: path,
        start: false,
        point: path.end,
      },
    ]);
    const toolpathNodes = toolpathPairs.flat();
    toolpathNodes.forEach((n) => toolpathGraph.addNode(n.name, n));
    toolpathGraph.addNode("RETRACT", {
      name: "retract",
      path: null,
      start: null,
      point: null,
    });

    toolpathNodes.forEach((node) => {
      toolpathGraph.addEdge(node.name, "RETRACT", 1e5); // from path to retract
      toolpathGraph.addEdge("RETRACT", node.name, 1e5); // from retract to path
    });

    // find a safe path between two nodes, if one exists
    const findConnectingPath = (n1, n2) => {
      if (!n1 || !n2) {
        return null;
      }
      let p1 = n1.point;
      let p2 = n2.point;

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
      samplePoints.last()._fouraxis.machinability.MDR = [
        p2._fouraxis.chosenMDS,
      ];

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

    for (let i = 0; i < toolpathNodes.length; i++) {
      const n1 = toolpathNodes[i];
      for (let j = 0; j < i; j++) {
        const n2 = toolpathNodes[j];

        if (n1.path.name == n2.path.name) {
          const pathBetween = structuredClone(n1.path.points);
          const pathBetweenRev = structuredClone(n1.path.points).reverse();
          // traversing the path the first time is 0-weight because it's
          // something we have to do, but the "re-traversal cost" is the length
          // of the path.
          const pathLength = n1.path.points.map((pt, i, pts) => {
            if (i == 0) {
              return 0;
            }
            return base.util.dist2D(pt, pts[i-1]);
          }).reduce((a, b) => a+b, 0);

          if (n1.start) {
            toolpathGraph.addEdge(n1.name, n2.name, 0, { path: pathBetween }, pathLength);
            toolpathGraph.addEdge(n2.name, n1.name, 0, {
              path: pathBetweenRev,
            }, pathLength);
          } else {
            toolpathGraph.addEdge(n1.name, n2.name, 0, {
              path: pathBetweenRev,
            }, pathLength);
            toolpathGraph.addEdge(n2.name, n1.name, 0, { path: pathBetween }, pathLength);
          }
        } else {
          // check if there's a safe machinable path between n1 and n2
          const forwardPath = findConnectingPath(n1, n2);
          if (forwardPath) {
            const forwardPathLength = n1.path.points.map((pt, i, pts) => {
              if (i == 0) {
                return 0;
              }
              return base.util.dist2D(pt, pts[i-1]);
            }).reduce((a, b) => a+b, 0);
            toolpathGraph.addEdge(
              n1.name,
              n2.name,
              base.util.dist2D(n1.point, n2.point),
              {
                path: { points: forwardPath },
              },
              forwardPathLength
            );
          }

          const reversePath = findConnectingPath(n2, n1);
          if (reversePath) {
            const reversePathLength = n1.path.points.map((pt, i, pts) => {
              if (i == 0) {
                return 0;
              }
              return base.util.dist2D(pt, pts[i-1]);
            }).reduce((a, b) => a+b, 0);
            toolpathGraph.addEdge(
              n2.name,
              n1.name,
              base.util.dist2D(n2.point, n1.point),
              {
                path: { points: reversePath },
              },
              reversePathLength
            );
          }
        }
      }
    }

    // now find a traversal order
    let tspPath = toolpathGraph.findPathTSP("RETRACT");
    let nodeTraversalOrder = tspPath.path;
    let edgeTraversalOrder = tspPath.edges;

    // create a toolpath without explicit angles assigned
    let toolpath = edgeTraversalOrder
      .map((e) => {
        if (e.from == "RETRACT" || e.to == "RETRACT") {
          return [null]; // Use null as a marker for retraction
        }
        const pathData = e.edgeData.data.path;
        return pathData.points || pathData || [];
      })
      .flat();


    // compute the actual machining angles along the toolpath. start by choosing
    // the middle of each point's MDS, then do laplacian smoothing until the
    // total angle variance converges within 1 degree.
    toolpath.forEach((p) => {
      if (!p) {
        // this is a retract point
        return;
      }
      const mds = p._fouraxis.chosenMDS;
      p._fouraxis.chosenAngle =
        (mds.start + sectorSize(mds.start, mds.stop) / 2) % 360;
    });

    const angleInMDS = (mds, a) => {
      if (mds.start < mds.stop) {
        return mds.start <= a && a <= mds.stop;
      } else {
        return mds.start <= a || a <= mds.stop;
      }
    };

    // compute whether we should move to each point with a clockwise or
    // counterclockwise rotation of the A axis. this is needed for interpolation
    toolpath.forEach((p, i, tp) => {
      if (!p || i == 0) {
        return;
      }
      let prevP = tp[i - 1];
      if (!prevP) {
        return;
      }

      let thisA = p._fouraxis.chosenAngle;
      let prevA = prevP._fouraxis.chosenAngle;

      const ccw_arc_len = sectorSize(prevA, thisA);
      const is_ccw_short = ccw_arc_len <= 180;
      const bisector = (prevA + ccw_arc_len / 2 + 360) % 360;

      // Check if the midpoint of the direct CCW arc is valid for both points
      const ccw_path_is_valid =
        angleInMDS(p._fouraxis.chosenMDS, bisector) &&
        angleInMDS(prevP._fouraxis.chosenMDS, bisector);

      if (ccw_path_is_valid) {
        // The direct CCW path is clear, so choose the shorter of the two rotational paths.
        p._fouraxis.ccw = is_ccw_short;
      } else {
        // The direct CCW path is blocked, so we MUST go the other way (CW).
        p._fouraxis.ccw = false;
      }
    });

    /*
    let angleDelta = 0;
    do {
      for (let i = 0; i < toolpath.length; i++) {
        let p = toolpath[i];
        if (!p) {
          continue; // retract point
        }
        
        let angles = [p._fouraxis.chosenAngle];
        if (i > 0 && toolpath[i - 1]) {
          angles.push(toolpath[i - 1]._fouraxis.chosenAngle);
        }
        if (i < toolpath.length - 1 && toolpath[i + 1]) {
          angles.push(toolpath[i + 1]._fouraxis.chosenAngle);
        }

        let sum_x = 0;
        let sum_y = 0;
        for (const angle of angles) {
            sum_x += Math.cos(angle * DEG2RAD);
            sum_y += Math.sin(angle * DEG2RAD);
        }

        p._fouraxis.newAngle = (Math.atan2(sum_y, sum_x) * RAD2DEG + 360) % 360;
      }

      angleDelta = 0;
      const angleInSector = (mds, a) =>
        (a - mds.start + 360) % 360 < (mds.stop - mds.start + 360) % 360;
      toolpath.forEach((p) => {
        if (!p) {
          return;
        }
        let newAngle = p._fouraxis.newAngle;
        let mds = p._fouraxis.chosenMDS;
        if (!angleInSector(mds, newAngle)) {
          const distToStart = Math.min(
            Math.abs(mds.start - newAngle),
            360 - Math.abs(mds.start - newAngle)
          );
          const distToStop = Math.min(
            Math.abs(mds.stop - newAngle),
            360 - Math.abs(mds.stop - newAngle)
          );
          newAngle = distToStart < distToStop ? mds.start : mds.stop;
        }
        angleDelta += Math.abs(newAngle - p._fouraxis.chosenAngle);
        p._fouraxis.chosenAngle = newAngle;
        p._fouraxis.newAngle = undefined;
      });
    } while (angleDelta > 1);
    */
    sanityCheckToolpath(toolpath, grid, toolObj);

    // iterate over the toolpath and approximate the actual distance travelled
    // by the tool between the given points (taking into account the rotation).
    // If that distance is larger than 0.5 work units, interpolate steps
    let interpolatedToolpath = [];
    let prevPoint = null;
    for (let i = 0; i < toolpath.length; i++) {
      const p = toolpath[i];
      if (!p || !prevPoint) {
        interpolatedToolpath.push(p);
        prevPoint = p;
        continue;
      }

      const p_angle = p._fouraxis.chosenAngle;
      const prev_angle = prevPoint._fouraxis.chosenAngle;

      let diff = p_angle - prev_angle;
      if (p._fouraxis.ccw) {
        if (diff < 0) diff += 360;
      } else {
        if (diff > 0) diff -= 360;
      }
      const da_rad = Math.abs(diff * DEG2RAD);

      const mag = (pp) => Math.sqrt(pp.x * pp.x + pp.y * pp.y);
      const dArc = (da_rad * (mag(p) + mag(prevPoint))) / 2;
      if (dArc > 0.5) {
        // TODO - make configurable?
        let segs = Math.max(Math.floor(dArc / 0.5), 1);
        let interpX = (p.x - prevPoint.x) / segs;
        let interpY = (p.y - prevPoint.y) / segs;
        let interpA = diff / segs;
        let interpPoint = structuredClone(prevPoint);
        const segmentDir = newPoint(
          p.x - prevPoint.x,
          p.y - prevPoint.y
        ).normalize();
        for (let step = 0; step < segs - 1; step++) {
          interpPoint.x += interpX;
          interpPoint.y += interpY;
          interpPoint._fouraxis.chosenAngle += interpA;
          interpPoint._fouraxis.vertex_normal = newPoint(
            segmentDir.y,
            -segmentDir.x
          );
          sanityCheckPoint(interpPoint, grid, toolObj);
          interpolatedToolpath.push(structuredClone(interpPoint));
        }
      }
      interpolatedToolpath.push(p);
      prevPoint = p;
    }

    // re-calculate ccw and totalRotations for the final, interpolated toolpath
    interpolatedToolpath.forEach((p, i, tp) => {
      if (!p) {
        return;
      }
      if (i == 0) {
        p._fouraxis.totalRotations = totalRotations;
        return;
      }
      let prevP = tp[i - 1];
      if (!prevP) {
        p._fouraxis.totalRotations = totalRotations;
        return;
      }

      let thisA = p._fouraxis.chosenAngle;
      let prevA = prevP._fouraxis.chosenAngle;

      // for interpolated points, we assume the shortest path is the correct one
      // because the MDS validity check was done on the original points.
      const ccw_arc_len = sectorSize(prevA, thisA);
      p._fouraxis.ccw = ccw_arc_len <= 180;

      if (prevA > thisA && p._fouraxis.ccw) {
        totalRotations++;
      } else if (thisA > prevA && !p._fouraxis.ccw) {
        totalRotations--;
      }
      p._fouraxis.totalRotations = totalRotations;
    });

    sanityCheckToolpath(interpolatedToolpath, grid, toolObj);

    let finalToolpath = interpolatedToolpath.map((p) => {
      if (!p) {
        return null;
      }
      return newPoint(p.z, p.x, p.y)
        .rotateYZ(p._fouraxis.chosenAngle * DEG2RAD)
        .setA(-p._fouraxis.chosenAngle - p._fouraxis.totalRotations * 360);
    });

    slice.camLines = [];
    let line = [];
    finalToolpath.forEach((p) => {
      if (p) {
        // if p is a valid point, add it to the current line
        line.push(p);
      } else {
        // if p is null, it's a retraction, so finalize the current line
        if (line.length > 0) {
          slice.camLines.push(newPolygon(line).setOpen());
          line = [];
        }
      }
    });
    if (line.length > 0) {
      slice.camLines.push(newPolygon(line).setOpen());
    }

    // visualizations for debugging
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

      if (slice_index % 10 == 0) {
        toolpath.forEach((p) => {
          if (!p) return;
          const viz_p = newPoint(p.z, p.x, p.y);
          const machiningAngle = p._fouraxis.chosenAngle * DEG2RAD;
          const [vx, vy] = [
            -Math.sin(-machiningAngle),
            Math.cos(-machiningAngle),
          ];
          slice
            .output()
            .setLayer("machinability-angle", { line: 0x00ffff })
            .addPoly(
              newPolygon([viz_p, newPoint(viz_p.x, viz_p.y + vx, viz_p.z + vy)])
            );
        });
      }

      resampledContours.forEach((poly) =>
        poly.points.forEach((p, i) => {
          p._fouraxis.machinability.MDR.forEach((mdr) => {
            // in order to handle wrap-around / zero-crossings, offset the MDR
            // so that our loop iterator always remains positive
            let viz_offset = (360 - mdr.start + 360) % 360;
            let viz_limit = (mdr.stop + viz_offset) % 360;
            for (let a = 0; a < viz_limit; a++)
              if ((a - viz_offset) % angleStep == 0) {
                visualizeMachinabilityAngle(p, (a - viz_offset + 360) % 360);
              }
          });
        })
      );
    }

    if (slice_index % 10 == 0) {
      const colors = [0xff0000, 0x00ff00, 0x0000ff];
      resampledContours.forEach((poly) => {
        Object.entries(poly._fouraxis.paths).forEach(([pathi, path]) => {
          let viz_path_pts = [];
          path.forEach((p) => {
            viz_path_pts.push(newPoint(p.z, p.x, p.y));
          });
          slice
            .output()
            .setLayer(`segments-${pathi % colors.length}`, {
              line: colors[pathi % colors.length],
            })
            .addPoly(newPolygon(viz_path_pts).setOpen());
        });
      });
    }
  }
  return sliced;
}
