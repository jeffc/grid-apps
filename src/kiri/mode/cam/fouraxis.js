// implements simultaneous four-axis machining based on
// https://haisenzhao.github.io/FourAxis/files/four-axis.pdf

"use strict";

import { base } from "../../../geo/base.js";
import { newPoint } from "../../../geo/point.js";
import { newPolygon } from "../../../geo/polygon.js";
import { SpatialGrid, fromSegments } from "../../../geo/spatial-grid.js";
import { Graph } from "../../../geo/graph.js";
import { printPoint, printPolygon } from "../../../geo/print-geom.js";
import {
  DEG2RAD,
  resampleContour,
  rotateXAxisSliced,
  rotateZAxisSliced,
  assignNormals,
  sectorSize,
  Segment,
} from "./fouraxis-util.js";
import {
  assignMDRs,
  assignMDSLabels,
  selectPaths,
  assignPaths,
  findConnectingPath,
} from "./fouraxis-core.js";

// root function that performs the four-axis toolpath generation
export async function generateFourAxis(params) {
  const { sliced, onupdate, toolObj, lineColor } = params;

  console.log(`Four axis slicing: ${sliced.length} slices`);

  // When computing machinability, check angles this far apart.
  // TODO - make this a parameter
  const angleStep = 5;

  // When subsampling segments, make the new segments at most this long
  // TODO - make this a parameter
  const resampleSegmentLength = 0.5;

  // track the index of the current slice, for use in progress bar and debugging
  let slice_index = 0;

  // track the total number of rotations made by the part, to prevent useless
  // 360-degree rotations between slices
  let totalRotations = 0;

  // iterate over every slice returned by the slicer
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
    //
    // This also adds a _fouraxis data object to each point.
    contours = contours.map((poly) => rotateXAxisSliced(poly.clone(true)));
    contours.forEach((poly) => poly.setCounterClockwise());

    // next, resample all of the contours into small segments.
    let resampledContours = contours
      .map((poly) => {
        let p = poly.clone(true, [], ["_fouraxis"]);
        p.points = resampleContour(p.points, resampleSegmentLength);
        return p;
      })
      .filter((poly) => poly.points.length > 2);

    // Compute and add in the normal vectors for each contour
    resampledContours.forEach((poly) => {
      poly.points = assignNormals(poly.points);
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

    // Set machinability for each point
    resampledContours.forEach((poly) =>
      assignMDRs(poly, grid, angleStep, toolObj)
    );

    // convert from points along the each contour into segments
    let resampledSegments = resampledContours.map((poly) =>
      poly.points.map(
        (p, i, pts) => new Segment(p, pts[(i + 1 + pts.length) % pts.length])
      )
    );

    // assign point segment candidate labels
    resampledSegments.forEach((seg) => assignMDSLabels(seg));

    // from the candidate labels, select a single path for each segment
    resampledSegments.forEach((seg) => selectPaths(seg));

    // group points into paths by segment label
    resampledSegments = resampledSegments.map((seg) => assignPaths(seg));

    // get all of the paths in the current slice and compute an ordering for
    // them
    let allPaths = resampledSegments
      .map((seg, segi) =>
        Object.entries(seg.paths).map(([pathidx, path]) => {
          return {
            name: `c${segi}-p${pathidx}`,
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
        segment: path.start,
      },
      {
        name: `${path.name}-end`,
        path: path,
        start: false,
        segment: path.end,
      },
    ]);
    const toolpathNodes = toolpathPairs.flat();
    toolpathNodes.forEach((n) => toolpathGraph.addNode(n.name, n));
    toolpathGraph.addNode("RETRACT", {
      name: "retract",
      path: null,
      start: null,
      segment: null,
    });

    toolpathNodes.forEach((node) => {
      toolpathGraph.addEdge(node.name, "RETRACT", 1e5); // from path to retract
      toolpathGraph.addEdge("RETRACT", node.name, 1e5); // from retract to path
    });

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
          const pathLength = n1.path.points
            .map((pt, i, pts) => {
              if (i == 0) {
                return 0;
              }
              return base.util.dist2D(pt, pts[i - 1]);
            })
            .reduce((a, b) => a + b, 0);

          if (n1.start) {
            toolpathGraph.addEdge(
              n1.name,
              n2.name,
              0,
              { path: pathBetween },
              pathLength
            );
            toolpathGraph.addEdge(
              n2.name,
              n1.name,
              0,
              {
                path: pathBetweenRev,
              },
              pathLength
            );
          } else {
            toolpathGraph.addEdge(
              n1.name,
              n2.name,
              0,
              {
                path: pathBetweenRev,
              },
              pathLength
            );
            toolpathGraph.addEdge(
              n2.name,
              n1.name,
              0,
              { path: pathBetween },
              pathLength
            );
          }
        } else {
          // check if there's a safe machinable path between n1 and n2
          const forwardPath = findConnectingPath(
            n1,
            n2,
            grid,
            angleStep,
            toolObj
          );
          if (forwardPath && forwardPath.length > 0) {
            const forwardPathLength = forwardPath
              .map((pt, i, pts) => {
                if (i === 0) {
                  return 0;
                }
                return base.util.dist2D(pt, pts[i - 1]);
              })
              .reduce((a, b) => a + b, 0);
            toolpathGraph.addEdge(
              n1.name,
              n2.name,
              forwardPathLength,
              {
                path: { points: forwardPath },
              },
              forwardPathLength
            );
          }

          const reversePath = findConnectingPath(
            n2,
            n1,
            grid,
            angleStep,
            toolObj
          );
          if (reversePath && reversePath.length > 0) {
            const reversePathLength = reversePath
              .map((pt, i, pts) => {
                if (i === 0) {
                  return 0;
                }
                return base.util.dist2D(pt, pts[i - 1]);
              })
              .reduce((a, b) => a + b, 0);
            toolpathGraph.addEdge(
              n2.name,
              n1.name,
              reversePathLength,
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
    // the middle of each segment's MDS, then do laplacian smoothing until the
    // total angle variance converges within 1 degree.
    toolpath.forEach((seg) => {
      if (!seg) {
        // this is a retract
        return;
      }
      const mds = seg.chosenMDS;
      seg.chosenAngle = (mds.start + sectorSize(mds.start, mds.stop) / 2) % 360;
    });

    const angleInMDS = (mds, a) => {
      if (mds.start < mds.stop) {
        return mds.start <= a && a <= mds.stop;
      } else {
        return mds.start <= a || a <= mds.stop;
      }
    };

    // compute whether each segment should use a clockwise or
    // counterclockwise rotation of the A axis. this is needed for interpolation
    toolpath.forEach((seg, i, tp) => {
      if (!seg || i == 0) {
        return;
      }
      let prevS = tp[i - 1];
      if (!prevS) {
        return;
      }
      let p = seg.start;

      let thisA = seg.chosenAngle;
      let prevA = prevS.chosenAngle;

      const ccw_arc_len = sectorSize(prevA, thisA);
      const is_ccw_short = ccw_arc_len <= 180;
      const bisector = (prevA + ccw_arc_len / 2 + 360) % 360;

      // Check if the midpoint of the direct CCW arc is valid for both points
      const ccw_path_is_valid =
        angleInMDS(seg.chosenMDS, bisector) &&
        angleInMDS(prevS.chosenMDS, bisector);

      if (ccw_path_is_valid) {
        // The direct CCW path is clear, so choose the shorter of the two rotational paths.
        p._fouraxis.ccw = is_ccw_short;
      } else {
        // The direct CCW path is blocked, so we MUST go the other way (CW).
        p._fouraxis.ccw = false;
      }
    });

    toolpath.forEach((seg, i, tp) => {
      if (!seg) {
        return;
      }
      if (!seg.start._fouraxis) seg.start._fouraxis = {};
      if (!seg.end._fouraxis) seg.end._fouraxis = {};

      if (i === 0) {
        seg.start._fouraxis.totalRotations = totalRotations;
        seg.end._fouraxis.totalRotations = totalRotations;
        return;
      }
      let prevSeg = tp[i - 1];
      if (!prevSeg) {
        seg.start._fouraxis.totalRotations = totalRotations;
        seg.end._fouraxis.totalRotations = totalRotations;
        return;
      }

      let thisA = seg.chosenAngle;
      let prevA = prevSeg.chosenAngle;
      // ccw is calculated in a previous loop and stored on the start point
      let use_ccw = seg.start._fouraxis.ccw;

      if (prevA > thisA && use_ccw) {
        totalRotations++;
      } else if (thisA > prevA && !use_ccw) {
        totalRotations--;
      }
      seg.start._fouraxis.totalRotations = totalRotations;
      seg.start._fouraxis.chosenAngle = seg.chosenAngle;
      seg.end._fouraxis.totalRotations = totalRotations;
      seg.end._fouraxis.chosenAngle = seg.chosenAngle;
    });

    const finalToolpathPoints = [];
    toolpath.forEach((seg) => {
      if (seg) {
        finalToolpathPoints.push(seg.start);
        finalToolpathPoints.push(seg.end);
      } else {
        finalToolpathPoints.push(null);
      }
    });

    let finalToolpath = finalToolpathPoints.map((p) => {
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
  }
  return sliced;
}
