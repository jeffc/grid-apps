// implements simultaneous four-axis machining based on
// https://haisenzhao.github.io/FourAxis/files/four-axis.pdf

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
    grid.rayCast(toolEdge1, upAxis) === null &&
    grid.rayCast(toolEdge2, upAxis) === null
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

// perform an exhaustive back-and-forth analysis to assign possible segment
// labels to each point (one label per MDS)
function assignMDSLabels(poly) {
  // compute if two sectors overlap by rotating both so that sector1 starts at
  // zero
  const sectorsOverlap = (s1, s2) => {
    let [a, b] = [s1.start, s1.stop];
    let [c, d] = [s2.start, s2.stop];

    let bb = (b - a + 360) % 360;
    let cc = (c - a + 360) % 360;
    let dd = (d - a + 360) % 360;

    // return true if s2 starts before s1 ends, OR if s2 wraps past the zero
    // mark (where s1 starts)
    return cc <= bb || cc > dd;
  };

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

// Assign individual path labels to points in a polygon
function assignPaths(poly) {
  const pts = poly.points;
  pts.forEach((p) => {
    // TODO - properly implement graph-cut-based assignment

    // choose the path label with the largest MDS
    const sectorSize = (start, stop) => (stop - start + 360) % 360;
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

// root function that performs the four-axis toolpath generation
export async function generateFourAxis(params) {
  const { sliced, onupdate, toolObj, lineColor } = params;

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
    const toolpathNodes = allPaths
      .map((path) => [
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
      ])
      .flat();
    toolpathNodes.forEach((n) => toolpathGraph.addNode(n.name, n));
    toolpathGraph.addNode("RETRACT", {
      name: "retract",
      path: null,
      start: null,
      point: null,
    });

    for (let i = 0; i < toolpathNodes.length; i++) {
      const n1 = toolpathNodes[i];
      toolpathGraph.addEdge(n1.name, "RETRACT", 1e5); // make retraction expensive, but possible, from any node
      for (let j = 0; j < i; j++) {
        const n2 = toolpathNodes[j];

        if (n1.path.name == n2.path.name) {
          const pathBetween = structuredClone(n1.path.points);
          const pathBetweenRev = structuredClone(n1.path.points);
          if (n1.start) {
            toolpathGraph.addEdge(n1.name, n2.name, 0, { path: pathBetween });
            toolpathGraph.addEdge(n2.name, n1.name, 0, {
              path: pathBetweenRev,
            });
          } else {
            toolpathGraph.addEdge(n1.name, n2.name, 0, {
              path: pathBetweenRev,
            });
            toolpathGraph.addEdge(n2.name, n1.name, 0, { path: pathBetween });
          }
        } else {
          // todo - check if segment from n1 to n2 is clear (interpolate points
          // along straight line path from n1.point to n2.point, compute the MDR
          // at each interpolated point, and see if there exists a traverse MDS
          // along that whole segment.

          const dist = base.util.dist2D(n1.point, n2.point);
          toolpathGraph.addEdge(n1.name, n2.name, dist, {
            path: { points: [n1.point, n2.point] },
          });
          toolpathGraph.addEdge(n2.name, n1.name, dist, {
            path: { points: [n2.point, n1.point] },
          });
        }
      }
    }

    // now find a traversal order
    let tspPath = toolpathGraph.findPathTSP("RETRACT");
    let nodeTraversalOrder = tspPath.path;
    let edgeTraversalOrder = tspPath.edges;

    let finalToolpath = edgeTraversalOrder
      .map((e) => {
        // TODO handle retraction
        if (e.from == "RETRACT" || e.to == "RETRACT") return [newPoint(0, 100)];

        return e.edgeData.data.path || [];
      })
      .flat();

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

    if (slice_index % 10 == 0) {
      slice
        .output()
        .setLayer(`toolpath`, { line: 0xffff00 })
        .addPoly(
          newPolygon(
            finalToolpath.map((p) => newPoint(p.z, p.x, p.y))
          ).setOpen()
        );
    }
  }
  return sliced;
}
