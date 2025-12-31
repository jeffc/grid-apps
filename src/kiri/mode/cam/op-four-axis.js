/** Copyright Stewart Allen <sa@grid.space> -- All Rights Reserved */

import { CamOp } from "./op.js";
import { generate as topo4_generate } from "./topo4.js";
import { newPoint } from "../../../geo/point.js";

class OpFourAxis extends CamOp {
  constructor(state, op) {
    super(state, op);
  }

  async slice(progress) {
    let { op, state } = this;
    let { addSlices, color } = state;
    this.topo = await topo4_generate(
      {
        op,
        state,
        onupdate: (pct, msg) => {
          progress(pct, msg);
        },
        ondone: (slices) => {
          this.slices = slices;
          addSlices(slices, false);
        },
      },
      true
    );
  }

  prepare(ops, progress) {
    let { slices, state, topo } = this;
    let { camOut, newLayer, setContouring, setNextIsMove, zSafe } = ops;
    let { gcodeResetA } = state.settings.device;

    // start top center, X = 0, Y = 0 closest to 4th axis chuck
    camOut(newPoint(0, 0, zSafe).setA(0), 0);
    setContouring(false); // contouring logic interferes with 4-axis moves
    setNextIsMove();

    for (let slice of slices) {
      // ignore debug slices
      if (!slice.camLines) {
        continue;
      }

      // get last point in widget coordinates
      let lastPoint = ops.getLastPoint();

      for (let path of slice.camLines) {
        if (path.points.length === 0) {
          continue;
        }

        let startPoint = path.points[0];

        // if moving between paths, perform a safe retract then rotate then travel
        if (lastPoint) {
          console.log(
            `Retract move from ${[lastPoint.y, lastPoint.z, lastPoint.a]} to ${[startPoint.y, startPoint.z, lastPoint.a]}`
          );
          // 1. retract to safe Z at last point's XY
          camOut(lastPoint.clone().setZ(zSafe), 0);
          // 2. at retracted XY, rotate to the next contour's start angle
          camOut(lastPoint.clone().setZ(zSafe).setA(startPoint.a), 0);
          // 3. at safe Z and new angle, travel to the start of the next contour
          camOut(startPoint.clone().setZ(zSafe), 0);
        }

        // emit the actual path
        // first point will be a plunge because of the state of printPoint
        setNextIsMove();
        for (let point of path.points) {
          camOut(point, 1);
        }
        lastPoint = path.points.peek();
      }

      newLayer();
    }

    // move to safe height and reset A axis
    newLayer();
    //ops.addGCode([
    //  `G0 Z${zSafe.round(2)}`,
    //  gcodeResetA.join("\n") ?? "G92.4 A0 R0",
    //]);
  }
}

export { OpFourAxis };
