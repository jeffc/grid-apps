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
    setContouring(true);
    setNextIsMove();

    for (let slice of slices) {
      // ignore debug slices
      if (!slice.camLines) {
        continue;
      }

      for (let path of slice.camLines) {
        let lastAngle = 0;
        let lastPoint = newPoint(0, 0, zSafe).setA(0);
        for (let point of path.points) {
          if (point !== null) {
            lastAngle = point.a;
            lastPoint = point;
            camOut(point, 1);
          } else {
            camOut(
              newPoint(lastPoint.x, lastpoint.y, zSafe).setA(lastAngle),
              0
            );
            //setNextIsMove();
          }
        }
      }

      newLayer();
    }

    // move to safe height and reset A axis
    newLayer();
    ops.addGCode([
      `G0 Z${zSafe.round(2)}`,
      gcodeResetA.join("\n") ?? "G92.4 A0 R0",
    ]);
  }
}

export { OpFourAxis };
