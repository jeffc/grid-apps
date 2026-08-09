/** Copyright Stewart Allen <sa@grid.space> -- All Rights Reserved */

import { CamOp } from '../core/op.js';
import { OpArea } from './op-area.js';
import { newPolygon } from '../../../../geo/polygon.js';
import { polygons as POLY } from '../../../../geo/polygons.js';

class OpAdaptive extends CamOp {
    constructor(state, op) {
        super(state, op);
    }

    async slice(progress) {
        let { op, state } = this;
        let { shadow, stock, tool, widget } = state;

        let shadowBase = shadow.base;

        if (op.down <= 0) {
            throw `invalid step down "${op.down}"`;
        }

        if (op.all) {
            shadowBase = [ newPolygon().centerRectangle(stock.center, stock.x, stock.y) ];
        }

        let expand_dist = op.wide ? (tool.fluteDiameter() * 2) : (op.all ? (tool.fluteDiameter() / 2 - 0.001) : (tool.fluteDiameter() * 1.5));
        let areas = POLY.flatten(POLY.expand(shadowBase, expand_dist));

        if (op.insideOnly) {
            areas = POLY.trimTo(areas, shadow.base);
        }

        let adaptiveConfig = {
            rename: op.rename ?? "adaptive",
            spindle: op.spindle,
            direction: op.direction,
            tool: op.tool,
            rate: op.rate,
            plunge: op.plunge,
            mode: 'adaptive',
            over: op.step,
            tea: op.tea,
            down: op.down,
            entry_helix_angle: op.entry_helix_angle,
            expand: 0,
            smooth: 0,
            outline: true,
            omitthru: op.omitthru,
            leave_xy: op.leave,
            leave_z: op.leavez,
            ov_botz: op.ov_botz,
            ov_topz: op.ov_topz,
            rotated: true,
            areas: { [widget.id]: areas.map(p => p.toArray()) },
            surfaces: {}
        };

        this.op_adaptive = new OpArea(state, adaptiveConfig);
        return this.op_adaptive.slice(progress);
    }

    prepare(ops, progress) {
        return this.op_adaptive.prepare(ops, progress);
    }
}

export { OpAdaptive };
