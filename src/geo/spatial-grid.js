export class SpatialGrid {
  /**
   * @param {Object} bounds - { min: {x, y}, max: {x, y} }
   * @param {number} cellSize - Size of each grid cell
   * @param {number} padding - Extra empty space around the grid boundaries
   */
  constructor(bounds, cellSize, padding = 0) {
    if (
      bounds.min.x == Infinity ||
      bounds.min.y == Infinity ||
      bounds.max.x == Infinity ||
      bounds.max.y == Infinity
    ) {
      throw "Can't create unbounded spatial grid!";
    }

    this.cellSize = cellSize;

    // 1. Calculate the "World Origin" offset.
    // We shift the world so that (minX - padding) becomes 0 internally.
    this.offsetX = bounds.min.x - padding;
    this.offsetY = bounds.min.y - padding;

    // Calculate total dimensions including padding
    const totalWidth = bounds.max.x - bounds.min.x + padding * 2;
    const totalHeight = bounds.max.y - bounds.min.y + padding * 2;

    this.cols = Math.ceil(totalWidth / cellSize);
    this.rows = Math.ceil(totalHeight / cellSize);

    // Initialize grid
    this.cells = new Array(this.cols * this.rows).fill(null).map(() => []);
  }

  // Helper: Converts World Coordinate -> Grid Cell Coordinate (Integer)
  _toGridCoord(val, offset) {
    return Math.floor((val - offset) / this.cellSize);
  }

  addSegment(p1, p2) {
    // Convert both points to Grid Coordinates
    const c1 = this._toGridCoord(p1.x, this.offsetX);
    const r1 = this._toGridCoord(p1.y, this.offsetY);
    const c2 = this._toGridCoord(p2.x, this.offsetX);
    const r2 = this._toGridCoord(p2.y, this.offsetY);

    const minCol = Math.min(c1, c2);
    const maxCol = Math.max(c1, c2);
    const minRow = Math.min(r1, r2);
    const maxRow = Math.max(r1, r2);

    const segment = { p1, p2, id: Math.random() };

    // Loop strictly over the touched cells
    // Clamp to 0 and cols-1 to handle lines that might slightly exceed bounds
    for (
      let c = Math.max(0, minCol);
      c <= Math.min(this.cols - 1, maxCol);
      c++
    ) {
      for (
        let r = Math.max(0, minRow);
        r <= Math.min(this.rows - 1, maxRow);
        r++
      ) {
        this.cells[r * this.cols + c].push(segment);
      }
    }
  }

  rayCast(origin, direction, maxDistance = Infinity) {

    if (isNaN(origin.x) || isNaN(origin.y) || isNaN(direction.x) || isNaN(direction.y)) {
      debugger;
      return null;
    }

    // 1. Convert Ray Origin to "Grid Space" (0-based)
    // The DDA algorithm now runs entirely in this shifted positive space
    const gridOriginX = origin.x - this.offsetX;
    const gridOriginY = origin.y - this.offsetY;

    let x = Math.floor(gridOriginX / this.cellSize);
    let y = Math.floor(gridOriginY / this.cellSize);

    const stepX = direction.x > 0 ? 1 : -1;
    const stepY = direction.y > 0 ? 1 : -1;

    const tDeltaX = Math.abs(this.cellSize / direction.x);
    const tDeltaY = Math.abs(this.cellSize / direction.y);

    // Distance to next grid boundary (calculated in Grid Space)
    let tMaxX =
      direction.x > 0
        ? ((x + 1) * this.cellSize - gridOriginX) / direction.x
        : (gridOriginX - x * this.cellSize) / Math.abs(direction.x);

    let tMaxY =
      direction.y > 0
        ? ((y + 1) * this.cellSize - gridOriginY) / direction.y
        : (gridOriginY - y * this.cellSize) / Math.abs(direction.y);

    const checkedSegmentIds = new Set();
    let closestHit = null;

    while (true) {
      // Check boundaries
      if (x < 0 || x >= this.cols || y < 0 || y >= this.rows) break;

      const cellIndex = y * this.cols + x;
      const segments = this.cells[cellIndex];

      if (segments && segments.length > 0) {
        for (const seg of segments) {
          if (checkedSegmentIds.has(seg.id)) continue;
          checkedSegmentIds.add(seg.id);

          // Ray vs Segment math uses original World Space coordinates
          // so we pass the original 'origin' here, not 'gridOrigin'
          const hit = this._getRaySegmentIntersection(
            origin,
            direction,
            seg.p1,
            seg.p2
          );

          if (hit && hit.distance < maxDistance) {
            if (!closestHit || hit.distance < closestHit.distance) {
              closestHit = hit;
            }
          }
        }
      }

      // Optimization: If hit is within the current cell's "t-boundary"
      if (closestHit && closestHit.distance < Math.min(tMaxX, tMaxY)) {
        return closestHit;
      }

      if (tMaxX < tMaxY) {
        tMaxX += tDeltaX;
        x += stepX;
      } else {
        tMaxY += tDeltaY;
        y += stepY;
      }

      if (tMaxX > maxDistance && tMaxY > maxDistance) break;
    }

    return closestHit;
  }

  // (Math Helper remains exactly the same as previous version)
  _getRaySegmentIntersection(rayOrigin, rayDir, segA, segB) {
    const v1 = { x: rayOrigin.x - segA.x, y: rayOrigin.y - segA.y };
    const v2 = { x: segB.x - segA.x, y: segB.y - segA.y };
    const v3 = { x: -rayDir.x, y: -rayDir.y };

    const dot = v2.x * v3.y - v2.y * v3.x;
    if (Math.abs(dot) < 0.000001) return null;

    const t1 = (v2.x * v1.y - v2.y * v1.x) / dot;
    const t2 = (v1.x * v3.y - v1.y * v3.x) / dot;

    if (t1 >= 0 && t2 >= 0 && t2 <= 1) {
      return {
        x: rayOrigin.x + rayDir.x * t1,
        y: rayOrigin.y + rayDir.y * t1,
        distance: t1,
      };
    }
    return null;
  }
}

export function fromSegments(segments, cellSize, padding = 0) {
  const bounds = {
    min: { x: Infinity, y: Infinity },
    max: { x: -Infinity, y: -Infinity },
  };

  for (const seg of segments) {
    const [p1, p2] = seg;
    bounds.min.x = Math.min(bounds.min.x, p1.x, p2.x);
    bounds.min.y = Math.min(bounds.min.y, p1.y, p2.y);
    bounds.max.x = Math.max(bounds.max.x, p1.x, p2.x);
    bounds.max.y = Math.max(bounds.max.y, p1.y, p2.y);
  }

  const grid = new SpatialGrid(bounds, cellSize, padding);

  for (const seg of segments) {
    grid.addSegment(seg[0], seg[1]);
  }

  return grid;
}
