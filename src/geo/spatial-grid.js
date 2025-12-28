
/**
 * A purely 2D spatial grid for fast line segment lookups.
 * Expects all inputs (bounds, segments, points) to have {x, y} properties.
 */
export class SpatialGrid {
  constructor(bounds, cellSize, padding = 0) {
    this.bounds = {
        min: { x: bounds.min.x - padding, y: bounds.min.y - padding },
        max: { x: bounds.max.x + padding, y: bounds.max.y + padding }
    };
    this.cellSize = cellSize > 0 ? cellSize : 1.0;
    this.padding = padding;
    this.grid = [];
    this.cols = Math.ceil((this.bounds.max.x - this.bounds.min.x) / this.cellSize) || 1;
    this.rows = Math.ceil((this.bounds.max.y - this.bounds.min.y) / this.cellSize) || 1;
    for (let i = 0; i < this.cols * this.rows; i++) {
      this.grid.push([]);
    }
  }

  _getCells(segment) {
    const [p1, p2] = segment;
    const b = {
      min: { x: Math.min(p1.x, p2.x), y: Math.min(p1.y, p2.y) },
      max: { x: Math.max(p1.x, p2.x), y: Math.max(p1.y, p2.y) }
    };

    const minX = Math.floor((b.min.x - this.bounds.min.x) / this.cellSize);
    const maxX = Math.floor((b.max.x - this.bounds.min.x) / this.cellSize);
    const minY = Math.floor((b.min.y - this.bounds.min.y) / this.cellSize);
    const maxY = Math.floor((b.max.y - this.bounds.min.y) / this.cellSize);

    const cells = [];
    for (let y = Math.max(0, minY); y <= Math.min(this.rows - 1, maxY); y++) {
      for (let x = Math.max(0, minX); x <= Math.min(this.cols - 1, maxX); x++) {
        cells.push(y * this.cols + x);
      }
    }
    return cells;
  }

  insert(segment) {
    this._getCells(segment).forEach(idx => {
      this.grid[idx].push(segment);
    });
  }

  queryRay(ray_origin) {
    // Assumes a horizontal ray in the +x direction, as is our use case.
    const candidates = new Set();
    const startX = Math.floor((ray_origin.x - this.bounds.min.x) / this.cellSize);
    const startY = Math.floor((ray_origin.y - this.bounds.min.y) / this.cellSize);

    if (startX >= this.cols || startY < 0 || startY >= this.rows) {
      return [];
    }

    // Traverse grid cells horizontally along the ray path
    for (let x = Math.max(0, startX); x < this.cols; x++) {
      const y = startY;
      const idx = y * this.cols + x;
      if (this.grid[idx]) {
        this.grid[idx].forEach(seg => candidates.add(seg));
      }
    }
    return [...candidates];
  }

  rotate(angle_deg) {
    const angle_rad = angle_deg * Math.PI / 180;
    const cos = Math.cos(angle_rad);
    const sin = Math.sin(angle_rad);

    const rotatePoint = (p) => ({
      x: p.x * cos - p.y * sin,
      y: p.x * sin + p.y * cos
    });

    const newBounds = {
      min: { x: Infinity, y: Infinity },
      max: { x: -Infinity, y: -Infinity }
    };

    const segments = this.grid.flat();
    const rotatedSegments = [];

    for (const seg of segments) {
      const p1_rot = rotatePoint(seg[0]);
      const p2_rot = rotatePoint(seg[1]);
      rotatedSegments.push([p1_rot, p2_rot]);

      newBounds.min.x = Math.min(newBounds.min.x, p1_rot.x, p2_rot.x);
      newBounds.min.y = Math.min(newBounds.min.y, p1_rot.y, p2_rot.y);
      newBounds.max.x = Math.max(newBounds.max.x, p1_rot.x, p2_rot.x);
      newBounds.max.y = Math.max(newBounds.max.y, p1_rot.y, p2_rot.y);
    }

    const newGrid = new SpatialGrid(newBounds, this.cellSize, this.padding);
    for (const seg of rotatedSegments) {
      newGrid.insert(seg);
    }

    return newGrid;
  }
}

export function createFromSegments(segments, cellSize, padding = 0) {
  const bounds = {
    min: { x: Infinity, y: Infinity },
    max: { x: -Infinity, y: -Infinity }
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
    grid.insert(seg);
  }

  return grid;
}
