/**
 * @file
 *
 * Functions to convert geometry primitives to human-readable strings.
 */

/**
 * Converts a point object to a human-readable string.
 * @param {object} point - The point object with x, y, z properties.
 * @returns {string} A string representation of the point.
 */
export function printPoint(point) {
  if (!point) {
    return "null_point";
  }
  return `(${point.x !== undefined ? point.x.toFixed(3) : "?"}, ${point.y !== undefined ? point.y.toFixed(3) : "?"}, ${point.z !== undefined ? point.z.toFixed(3) : "?"})`;
}

/**
 * Converts a polygon object to a human-readable string.
 * Assumes the polygon has a `points` array, where each element is a point object.
 * @param {object} polygon - The polygon object.
 * @returns {string} A string representation of the polygon.
 */
export function printPolygon(polygon) {
  if (!polygon || !polygon.points || polygon.points.length === 0) {
    return "empty_polygon";
  }
  const pointStrings = polygon.points.map(printPoint);
  return `[${pointStrings.join(", ")}]`;
}
