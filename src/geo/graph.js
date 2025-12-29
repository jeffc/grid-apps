/**
 * A directed, weighted graph data structure that can store arbitrary data on
 * both nodes and edges.
 */
export class Graph {
  /**
   * Initializes a new Graph.
   */
  constructor() {
    // Use a Map to store the adjacency list for graph structure.
    // The key is the node, and the value is another Map
    // where keys are neighbor nodes and values are the edge objects.
    this.adj = new Map();

    // Use a separate Map to store node-specific data.
    // The key is the node, and the value is an object holding the data.
    this.nodes = new Map();
  }

  /**
   * Adds a node to the graph. If the node already exists, it is not modified.
   * @param {*} node - The node to add. Can be any type that can be a Map key.
   * @param {Object} [data={}] - Optional arbitrary data to associate with the node.
   */
  addNode(node, data = {}) {
    if (!this.adj.has(node)) {
      this.adj.set(node, new Map());
      this.nodes.set(node, { data });
    }
  }

  /**
   * Adds a directed, weighted edge with optional data from a source to a destination node.
   * If the nodes do not exist, they will be added.
   * If an edge already exists, its weight and data will be updated.
   * @param {*} source - The source node of the edge.
   * @param {*} destination - The destination node of the edge.
   * @param {number} weight
   * @param {Object} [data={}] - Optional arbitrary data to associate with the edge.
   */
  addEdge(source, destination, weight, data = {}) {
    this.addNode(source);
    this.addNode(destination);

    const edge = { weight, data };
    this.adj.get(source).set(destination, edge);
  }

  /**
   * Retrieves the weight of the edge between a source and destination node.
   * @param {*} source
   * @param {*} destination
   * @returns {number|undefined} The weight of the edge, or undefined if no edge exists.
   */
  getWeight(source, destination) {
    const edge = this.adj.get(source)?.get(destination);
    return edge ? edge.weight : undefined;
  }

  /**
   * Retrieves the outgoing neighbors of a given node.
   * @param {*} node
   * @returns {Map<*, {weight: number, data: Object}>|undefined} A map of neighbor
   * nodes to their edge objects, or undefined if the node does not exist.
   */
  getNeighbors(node) {
    return this.adj.get(node);
  }

  /**
   * Returns all nodes in the graph.
   * @returns {IterableIterator<*>} An iterator for the nodes.
   */
  getNodes() {
    return this.adj.keys();
  }

  /**
   * Returns the total number of nodes in the graph.
   * @returns {number}
   */
  size() {
    return this.adj.size;
  }

  /**
   * Retrieves the data associated with a given node.
   * @param {*} node
   * @returns {Object|undefined} The data object, or undefined if the node does not exist.
   */
  getNodeData(node) {
    const nodeInfo = this.nodes.get(node);
    return nodeInfo ? nodeInfo.data : undefined;
  }

  /**
   * Sets or updates the data for a given node.
   * If the node does not exist, it will be added to the graph.
   * @param {*} node
   * @param {Object} data - The data to associate with the node.
   */
  setNodeData(node, data) {
    if (!this.nodes.has(node)) {
      this.addNode(node, data);
    } else {
      this.nodes.get(node).data = data;
    }
  }

  /**
   * Retrieves the data associated with a given edge.
   * @param {*} source
   * @param {*} destination
   * @returns {Object|undefined} The data object, or undefined if the edge does not exist.
   */
  getEdgeData(source, destination) {
    const edge = this.adj.get(source)?.get(destination);
    return edge ? edge.data : undefined;
  }

  /**
   * Sets or updates the data for a given edge.
   * If the edge does not exist, this method does nothing.
   * @param {*} source
   * @param {*} destination
   * @param {Object} data - The data to associate with the edge.
   */
  setEdgeData(source, destination, data) {
    const edge = this.adj.get(source)?.get(destination);
    if (edge) {
      edge.data = data;
    }
  }

  /**
   * Finds a path that visits every node in the graph using a nearest-neighbor
   * heuristic, which is a fast approximation for the Traveling Salesman Problem (TSP).
   *
   * @param {*} [startNode] - The node to start the path from. If not provided,
   *   it will start with an arbitrary node.
   * @returns {{path: Array<*>, edges: Array<{from: *, to: *, edgeData: {weight: number, data: Object}}>, weight: number, missing: Array<*>}|null} An object
   *   containing the generated path, the sequence of edges visited, its total weight,
   *   and any nodes that were unreachable (in case of a disconnected graph).
   *   Returns null if the graph is empty.
   */
  findPathTSP(startNode) {
    const nodes = Array.from(this.getNodes());
    if (nodes.length === 0) {
      return null;
    }

    if (!startNode || !this.adj.has(startNode)) {
      startNode = nodes[0];
    }

    const unvisited = new Set(nodes);
    const path = [];
    const edges = []; // New array to store visited edges
    let totalWeight = 0;
    let currentNode = startNode;

    while (unvisited.size > 0) {
      path.push(currentNode);
      unvisited.delete(currentNode);

      if (unvisited.size === 0) {
        break;
      }

      const neighbors = this.getNeighbors(currentNode);
      let nearestNeighbor = null;
      let minWeight = Infinity;
      let connectingEdge = null; // To store the edge object

      if (neighbors) {
        for (const [neighbor, edge] of neighbors.entries()) {
          if (unvisited.has(neighbor) && edge.weight < minWeight) {
            minWeight = edge.weight;
            nearestNeighbor = neighbor;
            connectingEdge = edge;
          }
        }
      }

      if (nearestNeighbor) {
        totalWeight += minWeight;
        edges.push({
          from: currentNode,
          to: nearestNeighbor,
          edgeData: connectingEdge,
        }); // Store the edge
        currentNode = nearestNeighbor;
      } else if (unvisited.size > 0) {
        // Handle disconnected graph: jump to the next available unvisited node.
        // This is no longer a single path, but a series of paths.
        // The connection between them has an effective "infinite" weight.
        // No edge is added here as it's a "jump" not a graph edge traversal.
        currentNode = unvisited.values().next().value;
      }
    }

    // Close the loop by returning to the start node to complete the tour
    const lastNodeInPath = path[path.length - 1];
    const closingEdgeObj = this.adj.get(lastNodeInPath)?.get(startNode);
    if (closingEdgeObj !== undefined) {
      totalWeight += closingEdgeObj.weight;
      edges.push({
        from: lastNodeInPath,
        to: startNode,
        edgeData: closingEdgeObj,
      }); // Store closing edge
      path.push(startNode);
    }

    return {
      path: path,
      edges: edges, // Include edges in the return object
      weight: totalWeight,
      missing: [...unvisited], // will be empty if all nodes were visited
    };
  }
}
