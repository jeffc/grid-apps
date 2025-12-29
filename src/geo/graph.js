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
     * Finds a shortest path between two nodes using Dijkstra's algorithm, constrained
     * to a specific set of allowed nodes.
     * @param {*} startNode 
     * @param {*} endNode 
     * @param {Set} allowedNodes - A set of nodes the path is allowed to traverse.
     * @returns {{path: Array<*>, edges: Array<*>, weight: number}|null}
     * @private
     */
    _findShortestPath(startNode, endNode, allowedNodes) {
        let distances = new Map();
        let prev = new Map();
        let pq = new Map(); // Using a Map as a min-priority queue

        for (const node of allowedNodes) {
            distances.set(node, Infinity);
            prev.set(node, null);
            pq.set(node, Infinity);
        }

        distances.set(startNode, 0);
        pq.set(startNode, 0);

        while (pq.size > 0) {
            // Get node with smallest distance from priority queue
            let closestNode = null;
            let minDistance = Infinity;
            for (const [node, dist] of pq.entries()) {
                if (dist < minDistance) {
                    minDistance = dist;
                    closestNode = node;
                }
            }
            
            if (closestNode === null) break;
            pq.delete(closestNode);

            if (closestNode === endNode) {
                // Reconstruct path
                let path = [];
                let edges = [];
                let current = endNode;
                while (current !== null) {
                    path.unshift(current);
                    let p = prev.get(current);
                    if (p) {
                        const edgeData = this.adj.get(p)?.get(current);
                        edges.unshift({ from: p, to: current, edgeData });
                    }
                    current = p;
                }
                return { path, edges, weight: distances.get(endNode) };
            }

            const neighbors = this.getNeighbors(closestNode);
            if (neighbors) {
                for (const [neighbor, edge] of neighbors.entries()) {
                    if (allowedNodes.has(neighbor)) {
                        let newDist = distances.get(closestNode) + edge.weight;
                        if (newDist < distances.get(neighbor)) {
                            distances.set(neighbor, newDist);
                            prev.set(neighbor, closestNode);
                            pq.set(neighbor, newDist);
                        }
                    }
                }
            }
        }

        return null; // No path found
    }

    /**
     * Finds a path that visits every node in the graph. It uses a nearest-neighbor
     * heuristic, but if it gets stuck, it finds the globally cheapest edge to an
     * unvisited node and uses Dijkstra's algorithm to pathfind to it, allowing
     * nodes to be re-visited to ensure a single continuous path.
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

        const visited = new Set();
        const path = [startNode];
        const edges = [];
        let totalWeight = 0;
        let currentNode = startNode;
        
        visited.add(startNode);

        while (visited.size < nodes.length) {
            // First, try a simple nearest-neighbor search from the current node
            const neighbors = this.getNeighbors(currentNode);
            let nearestNeighbor = null;
            let minWeight = Infinity;
            let connectingEdge = null;

            if (neighbors) {
                for (const [neighbor, edge] of neighbors.entries()) {
                    if (!visited.has(neighbor) && edge.weight < minWeight) {
                        minWeight = edge.weight;
                        nearestNeighbor = neighbor;
                        connectingEdge = edge;
                    }
                }
            }

            if (nearestNeighbor) {
                // Found a simple connection to an unvisited node
                totalWeight += minWeight;
                edges.push({ from: currentNode, to: nearestNeighbor, edgeData: connectingEdge });
                currentNode = nearestNeighbor;
                path.push(currentNode);
                visited.add(currentNode);
            } else {
                // Stuck. Find the globally cheapest edge from any visited node to any unvisited one.
                let bestV = null, bestU = null, minGlobalWeight = Infinity, bestEdge = null;

                for (const v of visited) {
                    const v_neighbors = this.getNeighbors(v);
                    if (v_neighbors) {
                        for (const [u, edge] of v_neighbors.entries()) {
                            if (!visited.has(u) && edge.weight < minGlobalWeight) {
                                minGlobalWeight = edge.weight;
                                bestV = v;
                                bestU = u;
                                bestEdge = edge;
                            }
                        }
                    }
                }

                if (bestV === null) {
                    // No path from visited to unvisited nodes, graph is disconnected
                    break; 
                }

                // Find a path from the current end of the path to the start of the best new edge
                const bridge = this._findShortestPath(currentNode, bestV, visited);
                if (bridge) {
                    // Append the bridging path (re-visiting nodes)
                    path.push(...bridge.path.slice(1));
                    edges.push(...bridge.edges);
                    totalWeight += bridge.weight;
                }

                // Append the new cheapest edge and node
                totalWeight += minGlobalWeight;
                edges.push({ from: bestV, to: bestU, edgeData: bestEdge });
                currentNode = bestU;
                path.push(currentNode);
                visited.add(currentNode);
            }
        }

        // Close the loop by finding the shortest path back to the start node
        const closing_path = this._findShortestPath(currentNode, startNode, new Set(nodes));
        if (closing_path) {
            totalWeight += closing_path.weight;
            edges.push(...closing_path.edges);
            path.push(...closing_path.path.slice(1));
        }

        return {
            path: path,
            edges: edges,
            weight: totalWeight,
            missing: nodes.filter(n => !visited.has(n))
        };
    }
}
