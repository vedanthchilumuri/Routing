# Graph Routing Engine: Dijkstra vs. A*

A C++ shortest-path engine that implements Dijkstra's algorithm and A* search on a weighted graph and compares how many nodes each one explores.

## Features

- Weighted graph stored as an adjacency list
- Dijkstra's algorithm with a min-heap priority queue
- A* search with a Euclidean distance heuristic
- Path reconstruction
- Side-by-side comparison of path length and nodes expanded

## How it works

Both algorithms repeatedly pull the most promising node from a priority queue, mark it visited, and relax its outgoing edges. They differ in what "most promising" means.

**Dijkstra** orders nodes by the distance traveled from the start. It expands outward in all directions, like a ripple, until it reaches the goal. It always finds the shortest path but explores many nodes that lead away from the target.

**A\*** orders nodes by `g + h`, where `g` is the distance traveled so far and `h` is a heuristic estimate of the remaining distance. Here `h` is the straight-line (Euclidean) distance to the goal. Nodes that head toward the goal get lower scores, so the search stays focused. Because the heuristic never overestimates the true remaining distance, A* still returns an optimal path.

The graph is an adjacency list: for each node, a list of `(neighbor, weight)` pairs. Nodes also store `(x, y)` coordinates for the heuristic. Duplicate queue entries are skipped with a visited array instead of using decrease-key.

## Build and run

```bash
g++ -std=c++17 -O2 graph_routing_dijkstra_astar.cpp -o routing
./routing
```

## Benchmark

The demo builds a 25x25 grid with unit edge weights, starts at the center cell `(12, 12)`, and routes to `(12, 20)`.

| Algorithm | Path length | Nodes expanded |
|-----------|-------------|----------------|
| Dijkstra | 8 | 130 |
| A* | 8 | 9 |

Both find the same optimal distance, and A* expands about 93% fewer nodes in this setup. The size of the gain depends on the graph and the goal's position. If the start is a grid corner and the goal is the opposite corner, for example, both explore every node. The start in the center of the grid is what lets Dijkstra waste effort in directions away from the goal.

## Complexity

- Time: O((V + E) log V) for both algorithms
- Space: O(V + E) for the graph, O(V) for the search state

A* has the same worst-case bound as Dijkstra. Its practical advantage comes from the heuristic reducing how many nodes are expanded.

## Limitations

- The graph is generated in code. There is no file input for real maps yet.
- The only heuristic is Euclidean distance, which is admissible on a grid with unit weights. A graph with different edge weights would need a heuristic that stays admissible.
- Only one benchmark scenario is included.

## Future work

- Load graphs and grids from a file
- Add obstacles and a Manhattan-distance heuristic
- Report execution time as well as nodes expanded
- Run many random start/goal pairs and average the results
