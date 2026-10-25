#pragma once
#include <vector>

#include "dsu.hpp"
#include "graph.hpp"

struct PathResult {
    bool found = false;
    double cost = 0.0;       // total road length along the path
    vector<NodeId> path;     // start ... goal, empty if not found
    size_t expanded = 0;     // how many places the search looked at: the fair
                             // way to compare algorithms, independent of CPU speed
};

// Fewest roads (hops), ignores road lengths. O(V + E).
PathResult bfs(const Graph& g, NodeId start, NodeId goal);

// Shortest total distance. Road lengths must not be negative.
// O((V + E) log V).
PathResult dijkstra(const Graph& g, NodeId start, NodeId goal);

// Dijkstra that is steered towards the goal by a guess (heuristic).
// Same worst case, but usually looks at far fewer places on a map.
PathResult astar(const Graph& g, NodeId start, NodeId goal);

// Builds a Union-Find over the roads (direction ignored). If two places are
// NOT connected here, no route can exist. With one-way roads, "connected"
// does not promise a route, so the real search still decides.
DSU build_components(const Graph& g);
