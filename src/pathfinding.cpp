#include "pathfinding.hpp"

#include <algorithm>
#include <functional>
#include <queue>


typedef pair<double, int> QItem;
typedef priority_queue<QItem, vector<QItem>, greater<QItem>> MinQueue;

static vector<NodeId> rebuild(const vector<NodeId>& parent, NodeId goal) {
    vector<NodeId> path;
    for (NodeId v = goal; v != -1; v = parent[v]) path.push_back(v);
    reverse(path.begin(), path.end());
    return path;
}

// BFS ignores road lengths while searching, so add them up afterwards.
static double path_cost(const Graph& g, const vector<NodeId>& path) {
    double total = 0.0;
    for (size_t i = 0; i + 1 < path.size(); i++) {
        double best = INF;  // if two roads join the same places, the cheaper one
        for (const Edge& e : g.neighbors(path[i]))  // "for each edge e in the list"
            if (e.to == path[i + 1]) best = min(best, e.weight);
        total += best;
    }
    return total;
}

PathResult bfs(const Graph& g, NodeId start, NodeId goal) {
    PathResult r;
    vector<NodeId> parent(g.size(), -1);  // parent[v] = place we came from to reach v
    vector<bool> seen(g.size(), false);
    queue<NodeId> q;                      // first in, first out
    q.push(start);
    seen[start] = true;

    while (!q.empty()) {
        NodeId u = q.front(); q.pop();
        r.expanded++;
        if (u == goal) {
            r.found = true;
            break;
        }
        for (const Edge& e : g.neighbors(u)) {
            if (seen[e.to]) continue;
            seen[e.to] = true;  // mark when PUSHED, so a place is queued only once
            parent[e.to] = u;
            q.push(e.to);
        }
    }
    if (r.found) {
        r.path = rebuild(parent, goal);
        r.cost = path_cost(g, r.path);
    }
    return r;
}
PathResult dijkstra(const Graph& g, NodeId start, NodeId goal) {
    PathResult r;
    // 1. dist[v] = best distance so far (all INF), a parent trail (all -1), a MinQueue
    // 2. dist[start] = 0, push (0, start)
    // 3. while the queue is not empty:
    //      read the smallest entry (its distance d and place u), THEN pop
    //      if d > dist[u] it is stale: skip
    //      count it as expanded; if u is the goal: found = true, stop
    //      for each road out of u:
    //          nd = d + road length
    //          if nd < dist[far end]: update dist, parent, push (nd, far end)
    // 4. if found: path from the parent trail, cost = dist[goal]
    // 5. return r
    vector<double> dist(g.size(), INF);
    vector<NodeId> parent(g.size(), -1);
    MinQueue pq;
    dist[start]=0.0;
    pq.push(QItem(0.0,start));
    while(!pq.empty())
    {
        double d=pq.top().first;
        NodeId u=pq.top().second;
        pq.pop();
        if (d > dist[u]) continue;
        r.expanded++;
        if (u==goal)
        {
            r.found=true;
            break;
            }
        for(const Edge& e:g.neighbors(u))
        {
            double nd=d+e.weight;
            if (nd < dist[e.to])
            {
                dist[e.to]=nd;
                parent[e.to]=u;
                pq.push(QItem(nd, e.to));
                }
            }
        }
        if (r.found)
        {
            r.path=rebuild(parent,goal);
            r.cost=dist[goal];
            }
        return r;
}

PathResult astar(const Graph& g, NodeId start, NodeId goal) {
    PathResult r;
    // 1. gscore (all INF), parent (all -1), closed (all false), a MinQueue
    // 2. gscore[start] = 0, push (heuristic(start, goal), start)
    // 3. while the queue is not empty:
    //      take the place u from the front, pop
    //      if closed[u] skip; else closed[u] = true
    //      count it as expanded; if goal: found = true, stop
    //      for each road out of u:
    //          ng = gscore[u] + road length
    //          if ng < gscore[far end]: update, parent, push (ng + heuristic(far end, goal), far end)
    // 4. if found: path from the parent trail, cost = gscore[goal]
    // 5. return r
    vector<double> gscore(g.size(), INF);
    vector<NodeId> parent(g.size(), -1);
    vector<bool> closed(g.size(), false);
    MinQueue pq;
    gscore[start]=0.0;
    pq.push(QItem(g.heuristic(start,goal),start));
    while(!pq.empty())
    {
        NodeId u=pq.top().second;
        pq.pop();
        if (closed[u]) continue;
        closed[u]=true;
        r.expanded++;
        if (u==goal)
        {
            r.found=true;
            break;
            }
        for(const Edge& e:g.neighbors(u))
        {
            double ng=gscore[u]+e.weight;
            if (ng<gscore[e.to])
            {
                gscore[e.to]=ng;
                parent[e.to]=u;
                pq.push(QItem(ng+g.heuristic(e.to,goal), e.to));
            }
        }
        
        }
        if (r.found)
        {
            r.path=rebuild(parent,goal);
            r.cost=gscore[goal];
             }
        return r; 
}

DSU build_components(const Graph& g) {
    // 1. make a DSU with one item per place
    // 2. for every place u, for every road out of u: unite u with the far end
    // 3. return the DSU
    DSU dsu(g.size());
    for(NodeId u=0; u<g.size(); u++)
    {
        for(const Edge& e:g.neighbors(u)) dsu.unite(u, e.to);
    }
    return dsu;
}
