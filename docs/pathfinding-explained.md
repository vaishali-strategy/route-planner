# include/pathfinding.hpp Explained: BFS, Dijkstra, A\* and Where Their Complexity Comes From

This document assumes you have never met these algorithms. Read Part 1 first (the vocabulary), then the header, then each algorithm.

---

# Part 1. Vocabulary you need

- **Graph.** Places (called **nodes** or **vertices**) joined by roads (called **edges**). In this project a place is a `NodeId` (a number 0, 1, 2, ...) and each road has a length (its **weight**).
- **V** is the number of places. **E** is the number of directed edges stored. A two-way road is stored as two edges, so it counts twice.
- **Neighbour.** A place you can reach by one road from where you stand. `g.neighbors(u)` gives the list of roads leaving `u`.
- **Path.** A list of places from the start to the goal, each joined to the next by a road.
- **Hops** = how many roads are on the path. **Cost** = the sum of the road lengths. A path with the fewest hops is not always the shortest in distance.
- **Big-O.** A way to say how the work grows as the input grows. O(V + E) means "roughly proportional to the number of places plus the number of roads". It ignores constant factors, because we care about the shape of the growth, not the exact seconds.

## The one idea behind all three algorithms

You stand at the start. You keep a **to-do list** of places you have discovered but not yet visited. Repeatedly:

1. Take one place off the to-do list.
2. Look at every road leaving it. Any place you reach, add to the to-do list (if it is worth it).
3. Stop when the place you took off is the goal, or when the list is empty (no route).

**The only difference between BFS, Dijkstra and A\* is which place they take off the list next.** That one choice changes what each algorithm guarantees and how fast it is.

| Algorithm | Takes next... | The to-do list is a... | Guarantee |
|---|---|---|---|
| BFS | the place discovered **earliest** | queue (first in, first out) | fewest hops |
| Dijkstra | the place **closest to the start** so far | min-heap (priority queue) | shortest distance |
| A\* | the place with the smallest (distance so far + guess of distance left) | min-heap | shortest distance, usually looking at fewer places |

---

# Part 2. The header, line by line

`pathfinding.hpp` only **declares** things. It says what exists and what each function promises. The bodies are in `src/pathfinding.cpp`.

## The top

```cpp
#pragma once
#include <vector>

#include "dsu.hpp"
#include "graph.hpp"
```

- `#pragma once`: paste this file only once, even if several files include it.
- `<vector>`: this header uses `vector`.
- `"dsu.hpp"`: `build_components` returns a `DSU`, so the compiler must know what a `DSU` is.
- `"graph.hpp"`: every function takes a `Graph`, and uses `NodeId`. Because `graph.hpp` already has `using namespace std;`, the names `vector` and `size_t` work here without `std::`.

Quotes `"..."` are for your own files. Angle brackets `<...>` are for library files.

## `PathResult`: what a search hands back

```cpp
struct PathResult {
    bool found = false;
    double cost = 0.0;
    vector<NodeId> path;
    size_t expanded = 0;
};
```

All four searches return the same kind of answer, so they can be compared fairly.

- `found`: did we reach the goal? Starts as `false`; the search flips it to `true` on success.
- `cost`: total road length of the path.
- `path`: the places in order, start to goal. Empty if there is no route.
- `expanded`: how many places the search took off its to-do list and processed. This is the **fair measure of work**. Seconds depend on your computer; the count of places looked at does not. The benchmark compares Dijkstra and A\* using this number.
- `size_t` is an unsigned whole number type used for sizes and counts.
- The `= false`, `= 0.0`, `= 0` are default values: a freshly made `PathResult r;` starts as "not found, cost 0, empty path, nothing expanded".

A `struct` is a class whose members are public by default. It is the usual choice for a plain bundle of data.

## The four function declarations

```cpp
PathResult bfs(const Graph& g, NodeId start, NodeId goal);
PathResult dijkstra(const Graph& g, NodeId start, NodeId goal);
PathResult astar(const Graph& g, NodeId start, NodeId goal);
DSU build_components(const Graph& g);
```

- `const Graph& g`: the graph is passed **by reference** (no copy of the whole map) and as **const** (the search promises not to change it).
- `NodeId start, NodeId goal`: the two places, as numbers.
- The return type comes first. `build_components` returns a `DSU` by value.

---

# Part 3. Breadth-First Search (BFS)

## The idea

Explore in **rings** around the start. First everything one road away, then everything two roads away, then three, and so on. Like a ripple spreading on water.

```
Ring 0:  A
Ring 1:  B, C         (one road from A)
Ring 2:  D            (two roads from A)
```

Because BFS finishes every ring before starting the next, the **first time it reaches the goal, it got there using the fewest roads possible**. It does not look at road lengths at all while searching.

## Why a queue gives rings

A **queue** is first-in, first-out, like a line at a shop. Places discovered earlier are served earlier. Everything in ring 1 is discovered before anything in ring 2, so ring 1 is served first.

## The code

```cpp
PathResult bfs(const Graph& g, NodeId start, NodeId goal) {
    PathResult r;
    vector<NodeId> parent(g.size(), -1);
    vector<bool> seen(g.size(), false);
    queue<NodeId> q;
    q.push(start);
    seen[start] = true;

    while (!q.empty()) {
        NodeId u = q.front();
        q.pop();
        r.expanded++;
        if (u == goal) {
            r.found = true;
            break;
        }
        for (const Edge& e : g.neighbors(u)) {
            if (seen[e.to]) continue;
            seen[e.to] = true;
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
```

**The three arrays/containers:**

- `parent[v]`: which place we came from to reach `v`. All start at -1, meaning "nobody yet". This is how we remember the route.
- `seen[v]`: have we already discovered `v`? Without this, the search would walk in circles forever (A to B to A to B...) because roads are two-way.
- `q`: the queue of places waiting their turn.

**The loop, in words:**

1. Take the front place `u` off the queue. Count it as expanded.
2. If `u` is the goal, stop.
3. Otherwise look at each road leaving `u`. If the place at the other end (`e.to`) is new, mark it seen, remember that we got there from `u`, and put it at the back of the queue.

**Why mark `seen` when pushing, not when popping?** If you waited until popping, the same place could be pushed many times by different neighbours before its turn came. Marking at push time means each place enters the queue **at most once**.

**After the loop:** `rebuild` walks the `parent` links backwards from the goal to the start (the start has parent -1, which is where it stops), then reverses the list. `path_cost` adds up the road lengths afterwards, because BFS did not track them.

## Complexity of BFS: O(V + E), and why

Count how many times each line can run.

- **Each place enters the queue at most once** (because of `seen`), so it is popped at most once. The pop, the `expanded++` and the goal check therefore run at most **V** times in total.
- **Each place's road list is scanned only when that place is popped**, which happens at most once per place. Scanning all the lists together touches every stored edge exactly once, which is **E** road-checks in total.
- Each queue push/pop and each `seen` check costs O(1) (constant time).

Total work is about V (for the pops) + E (for the road checks) = **O(V + E)**.

Space: `parent`, `seen` and the queue each hold at most V items, so **O(V)**.

A common mistake is to say O(V × E) or O(V²). That would be right only if you re-scanned the whole graph for every place. We scan each road list once.

**What BFS gets wrong for a city.** It treats a 0.1 km road and a 50 km road as equal, one hop each. In the project's sample map BFS finds a 4-hop route costing 13.90, but Dijkstra finds a 5-hop route costing 13.80, which is shorter in distance.

---

# Part 4. Dijkstra's algorithm

## The idea

Roads now have different lengths, so "fewest hops" is no longer what we want. Dijkstra explores places in order of **how far they are from the start**, nearest first. Like a ripple, but one that spreads faster along short roads.

It keeps, for every place, the **best distance found so far** (`dist`). At first this is infinity for everyone except the start (0). Each time it processes a place, it checks whether going through that place gives a shorter way to a neighbour. That check is called **relaxing** the road.

```
if  dist[u] + length(u, v)  <  dist[v]   then  dist[v] = that smaller number
```

## Why "nearest first" is correct

Suppose `u` is the nearest unprocessed place, at distance 10. Could some other route reach `u` in less than 10? It would have to go through some other unprocessed place first, but all of those are at distance 10 or more, and adding a road length of 0 or more cannot make it smaller. So **10 is final**. This is the reason road lengths must not be negative: a negative road could make a farther place suddenly give a shorter way, and the argument collapses. That is why `Graph::add_edge` rejects negative lengths.

## Why a min-heap (priority queue)

We need "give me the place with the smallest distance" again and again. A **min-heap** stores items so that the smallest one is always on top. Looking at the smallest is O(1). Adding or removing an item is **O(log n)**, because the heap is a tree of height about log n and an item moves up or down at most that many levels.

By default C++'s `priority_queue` gives back the *largest* item. The line

```cpp
typedef priority_queue<QItem, vector<QItem>, greater<QItem>> MinQueue;
```

flips it with `greater`. `QItem` is `pair<double, int>`: (distance, place). Pairs compare by the first value, so the smallest distance comes out first.

## The code

```cpp
PathResult dijkstra(const Graph& g, NodeId start, NodeId goal) {
    PathResult r;
    vector<double> dist(g.size(), INF);
    vector<NodeId> parent(g.size(), -1);
    MinQueue pq;
    dist[start] = 0.0;
    pq.push(QItem(0.0, start));

    while (!pq.empty()) {
        double d = pq.top().first;
        NodeId u = pq.top().second;
        pq.pop();
        if (d > dist[u]) continue;

        r.expanded++;
        if (u == goal) {
            r.found = true;
            break;
        }
        for (const Edge& e : g.neighbors(u)) {
            double nd = d + e.weight;
            if (nd < dist[e.to]) {
                dist[e.to] = nd;
                parent[e.to] = u;
                pq.push(QItem(nd, e.to));
            }
        }
    }
    if (r.found) {
        r.path = rebuild(parent, goal);
        r.cost = dist[goal];
    }
    return r;
}
```

**Line by line:**

- `dist` starts at `INF` (a huge number, 1e18, meaning "not reached yet"). Only the start is 0.
- The queue starts with the entry (0, start).
- Each loop takes the smallest entry: its distance `d` and place `u`.
- **`if (d > dist[u]) continue;`** is the stale-entry check, explained next.
- If `u` is the goal, stop. Its distance is final by the argument above.
- For each road from `u`, compute `nd`, the distance to the neighbour via `u`. If it beats the best known, update `dist`, remember the parent, and push the improved entry.

## The stale entry (lazy deletion)

The C++ priority queue cannot change an item that is already inside it. So when we find a shorter way to a place that is already in the queue, we just **push a second, better entry** and leave the old one.

Later the old entry (the longer distance) reaches the front. By then `dist[u]` holds the better value, so `d > dist[u]` is true and we skip it. This is called **lazy deletion**. It costs a few extra entries in the queue, but the code stays simple, and the complexity is the same as the fancier version.

## A worked example

Take places A, B, C, D with two-way roads: A-B = 1, A-C = 4, B-C = 2, B-D = 6, C-D = 3. Find the route A to D.

```
          B
        / | \
     1 /  2|  \ 6
      /    |    \
     A     |     D
      \    |    /
     4 \   |   / 3
        \  |  /
          C
```

| Step | Take off queue | What happens | dist (A, B, C, D) |
|---|---|---|---|
| start | | push (0, A) | 0, inf, inf, inf |
| 1 | (0, A) | B = 1, C = 4. Push both. | 0, 1, 4, inf |
| 2 | (1, B) | C via B = 1 + 2 = 3, better than 4. Update, push (3, C). D via B = 1 + 6 = 7. Push (7, D). | 0, 1, 3, 7 |
| 3 | (3, C) | D via C = 3 + 3 = 6, better than 7. Update, push (6, D). | 0, 1, 3, 6 |
| 4 | (4, C) | 4 is more than dist[C] = 3. **Stale. Skipped.** | unchanged |
| 5 | (6, D) | D is the goal. Stop. | 0, 1, 3, 6 |

Parents: D came from C, C came from B, B came from A. Path: **A, B, C, D**, cost **6**, with 4 places expanded (A, B, C, D; the stale entry does not count).

Notice the direct-looking route A, B, D costs 7 and A, C, D costs 7, but the three-road route costs 6. BFS would have given A, B, D (2 hops, cost 7). That is exactly the difference between the two algorithms.

## Complexity of Dijkstra: O((V + E) log V), and why

Count the heap operations, since each costs O(log n).

- **Pushes.** A push happens only when a distance improves. That can happen at most once per edge (each edge is relaxed when its start place is expanded, and each place is expanded once). The start adds one more. So at most about **E + 1 pushes**.
- **Pops.** Every item pushed is popped at most once, so at most **E + 1 pops**. Most pops are real expansions (at most V of them); the rest are skipped stale entries.
- **Cost per heap operation.** The heap never holds more than E + 1 items. The cost is O(log(E + 1)). In a simple connected graph E is at most V², and log(V²) = 2 log V, so this is O(log V). The constant 2 is dropped in Big-O.
- **Edge scanning.** Each place's road list is scanned once, when it is expanded: **E** checks in total, each O(1), plus the heap work already counted.

Total: about (E pushes + E pops) × log V for the heap, plus V + E for the rest.

That gives **O((V + E) log V)**. (If you only count the heap operations you get O(E log V); writing V + E keeps the case where there are few edges and many places covered.)

Space: `dist`, `parent` are O(V), and the heap holds up to O(E) entries in the worst case because of the stale duplicates. Many references quote O(V) for an indexed heap; with lazy deletion the heap can briefly be larger, but it is still small in practice.

## When to prefer Dijkstra over BFS

Whenever roads have different lengths. If every road has the same length, Dijkstra and BFS find equivalent routes and BFS is cheaper (O(V + E) instead of O((V + E) log V), no heap needed).

---

# Part 5. A\* (pronounced "A-star")

## The problem it fixes

Dijkstra spreads out in **all directions** equally. If you want to go from the north of a city to the east, Dijkstra also carefully explores the whole south and west, because those places are "close to the start" too. That is wasted work.

```
Dijkstra explores a full circle        A* explores a narrow cone
around the start:                      pointing at the goal:

        . . . . .                              
      . . . . . . .                                . . 
     . . . S . . . .   G                    S . . . . . G
      . . . . . . .                                . .
        . . . . .                              
```

## The idea: use a guess

A\* adds a **guess** of how much distance is left from each place to the goal. It then processes first the place with the smallest

```
f(v)  =  g(v)  +  h(v)

g(v) = the real distance from the start to v, found so far   (what Dijkstra uses)
h(v) = a guess of the distance from v to the goal            (the new part)
```

A place that is close to the start but pointing away from the goal has a big `h`, so it waits. A place heading toward the goal has a small `h`, so it is explored early. Dijkstra is just A\* with `h = 0` for every place.

## Where the guess comes from

On a map, the straight-line distance between two places (`sqrt(dx*dx + dy*dy)`, the Euclidean distance) is a natural guess: you can never travel between two points in less than the straight line. In this project that is `Graph::heuristic(a, b)`.

## When the guess is safe

A\* only promises the shortest route if the guess follows two rules:

- **Admissible:** it never **over**estimates. The guess must be less than or equal to the true remaining distance. If it ever guessed too high, A\* might wrongly ignore the real best route.
- **Consistent:** it respects the triangle rule. The guess at `u` is never more than (road length from `u` to `v`) + (guess at `v`). Consistency implies admissibility, and it means the first time a place is taken off the queue, its distance is already the best.

Straight-line distance satisfies both **only if no road is shorter than the straight line between its ends**. A sample file could contain a road marked length 1 between two places that are 5 apart. To stay safe on any map, `Graph::add_edge` tracks the smallest ratio (road length / straight line) and `heuristic` multiplies the guess by it. That is the `h_scale_` you typed in `graph.hpp`. On normal maps it stays at 1.0 and changes nothing.

## The code

```cpp
PathResult astar(const Graph& g, NodeId start, NodeId goal) {
    PathResult r;
    vector<double> gscore(g.size(), INF);
    vector<NodeId> parent(g.size(), -1);
    vector<bool> closed(g.size(), false);
    MinQueue pq;
    gscore[start] = 0.0;
    pq.push(QItem(g.heuristic(start, goal), start));

    while (!pq.empty()) {
        NodeId u = pq.top().second;
        pq.pop();
        if (closed[u]) continue;
        closed[u] = true;
        r.expanded++;
        if (u == goal) {
            r.found = true;
            break;
        }
        for (const Edge& e : g.neighbors(u)) {
            double ng = gscore[u] + e.weight;
            if (ng < gscore[e.to]) {
                gscore[e.to] = ng;
                parent[e.to] = u;
                pq.push(QItem(ng + g.heuristic(e.to, goal), e.to));
            }
        }
    }
    if (r.found) {
        r.path = rebuild(parent, goal);
        r.cost = gscore[goal];
    }
    return r;
}
```

**How it differs from Dijkstra, exactly:**

1. `gscore` is Dijkstra's `dist`: the real distance from the start.
2. The queue is ordered by `ng + g.heuristic(e.to, goal)`, which is `g + h`, not by `g` alone. The start is pushed with `0 + h(start)`.
3. The queue no longer stores `g` as the first number; it stores `f`. So there is no `d` to compare for staleness. Instead there is a `closed` array: a place is **closed** the first time it comes off the queue. Because the guess is consistent, that first time its `gscore` is already the best possible, so any later copy of the same place in the queue is stale and is skipped with `if (closed[u]) continue;`.
4. When the goal is taken off the queue, stop. The answer is `gscore[goal]`.

## Why it is still correct

When the goal comes off the queue with `f = g + 0 = g`, every other place waiting has `f` at least that large. Since `f` never overestimates the cheapest route through that place, no waiting place can lead to a cheaper path to the goal. So the route found is the cheapest. This is the same argument as Dijkstra's, applied to `f` instead of `g`.

## Complexity of A\*

**Worst case: O((V + E) log V), the same as Dijkstra.** The code has the same structure: each place closed at most once, each edge relaxed at most once, so at most E pushes and E pops costing log each. If the guess is useless (for instance `h = 0`), A\* *is* Dijkstra.

**In practice it is much less.** The number of places expanded depends on how good the guess is:

- Perfect guess (`h` equals the true remaining distance): A\* walks straight down the best route and expands only the places on it.
- Zero guess: A\* behaves exactly like Dijkstra.
- Real straight-line guess on a map: somewhere in between, a cone toward the goal.

This is measured in the project's benchmark on a 300 by 300 grid (90,000 places): Dijkstra expanded about 45,000 places per query and A\* about 15,700, around 2.9 times fewer. The speed gain in seconds was smaller (about 2 times), because each A\* step must also compute a square root for the guess. That is a good honest thing to say in an interview: **fewer expansions does not automatically mean the same speedup in time, because each step is a bit more expensive.**

**Space:** O(V) for the arrays, plus the heap.

---

# Part 6. The helpers in `pathfinding.cpp`

## `rebuild`: turn parent links into a path

```cpp
static vector<NodeId> rebuild(const vector<NodeId>& parent, NodeId goal) {
    vector<NodeId> path;
    for (NodeId v = goal; v != -1; v = parent[v]) path.push_back(v);
    reverse(path.begin(), path.end());
    return path;
}
```

Start at the goal. Add it to the list. Move to its parent. Repeat until you reach -1 (the start's parent). The list is now goal-to-start, so `reverse` flips it to start-to-goal. Cost is O(path length), which is at most V.

Example with the earlier map: parent[D] = C, parent[C] = B, parent[B] = A, parent[A] = -1. The list builds up as D, C, B, A and reversing gives A, B, C, D.

## `path_cost`

Used only by BFS (the other two already know the cost). For each pair of neighbours on the path it finds the cheapest road between them, and adds those up. The "cheapest road" detail matters because a map might contain two roads between the same pair of places.

## `build_components`

```cpp
DSU build_components(const Graph& g) {
    DSU dsu(g.size());
    for (NodeId u = 0; u < g.size(); u++)
        for (const Edge& e : g.neighbors(u)) dsu.unite(u, e.to);
    return dsu;
}
```

Merges the two ends of every road into one group. It runs `unite` E times, each almost O(1), so the whole thing is about O(E). See `dsu-explained.docx` for why this is useful.

---

# Part 7. All three side by side

| | BFS | Dijkstra | A\* |
|---|---|---|---|
| Looks at road lengths while searching | no | yes | yes |
| Finds | fewest roads | shortest distance | shortest distance |
| To-do list | queue | min-heap | min-heap |
| Ordered by | discovery order | distance from start | distance from start + guess to goal |
| Needs non-negative lengths | n/a | yes | yes |
| Needs map coordinates | no | no | yes (for the guess) |
| Time | O(V + E) | O((V + E) log V) | O((V + E) log V) worst case, usually far fewer expansions |
| Space | O(V) | O(V) | O(V) |

Rule of thumb for choosing:

- All roads count the same (or you just want fewest hops): **BFS**.
- Roads have lengths and you have no useful guess: **Dijkstra**.
- Roads have lengths and you have a good, safe guess (like map coordinates): **A\***.

---

# Part 8. Questions you should be able to answer

1. **Why is BFS O(V + E) and not O(V × E)?** Each place enters the queue once, and each road list is scanned once, so every road is looked at once in total.
2. **Why does BFS not find the shortest distance?** It counts roads, not lengths. A route with fewer long roads beats a route with more short roads.
3. **Why does Dijkstra fail with negative roads?** Its proof says the nearest unprocessed place has a final distance, because adding more non-negative roads can only make a route longer. A negative road breaks that.
4. **What is lazy deletion and why use it?** The C++ priority queue cannot update an item, so we push a better duplicate and skip the stale one when it surfaces. Simple code, same Big-O.
5. **Where does the log V in Dijkstra come from?** From the heap: each push and pop costs about log of the heap size, and the heap size is bounded by about V squared, so log of it is at most 2 log V.
6. **What is the difference between Dijkstra and A\*?** Only the ordering of the queue: A\* adds a guess of the remaining distance. Dijkstra is A\* with a guess of zero.
7. **What does admissible mean? Consistent?** Admissible: the guess never overestimates. Consistent: the guess obeys the triangle rule, which ensures a place's distance is final the first time it is taken off the queue.
8. **When is A\* no better than Dijkstra?** When the guess is poor or zero. The worst case complexity is the same.
9. **Why did you scale the heuristic by `h_scale_`?** Because a road shorter than the straight line between its ends would make the plain straight-line guess overestimate, and A\* could return a non-optimal route. Scaling by the smallest road-to-straight-line ratio keeps it safe on any map. The tests catch it: turning the scaling off makes 279 checks fail.
10. **What does `expanded` measure and why use it?** The number of places taken off the to-do list and processed. It does not depend on machine speed, so it compares algorithms fairly.

---

# Part 9. Mistakes to avoid when typing

- Forgetting `#pragma once` or the two `#include`s at the top of the header.
- Missing the semicolon after the closing brace of `struct PathResult { ... };`.
- Writing the declarations with `Graph g` (no `&`): it compiles but copies the whole map every call, and `const` is lost.
- In BFS, marking `seen` when popping instead of when pushing. It still works but pushes duplicates.
- In Dijkstra, forgetting `if (d > dist[u]) continue;`. It still gives correct answers (just slower), which makes the bug hard to notice.
- In A\*, pushing `ng` instead of `ng + g.heuristic(e.to, goal)`. That turns it into Dijkstra, and the benchmark would show no gain.
- Writing `less` or leaving out `greater<QItem>`: the queue then returns the **largest** distance first and every answer is wrong.
