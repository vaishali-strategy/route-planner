# City Route Planner

A command-line route planner in C++17. It loads a city map (places and roads) from a text file and finds the best route between two places with **BFS**, **Dijkstra** and **A\***, and uses **Union-Find** to reject trips between disconnected parts of the map before any search runs. Standard library only.

```
$ ./build/planner data/city.txt Station Airport
[bfs] Station -> Market -> Mall -> Stadium -> Airport
    cost 13.90, hops 4, expanded 11
[dijkstra] Station -> Market -> Library -> Hospital -> University -> Airport
    cost 13.80, hops 5, expanded 11
[astar] Station -> Market -> Library -> Hospital -> University -> Airport
    cost 13.80, hops 5, expanded 8
```

BFS minimises the number of roads (4 hops, 13.9 km); Dijkstra and A\* minimise distance (5 hops, 13.8 km). Same trip with different best route.

## Build and run

```bash
make all      # planner, test runner, benchmark (needs a C++17 compiler)
make test     # unit + randomized tests
make asan     # the same tests under AddressSanitizer + UBSan
make bench    # Dijkstra vs A* on a 90,000-node grid

./build/planner <map-file> <from> <to> [--algo bfs|dijkstra|astar|all]
```

Exit codes: `0` route printed, `1` bad input or unknown place, `2` no route exists.

## Map format

```
# comment
node <name> <x> <y>                # names have no spaces
edge <from> <to> <length> [oneway]
```

Roads are two-way unless marked `oneway`. Malformed input is reported as `file:line: message`. See `data/city.txt` for an example with a one-way road and a disconnected island.

## Complexity

| Operation | Time | Space |
|---|---|---|
| Load map | O(V + E) | O(V + E) |
| BFS | O(V + E) | O(V) |
| Dijkstra | O((V + E) log V) | O(V) |
| A\* | O((V + E) log V) worst case, far less in practice | O(V) |
| Union-Find build / query | O(E α(V)) / O(α(V)) | O(V) |

## Design decisions

- **Adjacency list, not matrix.** City maps are sparse, so memory is O(V + E). A matrix for 90,000 places would need about 65 GB.
- **Dijkstra with lazy deletion.** `std::priority_queue` has no decrease-key, so a shorter route pushes a new entry and the stale one is skipped when popped. Same complexity as an indexed heap with far less code.
- **A\* stays optimal on any map.** Straight-line distance is only a valid guess if no road is shorter than the straight line between its ends. The graph tracks the smallest (road length / straight line) ratio while edges are added and scales the heuristic by it, so it is always admissible and consistent.
- **Union-Find as a pre-check.** Without it, an impossible query makes Dijkstra explore the whole connected component before giving up. With one-way roads "connected" is necessary but not sufficient, so the search still decides.
- **Negative road lengths are rejected**, since Dijkstra is incorrect with them.

## Testing

`tests/test_main.cpp` checks Dijkstra and A\* against Floyd-Warshall on 300 random graphs (3,000 queries), with one-way roads and with roads shorter than the straight line. It also covers start = goal, unreachable places, one-way roads, negative lengths, the sample map and a missing file. All tests run clean under AddressSanitizer and UBSan.

## Benchmark

300 x 300 grid (90,000 places), 50 random queries, `-O2`. Both algorithms answer identical queries; equal cost sums confirm identical answers.

```
dijkstra   avg expanded 45344     total ms 187.3     (sum of costs 10772.96)
astar      avg expanded 15740     total ms 88.6      (sum of costs 10772.96)
```

A\* examines about 2.9x fewer places and runs about 2x faster. The time gain is smaller than the expansion gain because each A\* step also computes a square root for the heuristic. The expansion counts are exact and repeatable; the milliseconds vary by machine.

## Limitations

- Coordinates are flat 2D (no latitude/longitude), and there is no turn cost or live traffic.
- Maps are text files with made up values, not real map data.
- One shortest route is returned; ties are broken arbitrarily.

## Layout

```
include/graph.hpp         adjacency-list graph, map loader, A* heuristic
include/dsu.hpp           Union-Find (path compression + union by size)
include/pathfinding.hpp   BFS / Dijkstra / A* interface
src/                      implementations and the CLI
tests/test_main.cpp       randomized tests vs Floyd-Warshall
bench/bench.cpp           Dijkstra vs A* benchmark
data/city.txt             sample map
```
