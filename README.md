# City Route Planner

This is a command-line route planner in C++17. It loads a city map (places and roads) from a text file and finds the best route between two places using the **BFS**, **Dijkstra** and **A\*** algorithms, and uses **Union-Find** to reject trips between disconnected parts of the map before any search runs. 

```
Example:
$ ./build/planner data/city.txt Station Airport
[bfs] Station -> Market -> Mall -> Stadium -> Airport
    cost 13.90, hops 4, expanded 11
[dijkstra] Station -> Market -> Library -> Hospital -> University -> Airport
    cost 13.80, hops 5, expanded 11
[astar] Station -> Market -> Library -> Hospital -> University -> Airport
    cost 13.80, hops 5, expanded 8
```

Here in the example given, BFS minimises the number of roads (4 hops, 13.9 km), while Dijkstra and A\* minimise distance (5 hops, 13.8 km). Same trip with different best routes are found.

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

Roads are two-way unless marked `oneway`. Malformed input is reported as `file:line: message`. `data/city.txt` has both-way roads and one-way road and a disconnected island.

## Complexity

| Operation | Time | Space |
|---|---|---|
| Load map | O(V + E) | O(V + E) |
| BFS | O(V + E) | O(V) |
| Dijkstra | O((V + E) log V) | O(V) |
| A\* | O((V + E) log V) worst case, far less in practice | O(V) |
| Union-Find build / query | O(E α(V)) / O(α(V)) | O(V) |

## Design decisions

- **Adjacency list, not adjacency matrix.** City maps are sparse, so memory is O(V + E).
- **Dijkstra with lazy deletion.** `std::priority_queue` has no decrease-key, so a shorter route pushes a new entry and the old one is skipped when popped. It has the same complexity as an indexed heap with far less code.
- **A\* stays optimal on any map.** Straight-line distance is a valid guess only if no road is shorter than the straight line between its ends. The graph tracks the smallest (road length / straight line) ratio while edges are added and scales the heuristic by it, so it is always consistent.
- **Union-Find** Without it, an impossible query makes Dijkstra explore the whole connected component before giving up. With one-way roads "connected" is necessary but not sufficient, so the search still decides.
- **Negative road lengths are rejected**, since Dijkstra is incorrect with them.

## Testing

`tests/test_main.cpp` checks Dijkstra and A\* against Floyd-Warshall on 300 random graphs (3,000 queries), with one-way roads and with roads shorter than the straight line. It also covers start = goal, unreachable places, one-way roads, negative lengths, the sample map and a missing file. All tests run clean under AddressSanitizer and UBSan.

## Benchmark

300 x 300 grid (90,000 places), 50 random queries, `-O2`. Both algorithms answer same queries, and the equal cost sums confirm identical answers.

```
dijkstra   avg expanded 45344     total ms 187.3     (sum of costs 10772.96)
astar      avg expanded 15740     total ms 88.6      (sum of costs 10772.96)
```

A\* examines about 2.9x fewer places and runs about 2x faster. The time gain is smaller than the expansion gain because each A\* step also computes a square root for the heuristic. 

## Limitations

- The coordinates are flat 2D (no latitude/longitude) and there is no turn cost or live traffic. The maps are text files with made up values, and not real map data.
- Only one shortest route is returned as the final result; ties are broken arbitrarily.

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
