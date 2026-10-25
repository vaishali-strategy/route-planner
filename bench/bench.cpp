// Compares Dijkstra and A* on an N x N grid "city" with slightly wiggly roads.
// Build + run:  make bench   (or: ./build/bench 300)
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <string>

#include "pathfinding.hpp"

// A function pointer: a variable that holds a function. It lets us pass
// dijkstra or astar into run() below, as long as they have this shape.
typedef PathResult (*Algorithm)(const Graph&, int, int);

double rand_between(double lo, double hi) { return lo + (hi - lo) * rand() / RAND_MAX; }

// Places sit on a grid; each place has a road to its right and below it.
// A road is 1.0 to 1.5 times the straight-line length.
Graph make_grid(int n) {
    Graph g;
    for (int r = 0; r < n; r++)
        for (int c = 0; c < n; c++)
            g.add_node(to_string(r) + "_" + to_string(c), c, r);
    for (int r = 0; r < n; r++)
        for (int c = 0; c < n; c++) {
            int id = r * n + c;
            if (c + 1 < n) g.add_edge(id, id + 1, rand_between(1.0, 1.5));
            if (r + 1 < n) g.add_edge(id, id + n, rand_between(1.0, 1.5));
        }
    return g;
}

void run(const string& name, const Graph& g, Algorithm algo, int queries) {
    srand(99);  // same seed for every algorithm, so they answer the SAME queries
    size_t expanded = 0;
    double cost = 0;
    clock_t start_time = clock();  // clock() counts CPU time used so far
    for (int i = 0; i < queries; i++) {
        int s = rand() % g.size();
        int t = rand() % g.size();
        PathResult r = algo(g, s, t);
        expanded += r.expanded;
        cost += r.cost;
    }
    double ms = 1000.0 * (clock() - start_time) / CLOCKS_PER_SEC;
    cout << left << setw(10) << name << " avg expanded " << setw(9) << expanded / queries
         << " total ms " << setw(9) << fixed << setprecision(1) << ms
         << " (sum of costs " << setprecision(2) << cost << ")\n";
}

int main(int argc, char** argv) {
    int n = 200;
    if (argc > 1) n = stoi(argv[1]);  // stoi: text to int
    srand(7);
    Graph g = make_grid(n);
    int queries = 50;
    cout << "grid " << n << "x" << n << " = " << g.size() << " places, " << queries
         << " random queries\n";
    // If both algorithms give the same "sum of costs", they gave the same answers.
    run("dijkstra", g, dijkstra, queries);
    run("astar", g, astar, queries);
}
