#include <iomanip>
#include <iostream>
#include <string>

#include "pathfinding.hpp"

static void usage() {
    cerr << "usage: planner <map-file> <from> <to> [--algo bfs|dijkstra|astar|all]\n";
}

static void report(const Graph& g, const string& label, const PathResult& r) {
    cout << "[" << label << "] ";
    if (!r.found) {
        cout << "no route\n";
        return;
    }
    for (size_t i = 0; i < r.path.size(); i++) {
        if (i > 0) cout << " -> ";
        cout << g.node(r.path[i]).name;
    }
    // fixed + setprecision(2) = always print exactly 2 digits after the point
    cout << "\n    cost " << fixed << setprecision(2) << r.cost
         << ", hops " << r.path.size() - 1 << ", expanded " << r.expanded << "\n";
}

int main(int argc, char** argv) {
    // argc = number of words typed on the command line, argv[i] = word number i.
    // argv[0] is the program name, so we need at least 4 words.
    if (argc < 4) {
        usage();
        return 1;
    }
    string algo = "all";
    for (int i = 4; i < argc; i++) {
        if (string(argv[i]) == "--algo" && i + 1 < argc) {
            i = i + 1;
            algo = argv[i];
        } else {
            usage();
            return 1;
        }
    }
    if (algo != "all" && algo != "bfs" && algo != "dijkstra" && algo != "astar") {
        usage();
        return 1;
    }

    // try/catch: if Graph::load throws an error, jump to the catch block
    // below and print the message instead of crashing.
    try {
        Graph g = Graph::load(argv[1]);
        NodeId from = g.find(argv[2]);
        NodeId to = g.find(argv[3]);
        if (from == -1 || to == -1) {
            cerr << "unknown place: " << (from == -1 ? argv[2] : argv[3]) << "\n";
            return 1;
        }

        // Union-Find check first: if the places are in separate parts of the map,
        // skip the search entirely.
        DSU comps = build_components(g);
        if (!comps.connected(from, to)) {
            cout << "No route: " << argv[2] << " and " << argv[3]
                 << " are in disconnected parts of the map.\n";
            return 2;
        }

        if (algo == "all" || algo == "bfs") report(g, "bfs", bfs(g, from, to));
        if (algo == "all" || algo == "dijkstra") report(g, "dijkstra", dijkstra(g, from, to));
        if (algo == "all" || algo == "astar") report(g, "astar", astar(g, from, to));
    } catch (const exception& ex) {
        cerr << "error: " << ex.what() << "\n";
        return 1;
    }
    return 0;
}
