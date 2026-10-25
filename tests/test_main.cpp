#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "pathfinding.hpp"

int failures = 0;

// A macro is text that the compiler pastes in before compiling. CHECK(x)
// becomes an if-statement that counts and prints a failure when x is false.
// #cond turns the expression into text so the message shows what failed.
#define CHECK(cond)                                                              \
    if (!(cond)) {                                                               \
        failures++;                                                              \
        cerr << __FILE__ << ":" << __LINE__ << " FAILED: " #cond "\n";           \
    }

// Decimal numbers can differ in the last digits, so compare with a tiny margin.
bool same(double a, double b) { return fabs(a - b) < 1e-9; }

// rand() gives a random int. These helpers turn it into numbers we can use.
// srand(seed) in main makes the "random" numbers the same on every run,
// so a failing test can be repeated.
double rand_between(double lo, double hi) { return lo + (hi - lo) * rand() / RAND_MAX; }

// A -1- B -1- C, plus a long direct road A -5- C.
Graph triangle() {
    Graph g;
    g.add_node("A", 0, 0);
    g.add_node("B", 1, 0);
    g.add_node("C", 2, 0);
    g.add_edge(0, 1, 1.0);
    g.add_edge(1, 2, 1.0);
    g.add_edge(0, 2, 5.0);
    return g;
}

void test_dsu() {
    DSU d(5);
    CHECK(!d.connected(0, 1));
    CHECK(d.unite(0, 1));
    CHECK(d.unite(1, 2));
    CHECK(!d.unite(0, 2));  // already joined
    CHECK(d.connected(0, 2));
    CHECK(!d.connected(2, 3));
}

void test_basic_paths() {
    Graph g = triangle();
    // Dijkstra and A* take the two short roads; BFS takes the single direct hop.
    PathResult dj = dijkstra(g, 0, 2);
    CHECK(dj.found && same(dj.cost, 2.0) && dj.path.size() == 3 && dj.path[1] == 1);
    PathResult as = astar(g, 0, 2);
    CHECK(as.found && same(as.cost, 2.0));
    PathResult bf = bfs(g, 0, 2);
    CHECK(bf.found && bf.path.size() == 2 && same(bf.cost, 5.0));
}

void check_trivial(const PathResult& r) {
    CHECK(r.found && r.path.size() == 1 && r.path[0] == 1 && same(r.cost, 0.0));
}

void test_start_equals_goal() {
    Graph g = triangle();
    check_trivial(bfs(g, 1, 1));
    check_trivial(dijkstra(g, 1, 1));
    check_trivial(astar(g, 1, 1));
}

void test_unreachable_and_oneway() {
    Graph g;
    g.add_node("A", 0, 0);
    g.add_node("B", 1, 0);
    g.add_node("Z", 9, 9);
    g.add_edge(0, 1, 1.0, false);  // one-way: A -> B only
    CHECK(dijkstra(g, 0, 1).found);
    CHECK(!dijkstra(g, 1, 0).found);
    CHECK(!astar(g, 0, 2).found);
    CHECK(!bfs(g, 0, 2).found);
    DSU c = build_components(g);
    CHECK(c.connected(0, 1) && !c.connected(0, 2));
}

void test_negative_length_rejected() {
    Graph g = triangle();
    bool threw = false;
    try {
        g.add_edge(0, 1, -1.0);
    } catch (const invalid_argument&) {
        threw = true;
    }
    CHECK(threw);
}

// Reference answer: Floyd-Warshall tries every middle place k for every pair
// (i, j), O(V^3). Far too slow for real maps but obviously correct, so we use
// it to check our fast algorithms on small graphs.
vector<vector<double>> floyd(const Graph& g) {
    int n = g.size();
    vector<vector<double>> d(n, vector<double>(n, INF));
    for (int i = 0; i < n; i++) {
        d[i][i] = 0;
        for (const Edge& e : g.neighbors(i)) d[i][e.to] = min(d[i][e.to], e.weight);
    }
    for (int k = 0; k < n; k++)
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                if (d[i][k] + d[k][j] < d[i][j]) d[i][j] = d[i][k] + d[k][j];
    return d;
}

// n random places, m random roads. Each road is between min_factor and 2 times
// the straight-line distance. min_factor < 1 makes some roads SHORTER than the
// straight line, which tests the heuristic scaling in Graph::heuristic.
Graph random_graph(int n, int m, double min_factor) {
    Graph g;
    for (int i = 0; i < n; i++)
        g.add_node("n" + to_string(i), rand_between(0, 100), rand_between(0, 100));
    for (int j = 0; j < m; j++) {
        int a = rand() % n;
        int b = rand() % n;
        if (a == b) continue;
        double length = g.euclid(a, b) * rand_between(min_factor, 2.0);
        bool two_way = (rand() % 5 != 0);  // about 1 road in 5 is one-way
        g.add_edge(a, b, length, two_way);
    }
    return g;
}

// Add up the road lengths along a path, to prove the path is real.
double walk_cost(const Graph& g, const vector<NodeId>& p) {
    double total = 0;
    for (size_t i = 0; i + 1 < p.size(); i++) {
        double best = INF;
        for (const Edge& e : g.neighbors(p[i]))
            if (e.to == p[i + 1]) best = min(best, e.weight);
        total += best;
    }
    return total;
}

// Compare Dijkstra and A* with Floyd-Warshall on many random graphs.
void test_random_against_floyd() {
    double factors[2] = {1.0, 0.4};
    for (int f = 0; f < 2; f++) {
        for (int trial = 0; trial < 150; trial++) {
            Graph g = random_graph(25, 70, factors[f]);
            vector<vector<double>> ref = floyd(g);
            for (int q = 0; q < 10; q++) {
                int s = rand() % g.size();
                int t = rand() % g.size();
                PathResult dj = dijkstra(g, s, t);
                PathResult as = astar(g, s, t);
                bool reachable = ref[s][t] < INF;
                CHECK(dj.found == reachable);
                CHECK(as.found == reachable);
                if (!reachable) continue;
                CHECK(same(dj.cost, ref[s][t]));
                CHECK(same(as.cost, ref[s][t]));
                CHECK(same(walk_cost(g, as.path), ref[s][t]));  // the path really adds up
                CHECK(as.path.front() == s && as.path.back() == t);
            }
        }
    }
}

void test_sample_map_loads() {
    Graph g = Graph::load("data/city.txt");
    int s = g.find("Station");
    int a = g.find("Airport");
    int i = g.find("Island");
    CHECK(s != -1 && a != -1 && i != -1);
    PathResult r = dijkstra(g, s, a);
    CHECK(r.found);
    CHECK(same(astar(g, s, a).cost, r.cost));
    CHECK(!build_components(g).connected(s, i));
    // The one-way express road makes Airport -> Station cost 11.5.
    CHECK(dijkstra(g, a, s).cost < 11.6);
}

void test_missing_file_reports_error() {
    bool threw = false;
    try {
        Graph::load("data/does-not-exist.txt");
    } catch (const runtime_error&) {
        threw = true;
    }
    CHECK(threw);
}

int main() {
    srand(12345);
    test_dsu();
    test_basic_paths();
    test_start_equals_goal();
    test_unreachable_and_oneway();
    test_negative_length_rejected();
    test_random_against_floyd();
    test_sample_map_loads();
    test_missing_file_reports_error();
    if (failures > 0) {
        cerr << failures << " check(s) failed\n";
        return 1;
    }
    cout << "all tests passed\n";
    return 0;
}
