#pragma once  // C++ feature: if two files include this header, only paste it once
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>
using namespace std;

// NodeId is just another name for int. Every place on the map gets a number
// 0, 1, 2, ... so we can use it as an index into arrays (fast).
using NodeId = int;

// "Infinity" for distances: bigger than any real route length.
const double INF = 1e18;

// One road leaving some place. The place it leaves from is implied by which
// list it is stored in (see adj_ below).
struct Edge {
    NodeId to;
    double weight;  // road length
};

struct Node {
    string name;
    double x, y;  // position on the map, used by A*
};

// Weighted graph stored as an ADJACENCY LIST: adj_[u] is the list of roads
// leaving place u. A city map is sparse (few roads per place), so this uses
// O(V + E) memory and visiting u's neighbours costs O(degree of u).
// An adjacency matrix would need V * V memory, far too much for big maps.
class Graph {
public:
    // "const string&" = read the string without making a copy of it.
    NodeId add_node(const string& name, double x, double y);

    // A two-way road is stored as TWO directed edges (a->b and b->a).
    // A one-way road stores only a->b.
    void add_edge(NodeId a, NodeId b, double weight, bool two_way = true);

    // Look up a place by name. Returns -1 if there is no such place.
    // ("const" after the ) means: this function does not change the graph.)
    
    NodeId find(const string& name) const;

    //getter functions
    int size() const { return (int)nodes_.size(); }
    const Node& node(NodeId id) const { return nodes_[id]; }
    const vector<Edge>& neighbors(NodeId id) const { return adj_[id]; }

    // Straight-line distance between two places.
    double euclid(NodeId a, NodeId b) const {
        double dx = nodes_[a].x - nodes_[b].x;
        double dy = nodes_[a].y - nodes_[b].y;
        return sqrt(dx * dx + dy * dy);
    }

    // A* heuristic = a guess of the distance still to go: the straight line
    // to the goal. The guess must never be BIGGER than the real distance
    // (then A* stays correct). A road can be shorter than the straight line
    // between its ends in a badly made map, so we shrink the guess by
    // h_scale_, the smallest (road length / straight line) ratio seen so far.
    double heuristic(NodeId a, NodeId b) const { return euclid(a, b) * h_scale_; }

    // Reads a map file. "static" = call it as Graph::load(path), no Graph needed.
    // On bad input it stops with an error that says which file and line.
    static Graph load(const string& path);

private:
    vector<Node> nodes_;
    vector<vector<Edge>> adj_;            // the adjacency list
    unordered_map<string, NodeId> index_; // hash map: place name -> id, O(1) lookup
    double h_scale_ = 1.0;
};
