#include "graph.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>

NodeId Graph::add_node(const string& name, double x, double y) {
    // "throw" stops the function with an error; the caller can catch it.
    if (index_.count(name) > 0) throw runtime_error("duplicate place: " + name);
    NodeId id = (int)nodes_.size();
    nodes_.push_back({name, x, y});
    adj_.push_back(vector<Edge>());  // empty road list for the new place
    index_[name] = id;
    return id;
}

void Graph::add_edge(NodeId a, NodeId b, double weight, bool two_way) {
    // Dijkstra gives wrong answers with negative lengths, so refuse them.
    if (weight < 0) throw invalid_argument("negative road length");
    adj_[a].push_back({b, weight});
    if (two_way) adj_[b].push_back({a, weight});
    double d = euclid(a, b);
    if (d > 0) h_scale_ = min(h_scale_, weight / d);  // keep the A* guess safe
}

NodeId Graph::find(const string& name) const {
    auto it = index_.find(name);  // "auto" = let the compiler work out the type
    if (it == index_.end()) return -1;
    return it->second;  // the id stored in the hash map
}

// Builds the "file:line: message" text used for all load errors.
static void fail(const string& path, int line_number, const string& message) {
    throw runtime_error(path + ":" + to_string(line_number) + ": " + message);
}

Graph Graph::load(const string& path) {
    ifstream in(path);  // opens the file for reading, like cin but from a file
    if (!in) throw runtime_error("cannot open " + path);

    Graph g;
    string line;
    int line_number = 0;
    while (getline(in, line)) {
        line_number++;

        // Cut off a "# comment" if there is one.
        size_t hash_pos = line.find('#');
        if (hash_pos != string::npos) line.erase(hash_pos);

        // istringstream lets us read words out of a string the way cin does.
        istringstream words(line);
        string kind;
        if (!(words >> kind)) continue;  // blank line

        if (kind == "node") {
            string name;
            double x, y;
            if (!(words >> name >> x >> y)) fail(path, line_number, "expected: node <name> <x> <y>");
            g.add_node(name, x, y);
        } else if (kind == "edge") {
            string a, b, flag;
            double weight;
            if (!(words >> a >> b >> weight)) fail(path, line_number, "expected: edge <from> <to> <length> [oneway]");
            NodeId ia = g.find(a);
            NodeId ib = g.find(b);
            if (ia == -1 || ib == -1) fail(path, line_number, "road refers to an unknown place");
            bool two_way = true;
            if (words >> flag) {
                if (flag != "oneway") fail(path, line_number, "unknown flag: " + flag);
                two_way = false;
            }
            g.add_edge(ia, ib, weight, two_way);
        } else {
            fail(path, line_number, "unknown record: " + kind);
        }
    }
    return g;
}
