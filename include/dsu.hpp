#pragma once
#include <vector>
using namespace std;

// Union-Find (disjoint sets): keeps track of which places are connected.
// Every set is a tree; its root is the set's "leader". find(x) returns the
// leader of x's set, so two items are in the same set if they have the
// same leader.
class DSU {
public:
    DSU(int n) {
        for (int i = 0; i < n; i++) {
            parent_.push_back(i);  // at the start everyone is their own leader
            size_.push_back(1);
        }
    }

    // Path compression: after finding the leader, point x straight at it,
    // so the next find from x (or its descendants) is quicker.
    int find(int x) {
        if (parent_[x] != x) parent_[x] = find(parent_[x]);
        return parent_[x];
    }

    // Merge the sets of a and b. Union by size: hang the smaller tree under
    // the bigger one so trees stay shallow. Together with path compression
    // each operation is almost O(1) (technically O(inverse Ackermann(n))).
    bool unite(int a, int b) {
        a = find(a);
        b = find(b);
        if (a == b) return false;  // already in the same set
        if (size_[a] < size_[b]) swap(a, b);
        parent_[b] = a;
        size_[a] += size_[b];
        return true;
    }

    bool connected(int a, int b) { return find(a) == find(b); }

private:
    vector<int> parent_;
    vector<int> size_;
};
