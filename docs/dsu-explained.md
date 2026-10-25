# include/dsu.hpp Explained, and Why the Project Needs Union-Find

## 1. What it is, in one paragraph

DSU stands for **Disjoint Set Union**, also called **Union-Find**. It answers one question very fast: *"are these two things in the same group?"* You start with every item in its own group of one. You can **merge** two groups (`unite`), and you can ask **which group** an item belongs to (`find`). Two items are in the same group if `find` gives the same answer for both.

In this project the "items" are places on the map and a "group" is a set of places that are joined by roads. So `connected(Station, Airport)` means "can you get from Station to Airport by following roads?"

## 2. Why this project needs it

**The problem.** Suppose someone asks for a route from `Station` to `Island`, and `Island` has no road to anywhere. Dijkstra cannot know that in advance. It explores every place reachable from `Station`, one by one, before finally running out of places and giving up. On a big map that is a lot of work to learn "there is no route". BFS and A\* do the same.

**The fix.** Before searching, check whether the two places are in the same group. If they are not, say "no route" immediately and skip the search entirely.

In the sample map:

```
12 places, 2 separate groups
Station ~ Airport  -> connected: true   (search is worth running)
Station ~ Island   -> connected: false  (answer "no route" at once)
```

**What it costs.** Building the structure means looking at each road once, which is about O(E) work. After that, each "are these connected?" question costs almost O(1). A failed search costs O((V + E) log V). In `main.cpp` the check happens once, before running up to three algorithms, so one cheap check can save up to three expensive failed searches. If the program answered many queries on the same map, you would build the structure once and reuse it every time.

**An honest caveat: one-way roads.** `build_components` ignores road direction. So:

- If DSU says **not connected**, that is certain: there is no route, even ignoring direction.
- If DSU says **connected**, a route is *likely* but not guaranteed. With one-way roads, you could be able to reach B from A but not the other way round. The search still has the final word.

So the DSU check is a quick **rejection filter**. It never replaces the search.

**Where else you will meet Union-Find.** It is a very common interview topic:
- **Kruskal's algorithm** for a minimum spanning tree.
- Counting connected components or islands in a grid or graph.
- Detecting a cycle in an undirected graph.
- Grouping items by a "same as" relationship, such as accounts that share an email.

## 3. The idea: every group is a tree

Each group is stored as a **tree**. The root of the tree is the group's **leader**. Every item remembers one thing: its **parent**.

```
Before anything is merged: everyone is their own leader.

   0     1     2     3     4

After unite(0,1), unite(1,2), unite(3,4):

     0           3
    / \          |
   1   2         4

group {0,1,2} has leader 0.   group {3,4} has leader 3.
```

To find an item's group, follow parent links up until you reach an item that is its own parent. That is the leader. Two items are in the same group if they reach the same leader.

## 4. The code, piece by piece

### The top

```cpp
#pragma once
#include <vector>
using namespace std;
```

`#pragma once` stops the header being pasted twice. `<vector>` is needed because the data is stored in vectors. `using namespace std;` lets us write `vector` instead of `std::vector`.

### The data

```cpp
private:
    vector<int> parent_;
    vector<int> size_;
```

Two lists, each with one entry per item.

- `parent_[x]` is x's parent in its tree. If `parent_[x] == x`, then x is a leader.
- `size_[x]` is how many items are in the group whose leader is x. This is only meaningful for leaders. It is used to decide which tree to hang under which.

They are `private`, so only the class's own functions can touch them.

### The constructor

```cpp
DSU(int n) {
    for (int i = 0; i < n; i++) {
        parent_.push_back(i);
        size_.push_back(1);
    }
}
```

A constructor runs when you create a DSU, as in `DSU dsu(g.size());`. For each of the n items, `parent_[i] = i` makes everyone their own leader, and `size_[i] = 1` records that each group holds one item. That is the "everyone starts alone" state.

### `find`: who is the leader?

```cpp
int find(int x) {
    if (parent_[x] != x) parent_[x] = find(parent_[x]);
    return parent_[x];
}
```

Read it in plain English:

- If x is not its own parent, ask its parent who the leader is (a recursive call), and **remember the answer** by storing it in `parent_[x]`.
- Then return `parent_[x]`.

That "remember the answer" step is **path compression**. Suppose 3 -> 2 -> 1 -> 0 is a chain, with 0 as the leader. After `find(3)`:

```
Before:  3 -> 2 -> 1 -> 0          After find(3):   0
                                                   /|\
                                                  1 2 3
```

Every item on the path now points straight at the leader, so the next `find` from any of them takes one step instead of three. The trees flatten out as you use them.

The function is recursive, meaning it calls itself. It stops when it reaches a leader, where `parent_[x] == x`. This is safe here, because the next section keeps trees shallow.

### `unite`: merge two groups

```cpp
bool unite(int a, int b) {
    a = find(a);
    b = find(b);
    if (a == b) return false;
    if (size_[a] < size_[b]) swap(a, b);
    parent_[b] = a;
    size_[a] += size_[b];
    return true;
}
```

Step by step:

1. `a = find(a); b = find(b);` replace the two items with their **leaders**. We always merge whole groups, not single items. From here on, `a` and `b` are leaders.
2. `if (a == b) return false;` the two items already have the same leader, so they are already in the same group. Nothing to do. Returning `false` tells the caller "nothing changed".
3. `if (size_[a] < size_[b]) swap(a, b);` makes `a` the **bigger** group's leader. `swap` exchanges the two values.
4. `parent_[b] = a;` hangs the smaller tree under the bigger tree's leader.
5. `size_[a] += size_[b];` the merged group has both sizes added.
6. `return true;` something was merged.

**Why hang the small tree under the big one?** This is called **union by size**. If you attached the big tree under the small one, the big tree's items would all get one level deeper. Hanging the small under the big adds one level to only a few items. Trees stay short. A tree with n items can never be taller than about log n.

### `connected`

```cpp
bool connected(int a, int b) { return find(a) == find(b); }
```

Same leader means same group. It is one line because `find` does all the work.

## 5. A worked example

Using the demo from the project (items 0 to 4):

| Step | Call | Result | Groups afterwards |
|---|---|---|---|
| 1 | `unite(0, 1)` | `true` | {0,1} {2} {3} {4} |
| 2 | `unite(1, 2)` | `true` | {0,1,2} {3} {4} |
| 3 | `unite(3, 4)` | `true` | {0,1,2} {3,4} |
| 4 | `find(0)` ... `find(2)` | all give `0` | leader of the first group is 0 |
| 5 | `find(3)`, `find(4)` | both give `3` | leader of the second group is 3 |
| 6 | `connected(2, 4)` | `false` | different leaders, 0 and 3 |
| 7 | `unite(2, 4)` | `true` | the two groups merge into {0,1,2,3,4} |
| 8 | `find(2)`, `find(4)`, `find(3)` | all give `0` | the bigger group (size 3) kept its leader |
| 9 | `unite(0, 2)` | `false` | already in the same group |

In step 7, `find(2)` is 0 and `find(4)` is 3. Group 0 has size 3 and group 3 has size 2, so group 3 hangs under 0. That is union by size at work.

## 6. How the project uses it

In `pathfinding.cpp`:

```cpp
DSU build_components(const Graph& g) {
    DSU dsu(g.size());
    for (NodeId u = 0; u < g.size(); u++)
        for (const Edge& e : g.neighbors(u)) dsu.unite(u, e.to);
    return dsu;
}
```

Make a DSU with one item per place. Then go through every road and merge its two ends. When the loops finish, every set of places joined by roads is one group.

In `main.cpp`:

```cpp
DSU comps = build_components(g);
if (!comps.connected(from, to)) {
    cout << "No route: ...";
    return 2;
}
```

Before running any search, ask whether the two places are connected. If not, print the message and exit with code 2.

## 7. Speed

| Operation | Cost |
|---|---|
| Create `DSU(n)` | O(n) |
| `find` | almost O(1) |
| `unite` | almost O(1) |
| `connected` | almost O(1) |
| Build for a whole map | O(E) calls to `unite` |

"Almost O(1)" has a precise name: **O(alpha(n))**, where alpha is the *inverse Ackermann function*. It grows so slowly that it is at most 4 for any number of items you could ever store. You get that speed only by using **both** tricks together, path compression and union by size. With neither, a find can take O(n) in the worst case.

## 8. Questions you should be able to answer

1. **What problem does Union-Find solve?** Tracking which items belong to the same group while groups merge over time, with near-constant-time queries.
2. **Why do we need it in a route planner?** To reject a trip between disconnected parts of the map immediately, instead of making Dijkstra explore everything reachable first.
3. **If DSU says "connected", is there definitely a route?** Not with one-way roads. The check ignores direction. "Not connected" is a definite no. "Connected" only means a route is possible, so the search decides.
4. **What are the two optimisations, and what does each do?** Path compression flattens a tree whenever you search it. Union by size keeps trees shallow from the start by hanging the smaller tree under the bigger one.
5. **Why does `unite` call `find` first?** Because merging works on groups, so we need each item's leader. Hanging one *item* under another, instead of one *leader*, would break the trees.
6. **Why use `size_` only for leaders?** Only the leader's `size_` entry is kept up to date. Other entries become stale once the item stops being a leader.
7. **Could you use BFS instead?** Yes. BFS could check connectivity for one pair, but each query costs O(V + E). With DSU you build once and every later query is nearly free.

## 9. Mistakes to avoid when typing it

- In `find`, writing `find(parent_[x])` without storing it back (`parent_[x] = ...`). It still works, but you lose path compression.
- In `unite`, comparing `size_[a]` and `size_[b]` **before** replacing a and b with their leaders. Always call `find` first.
- Forgetting `return false` when both items already share a leader, or forgetting the final `return true`.
- Writing `size_[b] += size_[a]` in the last step instead of `size_[a] += size_[b]`. The sizes grow at the leader that kept the tree.
- Leaving off the `;` after the closing brace of the class: `};`
