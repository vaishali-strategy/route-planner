# src/graph.cpp Explained

`graph.cpp` is the **implementation** file for the `Graph` class. `graph.hpp` lists the functions and what they promise; this file writes the bodies. It has five parts: `add_node`, `add_edge`, `find`, the `fail` helper, and `load`.

## The `Graph::` prefix

```cpp
NodeId Graph::add_node(...)
```

`Graph::` means "this function belongs to the class `Graph`". That is what lets the body use `nodes_`, `adj_` and `index_` directly, because they are members of the same class.

## 1. `add_node`: register a place

```cpp
if (index_.count(name) > 0) throw runtime_error("duplicate place: " + name);
NodeId id = (int)nodes_.size();
nodes_.push_back({name, x, y});
adj_.push_back(vector<Edge>());
index_[name] = id;
return id;
```

1. **Duplicate check.** `index_` is the hash map from name to id. `count(name)` is 1 if the name is already in it. If so, `throw` stops the function with an error.
2. **New id.** The next free number is the current size. The first place gets 0, the second gets 1, and so on.
3. **Store the place.** `{name, x, y}` builds a `Node` and adds it to the end of the list.
4. **Give it an empty road list.** `adj_` must always have one list per place. Without this line, `adj_[id]` would be out of range later.
5. **Record the name-to-id link** in the hash map, so `find("Station")` works in O(1).

## 2. `add_edge`: register a road

```cpp
if (weight < 0) throw invalid_argument("negative road length");
adj_[a].push_back({b, weight});
if (two_way) adj_[b].push_back({a, weight});
double d = euclid(a, b);
if (d > 0) h_scale_ = min(h_scale_, weight / d);
```

- **Reject negative lengths.** Dijkstra gives wrong answers with them.
- **Store the road** in place a's list, as "go to b, costs weight". If the road is two-way, also store it in b's list. A one-way road gets only the first.
- **The last two lines are the A\* safety.** `d` is the straight-line distance between the road's ends. `weight / d` is how long the road is compared with the straight line. `h_scale_` keeps the smallest such ratio seen so far. Normally every road is at least as long as the straight line, so the ratio is at least 1 and `h_scale_` stays at 1.0. If some road is shorter than the straight line, the scale shrinks and A\*'s guess shrinks with it. `if (d > 0)` avoids dividing by zero when two places sit at the same spot.

## 3. `find`: name to id

```cpp
auto it = index_.find(name);
if (it == index_.end()) return -1;
return it->second;
```

- `index_.find(name)` searches the hash map. `auto` lets the compiler work out the long iterator type for you.
- `index_.end()` is a marker meaning "past the last item". If `it` equals it, the name was not found, so return -1.
- Otherwise `it->second` is the stored id. A map item is a pair, where `first` is the name and `second` is the id.

## 4. `fail`: the error helper

```cpp
static void fail(const string& path, int line_number, const string& message) {
    throw runtime_error(path + ":" + to_string(line_number) + ": " + message);
}
```

It builds messages like `data/city.txt:7: unknown flag: onway` and throws them. `static` means it can only be used inside this file. The point is that the same "file:line" formatting is not repeated seven times in `load`.

## 5. `load`: read the map file

It reads the file one line at a time. For each line:

1. **`getline(in, line)`** reads the next line into `line`. The loop ends at end of file.
2. **Strip comments.** Find `#`. If it is there, `erase(hash_pos)` deletes from the `#` to the end of the line. `npos` means "not found".
3. **Split into words.** `istringstream words(line)` lets you read words from a string like `cin`. `words >> kind` reads the first word. If there is not one, the line was blank, so `continue` skips it.
4. **A `node` line**, such as `node Station 0 0`:
    - `words >> name >> x >> y` reads three more values.
    - If reading fails, because a value is missing or `x` is not a number, the `!(...)` is true and we call `fail`.
    - Otherwise `add_node` is called.
5. **An `edge` line**, such as `edge Airport Station 11.5 oneway`:
    - Read the two names and the length.
    - Look up both names with `find`. If either gives -1, the map refers to a place that does not exist, so fail.
    - Try to read one more word. `if (words >> flag)` is true only if a word was there. It must be exactly `oneway`, and that sets `two_way = false`. Anything else is an error.
    - Call `add_edge`.
6. **Anything else** is an unknown record type, so fail.

At the end it does `return g;` with the finished graph.

## Things worth being able to say in an interview

- **Why throw exceptions instead of printing and exiting?** The caller decides what to do. `main.cpp` catches them and prints a clean message.
- **Why must `adj_` get an empty list in `add_node`?** `adj_` and `nodes_` must stay the same length. That is the invariant the whole class relies on.
- **Why does `load` call `add_node` and `add_edge` instead of touching the vectors itself?** So the validation lives in one place. A negative length is refused whether it came from a file or from a test.

## Easy mistakes when typing this file

The two bits people most often get wrong are `adj_.push_back(vector<Edge>())` in `add_node` and the `if (words >> flag)` check in `load`.
