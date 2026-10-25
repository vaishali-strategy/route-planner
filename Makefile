CXX      ?= clang++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wpedantic -Iinclude
LIB      := src/graph.cpp src/pathfinding.cpp

all: build/planner build/test_runner build/bench

build:
	mkdir -p build

build/planner: $(LIB) src/main.cpp | build
	$(CXX) $(CXXFLAGS) $^ -o $@

build/test_runner: $(LIB) tests/test_main.cpp | build
	$(CXX) $(CXXFLAGS) $^ -o $@

build/bench: $(LIB) bench/bench.cpp | build
	$(CXX) $(CXXFLAGS) $^ -o $@

# Run from the repo root: the tests load data/city.txt by relative path.
test: build/test_runner
	./build/test_runner

# Same tests under AddressSanitizer + UBSan: catches out-of-range indexing and UB.
asan: $(LIB) tests/test_main.cpp | build
	$(CXX) -std=c++17 -O1 -g -fsanitize=address,undefined -Iinclude $^ -o build/test_asan
	./build/test_asan

bench: build/bench
	./build/bench 300

clean:
	rm -rf build

.PHONY: all test asan bench clean
