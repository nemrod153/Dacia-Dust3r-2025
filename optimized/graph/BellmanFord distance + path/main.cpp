#include <bits/stdc++.h>
// Edge representation: directed edge u -> v with weight w
struct Edge {
    int from;
    int to;
    long long weight;
};

// Result of Bellman-Ford from a single source
struct BellmanFordResult {
    std::vector<long long> dist;    // dist[v] = shortest distance from source to v (or INF)
    std::vector<int> parent;        // parent[v] = previous vertex on shortest path (or -1)
    bool has_negative_cycle{};        // true if ANY negative cycle is reachable from source
};

// Bellman-Ford algorithm
// n       : number of vertices (0...n-1)
// edges   : list of directed edges
// source  : starting vertex
//
// Time complexity: O(n * edges.size())
// Space complexity: O(n)
// Focus is on *time*, so we:
//   - Use a flat edge list (cheap to iterate).
//   - Stop early if an iteration performs no relaxation.
BellmanFordResult bellman_ford(int n, const std::vector<Edge>& edges, int source) {
    constexpr long long INF = std::numeric_limits<long long>::max() / 4;

    BellmanFordResult res;
    res.dist.assign(n, INF);
    res.parent.assign(n, -1);
    res.has_negative_cycle = false;

    res.dist[source] = 0;

    // Relax edges up to (n-1) times
    for (int i = 0; i < n - 1; ++i) {
        bool any_relaxed = false;

        for (const Edge& e : edges) {
            if (res.dist[e.from] == INF) {
                // If we never reached e.from, it can't relax anything.
                continue;
            }

            long long candidate = res.dist[e.from] + e.weight;
            if (candidate < res.dist[e.to]) {
                res.dist[e.to] = candidate;
                res.parent[e.to] = e.from;
                any_relaxed = true;
            }
        }

        // Early stopping: no changes in this iteration -> the shortest paths found
        if (!any_relaxed) {
            break;
        }
    }

    // Check for negative-weight cycles reachable from the source
    for (const Edge& e : edges) {
        if (res.dist[e.from] == INF) {
            continue;
        }
        if (res.dist[e.from] + e.weight < res.dist[e.to]) {
            res.has_negative_cycle = true;
            break;
        }
    }

    return res;
}

// Reconstruct the shortest path from source to a single target vertex.
// - parent: predecessor array from Bellman-Ford
// - source: source vertex
// - target: target vertex
//
// Returns a vector of vertices [source, ..., target].
// If 'target' is unreachable, returns an empty vector.
std::vector<int> build_path_to_vertex(const std::vector<int>& parent,
                                      int source,
                                      int target) {
    std::vector<int> path;

    // If target is the source itself and parent[source] == -1, the path is just [source]
    if (target == source) {
        path.push_back(source);
        return path;
    }

    if (target < 0 || target >= static_cast<int>(parent.size())) {
        return {}; // invalid target
    }

    // If parent[target] == -1, and it's not the source, then there's no path
    if (parent[target] == -1) {
        return {};
    }

    int current = target;
    while (current != -1 && current != source) {
        path.push_back(current);
        current = parent[current];
    }

    if (current == -1) {
        // We did not reach the source -> no valid path
        return {};
    }

    // Add the source itself
    path.push_back(source);

    // Currently the path is [target, ..., source], so we reverse it
    std::ranges::reverse(path);
    return path;
}

// Build the shortest paths from the source to *every* vertex.
// - n     : number of vertices
// - source: source vertex
// - parent: predecessor array from Bellman-Ford
//
// Returns a vector of paths, where result[v] is a vector<int> representing
// the path from 'source' to 'v'. If 'v' is unreachable, result[v] is empty.
std::vector<std::vector<int>> build_all_shortest_paths(int n,
                                                       int source,
                                                       const std::vector<int>& parent) {
    std::vector<std::vector<int>> all_paths;
    all_paths.reserve(n);

    for (int v = 0; v < n; ++v) {
        all_paths.push_back(build_path_to_vertex(parent, source, v));
    }

    return all_paths;
}

#include <iostream>

int main() {
    int n = 5; // vertices 0..4
    std::vector<Edge> edges = {
        {0, 1, 6},
        {0, 2, 7},
        {1, 2, 8},
        {1, 3, 5},
        {1, 4, -4},
        {2, 3, -3},
        {2, 4, 9},
        {3, 1, -2},
        {4, 0, 2},
        {4, 3, 7}
    };

    int source = 0;
    BellmanFordResult res = bellman_ford(n, edges, source);

    if (res.has_negative_cycle) {
        std::cout << "Warning: negative-weight cycle reachable from source\n";
    }

    const auto all_paths = build_all_shortest_paths(n, source, res.parent);

    for (int v = 0; v < n; ++v) {
        std::cout << "Vertex " << v << ": dist = ";
        if (res.dist[v] == std::numeric_limits<long long>::max() / 4) {
            std::cout << "INF";
        } else {
            std::cout << res.dist[v];
        }
        std::cout << ", path =";
        for (int vertex : all_paths[v]) {
            std::cout << ' ' << vertex;
        }
        std::cout << '\n';
    }
    return 0;
}
