#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>

// Type aliases for convenience
using AdjList = std::vector<std::vector<std::pair<int, long long>>>;
// adj[u] contains pairs (v, w) meaning an edge u -> v with weight w

// Dijkstra's algorithm: computes shortest distances and parents
void dijkstra(int n,
              int source,
              const AdjList &adj,
              std::vector<long long> &dist,
              std::vector<int> &parent)
{
    const long long INF = std::numeric_limits<long long>::max() / 4;

    dist.assign(n, INF);
    parent.assign(n, -1);

    dist[source] = 0;

    // Min-heap: (distance, vertex)
    using Node = std::pair<long long, int>;
    std::priority_queue<Node, std::vector<Node>, std::greater<>> pq;

    pq.push({0, source});

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();

        // If this is an outdated entry, skip it
        if (d != dist[u]) {
            continue;
        }

        // Relax edges from u
        for (const auto &[v, w] : adj[u]) {
            if (dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
                parent[v] = u;
                pq.push({dist[v], v});
            }
        }
    }
}

// Build all shortest paths from the source using the parent array.
// Returns a vector of paths, where paths[v] is the path from source to v.
// If v is unreachable, paths[v] will be an empty vector.
std::vector<std::vector<int>> build_all_paths(int source,
                                              const std::vector<int> &parent)
{
    int n = static_cast<int>(parent.size());
    std::vector<std::vector<int>> paths(n);

    for (int v = 0; v < n; ++v) {
        if (v == source) {
            paths[v] = {source};
            continue;
        }

        // Reconstruct by walking backwards from v to source using parent[]
        std::vector<int> path;
        int current = v;

        // If parent[current] == -1 and current != source, it may be unreachable.
        // We detect reachability by seeing whether we eventually get to source.
        while (current != -1) {
            path.push_back(current);
            if (current == source) {
                break;
            }
            current = parent[current];
        }

        if (!path.empty() && path.back() == source) {
            // We reconstructed source -> ... -> v in reverse; fix the order
            std::reverse(path.begin(), path.end());
            paths[v] = std::move(path);
        } else {
            // Unreachable: leave paths[v] empty
            paths[v].clear();
        }
    }

    return paths;
}

int main()
{
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);

    int n, m;
    std::cin >> n >> m;

    AdjList adj(n);

    // Read m directed edges (u, v, w)
    for (int i = 0; i < m; ++i) {
        int u, v;
        long long w;
        std::cin >> u >> v >> w;
        adj[u].push_back({v, w});
    }

    int source;
    std::cin >> source;

    std::vector<long long> dist;
    std::vector<int> parent;

    dijkstra(n, source, adj, dist, parent);
    auto paths = build_all_paths(source, parent);

    const long long INF = std::numeric_limits<long long>::max() / 4;

    // Output distances
    for (int v = 0; v < n; ++v) {
        if (dist[v] >= INF) {
            std::cout << "INF\n";
        } else {
            std::cout << "Distance from " << source << " to " << v << " is: " << dist[v] << "\n";
        }
    }

    // Output shortest paths
    for (int v = 0; v < n; ++v) {
        if (paths[v].empty()) {
            std::cout << "Unreachable\n";
        } else {
            std::cout << "Shortest road from " << source << " to " << v << " is: ";
            for (std::size_t i = 0; i < paths[v].size(); ++i) {
                std::cout << paths[v][i] << (i + 1 < paths[v].size() ? ' ' : '\n');
            }
        }
    }

    return 0;
}


/*
Example input (copy-paste when running):

5 6
0 1 2
0 2 5
1 2 1
1 3 2
2 3 3
3 4 1
0

Meaning:
- 5 vertices (0..4), 6 edges
- Edges:
  0 -> 1 (weight 2)
  0 -> 2 (weight 5)
  1 -> 2 (weight 1)
  1 -> 3 (weight 2)
  2 -> 3 (weight 3)
  3 -> 4 (weight 1)
- Source vertex: 0
*/
