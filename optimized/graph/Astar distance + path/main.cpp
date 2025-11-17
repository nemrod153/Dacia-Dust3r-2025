#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <utility>
#include <algorithm>
#include <cmath>

// ------------------- Data structures -------------------

struct Edge {
    int to;
    double weight;
};

struct AStarResult {
    double distance;
    std::vector<int> path;
    bool found;
};

// ------------------- A* implementation -------------------

// graph: adjacency list, graph[u] = list of edges (u -> v, weight)
// heuristic: heuristic[v] = estimated distance from v to goal
// start, goal: vertex indices in [0, n)
AStarResult astar(int start,
                  int goal,
                  const std::vector<std::vector<Edge>>& graph,
                  const std::vector<double>& heuristic)
{
    const int n = static_cast<int>(graph.size());
    const double INF = std::numeric_limits<double>::infinity();

    std::vector<double> g(n, INF);         // g[v] = best known distance from start to v
    std::vector<int> parent(n, -1);        // for path reconstruction
    std::vector<bool> closed(n, false);    // "closed set" = already processed

    // priority queue of (f, vertex), where f = g[v] + h[v]
    using State = std::pair<double, int>;
    std::priority_queue<State, std::vector<State>, std::greater<State>> pq;

    g[start] = 0.0;
    pq.emplace(heuristic[start], start);

    while (!pq.empty()) {
        auto [f, u] = pq.top();
        pq.pop();

        // If this state is outdated, skip it
        if (f > g[u] + heuristic[u]) {
            continue;
        }

        // Once we pop the goal from the queue, we have the optimal path
        if (u == goal) {
            break;
        }

        if (closed[u]) {
            continue;
        }
        closed[u] = true;

        for (const Edge& e : graph[u]) {
            int v = e.to;
            double w = e.weight;
            if (closed[v]) {
                continue;
            }

            double tentative_g = g[u] + w;
            if (tentative_g < g[v]) {
                g[v] = tentative_g;
                parent[v] = u;
                double f_new = g[v] + heuristic[v];
                pq.emplace(f_new, v);
            }
        }
    }

    AStarResult result;
    const double dist = g[goal];

    if (dist == INF) {
        result.distance = INF;
        result.found = false;
        return result; // result.path stays empty
    }

    // Reconstruct path from goal back to start
    std::vector<int> path;
    for (int cur = goal; cur != -1; cur = parent[cur]) {
        path.push_back(cur);
    }
    std::reverse(path.begin(), path.end());

    result.distance = dist;
    result.path = std::move(path);
    result.found = true;
    return result;
}

// ------------------- Example usage in main -------------------

int main() {
    // Example graph:
    //
    // We model a small graph where each node has 2D coordinates,
    // and the cost of each edge is the Euclidean distance between nodes.
    //
    //       (0) (0, 0)
    //      /   \
    //   1 /     \ 2
    //    /       \
    // (1) (1, 0)  (2) (2, 0)
    //    \       /
    //   1 \     / 2
    //      \   /
    //       (3) (1, -1)
    //
    // plus a long detour:
    // (2) --5-- (4) --5-- (5) --5-- (3)
    //
    // We'll run A* from start = 0 to goal = 3.

    // Coordinates for heuristic (x, y) of each node
    std::vector<std::pair<double, double>> coords = {
        {0.0,  0.0},  // 0
        {1.0,  0.0},  // 1
        {2.0,  0.0},  // 2
        {1.0, -1.0},  // 3
        {3.0,  1.0},  // 4
        {4.0,  0.0}   // 5
    };

    const int n = static_cast<int>(coords.size());
    std::vector<std::vector<Edge>> graph(n);

    auto euclidean = [&](int a, int b) {
        double dx = coords[a].first  - coords[b].first;
        double dy = coords[a].second - coords[b].second;
        return std::sqrt(dx * dx + dy * dy);
    };

    // Helper to add undirected edge with Euclidean weight
    auto add_edge_undirected = [&](int u, int v) {
        double w = euclidean(u, v);
        graph[u].push_back({v, w});
        graph[v].push_back({u, w});
    };

    // Main "short" connections
    add_edge_undirected(0, 1);
    add_edge_undirected(0, 2);
    add_edge_undirected(1, 3);
    add_edge_undirected(2, 3);

    // Long detour
    add_edge_undirected(2, 4);
    add_edge_undirected(4, 5);
    add_edge_undirected(5, 3);

    // Build heuristic: h[v] = straight-line distance from v to goal
    int start = 0;
    int goal  = 3;

    std::vector<double> heuristic(n);
    for (int v = 0; v < n; ++v) {
        heuristic[v] = euclidean(v, goal);
    }

    AStarResult res = astar(start, goal, graph, heuristic);

    if (!res.found) {
        std::cout << "No path found from " << start << " to " << goal << '\n';
        return 0;
    }

    std::cout << "Shortest distance from " << start << " to " << goal << " = "
              << res.distance << '\n';

    std::cout << "Path: ";
    for (std::size_t i = 0; i < res.path.size(); ++i) {
        std::cout << res.path[i];
        if (i + 1 < res.path.size()) {
            std::cout << " -> ";
        }
    }
    std::cout << '\n';

    return 0;
}
