#include <iostream>
#include <vector>
#include <queue>

// ------------------------------------------------------------
// Breadth-First Search (BFS)
// ------------------------------------------------------------
// Performs BFS starting from a given source vertex.
// The graph is represented as an adjacency list.
// ------------------------------------------------------------
void BFS(const std::vector<std::vector<int>>& graph, int start)
{
    int n = static_cast<int>(graph.size());
    std::vector<bool> visited(n, false);
    std::queue<int> q;

    // Start BFS from the source vertex
    visited[start] = true;
    q.push(start);

    while (!q.empty()) {
        int u = q.front();
        q.pop();

        // ----------------------------------------------------
        // INSERT VERTEX PROCESSING CODE HERE
         std::cout << "Visited: " << u << "\n";
        // ----------------------------------------------------
        for (int v : graph[u]) {
            if (!visited[v]) {
                visited[v] = true;
                q.push(v);
            }
        }
    }
}

int main()
{
    // Example graph represented as an adjacency list
    // 0 -- 1 -- 2
    // |         |
    // 4 -- 3 ---+
    std::vector<std::vector<int>> graph = {
        {1, 4},    // Node 0
        {0, 2},    // Node 1
        {1, 3},    // Node 2
        {2, 4},    // Node 3
        {0, 3}     // Node 4
    };

    int start_vertex = 0;

    std::cout << "BFS starting from vertex " << start_vertex << ":\n";
    BFS(graph, start_vertex);

    return 0;
}
