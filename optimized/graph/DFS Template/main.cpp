#include <iostream>
#include <vector>

// ------------------------------------------------------------
// Depth-First Search (DFS) – recursive version
// ------------------------------------------------------------
void DFS_Util(const std::vector<std::vector<int>>& graph,
              std::vector<bool>& visited,
              int u)
{
    visited[u] = true;

    // --------------------------------------------------------
    // INSERT VERTEX PROCESSING CODE HERE
    std::cout << "Visited: " << u << "\n";
    // --------------------------------------------------------

    for (int v : graph[u]) {
        if (!visited[v]) {
            DFS_Util(graph, visited, v);
        }
    }
}

void DFS(const std::vector<std::vector<int>>& graph, int start)
{
    int n = static_cast<int>(graph.size());
    std::vector<bool> visited(n, false);
    DFS_Util(graph, visited, start);
}

int main()
{
    // Same example graph as before
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

    std::cout << "DFS starting from vertex " << start_vertex << ":\n";
    DFS(graph, start_vertex);

    return 0;
}
