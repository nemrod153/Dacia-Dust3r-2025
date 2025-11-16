#include <bits/stdc++.h>

// A coordinate (i, j).
using Coord = std::pair<int, int>;

// Directions: up, right, down, left
constexpr int DI[4] = {-1, 0, 1, 0};
constexpr int DJ[4] = {0, 1, 0, -1};

/* ---------------------------------------------------------
   LEE RESULT STRUCT
   --------------------------------------------------------- */
struct LeeResult {
    std::vector<std::vector<int>> dist;            // distance grid
    std::vector<std::vector<Coord>> parent;        // parent grid
};

/* ---------------------------------------------------------
   LEE BFS ALGORITHM
   Does NOT build the path. Only computes dist + parent.
   --------------------------------------------------------- */

LeeResult lee_bfs(
    const std::vector<std::vector<int>>& grid,
    int si, int sj,
    int gi = -1, int gj = -1,
    bool stop_at_goal = true
) {
    const int n = static_cast<int>(grid.size());
    const int m = n > 0 ? static_cast<int>(grid[0].size()) : 0;

    LeeResult result;
    result.dist.assign(n, std::vector<int>(m, -1));
    result.parent.assign(n, std::vector<Coord>(m, Coord{-1, -1}));

    auto in_bounds = [&](int i, int j) -> bool {
        return (i >= 0 && i < n && j >= 0 && j < m);
    };

    if (!in_bounds(si, sj) || grid[si][sj] == 1) {
        return result;
    }

    std::queue<Coord> q;
    result.dist[si][sj] = 0;
    result.parent[si][sj] = Coord{-1, -1};
    q.emplace(si, sj);

    const bool has_goal = in_bounds(gi, gj);

    while (!q.empty()) {
        Coord current = q.front();
        q.pop();

        int i = current.first;
        int j = current.second;

        if (has_goal && stop_at_goal && i == gi && j == gj) {
            break;
        }

        for (int dir = 0; dir < 4; ++dir) {
            int ni = i + DI[dir];
            int nj = j + DJ[dir];

            if (!in_bounds(ni, nj)) continue;
            if (grid[ni][nj] == 1) continue;
            if (result.dist[ni][nj] != -1) continue;

            result.dist[ni][nj] = result.dist[i][j] + 1;
            result.parent[ni][nj] = Coord{i, j};
            q.emplace(ni, nj);
        }
    }

    return result;
}

/* ---------------------------------------------------------
   PATH RECONSTRUCTION (ROAD ALGORITHM)
   Uses parent grid from lee_bfs.
   --------------------------------------------------------- */
std::vector<Coord> build_path(
    const std::vector<std::vector<Coord>>& parent,
    int si, int sj,
    int gi, int gj
) {
    std::vector<Coord> path;

    const int n = static_cast<int>(parent.size());
    const int m = n > 0 ? static_cast<int>(parent[0].size()) : 0;

    auto in_bounds = [&](int i, int j) -> bool {
        return (i >= 0 && i < n && j >= 0 && j < m);
    };

    if (!in_bounds(gi, gj) || !in_bounds(si, sj)) {
        return path;
    }

    if (gi != si || gj != sj) {
        Coord p = parent[gi][gj];
        if (p.first == -1 && p.second == -1) {
            return path;
        }
    }

    int ci = gi;
    int cj = gj;

    while (ci != -1 && cj != -1) {
        path.emplace_back(ci, cj);
        Coord p = parent[ci][cj];
        ci = p.first;
        cj = p.second;
    }

    std::reverse(path.begin(), path.end());
    return path;
}

/* ---------------------------------------------------------
   EXAMPLE MAIN for demonstration
   (remove or adjust for ICPC usage)
   --------------------------------------------------------- */
int main() {
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);

    //example matrix
    // 0 = free, 1 = blocked
    std::vector<std::vector<int>> grid = {
        {0, 0, 0, 0, 1, 0, 0},
        {1, 1, 0, 0, 1, 0, 1},
        {0, 0, 0, 0, 0, 0, 0},
        {0, 1, 1, 0, 0, 1, 0},
        {0, 0, 0, 0, 0, 0, 0}
    };

    int si = 0, sj = 0; // example start
    int gi = 4, gj = 6; // example goal

    LeeResult res = lee_bfs(grid, si, sj, gi, gj, true);

    int distance = res.dist[gi][gj];

    if (distance == -1) {
        std::cout << "No path exists.\n";
        return 0;
    }

    std::cout << "Shortest distance: " << distance << "\n";

    std::vector<Coord> path = build_path(res.parent, si, sj, gi, gj);

    std::cout << "Path (i, j):\n";
    for (const Coord& c : path) {
        std::cout << "(" << c.first << ", " << c.second << ")\n";
    }

    return 0;
}
