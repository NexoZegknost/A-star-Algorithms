#include "Algo.h"
#include "TestFramework.h"
#include "TestUtils.h"

namespace {

void fill(test::IntGrid& grid, const char* rows[], int m) {
    for (int x = 0; x < m; ++x)
        for (int y = 0; rows[x][y] != '\0'; ++y) grid.data[x][y] = rows[x][y] - '0';
}

double referenceCost(const test::IntGrid& grid, int m, int n, int sx, int sy, int gx, int gy) {
    const int dx[8] = {-1, 1, 0, 0, -1, -1, 1, 1};
    const int dy[8] = {0, 0, -1, 1, -1, 1, -1, 1};
    std::vector<std::vector<std::pair<int, double> > > adj(m * n);
    for (int x = 0; x < m; ++x)
        for (int y = 0; y < n; ++y) {
            if (grid.data[x][y] == 1) continue;
            for (int k = 0; k < 8; ++k) {
                int nx = x + dx[k], ny = y + dy[k];
                if (nx < 0 || ny < 0 || nx >= m || ny >= n || grid.data[nx][ny] == 1) continue;
                bool diagonal = dx[k] != 0 && dy[k] != 0;
                if (diagonal && (grid.data[nx][y] == 1 || grid.data[x][ny] == 1)) continue;
                double base = grid.data[nx][ny] == 2 ? 2.0 : 1.0;
                adj[x * n + y].push_back(std::make_pair(nx * n + ny, base * (diagonal ? 1.5 : 1.0)));
            }
        }
    return test::dijkstra(adj, sx * n + sy, gx * n + gy);
}

}

TEST_CASE(gameMovesDiagonallyOnOpenGround) {
    test::IntGrid* grid = new test::IntGrid;
    PathNode* path = findGamePath(grid->data, 3, 3, 0, 0, 2, 2, 2);
    CHECK(test::names(path) == test::split("Start Down-Right Down-Right"));
    CHECK_NEAR(path->h, 2.0);
    CHECK_NEAR(pathTail(path)->g, 3.0);
    freePath(path);
    delete grid;
}

TEST_CASE(gameBlocksCornerCutting) {
    test::IntGrid* grid = new test::IntGrid;
    const char* rows[] = {"000", "010", "000"};
    fill(*grid, rows, 3);
    PathNode* path = findGamePath(grid->data, 3, 3, 0, 0, 2, 2, 2);
    CHECK(pathLength(path) == 5);
    CHECK_NEAR(pathTail(path)->g, 4.0);
    for (const PathNode* node = path; node != nullptr; node = node->next)
        CHECK(node->name.find('-') == std::string::npos);
    freePath(path);
    delete grid;
}

TEST_CASE(gameChargesSwampTerrain) {
    test::IntGrid* grid = new test::IntGrid;
    const char* rows[] = {"020"};
    fill(*grid, rows, 1);
    PathNode* path = findGamePath(grid->data, 1, 3, 0, 0, 0, 2, 1);
    CHECK(test::names(path) == test::split("Start Right Right"));
    CHECK_NEAR(path->next->g, 2.0);
    CHECK_NEAR(pathTail(path)->g, 3.0);
    freePath(path);
    delete grid;
}

TEST_CASE(gameRejectsInvalidInput) {
    test::IntGrid* grid = new test::IntGrid;
    const char* rows[] = {"010", "010", "010"};
    fill(*grid, rows, 3);
    CHECK(findGamePath(grid->data, 3, 3, 0, 0, 0, 2, 1) == nullptr);
    CHECK(findGamePath(grid->data, 3, 3, 0, 1, 0, 2, 1) == nullptr);
    CHECK(findGamePath(grid->data, 3, 3, 0, 0, 5, 5, 1) == nullptr);
    CHECK(findGamePath(grid->data, 3, 3, 0, 0, 2, 0, 3) == nullptr);
    delete grid;
}

TEST_CASE(gameChebyshevMatchesDijkstraOnRandomMaps) {
    unsigned seed = 11;
    for (int trial = 0; trial < 40; ++trial) {
        test::IntGrid* grid = new test::IntGrid;
        const int m = 12, n = 15;
        for (int x = 0; x < m; ++x)
            for (int y = 0; y < n; ++y) {
                unsigned r = test::nextRandom(seed) % 10;
                grid->data[x][y] = r < 2 ? 1 : (r < 4 ? 2 : 0);
            }
        grid->data[0][0] = 0;
        grid->data[m - 1][n - 1] = 0;
        double expected = referenceCost(*grid, m, n, 0, 0, m - 1, n - 1);
        PathNode* path = findGamePath(grid->data, m, n, 0, 0, m - 1, n - 1, 2);
        if (expected >= 1e18) {
            CHECK(path == nullptr);
        } else {
            CHECK(path != nullptr);
            if (path != nullptr) CHECK_NEAR(pathTail(path)->g, expected);
        }
        freePath(path);
        delete grid;
    }
}
