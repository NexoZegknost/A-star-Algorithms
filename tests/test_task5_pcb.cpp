#include "Algo.h"
#include "TestFramework.h"
#include "TestUtils.h"

namespace {

int bends(const std::vector<std::string>& cells) {
    int count = 0;
    for (size_t i = 2; i < cells.size(); ++i) {
        int ax = cells[i - 1][1] - cells[i - 2][1], ay = cells[i - 1][3] - cells[i - 2][3];
        int bx = cells[i][1] - cells[i - 1][1], by = cells[i][3] - cells[i - 1][3];
        if (ax != bx || ay != by) ++count;
    }
    return count;
}

double referenceCost(const test::IntGrid& grid, int m, int n, int sx, int sy, int ex, int ey) {
    const int dx[4] = {-1, 1, 0, 0};
    const int dy[4] = {0, 0, -1, 1};
    std::vector<std::vector<std::pair<int, double> > > adj(m * n * 5 + 1);
    const int sink = m * n * 5;
    for (int x = 0; x < m; ++x)
        for (int y = 0; y < n; ++y) {
            if (grid.data[x][y] == 1) continue;
            for (int in = 0; in < 5; ++in) {
                int state = (x * n + y) * 5 + in;
                if (x == ex && y == ey) adj[state].push_back(std::make_pair(sink, 0.0));
                for (int k = 0; k < 4; ++k) {
                    int nx = x + dx[k], ny = y + dy[k];
                    if (nx < 0 || ny < 0 || nx >= m || ny >= n || grid.data[nx][ny] == 1) continue;
                    double cost = 1.0 + (in != 4 && in != k ? 0.5 : 0.0);
                    adj[state].push_back(std::make_pair((nx * n + ny) * 5 + k, cost));
                }
            }
        }
    return test::dijkstra(adj, (sx * n + sy) * 5 + 4, sink);
}

}

TEST_CASE(pcbStraightTraceHasNoBend) {
    test::IntGrid* grid = new test::IntGrid;
    test::Matrix* weights = new test::Matrix;
    PathNode* path = findPCBPath(grid->data, 1, 4, 0, 0, 0, 3, weights->data, 1);
    CHECK(test::names(path) == test::split("(0,0) (0,1) (0,2) (0,3)"));
    CHECK_NEAR(path->h, 3.0);
    CHECK_NEAR(pathTail(path)->g, 3.0);
    freePath(path);
    delete grid;
    delete weights;
}

TEST_CASE(pcbMinimisesBends) {
    test::IntGrid* grid = new test::IntGrid;
    test::Matrix* weights = new test::Matrix;
    for (int mode = 1; mode <= 2; ++mode) {
        PathNode* path = findPCBPath(grid->data, 4, 4, 0, 0, 3, 3, weights->data, mode);
        CHECK(pathLength(path) == 7);
        CHECK(bends(test::names(path)) == 1);
        CHECK_NEAR(pathTail(path)->g, 6.5);
        freePath(path);
    }
    delete grid;
    delete weights;
}

TEST_CASE(pcbPopulatesWeightMatrix) {
    test::IntGrid* grid = new test::IntGrid;
    test::Matrix* weights = new test::Matrix;
    weights->data[50][50] = 9.0;
    grid->data[1][1] = 1;
    PathNode* path = findPCBPath(grid->data, 3, 3, 0, 0, 2, 2, weights->data, 1);
    CHECK_NEAR(weights->data[0][1], 1.0);
    CHECK_NEAR(weights->data[1][0], 1.0);
    CHECK_NEAR(weights->data[0][3], 1.0);
    CHECK_NEAR(weights->data[1][4], 0.0);
    CHECK_NEAR(weights->data[4][1], 0.0);
    CHECK_NEAR(weights->data[0][4], 0.0);
    CHECK_NEAR(weights->data[50][50], 0.0);
    CHECK_NEAR(pathTail(path)->g, 4.5);
    freePath(path);
    delete grid;
    delete weights;
}

TEST_CASE(pcbRejectsInvalidInput) {
    test::IntGrid* grid = new test::IntGrid;
    test::Matrix* weights = new test::Matrix;
    grid->data[0][1] = 1;
    grid->data[1][0] = 1;
    CHECK(findPCBPath(grid->data, 3, 3, 0, 0, 2, 2, weights->data, 1) == nullptr);
    CHECK(findPCBPath(grid->data, 3, 3, 0, 1, 2, 2, weights->data, 1) == nullptr);
    CHECK(findPCBPath(grid->data, 3, 3, 1, 1, 2, 2, weights->data, 0) == nullptr);
    CHECK(findPCBPath(grid->data, 3, 3, 1, 1, 3, 3, weights->data, 1) == nullptr);
    delete grid;
    delete weights;
}

TEST_CASE(pcbMatchesDijkstraOnRandomBoards) {
    unsigned seed = 23;
    for (int trial = 0; trial < 30; ++trial) {
        test::IntGrid* grid = new test::IntGrid;
        test::Matrix* weights = new test::Matrix;
        const int m = 9, n = 9;
        for (int x = 0; x < m; ++x)
            for (int y = 0; y < n; ++y) grid->data[x][y] = test::nextRandom(seed) % 4 == 0 ? 1 : 0;
        grid->data[0][0] = 0;
        grid->data[m - 1][n - 1] = 0;
        double expected = referenceCost(*grid, m, n, 0, 0, m - 1, n - 1);
        PathNode* path = findPCBPath(grid->data, m, n, 0, 0, m - 1, n - 1, weights->data, 1);
        if (expected >= 1e18) {
            CHECK(path == nullptr);
        } else {
            CHECK(path != nullptr);
            if (path != nullptr) CHECK_NEAR(pathTail(path)->g, expected);
        }
        freePath(path);
        delete grid;
        delete weights;
    }
}
