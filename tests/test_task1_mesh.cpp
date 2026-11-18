#include "Algo.h"
#include "TestFramework.h"
#include "TestUtils.h"

namespace {

test::Matrix* sampleNetwork() {
    test::Matrix* m = new test::Matrix;
    m->link(0, 1, 1);
    m->link(0, 2, 4);
    m->link(1, 2, 1);
    m->link(1, 3, 5);
    m->link(2, 3, 1);
    return m;
}

}

TEST_CASE(meshFindsCongestionAwarePath) {
    test::Matrix* m = sampleNetwork();
    PathNode* path = findMeshPath(m->data, 0, 3);
    CHECK(test::names(path) == test::split("0 1 2 3"));
    CHECK_NEAR(path->h, 2.4);
    CHECK_NEAR(path->next->g, 1.3);
    CHECK_NEAR(path->next->f, 2.5);
    CHECK_NEAR(path->next->next->g, 2.6);
    CHECK_NEAR(pathTail(path)->g, 3.8);
    CHECK_NEAR(pathTail(path)->h, 0.0);
    freePath(path);
    delete m;
}

TEST_CASE(meshStartEqualsGoal) {
    test::Matrix* m = sampleNetwork();
    PathNode* path = findMeshPath(m->data, 2, 2);
    CHECK(pathLength(path) == 1);
    CHECK_NEAR(path->f, 0.0);
    freePath(path);
    delete m;
}

TEST_CASE(meshRejectsUnreachableAndInvalid) {
    test::Matrix* m = sampleNetwork();
    CHECK(findMeshPath(m->data, 0, 7) == nullptr);
    CHECK(findMeshPath(m->data, -1, 3) == nullptr);
    CHECK(findMeshPath(m->data, 0, 100) == nullptr);
    delete m;
}

TEST_CASE(meshMatchesDijkstraOnRandomNetworks) {
    unsigned seed = 7;
    for (int trial = 0; trial < 30; ++trial) {
        test::Matrix* m = new test::Matrix;
        const int size = 25;
        for (int u = 0; u < size; ++u)
            for (int v = 0; v < size; ++v)
                if (u != v && test::nextRandom(seed) % 5 == 0)
                    m->data[u][v] = 1 + test::nextRandom(seed) % 20;

        std::vector<int> degree(100, 0);
        for (int u = 0; u < 100; ++u)
            for (int v = 0; v < 100; ++v)
                if (u != v && m->data[u][v] > 0) ++degree[u];

        std::vector<std::vector<std::pair<int, double> > > adj(100);
        for (int u = 0; u < 100; ++u)
            for (int v = 0; v < 100; ++v)
                if (u != v && m->data[u][v] > 0)
                    adj[u].push_back(std::make_pair(v, m->data[u][v] + 0.1 * degree[v]));

        double expected = test::dijkstra(adj, 0, size - 1);
        PathNode* path = findMeshPath(m->data, 0, size - 1);
        if (expected >= 1e18) {
            CHECK(path == nullptr);
        } else {
            CHECK(path != nullptr);
            if (path != nullptr) CHECK_NEAR(pathTail(path)->g, expected);
        }
        freePath(path);
        delete m;
    }
}
