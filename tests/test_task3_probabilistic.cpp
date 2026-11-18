#include "Algo.h"
#include "TestFramework.h"
#include "TestUtils.h"

namespace {

struct HazardFixture {
    test::Matrix dist;
    test::Matrix risk;
    int coords[100][2];

    HazardFixture() {
        for (int i = 0; i < 100; ++i) coords[i][0] = coords[i][1] = 0;
        int points[4][2] = {{0, 0}, {5, 0}, {5, 4}, {10, 0}};
        for (int i = 0; i < 4; ++i) {
            coords[i][0] = points[i][0];
            coords[i][1] = points[i][1];
        }
        dist.link(0, 1, 5);
        dist.link(1, 3, 5);
        dist.link(0, 2, 7);
        dist.link(2, 3, 7);
        risk.link(0, 1, 0.5);
        risk.link(1, 3, 0.5);
    }
};

}

TEST_CASE(probabilisticPrefersShortRouteWithoutPenalty) {
    HazardFixture* fx = new HazardFixture;
    PathNode* path = findProbabilisticPath(fx->dist.data, fx->risk.data, fx->coords, 0, 3, 0.0, 2);
    CHECK(test::names(path) == test::split("0 1 3"));
    CHECK_NEAR(pathTail(path)->g, 10.0);
    CHECK_NEAR(path->pSafe, 1.0);
    CHECK_NEAR(path->next->pSafe, 0.5);
    CHECK_NEAR(pathTail(path)->pSafe, 0.25);
    freePath(path);
    delete fx;
}

TEST_CASE(probabilisticPrefersSafeRouteWithLargePenalty) {
    HazardFixture* fx = new HazardFixture;
    for (int mode = 1; mode <= 2; ++mode) {
        PathNode* path = findProbabilisticPath(fx->dist.data, fx->risk.data, fx->coords, 0, 3, 10.0, mode);
        CHECK(test::names(path) == test::split("0 2 3"));
        CHECK_NEAR(path->h, 10.0);
        CHECK_NEAR(pathTail(path)->g, 14.0);
        CHECK_NEAR(pathTail(path)->pSafe, 1.0);
        freePath(path);
    }
    delete fx;
}

TEST_CASE(probabilisticHeuristicModes) {
    HazardFixture* fx = new HazardFixture;
    PathNode* manhattan = findProbabilisticPath(fx->dist.data, fx->risk.data, fx->coords, 2, 0, 1.0, 1);
    PathNode* euclidean = findProbabilisticPath(fx->dist.data, fx->risk.data, fx->coords, 2, 0, 1.0, 2);
    CHECK_NEAR(manhattan->h, 9.0);
    CHECK_NEAR(euclidean->h, std::sqrt(41.0));
    freePath(manhattan);
    freePath(euclidean);
    CHECK(findProbabilisticPath(fx->dist.data, fx->risk.data, fx->coords, 0, 3, 1.0, 3) == nullptr);
    delete fx;
}
