#include "Algo.h"
#include "TestFramework.h"
#include "TestUtils.h"

namespace {

struct AUVFixture {
    test::Matrix weights;
    int coords[100][3];

    AUVFixture() {
        for (int i = 0; i < 100; ++i) coords[i][0] = coords[i][1] = coords[i][2] = 0;
        int stations[4][3] = {{0, 0, 0}, {5, 0, 50}, {5, 0, 0}, {10, 0, 0}};
        for (int i = 0; i < 4; ++i)
            for (int k = 0; k < 3; ++k) coords[i][k] = stations[i][k];
        weights.link(0, 1, 5);
        weights.link(1, 3, 5);
        weights.link(0, 2, 6);
        weights.link(2, 3, 6);
    }
};

}

TEST_CASE(auvAvoidsDepthPenalty) {
    AUVFixture* fx = new AUVFixture;
    for (int mode = 1; mode <= 3; ++mode) {
        PathNode* path = findAUVPath(fx->weights.data, fx->coords, 0, 3, mode);
        CHECK(test::names(path) == test::split("0 2 3"));
        CHECK_NEAR(path->h, 10.0);
        CHECK_NEAR(path->next->g, 6.0);
        CHECK_NEAR(path->next->h, 5.0);
        CHECK_NEAR(pathTail(path)->g, 12.0);
        freePath(path);
    }
    delete fx;
}

TEST_CASE(auvHeuristicModes) {
    AUVFixture* fx = new AUVFixture;
    double expected[3] = {55.0, std::sqrt(25.0 + 2500.0), 50.0};
    for (int mode = 1; mode <= 3; ++mode) {
        PathNode* path = findAUVPath(fx->weights.data, fx->coords, 3, 1, mode);
        CHECK(test::names(path) == test::split("3 1"));
        CHECK_NEAR(path->h, expected[mode - 1]);
        CHECK_NEAR(pathTail(path)->g, 7.5);
        freePath(path);
    }
    delete fx;
}

TEST_CASE(auvRejectsInvalidInput) {
    AUVFixture* fx = new AUVFixture;
    CHECK(findAUVPath(fx->weights.data, fx->coords, 0, 3, 0) == nullptr);
    CHECK(findAUVPath(fx->weights.data, fx->coords, 0, 3, 4) == nullptr);
    CHECK(findAUVPath(fx->weights.data, fx->coords, 0, 9, 1) == nullptr);
    delete fx;
}
