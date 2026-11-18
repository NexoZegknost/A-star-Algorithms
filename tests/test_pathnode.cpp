#include "PathNode.h"
#include "TestFramework.h"

TEST_CASE(createPathNodeComputesF) {
    PathNode* node = createPathNode("A", 2.5, 1.5);
    CHECK(node->name == "A");
    CHECK_NEAR(node->f, 4.0);
    CHECK(node->next == nullptr);
    CHECK_NEAR(node->pSafe, 1.0);
    freePath(node);
}

TEST_CASE(appendPathNodePreservesOrder) {
    PathNode* head = nullptr;
    PathNode* tail = nullptr;
    appendPathNode(head, tail, createPathNode("S", 0, 3));
    appendPathNode(head, tail, createPathNode("A", 1, 2));
    appendPathNode(head, tail, createPathNode("G", 4, 0));
    CHECK(pathLength(head) == 3);
    CHECK(head->name == "S");
    CHECK(pathTail(head)->name == "G");
    CHECK(head->next->name == "A");
    freePath(head);
}

TEST_CASE(emptyPathHelpers) {
    CHECK(pathLength(nullptr) == 0);
    CHECK(pathTail(nullptr) == nullptr);
    freePath(nullptr);
}
