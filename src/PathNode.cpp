#include "PathNode.h"

PathNode* createPathNode(const std::string& name, double g, double h) {
    PathNode* node = new PathNode;
    node->name = name;
    node->g = g;
    node->h = h;
    node->f = g + h;
    node->next = nullptr;
    node->pSafe = 1.0;
    return node;
}

void appendPathNode(PathNode*& head, PathNode*& tail, PathNode* node) {
    if (head == nullptr) {
        head = tail = node;
        return;
    }
    tail->next = node;
    tail = node;
}

int pathLength(const PathNode* head) {
    int length = 0;
    for (; head != nullptr; head = head->next) {
        ++length;
    }
    return length;
}

const PathNode* pathTail(const PathNode* head) {
    if (head == nullptr) {
        return nullptr;
    }
    while (head->next != nullptr) {
        head = head->next;
    }
    return head;
}

void freePath(PathNode* head) {
    while (head != nullptr) {
        PathNode* next = head->next;
        delete head;
        head = next;
    }
}
