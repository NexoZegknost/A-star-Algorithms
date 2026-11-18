#ifndef PATHNODE_H
#define PATHNODE_H

#include <string>

struct PathNode {
    std::string name;
    double f;
    double g;
    double h;
    PathNode* next;
    double pSafe;
};

PathNode* createPathNode(const std::string& name, double g, double h);
void appendPathNode(PathNode*& head, PathNode*& tail, PathNode* node);
int pathLength(const PathNode* head);
const PathNode* pathTail(const PathNode* head);
void freePath(PathNode* head);

#endif
