#ifndef TEST_UTILS_H
#define TEST_UTILS_H

#include <string>
#include <utility>
#include <vector>

#include "PathNode.h"

namespace test {

struct Matrix {
    double data[100][100];
    Matrix() { clear(); }
    void clear() {
        for (int i = 0; i < 100; ++i)
            for (int j = 0; j < 100; ++j) data[i][j] = 0.0;
    }
    void link(int u, int v, double w) { data[u][v] = data[v][u] = w; }
};

struct IntGrid {
    int data[100][100];
    IntGrid() {
        for (int i = 0; i < 100; ++i)
            for (int j = 0; j < 100; ++j) data[i][j] = 0;
    }
};

inline std::vector<std::string> names(const PathNode* head) {
    std::vector<std::string> out;
    for (; head != nullptr; head = head->next) out.push_back(head->name);
    return out;
}

inline std::vector<std::string> split(const std::string& text) {
    std::vector<std::string> out;
    std::string token;
    for (size_t i = 0; i <= text.size(); ++i) {
        if (i == text.size() || text[i] == ' ') {
            if (!token.empty()) out.push_back(token);
            token.clear();
        } else {
            token += text[i];
        }
    }
    return out;
}

inline double dijkstra(const std::vector<std::vector<std::pair<int, double> > >& adj, int s, int t) {
    const double inf = 1e18;
    std::vector<double> dist(adj.size(), inf);
    std::vector<bool> done(adj.size(), false);
    dist[s] = 0.0;
    for (size_t iter = 0; iter < adj.size(); ++iter) {
        int u = -1;
        for (size_t v = 0; v < adj.size(); ++v)
            if (!done[v] && (u == -1 || dist[v] < dist[u])) u = static_cast<int>(v);
        if (u == -1 || dist[u] >= inf) break;
        done[u] = true;
        for (size_t i = 0; i < adj[u].size(); ++i) {
            int v = adj[u][i].first;
            if (dist[u] + adj[u][i].second < dist[v]) dist[v] = dist[u] + adj[u][i].second;
        }
    }
    return dist[t];
}

inline unsigned nextRandom(unsigned& state) {
    state = state * 1103515245u + 12345u;
    return (state >> 16) & 0x7fff;
}

}

#endif
