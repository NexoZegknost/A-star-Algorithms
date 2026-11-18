#include "Algo.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace astar {

const double INF = 1e18;
const double EPS = 1e-9;

struct Edge {
    int to;
    double cost;
};

struct Graph {
    std::vector<std::vector<Edge> > adj;
    std::vector<double> h;

    explicit Graph(int size) : adj(size), h(size, 0.0) {}

    int size() const { return static_cast<int>(adj.size()); }

    void addEdge(int from, int to, double cost) {
        Edge edge = {to, cost};
        adj[from].push_back(edge);
    }
};

struct OpenEntry {
    double f;
    double g;
    double h;
    long order;
    int node;
};

struct OpenEntryGreater {
    bool operator()(const OpenEntry& a, const OpenEntry& b) const {
        if (std::fabs(a.f - b.f) > EPS) return a.f > b.f;
        if (std::fabs(a.h - b.h) > EPS) return a.h > b.h;
        return a.order > b.order;
    }
};

class OpenList {
public:
    bool empty() const { return heap_.empty(); }

    void push(int node, double g, double h) {
        OpenEntry entry = {g + h, g, h, counter_++, node};
        heap_.push_back(entry);
        std::push_heap(heap_.begin(), heap_.end(), OpenEntryGreater());
    }

    OpenEntry pop() {
        std::pop_heap(heap_.begin(), heap_.end(), OpenEntryGreater());
        OpenEntry top = heap_.back();
        heap_.pop_back();
        return top;
    }

private:
    std::vector<OpenEntry> heap_;
    long counter_ = 0;
};

struct SearchResult {
    bool found;
    std::vector<int> nodes;
    std::vector<double> g;
};

SearchResult search(const Graph& graph, int start, const std::vector<bool>& isGoal) {
    const int size = graph.size();
    std::vector<double> best(size, INF);
    std::vector<int> parent(size, -1);
    OpenList open;

    best[start] = 0.0;
    open.push(start, 0.0, graph.h[start]);

    SearchResult result;
    result.found = false;

    while (!open.empty()) {
        OpenEntry current = open.pop();
        int u = current.node;
        if (current.g > best[u] + EPS) continue;

        if (isGoal[u]) {
            for (int v = u; v != -1; v = parent[v]) {
                result.nodes.push_back(v);
                result.g.push_back(best[v]);
            }
            std::reverse(result.nodes.begin(), result.nodes.end());
            std::reverse(result.g.begin(), result.g.end());
            result.found = true;
            return result;
        }

        for (size_t i = 0; i < graph.adj[u].size(); ++i) {
            const Edge& edge = graph.adj[u][i];
            double candidate = best[u] + edge.cost;
            if (candidate < best[edge.to] - EPS) {
                best[edge.to] = candidate;
                parent[edge.to] = u;
                open.push(edge.to, candidate, graph.h[edge.to]);
            }
        }
    }
    return result;
}

SearchResult search(const Graph& graph, int start, int goal) {
    std::vector<bool> isGoal(graph.size(), false);
    isGoal[goal] = true;
    return search(graph, start, isGoal);
}

PathNode* toLinkedList(const SearchResult& result, const Graph& graph,
                       const std::vector<std::string>& labels) {
    PathNode* head = nullptr;
    PathNode* tail = nullptr;
    for (size_t i = 0; i < result.nodes.size(); ++i) {
        int node = result.nodes[i];
        appendPathNode(head, tail, createPathNode(labels[node], result.g[i], graph.h[node]));
    }
    return head;
}

double manhattan(double dx, double dy, double dz = 0.0) {
    return std::fabs(dx) + std::fabs(dy) + std::fabs(dz);
}

double euclidean(double dx, double dy, double dz = 0.0) {
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double chebyshev(double dx, double dy, double dz = 0.0) {
    return std::max(std::fabs(dx), std::max(std::fabs(dy), std::fabs(dz)));
}

bool inRange(int value, int low, int high) {
    return value >= low && value < high;
}

}
