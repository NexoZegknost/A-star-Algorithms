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

enum Metric { MANHATTAN, EUCLIDEAN, CHEBYSHEV, INVALID_METRIC };

double distance(Metric metric, double dx, double dy, double dz = 0.0) {
    switch (metric) {
        case MANHATTAN: return manhattan(dx, dy, dz);
        case EUCLIDEAN: return euclidean(dx, dy, dz);
        case CHEBYSHEV: return chebyshev(dx, dy, dz);
        default: return 0.0;
    }
}

Metric metricFromMode(int mode, Metric mode1, Metric mode2, Metric mode3 = INVALID_METRIC) {
    if (mode == 1) return mode1;
    if (mode == 2) return mode2;
    if (mode == 3) return mode3;
    return INVALID_METRIC;
}

bool inRange(int value, int low, int high) {
    return value >= low && value < high;
}

const int MAX_NODES = 100;

std::vector<std::string> indexLabels(int size) {
    std::vector<std::string> labels(size);
    for (int i = 0; i < size; ++i) {
        labels[i] = std::to_string(i);
    }
    return labels;
}

std::vector<int> hopsToGoal(const Graph& graph, int goal) {
    const int size = graph.size();
    std::vector<std::vector<int> > reverse(size);
    for (int u = 0; u < size; ++u) {
        for (size_t i = 0; i < graph.adj[u].size(); ++i) {
            reverse[graph.adj[u][i].to].push_back(u);
        }
    }

    std::vector<int> hops(size, -1);
    std::vector<int> queue(1, goal);
    hops[goal] = 0;
    for (size_t head = 0; head < queue.size(); ++head) {
        int v = queue[head];
        for (size_t i = 0; i < reverse[v].size(); ++i) {
            int u = reverse[v][i];
            if (hops[u] == -1) {
                hops[u] = hops[v] + 1;
                queue.push_back(u);
            }
        }
    }
    return hops;
}

}

PathNode* findMeshPath(double weightMatrix[100][100], int startNode, int goalNode) {
    using namespace astar;
    if (!inRange(startNode, 0, MAX_NODES) || !inRange(goalNode, 0, MAX_NODES)) return nullptr;

    std::vector<int> degree(MAX_NODES, 0);
    for (int u = 0; u < MAX_NODES; ++u) {
        for (int v = 0; v < MAX_NODES; ++v) {
            if (u != v && weightMatrix[u][v] > 0) ++degree[u];
        }
    }

    Graph graph(MAX_NODES);
    double minStepCost = INF;
    for (int u = 0; u < MAX_NODES; ++u) {
        for (int v = 0; v < MAX_NODES; ++v) {
            if (u == v || weightMatrix[u][v] <= 0) continue;
            double cost = weightMatrix[u][v] + 0.1 * degree[v];
            graph.addEdge(u, v, cost);
            minStepCost = std::min(minStepCost, cost);
        }
    }

    std::vector<int> hops = hopsToGoal(graph, goalNode);
    if (hops[startNode] == -1) return nullptr;
    for (int v = 0; v < MAX_NODES; ++v) {
        graph.h[v] = hops[v] == -1 ? INF : hops[v] * minStepCost;
    }

    SearchResult result = search(graph, startNode, goalNode);
    if (!result.found) return nullptr;
    return toLinkedList(result, graph, indexLabels(MAX_NODES));
}

PathNode* findAUVPath(double weightMatrix[100][100], int coords[100][3],
                      int startPoint, int goalPoint, int mode) {
    using namespace astar;
    Metric metric = metricFromMode(mode, MANHATTAN, EUCLIDEAN, CHEBYSHEV);
    if (metric == INVALID_METRIC) return nullptr;
    if (!inRange(startPoint, 0, MAX_NODES) || !inRange(goalPoint, 0, MAX_NODES)) return nullptr;

    Graph graph(MAX_NODES);
    for (int u = 0; u < MAX_NODES; ++u) {
        for (int v = 0; v < MAX_NODES; ++v) {
            if (u == v || weightMatrix[u][v] <= 0) continue;
            double depthChange = std::fabs(static_cast<double>(coords[v][2] - coords[u][2]));
            graph.addEdge(u, v, weightMatrix[u][v] * (1.0 + depthChange / 100.0));
        }
        graph.h[u] = distance(metric, coords[u][0] - coords[goalPoint][0],
                              coords[u][1] - coords[goalPoint][1],
                              coords[u][2] - coords[goalPoint][2]);
    }

    SearchResult result = search(graph, startPoint, goalPoint);
    if (!result.found) return nullptr;
    return toLinkedList(result, graph, indexLabels(MAX_NODES));
}
