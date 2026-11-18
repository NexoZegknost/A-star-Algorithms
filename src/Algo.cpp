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

const int EVENT_EXPAND = 0;
const int EVENT_PUSH = 1;

long expandedCount = 0;
std::vector<std::vector<double> >* eventLog = nullptr;

void record(int type, int node, double g, double h) {
    if (eventLog == nullptr) return;
    std::vector<double> event(4);
    event[0] = type;
    event[1] = node;
    event[2] = g;
    event[3] = h;
    eventLog->push_back(event);
}

SearchResult search(const Graph& graph, int start, const std::vector<bool>& isGoal) {
    const int size = graph.size();
    std::vector<double> best(size, INF);
    std::vector<int> parent(size, -1);
    OpenList open;

    best[start] = 0.0;
    open.push(start, 0.0, graph.h[start]);
    record(EVENT_PUSH, start, 0.0, graph.h[start]);
    expandedCount = 0;

    SearchResult result;
    result.found = false;

    while (!open.empty()) {
        OpenEntry current = open.pop();
        int u = current.node;
        if (current.g > best[u] + EPS) continue;
        ++expandedCount;
        record(EVENT_EXPAND, u, current.g, current.h);

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
                record(EVENT_PUSH, edge.to, candidate, graph.h[edge.to]);
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
                       const std::vector<std::string>& pathLabels) {
    PathNode* head = nullptr;
    PathNode* tail = nullptr;
    for (size_t i = 0; i < result.nodes.size(); ++i) {
        int node = result.nodes[i];
        appendPathNode(head, tail, createPathNode(pathLabels[i], result.g[i], graph.h[node]));
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

std::vector<std::string> indexLabels(const SearchResult& result) {
    std::vector<std::string> labels;
    for (size_t i = 0; i < result.nodes.size(); ++i) {
        labels.push_back(std::to_string(result.nodes[i]));
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

const int OBSTACLE = 1;
const int SWAMP = 2;

struct Direction {
    int dx;
    int dy;
    const char* label;
};

const Direction DIRECTIONS[8] = {
    {-1, 0, "Up"},       {1, 0, "Down"},       {0, -1, "Left"},     {0, 1, "Right"},
    {-1, -1, "Up-Left"}, {-1, 1, "Up-Right"},  {1, -1, "Down-Left"}, {1, 1, "Down-Right"}};

struct GridShape {
    int m;
    int n;

    bool valid() const { return m > 0 && n > 0 && m <= MAX_NODES && n <= MAX_NODES; }
    bool contains(int x, int y) const { return inRange(x, 0, m) && inRange(y, 0, n); }
    int id(int x, int y) const { return x * n + y; }
    int row(int id) const { return id / n; }
    int col(int id) const { return id % n; }
    int cells() const { return m * n; }
};

bool isDiagonal(const Direction& d) {
    return d.dx != 0 && d.dy != 0;
}

std::string directionLabel(const GridShape& grid, int from, int to) {
    int dx = grid.row(to) - grid.row(from);
    int dy = grid.col(to) - grid.col(from);
    for (int i = 0; i < 8; ++i) {
        if (DIRECTIONS[i].dx == dx && DIRECTIONS[i].dy == dy) return DIRECTIONS[i].label;
    }
    return "";
}

std::vector<std::string> directionLabels(const GridShape& grid, const std::vector<int>& cells) {
    std::vector<std::string> labels;
    for (size_t i = 0; i < cells.size(); ++i) {
        labels.push_back(i == 0 ? std::string("Start") : directionLabel(grid, cells[i - 1], cells[i]));
    }
    return labels;
}

std::string coordinateLabel(const GridShape& grid, int cell) {
    return "(" + std::to_string(grid.row(cell)) + "," + std::to_string(grid.col(cell)) + ")";
}

struct Wire {
    int to;
    int direction;
};

double terrainCost(int cell) {
    return cell == SWAMP ? 2.0 : 1.0;
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
    return toLinkedList(result, graph, indexLabels(result));
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
    return toLinkedList(result, graph, indexLabels(result));
}

PathNode* findProbabilisticPath(double distMatrix[100][100], double riskMatrix[100][100],
                                int coords[100][2], int startPoint, int goalPoint,
                                double lambda, int mode) {
    using namespace astar;
    Metric metric = metricFromMode(mode, MANHATTAN, EUCLIDEAN);
    if (metric == INVALID_METRIC) return nullptr;
    if (!inRange(startPoint, 0, MAX_NODES) || !inRange(goalPoint, 0, MAX_NODES)) return nullptr;

    Graph graph(MAX_NODES);
    for (int u = 0; u < MAX_NODES; ++u) {
        for (int v = 0; v < MAX_NODES; ++v) {
            if (u == v || distMatrix[u][v] <= 0) continue;
            graph.addEdge(u, v, distMatrix[u][v] + lambda * riskMatrix[u][v]);
        }
        graph.h[u] = distance(metric, coords[u][0] - coords[goalPoint][0],
                              coords[u][1] - coords[goalPoint][1]);
    }

    SearchResult result = search(graph, startPoint, goalPoint);
    if (!result.found) return nullptr;

    PathNode* head = toLinkedList(result, graph, indexLabels(result));
    double survival = 1.0;
    PathNode* node = head;
    for (size_t i = 0; i < result.nodes.size(); ++i, node = node->next) {
        if (i > 0) survival *= 1.0 - riskMatrix[result.nodes[i - 1]][result.nodes[i]];
        node->pSafe = survival;
    }
    return head;
}

PathNode* findGamePath(int mapGrid[100][100], int m, int n, int startX, int startY,
                       int goalX, int goalY, int mode) {
    using namespace astar;
    Metric metric = metricFromMode(mode, MANHATTAN, CHEBYSHEV);
    GridShape grid = {m, n};
    if (metric == INVALID_METRIC || !grid.valid()) return nullptr;
    if (!grid.contains(startX, startY) || !grid.contains(goalX, goalY)) return nullptr;
    if (mapGrid[startX][startY] == OBSTACLE || mapGrid[goalX][goalY] == OBSTACLE) return nullptr;

    Graph graph(grid.cells());
    for (int x = 0; x < m; ++x) {
        for (int y = 0; y < n; ++y) {
            int u = grid.id(x, y);
            graph.h[u] = distance(metric, x - goalX, y - goalY);
            if (mapGrid[x][y] == OBSTACLE) continue;

            for (int i = 0; i < 8; ++i) {
                const Direction& d = DIRECTIONS[i];
                int nx = x + d.dx;
                int ny = y + d.dy;
                if (!grid.contains(nx, ny) || mapGrid[nx][ny] == OBSTACLE) continue;
                if (isDiagonal(d) && (mapGrid[nx][y] == OBSTACLE || mapGrid[x][ny] == OBSTACLE)) continue;
                double cost = terrainCost(mapGrid[nx][ny]) * (isDiagonal(d) ? 1.5 : 1.0);
                graph.addEdge(u, grid.id(nx, ny), cost);
            }
        }
    }

    SearchResult result = search(graph, grid.id(startX, startY), grid.id(goalX, goalY));
    if (!result.found) return nullptr;
    return toLinkedList(result, graph, directionLabels(grid, result.nodes));
}

PathNode* findPCBPath(int pcbGrid[100][100], int m, int n, int startX, int startY,
                      int exitX, int exitY, double weightMatrix[100][100], int mode) {
    using namespace astar;
    const double TRACE_UNIT = 1.0;
    const double BEND_PENALTY = 0.5;
    const int NO_DIRECTION = 4;
    const int STATES_PER_CELL = 5;

    for (int i = 0; i < MAX_NODES; ++i) {
        for (int j = 0; j < MAX_NODES; ++j) weightMatrix[i][j] = 0.0;
    }

    Metric metric = metricFromMode(mode, MANHATTAN, CHEBYSHEV);
    GridShape grid = {m, n};
    if (metric == INVALID_METRIC || !grid.valid()) return nullptr;
    if (!grid.contains(startX, startY) || !grid.contains(exitX, exitY)) return nullptr;

    std::vector<std::vector<Wire> > wires(grid.cells());
    for (int x = 0; x < m; ++x) {
        for (int y = 0; y < n; ++y) {
            if (pcbGrid[x][y] == OBSTACLE) continue;
            int u = grid.id(x, y);
            for (int d = 0; d < 4; ++d) {
                int nx = x + DIRECTIONS[d].dx;
                int ny = y + DIRECTIONS[d].dy;
                if (!grid.contains(nx, ny) || pcbGrid[nx][ny] == OBSTACLE) continue;
                int v = grid.id(nx, ny);
                Wire wire = {v, d};
                wires[u].push_back(wire);
                if (u < MAX_NODES && v < MAX_NODES) weightMatrix[u][v] = TRACE_UNIT;
            }
        }
    }
    if (pcbGrid[startX][startY] == OBSTACLE || pcbGrid[exitX][exitY] == OBSTACLE) return nullptr;

    Graph graph(grid.cells() * STATES_PER_CELL);
    std::vector<bool> isGoal(graph.size(), false);
    for (int u = 0; u < grid.cells(); ++u) {
        double h = distance(metric, grid.row(u) - exitX, grid.col(u) - exitY);
        for (int incoming = 0; incoming < STATES_PER_CELL; ++incoming) {
            int state = u * STATES_PER_CELL + incoming;
            graph.h[state] = h;
            isGoal[state] = u == grid.id(exitX, exitY);
            for (size_t i = 0; i < wires[u].size(); ++i) {
                const Wire& wire = wires[u][i];
                bool bends = incoming != NO_DIRECTION && incoming != wire.direction;
                double cost = TRACE_UNIT + (bends ? BEND_PENALTY : 0.0);
                graph.addEdge(state, wire.to * STATES_PER_CELL + wire.direction, cost);
            }
        }
    }

    int startState = grid.id(startX, startY) * STATES_PER_CELL + NO_DIRECTION;
    SearchResult result = search(graph, startState, isGoal);
    if (!result.found) return nullptr;

    std::vector<std::string> labels;
    for (size_t i = 0; i < result.nodes.size(); ++i) {
        labels.push_back(coordinateLabel(grid, result.nodes[i] / STATES_PER_CELL));
    }
    return toLinkedList(result, graph, labels);
}
