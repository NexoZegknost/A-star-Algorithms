#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include "Algo.h"

namespace astar {
extern long expandedCount;
extern std::vector<std::vector<double> >* eventLog;
}

namespace {

double W[100][100];
double R[100][100];
int C3[100][3];
int C2[100][2];
int G[100][100];

void clearAll() {
    for (int i = 0; i < 100; ++i) {
        for (int j = 0; j < 100; ++j) W[i][j] = R[i][j] = 0, G[i][j] = 0;
        C3[i][0] = C3[i][1] = C3[i][2] = C2[i][0] = C2[i][1] = 0;
    }
}

void link(double m[100][100], int u, int v, double w) { m[u][v] = m[v][u] = w; }

unsigned seed = 2026;
double uniform() {
    seed = seed * 1103515245u + 12345u;
    return ((seed >> 8) & 0xffffff) / double(0x1000000);
}

std::string pathString(const PathNode* p) {
    std::string out;
    for (; p; p = p->next) out += (out.empty() ? "" : " -> ") + p->name;
    return out;
}

void printPathTable(const PathNode* p) {
    for (; p; p = p->next) std::printf("    %-12s g=%8.3f h=%8.3f f=%8.3f psafe=%.4f\n", p->name.c_str(), p->g, p->h, p->f, p->pSafe);
}

void printEvents(const std::vector<std::vector<double> >& log, std::string (*label)(int)) {
    int step = 0;
    for (size_t i = 0; i < log.size(); ++i) {
        const std::vector<double>& e = log[i];
        if (e[0] == 0) std::printf("  [%2d] EXPAND %-8s g=%.3f h=%.3f f=%.3f\n", ++step, label(int(e[1])).c_str(), e[2], e[3], e[2] + e[3]);
        else std::printf("         push %-8s g=%.3f h=%.3f f=%.3f\n", label(int(e[1])).c_str(), e[2], e[3], e[2] + e[3]);
    }
}

std::string indexLabel(int v) { return std::to_string(v); }
int gridCols = 0;
std::string cellLabel(int v) { return "(" + std::to_string(v / gridCols) + "," + std::to_string(v % gridCols) + ")"; }
std::string pcbLabel(int v) {
    static const char* dirs[5] = {"U", "D", "L", "R", "-"};
    int cell = v / 5;
    return "(" + std::to_string(cell / gridCols) + "," + std::to_string(cell % gridCols) + ")" + dirs[v % 5];
}

double dijkstraCost(int n, double cost[100][100], int s, int t) {
    std::vector<double> d(n, 1e18);
    std::vector<bool> done(n, false);
    d[s] = 0;
    for (int it = 0; it < n; ++it) {
        int u = -1;
        for (int v = 0; v < n; ++v) if (!done[v] && (u < 0 || d[v] < d[u])) u = v;
        if (u < 0 || d[u] >= 1e18) break;
        done[u] = true;
        for (int v = 0; v < n; ++v) if (cost[u][v] > 0 && d[u] + cost[u][v] < d[v]) d[v] = d[u] + cost[u][v];
    }
    return d[t];
}

void task1() {
    std::printf("\n##### TASK 1 #####\n");
    clearAll();
    link(W, 0, 1, 1); link(W, 0, 2, 4); link(W, 1, 2, 1); link(W, 1, 3, 5);
    link(W, 2, 3, 1); link(W, 3, 4, 2); link(W, 2, 4, 6);
    std::vector<std::vector<double> > log;
    astar::eventLog = &log;
    PathNode* p = findMeshPath(W, 0, 4);
    astar::eventLog = nullptr;
    printEvents(log, indexLabel);
    std::printf("  path %s, expanded %ld\n", pathString(p).c_str(), astar::expandedCount);
    printPathTable(p);
    freePath(p);
}

void buildStations(int n, bool coupled) {
    clearAll();
    for (int i = 0; i < n; ++i) {
        C3[i][0] = int(uniform() * 100);
        C3[i][1] = int(uniform() * 100);
        C3[i][2] = int(uniform() * 60);
    }
    for (int u = 0; u < n; ++u) {
        std::vector<std::pair<double, int> > near;
        for (int v = 0; v < n; ++v) {
            if (u == v) continue;
            double dx = C3[u][0] - C3[v][0], dy = C3[u][1] - C3[v][1], dz = C3[u][2] - C3[v][2];
            near.push_back(std::make_pair(std::sqrt(dx * dx + dy * dy + dz * dz), v));
        }
        std::sort(near.begin(), near.end());
        for (int k = 0; k < 4; ++k) {
            double w = coupled ? std::ceil(near[k].first) : 1 + int(uniform() * 9);
            link(W, u, near[k].second, w);
        }
    }
}

void task2() {
    std::printf("\n##### TASK 2 #####\n");
    const char* names[3] = {"Manhattan", "Euclidean", "Chebyshev"};
    struct Case { int n; bool coupled; unsigned seed; };
    Case cases[4] = {{30, true, 11}, {60, true, 12}, {90, true, 13}, {60, false, 14}};
    for (int c = 0; c < 4; ++c) {
        seed = cases[c].seed;
        buildStations(cases[c].n, cases[c].coupled);
        double cost[100][100];
        for (int u = 0; u < 100; ++u)
            for (int v = 0; v < 100; ++v)
                cost[u][v] = W[u][v] > 0 ? W[u][v] * (1 + std::fabs(double(C3[v][2] - C3[u][2])) / 100.0) : 0;
        int goal = cases[c].n - 1;
        double opt = dijkstraCost(cases[c].n, cost, 0, goal);
        std::printf("case %d: n=%d coupled=%d optimal=%.3f\n", c + 1, cases[c].n, cases[c].coupled, opt);
        for (int mode = 1; mode <= 3; ++mode) {
            PathNode* p = findAUVPath(W, C3, 0, goal, mode);
            std::printf("  %-10s cost=%9.3f expanded=%3ld hops=%2d gap=%.2f%%\n", names[mode - 1],
                        p ? pathTail(p)->g : -1.0, astar::expandedCount, pathLength(p) - 1,
                        p ? 100.0 * (pathTail(p)->g - opt) / opt : 0.0);
            freePath(p);
        }
    }
    clearAll();
    int st[5][3] = {{0, 0, 0}, {4, 2, 60}, {5, 0, 10}, {9, 3, 20}, {12, 0, 0}};
    for (int i = 0; i < 5; ++i) for (int k = 0; k < 3; ++k) C3[i][k] = st[i][k];
    link(W, 0, 1, 5); link(W, 0, 2, 6); link(W, 1, 3, 5); link(W, 2, 3, 4); link(W, 3, 4, 4); link(W, 2, 4, 9);
    for (int mode = 1; mode <= 3; ++mode) {
        std::vector<std::vector<double> > log;
        astar::eventLog = &log;
        PathNode* p = findAUVPath(W, C3, 0, 4, mode);
        astar::eventLog = nullptr;
        std::printf(" demo %s: %s expanded=%ld\n", names[mode - 1], pathString(p).c_str(), astar::expandedCount);
        printPathTable(p);
        printEvents(log, indexLabel);
        freePath(p);
    }
}

void task3() {
    std::printf("\n##### TASK 3 #####\n");
    clearAll();
    int pts[4][2] = {{0, 0}, {5, 0}, {5, 4}, {10, 0}};
    for (int i = 0; i < 4; ++i) C2[i][0] = pts[i][0], C2[i][1] = pts[i][1];
    link(W, 0, 1, 5); link(W, 1, 3, 5); link(W, 0, 2, 7); link(W, 2, 3, 7);
    link(R, 0, 1, 0.5); link(R, 1, 3, 0.5);
    double lambdas[6] = {0, 2, 4, 4.01, 6, 10};
    for (int i = 0; i < 6; ++i) {
        PathNode* p = findProbabilisticPath(W, R, C2, 0, 3, lambdas[i], 2);
        std::printf(" lambda=%.2f %s g=%.3f psafe=%.4f\n", lambdas[i], pathString(p).c_str(), pathTail(p)->g, pathTail(p)->pSafe);
        freePath(p);
    }
    clearAll();
    seed = 99;
    const int side = 6;
    for (int x = 0; x < side; ++x)
        for (int y = 0; y < side; ++y) {
            int id = x * side + y;
            C2[id][0] = x * 10;
            C2[id][1] = y * 10;
        }
    for (int x = 0; x < side; ++x)
        for (int y = 0; y < side; ++y) {
            int id = x * side + y;
            bool hazardous = x >= 1 && x <= 4 && y >= 1 && y <= 4;
            if (y + 1 < side) {
                link(W, id, id + 1, 10 + int(uniform() * 3));
                link(R, id, id + 1, hazardous ? 0.05 + uniform() * 0.15 : uniform() * 0.01);
            }
            if (x + 1 < side) {
                link(W, id, id + side, 10 + int(uniform() * 3));
                link(R, id, id + side, hazardous ? 0.05 + uniform() * 0.15 : uniform() * 0.01);
            }
            if (x + 1 < side && y + 1 < side) {
                link(W, id, id + side + 1, 14 + int(uniform() * 3));
                link(R, id, id + side + 1, hazardous ? 0.1 + uniform() * 0.2 : uniform() * 0.01);
            }
        }
    double sweep[7] = {0, 10, 25, 50, 100, 200, 500};
    for (int i = 0; i < 7; ++i) {
        PathNode* p = findProbabilisticPath(W, R, C2, 0, side * side - 1, sweep[i], 2);
        double dist = 0, risk = 0;
        int prev = -1;
        for (const PathNode* q = p; q; q = q->next) {
            int v = std::stoi(q->name);
            if (prev >= 0) dist += W[prev][v], risk += R[prev][v];
            prev = v;
        }
        std::printf(" grid lambda=%5.0f hops=%d dist=%.0f risk=%.3f g=%.3f psafe=%.4f expanded=%ld\n  %s\n", sweep[i],
                    pathLength(p) - 1, dist, risk, pathTail(p)->g, pathTail(p)->pSafe, astar::expandedCount, pathString(p).c_str());
        freePath(p);
    }
}

void loadGrid(const char* rows[], int m) {
    clearAll();
    for (int x = 0; x < m; ++x)
        for (int y = 0; rows[x][y]; ++y) G[x][y] = rows[x][y] - '0';
}

void task4() {
    std::printf("\n##### TASK 4 #####\n");
    const char* rows[4] = {"0000", "0120", "0100", "0000"};
    loadGrid(rows, 4);
    gridCols = 4;
    for (int mode = 1; mode <= 2; ++mode) {
        std::vector<std::vector<double> > log;
        astar::eventLog = &log;
        PathNode* p = findGamePath(G, 4, 4, 0, 0, 3, 3, mode);
        astar::eventLog = nullptr;
        std::printf(" mode %d expanded=%ld\n", mode, astar::expandedCount);
        printEvents(log, cellLabel);
        printPathTable(p);
        freePath(p);
    }
    const char* names[2] = {"Manhattan", "Chebyshev"};
    seed = 7;
    for (int t = 0; t < 3; ++t) {
        int m = 20 + 10 * t, n = 20 + 10 * t;
        clearAll();
        for (int x = 0; x < m; ++x)
            for (int y = 0; y < n; ++y) {
                double r = uniform();
                G[x][y] = r < 0.2 ? 1 : (r < 0.35 ? 2 : 0);
            }
        G[0][0] = G[m - 1][n - 1] = 0;
        std::printf(" random %dx%d\n", m, n);
        for (int mode = 1; mode <= 2; ++mode) {
            PathNode* p = findGamePath(G, m, n, 0, 0, m - 1, n - 1, mode);
            std::printf("  %-10s cost=%.2f expanded=%ld moves=%d\n", names[mode - 1], p ? pathTail(p)->g : -1.0,
                        astar::expandedCount, pathLength(p) - 1);
            freePath(p);
        }
    }
}

void task5() {
    std::printf("\n##### TASK 5 #####\n");
    const char* rows[4] = {"0000", "0110", "0000", "1000"};
    loadGrid(rows, 4);
    gridCols = 4;
    std::vector<std::vector<double> > log;
    astar::eventLog = &log;
    PathNode* p = findPCBPath(G, 4, 4, 0, 0, 3, 3, W, 1);
    astar::eventLog = nullptr;
    std::printf(" expanded=%ld\n", astar::expandedCount);
    printEvents(log, pcbLabel);
    printPathTable(p);
    freePath(p);
    const char* names[2] = {"Manhattan", "Chebyshev"};
    seed = 5;
    for (int t = 0; t < 3; ++t) {
        int m = 8 + t, n = 10;
        clearAll();
        for (int x = 0; x < m; ++x)
            for (int y = 0; y < n; ++y) G[x][y] = uniform() < 0.2 ? 1 : 0;
        G[0][0] = G[m - 1][n - 1] = 0;
        std::printf(" random %dx%d\n", m, n);
        for (int mode = 1; mode <= 2; ++mode) {
            PathNode* q = findPCBPath(G, m, n, 0, 0, m - 1, n - 1, W, mode);
            std::printf("  %-10s cost=%.2f expanded=%ld moves=%d\n", names[mode - 1], q ? pathTail(q)->g : -1.0,
                        astar::expandedCount, pathLength(q) - 1);
            freePath(q);
        }
    }
}

}

int main() {
    task1();
    task2();
    task3();
    task4();
    task5();
    return 0;
}
