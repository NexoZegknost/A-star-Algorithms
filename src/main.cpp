#include <iostream>
#include <string>

#include "Algo.h"
#include "PathNode.h"

using namespace std;

static double weightMatrix[100][100];
static double riskMatrix[100][100];
static int coords3D[100][3];
static int coords2D[100][2];
static int grid[100][100];

void printPath(PathNode* head) {
    cout << "Solution Path:\n";
    while (head != nullptr) {
        cout << "Node: " << head->name
             << " | f: " << head->f
             << " | g: " << head->g
             << " | h: " << head->h << "\n";
        head = head->next;
    }
}

static void printSafety(const PathNode* head) {
    cout << "Safety:";
    for (; head != nullptr; head = head->next) cout << " " << head->name << "=" << head->pSafe;
    cout << "\n";
}

static void reset() {
    for (int i = 0; i < 100; ++i) {
        for (int j = 0; j < 100; ++j) {
            weightMatrix[i][j] = riskMatrix[i][j] = 0.0;
            grid[i][j] = 0;
        }
        coords3D[i][0] = coords3D[i][1] = coords3D[i][2] = 0;
        coords2D[i][0] = coords2D[i][1] = 0;
    }
}

static void link(double matrix[100][100], int u, int v, double value) {
    matrix[u][v] = matrix[v][u] = value;
}

static void loadGrid(const string rows[], int m) {
    for (int x = 0; x < m; ++x)
        for (size_t y = 0; y < rows[x].size(); ++y) grid[x][y] = rows[x][y] - '0';
}

static void show(const string& title, PathNode* path) {
    cout << "\n=== " << title << " ===\n";
    if (path == nullptr) {
        cout << "No path found\n";
        return;
    }
    printPath(path);
}

static void demoMesh() {
    reset();
    link(weightMatrix, 0, 1, 1);
    link(weightMatrix, 0, 2, 4);
    link(weightMatrix, 1, 2, 1);
    link(weightMatrix, 1, 3, 5);
    link(weightMatrix, 2, 3, 1);
    link(weightMatrix, 3, 4, 2);
    link(weightMatrix, 2, 4, 6);
    PathNode* path = findMeshPath(weightMatrix, 0, 4);
    show("Task 1: Mesh network 0 -> 4", path);
    freePath(path);
}

static void demoAUV() {
    reset();
    int stations[5][3] = {{0, 0, 0}, {4, 2, 60}, {5, 0, 10}, {9, 3, 20}, {12, 0, 0}};
    for (int i = 0; i < 5; ++i)
        for (int k = 0; k < 3; ++k) coords3D[i][k] = stations[i][k];
    link(weightMatrix, 0, 1, 5);
    link(weightMatrix, 0, 2, 6);
    link(weightMatrix, 1, 3, 5);
    link(weightMatrix, 2, 3, 4);
    link(weightMatrix, 3, 4, 4);
    link(weightMatrix, 2, 4, 9);
    const string names[3] = {"Manhattan", "Euclidean", "Chebyshev"};
    for (int mode = 1; mode <= 3; ++mode) {
        PathNode* path = findAUVPath(weightMatrix, coords3D, 0, 4, mode);
        show("Task 2: AUV 0 -> 4 (" + names[mode - 1] + ")", path);
        freePath(path);
    }
}

static void demoProbabilistic() {
    reset();
    int points[4][2] = {{0, 0}, {5, 0}, {5, 4}, {10, 0}};
    for (int i = 0; i < 4; ++i) {
        coords2D[i][0] = points[i][0];
        coords2D[i][1] = points[i][1];
    }
    link(weightMatrix, 0, 1, 5);
    link(weightMatrix, 1, 3, 5);
    link(weightMatrix, 0, 2, 7);
    link(weightMatrix, 2, 3, 7);
    link(riskMatrix, 0, 1, 0.5);
    link(riskMatrix, 1, 3, 0.5);
    const int lambdas[3] = {0, 3, 10};
    for (int i = 0; i < 3; ++i) {
        PathNode* path = findProbabilisticPath(weightMatrix, riskMatrix, coords2D, 0, 3, lambdas[i], 2);
        show("Task 3: Hazard routing 0 -> 3 (lambda = " + to_string(lambdas[i]) + ")", path);
        if (path != nullptr) printSafety(path);
        freePath(path);
    }
}

static void demoGame() {
    reset();
    const string rows[5] = {"00000", "01120", "00100", "02000", "00010"};
    loadGrid(rows, 5);
    PathNode* manhattan = findGamePath(grid, 5, 5, 0, 0, 4, 4, 1);
    PathNode* chebyshev = findGamePath(grid, 5, 5, 0, 0, 4, 4, 2);
    show("Task 4: Game grid (0,0) -> (4,4) (Manhattan)", manhattan);
    show("Task 4: Game grid (0,0) -> (4,4) (Chebyshev)", chebyshev);
    freePath(manhattan);
    freePath(chebyshev);
}

static void demoPCB() {
    reset();
    const string rows[4] = {"0000", "0110", "0000", "1000"};
    loadGrid(rows, 4);
    PathNode* path = findPCBPath(grid, 4, 4, 0, 0, 3, 3, weightMatrix, 1);
    show("Task 5: PCB trace (0,0) -> (3,3) (Manhattan)", path);
    cout << "Weight matrix (cells 0..15):\n";
    for (int i = 0; i < 16; ++i) {
        for (int j = 0; j < 16; ++j) cout << weightMatrix[i][j] << (j == 15 ? "\n" : " ");
    }
    freePath(path);
}

int main() {
    demoMesh();
    demoAUV();
    demoProbabilistic();
    demoGame();
    demoPCB();
    return 0;
}
