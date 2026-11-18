# Design Notes

## Shared engine

All tasks reduce to an explicit directed graph `astar::Graph` (adjacency lists plus one
heuristic value per vertex) and call `astar::search`.

- Open list: binary heap built on `std::push_heap` / `std::pop_heap` (no `<queue>` on CodeRunner).
- Ordering: smallest `f`, then smallest `h`, then earliest insertion.
- Lazy deletion: stale heap entries (`g` larger than the best known) are skipped.
- No closed set: a vertex is re-opened whenever a cheaper `g` is found, so the search stays
  correct for inconsistent heuristics (Task 2 and Task 3 metrics are not guaranteed admissible).
- Output: one `PathNode` per vertex on the path, carrying the final `g`, the vertex `h`
  and `f = g + h`.
- Invalid input (out-of-range ids, blocked endpoints) or no path returns `nullptr`.

## Task 1 — Mesh network routing

- Edge `u → v` exists when `weightMatrix[u][v] > 0` and `u ≠ v`.
- `degree(v)` is the number of positive entries in row `v` (active outgoing links; equal to
  the undirected degree for symmetric matrices).
- `Cost(u → v) = weightMatrix[u][v] + 0.1 · degree(v)`.
- `h(v) = hops(v) · c_min`, where `hops(v)` is the reverse-BFS hop count from `v` to the goal
  and `c_min` the cheapest edge cost in the network. Every remaining path needs at least
  `hops(v)` edges costing at least `c_min`, so `h` is admissible and consistent.
- Node labels are vertex indices (`"0"`, `"1"`, …).

## Task 2 — AUV navigation in 3D

- Edge `u → v` exists when `weightMatrix[u][v] > 0` and `u ≠ v`.
- Step cost `w(u, v) · (1 + |z_v − z_u| / 100)`.
- Heuristic modes: `1` Manhattan, `2` Euclidean, `3` Chebyshev on `(x, y, z)`; any other mode
  returns `nullptr`.
- The metrics are measured in coordinate units while costs are energy units, so admissibility
  depends on the data; the engine re-opens vertices to remain correct either way.

## Task 3 — Probabilistic hazard routing

- Edge `u → v` exists when `distMatrix[u][v] > 0` and `u ≠ v`.
- Step cost `d(u, v) + λ · P_risk(u, v)`; `g` is the expected cost accumulated along the path.
- Heuristic modes: `1` Manhattan, `2` Euclidean on `(x, y)`.
- `PathNode::pSafe` stores the survival probability `∏ (1 − P_risk)` accumulated from the start
  up to that node (`1` at the start). The field sits after `next` so the struct still matches the
  LMS `printPath` and aggregate initialisation of the five required fields.
- Break-even example used in the tests: a risky route `d = 10`, total risk `1.0` versus a safe route
  `d = 14`; the safe route wins as soon as `λ > 4`.

## Task 4 — Game navigation on multi-terrain grid

- Cell `(x, y)` is `mapGrid[x][y]`, `x` is the row (`0 … m−1`), `y` the column (`0 … n−1`);
  vertex id `x · n + y`.
- Directions and expansion order: `Up (−1,0)`, `Down (+1,0)`, `Left (0,−1)`, `Right (0,+1)`,
  `Up-Left`, `Up-Right`, `Down-Left`, `Down-Right`.
- Entering a cell costs `1.0` (flat), `2.0` (swamp); obstacles (`1`) are never entered.
  Diagonal moves multiply the cost by `1.5` and require both adjacent cardinal cells
  `(x+dx, y)` and `(x, y+dy)` to be non-obstacles.
- Heuristic modes: `1` Manhattan, `2` Chebyshev. Chebyshev is admissible (every move lowers
  it by at most `1` and costs at least `1`); Manhattan can overestimate after a diagonal step.
- Labels: the first node is `"Start"`, every following node carries the move that reached it.
