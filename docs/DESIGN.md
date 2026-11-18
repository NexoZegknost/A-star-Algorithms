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
