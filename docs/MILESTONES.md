# Milestones

Deadline: **December 6th, 2026**.

| ID | Milestone | Deliverable | Status |
|----|-----------|-------------|--------|
| M0 | Repository scaffold | Layout, Makefile, header check, docs | Done |
| M1 | Shared core | `PathNode`, binary-heap A* engine, heuristics | Pending |
| M2 | Task 1 — Mesh network routing | `findMeshPath` + tests | Pending |
| M3 | Task 2 — AUV 3D navigation | `findAUVPath` + tests | Pending |
| M4 | Task 3 — Probabilistic hazard routing | `findProbabilisticPath` + tests | Pending |
| M5 | Task 4 — Game multi-terrain grid | `findGamePath` + tests | Pending |
| M6 | Task 5 — PCB wire routing | `findPCBPath` + tests | Pending |
| M7 | Demo and packaging | `main.cpp`, `scripts/package.sh` | Pending |
| M8 | Report | `report/<StudentID>.pdf` | Pending |

## Constraints from the specification

- C++ only, headers limited to `<iostream>`, `<fstream>`, `<string>`, `<cmath>`, `<vector>`, `<algorithm>` (enforced by `make check-headers`).
- Submission files: `PathNode.h`, `PathNode.cpp`, `Algo.h`, `Algo.cpp`, `main.cpp`, `<StudentID>.pdf`.
- `printPath` is provided by the LMS and must not be part of the submitted code.
- Folder `<StudentID>/` compressed as a genuine `.rar` archive named `<StudentID>.rar`.
- Tie-breaking ambiguities must be aligned with the LMS test cases once the portal opens.
