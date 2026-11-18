# A* Algorithms — CO1007 Discrete Structures Assignment

Five pathfinding problems solved with a single A* engine, written in C++ restricted to the
LMS CodeRunner standard headers.

| Task | Function | Problem |
|------|----------|---------|
| 1 | `findMeshPath` | Mesh network packet routing with congestion penalty |
| 2 | `findAUVPath` | AUV submarine navigation in 3D space |
| 3 | `findProbabilisticPath` | Probabilistic pathfinding under hazard risk |
| 4 | `findGamePath` | Game character navigation with multi-terrain obstacles |
| 5 | `findPCBPath` | VLSI PCB wire routing with bending penalty |

## Layout

```
src/        PathNode.{h,cpp}, Algo.{h,cpp}, main.cpp  (submission sources)
tests/      Self-contained test runner
docs/       Milestones and design notes
report/     Report sources
tools/      Experiment and trace generator used by the report
scripts/    Header check and submission packaging
```

## Build

```bash
make          # build the demo
make run      # run the demo
make test     # header check + unit tests
make report-data  # regenerate traces and benchmarks in docs/report_data.txt
```

Sample demo output: [docs/sample_output.txt](docs/sample_output.txt).
Design decisions and assumptions per task: [docs/DESIGN.md](docs/DESIGN.md).

## Submission

```bash
scripts/package.sh <StudentID>   # needs report/<StudentID>.pdf and the rar tool
```

Produces `dist/<StudentID>.rar` containing `<StudentID>/` with the five source files and the report.

See [docs/MILESTONES.md](docs/MILESTONES.md) for progress.
