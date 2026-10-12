# LiDAR Perception for Driverless Racing

A C++20 perception pipeline for a Formula Student Driverless car, built on PCL, Eigen and NLopt:

- it finds the track cones in a LiDAR scan,
- assigns them to the left or right track boundary,
- computes a minimum-curvature racing line,
- and estimates the car's motion between two scans with ICP.

A Qt viewer shows every stage and lets you tune the parameters live. It was built for the E-Agle Formula Student Driverless perception task, LiDAR path ([task brief](docs/task_brief.md)).

**Documents:** [Technical report](docs/technical_report.pdf) (results, validation, roadmap) · [Change log](docs/change_log.pdf) (every change of October 2026, with before/after numbers)

## Results at a glance

Measured on an Apple M3 (macOS 15.6.1), Release build, single thread.

| What | Result |
|---|---|
| Recorded scan (`cones.pcd`, 24,000 points) | 67 clusters → 6 cones: 2 left, 3 right, 1 unassigned; racing line through 2 gates |
| Left/right split, 10 synthetic tracks | **10/10** clean tracks correct (previous version: 2/10); 100 % perfect runs with 5 cm noise |
| Racing line, known optima | Centreline on a straight (error 0); outer edge on an arc (within 2.3 % of the half-width) |
| Odometry, known motions | **2 mm** and < 0.01° error (previous settings: 0.77–0.96 m, ≈ 10°) |
| Odometry, recorded pair | 0.41 m forward, 0.27 m left (0.49 m planar), 6.3° rotation; not verifiable (see report) |
| Cone detection, simulated scans | Precision 79 %, recall 37 %, centroid error 6.9 cm |
| Runtime, cone pipeline | 485 ± 22 ms per scan (2.1 Hz): detection 221 ms, obstacles 264 ms, left/right + racing line < 0.1 ms |
| Runtime, odometry ICP | 2.20 s per scan pair (median of 10), 92 % of one core, 108 MiB peak memory |
| Automated tests | 4 `ctest` targets, all passing; 2 tools that write visual HTML reports |

**Where it stands.** The pipeline runs end to end. Everything downstream of detection (boundary assignment, racing line, frame conventions, odometry transform) is verified against exact answers. The weak points are cone recall at range, a cone/non-cone decision that does not separate cones from cone-sized objects, and throughput: about 5× short of a 10 Hz sensor. The [technical report](docs/technical_report.pdf) quantifies each one and gives the next steps.

## Pipeline

Counts on `cones.pcd` with the viewer's default parameters:

| Stage | Method | Output |
|---|---|---|
| Load and level | Fixed rotation to *x* forward, *y* up, *z* right | 24,000 points |
| Height crop | Pass-through, *y* ∈ [−10, −0.2] m | 9,089 |
| Outlier removal | Statistical outlier removal, *k* = 50, α = 1.1 | 8,251 |
| Ground and walls | Region growing on normals (regions ≥ 500 points removed) | 4,025 |
| Clustering | Euclidean, 5 cm tolerance, 8–500 points | 67 clusters |
| Gates | ≥ 10 points, touches the ground, 0.2–0.5 m tall | 9 candidates |
| Cone recognition | ICP against a cone model; fitness < 0.005 m², yaw < 13° | 6 cones |
| Left/right | Side-consistent greedy growth from the closest cone on each side, turn ≤ 45° | 2 + 3 |
| Racing line | Offsets along gate normals, bounded by the track; turning-angle curvature; BOBYQA | 2 gates |
| Obstacles | Scan minus cones, region growing, outlier removal | 5,731 points |

**Odometry** is a separate program. It crops and levels both scans, removes outliers, runs ICP (70 iterations, correspondences within 1 m), and reports the displacement −Rᵀt in the frame of the first scan.

## Validation

The recorded scans have no labels, so every module is tested on inputs whose correct answer is known by construction:

| Module | Ground truth | Tool |
|---|---|---|
| Left/right split | 10 synthetic tracks, clean and with noise or missing cones, 100 seeds each | `eval_left_right` (ctest + HTML report) |
| Racing line | Analytic optima, exact interpolation geometry, bounds | `test_racing_line` (ctest) |
| Cone ICP | Known rotations of the cone model; simulated views of cones and decoys | `test_cone_icp` (ctest) |
| Odometry | Known motion applied to `first.pcd`, independent sampling, noise, partial overlap | `test_odometry` (ctest) |
| Whole cone pipeline | Ray-cast LiDAR scans of 5 scenes with cones, walls, obstacles and decoys | `eval_pipeline` (HTML report) |

The validation found three defects that looked plausible on the recorded scan but were wrong:
- the left/right split used only the lateral coordinate;
- the drawn racing line differed from the optimised one;
- the odometry was off by about 0.8 m.

All three are fixed. Details are in the [change log](docs/change_log.pdf).

## Repository layout

| Path | Content |
|---|---|
| [`GUIAndRacingLine/`](GUIAndRacingLine) | Qt viewer and cone pipeline: `logic/` (one file per stage), `tests/`, `external_files/` (`cones.pcd`, `ideal_cone.ply`) |
| [`OdometryWithProcessing/`](OdometryWithProcessing) | Odometry library (`odometry.h`), command-line program, test, scans in `external files/` |
| [`GenConeTool/`](GenConeTool) | Python generator of the cone surface model |
| [`docs/`](docs) | Technical report and change log (PDF and LaTeX), task brief, bibliography |
| [`template/`](template) | LaTeX template used by the documents |
| [`other/`](other) | R&D archive: prototypes and notes from development, not maintained |
| [`screenshots/`](screenshots) | Screen recording of the first GUI version (November 2025) |

## Build, run and test

Tested on macOS 15.6 (Apple M3) with Homebrew PCL 1.15.1, VTK 9.5.2, NLopt 2.10.0, Eigen 5.0.0 and Boost 1.89. The viewer needs Qt 5 or 6.

**Viewer and cone pipeline**
```sh
cd GUIAndRacingLine
cmake -DCMAKE_BUILD_TYPE=Release -G Ninja -S . -B build
cmake --build build
ctest --test-dir build                          # left_right_synthetic, racing_line, cone_icp
cd external_files && ../build/pcl_visualizer    # Load, then Compute
```
Left cones are drawn as large blue points, right cones as large yellow points, and the racing line as a thin amber line. With *verbose* checked, every cone candidate prints its fitness, yaw, tilt, size and position.

**Visual reports** (from `GUIAndRacingLine/build`)
```sh
./eval_left_right --pcd ../external_files/cones.pcd --model ../external_files/ideal_cone.ply
./eval_pipeline --model ../external_files/ideal_cone.ply
open left_right_report.html pipeline_report.html
```

**Odometry**
```sh
cd OdometryWithProcessing
cmake -DCMAKE_BUILD_TYPE=Release -G Ninja -S . -B build
cmake --build build
ctest --test-dir build                          # odometry_ground_truth
cd "external files" && ../build/sample_consensus first.pcd second.pcd
```
Set `PERCEPTION_HEADLESS=1` to skip the viewer window.

## Limitations and next steps

1. **Cone recall at range.** Outlier removal deletes cones beyond about 6 m and clustering splits them from about 5 m. Retuned filters reach 70 % recall in simulation.
2. **Cone/non-cone decision.** A cone's yaw cannot be measured, so the yaw gate is arbitrary. A tilt gate reaches 94 % precision in simulation. It is pending a check of two rejected candidates on the recorded scan, which look like the nearest pair of cones.
3. **Throughput.** 0.49 s per scan against a 0.1 s budget. Share normals and kd-trees between the two region-growing passes, downsample, parallelise.
4. **Odometry on real data.** Automatic levelling, a motion prior, and point-to-plane ICP or GICP.
5. **Code structure.** Free functions with constants inside the code; move to classes with explicit parameters.

## Engineering footprint

C++20, CMake, PCL, Eigen, NLopt, Qt. Non-blank lines of code:

| Part | Lines |
|---|---|
| Pipeline logic | 1,119 |
| Viewer | 452 |
| Odometry | 239 |
| Tests and evaluation tools | 1,361 |

There are 4 automated tests and 2 report generators.
