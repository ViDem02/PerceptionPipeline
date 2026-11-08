# E‑Agle Driverless Perception — LiDAR Pipeline

This repo contains my complete LiDAR-only perception pipeline for a Formula Student Driverless vehicle. I implemented all five levels plus the bonus interactive visualizer. The work is written in modern C++ with CMake, Point Cloud Library (PCL), and an optimization component for the racing line.

I did not implement the image path; all deliverables are LiDAR-focused.

## What’s included
- Level 1 — LiDAR loading, visualization, and basic preprocessing
- Level 2 — Cone extraction from point clouds
- Level 3 — Left/Right cone discrimination (track edge separation)
- Level 4 — Racing line optimization (minimal curvature with bounds)
- Level 5 — LiDAR odometry (pairwise registration)
- Bonus — Interactive GUI to visualize clouds, edges, and racing line

## Key modules and where to find them

Interactive GUI + pipeline (bonus and integration):
- `branch_deliverable/GUIAndRacingLine/` — Qt + PCL visualizer app
	- `logic/get_cones_cloud.cpp` — extracts cloud of candidate cones from the input point cloud
	- `logic/get_obstacle_cloud.cpp` — optional obstacle isolation for clarity and evaluation
	- `logic/distinguish_left_right.cpp` — separates cones into left and right track edges
	- `logic/racingline_nlopt.cpp` — racing line optimization (see “Racing line” below)
	- `pclviewer.*` and `*.ui` — Qt-based viewer (bonus), overlays edges and racing line

Odometry (Level 5):
- `branch_odometry/odometry_with_processing/` — LiDAR odometry with preprocessing
	- `main.cpp` — pairwise registration pipeline (ICP-based), displacement export
	- `displacement_plot.m` — quick plot of estimated motion

Racing line (standalone minimal example used during development):
- `pathplanning/racingline_nlopt_cpp/` — small NLOPT demo of the racing-line objective

Documentation and task specification:
- `branch_deliverable/original_readme.md` — original task brief

## Algorithms (concise)

1) Cone extraction (Level 2)
- Spatial filtering and geometric priors to isolate 3D points consistent with cones
- Output: PCL cloud of cone centroids for subsequent track-edge logic

2) Left/Right cone discrimination (Level 3)
- Greedy growth on both sides using local track direction (heading from the last two cones)
- Dynamic side assignment on curves: a cone is “left” if the cross product with the current heading is > 0, otherwise “right”
- Angle gating: the incremental turning angle must stay within bounds to avoid zig‑zags

3) Racing line optimization (Level 4)
- Decision variables: lateral offsets along local normals of the midpoints between left and right edges
- Objective: minimize squared curvature + small smoothness term, subject to staying inside the corridor (bound constraints)
- Solver: derivative‑free NLOPT (LN_BOBYQA) — robust and simple to deploy

4) LiDAR odometry (Level 5)
- Pairwise registration of successive point clouds (ICP)
- Produces an estimated displacement/trajectory; plotting provided for quick inspection


## Build and run

### GUIAndRacingLine

This process has been tested for MacOS 15, M3 processor. 

Run these commands from the repository root:

```
cmake -DCMAKE_BUILD_TYPE=Debug -G Ninja -S GUIAndRacingLine -B GUIAndRacingLine/cmake-build-debug
cmake --build GUIAndRacingLine/cmake-build-debug
./GUIAndRacingLine/cmake-build-debug/pcl_visualizer
```

### OdometryWithProcessing

This process has been tested for MacOS 15, M3 processor. 

```
cmake -DCMAKE_BUILD_TYPE=Debug -G Ninja -S OdometryWithProcessing -B OdometryWithProcessing/cmake-build-debug
cmake --build OdometryWithProcessing/cmake-build-debug
./OdometryWithProcessing/cmake-build-debug/sample_consensus first.pcd second.pcd
```


## Repository guide (selected files)
- `branch_deliverable/GUIAndRacingLine/logic/get_cones_cloud.cpp` — cone segmentation
- `branch_deliverable/GUIAndRacingLine/logic/distinguish_left_right.cpp` — left/right split
- `branch_deliverable/GUIAndRacingLine/logic/racingline_nlopt.cpp` — racing line
- `branch_odometry/odometry_with_processing/main.cpp` — LiDAR odometry
- `pathplanning/racingline_nlopt_cpp/racingline_nlopt.cpp` — compact NLOPT example

## Results at a glance
- Clean separation of left/right edges on curved tracks (dynamic side logic)
- Smooth, corridor‑constrained racing line with low curvature
- Consistent odometry over short sequences with ICP registration

## Notes & future work
- Add dataset loaders and scripted benchmarks for reproducibility
- Extend odometry with robust outlier rejection and loop closure
- Export artifacts (edges, racing line) as PCD/CSV for downstream modules