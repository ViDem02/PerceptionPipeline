# Project Statistics

## Formula Student Driverless — C++ LiDAR Perception

**Snapshot date:** 2026-10-04  
**Scope:** LiDAR perception, cone recognition, track-edge classification, racing-line optimization, ICP odometry, and interactive visualization.

## Achievement summary

- Completed **5 LiDAR perception levels**:
  1. Point-cloud loading, preprocessing, and visualization
  2. Cone extraction and recognition
  3. Left/right track-edge discrimination
  4. Corridor-constrained racing-line optimization
  5. Pairwise LiDAR odometry using ICP registration
- Delivered **1 interactive GUI bonus module** for inspecting point clouds, obstacles, cone edges, and the racing line.
- Implemented the core pipeline in **modern C++20** with CMake.
- Added cross-version Qt discovery for **Qt 5 or Qt 6**, plus PCL, VTK, and NLopt integration.
- Tested build/run instructions on **macOS 15 with an Apple M3 processor**.

## Engineering footprint

| Metric | Result |
|---|---:|
| Repository commits | **89** |
| Tracked C++ implementation files | **47** |
| Tracked header files | **25** |
| GUIAndRacingLine C++/header LOC | **1,862** |
| OdometryWithProcessing LOC | **225** |
| Main delivered C++ modules | **7** |
| Qt UI definitions | **2** |
| Sample point-cloud assets tracked | **14 PCD** |
| Cone/model assets tracked | **7 PLY** |

The GUI module includes dedicated implementations for cone recognition, cone-cloud extraction, obstacle isolation, left/right discrimination, racing-line optimization, and the Qt/PCL viewer.

## Algorithm and runtime statistics

| Capability | Implementation evidence |
|---|---|
| Point-cloud processing | PCL `PointCloud<PointXYZ>` pipeline with PCD input |
| Preprocessing | Pass-through filtering, coordinate transforms, and statistical outlier removal |
| Outlier-removal configuration | `MeanK = 50`, standard-deviation multiplier `= 1.0` |
| ICP registration | PCL `IterativeClosestPoint<PointXYZ, PointXYZ>` |
| ICP iteration budget | **70 maximum iterations** |
| ICP timing | Runtime timing captured with PCL `TicToc` and reported in **milliseconds** |
| ICP quality output | Fitness score, convergence status, final 4x4 transformation, and north/east displacement |
| Cone recognition timing | Runtime timing instrumented around cone-model ICP alignment |
| Racing-line solver | NLopt derivative-free `LN_BOBYQA` optimizer |
| Racing-line optimization budget | **1,000 maximum objective evaluations** per optimization |
| Racing-line stopping tolerance | Relative tolerance `1e-6` |
| Track-edge logic | Dynamic left/right assignment using heading and cross-product geometry |
| Visualization | Qt + PCL interactive viewer with cloud, obstacle, edge, and racing-line overlays |

## Performance measurement status

The odometry pipeline was benchmarked on 2026-10-04 using the tracked
`first.pcd` and `second.pcd` inputs (24,000 points each) on macOS 15.6.1,
Mac15,12, with 8 logical CPUs. Ten headless runs used the same preprocessing
and 70-iteration ICP path as the GUI executable; visualization was disabled
only through `PERCEPTION_HEADLESS=1`. CPU and memory values below come from
`/usr/bin/time -l`, and ICP timing comes from the application's PCL `TicToc`.

| Requested metric | Current status |
|---|---|
| Processing time per point cloud (ms) | **2,291.18 ms mean** (median 2,268.88; min 2,253.55; max 2,401.99) for the measured ICP pipeline |
| Points processed per frame | **24,000 input points per cloud** |
| Achievable processing frequency (Hz) | **0.436 Hz** (`1000 / 2,291.18 ms`, approximately 2.29 s per pair) |
| ICP execution time | **Measured** by PCL `TicToc`; the ten individual timings are retained in the benchmark run output |
| CPU usage | **94.9% average process utilization** over ten headless runs (`(30.49 s user + 0.72 s sys) / 32.88 s wall`) |
| Memory usage | **110.6 MiB peak resident set size** (`115,982,336` bytes) over the benchmark |

These measurements describe the tracked 24,000-point pair on this host; they
are not a claim of real-time performance for larger clouds, different sensors,
or GUI rendering.
