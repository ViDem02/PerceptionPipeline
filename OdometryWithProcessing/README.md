# LiDAR Odometry with ICP

Estimates the vehicle motion between two LiDAR scans with point-to-point ICP (PCL). The logic is in
`odometry.h`/`odometry.cpp`; `main.cpp` is the command-line program and viewer.

## Features

- Loads two point cloud files (`.pcd`)
- Preprocesses clouds (pass-through filter, outlier removal, transforms)
- Runs ICP to estimate relative pose
- Prints the transformation and the vehicle displacement
- Visualizes original and aligned clouds side-by-side

## Usage

### Build

Tested on macOS 15 (Apple M3).

```
cmake -DCMAKE_BUILD_TYPE=Release -G Ninja -S . -B build
cmake --build build
ctest --test-dir build
```

### Run

```sh
cd "external files" && ../build/sample_consensus first.pcd second.pcd
```

For a repeatable headless benchmark using the same preprocessing and ICP path
without opening the visualization window:

```sh
PERCEPTION_HEADLESS=1 ./sample_consensus first.pcd second.pcd
```

## Output

- Console: ICP score (mean squared distance of the matched pairs), whether ICP stopped at the
  iteration cap, the transformation first -> second, and the vehicle displacement
  (-R^T t, in the frame of the first scan)
- GUI: left, second.pcd (white) and first.pcd (green); right, second.pcd (white) and first.pcd
  aligned by ICP (red)

## ICP settings and ground-truth test

Correspondences farther than 1 m are rejected (`kMaxCorrespondenceDistance` in `odometry.h`).
`test_odometry` (run by `ctest`) builds a synthetic second scan from `first.pcd` with a known
motion (independent sampling, noise, partial overlap) and checks the estimate. With 1 m the
motion is recovered within 2 mm and below 0.01 deg; with PCL's default unbounded correspondences
(the previous setting) the error is about 0.8 m and 10 deg.

## Notes

- Axes (levelled frame): x forward ("East" in the original naming), y up, z right ("North")
- Adjust file names and parameters as needed
