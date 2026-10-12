# LiDAR Odometry with ICP (`main.cpp`)

This C++ script performs point cloud odometry using the Iterative Closest Point (ICP) algorithm with PCL. It aligns two `.pcd` files, estimates the transformation, and visualizes the result.

## Features

- Loads two point cloud files (`.pcd`)
- Preprocesses clouds (pass-through filter, outlier removal, transforms)
- Runs ICP to estimate relative pose
- Prints transformation matrix and north/east displacement
- Visualizes original and aligned clouds side-by-side

## Usage

### Build

This process has been tested for MacOS 15, M3 processor. 

For the building of this project using CMake is the easiest root. 

```
cmake -DCMAKE_BUILD_TYPE=Debug -G Ninja -S . -B cmake-build-debug  
cd cmake-build-debug  
cmake --build .
./sample_consensus first.pcd second.pcd
```

### Run

```sh
./odometry_icp first.pcd second.pcd
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
