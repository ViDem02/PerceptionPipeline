# GUIAndRacingLine

To build this project, the use of CLion is advised. 

This process has been tested for MacOS 15, M3 processor. 

```
cmake -DCMAKE_BUILD_TYPE=Debug -G Ninja -S . -B cmake-build-debug  
cd cmake-build-debug  
cmake --build .
./sample_consensus first.pcd second.pcd
```
## Tests and evaluations

`ctest` (from the build directory) runs the automatic checks:

| Test | What it checks against a known answer |
|---|---|
| `left_right_synthetic` (`eval_left_right`) | Left/right split on synthetic tracks: every clean track classified perfectly |
| `racing_line` (`test_racing_line`) | Racing line: centreline on a straight, outer edge on an arc, exact interpolation through the optimised points, bounds, no reversal, degenerate inputs |
| `cone_icp` (`test_cone_icp`) | Cone ICP: the transform is the inverse of a known tilt; a rotation about the cone axis (yaw) is not observable |

Two programs also write HTML reports for visual review:

- `eval_left_right --pcd cones.pcd --model ideal_cone.ply --report left_right_report.html`:
  the left/right split on synthetic tracks (clean, noisy, missing cones; 100 seeds each)
  and on the real cloud. Wrong cones are circled in red.
- `eval_pipeline --model ideal_cone.ply --report pipeline_report.html`: the whole pipeline
  (detection, left/right, racing line) on simulated LiDAR scans (`tests/lidar_sim.h`: 24,000
  rays, sensor 0.82 m above the ground, walls, obstacles, cone-sized hard negatives) with
  precision, recall, centroid error, side accuracy and share of the racing line inside the
  true track. No pass/fail threshold: it measures the detector as it is.

`test_cone_icp` also prints yaw, tilt and fitness of simulated views of cones and of
cone-sized non-cones, to compare the recognition gates.

In the GUI, the classified cone centres are drawn in blue (left) and yellow (right).
With "verbose" checked, each cone candidate prints fitness, yaw, tilt, size and position.

Frame convention (after the transform applied on load): x forward, y up, z right. The track
lies on the (x, z) plane.
