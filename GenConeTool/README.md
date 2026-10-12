# Cone Surface Point Cloud Generator (`gen_cone.py`)

This script generates a PLY file containing randomly sampled points on the *outer surface* of a cone. It is useful for recognizing cones via ICP.

## Features

- Generates only surface points (no base or interior points)
- Configurable cone position, height, base radius, and number of points
- Outputs a standard ASCII PLY file for use in 3D processing tools

## Usage

### Requirements

- Python 3.x
- numpy

Install numpy if needed:
```sh
pip install numpy
```

### Run the Script

```sh
python3 gen_cone.py
```

By default, this will create a file named `cmake-build-debug/cone_surface_only.ply` with 200 surface points.

The model shipped with the viewer, `GUIAndRacingLine/external_files/ideal_cone.ply`, has 150 surface
points, height 0.40 m and base radius 0.20 m.

### Customization

You can change the cone parameters by editing the `generate_cone_surface_ply` call in the `__main__` block:
- `x0`, `y0`: Base center coordinates
- `height`: Cone height
- `base_radius`: Radius at the base
- `num_surface_points`: Number of points to generate

Example:
```python
generate_cone_surface_ply(
    "output.ply",
    x0=1.0,
    y0=2.0,
    height=0.5,
    base_radius=0.3,
    num_surface_points=500,
)
```

## Output

- ASCII PLY file with columns: x, y, z
- Each row is a point on the cone's surface

## Example

```
ply
format ascii 1.0
element vertex 200
property float x
property float y
property float z
end_header
0.0000 0.0000 0.2000
...
```
