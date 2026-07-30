# Spot Navigation Map and Mission Workflow

This package provides a four-stage workflow for preparing a point-cloud map and
authoring a FAR-compatible mission:

1. Downsample, remove sparse outliers, and align a raw point cloud to a local
   `map` frame.
2. Build the FAR boundary and prior visibility graph.
3. Record mission goals interactively in RViz.
4. Validate and densify the recorded mission with the FAR mission compiler.

The commands below assume ROS 2 Humble and this workspace are installed under
`~/spot_ws`.

## Workspace setup

Build and source the package before using its ROS executables:

```bash
cd ~/spot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select spot_navigation
source install/setup.bash

export SPOT_NAV=~/spot_ws/src/spot_navigation
```

The Python dependencies are declared by the package. In particular, PCD loading
requires `pypcd4>=1.4.3,<2`. The alignment tool also requires a graphical
environment because selecting the map frame is interactive.

## Streamlined map preparation

Because alignment and FAR prior-map generation normally run back to back,
`prepare_map.py` provides one transparent two-stage command. It prints each
underlying command before running it and streams both tools' normal output, so
the detected lines, selected frame, grid statistics, warnings, and generated
files remain visible.

The executable entry scripts stay directly under `scripts/`. Their shared
offline map-processing implementation is colocated under `scripts/map_tools/`;
it is internal to these standalone tools rather than part of the ROS package
API.

```bash
python3 "$SPOT_NAV/scripts/prepare_map.py" \
  /path/to/raw_dorsett.pcd \
  --output-dir "$SPOT_NAV/map" \
  --name dorsett
```

This runs the interactive alignment first, names its outputs
`dorsett_transformed.pcd`, `dorsett_transform.json`, and
`dorsett_transform_debug.png`, then builds the `dorsett_transformed` boundary,
trajectory, VGH, statistics, and preview files in the same directory. The
transformed PCD is both voxel-downsampled and noise-filtered; the raw input is
never modified.

Alignment options stay before `--`; everything after `--` is passed unchanged
to `build_far_prior_map.py`:

```bash
python3 "$SPOT_NAV/scripts/prepare_map.py" \
  /path/to/raw_dorsett.pcd \
  --output-dir "$SPOT_NAV/map" \
  --name dorsett \
  --voxel-size 0.05 \
  --statistical-neighbors 20 \
  --statistical-std-ratio 2.0 \
  --radius-outlier-neighbors 3 \
  --radius-outlier-radius 0.30 \
  --ransac-threshold 0.08 \
  --max-lines 16 \
  -- \
  --resolution 0.15 \
  --obstacle-height 0.35 \
  --inflate-radius 0.10 \
  --min-area 0.25
```

Use `--dry-run` to inspect both commands and all planned outputs without
creating the output directory. The individual commands below remain available
for rerunning or debugging either stage independently.

## 1. Align the point cloud

Run `align_pointcloud_to_map_frame.py` from the directory where its outputs
should be written. The script always writes these filenames into the current
working directory:

- `transformed.pcd`
- `estimated_transform.json`
- `transform_debug.png`

For example:

```bash
mkdir -p /tmp/dorsett_alignment
cd /tmp/dorsett_alignment

python3 "$SPOT_NAV/scripts/align_pointcloud_to_map_frame.py" \
  /path/to/raw_dorsett.pcd
```

In the top-down window:

1. Click the desired local origin.
2. Click a second point in the desired positive-X direction.

The script estimates the floor height, transforms the point cloud into the
selected frame, and opens an Open3D quality-control view. Confirm that the axes
and floor alignment are correct before using the result.

Before line detection and alignment, the script applies two conservative
Open3D filters by default:

1. Statistical outlier removal rejects points whose neighborhood distance is
   unusually large (`20` neighbors and a `2.0` standard-deviation ratio).
2. Radius outlier removal rejects points without at least `3` nearby samples
   inside `0.30 m`.

The console reports the points removed by each pass, and the same counts and
parameters are recorded under `noise_filter` in `estimated_transform.json`.
Lowering `--statistical-std-ratio`, increasing
`--radius-outlier-neighbors`, or reducing `--radius-outlier-radius` makes the
filter more aggressive, but can erase valid sparse walls and distant geometry.
Use `--no-noise-filter` to reproduce the previous downsampling-only behavior.

These geometric filters remove isolated and weakly supported scan noise. Dense,
coherent vehicles, people, and other temporary objects usually remain because
their local point neighborhoods resemble real fixed geometry; those still need
manual, semantic, or temporal cleanup.

Useful tuning options:

```bash
python3 "$SPOT_NAV/scripts/align_pointcloud_to_map_frame.py" \
  /path/to/raw_dorsett.pcd \
  --voxel-size 0.05 \
  --statistical-neighbors 20 \
  --statistical-std-ratio 2.0 \
  --radius-outlier-neighbors 3 \
  --radius-outlier-radius 0.30 \
  --ransac-threshold 0.08 \
  --max-lines 16
```

Use `--no-show-3d-qc` to skip the final Open3D view. The top-down origin and
positive-X selection window is still required.

After reviewing `transform_debug.png`, place the aligned cloud in the package
map directory:

```bash
cp /tmp/dorsett_alignment/transformed.pcd "$SPOT_NAV/map/dorsett_transformed.pcd"
```

## 2. Build the FAR prior map

Run the prior-map builder on the aligned PCD:

```bash
cd "$SPOT_NAV"
python3 scripts/build_far_prior_map.py map/dorsett_transformed.pcd
```

With no additional options, the script selects the largest connected free-space
component and writes:

- `map/dorsett_transformed.vgh`
- `map/dorsett_transformed_boundary.ply`
- `map/dorsett_transformed_trajectory.txt`
- `map/dorsett_transformed_boundary_stats.json`
- `map/dorsett_transformed_boundary_preview.png`

Open `dorsett_transformed_boundary_preview.png` and verify that:

- The green outer boundary encloses the traversable map.
- Red contours follow walls and obstacles.
- The blue free point lies in the intended traversable region.

For maps with multiple disconnected areas, specify a known traversable point
instead of relying on automatic component selection:

```bash
python3 scripts/build_far_prior_map.py map/dorsett_transformed.pcd \
  --free-point X Y Z
```

Common tuning options include:

```bash
python3 scripts/build_far_prior_map.py map/dorsett_transformed.pcd \
  --resolution 0.15 \
  --obstacle-height 0.35 \
  --inflate-radius 0.10 \
  --min-area 0.25
```

The command replaces prior outputs with the same map name. Rebuild the package
after changing map assets so ROS package-share paths receive the new files:

```bash
cd ~/spot_ws
colcon build --packages-select spot_navigation
source install/setup.bash
```

## 3. Record a mission in RViz

Launch the recorder with the aligned PCD, generated boundary, and desired raw
mission filename:

```bash
ros2 launch spot_navigation mission_recorder.launch.py \
  pcd_file:="$SPOT_NAV/map/dorsett_transformed.pcd" \
  boundary_file:="$SPOT_NAV/map/dorsett_transformed_boundary.ply" \
  output_mission_file:="$SPOT_NAV/map/dorsett_mission.yaml"
```

RViz starts by default. In RViz:

1. Use **2D Pose Estimate** once to set the mission's initial pose.
2. Use **2D Goal Pose** repeatedly to add waypoints in route order.
3. Press `Ctrl+C` in the launch terminal when the route is complete.

On shutdown, the recorder writes:

- `map/dorsett_mission.yaml`
- `map/dorsett_mission.png`

The recorder rejects goals submitted before the initial pose and poses whose
frame does not match `map`. It writes nothing unless it captured an initial pose
and at least one waypoint.

The recorder only writes the manually selected goals and a PCD-based preview.
It does **not** run `mission_compiler` automatically.

To choose a different preview path or run without RViz:

```bash
ros2 launch spot_navigation mission_recorder.launch.py \
  pcd_file:="$SPOT_NAV/map/dorsett_transformed.pcd" \
  boundary_file:="$SPOT_NAV/map/dorsett_transformed_boundary.ply" \
  output_mission_file:="$SPOT_NAV/map/dorsett_mission.yaml" \
  output_preview_file:="$SPOT_NAV/map/dorsett_mission_raw.png" \
  rviz:=false
```

## 4. Compile and validate the mission

Compile the raw mission against the generated VGH and the FAR parameters used
by this package:

```bash
ros2 run spot_navigation mission_compiler \
  --mission "$SPOT_NAV/map/dorsett_mission.yaml" \
  --pcd "$SPOT_NAV/map/dorsett_transformed.pcd" \
  --vgh "$SPOT_NAV/map/dorsett_transformed.vgh" \
  --far-config "$SPOT_NAV/config/far_planner.yaml"
```

The default outputs are:

- `map/dorsett_mission_compiled.yaml`
- `map/dorsett_mission_compiled.report.json`
- `map/dorsett_mission_compiled.png`

The compiler checks the recorded route against the serialized FAR prior graph,
inserts intermediate waypoints where needed, and renders a preview containing
the PCD, VGH visibility graph, boundary contours, and compiled route.

Existing output files are preserved by default. To regenerate them:

```bash
ros2 run spot_navigation mission_compiler \
  --mission "$SPOT_NAV/map/dorsett_mission.yaml" \
  --pcd "$SPOT_NAV/map/dorsett_transformed.pcd" \
  --vgh "$SPOT_NAV/map/dorsett_transformed.vgh" \
  --far-config "$SPOT_NAV/config/far_planner.yaml" \
  --force
```

To control waypoint density explicitly, add
`--max-goal-spacing DISTANCE_METERS`. Without this option, the compiler uses
FAR's configured `local_planner_range`.

Review the JSON report and compiled PNG before deploying the mission. Static
validation checks the available prior graph and FAR endpoint criteria, but it
cannot guarantee live success under changed obstacles, localization error, or
different runtime sensor data.

