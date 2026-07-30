# Spot Navigation Map and Mission Workflow

This workflow converts a raw E57 point cloud into an aligned PCD and FAR prior
map, then opens the result in RViz to record an initial pose and ordered mission
waypoints. All generated files remain in the host workspace.

## Prerequisites

- WSL2 with WSLg
- Docker with NVIDIA GPU support
- This repository at `<workspace>/src/spot_navigation`
- The raw `dorsett.e57` file at `<workspace>/dorsett.e57`

Replace `/path/to` below with the actual workspace path.

## 1. Build the Docker image

Open WSL and enter the package directory:

```bash
cd /path/to/src/spot_navigation
```

Build the image:

```bash
docker build -t spot_mission_planning .
```

This build is needed once initially. Rebuild the image after changing the
Dockerfile or package source because the ROS package is built into the image.

## 2. Start the container

Move from `src/spot_navigation` to the workspace root:

```bash
cd ../..
```

Start an interactive container with WSLg, NVIDIA GPU access, and the workspace
mounted read-write at the same absolute path:

```bash
docker run --rm -it \
  --gpus all \
  --device=/dev/dxg \
  -v /usr/lib/wsl:/usr/lib/wsl:ro \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /mnt/wslg:/mnt/wslg:rw \
  -v "$PWD:$PWD:rw" \
  --workdir "$PWD" \
  -e DISPLAY="$DISPLAY" \
  -e WAYLAND_DISPLAY="$WAYLAND_DISPLAY" \
  -e XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
  -e PULSE_SERVER="$PULSE_SERVER" \
  -e LD_LIBRARY_PATH=/usr/lib/wsl/lib \
  spot_mission_planning
```

Run all remaining commands inside this container shell from the mounted
workspace root.

## 3. Convert the E57 point cloud to PCD

```bash
python3 src/spot_navigation/scripts/e57_to_pcd.py dorsett.e57
```

The converter downsamples the first E57 scan with a 0.1 m voxel and writes
`dorsett.pcd` beside the input file.

## 4. Copy the PCD into the package map directory

```bash
cp dorsett.pcd src/spot_navigation/map/dorsett.pcd
```

## 5. Align the map and generate FAR assets

```bash
python3 src/spot_navigation/scripts/prepare_map.py \
  src/spot_navigation/map/dorsett.pcd \
  --output-dir src/spot_navigation/map \
  --name dorsett
```

In the alignment window, click the map origin and then a point in the positive-X
direction. Review the alignment view and boundary preview before continuing.
The raw input PCD is not modified.

The command writes:

```text
src/spot_navigation/map/dorsett_transformed.pcd
src/spot_navigation/map/dorsett_transform.json
src/spot_navigation/map/dorsett_transform_debug.png
src/spot_navigation/map/dorsett_transformed.vgh
src/spot_navigation/map/dorsett_transformed_boundary.ply
src/spot_navigation/map/dorsett_transformed_trajectory.txt
src/spot_navigation/map/dorsett_transformed_boundary_stats.json
src/spot_navigation/map/dorsett_transformed_boundary_preview.png
```

## 6. Record the mission in RViz

```bash
ros2 launch spot_navigation mission_recorder.launch.py \
  pcd_file:=src/spot_navigation/map/dorsett_transformed.pcd \
  boundary_file:=src/spot_navigation/map/dorsett_transformed_boundary.ply \
  output_mission_file:=src/spot_navigation/map/dorsett_mission.yaml
```

In RViz:

1. Use **2D Pose Estimate** once to set the initial pose.
2. Use **2D Goal Pose** to add waypoints in route order.
3. Press `Ctrl+C` in the launch terminal when finished.

Accepted poses remain visible as `START`, `W1`, `W2`, and so on, with arrows
showing their headings and a line showing mission order.

The recorder writes directly to the mounted host workspace:

```text
src/spot_navigation/map/dorsett_mission.yaml
src/spot_navigation/map/dorsett_mission.png
```

For later sessions, repeat from step 2. Rebuild the image only when the
Dockerfile or package source changes.
