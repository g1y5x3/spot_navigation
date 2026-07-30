# Spot Navigation Map and Mission Workflow

The operator workflow has only two steps:

1. Run [`scripts/prepare_map.py`](scripts/prepare_map.py) to align and filter the
   raw point cloud and generate the FAR map assets.
2. Run `mission_recorder.launch.py` to open those assets in RViz and record the
   initial pose and ordered mission waypoints.

The native-workspace commands are in [`scripts/README.md`](scripts/README.md).

`prepare_map.py` produces the transformed PCD, transform metadata, FAR boundary,
visibility graph, statistics, and preview without modifying the raw input.

`mission_recorder.launch.py` displays the transformed PCD and boundary in RViz.
Use **2D Pose Estimate** once, use **2D Goal Pose** for each waypoint, and press
`Ctrl+C` when finished to write the mission YAML and PNG preview.

## Run the mission recorder in WSL2

On a WSL2 machine with WSLg, NVIDIA Container Toolkit, and the
`spot-navigation:humble` image, launch the mission recorder with GPU and GUI
support:

```bash
docker run --rm -it \
  --gpus all \
  --device=/dev/dxg \
  -v /usr/lib/wsl:/usr/lib/wsl:ro \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /mnt/wslg:/mnt/wslg:rw \
  -v "$HOME/mission_ws/src/spot_navigation/map:/maps:rw" \
  -e DISPLAY="$DISPLAY" \
  -e WAYLAND_DISPLAY="$WAYLAND_DISPLAY" \
  -e XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
  -e PULSE_SERVER="$PULSE_SERVER" \
  -e LD_LIBRARY_PATH=/usr/lib/wsl/lib \
  spot-navigation:humble \
  ros2 launch spot_navigation mission_recorder.launch.py \
    pcd_file:=/maps/dorsett_transformed.pcd \
    boundary_file:=/maps/dorsett_transformed_boundary.ply \
    output_mission_file:=/maps/dorsett_mission.yaml
```

The map directory is mounted read-write at `/maps`, so the mission YAML and PNG
preview are written back to the host map directory.
