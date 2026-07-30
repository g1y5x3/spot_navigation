# Run Map Preparation and Mission Recording

Create a ROS 2 workspace and place this repository at
`~/mission_ws/src/spot_navigation`:

```bash
mkdir -p ~/mission_ws/src
cd ~/mission_ws/src
# Clone or copy the spot_navigation repository into this directory.
```

Place the raw point cloud at
`~/mission_ws/src/spot_navigation/map/dorsett.pcd`, then build and source the
workspace:

```bash
cd ~/mission_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select spot_navigation
source install/setup.bash
```

## 1. Run `prepare_map.py`

```bash
python3 ~/mission_ws/src/spot_navigation/scripts/prepare_map.py \
  ~/mission_ws/src/spot_navigation/map/dorsett.pcd \
  --output-dir ~/mission_ws/src/spot_navigation/map \
  --name dorsett
```

In the alignment window, click the map origin and then a point in the positive-X
direction. Review the alignment view and generated boundary preview before
continuing.

The main outputs are:

```text
map/dorsett_transformed.pcd
map/dorsett_transformed.vgh
map/dorsett_transformed_boundary.ply
map/dorsett_transformed_boundary_preview.png
```

## 2. Run `mission_recorder.launch.py`

```bash
ros2 launch spot_navigation mission_recorder.launch.py \
  pcd_file:="$HOME/mission_ws/src/spot_navigation/map/dorsett_transformed.pcd" \
  boundary_file:="$HOME/mission_ws/src/spot_navigation/map/dorsett_transformed_boundary.ply" \
  output_mission_file:="$HOME/mission_ws/src/spot_navigation/map/dorsett_mission.yaml"
```

In RViz:

1. Use **2D Pose Estimate** once to set the initial pose.
2. Use **2D Goal Pose** to add waypoints in route order.
3. Press `Ctrl+C` in the launch terminal when finished.

The recorder writes:

```text
map/dorsett_mission.yaml
map/dorsett_mission.png
```
