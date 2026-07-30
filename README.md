# Spot Navigation Map and Mission Workflow

The operator workflow has only two steps:

1. Run [`scripts/prepare_map.py`](scripts/prepare_map.py) to align and filter the
   raw point cloud and generate the FAR map assets.
2. Run `mission_recorder.launch.py` to open those assets in RViz and record the
   initial pose and ordered mission waypoints.

The exact commands are in [`scripts/README.md`](scripts/README.md).

`prepare_map.py` produces the transformed PCD, transform metadata, FAR boundary,
visibility graph, statistics, and preview without modifying the raw input.

`mission_recorder.launch.py` displays the transformed PCD and boundary in RViz.
Use **2D Pose Estimate** once, use **2D Goal Pose** for each waypoint, and press
`Ctrl+C` when finished to write the mission YAML and PNG preview.
