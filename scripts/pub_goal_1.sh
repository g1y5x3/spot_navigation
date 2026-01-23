#!/bin/bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 12.449, y: -11.669, z: 0.236}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
