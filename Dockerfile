ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop-full

ARG ROS_DISTRO
ARG USERNAME=rosuser
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Install every direct system/Python dependency used by the ROS nodes and the
# point-cloud/mission-authoring tools. Custom navigation packages such as FAR
# Planner and FAST-LIO remain workspace source dependencies.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        git \
        libgl1-mesa-dri \
        libglx-mesa0 \
        mesa-utils \
        python3-colcon-common-extensions \
        python3-matplotlib \
        python3-numpy \
        python3-opencv \
        python3-open3d \
        python3-pip \
        python3-pytest \
        python3-serial \
        python3-tk \
        python3-yaml \
        ros-${ROS_DISTRO}-ament-copyright \
        ros-${ROS_DISTRO}-ament-flake8 \
        ros-${ROS_DISTRO}-ament-pep257 \
        ros-${ROS_DISTRO}-pcl-ros \
        ros-${ROS_DISTRO}-rosbag2-storage-mcap \
        ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-velodyne-driver \
        ros-${ROS_DISTRO}-velodyne-pointcloud \
        ros-${ROS_DISTRO}-visualization-msgs \
        xauth \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir "pypcd4==1.4.3" && \
#    source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
#    python3 -c "import cv2, matplotlib, numpy, open3d, pypcd4, rclpy, rosbag2_py, serial, tkinter, yaml; from geometry_msgs.msg import Pose; from visualization_msgs.msg import Marker; print('spot_navigation dependencies ready')"
RUN groupadd --gid "${USER_GID}" "${USERNAME}" && \
    useradd --uid "${USER_UID}" --gid "${USER_GID}" --create-home --shell /bin/bash "${USERNAME}" && \
    usermod --append --groups dialout,plugdev,video "${USERNAME}"

ENV SPOT_WS=/home/${USERNAME}/spot_ws
WORKDIR ${SPOT_WS}

COPY --chown=${USER_UID}:${USER_GID} . src/spot_navigation
COPY --chmod=0755 docker/ros_entrypoint.sh /spot_navigation_entrypoint.sh

RUN chown "${USER_UID}:${USER_GID}" "${SPOT_WS}"

USER ${USERNAME}

RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    colcon build --symlink-install --packages-select spot_navigation && \
    source "${SPOT_WS}/install/setup.bash" && \
    python3 -c "import spot_navigation.boundary_marker_publisher, spot_navigation.mission_compiler, spot_navigation.mission_recorder, spot_navigation.radio_bridge, spot_navigation.route_manager; print('spot_navigation package ready')"

RUN printf '%s\n' \
        "source /opt/ros/${ROS_DISTRO}/setup.bash" \
        "source ${SPOT_WS}/install/setup.bash" \
        >> "/home/${USERNAME}/.bashrc"

ENTRYPOINT ["/spot_navigation_entrypoint.sh"]
CMD ["bash"]
