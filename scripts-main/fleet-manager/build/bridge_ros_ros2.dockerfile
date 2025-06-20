FROM fleetman/ros:galactic

# Define shell, workspace dir and create base directories
SHELL ["/bin/bash", "-c"]
WORKDIR /root
RUN mkdir -p /root/ros1_ws/src
RUN mkdir -p /root/ros2_ws/src
RUN mkdir -p /root/bridge_ws/src

# Install ROS1 noetic
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt update && \
    apt install -y ros-noetic-ros-base && \
    rm -rf /var/lib/apt/lists/*

# Copy message and service definitions
COPY vendor/friends/ /root/ros1_ws/src/
COPY pkg/drone_interfaces /root/ros2_ws/src/drone_interfaces
COPY pkg/friends_interfaces /root/ros2_ws/src/friends_interfaces

# Build ROS1 and ROS2 message packages and build ros bridge
RUN cd /root/ros2_ws && \
    . /opt/ros/galactic/setup.bash && \
    colcon build && \
    cd /root/ros1_ws && \
    . /opt/ros/noetic/setup.bash && \
    catkin_make friends_ros_generate_messages && \
    cd /root/bridge_ws/src && \
    git clone https://github.com/ros2/ros1_bridge.git && \
    cd ros1_bridge && \
    git checkout galactic && \
    cd /root/bridge_ws && \
    . /opt/ros/galactic/setup.bash && \
    . /root/ros1_ws/devel/setup.bash && \
    . /root/ros2_ws/install/setup.bash && \
    colcon build --catkin-skip-building-tests --cmake-args -DBUILD_TESTING=OFF && \
    rm -rf build && \
    echo source /root/bridge_ws/install/setup.bash >> /root/.bashrc

# Whether to start roscore when launching the ros bridge script
ENV START_ROSCORE=true
# Default ROS_MASTER_URI is local
ENV ROS_MASTER_URI=http://localhost:11311
COPY scripts/run_ros1_bridge.sh /root/run_ros1_bridge.sh

CMD ["./run_ros1_bridge.sh"]
