FROM fleetman/ros:galactic

# Install dependencies, gradle, and jdk
RUN export DEBIAN_FRONTEND=noninteractive && \
    apt update && \
    apt install -y \
      python3-pip \
      python3-rosdep \
      python3-vcstool \
      gradle \
      default-jdk && \
    apt clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Colcon gradle support
RUN python3 -m pip install -U git+https://github.com/colcon/colcon-gradle
RUN python3 -m pip install --no-deps -U git+https://github.com/colcon/colcon-ros-gradle@0.1.0

RUN rm -f rcljava.repos
ADD rcljava.repos .
RUN ls -a
RUN mkdir -p ~/ros2_java_ws/src && \
    ls -a && \
    cp rcljava.repos ~/ros2_java_ws && \
    cd ~/ros2_java_ws && \
# Fetch code and install dependencies with rosdep
    #curl -skL https://raw.githubusercontent.com/osrf/ros2_java/galactic-devel/ros2_java_desktop.repos -o rcljava.repos && \
    vcs import src < rcljava.repos && \
    apt update -qq && \
    rosdep init && \
    rosdep update && \
    RTI_NC_LICENSE_ACCEPTED=yes rosdep install --from-paths src --ignore-src -r --rosdistro $ROS_DISTRO -y --skip-keys "console_bridge fastcdr fastrtps urdfdom_headers rosidl_typesupport_java" && \
    apt clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* && \
    rm -rf /root/.ros

# Build rcljava
RUN cd ~/ros2_java_ws/src/ros2_java/ros2_java && \
    git checkout 09c0e681911dcdb15f70ee92e22b77a45199e8ab && \
    cd ~/ros2_java_ws && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    #colcon build --help && \
    colcon build --packages-up-to rcljava --allow-overriding action_msgs builtin_interfaces rcl_interfaces rosgraph_msgs rosidl_default_generators rosidl_default_runtime std_msgs unique_identifier_msgs && \
    echo "source /root/ros2_java_ws/install/setup.bash" >> /root/.bashrc && \
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc