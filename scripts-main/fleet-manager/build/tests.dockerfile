FROM fleetman/ros:galactic

SHELL ["/bin/bash", "-c"]
ARG WS_DIR=/ws
RUN mkdir $WS_DIR
WORKDIR $WS_DIR

# Install basic tools
RUN apt update && apt upgrade -y && \
    apt install -y \
        ros-$ROS_DISTRO-test-msgs \
        python3-pybind11 \
        python3-pip \
        libcairo2-dev \
        libgirepository1.0-dev \
        python3-gi \
        gobject-introspection \
        gir1.2-gtk-3.0 \
        iproute2 \
        iw \
        iputils-ping && \
    rm -rf /var/lib/apt/lists/*

# Install required python dependencies
RUN pip3 install gmplot haversine matplotlib paho-mqtt PyGObject

# Copy tools and source files
COPY tools/sensors sensors
COPY tools/log_analysis log_analysis
COPY tools/yet_another_drone_dashboard yet_another_drone_dashboard
COPY pkg/drone_interfaces src/drone_interfaces
COPY pkg/friends_interfaces src/friends_interfaces

# Build drone and friends interfaces
RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build && echo "source $WS_DIR/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
