FROM fleetman/rcljava:galactic

# Install basic tools
RUN apt update && apt upgrade -y && apt install -y ntp && rm -rf /var/lib/apt/lists/*

# Allow ntp server to run without internet access
RUN printf "server 127.127.1.0 prefer\nfudge  127.127.1.0\n" >> /etc/ntp.conf

# Set workspace dir
SHELL ["/bin/bash", "-c"]
ARG WS_DIR=/ws
RUN mkdir -p $WS_DIR/src
WORKDIR $WS_DIR

# Copy source files and scripts
COPY scripts/run_fleetman_server.sh $WS_DIR
COPY scripts/fleetman_aux_dev/* $WS_DIR
COPY pkg/drone_interfaces $WS_DIR/src/drone_interfaces
COPY pkg/friends_interfaces $WS_DIR/src/friends_interfaces
COPY configs/sensors $WS_DIR/sensors
COPY configs/plugins/ $WS_DIR/plugins
COPY cmd/server/ $WS_DIR/src/server

# Build fleet manager server
RUN cd $WS_DIR && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    source /root/ros2_java_ws/install/setup.bash && \
    colcon build && \
    gradle -b src/server/build.gradle fetchDeps && \
    rm -rf /root/.gradle && \
    echo "source $WS_DIR/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash", "-c", "service ntp start && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    source /root/ros2_java_ws/install/setup.bash && \
    source /ws/install/setup.bash && \
    chmod +x run_fleetman_server.sh && ./run_fleetman_server.sh"]
