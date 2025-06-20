# This dockerfile should be used to install the drone controller software in the drone's SBC (jetson nano, rpi, etc...)
# The difference is that MAVSDK does not provide binaries for ARM architectures, and as such it would have to be locally
# compiled, which would take a long time on those devices.
# The generate_mavsdk_binaries.sh script should be ran in one of the devices and the output files stored to be distributed
# among the remaining devices.
# This dockerfile depends on the availability of those files on the /vendor/mavsdk directory (you should have a
# vendor/mavsdk/usr/local directory)

FROM fleetman/ros:galactic

# Define shell, create workspace dir, define ROS domain ID
SHELL ["/bin/bash", "-c"]
ARG WS_DIR=/ws
WORKDIR $WS_DIR
RUN mkdir -p $WS_DIR/src

# Install required/useful packages
RUN apt update && \
    apt upgrade -y && \
    apt install libjsoncpp-dev iproute2 -y && \
    apt install libgeos++-dev -y && \
    rm -rf /var/lib/apt/lists/*

# MAVSDK binaries should be available in this directory
COPY vendor/mavsdk /
# Create symlink for libmavsdk
RUN for f in /usr/local/lib/libmavsdk* ; do ln -s "$f" "${f/.so*/.so}"; done && ldconfig

# Copy source files, scripts and configs
COPY scripts/run_drone_controller.sh $WS_DIR
COPY configs/drone_*  $WS_DIR/
COPY cmd/drone $WS_DIR/src/drone

# Build drone code and update bashrc
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    cd $WS_DIR && \
    colcon build && \
    echo "source  $WS_DIR/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash", "-c", "source install/setup.bash && ./run_drone_controller.sh $DRONE_ID $CFG_FILE"]
