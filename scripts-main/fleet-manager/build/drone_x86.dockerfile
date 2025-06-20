# This dockerfile can only be used to build the drone docker image on a x86/x64 system due to MAVSDK not releasing
# binaries for ARM architectures. As such, this should only be used for simulation purposes, as the dockerfile present
# in build/drone_sbc should be used to build the software on the drone's SBC instead.

FROM fleetman/ros:galactic

# Define shell, create workspace dir
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

# Install MAVSDK
RUN wget https://github.com/mavlink/MAVSDK/releases/download/v0.40.0/mavsdk_0.40.0_ubuntu20.04_amd64.deb && \
    dpkg -i mavsdk_0.40.0_ubuntu20.04_amd64.deb && \
    rm -rf mavsdk_0.40.0_ubuntu20.04_amd64.deb && \
    # Colcon cannot find the link to mavsdk without a file ending in .so without further versioning
    for f in /lib/libmavsdk*.so.0 ; do ln -s "$f" "${f/.0/}" ; done

# Copy source files, scripts and configs
COPY scripts/run_drone_controller.sh $WS_DIR
COPY configs/drone_*  $WS_DIR/
COPY cmd/drone $WS_DIR/src/drone

# Build drone code and update bashrc
RUN cd  $WS_DIR && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build && \
    echo "source  $WS_DIR/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash", "-c", "source install/setup.bash && ./run_drone_controller.sh $DRONE_ID $CFG_FILE"]
