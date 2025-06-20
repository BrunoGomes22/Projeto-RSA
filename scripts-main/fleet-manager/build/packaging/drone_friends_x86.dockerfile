# This dockerfile should be used to install the drone controller software on a x86/x64 system for simulation purposes
FROM fleetman/ros:galactic

# Define shell, create workspace dir, define ROS domain ID
SHELL ["/bin/bash", "-c"]
ARG WS_DIR=/ws
WORKDIR $WS_DIR
RUN mkdir -p $WS_DIR/src

# Install required packages
RUN apt update && \
    apt upgrade -y && \
    apt install libjsoncpp-dev wget -y && \
    rm -rf /var/lib/apt/lists/*

# Install MAVSDK
RUN wget https://github.com/mavlink/MAVSDK/releases/download/v0.40.0/mavsdk_0.40.0_ubuntu20.04_amd64.deb && \
    dpkg -i mavsdk_0.40.0_ubuntu20.04_amd64.deb && \
    rm -rf mavsdk_0.40.0_ubuntu20.04_amd64.deb && \
    # Colcon cannot find the link to mavsdk without a file ending in .so without further versioning
    for f in /lib/libmavsdk*.so.0 ; do ln -s "$f" "${f/.0/}" ; done

# Copy executables, scripts and configs
COPY build/install /$WS_DIR/install
COPY build/run_drone_controller.sh /$WS_DIR
COPY configs/drone_*  $WS_DIR/

# Update bashrc
RUN cd $WS_DIR && echo "source  $WS_DIR/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash", "-c", "source install/setup.bash && ./run_drone_controller.sh $DRONE_ID $CFG_FILE"]
