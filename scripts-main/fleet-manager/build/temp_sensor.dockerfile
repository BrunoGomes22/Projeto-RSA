FROM fleetman/ros:galactic

# Install dependencies
RUN apt update && \
    apt install -y \
      python3-pip

# Install pip requirements
RUN pip3 install \
        matplotlib \
        haversine 

# Install rclpy
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    pip3 install -U ros2pkg && \
    pip3 install -U ament_index_python && \
    pip3 install -U rclpy

# Copy source files
COPY tools/sensors sensors

# Set the working directory
WORKDIR /sensors

# Set the entrypoint command
CMD ["/bin/bash", "-c", ". /opt/ros/$ROS_DISTRO/setup.sh && python3 temperature.py -c configs/single_zone_fire_cfg.yml"]