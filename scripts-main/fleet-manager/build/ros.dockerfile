FROM ubuntu:focal
ENV ROS_DOMAIN_ID=128
ENV ROS_DISTRO=galactic

# Install base dependencies
RUN apt update && apt upgrade -y && \
    apt install -y \
        curl \
        gnupg \
        lsb-release \
        build-essential \
        git \
        vim \
        wget && \
        rm -rf /var/lib/apt/lists/*

# Install ROS2 and FastRTPS
# As of 02/06/2021, FastRTPS was used instead of the default Cyclone DDS because it
# supported connecting ROS2 to multiple network interfaces simultaneously, which Cyclone
# did not allow. If this changes in the future, the default DDS should be used instead.
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt update && apt-get --fix-missing install -y && apt upgrade -y && DEBIAN_FRONTEND=noninteractive apt install \
    ros-$ROS_DISTRO-ros-base \
    ros-$ROS_DISTRO-rmw-fastrtps-cpp \
    python3-colcon-common-extensions -y && \
    rm -rf /var/lib/apt/lists/*

# Add source to bashrc
# As of 02/06/2021, it was necessary to use this custom configuration for FastRTPS to disable shared
# memory transport, as it would not work properly using different containers on the same machine.
# See: https://answers.ros.org/question/370595/ros2-foxy-nodes-cant-communicate-through-docker-container-border
# If this changes in the future, this configuration should be removed.
RUN echo source /opt/ros/$ROS_DISTRO/setup.bash >> /root/.bashrc && \
    echo '<?xml version="1.0" encoding="UTF-8" ?> \
  <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles" > \
      <transport_descriptors> \
          <transport_descriptor> \
              <transport_id>CustomUdpTransport</transport_id> \
              <type>UDPv4</type> \
          </transport_descriptor> \
      </transport_descriptors> \
      <participant profile_name="participant_profile" is_default_profile="true"> \
          <rtps> \
              <userTransports> \
                  <transport_id>CustomUdpTransport</transport_id> \
              </userTransports> \
              <useBuiltinTransports>false</useBuiltinTransports> \
          </rtps> \
      </participant> \
  </profiles>' \
  >> /root/DEFAULT_FASTRTPS_PROFILES.xml

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/root/DEFAULT_FASTRTPS_PROFILES.xml