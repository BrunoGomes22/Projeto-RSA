FROM ubuntu:focal

ARG WS_DIR=/home
WORKDIR $WS_DIR

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && \
    apt install curl git cmake python3-pip -y && \
    apt-get install -y \
        lsb-release \
        sudo \
        wget && \
    rm -rf /var/lib/apt/lists/*

RUN cd $WS_DIR && \
    pip uninstall em && \
    # freeze empy version
    pip install empy==3.3.4 && \
    git clone https://github.com/PX4/Firmware.git --recursive --branch v1.10.1 --depth 1 && \
    sed -i -- s/50.0f/38.0f/g Firmware/src/modules/simulator/simulator_params.c && \
    sed -i -- s/60/180/g Firmware/src/modules/simulator/simulator_params.c && \
    # freeze empy version
    pip install empy==3.3.4 && \
    ./Firmware/Tools/setup/ubuntu.sh --no-nuttx && \
    cd Firmware && HEADLESS=1 make px4_sitl jmavsim && \
    # Only keep necessary files in order to maintain a smaller docker image size
    cd $WS_DIR && \
    mkdir -p build/px4_sitl_default && \
    mkdir tools && \
    mv Firmware/ROMFS . && \
    mv Firmware/build/px4_sitl_default/bin build/px4_sitl_default/ && \
    mv Firmware/Tools/sitl_multiple_run.sh tools && \
    mv Firmware/Tools/jmavsim_run.sh tools && \
    mv Firmware/Tools/jMAVSim tools && \
    rm -rf Firmware && \
    # Clear apt cache
    rm -rf /var/lib/apt/lists/*

CMD ["/bin/bash"]
