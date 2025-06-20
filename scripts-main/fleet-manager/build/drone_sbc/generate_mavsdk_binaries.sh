#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

if [ $# -eq 0 ]
  then
  	OUTPUT_DIR=$SCRIPT_DIR/../../vendor
else
	if [[ -d $1 ]]; then
		OUTPUT_DIR=$1
	else
		echo "Error: '$1' is not a directory."
		exit
	fi
fi

mkdir -p $OUTPUT_DIR/mavsdk/usr/local/lib
mkdir -p $OUTPUT_DIR/mavsdk/usr/local/include/mavsdk
docker run -itd --rm --name tmp_drone ubuntu:focal bash
docker exec tmp_drone \
	sh -c "apt update && \
	DEBIAN_FRONTEND=noniteractive apt install -y git build-essential cmake libtinyxml2-dev && \
	git clone https://github.com/mavlink/MAVSDK.git && \
	cd MAVSDK && \
	git checkout v0.40.0 && \
	git submodule update --init --recursive && \
	cmake -DCMAKE_BUILD_TYPE=Debug -DBUILD_SHARED_LIBS=ON -Bbuild/default -H. && \
	cmake --build build/default --target install && \
	ldconfig"
docker cp tmp_drone:/usr/local/lib/. $OUTPUT_DIR/mavsdk/usr/local/lib
docker cp tmp_drone:/usr/local/include/mavsdk/. $OUTPUT_DIR/mavsdk/usr/local/include/mavsdk
docker stop tmp_drone
# Remove links
for f in $OUTPUT_DIR/mavsdk/usr/local/lib/libmavsdk*.so.0 ; do rm $f ; done
for f in $OUTPUT_DIR/mavsdk/usr/local/lib/libmavsdk*.so ; do rm $f ; done

