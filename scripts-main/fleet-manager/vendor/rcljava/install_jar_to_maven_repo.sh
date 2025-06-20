#!/bin/bash
# Install rcljava, rcljava_common, std_messages and example_interfaces_messages to Maven local repository
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
mvn install:install-file \
	-Dfile=$SCRIPT_DIR/rcljava.jar \
	-DgroupId=org.ros2 \
	-DartifactId=rcljava \
	-Dversion=1.0 \
	-Dpackaging=jar \
	-DgeneratePom=true \
&&
mvn install:install-file \
   -Dfile=$SCRIPT_DIR/rcljava_common.jar \
   -DgroupId=org.ros2 \
   -DartifactId=rcljava_common \
   -Dversion=1.0 \
   -Dpackaging=jar \
   -DgeneratePom=true \
&&
mvn install:install-file \
	-Dfile=$SCRIPT_DIR/std_msgs_messages.jar \
	-DgroupId=std_msgs \
	-DartifactId=msg \
	-Dversion=1.0 \
	-Dpackaging=jar \
	-DgeneratePom=true \
&&
mvn install:install-file \
	-Dfile=$SCRIPT_DIR/rcl_interfaces_messages.jar \
	-DgroupId=rcl_interfaces \
	-DartifactId=srv \
	-Dversion=1.0 \
	-Dpackaging=jar \
	-DgeneratePom=true \
&&
mvn install:install-file \
	-Dfile=$SCRIPT_DIR/rcl_interfaces_messages.jar \
	-DgroupId=rcl_interfaces \
	-DartifactId=msg \
	-Dversion=1.0 \
	-Dpackaging=jar \
	-DgeneratePom=true \
&&
mvn install:install-file \
	-Dfile=$SCRIPT_DIR/builtin_interfaces_messages.jar \
	-DgroupId=builtin_interfaces \
	-DartifactId=msg \
	-Dversion=1.0 \
	-Dpackaging=jar \
	-DgeneratePom=true \
&&
mvn install:install-file \
	-Dfile=$SCRIPT_DIR/drone_interfaces_messages.jar \
	-DgroupId=drone_interfaces \
	-DartifactId=msg \
	-Dversion=1.0 \
	-Dpackaging=jar \
	-DgeneratePom=true \
&&
mvn install:install-file \
	-Dfile=$SCRIPT_DIR/friends_interfaces_messages.jar \
	-DgroupId=friends_interfaces \
	-DartifactId=msg \
	-Dversion=1.0 \
	-Dpackaging=jar \
	-DgeneratePom=true \
&&
mvn install:install-file \
	-Dfile=$SCRIPT_DIR/friends_interfaces_messages.jar \
	-DgroupId=friends_interfaces \
	-DartifactId=srv \
	-Dversion=1.0 \
	-Dpackaging=jar \
	-DgeneratePom=true

# These JARs were retrieved by building rcljava on a ROS2 environment and copying
# the generated JARs from the installation directory.