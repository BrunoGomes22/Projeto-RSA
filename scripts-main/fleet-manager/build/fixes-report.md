# build_docker_images.sh

## Table of Contents
1. [drone](#drone)
2. [ground](#ground)
3. [simulator](#simulator)

## drone

```> ./build_docker_images.sh drone``` 

### Issue #1

```
49.59 E: Failed to fetch http://packages.ros.org/ros2/ubuntu/pool/main/r/ros-galactic-rosbag2-storage-default-plugins/ros-galactic-rosbag2-storage-default-plugins_0.9.2-1focal.20221207.122241_amd64.deb  Undetermined Error [IP: 140.211.166.134 80]
49.59 E: Unable to fetch some archives, maybe run apt-get update or try with --fix-missing?
------
ros.dockerfile:21
```

### Fix #1

```ros.dockerfile > line 23```

**From:**
```
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt update && apt upgrade -y && DEBIAN_FRONTEND=noninteractive apt install \
    ros-$ROS_DISTRO-ros-base \
    ros-$ROS_DISTRO-rmw-fastrtps-cpp \
    python3-colcon-common-extensions -y && \
    rm -rf /var/lib/apt/lists/*
```

**To:**

```
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt update && apt-get --fix-missing install -y && apt upgrade -y && DEBIAN_FRONTEND=noninteractive apt install \
    ros-$ROS_DISTRO-ros-base \
    ros-$ROS_DISTRO-rmw-fastrtps-cpp \
    python3-colcon-common-extensions -y && \
    rm -rf /var/lib/apt/lists/*
```

### Comments

- **Issue #1:** Parece ter sido o endereço de uma depedência desatualizado, que foi corrigido aplicando a diretiva --fix-missing como sugerido no erro.

## ground

```> ./build_docker_images.sh ground``` 

### Issue #1

```
1.098 [0.594s] ERROR:colcon:colcon build: Unable to order packages topologically:
1.098 action_msgs: ['builtin_interfaces', 'rosidl_default_generators', 'rosidl_default_runtime']
1.098 actionlib_msgs: ['action_msgs', 'builtin_interfaces', 'rosidl_default_generators', 'rosidl_default_runtime', 'std_msgs']
1.098 builtin_interfaces: ['action_msgs', 'rosidl_default_generators', 'rosidl_default_runtime']
1.098 common_interfaces: ['action_msgs', 'actionlib_msgs', 'builtin_interfaces', 'diagnostic_msgs', 'geometry_msgs', 'nav_msgs', 'rosidl_default_runtime', 'sensor_msgs', 'shape_msgs', 'std_msgs', 'std_srvs', 'stereo_msgs', 'trajectory_msgs', 'visualization_msgs']
1.098 composition_interfaces: ['action_msgs', 'builtin_interfaces', 'rcl_interfaces', 'rosidl_default_generators', 'rosidl_default_runtime']
1.098 diagnostic_msgs: ['action_msgs', 'builtin_interfaces', 'geometry_msgs', 'rosidl_default_generators', 'rosidl_default_runtime', 'std_msgs']
1.098 example_interfaces: ['action_msgs', 'builtin_interfaces', 'rosidl_default_generators', 'rosidl_default_runtime']
1.098 geometry_msgs: ['action_msgs', 'builtin_interfaces', 'rosidl_default_generators', 'rosidl_default_runtime', 'std_msgs']
1.098 lifecycle_msgs: ['action_msgs', 'builtin_interfaces', 'rosidl_default_generators', 'rosidl_default_runtime']
1.098 nav_msgs: ['action_msgs', 'builtin_interfaces', 'geometry_msgs', 'rosidl_default_generators', 'rosidl_default_runtime', 'std_msgs']
1.098 rcl_interfaces: ['action_msgs', 'builtin_interfaces', 'rosidl_default_generators', 'rosidl_default_runtime']
1.098 rcljava: ['action_msgs', 'builtin_interfaces', 'rcl_interfaces', 'rosgraph_msgs', 'rosidl_default_runtime', 'std_msgs', 'test_msgs']
1.098 rcljava_examples: ['action_msgs', 'builtin_interfaces', 'example_interfaces', 'geometry_msgs', 'rcl_interfaces', 'rcljava', 'rosgraph_msgs', 'rosidl_default_runtime', 'sensor_msgs', 'std_msgs']
1.098 rosgraph_msgs: ['action_msgs', 'builtin_interfaces', 'rosidl_default_generators', 'rosidl_default_runtime']
1.098 rosidl_default_generators: ['action_msgs', 'builtin_interfaces', 'rosidl_default_runtime']
1.098 rosidl_default_runtime: ['action_msgs', 'builtin_interfaces']
1.098 sensor_msgs: ['action_msgs', 'builtin_interfaces', 'geometry_msgs', 'rosidl_default_generators', 'rosidl_default_runtime', 'std_msgs']
1.098 sensor_msgs_py: ['action_msgs', 'builtin_interfaces', 'geometry_msgs', 'rosidl_default_runtime', 'sensor_msgs', 'std_msgs']
1.098 shape_msgs: ['action_msgs', 'builtin_interfaces', 'geometry_msgs', 'rosidl_default_generators', 'rosidl_default_runtime', 'std_msgs']
1.098 statistics_msgs: ['action_msgs', 'builtin_interfaces', 'rosidl_default_generators', 'rosidl_default_runtime']
1.098 std_msgs: ['action_msgs', 'builtin_interfaces', 'rosidl_default_generators', 'rosidl_default_runtime']
1.098 std_srvs: ['action_msgs', 'builtin_interfaces', 'rosidl_default_generators', 'rosidl_default_runtime']
1.098 stereo_msgs: ['action_msgs', 'builtin_interfaces', 'geometry_msgs', 'rosidl_default_generators', 'rosidl_default_runtime', 'sensor_msgs', 'std_msgs']
1.098 test_msgs: ['action_msgs', 'builtin_interfaces', 'rosidl_default_generators', 'rosidl_default_runtime']
1.098 trajectory_msgs: ['action_msgs', 'builtin_interfaces', 'geometry_msgs', 'rosidl_default_generators', 'rosidl_default_runtime', 'std_msgs']
1.098 visualization_msgs: ['action_msgs', 'builtin_interfaces', 'geometry_msgs', 'rosidl_default_generators', 'rosidl_default_runtime', 'sensor_msgs', 'std_msgs']
------
rcljava.dockerfile:38
```

### Fix #1:

```rcljava.repos > lines 21, 25```

**From:**
```
ros2/rosidl_defaults:
    type: git
    url: https://github.com/ros2/rosidl_defaults
    version: master
ros2/unique_identifier_msgs:
    type: git
    url: https://github.com/ros2/unique_identifier_msgs
    version: master
```

**To:**
```
ros2/rosidl_defaults:
    type: git
    url: https://github.com/ros2/rosidl_defaults
    version: galactic
ros2/unique_identifier_msgs:
    type: git
    url: https://github.com/ros2/unique_identifier_msgs
    version: galactic
```

### Issue #2

```
1.252 Starting >>> ament_java_resources
1.815 Finished <<< ament_java_resources [0.56s]
1.816 Starting >>> ament_build_type_gradle
2.836 Finished <<< ament_build_type_gradle [1.02s]
2.838 Starting >>> ament_cmake_export_jars
2.849 Starting >>> ament_cmake_export_jni_libraries
3.235 Finished <<< ament_cmake_export_jni_libraries [0.39s]
3.260 Finished <<< ament_cmake_export_jars [0.42s]
3.262 Starting >>> rcljava_common
3.273 Starting >>> mockito_vendor
4.253 Finished <<< mockito_vendor [0.98s]
6.375 Finished <<< rcljava_common [3.11s]
6.377 Starting >>> rosidl_generator_java
7.652 --- stderr: rosidl_generator_java
7.652 CMake Error at CMakeLists.txt:32 (find_package):
7.652   By not providing "Findtest_interface_files.cmake" in CMAKE_MODULE_PATH this
7.652   project has asked CMake to find a package configuration file provided by
7.652   "test_interface_files", but CMake did not find one.
7.652 
7.652   Could not find a package configuration file provided by
7.652   "test_interface_files" with any of the following names:
7.652 
7.652     test_interface_filesConfig.cmake
7.652     test_interface_files-config.cmake
7.652 
7.652   Add the installation prefix of "test_interface_files" to CMAKE_PREFIX_PATH
7.652   or set "test_interface_files_DIR" to a directory containing one of the
7.652   above files.  If "test_interface_files" provides a separate development
7.652   package or SDK, be sure it has been installed.
7.652 
7.652 
7.652 ---
7.652 Failed   <<< rosidl_generator_java [1.28s, exited with code 1]
7.700 
7.700 Summary: 6 packages finished [6.82s]
7.700   1 package failed: rosidl_generator_java
7.700   1 package had stderr output: rosidl_generator_java
7.700   10 packages not processed
------
rcljava.dockerfile:39
```

### Fix #2:

```rcljava.repos```

**Added:**
```
ros2/test_interface_files:
    type: git
    url: https://github.com/ros2/test_interface_files.git
    version: galactic
```

### Comments

- **Issue #1:** O erro das dependências do colcon ficou resolvido com modificação das versões dos respotórios do rosidl_defaults e unique_identifier_msgs para galactic.
- **Issue #2:** Esta atualização causou um novo erro de falta do package test_interface_files, que foi resolvido com a adição do mesmo no arquivo rcljava.repos.

## simulator

```> ./build_docker_images.sh simulator``` 

### Issue #1

```
207.9 ./Firmware/Tools/setup/ubuntu.sh: line 58: lsb_release: command not found
------
jmavsim.dockerfile:12
```

### Issue #2


- Necessidade de sudo para executar vários comandos no script Tools/setup/ubuntu.sh.

### Issue #3

```
349.5 ./Firmware/Tools/setup/ubuntu.sh: line 234: wget: command not found
349.6 Warning: apt-key output should not be parsed (stdout is not a terminal)
349.6 gpg: no valid OpenPGP data found.
------
jmavsim.dockerfile:14
```

### Fix #1, #2, #3

```jmavsim.dockerfile```

**Added:**
```
apt-get install -y \
    lsb-release \
    sudo \
    wget && \
```

### Issue #4

```
620.5 [26/868] Generating git version header
620.5 FAILED: src/lib/version/build_git_version.h 
620.5 cd /home/Firmware && /usr/bin/python3 /home/Firmware/src/lib/version/px_update_git_header.py /home/Firmware/build/px4_sitl_default/src/lib/version/build_git_version.h --validate
620.5 
620.5 Error: the git tag '55563eb' does not match the expected format.
620.5 
620.5 The expected format is 'v<PX4 version>[-<custom version>]'
620.5   <PX4 version>: v<major>.<minor>.<patch>[-rc<rc>|-beta<beta>|-alpha<alpha>|-dev]
620.5   <custom version>: <major>.<minor>.<patch>[-rc<rc>|-beta<beta>|-alpha<alpha>|-dev]
620.5 Examples:
620.5   v1.9.0-rc3 (preferred)
620.5   v1.9.0-beta1
620.5   v1.9.0-1.0.0
620.5   v1.9.0-1.0.0-alpha2
620.5 See also https://dev.px4.io/master/en/setup/building_px4.html#firmware_version
```

### Fix #4

```jmavsim.dockerfile > line 17```

**From:**
```
git clone https://github.com/PX4/Firmware.git --recursive --branch v1.10.2 --depth 1 && \
```

**To:**
```
git clone https://github.com/PX4/Firmware.git --recursive --branch v1.10.1 --depth 1 && \
```

### Comments
- **Issues #1, #2 e #3:** A Instalação dos vários pacotes em falta resolveu os erros de execução do script Tools/setup/ubuntu.sh.
- **Issue #4:** O erro da git tag foi resolvido com o downgrade da versão do Firmware para a versão v1.10.1 Stable Release