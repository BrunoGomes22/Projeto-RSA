cmake_minimum_required(VERSION 3.5)

project(drone_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set (INTERFACES_LIST
  "msg/MissionStatus.msg"
)

set (DEPENDENCIES_LIST
  "std_msgs"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${INTERFACES_LIST}
  DEPENDENCIES ${DEPENDENCIES_LIST}
)

install(
  FILES ros1_mapping_rules.yml
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)

ament_package()
