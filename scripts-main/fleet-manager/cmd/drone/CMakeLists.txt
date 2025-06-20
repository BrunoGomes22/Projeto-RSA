cmake_minimum_required(VERSION 3.5)

project(fleetman_drone)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

set(INCLUDE_PATH ${PROJECT_SOURCE_DIR}/include)

set(COMMON_LIBS
    mavsdk
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_offboard
    jsoncpp
)

set(COMMON_DEPS
    rclcpp
    MAVSDK
)

add_subdirectory(src)

install(TARGETS
  drone_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
