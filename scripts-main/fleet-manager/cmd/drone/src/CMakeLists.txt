# src/CMakeLists.txt

include_directories(${INCLUDE_PATH})

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(MAVSDK REQUIRED)

add_subdirectory(restrictions)

add_executable(drone_controller
        drone_controller.cpp
        mavsdk_node.cpp
        drone_utils.cpp
        status_tracker.cpp
        coord_utils.cpp
        ${INCLUDE_PATH}/drone_core/cmd_handler.hpp
        cmd_handler.cpp
        )

target_link_libraries(drone_controller
        ${COMMON_LIBS}
        restrictions
        )

ament_target_dependencies(drone_controller
        ${COMMON_DEPS}
        rclcpp_components
        std_msgs
        class_loader
        )