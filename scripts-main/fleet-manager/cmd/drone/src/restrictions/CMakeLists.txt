# src/restrictions/CMakeLists.txt

# GEOS unstable API warning suppression
add_definitions(-DUSE_UNSTABLE_GEOS_CPP_API)

include_directories(${INCLUDE_PATH})

find_package(rclcpp REQUIRED)
find_package(MAVSDK REQUIRED)

add_library(restrictions
    restrictions.cpp
    geofence.cpp
)

target_link_libraries(restrictions
	${COMMON_LIBS}
    geos
)

ament_target_dependencies(restrictions
	${COMMON_DEPS}
)
