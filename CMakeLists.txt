project(mppi)

find_package(CUDA REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(mppi_node
    src/mppi_node.cpp
    src/trajectory_generator.cpp
    src/trajectory_sim.cu
    src/ros_functions.cpp
    src/track_handler.cpp
)

ament_target_dependencies(mppi_node
    rclcpp
    geometry_msgs
    nav_msgs
)

target_include_directories(mppi_node PRIVATE include)
target_link_libraries(mppi_node ${CUDA_LIBRARIES} yaml-cpp)

find_package(yaml-cpp REQUIRED)