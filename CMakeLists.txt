project(mppi)

find_package(CUDA REQUIRED)

add_executable(mppi_node
    src/mppi_node.cpp
    src/trajectory_generator.cpp
    src/trajectory_generator.cu
    src/ros_functions.cpp
)

target_include_directories(mppi_node PRIVATE include)
target_link_libraries(mppi_node ${CUDA_LIBRARIES})