cmake_minimum_required(VERSION 3.8)

project(tf_tree)
SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include_directories(${CMAKE_SOURCE_DIR}/../../utils/check_utils)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN TF2Node
    EXECUTABLE tf2_node
)

ament_auto_package(
    INSTALL_TO_SHARE
    launch
    config
)