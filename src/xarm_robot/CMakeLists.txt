cmake_minimum_required(VERSION 3.8)
project(xarm_robot)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE debug)
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(interfaces REQUIRED)
find_package(PkgConfig REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/../../include)
link_directories(${CMAKE_SOURCE_DIR}/../../lib)
include_directories(${CMAKE_SOURCE_DIR}/../../utils/check_utils)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
)
rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN XarmNode
    EXECUTABLE ${PROJECT_NAME}_node
)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp rclcpp_components )


set_target_properties(${PROJECT_NAME}_node PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "$ORIGIN:$ORIGIN/../../../../lib")
# 使用一致的语法格式
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${SERIAL_INCLUDE_DIRS}
)


#将动态链接库链接到头文件
target_link_libraries(${PROJECT_NAME} xarm rclcpp::rclcpp serial ${SERIAL_LIBRARIES})
#将动态链接库链接到可执行文件
target_link_libraries(${PROJECT_NAME}_node xarm ${PROJECT_NAME} rclcpp::rclcpp)

############
# Install ##
############

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)
install(FILES ${PROJECT_SOURCE_DIR}/../../lib/libxarm.so
        DESTINATION lib)

ament_package()
