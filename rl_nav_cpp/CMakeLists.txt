cmake_minimum_required(VERSION 3.8)
project(rl_nav_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(ONNX_INCLUDE_DIR /home/user/Documents/ros2_ws_ict_nav2/src/rl_nav_cpp/onnxruntime-linux-x64-1.24.4/include)
set(ONNX_LIB_DIR /home/user/Documents/ros2_ws_ict_nav2/src/rl_nav_cpp/onnxruntime-linux-x64-1.24.4/lib)

add_executable(onnx_node src/onnx_node.cpp)
add_library(rl_controller SHARED src/rl_controller.cpp)

target_include_directories(onnx_node PUBLIC ${ONNX_INCLUDE_DIR})
target_link_directories(onnx_node PUBLIC ${ONNX_LIB_DIR})
target_link_libraries(onnx_node onnxruntime)
ament_target_dependencies(onnx_node rclcpp)

target_include_directories(rl_controller PUBLIC ${ONNX_INCLUDE_DIR})
target_link_directories(rl_controller PUBLIC ${ONNX_LIB_DIR})
target_link_libraries(rl_controller onnxruntime)

ament_target_dependencies(rl_controller
  rclcpp
  nav2_core
  geometry_msgs
  nav_msgs
  sensor_msgs
  pluginlib
)

install(TARGETS onnx_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS rl_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

pluginlib_export_plugin_description_file(nav2_core rl_controller_plugin.xml)

ament_package()
