cmake_minimum_required(VERSION 3.8)
project(ict_bot_nav)

find_package(ament_cmake REQUIRED)

# Install all folders so ROS2 can find them
install(DIRECTORY
  urdf
  worlds
  config
  launch
  maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
