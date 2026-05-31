cmake_minimum_required(VERSION 3.8)
project(grasp_bringup)

find_package(ament_cmake REQUIRED)

# Install all resource directories so ROS2 can find them at runtime
install(DIRECTORY launch config urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
