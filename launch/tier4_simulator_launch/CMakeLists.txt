cmake_minimum_required(VERSION 3.14)
project(tier4_simulator_launch)

find_package(autoware_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)

autoware_package()
ament_auto_find_build_dependencies()

ament_auto_package(INSTALL_TO_SHARE
  launch
  rviz
)
