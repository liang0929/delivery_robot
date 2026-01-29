cmake_minimum_required(VERSION 3.8)
project(robot_description)

find_package(ament_cmake REQUIRED)

# 安裝 URDF/Xacro 檔案
install(DIRECTORY
  urdf
  launch
  meshes
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
