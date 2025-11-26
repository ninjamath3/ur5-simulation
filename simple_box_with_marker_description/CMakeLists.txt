cmake_minimum_required(VERSION 3.8)
project(simple_box_with_marker_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
