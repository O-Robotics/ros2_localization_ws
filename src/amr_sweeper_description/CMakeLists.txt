cmake_minimum_required(VERSION 3.8)
project(amr_sweeper_description)



find_package(ament_cmake REQUIRED)



install(
  DIRECTORY config launch meshes urdf worlds
  DESTINATION share/${PROJECT_NAME}  
)

ament_package()
