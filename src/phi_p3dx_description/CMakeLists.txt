cmake_minimum_required(VERSION 3.8)
project(phi_p3dx_description)

# find dependencies
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf models worlds rviz map
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
