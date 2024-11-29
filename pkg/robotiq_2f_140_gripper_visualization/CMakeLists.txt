cmake_minimum_required(VERSION 3.5)
project(robotiq_2f_140_gripper_visualization)

# Find necessary packages
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Declare the package
ament_package()

