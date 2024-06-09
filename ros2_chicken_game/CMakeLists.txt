cmake_minimum_required(VERSION 3.8)
project(ros2_chicken_game)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
#This package is cmake build and to support python scripts below is added.
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yolov8_msgs REQUIRED)
find_package(pod2_msgs REQUIRED)

# Add the package specific directories in the install directory
install(DIRECTORY ROS2_Chicken_model
  DESTINATION share/${PROJECT_NAME}/
)
# Install Python modules.
# This is important to provide in order to use python executables
ament_python_install_package(${PROJECT_NAME})

# provide the paths of python scripts for install directory otherwise it will cause the error 'executable not found on libexec directory'


########### Chicken model specific Nodes ROS2################################################################ 

install(PROGRAMS
ROS2_Chicken_model/sequentialChicken.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
ROS2_Chicken_model/solutionConcepts.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
ROS2_Chicken_model/ChickenSpeedModulatorNode.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
ROS2_Chicken_model/chicken_node_backup.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
