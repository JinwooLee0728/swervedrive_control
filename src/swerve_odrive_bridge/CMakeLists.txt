cmake_minimum_required(VERSION 3.5)
project(swerve_odrive_bridge)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(odrive_can REQUIRED)

add_executable(swerve_odrive_bridge src/swerve_odrive_bridge.cpp)

ament_target_dependencies(
  swerve_odrive_bridge
  rclcpp
  geometry_msgs
  sensor_msgs
  std_srvs
  odrive_can
)

install(TARGETS
  swerve_odrive_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
