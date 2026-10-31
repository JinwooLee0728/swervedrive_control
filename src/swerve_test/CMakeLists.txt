cmake_minimum_required(VERSION 3.8)
project(swerve_test)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)

include_directories(include)

add_library(swerve_test_lib
  src/dynamixel_sdk_interface.cpp
  src/swerve.cpp
)
ament_target_dependencies(swerve_test_lib
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  dynamixel_sdk
)

add_executable(swerve_steering_node src/swerve_node.cpp)
target_link_libraries(swerve_steering_node swerve_test_lib)
ament_target_dependencies(swerve_steering_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

add_executable(dxl_scan src/dxl_scan.cpp)
ament_target_dependencies(dxl_scan dynamixel_sdk)

install(TARGETS
  swerve_test_lib
  swerve_steering_node
  dxl_scan
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

ament_package()
