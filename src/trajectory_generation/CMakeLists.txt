cmake_minimum_required(VERSION 3.8)
project(trajectory_generation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(trajectory_generator src/trajectory_generator.cpp)
target_include_directories(trajectory_generator PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(trajectory_generator rclcpp geometry_msgs std_msgs nav_msgs)

install(TARGETS trajectory_generator DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY DESTINATION share/${PROJECT_NAME})

ament_package()