cmake_minimum_required(VERSION 3.8)
project(controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(controller_node
  src/controller_node.cpp
  src/controller_factory.cpp
  src/proposed.cpp
  src/kanayama.cpp
  src/li.cpp
  src/peng.cpp
  src/muhamed.cpp)
target_include_directories(controller_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(controller_node rclcpp geometry_msgs nav_msgs std_msgs)
target_link_libraries(controller_node Eigen3::Eigen)

install(TARGETS controller_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY DESTINATION share/${PROJECT_NAME})

ament_package()