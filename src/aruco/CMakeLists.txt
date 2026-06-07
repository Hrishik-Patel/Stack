cmake_minimum_required(VERSION 3.8)
project(obstacle_search)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(search_test_node src/search_test_node.cpp)
ament_target_dependencies(search_test_node
  rclcpp geometry_msgs std_msgs)

add_executable(obstacle_avoidance_node src/obstacle_avoidance_node.cpp)
ament_target_dependencies(obstacle_avoidance_node
  rclcpp geometry_msgs std_msgs sensor_msgs)

install(TARGETS
  search_test_node
  obstacle_avoidance_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  urdf
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
