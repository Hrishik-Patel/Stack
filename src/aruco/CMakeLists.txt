cmake_minimum_required(VERSION 3.8)
project(aruco)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# --- Find ROS 2 Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED) # <--- Added for topic sync
find_package(OpenCV REQUIRED COMPONENTS objdetect core imgproc)

# ==========================================
# 1. Build Executable: aruco_node
# ==========================================
add_executable(aruco_node src/aruco_node.cpp)

target_include_directories(aruco_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(aruco_node
  rclcpp
  geometry_msgs
  sensor_msgs
  cv_bridge
  image_transport
  message_filters
)
target_link_libraries(aruco_node ${OpenCV_LIBRARIES})

# ==========================================
# 2. Build Executable: camera_publisher
# ==========================================
add_executable(camera_publisher src/camera_publisher.cpp)

target_include_directories(camera_publisher PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(camera_publisher
  rclcpp
  sensor_msgs
  cv_bridge
)
target_link_libraries(camera_publisher ${OpenCV_LIBRARIES})

# ==========================================
# Installation Target Outputs
# ==========================================
install(TARGETS
  aruco_node
  camera_publisher
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()