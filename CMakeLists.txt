cmake_minimum_required(VERSION 3.5)
project(image_splitter_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Include C++ libraries
include_directories(include)

# Create the stereo_image_splitter component
add_library(stereo_image_splitter SHARED
  src/stereo_image_splitter.cpp)
ament_target_dependencies(stereo_image_splitter
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  OpenCV)
rclcpp_components_register_nodes(stereo_image_splitter "StereoImageSplitter")

# Create the stereo_image_splitter_node executable
add_executable(stereo_image_splitter_node
  src/stereo_image_splitter_node.cpp)
target_link_libraries(stereo_image_splitter_node
  stereo_image_splitter)
ament_target_dependencies(stereo_image_splitter_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV)

# Install targets
install(TARGETS
  stereo_image_splitter
  stereo_image_splitter_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
