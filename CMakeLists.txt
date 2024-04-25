cmake_minimum_required(VERSION 3.0.2)
project(image_splitter_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Add compile options, if necessary
add_compile_options(-Wall -Wextra -Wpedantic)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
)
find_package(OpenCV REQUIRED)

# Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES image_splitter_ros
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_transport
  DEPENDS OpenCV
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Declare a C++ library if there are multiple source files or commonly reused components
add_library(${PROJECT_NAME}_lib
  src/stereo_image_splitter.cpp
)

# Add an executable for the node
add_executable(stereo_image_splitter_node src/stereo_image_splitter_node.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(stereo_image_splitter_node
  ${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Install the executable
install(TARGETS stereo_image_splitter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install other directories if necessary
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
