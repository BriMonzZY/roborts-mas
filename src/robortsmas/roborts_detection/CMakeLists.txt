project(roborts_detection)
cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)
set(COMPILE_PROTO 0)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  actionlib
  cv_bridge
  image_transport
  std_msgs
  sensor_msgs
  roborts_msgs
  roborts_common
  )

find_package(ProtoBuf REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)

message(STATUS "OpenCV library status:")
message(STATUS "version: ${OpenCV_VERSION}")
message(STATUS "libraries: ${OpenCV_LIBS}")
message(STATUS "include path: ${OpenCV_INCLUDE_DIRS}")

catkin_package()

include_directories(${EIGEN3_INCLUDE_DIR} util)

add_subdirectory(util)
add_subdirectory(armor_detection)
#add_subdirectory(color_detection)
