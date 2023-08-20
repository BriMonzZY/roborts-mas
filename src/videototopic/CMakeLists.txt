project(videototopic)
cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
add_definitions(-w -Wno-dev)

find_package(catkin REQUIRED COMPONENTS
  roscpp  
  cv_bridge
  image_transport
)
find_package(OpenCV 4 REQUIRED)

message(STATUS "OpenCV library status:")
message(STATUS "version: ${OpenCV_VERSION}")
message(STATUS "libraries: ${OpenCV_LIBS}")
message(STATUS "include path: ${OpenCV_INCLUDE_DIRS}")

catkin_package()

# set(LIBS
#   ${OpenCV_LIBS}
#   -lboost_system
# )

add_executable(image_publisher
  src/videototopic.cpp
)

target_link_libraries(image_publisher
  PRIVATE
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_include_directories(image_publisher
  PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRECTORIES}
)
