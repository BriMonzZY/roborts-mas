project(a_star_planner)
set(CMAKE_BUILD_TYPE Release)
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto AStarPlannerConfigProtoSrc AStarPlannerConfigProtoHds ${ProtoFiles})

include_directories(${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  SHARED
  ${AStarPlannerConfigProtoSrc}
  ${AStarPlannerConfigProtoHds}
  a_star_planner.cpp
  )

FIND_LIBRARY(roborts_costmap_so libroborts_costmap.so ${CMAKE_BINARY_DIR}/../roborts_costmap/lib)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
  # roborts_costmap
  ${roborts_costmap_so}
)
add_library(planning::global_planner::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
