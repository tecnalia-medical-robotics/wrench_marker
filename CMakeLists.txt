cmake_minimum_required(VERSION 2.8.3)
project(wrench_marker)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  visualization_msgs
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs visualization_msgs
)

include_directories( include )
include_directories( ${catkin_INCLUDE_DIRS} )

add_library( wrench_marker src/wrench_marker.cpp )
target_link_libraries( wrench_marker ${catkin_LIBRARIES} )

add_executable( wrench_marker_publisher src/wrench_marker_publisher.cpp )
target_link_libraries( wrench_marker_publisher wrench_marker ${catkin_LIBRARIES} )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
