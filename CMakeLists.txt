cmake_minimum_required(VERSION 2.8.3)
project(wrench_marker)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs visualization_msgs
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
