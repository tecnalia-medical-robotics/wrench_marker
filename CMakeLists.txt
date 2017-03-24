cmake_minimum_required(VERSION 2.8.3)
project(wrench_marker)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  visualization_msgs
  roscpp
)

find_package( Boost REQUIRED )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library( ${PROJECT_NAME} src/${PROJECT_NAME}.cpp )
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} )

add_executable( ${PROJECT_NAME}_publisher src/${PROJECT_NAME}_publisher.cpp )
target_link_libraries( ${PROJECT_NAME}_publisher ${PROJECT_NAME} ${catkin_LIBRARIES} )

install( TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install( DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
