cmake_minimum_required(VERSION 2.8.3)
project(descartes_tutorials)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roslib
  descartes_core
  descartes_moveit
  descartes_trajectory
  descartes_planner
  descartes_utilities
  trajectory_msgs
  tf
  tf_conversions
  descartes_opw_model
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    positioner_point
  CATKIN_DEPENDS
    roslib
    descartes_core
    descartes_moveit
    descartes_trajectory
    descartes_planner
    descartes_utilities
    trajectory_msgs
    tf
    tf_conversions
    descartes_opw_model
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Positioner point library
add_library(positioner_point
  src/positioner_point.cpp
)
target_link_libraries(positioner_point
  ${catkin_LIBRARIES}
)

# Tutorial 1
add_executable(${PROJECT_NAME}_tutorial1 src/tutorial1.cpp)

target_link_libraries(${PROJECT_NAME}_tutorial1
 ${catkin_LIBRARIES}
)

set_target_properties(${PROJECT_NAME}_tutorial1
  PROPERTIES OUTPUT_NAME tutorial1
  PREFIX "")

# Tutorial 2 - Puzzle Demo
add_executable(${PROJECT_NAME}_tutorial2 src/tutorial2.cpp)

target_link_libraries(${PROJECT_NAME}_tutorial2
 ${catkin_LIBRARIES}
)

set_target_properties(${PROJECT_NAME}_tutorial2
  PROPERTIES OUTPUT_NAME tutorial2
  PREFIX "")

# Tutorial 3 -
add_executable(${PROJECT_NAME}_tutorial3 src/tutorial3.cpp src/positioner_point.cpp)

target_link_libraries(${PROJECT_NAME}_tutorial3
 ${catkin_LIBRARIES}
)

set_target_properties(${PROJECT_NAME}_tutorial3
  PROPERTIES OUTPUT_NAME tutorial3
  PREFIX "")


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_tutorial1
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
