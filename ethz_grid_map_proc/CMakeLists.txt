cmake_minimum_required(VERSION 2.8.3)
project(ethz_grid_map_proc)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  grid_map_ros
  nav_msgs
  roscpp
  dynamic_reconfigure
  hector_obstacle_msgs
)

generate_dynamic_reconfigure_options(
  cfg/GridMapProc.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES grid_map_to_occupancy_grid
  CATKIN_DEPENDS grid_map_ros nav_msgs roscpp dynamic_reconfigure hector_obstacle_msgs
  #  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(HEADERS
  include/${PROJECT_NAME}/grid_map_to_occupancy_grid.h
)

set(SOURCES
  src/grid_map_to_occupancy_grid.cpp
  src/grid_map_to_occupancy_grid_node.cpp
)

add_library(grid_map_to_occupancy_grid ${HEADERS} ${SOURCES})
target_link_libraries(grid_map_to_occupancy_grid ${catkin_LIBRARIES})
add_dependencies(grid_map_to_occupancy_grid ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/grid_map_to_occupancy_grid_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  grid_map_to_occupancy_grid
)

#############
## Install ##
#############

install(TARGETS grid_map_to_occupancy_grid ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
