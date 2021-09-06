cmake_minimum_required(VERSION 3.0.2)
project(bender_perception)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  image_geometry
  nav_msgs
  pluginlib
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_core costmap_layers
 CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_core
  src/vision.cpp 
  src/lidar.cpp
)
add_dependencies(${PROJECT_NAME}_core 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_core
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}  
)

add_library(costmap_layers
  plugins/costmap_lane_layer.cpp
)
add_dependencies(costmap_layers 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(costmap_layers
  ${PROJECT_NAME}_core
)

add_executable(${PROJECT_NAME} src/bender_perception.cpp)
target_link_libraries(${PROJECT_NAME} 
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES} 
)

#############
## Install ##
#############

install(TARGETS
  ${PROJECT_NAME}_core
  costmap_layers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
