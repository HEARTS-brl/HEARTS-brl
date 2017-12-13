cmake_minimum_required(VERSION 2.8.3)
project(hearts_xtion)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  
  actionlib

  pcl_conversions
  pcl_ros

  dynamic_reconfigure
  tf

  cv_bridge
  image_transport
  
  sensor_msgs
  control_msgs
  geometry_msgs
)

catkin_package()

include_directories(SYSTEM 
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(xtion_src_node src/xtion_src_node.cpp)
target_link_libraries(xtion_src_node ${catkin_LIBRARIES})

### INSTALL ###

install(TARGETS xtion_src_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
