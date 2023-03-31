cmake_minimum_required(VERSION 2.8.3)
project(base_controller)
set( CMAKE_CXX_FLAGS "-std=c++11" )
set(OpenCV_DIR /usr/share/OpenCV)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  #serial
  tf
  nav_msgs
  geometry_msgs
  actionlib
  actionlib_msgs
  OpenCV
  cv_bridge
  image_transport
  sensor_msgs
  )

add_message_files(
  FILES
  #Num.msg
  csgPoseStampedMatchValue.msg
  csgPoseStampedRoute.msg
  Error.msg
  NavParams.msg
  CSGMapInfo.msg
  CSGMagneticInfo.msg
  CSGMagneticCtrl.msg
  CSGVisionNavCtrl.msg
)
add_action_files(
DIRECTORY action 
FILES 
DoDishes.action
Inspect.action
Fibonacci.action
Averaging.action
)
generate_messages(
 DEPENDENCIES
 geometry_msgs
 actionlib_msgs
)
catkin_package(
)

include_directories(include 
  ${catkin_INCLUDE_DIRS}
  #${serial_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
link_directories(
  ${catkin_LIBRARY_DIRS}
)
add_executable(base_nav src/base_nav.cpp)
target_link_libraries(base_nav ${catkin_LIBRARIES} zlog)

add_executable(odom_test src/odom_test.cpp)
target_link_libraries(odom_test ${catkin_LIBRARIES})
add_executable(joy_node src/joy_node.cpp)
target_link_libraries(joy_node ${catkin_LIBRARIES})
