cmake_minimum_required(VERSION 3.0.2)
project(loop_closure_gtsam)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "RELEASE")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  rospy
  tf
  std_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  eigen_conversions
  pcl_ros
  base_controller
  nav_msgs
)

find_package(Eigen3 REQUIRED)

find_package(GTSAM REQUIRED)
message(STATUS "------------gtsam :${gtsam}")
message(STATUS "------------GTSAM_LIBRARY_DIRS : ${GTSAM_LIBRARY_DIRS}")

find_package(PCL 1.8 REQUIRED )
# Generate services in the 'srv' folder
add_service_files(
  DIRECTORY
  srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})


link_directories(
	include
	${PCL_LIBRARY_DIRS}
	${GTSAM_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# 添加exec执行的依赖
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
  SYSTEM
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

SET(src_file loop_closure_gtsam)
add_executable (${src_file} src/${src_file}.cpp src/LaserLoopClosure_time.cc)

add_dependencies(${src_file} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries (${src_file} gtsam ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${GTSAM_LIBRARY_DIRS} libzlog.a)
