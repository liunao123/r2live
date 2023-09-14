cmake_minimum_required(VERSION 3.0.2)
project(odom)
set(CMAKE_BUILD_TYPE "RELEASE")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  tf
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  eigen_conversions
  pcl_ros
  geometry_msgs
  message_generation
  std_msgs  
  # livox_ros_driver
  sensor_msgs
)


find_package(Eigen3 REQUIRED)

find_package(GTSAM REQUIRED)

message(STATUS "------------gtsam :${gtsam}")
message(STATUS "------------GTSAM_LIBRARY_DIRS : ${GTSAM_LIBRARY_DIRS}")

find_package(PCL 1.8 REQUIRED )
# Generate services in the 'srv' folder
add_service_files(
  # FILES
  # LoopTimePair.srv
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
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime roscpp rospy sensor_msgs std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES odom
#  CATKIN_DEPENDS nav_msgs roscpp tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
  SYSTEM
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  # "/usr/include/eigen3"
)


SET(src_file rotation_pcd_to_horizon_pcl)
add_executable (${src_file} src/${src_file}.cpp )
add_dependencies(${src_file} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries (${src_file} ${PCL_LIBRARIES} )


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/odom.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"

# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")


# SET(src_file optimize_map)
# add_executable (${src_file} src/${src_file}.cpp )
# add_dependencies(${src_file} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries (${src_file}  ${catkin_LIBRARIES} ${PCL_LIBRARIES} )


# SET(src_file optimize_lio_path_z)
# add_executable (${src_file} src/${src_file}.cpp )
# add_dependencies(${src_file} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries (${src_file}  ${catkin_LIBRARIES} ${PCL_LIBRARIES})


# SET(src_file undistorted)
# add_executable (${src_file} src/${src_file}.cpp )
# target_link_libraries (${src_file}  ${catkin_LIBRARIES} ${PCL_LIBRARIES})
# add_dependencies(${src_file} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# SET(src_file get_bds_by_two_odom)
# add_executable (${src_file} src/${src_file}.cpp )
# add_dependencies(${src_file} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries (${src_file}  ${catkin_LIBRARIES} )


# SET(src_file optimize_map_tf)
# add_executable (${src_file} src/${src_file}.cpp )
# add_dependencies(${src_file} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries (${src_file}  ${catkin_LIBRARIES} ${PCL_LIBRARIES})



# SET(src_file optimize_map_gtsam_time)
# add_executable (${src_file} src/${src_file}.cpp src/LaserLoopClosure_time.cc)
# add_dependencies(${src_file} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries (${src_file} gtsam ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${GTSAM_LIBRARY_DIRS} libzlog.a)


# SET(src_file optimize_map_gtsam_time_all_pc)
# add_executable (${src_file} src/${src_file}.cpp src/LaserLoopClosure_time.cc)
# add_dependencies(${src_file} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries (${src_file} gtsam ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${GTSAM_LIBRARY_DIRS} libzlog.a)

# SET(src_file loop_closure_gtsam)
# add_executable (${src_file} src/${src_file}.cpp src/LaserLoopClosure_time_service.cc)
# add_dependencies(${src_file} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries (${src_file} gtsam ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${GTSAM_LIBRARY_DIRS} libzlog.a)

# SET(src_file loop_closure_gtsam_tf2)
# add_executable (${src_file} src/${src_file}.cpp src/LaserLoopClosure_time_service.cc)
# add_dependencies(${src_file} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries (${src_file} gtsam ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${GTSAM_LIBRARY_DIRS} libzlog.a)


# SET(src_file down_filtered_pcd)
# add_executable (${src_file} src/${src_file}.cpp )
# add_dependencies(${src_file} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries (${src_file} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${GTSAM_LIBRARY_DIRS} libzlog.a)



# SET(src_file livox_to_pointcloud2)
# add_executable (${src_file} src/${src_file}.cpp )
# add_dependencies(${src_file} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries (${src_file} ${catkin_LIBRARIES} ${PCL_LIBRARIES} )


# SET(src_file reproject_r2live_path)
# add_executable (${src_file} src/${src_file}.cpp )
# add_dependencies(${src_file} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries (${src_file} ${catkin_LIBRARIES} ${PCL_LIBRARIES} )

# SET(src_file read_xml_)
# add_executable (${src_file} src/${src_file}.cpp )
# add_dependencies(${src_file} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add 20221216 by ln

