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
  geometry_msgs
  message_generation
  std_msgs  
)


find_package(Eigen3 REQUIRED)

find_package(GTSAM REQUIRED)

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
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
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


SET(src_file optimize_map)
add_executable (${src_file} src/${src_file}.cpp )
add_dependencies(${src_file} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries (${src_file}  ${catkin_LIBRARIES} ${PCL_LIBRARIES} )


# SET(src_file optimize_lio_path_z)
# add_executable (${src_file} src/${src_file}.cpp )
# add_dependencies(${src_file} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries (${src_file}  ${catkin_LIBRARIES} ${PCL_LIBRARIES})


# SET(src_file get_bds_by_two_odom)
# add_executable (${src_file} src/${src_file}.cpp )
# add_dependencies(${src_file} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries (${src_file}  ${catkin_LIBRARIES} )


# SET(src_file optimize_map_tf)
# add_executable (${src_file} src/${src_file}.cpp )
# add_dependencies(${src_file} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries (${src_file}  ${catkin_LIBRARIES} ${PCL_LIBRARIES})


# SET(src_file optimize_map_gtsam)
# add_executable (${src_file} src/${src_file}.cpp src/LaserLoopClosure.cc)
# add_dependencies(${src_file} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries (${src_file} gtsam ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${GTSAM_LIBRARY_DIRS} libzlog.a)


SET(src_file optimize_map_gtsam_time)
add_executable (${src_file} src/${src_file}.cpp src/LaserLoopClosure_time.cc)
add_dependencies(${src_file} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries (${src_file} gtsam ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${GTSAM_LIBRARY_DIRS} libzlog.a)


SET(src_file optimize_map_gtsam_time_all_pc)
add_executable (${src_file} src/${src_file}.cpp src/LaserLoopClosure_time.cc)
add_dependencies(${src_file} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries (${src_file} gtsam ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${GTSAM_LIBRARY_DIRS} libzlog.a)

SET(src_file get_two_msg)
add_executable (${src_file} src/${src_file}.cpp src/LaserLoopClosure_time_service.cc)
add_dependencies(${src_file} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries (${src_file} gtsam ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${GTSAM_LIBRARY_DIRS} libzlog.a)

message(STATUS "------------gtsam :${gtsam}")
message(STATUS "------------gtsam 2222 : ${GTSAM_LIBRARY_DIRS}")


## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_odom.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
