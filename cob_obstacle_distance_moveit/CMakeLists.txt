cmake_minimum_required(VERSION 2.8.3)
project(cob_obstacle_distance_moveit)

add_compile_options(-std=c++11)

# enable debugging
set(ROS_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Debug)

# Set catkin run and build packages
SET(CATKIN_RUN_PACKAGES
        cob_control_msgs
        cob_srvs
        eigen_conversions
        geometric_shapes
        geometry_msgs
        moveit_core
        moveit_ros_perception
        moveit_ros_planning_interface
        roscpp
        tf
	tf_conversions
)

# set cakin build package
SET(CATKIN_BUILD_PACKAGES
  ${CATKIN_RUN_PACKAGES}
  cmake_modules
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS ${CATKIN_BUILD_PACKAGES})

find_package(Boost REQUIRED COMPONENTS)

message("Finding EIGEN library")
find_package(Eigen REQUIRED)

add_definitions(${EIGEN_DEFINITIONS})


find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFCL REQUIRED fcl)
find_library(LIBFCL_LIBRARIES_FULL ${LIBFCL_LIBRARIES} ${LIBFCL_LIBRARY_DIRS})
set(fcl_LIBRARIES "${LIBFCL_LIBRARIES_FULL}")

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES pd_motion_control
  CATKIN_DEPENDS ${CATKIN_RUN_PACKAGES}
  DEPENDS Boost 
)


### Build ###
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)

add_library(obstacle_distance_moveit src/${PROJECT_NAME}/obstacle_distance_moveit.cpp)
add_dependencies(obstacle_distance_moveit ${catkin_EXPORTED_TARGETS})
target_link_libraries(obstacle_distance_moveit ${catkin_LIBRARIES} ${FCL_LIBRARIES})

add_executable(obstacle_distance_node src/obstacle_distance_moveit_node.cpp)
add_dependencies(obstacle_distance_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(obstacle_distance_node obstacle_distance_moveit ${catkin_LIBRARIES})

add_executable(visualize_obstacle_distance_node src/visualize_obstacle_distance_node.cpp)
add_dependencies(visualize_obstacle_distance_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(visualize_obstacle_distance_node ${catkin_LIBRARIES})

add_executable(test_obstacle_publisher_node src/test_obstacle_publisher_node.cpp)
add_dependencies(test_obstacle_publisher_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_obstacle_publisher_node ${catkin_LIBRARIES})


### INSTALL ###
install(TARGETS obstacle_distance_moveit obstacle_distance_node visualize_obstacle_distance_node test_obstacle_publisher_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(PROGRAMS scripts/test_obstacle_publisher.py scripts/test_registration_client.py scripts/test_service_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)

install(DIRECTORY files launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

