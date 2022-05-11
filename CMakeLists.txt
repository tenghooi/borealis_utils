cmake_minimum_required(VERSION 3.0.2)
project(borealis_utils)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  tf2
  tf2_ros
  pcl_ros
  sensor_msgs
  std_msgs
)

find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})


catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${PROJECT_SOURCE_DIR}/include)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pseudo_frame_tf_broadcaster 
               src/pseudo_frame_tf_broadcaster.cpp
               )
target_link_libraries(pseudo_frame_tf_broadcaster
                      ${catkin_LIBRARIES}
                      )

add_executable(pseudo_lidar_frame_broadcaster 
               src/pseudo_lidar_frame_broadcaster.cpp
               )
target_link_libraries(pseudo_lidar_frame_broadcaster
                      ${catkin_LIBRARIES}
                      )

add_executable(pseudo_pointcloud_tf 
               src/pseudo_pointcloud_tf.cpp
               )
target_link_libraries(pseudo_pointcloud_tf
                      ${catkin_LIBRARIES}
                      )                      