cmake_minimum_required(VERSION 2.8.3)
project(stereo_visual_odom)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package( OpenCV REQUIRED )
find_package(Eigen3 REQUIRED)
FIND_PACKAGE(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_filters
  g3log_catkin
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES stereo_visual_odom
CATKIN_DEPENDS g3log_catkin
pcl_ros pcl
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_subdirectory(src/evaluate)
#
include_directories(${OpenCV_INCLUDE_DIRS} )
include_directories(evaluate)
#
#
add_library(feature SHARED "src/feature.cpp")
add_library(bucket SHARED "src/bucket.cpp")
add_library(utils SHARED "src/utils.cpp")
add_library(visualOdometry SHARED "src/visualOdometry.cpp")
add_library(input SHARED "src/input.cpp")

#
#
#
add_library(Frame SHARED "src/Frame.cpp")
#
#
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})
message(${PROJECT_SOURCE_DIR})
#
add_executable( run src/main.cpp )
target_link_libraries( bucket ${OpenCV_LIBS} )
target_link_libraries( feature ${OpenCV_LIBS} bucket)
target_link_libraries( utils evaluate_odometry feature ${OpenCV_LIBS} )
target_link_libraries( visualOdometry utils bucket feature ${OpenCV_LIBS})
#
#
target_link_libraries( Frame ${catkin_LIBRARIES} ${OpenCV_LIBS} )
target_link_libraries( run ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS} bucket input feature utils visualOdometry Frame  )
