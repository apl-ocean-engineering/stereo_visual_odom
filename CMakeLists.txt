cmake_minimum_required(VERSION 2.8.3)
project(stereo_visual_odom)

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
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/StereoVisualOdom.cfg
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES stereo_visual_odom
CATKIN_DEPENDS g3log_catkin
#pcl_ros pcl
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  # ${PCL_INCLUDE_DIRS}
)

set( CMAKE_CXX_FLAGS " -pthread " )


include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB stereo_odom_libs_SRCS
          lib/*.cpp)

add_library(stereo_odom_libs ${stereo_odom_libs_SRCS})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})
message(${PROJECT_SOURCE_DIR})


add_executable( stereo_odom src/stereo_odom.cpp )

target_link_libraries( stereo_odom_libs ${OpenCV_LIBS})


target_link_libraries( stereo_odom stereo_odom_libs ${catkin_LIBRARIES})


add_dependencies(stereo_odom ${PROJECT_NAME}_gencfg)
