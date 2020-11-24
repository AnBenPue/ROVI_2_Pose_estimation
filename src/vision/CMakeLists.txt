cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(vision)
# Hemanth:
set(PCL_DIR /usr/local/share/pcl-1.8)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
find_package(OpenCV REQUIRED)

find_package(PCL 1.3 REQUIRED)
set(PCL_INCLUDE_DIRS /usr/include/pcl-1.8 /usr/include/eigen3/)
#SET(RobWork_DIR ~/RobWork/RobWork/cmake)


SET(RW_ROOT $ENV{RW_ROOT})
SET(CMAKE_BUILD_TYPE Release)
MESSAGE("-- Build type: " ${CMAKE_BUILD_TYPE})

set(RobWork_DIR ${RW_ROOT}/cmake) 



FIND_PACKAGE(RobWork REQUIRED)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )


catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs tf2 tf2_ros
  DEPENDS PCL
  DEPENDS ROBWORK
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  #include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${ROBWORK_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src
)
add_library(functionLib SHARED
  src/Utils.cpp
  src/PreProcessing.cpp
  src/localization.cpp
  )

target_link_libraries(functionLib ${catkin_LIBRARIES} ${ROBWORK_LIBRARIES} ${PCL_LIBRARIES})
link_directories(
  ${PCL_LIBRARY_DIRS}

)
add_definitions(${PCL_DEFINITIONS})

add_executable(pose_estimation src/pose_estimation.cpp)
target_link_libraries(pose_estimation ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${ROBWORK_LIBRARIES} functionLib)

#add_executable(pose_estimation_shot src/pose_estimation_shot.cpp)
#target_link_libraries(pose_estimation_shot ${catkin_LIBRARIES} ${PCL_LIBRARIES})


#add_executable(Filter_XYZ src/Filter_XYZ.cpp)
#target_link_libraries(Filter_XYZ ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#add_executable(PLY_2_PCL src/PLY_2_PCL.cpp)
#target_link_libraries(PLY_2_PCL ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#add_executable(Table_Markers_detection src/Table_Markers_detection.cpp)
#target_link_libraries(Table_Markers_detection ${PCL_LIBRARIES} ${OpenCV_LIBS})

#add_executable(fit_plane src/fit_plane.cpp)
#target_link_libraries(fit_plane ${PCL_LIBRARIES})

#add_executable(remove_small_clusters src/remove_small_clusters.cpp)
#target_link_libraries(remove_small_clusters ${PCL_LIBRARIES})

#add_executable(locate_piece src/locate_piece.cpp)
#target_link_libraries(locate_piece ${PCL_LIBRARIES})

#add_executable(locate_orientation src/locate_orientation.cpp)
#target_link_libraries(locate_orientation ${PCL_LIBRARIES})


