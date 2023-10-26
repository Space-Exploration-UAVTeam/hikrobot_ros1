cmake_minimum_required(VERSION 2.8.3)
project(hikrobot)
set(CMAKE_CXX_FLAGS "-std=c++11")
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  mavros_msgs
  std_srvs
  nodelet
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)


## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   exposure.msg
 )
## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
 )

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES hikrobot
  CATKIN_DEPENDS nodelet cv_bridge image_transport roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${catkin_INCLUDE_DIRS} /opt/MVS/include)
include_directories(include ${OpenCV_INCLUDE_DIRS})
link_directories(/opt/MVS/lib/64)
link_libraries(MvCameraControl)

## Declare a C++ library
add_executable(MvCameraPub src/MvCameraPub.cpp)
add_dependencies(MvCameraPub hikrobot_generate_messages_cpp)
target_link_libraries(MvCameraPub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${MvCameraControl_LIBRARIES})

add_executable(MvCameraPub_Auto src/MvCameraPub_Auto.cpp)
target_link_libraries(MvCameraPub_Auto ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${MvCameraControl_LIBRARIES})

add_executable(MvCameraPub_Trigger src/MvCameraPub_Trigger.cpp)
add_dependencies(MvCameraPub_Trigger hikrobot_generate_messages_cpp)
target_link_libraries(MvCameraPub_Trigger ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${MvCameraControl_LIBRARIES})

add_executable(MvCameraPub_Multiple src/MvCameraPub_Multiple.cpp)
add_dependencies(MvCameraPub_Multiple hikrobot_generate_messages_cpp)
target_link_libraries(MvCameraPub_Multiple ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${MvCameraControl_LIBRARIES})

add_executable(Brightness_Count src/Brightness_Count.cpp)
add_dependencies(Brightness_Count hikrobot_generate_messages_cpp)
target_link_libraries(Brightness_Count ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${MvCameraControl_LIBRARIES})