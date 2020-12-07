cmake_minimum_required(VERSION 2.8.3)
project(pepper_developed)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
             roscpp
             rospy
             tf2
             std_msgs
             naoqi_libqi
             naoqi_libqicore
             pepper_lib_developed
)


## Declare a catkin package
catkin_package()

## Declare directories where files of this package will find the headers
include_directories(${catkin_INCLUDE_DIRS})

## Executables
add_executable(start_pepper src/start_pepper.cpp)
target_link_libraries(start_pepper ${catkin_LIBRARIES})

add_executable(enable_pepper src/enable_pepper.cpp)
target_link_libraries(enable_pepper ${catkin_LIBRARIES})

add_executable(head_control src/head_control.cpp)
target_link_libraries(head_control ${catkin_LIBRARIES})
