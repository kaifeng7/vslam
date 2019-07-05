cmake_minimum_required(VERSION 2.8.3)
project(vslam)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

#set (Eigen_DIRS "/usr/include/eigen3")
set (CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    sensor_msgs
    nav_msgs
    cv_bridge
    tf
    message_generation
)

find_package(OpenCV 3 REQUIRED)

find_package(Sophus REQUIRED)

find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


add_message_files(
    DIRECTORY msg
    FILES Camera.msg
          Card.msg
          Viz.msg
)
generate_messages(
    DEPENDENCIES std_msgs
                 geometry_msgs
)
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES 
    CATKIN_DEPENDS  roscpp
                    sensor_msgs
                    nav_msgs
                    cv_bridge
                    tf
    DEPENDS Sophus

)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}

    ${Sophus_INCLUDE_DIRS}
    #${EIGEN3_INCLUDE_DIRS}
    "/usr/local/include/eigen3"

)






add_executable(vslam_node src/card_detection.cpp
                          src/card_detection_node.cpp
                          src/camera.cpp
                          src/key_frame.cpp
                          src/map_point.cpp
                          src/map.cpp
                          )

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(vslam_node ${catkin_LIBRARIES} 
                                                ${Sophus_LIBRARIES}
                                                ${OpenCV_LIBS}
                                                )

