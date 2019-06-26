cmake_minimum_required(VERSION 2.8.3)
project(color_recognitioning)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

#set (Eigen_DIRS "/usr/include/eigen3")
set (CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    nav_msgs
    cv_bridge
    tf
)

find_package(OpenCV REQUIRED)

find_package(Sophus REQUIRED)

find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)




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

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/color_recognitioning.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(color_recognitioning_node src/card_detection.cpp
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
target_link_libraries(color_recognitioning_node ${catkin_LIBRARIES} 
                                                ${Sophus_LIBRARIES}
                                                ${OpenCV_LIBS}
                                                )

