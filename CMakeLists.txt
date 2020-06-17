cmake_minimum_required(VERSION 3.10)

project(stereo_odometry)

set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(OpenCV 3 REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/viso2/src
)

find_package(
    catkin REQUIRED COMPONENTS
        roscpp
        tf
        pcl_ros
        sensor_msgs
        pcl_conversions
        nav_msgs
        geometry_msgs
        cv_bridge
)
include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

add_executable(stereo_odometry src/stereo_odometry_node.cpp
                               src/System.cpp 
)
target_link_libraries(stereo_odometry   ${catkin_LIBRARIES} 
                                        ${OpenCV_LIBS} 
                                        ${PROJECT_SOURCE_DIR}/viso2/lib/libviso2.so
)