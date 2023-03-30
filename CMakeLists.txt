cmake_minimum_required(VERSION 2.8.3)
project(cloudline2plane)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(catkin REQUIRED
    COMPONENTS
    rosbag
    roscpp
    rospy
    std_msgs
    nav_msgs
    tf
    tf_conversions
    pcl_conversions
    pcl_ros
    rviz_visual_tools
    cv_bridge
    image_transport
)

find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(Eigen3 REQUIRED)
find_package(YAML-CPP REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS 
    LIBRARIES  
    CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(SYSTEM
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

include_directories(include 
  ${catkin_INCLUDE_DIRS}
)

add_executable(cloudline2plane 
src/cloudline2plane.cpp
src/line_clustering.cpp
src/line_extraction.cpp
src/line_feature.cpp
src/plane_extraction.cpp
src/visualizer.cpp
)
target_link_libraries(cloudline2plane 
${PCL_LIBRARIES} 
${OpenCV_LIBRARIES}
/usr/lib/x86_64-linux-gnu/libyaml-cpp.so)


add_executable(cloudline2plane_proj 
src/cloudline2plane_proj.cpp
src/line_clustering.cpp
src/line_extraction.cpp
src/line_feature.cpp
src/plane_extraction.cpp
src/visualizer.cpp
)
target_link_libraries(cloudline2plane_proj 
${PCL_LIBRARIES} 
${OpenCV_LIBRARIES}
/usr/lib/x86_64-linux-gnu/libyaml-cpp.so)

add_executable(visPlaneCloud
src/visPlaneCloud.cpp
src/visualizer.cpp
)
target_link_libraries(visPlaneCloud
${PCL_LIBRARIES} 
${OpenCV_LIBRARIES}
)

add_executable(cloudline2plane_kitti
src/cloudline2plane_kitti.cpp
src/line_clustering.cpp
src/line_extraction.cpp
src/line_feature.cpp
src/plane_extraction.cpp
src/visualizer.cpp
)

target_link_libraries(cloudline2plane_kitti
${catkin_LIBRARIES}
${PCL_LIBRARIES} 
${OpenCV_LIBRARIES}
/usr/lib/x86_64-linux-gnu/libyaml-cpp.so
)

