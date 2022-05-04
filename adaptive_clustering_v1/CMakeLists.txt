cmake_minimum_required(VERSION 2.8.3)
project(adaptive_clustering_v1)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs visualization_msgs geometry_msgs pcl_conversions pcl_ros)

find_package(PCL REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})

add_message_files(FILES ClusterArray.msg)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package()

add_executable(adaptive_clustering_v1 src/adaptive_clustering_v1.cpp)
target_link_libraries(adaptive_clustering_v1 ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(adaptive_clustering_v1 ${catkin_EXPORTED_TARGETS})
endif()
