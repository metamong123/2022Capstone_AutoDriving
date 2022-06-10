# Install script for directory: /home/mbsnscl/map_ws/src/frenet_frame-and-stanley-in-rviz/src/custom_msgs/object_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/mbsnscl/map_ws/src/frenet_frame-and-stanley-in-rviz/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/object_msgs/msg" TYPE FILE FILES
    "/home/mbsnscl/map_ws/src/frenet_frame-and-stanley-in-rviz/src/custom_msgs/object_msgs/msg/Object.msg"
    "/home/mbsnscl/map_ws/src/frenet_frame-and-stanley-in-rviz/src/custom_msgs/object_msgs/msg/Prediction.msg"
    "/home/mbsnscl/map_ws/src/frenet_frame-and-stanley-in-rviz/src/custom_msgs/object_msgs/msg/Control.msg"
    "/home/mbsnscl/map_ws/src/frenet_frame-and-stanley-in-rviz/src/custom_msgs/object_msgs/msg/Trajectory.msg"
    "/home/mbsnscl/map_ws/src/frenet_frame-and-stanley-in-rviz/src/custom_msgs/object_msgs/msg/trajectory_array.msg"
    "/home/mbsnscl/map_ws/src/frenet_frame-and-stanley-in-rviz/src/custom_msgs/object_msgs/msg/trajectory.msg"
    "/home/mbsnscl/map_ws/src/frenet_frame-and-stanley-in-rviz/src/custom_msgs/object_msgs/msg/Predictions.msg"
    "/home/mbsnscl/map_ws/src/frenet_frame-and-stanley-in-rviz/src/custom_msgs/object_msgs/msg/DebugPrediction.msg"
    "/home/mbsnscl/map_ws/src/frenet_frame-and-stanley-in-rviz/src/custom_msgs/object_msgs/msg/PlanningDebug.msg"
    "/home/mbsnscl/map_ws/src/frenet_frame-and-stanley-in-rviz/src/custom_msgs/object_msgs/msg/Polygon.msg"
    "/home/mbsnscl/map_ws/src/frenet_frame-and-stanley-in-rviz/src/custom_msgs/object_msgs/msg/PolygonArray.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/object_msgs/cmake" TYPE FILE FILES "/home/mbsnscl/map_ws/src/frenet_frame-and-stanley-in-rviz/build/custom_msgs/object_msgs/catkin_generated/installspace/object_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/mbsnscl/map_ws/src/frenet_frame-and-stanley-in-rviz/devel/include/object_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/mbsnscl/map_ws/src/frenet_frame-and-stanley-in-rviz/devel/share/roseus/ros/object_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/mbsnscl/map_ws/src/frenet_frame-and-stanley-in-rviz/devel/share/common-lisp/ros/object_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/mbsnscl/map_ws/src/frenet_frame-and-stanley-in-rviz/devel/share/gennodejs/ros/object_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/mbsnscl/map_ws/src/frenet_frame-and-stanley-in-rviz/devel/lib/python2.7/dist-packages/object_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/mbsnscl/map_ws/src/frenet_frame-and-stanley-in-rviz/devel/lib/python2.7/dist-packages/object_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/mbsnscl/map_ws/src/frenet_frame-and-stanley-in-rviz/build/custom_msgs/object_msgs/catkin_generated/installspace/object_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/object_msgs/cmake" TYPE FILE FILES "/home/mbsnscl/map_ws/src/frenet_frame-and-stanley-in-rviz/build/custom_msgs/object_msgs/catkin_generated/installspace/object_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/object_msgs/cmake" TYPE FILE FILES
    "/home/mbsnscl/map_ws/src/frenet_frame-and-stanley-in-rviz/build/custom_msgs/object_msgs/catkin_generated/installspace/object_msgsConfig.cmake"
    "/home/mbsnscl/map_ws/src/frenet_frame-and-stanley-in-rviz/build/custom_msgs/object_msgs/catkin_generated/installspace/object_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/object_msgs" TYPE FILE FILES "/home/mbsnscl/map_ws/src/frenet_frame-and-stanley-in-rviz/src/custom_msgs/object_msgs/package.xml")
endif()

