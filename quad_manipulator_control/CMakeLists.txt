cmake_minimum_required(VERSION 2.8.3)
project(quad_manipulator_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  tf
  message_generation
  std_msgs
  mavros_msgs
  nav_msgs
  cmake_modules
  cv_bridge
  image_transport
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(manipulator_velocity_node src/sandbox/manipulator_velocity_node.cpp)
target_link_libraries(manipulator_velocity_node ${catkin_LIBRARIES})

add_executable(quad_velocity_control src/sandbox/quad_velocity_control.cpp)
target_link_libraries(quad_velocity_control ${catkin_LIBRARIES})

add_executable(quad_manipulator_startup src/sandbox/quad_manipulator_startup.cpp)
target_link_libraries(quad_manipulator_startup ${catkin_LIBRARIES})

add_executable(listener_arm_pose src/sandbox/listener_arm_pose.cpp)
target_link_libraries(listener_arm_pose ${catkin_LIBRARIES})

add_executable(broadcaster_base_world src/sandbox/broadcaster_base_world.cpp)
target_link_libraries(broadcaster_base_world ${catkin_LIBRARIES})

add_library(quad_manipulator src/quad_manipulator.cpp)
target_link_libraries(quad_manipulator ${catkin_LIBRARIES})

add_executable(simple_pick_fly_dispose src/simple_pick_fly_dispose.cpp)
target_link_libraries(simple_pick_fly_dispose quad_manipulator)


install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})