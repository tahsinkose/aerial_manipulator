cmake_minimum_required(VERSION 2.8.3)
project(quad_manipulator_control)

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
  trajectory_msgs
  sensor_msgs
  eigen_conversions
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(CMAKE_CXX_STANDARD 17)

add_compile_options(
  -Wall
  -Werror
  -pedantic
)


#add_executable(manipulator_velocity_node src/sandbox/manipulator_velocity_node.cpp)
#target_link_libraries(manipulator_velocity_node ${catkin_LIBRARIES})
#
#add_executable(broadcaster_base_world src/sandbox/broadcaster_base_world.cpp)
#target_link_libraries(broadcaster_base_world ${catkin_LIBRARIES})

add_library(quad_manipulator src/quad_manipulator.cpp src/utils.cpp)
target_link_libraries(quad_manipulator ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_simple_pick_fly_dispose test/simple_pick_fly_dispose.test
    src/simple_pick_fly_dispose.cpp)
  target_link_libraries(test_simple_pick_fly_dispose quad_manipulator ${catkin_LIBRARIES})

  add_rostest_gtest(test_simple_orchard_patrol test/simple_orchard_patrol.test
    src/simple_orchard_patrol.cpp)
  target_link_libraries(test_simple_orchard_patrol quad_manipulator ${catkin_LIBRARIES})
endif()

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})