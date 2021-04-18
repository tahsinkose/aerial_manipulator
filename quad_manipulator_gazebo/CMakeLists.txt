cmake_minimum_required(VERSION 2.8.3)
project(quad_manipulator_gazebo)

set(PACKAGE_DEPENDENCIES
  roscpp
  gazebo_dev
  gazebo_plugins
  quad_manipulator_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${PACKAGE_DEPENDENCIES}
)

find_package(GAZEBO REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    gazebo_ros_robot_collisions
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS
    GAZEBO
)

include_directories(include ${catkin_INCLUDE_DIRS} /usr/include/gazebo-9/)

set(CMAKE_CXX_STANDARD 14)

add_compile_options(
  -Wall
  -Werror
  -pedantic
)

add_library(${PROJECT_NAME}_dependencies INTERFACE)
target_link_libraries(${PROJECT_NAME}_dependencies INTERFACE
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${OGRE_LIBRARIES}
  -Wl,--no-undefined
)

target_include_directories(${PROJECT_NAME}_dependencies SYSTEM INTERFACE
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
)

target_include_directories(${PROJECT_NAME}_dependencies INTERFACE
  include
)

add_dependencies(${PROJECT_NAME}_dependencies
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_library(gazebo_ros_robot_collisions src/robot_collisions.cpp src/fake_grasping_gripper.cpp)
target_link_libraries(gazebo_ros_robot_collisions ${PROJECT_NAME}_dependencies)

catkin_add_env_hooks (custom_env SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY Media
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)