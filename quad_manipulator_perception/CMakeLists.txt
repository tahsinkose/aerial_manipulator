cmake_minimum_required(VERSION 3.0.2)
project(quad_manipulator_perception)

find_package(catkin REQUIRED)

catkin_package()

include_directories()


install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})