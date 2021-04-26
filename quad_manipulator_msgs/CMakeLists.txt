cmake_minimum_required(VERSION 3.0.2)
project(quad_manipulator_msgs)

find_package(catkin REQUIRED)

set(PACKAGE_DEPENDENCIES
  gazebo_msgs
  geometry_msgs
  std_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${PACKAGE_DEPENDENCIES}
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
    RobotCollisions.msg
    FakeGraspingEvent.msg
    Detection.msg
    Detections.msg
)


add_service_files(
  DIRECTORY srv
  FILES
    RecoStats.srv
    RecoCmd.srv
)

generate_messages(
  DEPENDENCIES ${PACKAGE_DEPENDENCIES}
  LANGS gencpp genpy
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES} message_runtime
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)