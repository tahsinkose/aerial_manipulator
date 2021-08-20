#! /bin/bash
echo "*******"
echo "Preparing specific variables for the Quad Manipulator Stack"
echo "*******"

export ROS_TEST_RESULTS_DIR="$(pwd)/test_results"
export ROS_LOG_DIR="$(pwd)/ros_logs"
# PX4 stuff
HOME_DIR=$(pwd)
SRC_DIR=$(pwd)/src/quad_manipulator/Firmware
BUILD_DIR=$SRC_DIR/build/px4_sitl_default

# setup Gazebo env and update package path
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:${HOME_DIR}/devel/lib:${BUILD_DIR}/build_gazebo/devel/lib
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${SRC_DIR}/Tools/sitl_gazebo/models
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${BUILD_DIR}/build_gazebo/devel/lib
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$SRC_DIR:$SRC_DIR/Tools/sitl_gazebo
