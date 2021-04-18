#include <geometry_msgs/PoseStamped.h>
#include <quad_manipulator_control/quad_manipulator.h>
#include <quad_manipulator_control/utils.h>

#include <ros/ros.h>
int main(int argc, char **argv) {
  ros::init(argc, argv, "global_orchard_search");
  ros::NodeHandle nh;
  QuadManipulator quad_manip;
  // 1 - Connect
  ROS_INFO("Connecting...");
  if (!quad_manip.wait_for_connection()) {
    ROS_ERROR("Robot could connect to PX4 board");
    return -1;
  }
  if (!quad_manip.motor_arm(true)) {
    ROS_ERROR("Robot could not be armed");
    return -1;
  }
  if (!quad_manip.takeoff()) {
    ROS_ERROR("Robot could not takeoff");
    return -1;
  }
  return 0;
}