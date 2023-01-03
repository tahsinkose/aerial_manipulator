#include <quad_manipulator_control/quad_manipulator.h>
#include <quad_manipulator_control/utils.h>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>


int main(int argc, char **argv) {
  ros::init(argc, argv, "simple_fly_test");
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
  // 2 - Takeoff
  ROS_INFO("Takeoff...");
  if (!quad_manip.takeoff()) {
    ROS_ERROR("Robot could not takeoff");
    return -1;
  }
  ROS_INFO("Returning home...");
  if (!quad_manip.go_to_home()) {
    ROS_ERROR("Could not go to home: ");
    return -1;
  }

  return 0;
}