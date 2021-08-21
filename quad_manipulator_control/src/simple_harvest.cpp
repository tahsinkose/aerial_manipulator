#include <quad_manipulator_control/quad_manipulator.h>
#include <quad_manipulator_control/utils.h>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "simple_harvest");
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
  ROS_INFO("Pick Apple...");
  geometry_msgs::PoseStamped base_link_T_apple;
  base_link_T_apple.header.frame_id = "uav1/base_link";
  base_link_T_apple.pose.position.x = 0.2;
  base_link_T_apple.pose.position.z = -0.25;
  base_link_T_apple.pose.orientation.w = 1.0;
  quad_manip.move_eef(base_link_T_apple);
  return 0;
}