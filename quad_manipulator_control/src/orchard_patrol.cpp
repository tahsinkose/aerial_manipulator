#include <quad_manipulator_control/quad_manipulator.h>
#include <quad_manipulator_control/utils.h>

#include <geometry_msgs/PoseStamped.h>
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
  geometry_msgs::PoseStamped earth_T_tree;
  earth_T_tree.header.frame_id = "earth";
  earth_T_tree.pose.position.z = 2.5;
  if (!quad_manip.orbit_around(earth_T_tree, 1.0, 0.4)) {
    ROS_ERROR("Robot could not successfully orbit around %s", ToString(earth_T_tree).c_str());
    return -1;
  }

  geometry_msgs::PoseStamped earth_T_tree2;
  earth_T_tree2.header.frame_id = "earth";
  earth_T_tree2.pose.position.z = 2.5;
  earth_T_tree2.pose.position.x = -2.0;
  if (!quad_manip.orbit_around(earth_T_tree2, 1.0, 0.4)) {
    ROS_ERROR("Robot could not successfully orbit around %s", ToString(earth_T_tree).c_str());
    return -1;
  }
  return 0;
}