#include <quad_manipulator_control/quad_manipulator.h>
#include <quad_manipulator_control/utils.h>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>


namespace{
constexpr const double kTreeOrbitRadius = 1.0; // m
constexpr const double kTreeOrbitVelocity = 0.6; // m/s
}

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

  geometry_msgs::PoseStamped earth_T_tree1;
  earth_T_tree1.header.frame_id = "earth";
  earth_T_tree1.pose.position.x = -5;
  earth_T_tree1.pose.position.y = -2;
  earth_T_tree1.pose.position.z = 1.5;
  ROS_INFO("Orbitting around %s...", ToString(earth_T_tree1).c_str());
  if (!quad_manip.orbit_around(earth_T_tree1, kTreeOrbitRadius, kTreeOrbitVelocity)) {
    ROS_ERROR_STREAM("Robot could not successfully orbit around " << ToString(earth_T_tree1));
  }

  ROS_INFO("Returning home...");
  if (!quad_manip.go_to_home()) {
    ROS_ERROR("Could not go to home: ");
    return -1;
  }

  return 0;
}