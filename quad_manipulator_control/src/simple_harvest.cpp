#include <quad_manipulator_control/quad_manipulator.h>
#include <quad_manipulator_control/utils.h>

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

namespace {
const Eigen::Affine3d earth_T_apple(Eigen::Translation3d(Eigen::Vector3d(7.765, -0.08, 0.28)));
// A known pose with a good IK solution
const Eigen::Affine3d base_link_T_apple(Eigen::Translation3d(0.182947, 0.0, -0.227281));
}

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
  geometry_msgs::PoseStamped base_link_T_apple_ros;
  base_link_T_apple_ros.header.frame_id = "uav1/base_link";
  tf::poseEigenToMsg(base_link_T_apple, base_link_T_apple_ros.pose);
  const auto apple_T_base_link_in_earth = quad_manip.express_in_earth(base_link_T_apple_ros, true);
  if (!apple_T_base_link_in_earth) {
    ROS_ERROR("Could not compute apple_T_base_link in earth frame!");
    return -1;
  }
  ROS_INFO("apple_T_base_link: %s", ToString(*apple_T_base_link_in_earth, "earth").c_str());
  // 3- Move to pre-grab location
  geometry_msgs::PoseStamped earth_T_base_target_ros;
  Eigen::Affine3d earth_T_base_target = earth_T_apple * *apple_T_base_link_in_earth;
  ROS_INFO("earth_T_base_target: %s", ToString(earth_T_base_target, "earth").c_str());
  earth_T_base_target_ros.header.frame_id = "earth";
  tf::poseEigenToMsg(earth_T_base_target, earth_T_base_target_ros.pose);
  quad_manip.move_base(earth_T_base_target_ros, 0.05);
  // 4- Start picking sequence.
  ROS_INFO("Pick Apple...");
  if (!quad_manip.pick(earth_T_base_target_ros, base_link_T_apple_ros)) {
    ROS_ERROR("Failed to pick apple");
    return -1;
  }
  // 5- Dispose apple.
  ROS_INFO("Dispose Apple...");
  if (!quad_manip.dispose()) {
    ROS_ERROR("Failed to dispose apple");
    return -1;
  }
  return 0;
}