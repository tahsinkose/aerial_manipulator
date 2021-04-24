#include <quad_manipulator_control/quad_manipulator.h>
#include <quad_manipulator_control/utils.h>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

namespace {
constexpr const double kTreeOrbitRadius = 1.2;    // m
constexpr const double kTreeOrbitVelocity = 0.4;  // m/s
constexpr const char *kOrchardVisitParamName = "/orchard/visit";
}  // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "medium_orchard_patrol");
  ros::NodeHandle nh;
  std::vector<std::pair<double, double>> trees_to_visit;
  XmlRpc::XmlRpcValue two_d_vector;
  if (!nh.getParam(kOrchardVisitParamName, two_d_vector)) {
    ROS_ERROR("Missing parameter: '%s'", kOrchardVisitParamName);
    return -1;
  }
  for (int i = 0; i < two_d_vector.size(); i++) {
    ROS_ASSERT(two_d_vector[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
    if (two_d_vector[i].size() != 2) {
      ROS_ERROR("Parameter '%s' has non-pair entry. Not-allowed..", kOrchardVisitParamName);
      return -1;
    }
    ROS_ASSERT(two_d_vector[i][0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    ROS_ASSERT(two_d_vector[i][1].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    trees_to_visit.push_back({two_d_vector[i][0], two_d_vector[i][1]});
  }

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

  // 3 - Orbit around trees, lower halves
  for (const std::pair<double, double> &tree_pose : trees_to_visit) {
    geometry_msgs::PoseStamped earth_T_tree;
    earth_T_tree.header.frame_id = "earth";
    earth_T_tree.pose.position.x = tree_pose.first;
    earth_T_tree.pose.position.y = tree_pose.second;
    earth_T_tree.pose.position.z = 1.25;
    ROS_INFO("Orbitting around %s...", ToString(earth_T_tree).c_str());
    if (!quad_manip.orbit_around(earth_T_tree, kTreeOrbitRadius, kTreeOrbitVelocity)) {
      ROS_ERROR("Robot could not successfully orbit around %s", ToString(earth_T_tree).c_str());
    }
  }
  // 4 - Orbit around trees, upper halves
  for (const std::pair<double, double> &tree_pose : trees_to_visit) {
    geometry_msgs::PoseStamped earth_T_tree;
    earth_T_tree.header.frame_id = "earth";
    earth_T_tree.pose.position.x = tree_pose.first;
    earth_T_tree.pose.position.y = tree_pose.second;
    earth_T_tree.pose.position.z = 2.5;
    ROS_INFO("Orbitting around %s...", ToString(earth_T_tree).c_str());
    if (!quad_manip.orbit_around(earth_T_tree, kTreeOrbitRadius, kTreeOrbitVelocity)) {
      ROS_ERROR("Robot could not successfully orbit around %s", ToString(earth_T_tree).c_str());
    }
  }
  ROS_INFO("Finished orbit");
  if (!quad_manip.go_to_home()) {
    ROS_ERROR("Could not go to home");
    return -1;
  }
  return 0;
}