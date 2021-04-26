#include <quad_manipulator_control/quad_manipulator.h>
#include <quad_manipulator_control/utils.h>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

namespace {
const std::vector<std::pair<double, double>> kTestCases{{1.2, 0.4}, {1.2, 0.8}, {1.0, 0.4}, {1.0, 0.8}};
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
  for (const auto &test_case : kTestCases) {
    // 3- Enable reco
    if (!quad_manip.enable_reco()) {
      ROS_ERROR("Could not enable reco. Continuing with next test case");
      continue;
    }
    // 4 - Orbit around trees
    const double orbit_radius = test_case.first;
    const double orbit_velocity = test_case.second;
    const std::vector<double> orbit_heights{1.2, 2.5};

    for (const double &orbit_height : orbit_heights) {
      for (const std::pair<double, double> &tree_pose : trees_to_visit) {
        geometry_msgs::PoseStamped earth_T_tree;
        earth_T_tree.header.frame_id = "earth";
        earth_T_tree.pose.position.x = tree_pose.first;
        earth_T_tree.pose.position.y = tree_pose.second;
        earth_T_tree.pose.position.z = orbit_height;
        ROS_INFO("Orbitting around %s with radius %lf and velocity %lf...", ToString(earth_T_tree).c_str(),
                 orbit_radius, orbit_velocity);
        if (!quad_manip.orbit_around(earth_T_tree, orbit_radius, orbit_velocity)) {
          ROS_ERROR("Robot could not successfully orbit around %s", ToString(earth_T_tree).c_str());
          return -1;
        }
      }
      ROS_INFO("Finished orbit at height %lf", orbit_height);
      if (!quad_manip.go_to_home()) {
        ROS_ERROR("Could not go to home");
        return -1;
      }
    }
    // 5- Disable reco

    if (!quad_manip.disable_reco()) {
      ROS_ERROR("Could not disable reco. Failing");
      return -1;
    }
    // 6- Retrieve results
    std::ostringstream oss;
    oss << "Orbit Radius: " << orbit_radius << " - orbit velocity: " << orbit_velocity;
    if (!quad_manip.compute_reco_results(oss.str())) {
      ROS_ERROR("Could not receive reco results for '%s'", oss.str().c_str());
    }
  }

  return 0;
}