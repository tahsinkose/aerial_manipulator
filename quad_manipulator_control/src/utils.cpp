#include <quad_manipulator_control/utils.h>
#include <sstream>

std::string ToString(const geometry_msgs::PoseStamped &pose) {
  std::ostringstream oss;
  oss << "Point: [" << std::to_string(pose.pose.position.x) << ", "
      << std::to_string(pose.pose.position.y) << ", "
      << std::to_string(pose.pose.position.z) << "] - Quaternion: ["
      << std::to_string(pose.pose.orientation.x) << ", "
      << std::to_string(pose.pose.orientation.y) << ", "
      << std::to_string(pose.pose.orientation.z) << ", "
      << std::to_string(pose.pose.orientation.w)
      << "] in frame: " << pose.header.frame_id;
  return oss.str();
}

std::string ToString(const std::map<std::string, double>& joint_states) {
  std::ostringstream oss;
  oss << "Joint States: [";
  for(auto js : joint_states){
    oss << "(" << js.first << ") -> " << js.second <<"\n";
  }
  oss << "]";
  return oss.str();
}