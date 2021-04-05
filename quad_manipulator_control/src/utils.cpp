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