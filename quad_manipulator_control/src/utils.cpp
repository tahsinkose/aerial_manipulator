#include <quad_manipulator_control/utils.h>
#include <sstream>

#include <gazebo_msgs/SetModelState.h>
#include <ros/ros.h>
std::string ToString(const geometry_msgs::PoseStamped &pose) {
  std::ostringstream oss;
  oss << "Point: [" << std::to_string(pose.pose.position.x) << ", " << std::to_string(pose.pose.position.y) << ", "
      << std::to_string(pose.pose.position.z) << "] - Quaternion: [" << std::to_string(pose.pose.orientation.x) << ", "
      << std::to_string(pose.pose.orientation.y) << ", " << std::to_string(pose.pose.orientation.z) << ", "
      << std::to_string(pose.pose.orientation.w) << "] in frame: " << pose.header.frame_id;
  return oss.str();
}

std::string ToString(const geometry_msgs::TwistStamped &twist) {
  std::ostringstream oss;
  oss << "Linear: [" << std::to_string(twist.twist.linear.x) << ", " << std::to_string(twist.twist.linear.y) << ", "
      << std::to_string(twist.twist.linear.z) << "] - Angular: [" << std::to_string(twist.twist.angular.x) << ", "
      << std::to_string(twist.twist.angular.y) << ", " << std::to_string(twist.twist.angular.z)
      << "] in frame: " << twist.header.frame_id;
  return oss.str();
}

std::string ToString(const std::map<std::string, double> &joint_states) {
  std::ostringstream oss;
  oss << "Joint States: [";
  for (auto js : joint_states) {
    oss << "(" << js.first << ") -> " << js.second << "\n";
  }
  oss << "]";
  return oss.str();
}

bool set_gazebo_model_pose(const std::string &model_name, const geometry_msgs::Pose &pose) {
  gazebo_msgs::ModelState state_msg;
  state_msg.model_name = model_name;
  state_msg.pose.position.x = pose.position.x;
  state_msg.pose.position.y = pose.position.y;
  state_msg.pose.position.z = pose.position.z;
  state_msg.pose.orientation.x = pose.orientation.x;
  state_msg.pose.orientation.y = pose.orientation.y;
  state_msg.pose.orientation.z = pose.orientation.z;
  state_msg.pose.orientation.w = pose.orientation.w;
  ros::NodeHandle nh;

  auto gazebo_set_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  gazebo_msgs::SetModelState set_srv;
  set_srv.request.model_state = state_msg;
  return gazebo_set_client.call(set_srv) && set_srv.response.success;
}

geometry_msgs::PoseStamped px4_ned_T_gazebo_convert(const geometry_msgs::PoseStamped& pose) {
  geometry_msgs::PoseStamped px4_ned_pose;
  px4_ned_pose.header.frame_id = "px4";
  px4_ned_pose.pose.position.x = pose.pose.position.y;
  px4_ned_pose.pose.position.y = pose.pose.position.x;
  px4_ned_pose.pose.position.z = -pose.pose.position.z;
  px4_ned_pose.pose.orientation = pose.pose.orientation;
  return px4_ned_pose;
}