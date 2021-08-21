#include <quad_manipulator_control/utils.h>
#include <eigen_conversions/eigen_msg.h>
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

std::string ToString(const Eigen::Affine3d &pose, const std::string& frame_id) {
  geometry_msgs::PoseStamped target;
  tf::poseEigenToMsg(pose, target.pose);
  target.header.frame_id = frame_id;
  return ToString(target);
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

geometry_msgs::PoseStamped px4_ned_T_map(const geometry_msgs::PoseStamped& pose) {
  geometry_msgs::PoseStamped px4_ned_pose;
  px4_ned_pose.header.frame_id = "px4";
  px4_ned_pose.pose.position.x = pose.pose.position.y;
  px4_ned_pose.pose.position.y = pose.pose.position.x;
  px4_ned_pose.pose.position.z = -pose.pose.position.z;
  px4_ned_pose.pose.orientation = pose.pose.orientation;
  return px4_ned_pose;
}

KDL::Frame to_kdl(const Eigen::Affine3d &in) {
  const auto p = in.translation();
  const auto R = in.rotation();
  // clang-format off
  return KDL::Frame(KDL::Rotation(R(0, 0), R(0, 1), R(0, 2),
                                  R(1, 0), R(1, 1), R(1, 2),
                                  R(2, 0), R(2, 1), R(2, 2)),
                    KDL::Vector(p(0), p(1), p(2)));
  // clang-format on
}

Eigen::Affine3d from_kdl(const KDL::Frame &in) {
  double quat_x, quat_y, quat_z, quat_w;
  in.M.GetQuaternion(quat_x, quat_y, quat_z, quat_w);
  return Eigen::Affine3d(Eigen::Translation3d(Eigen::Vector3d(in.p.x(), in.p.y(), in.p.z())) *
                         Eigen::Quaterniond(quat_x, quat_y, quat_z, quat_w));
}

std::string get_qualified_frame(const ros::NodeHandle &nh, const std::string &frame) {
  std::string resolved_frame = nh.resolveName(frame);
  if (resolved_frame.at(0) == '/') {
    resolved_frame.erase(resolved_frame.begin());
  }
  return resolved_frame;
}