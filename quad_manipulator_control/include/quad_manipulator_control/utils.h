#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/buffer.h>
#include <Eigen/Geometry>
#include <kdl/frames.hpp>
#include <sstream>

std::string ToString(const geometry_msgs::PoseStamped &pose);
std::string ToString(const geometry_msgs::TwistStamped &pose);
std::string ToString(const Eigen::Affine3d &pose);
std::string ToString(const std::map<std::string, double> &joint_states);

bool set_gazebo_model_pose(const std::string &model_name, const geometry_msgs::Pose &pose);

geometry_msgs::PoseStamped px4_ned_T_map(const geometry_msgs::PoseStamped &pose);
KDL::Frame to_kdl(const Eigen::Affine3d &in);
Eigen::Affine3d from_kdl(const KDL::Frame &in);
std::string get_qualified_frame(const ros::NodeHandle &nh, const std::string &frame);