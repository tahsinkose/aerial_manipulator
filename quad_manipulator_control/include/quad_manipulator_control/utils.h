#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <sstream>

#include <tf2_ros/buffer.h>

std::string ToString(const geometry_msgs::PoseStamped& pose);
std::string ToString(const geometry_msgs::TwistStamped& pose);
std::string ToString(const std::map<std::string, double>& joint_states);

bool set_gazebo_model_pose(const std::string &model_name, const geometry_msgs::Pose &pose);

geometry_msgs::PoseStamped px4_ned_T_map(const geometry_msgs::PoseStamped& pose);