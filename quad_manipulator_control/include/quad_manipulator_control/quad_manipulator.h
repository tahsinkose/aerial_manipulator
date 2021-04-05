#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>

class QuadManipulator {
public:
  QuadManipulator();
  /**
   * @brief Receives state information of the robot, e.g. armed, mode.
   * @param msg mavros_msgs::State message representing the robot state.
   */
  void state_cb(const mavros_msgs::State::ConstPtr &msg);

  /**
   * @brief Receives the pose of the robot in local coordinate frame.
   * @param msg geometry_msgs::PoseStamped message representing the pose of the robot in local coordinate frame.
   */
  void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);

  /**
   * @brief Arms the robot.
   * @returns True if successfully armed, false otherwise
   */
  bool arm();

  /**
   * @brief Takes off the robot to a hardcoded altitude in local coordinate frame.
   * @returns True if successfully took off, false otherwise.
   */
  bool takeoff();

  /**
   * @brief Moves the robot to passed in @p target_pose.
   * @param target_pose Target pose
   * @param position_tolerance Tolerance in Euclidean distance, defaults to 5 cms.
   * @returns True if the robot ends up within the tolerance volume of @p target_pose, false otherwise.
   */
  bool move_base(const geometry_msgs::PoseStamped& target_pose, double position_tolerance = 0.05);

  /**
   * @brief Sends request and waits for specified mode, if the robot is already not in that mode.
   * @param mode String representation of mode.
   * @param timeout Timeout in seconds.
   * @returns True if the robot transitions into the specified @p mode in @p timeout seconds, false otherwise.
   */
  bool wait_for_mode(const std::string& mode,  const ros::Duration &timeout = ros::Duration(15));

  bool wait_for_armed(const ros::Duration &timeout = ros::Duration(15));

private:
  mavros_msgs::State current_state_;
  geometry_msgs::PoseStamped local_pose_;
  ros::NodeHandle nh_;
  ros::Subscriber state_sub_;
  ros::Subscriber pose_sub_;
  ros::Publisher local_pos_pub_;
  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;
  ros::ServiceClient landing_client_;
};