#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

class QuadManipulator {
 public:
  QuadManipulator();

  /**
   * @brief Waits for connection with PX4 autopilot.
   * @returns True if successfully connected, false otherwise
   */
  bool wait_for_connection();

  /**
   * @brief Arms the robot.
   * @returns True if successfully armed, false otherwise
   */
  bool motor_arm();

  /**
   * @brief Takes off the robot to a hardcoded altitude in local coordinate
   * frame.
   * @returns True if successfully took off, false otherwise.
   */
  bool takeoff();

  /**
   * @brief Moves the robot to passed in @p target_pose.
   * @param target_pose Target pose
   * @param position_tolerance Tolerance in Euclidean distance, defaults to 5
   * cms.
   * @returns True if the robot ends up within the tolerance volume of @p
   * target_pose, false otherwise.
   */
  bool move_base(const geometry_msgs::PoseStamped &target_pose, double position_tolerance = 0.05);

  /**
   * @brief Moves the manipulator to @p desired setpoint.
   * @param desired Goal joint positions.
   * @returns True if it was possible to move the robot to desired setpoint,
   * false otherwise.
   */
  bool move_manip(const std::vector<double> &desired);

  /**
   * @brief Opens the gripper to the limits.
   * @param timeout Timeout in seconds.
   * @returns True if it was possible to open the gripper.
   * false otherwise.
   */
  bool open_gripper(const ros::Duration &timeout = ros::Duration(15));

  /**
   * @brief Closes the gripper with desired amount.
   * @param desired Goal joint positions for grippers, defaults to {-0.35,
   * 0.35}.
   * @param timeout Timeout in seconds.
   * @returns True if it was possible to close the gripper.
   * false otherwise.
   */
  bool close_gripper(const std::vector<double> &desired = {-0.35, 0.35},
                     const ros::Duration &timeout = ros::Duration(15));

 private:
  /**
   * @brief Receives state information of the robot, e.g. armed, mode.
   * @param msg mavros_msgs::State message representing the robot state.
   */
  void state_cb(const mavros_msgs::State::ConstPtr &msg);

  /**
   * @brief Receives the pose of the robot in local coordinate frame.
   * @param msg geometry_msgs::PoseStamped message representing the pose of the
   * robot in local coordinate frame.
   */
  void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);

  /**
   * @brief Receives the joint states of the robot.
   */
  void joint_states_cb(const sensor_msgs::JointState::ConstPtr &msg);

  /**
   * @brief Sends request and waits for specified mode, if the robot is already
   * not in that mode.
   * @param mode String representation of mode.
   * @param timeout Timeout in seconds.
   * @returns True if the robot transitions into the specified @p mode in @p
   * timeout seconds, false otherwise.
   */
  bool wait_for_mode(const std::string &mode, const ros::Duration &timeout = ros::Duration(15));

  /**
   * @brief Waits for armed status, if not already armed.
   * @param timeout Timeout in seconds.
   * @returns True if the robot transitions into the armed status in @p timeout
   * seconds, false otherwise.
   */
  bool wait_for_armed(const ros::Duration &timeout = ros::Duration(15));

  /**
   * @brief Check whether the specified @p joint_names of manipulator is in @p desired pose.
   * @param joint_names Name of the joints to check for in position.
   * @param desired Desired joint configurations.
   * @returns True if the respective joints of manipulator is in @p desired pose, false otherwise.
   */
  bool in_pose(const std::vector<std::string> &joint_names, const std::vector<double> &desired);

  /**
   * @brief Moves the gripper to @p desired joint configuration.
   * @param desired Desired joint configurations.
   * @param timeout Timeout in seconds.
   * @returns True if it was possible to move the gripper to desired setpoint,
   * false otherwise.
   */
  bool cmd_gripper(const std::vector<double> &desired, const ros::Duration &timeout = ros::Duration(15));

  /**
   * @brief Moves the manipulator to @p desired joint configuration.
   * @param desired Desired joint configurations.
   * @param timeout Timeout in seconds.
   * @returns True if it was possible to move the manipulator to desired setpoint,
   * false otherwise.
   */
  bool cmd_manip(const std::vector<double> &desired, const ros::Duration &timeout = ros::Duration(15));

 private:
  mavros_msgs::State current_state_;
  geometry_msgs::PoseStamped local_pose_;
  std::map<std::string, double> joint_states_;

  ros::NodeHandle nh_;
  ros::Subscriber mav_state_sub_;
  ros::Subscriber mav_local_pose_sub_;
  ros::Subscriber joint_states_sub_;

  ros::Publisher local_pos_pub_;
  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;
  ros::ServiceClient landing_client_;

  ros::Publisher arm_control_pub_;
  ros::Publisher hand_control_pub_;
};