#pragma once
#include <stdint.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/OrbitStatus.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class QuadManipulator {
 public:
  QuadManipulator();

  /**
   * @brief Waits for connection with PX4 autopilot.
   * @param timeout Timeout in seconds.
   * @returns True if successfully connected, false otherwise
   */
  bool wait_for_connection(const ros::WallDuration &timeout = ros::WallDuration(45));

  /**
   * @brief Arms the robot.
   * @param arm Whether to arm/disarm.
   * @returns True if successfully armed, false otherwise
   */
  bool motor_arm(bool arm);

  /**
   * @brief Takes off the robot to a hardcoded altitude in local coordinate
   * frame.
   * @returns True if successfully took off, false otherwise.
   */
  bool takeoff();

  /**
   * @brief Lands the robot to ground below
   * @returns True if successfully landed, false otherwise.
   */
  bool land();

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
  bool close_gripper(const std::vector<double> &desired = {-0.4, 0.4},
                     const ros::Duration &timeout = ros::Duration(15));

  /**
   * @brief Orbits around the setpoint @p pose.
   * @param pose Pose in any frame convertible to vehicle's map frame.
   * @param orbit_radius Radius of the orbit.
   * @param velocity_during_orbit Velocity during the orbit.
   * @return True if it was possible to start orbitting.
   */
  bool orbit_around(const geometry_msgs::PoseStamped &pose, double orbit_radius = 2.0,
                    double velocity_during_orbit = 0.5);
  /**
   * @brief Moves the robot to 'Home' location.
   * @returns True if the robot went to 'Home', false otherwise.
   */
  bool go_to_home();

 private:
  /**
   * @brief Receives state information of the robot, e.g. armed, mode.
   * @param msg mavros_msgs::State message representing the robot state.
   */
  void state_cb(const mavros_msgs::State::ConstPtr &msg);

  /**
   * @brief Receives extended state information of the robot, e.g. landed, vtol.
   * @param msg mavros_msgs::ExtendedState message representing the extended robot state.
   */
  void extended_state_cb(const mavros_msgs::ExtendedState::ConstPtr &msg);

  /**
   * @brief Receives the pose of the robot in local coordinate frame.
   * @param msg geometry_msgs::PoseStamped message representing the pose of the
   * robot in local coordinate frame.
   */
  void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);

  /**
   * @brief Receives the velocity of the robot in local coordinate frame.
   * @param msg geometry_msgs::TwistStamped message representing the velocity of the
   * robot in local coordinate frame.
   */
  void local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);

  /**
   * @brief Receives the execution state of orbit action.
   * @param msg mavros_msgs::OrbitStatus message representing the current execution state of the vehicle on orbit.
   */
  void orbit_state_cb(const mavros_msgs::OrbitStatus::ConstPtr &msg);

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
  bool wait_for_mode(const std::string &mode, uint8_t base_mode = mavros_msgs::SetModeRequest::MAV_MODE_PREFLIGHT,
                     const ros::Duration &timeout = ros::Duration(15));

  /**
   * @brief Waits for armed status, if not already armed.
   * @param arm Whether to arm/disarm.
   * @param timeout Timeout in seconds.
   * @returns True if the robot transitions into the armed status in @p timeout
   * seconds, false otherwise.
   */
  bool wait_for_armed(bool arm, const ros::Duration &timeout = ros::Duration(15));

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

  /**
   * @brief Checks whether the base of the vehicle is stabilized or not.
   * @returns true if it is, false otherwise.
   */
  bool base_not_moving();

  /**
   * @brief Transforms the @p target_pose into map frame.
   * @param target_pose Target pose in any expressible frame wrt. map frame.
   * @returns geometry_msgs::PoseStamped in map frame if it was possible to transform, std::nullopt otherwise.
   */
  std::optional<geometry_msgs::PoseStamped> target_in_map(const geometry_msgs::PoseStamped &target_pose);

  /**
   * @brief Returns landed status of the robot
   */
  bool is_landed();

 private:
  mavros_msgs::State current_state_;
  mavros_msgs::ExtendedState extended_current_state_;
  geometry_msgs::PoseStamped local_pose_;
  geometry_msgs::TwistStamped local_velocity_;
  std::map<std::string, double> joint_states_;
  std::optional<mavros_msgs::OrbitStatus::_state_type> orbit_state_;

  ros::NodeHandle nh_;
  ros::Subscriber mav_state_sub_;
  ros::Subscriber extended_state_sub_;
  ros::Subscriber mav_local_pose_sub_;
  ros::Subscriber mav_local_vel_sub_;
  ros::Subscriber joint_states_sub_;
  ros::Subscriber orbit_execution_state_sub_;


  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;
  ros::ServiceClient landing_client_;
  ros::ServiceClient command_client_;

  ros::Publisher local_pos_pub_;
  ros::Publisher arm_control_pub_;
  ros::Publisher hand_control_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string map_frame_;
};