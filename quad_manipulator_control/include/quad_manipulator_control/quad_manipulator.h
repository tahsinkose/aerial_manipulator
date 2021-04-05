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

  bool takeoff();

  /**
   * @brief Moves the robot to passed in @p target_pose.
   * @param target_pose Target pose
   * @param position_tolerance Tolerance in Euclidean distance, defaults to 5 cms.
   * @returns True if the robot ends up within the tolerance volume of @p target_pose, false otherwise.
   */
  bool move_base(const geometry_msgs::PoseStamped& target_pose, double position_tolerance = 0.05);

private:
  mavros_msgs::State current_state_;
  geometry_msgs::PoseStamped local_pose_;
  ros::NodeHandle nh_;
  ros::Subscriber state_sub_;
  ros::Subscriber pose_sub_;
  ros::Publisher local_pos_pub_;
  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;
  ros::ServiceClient takeoff_client_;
  ros::ServiceClient landing_client_;
};