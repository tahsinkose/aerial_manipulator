#include <quad_manipulator_control/quad_manipulator.h>
#include <quad_manipulator_control/utils.h>

#include <eigen_conversions/eigen_msg.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/WaypointPush.h>
#include <tf/transform_listener.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <vector>

namespace {
constexpr const char *kOffboardMode = "OFFBOARD";
constexpr const char *kAutoLoiterMode = "AUTO.LOITER";
constexpr const char *kAutoMissionMode = "AUTO.MISSION";
constexpr const char *kStabilizedMode = "STABILIZED";
constexpr double kDefaultTakeoffAltitude = 1.5;  // in meters.
constexpr double kJointTolerance = 1e-2;         // in rad.
constexpr double kLinearVelTolerance = 1e-2;     // in meters
constexpr double kAngularVelTolerance = 3e-2;    // in rad
const std::vector<std::string> kValidFlightModes{"MANUAL",        "ACRO",         "ALTCTL",    "POSCTL",
                                                 kOffboardMode,   kStabilizedMode,   "RATTITUDE", kAutoMissionMode,
                                                 kAutoLoiterMode, "AUTO.TAKEOFF", "AUTO.RTL",  "AUTO.LAND",
                                                 "AUTO.RTGS",     "AUTO.READY",   "GUIDED"};

const std::vector<std::string> kArmJointNames{"arm_joint_1", "arm_joint_2", "arm_joint_3"};
const std::vector<std::string> kGraspJointNames{"grasp_joint_1", "grasp_joint_2"};
constexpr const int kLocalNedFrame = 1;  // MAV_FRAME::MAV_FRAME_LOCAL_NED
constexpr const int kMavOrbitCmd = 34;   // MAV_CMD::MAV_CMD_DO_ORBIT
constexpr const char *kMapFrame = "map";
}  // namespace

QuadManipulator::QuadManipulator() {
  mav_state_sub_ = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &QuadManipulator::state_cb, this);
  mav_local_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 100,
                                                                  &QuadManipulator::local_pose_cb, this);
  mav_local_vel_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 100,
                                                                  &QuadManipulator::local_vel_cb, this);
  // These are all we need for now.
  for (const auto &jn : kArmJointNames) {
    joint_states_[jn] = 0.0;
  }
  for (const auto &jn : kGraspJointNames) {
    joint_states_[jn] = 0.0;
  }

  joint_states_sub_ =
      nh_.subscribe<sensor_msgs::JointState>("joint_states", 100, &QuadManipulator::joint_states_cb, this);
  local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

  arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  landing_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  command_client_ = nh_.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");
  arm_control_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("arm_controller/command", 10);
  hand_control_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("grasp_controller/command", 10);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void QuadManipulator::state_cb(const mavros_msgs::State::ConstPtr &msg) {
  current_state_ = *msg;
  ROS_DEBUG("Received current state with mode %s", msg->mode.c_str());
}

void QuadManipulator::local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  local_pose_ = *msg;
  ROS_DEBUG("Received current pose: %s", ToString(local_pose_).c_str());
}

void QuadManipulator::local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg) {
  local_velocity_ = *msg;
  ROS_DEBUG("Received current velocity: %s", ToString(local_velocity_).c_str());
}

void QuadManipulator::joint_states_cb(const sensor_msgs::JointState::ConstPtr &msg) {
  for (size_t i = 0; i < msg->name.size(); i++) {
    const std::string &jn = msg->name[i];
    if (joint_states_.count(jn) != 0) {
      joint_states_[jn] = msg->position[i];
    }
  }
  ROS_DEBUG("Updated joint states: %s", ToString(joint_states_).c_str());
}

bool QuadManipulator::wait_for_connection(const ros::WallDuration &timeout) {
  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);
  // wait for FCU connection
  ros::WallTime init = ros::WallTime::now();
  ros::WallTime finish_time = init + timeout;
  while (ros::ok() && current_state_.connected != true && ros::WallTime::now() < finish_time) {
    ros::spinOnce();
    rate.sleep();
  }
  if (!current_state_.connected) {
    ROS_ERROR("Couldn't connect to the board!");
    return false;
  } else {
    ROS_INFO("Connected to the board!");
  }
  return true;
}

bool QuadManipulator::motor_arm(bool arm) {
  ros::Rate rate(20.0);
  geometry_msgs::PoseStamped dummy;

  // send a few setpoints before starting
  for (int i = 100; ros::ok() && i > 0; --i) {
    local_pos_pub_.publish(dummy);
    ros::spinOnce();
    rate.sleep();
  }

  if (!wait_for_mode(kOffboardMode)) {
    return false;
  }
  if (!wait_for_armed(arm)) {
    return false;
  }
  return true;
}

bool QuadManipulator::takeoff() {
  geometry_msgs::PoseStamped local_T_takeoff;
  local_T_takeoff.pose.position.x = 0;
  local_T_takeoff.pose.position.y = 0;
  local_T_takeoff.pose.position.z = kDefaultTakeoffAltitude;
  return move_base(local_T_takeoff);
}

bool QuadManipulator::land() {
  mavros_msgs::CommandTOL srv_land;
  return landing_client_.call(srv_land) && srv_land.response.success;
}

bool QuadManipulator::move_base(const geometry_msgs::PoseStamped &target_pose, double position_tolerance) {
  // Make sure we are in offboard mode.
  if (!wait_for_mode(kOffboardMode)) {
    return false;
  }

  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);
  double dist = DBL_MAX;
  ROS_INFO("Going to pose: %lf, %lf, %lf in frame '%s'", target_pose.pose.position.x, target_pose.pose.position.y,
           target_pose.pose.position.z, target_pose.header.frame_id.c_str());
  do {
    local_pos_pub_.publish(target_pose);

    ros::spinOnce();
    rate.sleep();
    dist = sqrt(pow((local_pose_.pose.position.x - target_pose.pose.position.x), 2) +
                pow((local_pose_.pose.position.y - target_pose.pose.position.y), 2) +
                pow((local_pose_.pose.position.z - target_pose.pose.position.z), 2));
  } while (ros::ok() && dist > position_tolerance);

  if (!ros::ok()) {
    return false;
  }

  // Transition back to auto loiter mode.
  return wait_for_mode(kAutoLoiterMode);
}

bool QuadManipulator::wait_for_mode(const std::string &mode, uint8_t base_mode, const ros::Duration &timeout) {
  if (std::find(kValidFlightModes.begin(), kValidFlightModes.end(), mode) == kValidFlightModes.end()) {
    ROS_ERROR("Unknown flight mode %s. Failing!", mode.c_str());
    return false;
  }

  if (current_state_.mode == mode) {
    ROS_INFO("Robot is already in %s mode", mode.c_str());
    return true;
  }

  mavros_msgs::SetMode set_mode;
  set_mode.request.base_mode = base_mode;
  set_mode.request.custom_mode = mode;
  if (set_mode_client_.call(set_mode) && set_mode.response.mode_sent) {
    ROS_INFO("Sent %s mode request.", mode.c_str());
  } else {
    ROS_ERROR("Could not sent %s mode request!", mode.c_str());
    return false;
  }

  ros::Rate rate(20.0);
  ros::Time start_time = ros::Time::now();
  ros::Time finish_time = start_time + timeout;
  while (ros::ok() && current_state_.mode != mode && ros::Time::now() < finish_time) {
    ROS_DEBUG("Current mode: %s. Pending until %s mode..", current_state_.mode.c_str(), mode.c_str());
    set_mode_client_.call(set_mode);
    ros::spinOnce();
    rate.sleep();
    geometry_msgs::PoseStamped dummy;
    local_pos_pub_.publish(dummy);
  }
  const bool result = current_state_.mode == mode;
  ROS_INFO("Finishing %s in %s mode ", (result ? "successfully" : "unsuccessfully"), current_state_.mode.c_str());
  return result;
}

bool QuadManipulator::wait_for_armed(bool arm, const ros::Duration &timeout) {
  mavros_msgs::CommandBool arming_cmd;
  arming_cmd.request.value = arm;
  if (current_state_.armed) {
    ROS_INFO("Robot is already %s!", (arm ? "armed" : "disarmed"));
    return true;
  }

  if (arming_client_.call(arming_cmd) && arming_cmd.response.success) {
    ROS_INFO("Vehicle motor_%s request is sent!", (arm ? "armed" : "disarmed"));
  } else {
    ROS_ERROR("Vehicle motor_%s request could not be sent. Failing..", (arm ? "armed" : "disarmed"));
    return false;
  }
  ros::Rate rate(20.0);
  ros::Time start_time = ros::Time::now();
  ros::Time finish_time = start_time + timeout;
  ROS_INFO("Waiting for %s status..", (arm ? "armed" : "disarmed"));
  while (ros::ok() && current_state_.armed != true && ros::Time::now() < finish_time) {
    ros::spinOnce();
    rate.sleep();
  }
  const bool result = current_state_.armed;
  ROS_INFO("%s %s", (arm ? "armed" : "disarmed"), (result ? "successfully" : "unsuccessfully"));
  return result;
}

bool QuadManipulator::in_pose(const std::vector<std::string> &joint_names, const std::vector<double> &desired) {
  for (size_t i = 0; i < joint_names.size(); i++) {
    const auto &jn = joint_names[i];
    if (!joint_states_.count(jn)) {
      ROS_ERROR("Unknown joint %s", jn.c_str());
      return false;
    }
    if (fabs(joint_states_[jn] - desired[i]) > kJointTolerance) {
      return false;
    }
  }
  return true;
}

bool QuadManipulator::cmd_gripper(const std::vector<double> &desired, const ros::Duration &timeout) {
  trajectory_msgs::JointTrajectory grasp_trajectory;
  grasp_trajectory.joint_names = kGraspJointNames;
  trajectory_msgs::JointTrajectoryPoint trajectory_point;
  trajectory_point.positions = desired;
  trajectory_point.time_from_start = ros::Duration(1.0);  // Make it by default 1.0 seconds for now
  grasp_trajectory.points.push_back(trajectory_point);

  ros::Rate rate(20.0);
  ros::Time start_time = ros::Time::now();
  ros::Time finish_time = start_time + timeout;
  while (ros::ok() && !in_pose(kGraspJointNames, trajectory_point.positions) && ros::Time::now() < finish_time) {
    grasp_trajectory.header.stamp = ros::Time::now();
    hand_control_pub_.publish(grasp_trajectory);
    ros::spinOnce();
    rate.sleep();
  }
  return in_pose(kGraspJointNames, trajectory_point.positions);
}

bool QuadManipulator::cmd_manip(const std::vector<double> &desired, const ros::Duration &timeout) {
  trajectory_msgs::JointTrajectory arm_trajectory;
  arm_trajectory.joint_names = kArmJointNames;
  trajectory_msgs::JointTrajectoryPoint trajectory_point;
  trajectory_point.positions = desired;
  trajectory_point.time_from_start = ros::Duration(2.5);  // Make it by default 2.5 seconds for now
  arm_trajectory.points.push_back(trajectory_point);

  ros::Rate rate(20.0);
  ros::Time start_time = ros::Time::now();
  ros::Time finish_time = start_time + timeout;
  while (ros::ok() && !in_pose(kArmJointNames, trajectory_point.positions) && ros::Time::now() < finish_time) {
    arm_trajectory.header.stamp = ros::Time::now();
    arm_control_pub_.publish(arm_trajectory);
    ros::spinOnce();
    rate.sleep();
  }
  return in_pose(kArmJointNames, trajectory_point.positions);
}

bool QuadManipulator::open_gripper(const ros::Duration &timeout) {
  ROS_INFO("Opening gripper..");
  const bool is_open = cmd_gripper({-0.8, 0.8}, timeout);
  ROS_INFO("Grippers %s", (is_open ? "are open" : "could not be fully opened"));
  return is_open;
}

bool QuadManipulator::close_gripper(const std::vector<double> &desired, const ros::Duration &timeout) {
  ROS_INFO("Closing gripper..");
  const bool is_closed = cmd_gripper(desired, timeout);
  ROS_INFO("Grippers %s", (is_closed ? "are closed" : "could not be closed"));
  return is_closed;
}

bool QuadManipulator::move_manip(const std::vector<double> &desired) {
  // ROS_INFO("Moving arm to desired setpoint: %s", ToString(desired).c_str());
  const bool moved = cmd_manip(desired);
  ROS_INFO("Arm %s", (moved ? "successfully moved" : "could not be moved"));
  return moved;
}

bool QuadManipulator::orbit_around(const geometry_msgs::PoseStamped &pose, double orbit_radius,
                                   double velocity_during_orbit) {
  mavros_msgs::CommandLong orbit_srv;
  orbit_srv.request.broadcast = false;
  orbit_srv.request.command = kMavOrbitCmd;
  orbit_srv.request.confirmation = 1;
  orbit_srv.request.param1 = orbit_radius;
  orbit_srv.request.param2 = velocity_during_orbit;

  std::string source_frame = pose.header.frame_id;
  std::string map_frame = nh_.resolveName(kMapFrame);
  if (map_frame.at(0) == '/') {
    map_frame.erase(map_frame.begin());
  }

  // If no source frame is specified, assume it is in map frame.
  if (source_frame == "") {
    source_frame = map_frame;
  }

  std::string err_msg;
  if (!tf_buffer_->canTransform(map_frame, source_frame, ros::Time(0), &err_msg)) {
    ROS_ERROR("Could not transform from %s to %s: %s", source_frame.c_str(), map_frame.c_str(), err_msg.c_str());
    return false;
  } else {
    geometry_msgs::TransformStamped map_T_source = tf_buffer_->lookupTransform(map_frame, source_frame, ros::Time(0));
    Eigen::Affine3d map_T_source_eigen, source_T_target_eigen;
    tf::transformMsgToEigen(map_T_source.transform, map_T_source_eigen);
    tf::poseMsgToEigen(pose.pose, source_T_target_eigen);
    const Eigen::Affine3d map_T_target_eigen = map_T_source_eigen * source_T_target_eigen;
    geometry_msgs::PoseStamped map_T_target;
    tf::poseEigenToMsg(map_T_target_eigen, map_T_target.pose);
    map_T_target.header.frame_id = map_frame;
    ROS_INFO("Center: %s for orbit", ToString(map_T_target).c_str());
    const auto map_T_target_in_ned = px4_ned_T_gazebo_convert(map_T_target);
    ROS_INFO("Center: %s for orbit", ToString(map_T_target_in_ned).c_str());
    orbit_srv.request.param5 = map_T_target_in_ned.pose.position.x;
    orbit_srv.request.param6 = map_T_target_in_ned.pose.position.y;
    orbit_srv.request.param7 = map_T_target_in_ned.pose.position.z;
  }

  // Orbit in local NED frame.
  orbit_srv.request.param4 = kLocalNedFrame;
  bool sent = command_client_.call(orbit_srv);
  if (!(sent && orbit_srv.response.success)) {
    return false;
  }
  ros::Duration(3.0).sleep(); //Sleep for 3 seconds for robot to start moving.
  ros::Time start_time = ros::Time::now();
  ros::Time finish_time = start_time + ros::Duration(100);
  ros::Rate rate(20.0);
  bool timeout = false;
  // Poll until base is stabilized or timeout
  while (ros::ok() && !base_not_moving()) {
    if (ros::Time::now() >= finish_time) {
      timeout = true;
      break;
    }
    ros::spinOnce();
    rate.sleep();
  }
  if (timeout) {
    ROS_ERROR("Finite orbit didn't complete in time. This is a failure");
    return false;
  }
  // Transition back to auto loiter mode.
  return wait_for_mode(kAutoLoiterMode);
}

bool QuadManipulator::base_not_moving() {
  return fabs(local_velocity_.twist.linear.x) <= kLinearVelTolerance &&
         fabs(local_velocity_.twist.linear.y) <= kLinearVelTolerance &&
         fabs(local_velocity_.twist.linear.z) <= kLinearVelTolerance &&
         fabs(local_velocity_.twist.angular.z) <= kAngularVelTolerance;
}