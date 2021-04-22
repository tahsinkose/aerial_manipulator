#include <quad_manipulator_control/quad_manipulator.h>
#include <quad_manipulator_control/utils.h>

#include <eigen_conversions/eigen_msg.h>
#include <mavros_msgs/CommandLong.h>
#include <tf/transform_listener.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <vector>

namespace {
constexpr const char *kOffboardMode = "OFFBOARD";
constexpr const char *kAutoLoiterMode = "AUTO.LOITER";
constexpr const char *kAutoMissionMode = "AUTO.MISSION";
constexpr const char *kStabilizedMode = "STABILIZED";
constexpr double kDefaultTakeoffAltitude = 1.5;  // in meters.
constexpr double kDefaultLandingAltitude = 1.5;  // in meters
constexpr double kDefaultRTLAltitude = 5.0; // in meters
constexpr double kJointTolerance = 1e-2;         // in rad.
constexpr double kLinearVelTolerance = 1e-2;     // in meters
constexpr double kAngularVelTolerance = 3e-2;    // in rad
const std::vector<std::string> kValidFlightModes{"MANUAL",        "ACRO",          "ALTCTL",    "POSCTL",
                                                 kOffboardMode,   kStabilizedMode, "RATTITUDE", kAutoMissionMode,
                                                 kAutoLoiterMode, "AUTO.TAKEOFF",  "AUTO.RTL",  "AUTO.LAND",
                                                 "AUTO.RTGS",     "AUTO.READY",    "GUIDED"};

const std::vector<std::string> kArmJointNames{"arm_joint_1", "arm_joint_2", "arm_joint_3"};
const std::vector<std::string> kGraspJointNames{"grasp_joint_1", "grasp_joint_2"};
constexpr const int kLocalNedFrame = 1;  // MAV_FRAME::MAV_FRAME_LOCAL_NED
constexpr const int kMavOrbitCmd = 34;   // MAV_CMD::MAV_CMD_DO_ORBIT
constexpr const char *kMapFrame = "map";
}  // namespace

QuadManipulator::QuadManipulator() {
  mav_state_sub_ = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &QuadManipulator::state_cb, this);
  extended_state_sub_ =
      nh_.subscribe<mavros_msgs::ExtendedState>("mavros/extended_state", 10, &QuadManipulator::extended_state_cb, this);
  mav_local_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 100,
                                                                  &QuadManipulator::local_pose_cb, this);
  mav_local_vel_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 100,
                                                                  &QuadManipulator::local_vel_cb, this);
  orbit_execution_state_sub_ = nh_.subscribe<mavros_msgs::OrbitStatus>("mavros/orbit_status/execution", 1,
                                                                       &QuadManipulator::orbit_state_cb, this);
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

  map_frame_ = nh_.resolveName(kMapFrame);
  if (map_frame_.at(0) == '/') {
    map_frame_.erase(map_frame_.begin());
  }
}

void QuadManipulator::state_cb(const mavros_msgs::State::ConstPtr &msg) {
  current_state_ = *msg;
  ROS_DEBUG("Received current state with mode %s", msg->mode.c_str());
}

void QuadManipulator::extended_state_cb(const mavros_msgs::ExtendedState::ConstPtr &msg) {
  extended_current_state_ = *msg;
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

void QuadManipulator::orbit_state_cb(const mavros_msgs::OrbitStatus::ConstPtr &msg) {
  orbit_state_ = msg->state;
}

bool QuadManipulator::wait_for_connection(const ros::WallDuration &timeout) {
  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);
  // wait for FCU connection
  ros::WallTime init = ros::WallTime::now();
  ros::WallTime orbit_end_timeout = init + timeout;
  while (ros::ok() && current_state_.connected != true && ros::WallTime::now() < orbit_end_timeout) {
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
  geometry_msgs::PoseStamped map_T_takeoff;
  map_T_takeoff.pose.position.x = local_pose_.pose.position.x;
  map_T_takeoff.pose.position.y = local_pose_.pose.position.y;
  map_T_takeoff.pose.position.z = kDefaultTakeoffAltitude;
  map_T_takeoff.pose.orientation = local_pose_.pose.orientation;
  return move_base(map_T_takeoff);
}

bool QuadManipulator::land() {
  mavros_msgs::CommandTOL srv_land;
  bool success = landing_client_.call(srv_land) && srv_land.response.success;
  if (success) {
    ros::Rate rate(10.0);
    while (ros::ok() && extended_current_state_.landed_state != mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) {
      rate.sleep();
      ros::spinOnce();
    }
    return true;
  } else {
    return false;
  }
}

bool QuadManipulator::move_base(const geometry_msgs::PoseStamped &target_pose, double position_tolerance) {
  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);
  double dist = DBL_MAX;
  const auto map_T_target = target_in_map(target_pose);
  if (!map_T_target) {
    return false;
  }
  ROS_INFO("Going to pose: %s", ToString(*map_T_target).c_str());
  for (int i = 0; i < 100; i++) {
    local_pos_pub_.publish(*map_T_target);
  }
  // Make sure we are in offboard mode.
  if (!wait_for_mode(kOffboardMode)) {
    return false;
  }
  do {
    local_pos_pub_.publish(*map_T_target);

    ros::spinOnce();
    rate.sleep();
    dist = sqrt(pow((local_pose_.pose.position.x - map_T_target->pose.position.x), 2) +
                pow((local_pose_.pose.position.y - map_T_target->pose.position.y), 2) +
                pow((local_pose_.pose.position.z - map_T_target->pose.position.z), 2));
  } while (ros::ok() && dist > position_tolerance);

  if (!ros::ok()) {
    return false;
  }
  return true;
}

bool QuadManipulator::wait_for_mode(const std::string &mode, uint8_t base_mode, const ros::Duration &timeout) {
  if (current_state_.mode == mode) {
    ROS_INFO("Robot is already in %s mode", mode.c_str());
    return true;
  }
  if (std::find(kValidFlightModes.begin(), kValidFlightModes.end(), mode) == kValidFlightModes.end()) {
    ROS_ERROR("Unknown flight mode %s. Failing!", mode.c_str());
    return false;
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
  ros::Time orbit_end_timeout = start_time + timeout;
  while (ros::ok() && current_state_.mode != mode && ros::Time::now() < orbit_end_timeout) {
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
  ros::Time orbit_end_timeout = start_time + timeout;
  ROS_INFO("Waiting for %s status..", (arm ? "armed" : "disarmed"));
  while (ros::ok() && current_state_.armed != true && ros::Time::now() < orbit_end_timeout) {
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
  ros::Time orbit_end_timeout = start_time + timeout;
  while (ros::ok() && !in_pose(kGraspJointNames, trajectory_point.positions) && ros::Time::now() < orbit_end_timeout) {
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
  ros::Time orbit_end_timeout = start_time + timeout;
  while (ros::ok() && !in_pose(kArmJointNames, trajectory_point.positions) && ros::Time::now() < orbit_end_timeout) {
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

  // If no source frame is specified, assume it is in map frame.
  if (source_frame == "") {
    source_frame = map_frame_;
  }
  const auto map_T_target = target_in_map(pose);
  if (!map_T_target) {
    return false;
  }
  ROS_INFO("Center: %s for orbit", ToString(*map_T_target).c_str());
  const auto map_T_target_in_ned = px4_ned_T_map(*map_T_target);
  ROS_INFO("Center: %s for orbit", ToString(map_T_target_in_ned).c_str());
  orbit_srv.request.param5 = map_T_target_in_ned.pose.position.x;
  orbit_srv.request.param6 = map_T_target_in_ned.pose.position.y;
  orbit_srv.request.param7 = map_T_target_in_ned.pose.position.z;

  // Orbit in local NED frame.
  orbit_srv.request.param4 = kLocalNedFrame;
  bool sent = command_client_.call(orbit_srv);
  if (!(sent && orbit_srv.response.success)) {
    return false;
  }

  bool timed_out = false;
  ros::Rate rate(20.0);
  ros::Time orbit_start_timeout = ros::Time::now() + ros::Duration(30);
  while (ros::ok() && orbit_state_ != mavros_msgs::OrbitStatus::ORBIT_EXECUTION_STATE_STARTED) {
    if (ros::Time::now() >= orbit_start_timeout) {
      timed_out = true;
      break;
    }
    ros::spinOnce();
    rate.sleep();
  }
  if (timed_out) {
    ROS_ERROR("Orbit didn't start in time. This is a failure");
    return false;
  } else {
    ROS_INFO("Base started to move for orbit!");
  }
  ros::Time orbit_end_timeout = ros::Time::now() + ros::Duration(100);
  // Poll until base is stabilized or timeout
  while (ros::ok() && orbit_state_ != mavros_msgs::OrbitStatus::ORBIT_EXECUTION_STATE_FINISHED) {
    if (ros::Time::now() >= orbit_end_timeout) {
      timed_out = true;
      break;
    }
    ros::spinOnce();
    rate.sleep();
  }
  if (timed_out) {
    ROS_ERROR("Orbit didn't end in time. This is a failure");
    return false;
  } else {
    ROS_INFO("Base has stabilized!");
  }
  orbit_state_ = std::nullopt;
  // Transition back to auto loiter mode.
  return wait_for_mode(kAutoLoiterMode);
}

std::optional<geometry_msgs::PoseStamped> QuadManipulator::target_in_map(
    const geometry_msgs::PoseStamped &target_pose) {
  // If no source frame is specified, assume it is in map frame.
  std::string source_frame = target_pose.header.frame_id;
  if (source_frame == "") {
    source_frame = map_frame_;
  }

  std::string err_msg;
  if (!tf_buffer_->canTransform(map_frame_, source_frame, ros::Time(0), &err_msg)) {
    ROS_ERROR("Could not transform from %s to %s: %s", source_frame.c_str(), map_frame_.c_str(), err_msg.c_str());
    return std::nullopt;
  }
  geometry_msgs::TransformStamped map_T_source = tf_buffer_->lookupTransform(map_frame_, source_frame, ros::Time(0));
  Eigen::Affine3d map_T_source_eigen, source_T_target_eigen;
  tf::transformMsgToEigen(map_T_source.transform, map_T_source_eigen);
  tf::poseMsgToEigen(target_pose.pose, source_T_target_eigen);
  const Eigen::Affine3d map_T_target_eigen = map_T_source_eigen * source_T_target_eigen;
  geometry_msgs::PoseStamped map_T_target;
  tf::poseEigenToMsg(map_T_target_eigen, map_T_target.pose);
  map_T_target.header.frame_id = map_frame_;
  return map_T_target;
}

bool QuadManipulator::go_to_home() {
  geometry_msgs::PoseStamped map_T_home;
  map_T_home.pose.position.x = local_pose_.pose.position.x;
  map_T_home.pose.position.y = local_pose_.pose.position.y;
  map_T_home.pose.position.z = kDefaultRTLAltitude;
  map_T_home.pose.orientation = local_pose_.pose.orientation;
  map_T_home.header.frame_id = map_frame_;
  bool success = move_base(map_T_home);
  map_T_home.pose.position.x = 0;
  map_T_home.pose.position.y = 0;
  success &= move_base(map_T_home);
  success &= land();
  return success;
}

bool QuadManipulator::base_not_moving() {
  return fabs(local_velocity_.twist.linear.x) <= kLinearVelTolerance &&
         fabs(local_velocity_.twist.linear.y) <= kLinearVelTolerance &&
         fabs(local_velocity_.twist.linear.z) <= kLinearVelTolerance &&
         fabs(local_velocity_.twist.angular.z) <= kAngularVelTolerance;
}