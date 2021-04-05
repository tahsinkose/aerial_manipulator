#include <quad_manipulator_control/quad_manipulator.h>
#include <quad_manipulator_control/utils.h>

#include <vector>

namespace {
constexpr const char *kOffboardMode = "OFFBOARD";
constexpr const char *kAutoLoiterMode = "AUTO.LOITER";
constexpr double kDefaultTakeoffAltitude = 2.5; // in meters.

const std::vector<std::string> kValidFlightModes{
    "MANUAL",     "ACRO",      "ALTCTL",       "POSCTL",      "OFFBOARD",
    "STABILIZED", "RATTITUDE", "AUTO.MISSION", "AUTO.LOITER", "AUTO.TAKEOFF",
    "AUTO.RTL",   "AUTO.LAND", "AUTO.RTGS",    "AUTO.READY"};
} // namespace

QuadManipulator::QuadManipulator() {
  state_sub_ = nh_.subscribe<mavros_msgs::State>(
      "mavros/state", 10, &QuadManipulator::state_cb, this);
  pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
      "/mavros/local_position/pose", 100, &QuadManipulator::local_pose_cb,
      this);
  local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "mavros/setpoint_position/local", 10);
  arming_client_ =
      nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  landing_client_ =
      nh_.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
}

void QuadManipulator::state_cb(const mavros_msgs::State::ConstPtr &msg) {
  current_state_ = *msg;
  ROS_DEBUG("Received current state with mode %s", msg->mode.c_str());
}

void QuadManipulator::local_pose_cb(
    const geometry_msgs::PoseStamped::ConstPtr &msg) {
  local_pose_ = *msg;
  ROS_DEBUG("Received current pose: %s", ToString(local_pose_).c_str());
}

bool QuadManipulator::arm() {
  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);
  // wait for FCU connection
  ros::WallTime init = ros::WallTime::now();
  double elapsed = ros::WallTime::now().toSec() - init.toSec();
  while (elapsed < 10.0 && !current_state_.connected) {
    ros::spinOnce();
    rate.sleep();
    elapsed = ros::WallTime::now().toSec() - init.toSec();
  }

  if (!current_state_.connected) {
    ROS_ERROR("Couldn't connect to the board!");
    return false;
  }

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
  if (!wait_for_armed()) {
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

bool QuadManipulator::move_base(const geometry_msgs::PoseStamped &target_pose,
                                double position_tolerance) {

  // Make sure we are in offboard mode.
  if (!wait_for_mode(kOffboardMode)) {
    return false;
  }

  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);
  double dist = DBL_MAX;
  ROS_INFO("Going to pose: %lf, %lf, %lf in frame '%s'",
           target_pose.pose.position.x, target_pose.pose.position.y,
           target_pose.pose.position.z, target_pose.header.frame_id.c_str());
  do {
    local_pos_pub_.publish(target_pose);

    ros::spinOnce();
    rate.sleep();
    dist = sqrt(
        pow((local_pose_.pose.position.x - target_pose.pose.position.x), 2) +
        pow((local_pose_.pose.position.y - target_pose.pose.position.y), 2) +
        pow((local_pose_.pose.position.z - target_pose.pose.position.z), 2));
  } while (ros::ok() && dist > position_tolerance);

  if (!ros::ok()) {
    return false;
  }

  // Transition back to auto loiter mode.
  return wait_for_mode(kAutoLoiterMode);
}

bool QuadManipulator::wait_for_mode(const std::string &mode,
                                    const ros::Duration &timeout) {

  if (std::find(kValidFlightModes.begin(), kValidFlightModes.end(), mode) ==
      kValidFlightModes.end()) {
    ROS_ERROR("Unknown flight mode %s. Failing!", mode.c_str());
    return false;
  }

  if (current_state_.mode == mode) {
    ROS_INFO("Robot is already in %s mode", mode.c_str());
    return true;
  }

  mavros_msgs::SetMode set_mode;
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
  while (ros::ok() && current_state_.mode != mode &&
         ros::Time::now() < finish_time) {
    ROS_DEBUG("Current mode: %s. Pending until %s mode..",
                  current_state_.mode.c_str(), mode.c_str());
    ros::spinOnce();
    rate.sleep();
    geometry_msgs::PoseStamped dummy;
    local_pos_pub_.publish(dummy);
  }
  const bool result = current_state_.mode == mode;
  ROS_INFO("Finishing %s in %s mode ",
           (result ? "successfully" : "unsuccessfully"), current_state_.mode.c_str());
  return result;
}

bool QuadManipulator::wait_for_armed(const ros::Duration &timeout) {
  mavros_msgs::CommandBool arming_cmd;
  arming_cmd.request.value = true;
  if (current_state_.armed) {
    ROS_INFO("Robot is already armed!");
    return true;
  }

  if (arming_client_.call(arming_cmd) && arming_cmd.response.success) {
    ROS_INFO("Vehicle arm request is sent!");
  } else {
    ROS_ERROR("Vehicle arm request could not be sent. Failing..");
    return false;
  }
  ros::Rate rate(20.0);
  ros::Time start_time = ros::Time::now();
  ros::Time finish_time = start_time + timeout;
  ROS_INFO("Waiting for armed status..");
  while (ros::ok() && current_state_.armed != true &&
         ros::Time::now() < finish_time) {
    ros::spinOnce();
    rate.sleep();
  }
  const bool result = current_state_.armed;
  ROS_INFO("Armed %s", (result ? "successfully" : "unsuccessfully"));
  return result;
}