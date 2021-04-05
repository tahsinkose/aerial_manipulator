#include <quad_manipulator_control/quad_manipulator.h>

QuadManipulator::QuadManipulator() {
  state_sub_ = nh_.subscribe<mavros_msgs::State>(
      "mavros/state", 10, &QuadManipulator::state_cb, this);
  pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
      "/mavros/local_position/pose", 100, &QuadManipulator::local_pose_cb, this);
  local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "mavros/setpoint_position/local", 10);
  arming_client_ =
      nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  takeoff_client_ =
      nh_.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
  landing_client_ =
      nh_.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
}

void QuadManipulator::state_cb(const mavros_msgs::State::ConstPtr &msg) {
  current_state_ = *msg;
  ROS_INFO_ONCE("Received current state with mode %s", msg->mode.c_str());
}

void QuadManipulator::local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  local_pose_ = *msg;
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
    ROS_ERROR_NAMED("verify_quad_manip", "Couldn't connect to the board!");
    return false;
  }

  geometry_msgs::PoseStamped dummy;

  // send a few setpoints before starting
  for (int i = 100; ros::ok() && i > 0; --i) {
    local_pos_pub_.publish(dummy);
    ros::spinOnce();
    rate.sleep();
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arming_cmd;
  arming_cmd.request.value = true;

  ros::Time last_request = ros::Time::now();
  int arming_timeout = 10;
  int current_arm_cnt = 0;
  while (ros::ok()) {
    if (current_state_.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(5.0))) {
      if (set_mode_client_.call(offb_set_mode) &&
          offb_set_mode.response.mode_sent) {
        ROS_INFO("Offboard mode request sent to PX4");
      }
      last_request = ros::Time::now();
    } else if (current_state_.mode != "OFFBOARD") {
      ROS_INFO_ONCE("Current mode: %s. Sent offboard mode request within last 5 "
               "seconds. Pending..",
               current_state_.mode.c_str());
    } else {
      ROS_INFO_ONCE("Offboard enabled");
      if (!current_state_.armed &&
          (ros::Time::now() - last_request > ros::Duration(5.0))) {
        if (arming_client_.call(arming_cmd) && arming_cmd.response.success) {
          ROS_INFO("Vehicle armed");
        } else {
          current_arm_cnt++;
          if (current_arm_cnt == arming_timeout) {
            return false;
          }
        }
        last_request = ros::Time::now();
      }
    }
    local_pos_pub_.publish(dummy);
    if (current_state_.mode == "OFFBOARD" && current_state_.armed) {
      break;
    }
    ros::spinOnce();
    rate.sleep();
  }
  return true;
}

bool QuadManipulator::takeoff() {
  mavros_msgs::CommandTOL srv_takeoff;
  srv_takeoff.request.altitude = 1.5;

  if (takeoff_client_.call(srv_takeoff)) {
    ROS_INFO("Takeoff sent %d", srv_takeoff.response.success);
    return true;
  } else {
    ROS_ERROR("Failed Takeoff");
    return false;
  }
}

bool QuadManipulator::move_base(const geometry_msgs::PoseStamped &target_pose,
                                 double position_tolerance) {
  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);
  double dist = DBL_MAX;

  do {
    local_pos_pub_.publish(target_pose);

    ros::spinOnce();
    rate.sleep();
    dist = sqrt(pow((local_pose_.pose.position.x - target_pose.pose.position.x), 2) +
                pow((local_pose_.pose.position.y - target_pose.pose.position.y), 2) +
                pow((local_pose_.pose.position.z - target_pose.pose.position.z), 2));
  } while (dist > position_tolerance);

  // Transition back to auto loiter mode.
  mavros_msgs::SetMode hover_set_mode;
  hover_set_mode.request.custom_mode = "AUTO.LOITER";
  if (set_mode_client_.call(hover_set_mode) &&
      hover_set_mode.response.mode_sent) {
    ROS_INFO("Finishing with hover mode.");
    return true;
  } else {
    ROS_ERROR("Could not finish at hover mode!");
    return false;
  }
}