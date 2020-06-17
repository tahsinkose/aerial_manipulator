#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>

class QuadManipulator {
public:
  QuadManipulator() {
    state_sub_ = nh_.subscribe<mavros_msgs::State>(
        "mavros/state", 10, &QuadManipulator::state_cb, this);
    pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
        "/mavros/local_position/pose", 100, &QuadManipulator::pose_cb, this);
    local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "mavros/setpoint_position/local", 10);
    arming_client_ =
        nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    landing_client_ =
        nh_.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    set_mode_client_ =
        nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  }
  void state_cb(const mavros_msgs::State::ConstPtr &msg) {
    current_state_ = *msg;
    ROS_INFO_ONCE("Received current state with mode %s", msg->mode.c_str());
  }
  void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    current_pose_ = *msg;
  }
  void arm() {
    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    // wait for FCU connection
    while (ros::ok() && !current_state_.connected) {
      ros::spinOnce();
      rate.sleep();
    }
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    // send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i) {
      local_pos_pub_.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    while (ros::ok()) {
      if (current_state_.mode != "OFFBOARD" &&
          (ros::Time::now() - last_request > ros::Duration(5.0))) {
        if (set_mode_client_.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent) {
          ROS_INFO("Offboard enabled");
        }
        last_request = ros::Time::now();
      } else {
        if (!current_state_.armed &&
            (ros::Time::now() - last_request > ros::Duration(5.0))) {
          if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
            ROS_INFO("Vehicle armed");
          }
          last_request = ros::Time::now();
        }
      }
      local_pos_pub_.publish(pose);
      if (current_state_.mode == "OFFBOARD" && current_state_.armed) {
        break;
      }
      ros::spinOnce();
      rate.sleep();
    }
  }

  void test() {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    double dist = DBL_MAX;

    do {
      local_pos_pub_.publish(pose);

      ros::spinOnce();
      rate.sleep();
      dist =
          sqrt(pow((current_pose_.pose.position.x - pose.pose.position.x), 2) +
               pow((current_pose_.pose.position.y - pose.pose.position.y), 2) +
               pow((current_pose_.pose.position.z - pose.pose.position.z), 2));
    } while (dist > 0.1);
    mavros_msgs::CommandTOL land_cmd;
    if(landing_client_.call(land_cmd)){
      ROS_INFO("Landing request was %s",
               (land_cmd.response.success ? "successful" : "unsuccesful"));
    }

  }

private:
  mavros_msgs::State current_state_;
  geometry_msgs::PoseStamped current_pose_;
  ros::NodeHandle nh_;
  ros::Subscriber state_sub_;
  ros::Subscriber pose_sub_;
  ros::Publisher local_pos_pub_;
  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;
  ros::ServiceClient landing_client_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "verify_quad_manip");
  QuadManipulator quad_manip;
  quad_manip.arm();
  quad_manip.test();
  return 0;
}