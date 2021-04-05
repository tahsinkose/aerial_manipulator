#include <geometry_msgs/PoseStamped.h>
#include <quad_manipulator_control/quad_manipulator.h>

namespace {
constexpr double distance_tolerance = 0.02; // 2cm tolerance for this test.
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "simple_pick_fly_dispose");
  QuadManipulator quad_manip;
  const bool armed = quad_manip.arm();
  if (!armed) {
    ROS_ERROR("Robot could not be armed. Failing..");
    return -1;
  }

  const bool took_off = quad_manip.takeoff();
  if (!took_off) {
    ROS_ERROR("Robot could not takeoff. Failing..");
    return -1;
  }
  // Sleep for 3 seconds
  ros::Duration(3.0).sleep();

  geometry_msgs::PoseStamped target_pose;
  target_pose.pose.position.x = -7.9;
  target_pose.pose.position.y = 7.9;
  target_pose.pose.position.z = 0.97;
  quad_manip.move_base(target_pose);

  return 0;
}