#include <geometry_msgs/PoseStamped.h>
#include <quad_manipulator_control/quad_manipulator.h>
#include <quad_manipulator_control/utils.h>

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

  geometry_msgs::PoseStamped target_pose;
  target_pose.pose.position.x = -7.9;
  target_pose.pose.position.y = 7.9;
  target_pose.pose.position.z = 0.97;
  const bool moved = quad_manip.move_base(target_pose, distance_tolerance);
  if (!moved) {
    ROS_ERROR("Could not move to target pose: %s", ToString(target_pose).c_str());
  }

  return 0;
}