#include <geometry_msgs/PoseStamped.h>
#include <quad_manipulator_control/quad_manipulator.h>
#include <quad_manipulator_control/utils.h>

namespace {
constexpr double distance_tolerance = 0.05; // 5cm tolerance for this test.
constexpr const char *kAppleName = "target_apple";
const std::vector<double> kArmGrab = {0.0, 0.0, -0.1};
const std::vector<double> kArmDispose = {0.0, -0.37, -0.99};
const std::vector<double> kArmZero = {0.0, 0.0, 0.0};
const std::vector<double> kGripperFullClose = {0.0, 0.0};
} // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "simple_pick_fly_dispose");
  QuadManipulator quad_manip;
  // 1 - Connect
  const bool connected = quad_manip.wait_for_connection();
  if (!connected) {
    ROS_ERROR("Robot could connect to PX4 board. Failing..");
    return -1;
  }
  const bool armed = quad_manip.motor_arm();
  if (!armed) {
    ROS_ERROR("Robot could not be armed. Failing..");
    return -1;
  }

  // 2 - Pick
  if (!quad_manip.open_gripper()) {
    ROS_ERROR("Robot could not open grippers. Failing..");
    return -1;
  }

  if (!quad_manip.move_manip(kArmGrab)) {
    ROS_ERROR(
        "Robot could not move the arm to desired configuration. Failing..");
    return -1;
  }

  geometry_msgs::Pose correct_pose;
  correct_pose.position.x = 0.003;
  correct_pose.position.y = 0.0;
  correct_pose.position.z = 0.008;
  if (!set_gazebo_model_pose(kAppleName, correct_pose)) {
    ROS_ERROR("Could not set the apple's pose properly.");
    return -1;
  }

  if (!quad_manip.close_gripper()) {
    ROS_ERROR("Robot could not open grippers. Failing..");
    return -1;
  }
  // 3 - Fly
  const bool took_off = quad_manip.takeoff();
  if (!took_off) {
    ROS_ERROR("Robot could not takeoff. Failing..");
    return -1;
  }

  geometry_msgs::PoseStamped target_pose;
  target_pose.pose.position.x = -7.9;
  target_pose.pose.position.y = 7.9;
  target_pose.pose.position.z = 1.02;
  const bool moved = quad_manip.move_base(target_pose, distance_tolerance);
  if (!moved) {
    ROS_ERROR("Could not move to target pose: %s",
              ToString(target_pose).c_str());
    return -1;
  }

  // 4 - Dispose.
  ROS_INFO("Disposal started!");
  if (!quad_manip.move_manip(kArmDispose)) {
    ROS_ERROR(
        "Robot could not move the arm to desired configuration. Failing..");
    return -1;
  }
  if (!quad_manip.open_gripper()) {
    ROS_ERROR("Robot could not open grippers. Failing..");
    return -1;
  }

  if (!quad_manip.close_gripper(kGripperFullClose)) {
    ROS_ERROR("Robot could not open grippers. Failing..");
    return -1;
  }
  if (!quad_manip.move_manip(kArmZero)) {
    ROS_ERROR(
        "Robot could not move the arm to desired configuration. Failing..");
    return -1;
  }
  ROS_INFO("Disposal finished!");

  return 0;
}