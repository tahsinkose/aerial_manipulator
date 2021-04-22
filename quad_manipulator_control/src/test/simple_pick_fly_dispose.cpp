#include <gtest/gtest.h>

#include <geometry_msgs/PoseStamped.h>
#include <quad_manipulator_control/quad_manipulator.h>
#include <quad_manipulator_control/utils.h>

namespace {
constexpr double distance_tolerance = 0.05;  // 5cm tolerance for this test.
constexpr const char *kAppleName = "target_apple";
const std::vector<double> kArmGrab = {0.0, 0.0, -0.1};
const std::vector<double> kArmDispose = {0.0, -0.37, -0.99};
const std::vector<double> kArmZero = {0.0, 0.0, 0.0};
const std::vector<double> kGripperFullClose = {0.0, 0.0};
}  // namespace

TEST(QuadManipulatorTest, SimplePickFlyDispose) {
  QuadManipulator quad_manip;
  // 1 - Connect
  ROS_INFO("Connecting...");
  ASSERT_TRUE(quad_manip.wait_for_connection()) << "Robot could connect to PX4 board";
  // 2 - Pick
  ROS_INFO("Picking apple...");
  ASSERT_TRUE(quad_manip.open_gripper()) << "Robot could not open grippers";
  ASSERT_TRUE(quad_manip.move_manip(kArmGrab)) << "Robot could not move the arm to desired configuration";

  geometry_msgs::Pose correct_pose;
  correct_pose.position.x = 0.0025;
  correct_pose.position.y = 0.0;
  correct_pose.position.z = 0.008;

  ASSERT_TRUE(set_gazebo_model_pose(kAppleName, correct_pose)) << "Could not set the apple's pose properly.";

  ASSERT_TRUE(quad_manip.close_gripper()) << "Robot could not open grippers";
  // 3 - Fly
  ASSERT_TRUE(quad_manip.motor_arm(true)) << "Robot could not be armed";
  ROS_INFO("Takeoff...");
  ASSERT_TRUE(quad_manip.takeoff()) << "Robot could not takeoff";

  geometry_msgs::PoseStamped target_pose;
  target_pose.pose.position.x = -8.0;
  target_pose.pose.position.y = 8.0;
  target_pose.pose.position.z = 1.02;
  target_pose.pose.orientation.w = 0.707;
  target_pose.pose.orientation.x = 0.0;
  target_pose.pose.orientation.y = 0.0;
  target_pose.pose.orientation.z = 0.707;
  ASSERT_TRUE(quad_manip.move_base(target_pose, distance_tolerance))
      << "Could not move to target pose: " << ToString(target_pose);
  // 4 - Dispose.
  ROS_INFO("Disposal started!");
  ASSERT_TRUE(quad_manip.move_manip(kArmDispose)) << "Robot could not move the arm to desired configuration";
  ASSERT_TRUE(quad_manip.open_gripper()) << "Robot could not open grippers.";
  ASSERT_TRUE(quad_manip.close_gripper(kGripperFullClose)) << "Robot could not open grippers";

  ASSERT_TRUE(quad_manip.move_manip(kArmZero)) << "Robot could not move the arm to desired configuration.";
  ROS_INFO("Disposal finished!");

  ROS_INFO("Returning home...");
  ASSERT_TRUE(quad_manip.go_to_home()) << "Could not go to home";
  ROS_INFO("Returned home!");
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "simple_pick_fly_dispose");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}