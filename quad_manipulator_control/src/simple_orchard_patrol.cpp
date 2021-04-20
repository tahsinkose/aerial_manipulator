#include <gtest/gtest.h>

#include <quad_manipulator_control/quad_manipulator.h>
#include <quad_manipulator_control/utils.h>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

TEST(QuadManipulatorTest, SimpleOrchardPatrol) {
  QuadManipulator quad_manip;
  // 1 - Connect
  ROS_INFO("Connecting...");
  ASSERT_TRUE(quad_manip.wait_for_connection()) << "Robot could connect to PX4 board";
  ASSERT_TRUE(quad_manip.motor_arm(true)) << "Robot could not be armed";
  // 2 - Takeoff
  ROS_INFO("Takeoff...");
  ASSERT_TRUE(quad_manip.takeoff()) << "Robot could not takeoff";

  // 3 - Orbit around trees
  geometry_msgs::PoseStamped earth_T_tree1;
  earth_T_tree1.header.frame_id = "earth";
  earth_T_tree1.pose.position.x = 2;
  earth_T_tree1.pose.position.y = -2;
  earth_T_tree1.pose.position.z = 1.5;
  ROS_INFO("Orbitting around %s...", ToString(earth_T_tree1).c_str());
  ASSERT_TRUE(quad_manip.orbit_around(earth_T_tree1, 1.0, 0.4))
      << "Robot could not successfully orbit around " << ToString(earth_T_tree1);

  geometry_msgs::PoseStamped earth_T_tree2;
  earth_T_tree2.header.frame_id = "earth";
  earth_T_tree2.pose.position.x = -2.0;
  earth_T_tree2.pose.position.y = -2.0;
  earth_T_tree2.pose.position.z = 1.5;
  ROS_INFO("Orbitting around %s...", ToString(earth_T_tree2).c_str());
  ASSERT_TRUE(quad_manip.orbit_around(earth_T_tree2, 1.0, 0.4))
      << "Robot could not successfully orbit around " << ToString(earth_T_tree2);

  geometry_msgs::PoseStamped earth_T_tree3;
  earth_T_tree3.header.frame_id = "earth";
  earth_T_tree3.pose.position.x = -2.0;
  earth_T_tree3.pose.position.y = 2.0;
  earth_T_tree3.pose.position.z = 1.5;
  ROS_INFO("Orbitting around %s...", ToString(earth_T_tree3).c_str());
  ASSERT_TRUE(quad_manip.orbit_around(earth_T_tree3, 1.0, 0.4))
      << "Robot could not successfully orbit around " << ToString(earth_T_tree3);

  geometry_msgs::PoseStamped earth_T_tree4;
  earth_T_tree4.header.frame_id = "earth";
  earth_T_tree4.pose.position.x = 2.0;
  earth_T_tree4.pose.position.y = 2.0;
  earth_T_tree4.pose.position.z = 1.5;
  ROS_INFO("Orbitting around %s...", ToString(earth_T_tree4).c_str());
  ASSERT_TRUE(quad_manip.orbit_around(earth_T_tree4, 1.0, 0.4)) <<
    "Robot could not successfully orbit around " << ToString(earth_T_tree3);
  geometry_msgs::PoseStamped home_pose;
  home_pose.pose.position.x = 0;
  home_pose.pose.position.y = 0;
  home_pose.pose.position.z = 2.0;
  ROS_INFO("Returning home...");
  constexpr double distance_tolerance = 0.05;  // 5cm tolerance for this test.
  ASSERT_TRUE(quad_manip.move_base(home_pose, distance_tolerance))
      << "Could not move to home pose: " << ToString(home_pose).c_str();

  ROS_INFO("Landing...");
  ASSERT_TRUE(quad_manip.land()) << "Could not landed";
  ROS_INFO("Returned home!");
  ASSERT_TRUE(quad_manip.motor_arm(false)) << "Robot could not be armed.";
  ROS_INFO("Robot disarmed!");
  ROS_INFO("Finished simple orchard test successfully!");
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "simple_orchard_patrol");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}