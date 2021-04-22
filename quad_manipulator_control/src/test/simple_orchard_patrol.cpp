#include <gtest/gtest.h>

#include <quad_manipulator_control/quad_manipulator.h>
#include <quad_manipulator_control/utils.h>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

namespace{
constexpr const double kTreeOrbitRadius = 1.0; // m
constexpr const double kTreeOrbitVelocity = 0.6; // m/s
}

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
  ASSERT_TRUE(quad_manip.orbit_around(earth_T_tree1, kTreeOrbitRadius, kTreeOrbitVelocity))
      << "Robot could not successfully orbit around " << ToString(earth_T_tree1);

  geometry_msgs::PoseStamped earth_T_tree2;
  earth_T_tree2.header.frame_id = "earth";
  earth_T_tree2.pose.position.x = -2.0;
  earth_T_tree2.pose.position.y = -2.0;
  earth_T_tree2.pose.position.z = 1.5;
  ROS_INFO("Orbitting around %s...", ToString(earth_T_tree2).c_str());
  ASSERT_TRUE(quad_manip.orbit_around(earth_T_tree2, kTreeOrbitRadius, kTreeOrbitVelocity))
      << "Robot could not successfully orbit around " << ToString(earth_T_tree2);

  geometry_msgs::PoseStamped earth_T_tree3;
  earth_T_tree3.header.frame_id = "earth";
  earth_T_tree3.pose.position.x = -2.0;
  earth_T_tree3.pose.position.y = 2.0;
  earth_T_tree3.pose.position.z = 1.5;
  ROS_INFO("Orbitting around %s...", ToString(earth_T_tree3).c_str());
  ASSERT_TRUE(quad_manip.orbit_around(earth_T_tree3, kTreeOrbitRadius, kTreeOrbitVelocity))
      << "Robot could not successfully orbit around " << ToString(earth_T_tree3);

  geometry_msgs::PoseStamped earth_T_tree4;
  earth_T_tree4.header.frame_id = "earth";
  earth_T_tree4.pose.position.x = 2.0;
  earth_T_tree4.pose.position.y = 2.0;
  earth_T_tree4.pose.position.z = 1.5;
  ROS_INFO("Orbitting around %s...", ToString(earth_T_tree4).c_str());
  ASSERT_TRUE(quad_manip.orbit_around(earth_T_tree4, kTreeOrbitRadius, kTreeOrbitVelocity))
      << "Robot could not successfully orbit around " << ToString(earth_T_tree3);

  ROS_INFO("Returning home...");
  ASSERT_TRUE(quad_manip.go_to_home()) << "Could not go to home: ";
  ROS_INFO("Returned home!");
  ROS_INFO("Finished simple orchard patrol test successfully!");
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "simple_orchard_patrol");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}