#pragma once

#include <algorithm>
#include <assert.h>
#include <mutex>
#include <string>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <quad_manipulator_msgs/RobotCollisions.h>
#include <quad_manipulator_gazebo/fake_grasping_gripper.h>
#include <gazebo_msgs/ContactState.h>
#include <ros/advertise_options.h>
#include <ros/ros.h>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <sdf/Param.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <gazebo_plugins/PubQueue.h>

namespace gazebo {
class RobotCollisions : public gazebo::ModelPlugin {
public:
  /// \brief Constructor
  RobotCollisions();

  /// \brief Destructor
  ~RobotCollisions();

  /// \brief Load function called when the plugin is loaded
  void Load(physics::ModelPtr parent, sdf::ElementPtr _sdf);

  /// \brief OnUpdate is called on each simulation iteration
  void OnUpdate(const common::UpdateInfo &);

  /// \brief OnContactsUpdate is called when the physics collisions
  /// messages are published
  void virtual OnContactsUpdate(const ConstContactsPtr &msg);

  /// \brief Reset function called when the world is reseted
  void Reset();

private:
  /// \brief Pointer to the world
  physics::WorldPtr world_;

  /// \brief Pointer to the model
  physics::ModelPtr model_;

  /// \brief Updated signal and the OnUpdate callback.
  event::ConnectionPtr updateConnection_;

  /// \brief Last time collisions were updated
  common::Time last_time_collisions_update_;

  /// \brief Last time contacts were updated
  common::Time last_time_on_contact_update_;

  /// \brief pointer to ros node
  std::unique_ptr<ros::NodeHandle> rosnode_;

  /// \brief ROS publisher
  ros::Publisher collision_pub_;

  /// \brief Queue publisher
  PubQueue<quad_manipulator_msgs::RobotCollisions>::Ptr collision_pub_queue_;

  /// \brief ROS message for collision event
  quad_manipulator_msgs::RobotCollisions collision_event_;

  /// \brief Stored contacts
  std::vector<quad_manipulator_msgs::RobotCollisions> stored_collision_events_;

  /// \brief Gazebo topic subscriber
  gazebo::transport::SubscriberPtr gazebo_sub_contacts_;

  /// \brief OnContact collision callback update rate
  double collisions_min_update_dt_;

  /// \brief OnUpdate update rate
  double min_update_dt_;

  /// \brief Previous OnUpdate call
  common::Time previous_update_time;

  /// \brief Message publishing rate
  double pub_rate_;

  /// \brief topic name
  std::string topic_name_;

  /// \brief links list
  physics::Link_V links_vector_;

  /// \brief vector of excluded elements
  std::vector<std::string> excluded_objects_;

  /// \brief prevents blocking
  PubMultiQueue pmq_;

  // List of grippers by name
  std::map<std::string, FakeGraspingGripper> grippers;

  /// \brief Mutex to protect reads and writes.
private:
  mutable std::mutex mutex_;
};
}
