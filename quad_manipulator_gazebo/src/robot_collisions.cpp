
#include <quad_manipulator_gazebo/robot_collisions.h>
#include <quad_manipulator_msgs/FakeGraspingEvent.h>
namespace {
constexpr const char *kLoggerName = "Fake Grasping";
}
namespace gazebo {

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(RobotCollisions)

////////////////////////////////////////////////////////////////////////////////
// Constructor
RobotCollisions::RobotCollisions() : ModelPlugin() {}

////////////////////////////////////////////////////////////////////////////////
// Destructor
RobotCollisions::~RobotCollisions() {}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void RobotCollisions::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  model_ = _parent;
  world_ = _parent->GetWorld();

  ROS_INFO("Starting plugin Robot Collisions!");

  transport::NodePtr gazebo_node(new transport::Node());
  gazebo_node->Init();

  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&RobotCollisions::OnUpdate, this, _1));

  rosnode_.reset(new ros::NodeHandle());
  pmq_.startServiceThread();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM("Ros not initialized!");
    return;
  }

  // Load SDF parameters
  if (!_sdf->HasElement("topicName")) {
    topic_name_ = "/robot_collisions";
    ROS_WARN_STREAM("Missing <topicName> default to /robot_collisions");
  } else
    topic_name_ = _sdf->Get<std::string>("topicName");

  if (!_sdf->HasElement("minUpdateDT")) {
    min_update_dt_ = 1.0 / 100.0;
    ROS_WARN_STREAM(
        "Missing <minUpdateDT>, using default value: " << min_update_dt_);
  } else
    min_update_dt_ = _sdf->Get<double>("minUpdateDT");

  if (!_sdf->HasElement("collisionsMinUpdateDT")) {
    collisions_min_update_dt_ = 1.0 / 20000;
    ROS_WARN_STREAM("Missing <collisionsMinUpdateDT>, using default value: "
                    << collisions_min_update_dt_);
  } else
    collisions_min_update_dt_ = _sdf->Get<double>("collisionsMinUpdateDT");

  if (!_sdf->HasElement("messagePubRate")) {
    pub_rate_ = 1.0 / 10.0;
    ROS_WARN_STREAM(
        "Missing <messagePubRate>, using default value: " << pub_rate_);
  } else
    pub_rate_ = _sdf->Get<double>("messagePubRate");

  // Initialize ROS publisher
  if (topic_name_ != "") {
    collision_pub_ =
        rosnode_->advertise<quad_manipulator_msgs::RobotCollisions>(topic_name_,
                                                                    1);
    collision_pub_queue_ =
        pmq_.addPub<quad_manipulator_msgs::RobotCollisions>();
  }

  // Subscribe to the physics contacts topic
  gazebo_sub_contacts_ = gazebo_node->Subscribe(
      "~/physics/contacts", &RobotCollisions::OnContactsUpdate, this);

  // Get robot link vector
  links_vector_ = _parent->GetLinks();

  // Start tracking simulation time
  last_time_on_contact_update_ = world_->SimTime();
  last_time_collisions_update_ = world_->SimTime();

  // Get excluded objects from the rosparams
  XmlRpc::XmlRpcValue v;
  if (!rosnode_->getParam("excluded_objects", v)) {
    ROS_INFO("No excluded object listed for RobotCollisions plugin!");
  } else {
    std::string output_rosparam =
        "Excluded object list found for RobotCollisions, excluding: ";
    for (int i = 0; i < v.size(); i++) {
      excluded_objects_.push_back(v[i]);
      output_rosparam.append(" " + excluded_objects_[i]);
    }
    ROS_INFO_STREAM(output_rosparam);
  }

  if (!_sdf->HasElement("gripper")) {
    ROS_WARN_NAMED(
        kLoggerName,
        "Missing gripper in sdf. Fake grasping won't be configured.");
  } else {
    ROS_INFO_NAMED(kLoggerName, "Gripper found, configuring grippers.");

    bool disableCollisionsOnAttach;
    double forcesAngleTolerance;
    double minForceRatio;
    double maxGripCount;
    double gripCountThreshold;
    double releaseTolerance;
    std::string gripperTopic;

    if (!_sdf->HasElement("disable_collisions_on_attach")) {
      disableCollisionsOnAttach = true;
      ROS_WARN_NAMED(
          kLoggerName,
          "Missing <disable_collisions_on_attach>, using default value: %s",
          disableCollisionsOnAttach ? "true" : "false");
    } else
      disableCollisionsOnAttach =
          _sdf->Get<bool>("disable_collisions_on_attach");

    if (!_sdf->HasElement("forces_angle_tolerance_deg")) {
      forcesAngleTolerance = 120 * M_PI / 180;
      ROS_WARN_NAMED(
          kLoggerName,
          "Missing <forces_angle_tolerance_deg>, using default value [rad]: %f",
          forcesAngleTolerance);
    } else
      forcesAngleTolerance =
          _sdf->Get<double>("forces_angle_tolerance_deg") * M_PI / 180;

    if (!_sdf->HasElement("min_force_ratio")) {
      minForceRatio = 0.3;
      ROS_WARN_NAMED(kLoggerName,
                     "Missing <min_force_ratio>, using default value [rad]: %f",
                     minForceRatio);
    } else
      minForceRatio = _sdf->Get<double>("min_force_ratio");

    if (!_sdf->HasElement("max_grip_count")) {
      maxGripCount = 10;
      ROS_WARN_NAMED(kLoggerName,
                     "Missing <max_grip_count>, using default value: %f",
                     maxGripCount);
    } else
      maxGripCount = _sdf->Get<double>("max_grip_count");

    if (!_sdf->HasElement("grip_count_threshold")) {
      gripCountThreshold = floor(maxGripCount / 2.0);
      ROS_WARN_NAMED(kLoggerName,
                     "Missing <grip_count_threshold>, using default value: %f",
                     gripCountThreshold);
    } else
      gripCountThreshold = _sdf->Get<double>("grip_count_threshold");

    if (!_sdf->HasElement("release_tolerance")) {
      releaseTolerance = 0.005;
      ROS_WARN_NAMED(kLoggerName,
                     "Missing <release_tolerance>, using default value: %f",
                     releaseTolerance);
    } else
      releaseTolerance = _sdf->Get<double>("release_tolerance");

    if (!_sdf->HasElement("gripper_topic")) {
      gripperTopic = "gripper_event";
      ROS_WARN_NAMED(kLoggerName,
                     "Missing <gripper_topic>, using default value: %s",
                     gripperTopic.c_str());
    } else
      gripperTopic = _sdf->Get<std::string>("gripper_topic");

    ros::Publisher grasping_pub =
        rosnode_->advertise<quad_manipulator_msgs::FakeGraspingEvent>(
            gripperTopic, 5);

    // Support multiple grippers:
    for (sdf::ElementPtr gripperElement = _sdf->GetElement("gripper");
         gripperElement != NULL;
         gripperElement = gripperElement->GetNextElement("gripper")) {
      std::string gripperName;
      if (!gripperElement->HasElement("gripper_name")) {
        gripperName = "fake_gripper";
        ROS_WARN_NAMED(kLoggerName,
                       "Missing <gripper_name>, using default value: %s",
                       gripperName.c_str());
      } else
        gripperName = gripperElement->Get<std::string>("gripper_name");

      std::string palmLinkName;
      if (!gripperElement->HasElement("palm_name")) {
        palmLinkName = "fake_gripper";
        ROS_WARN_NAMED(kLoggerName,
                       "Missing <palm_name>, using default value: %s",
                       palmLinkName.c_str());
      } else
        palmLinkName = gripperElement->Get<std::string>("palm_name");

      // collect all finger names:
      std::vector<std::string> fingerLinkNames;
      for (sdf::ElementPtr fingerElement =
               gripperElement->GetElement("finger_link_name");
           fingerElement != NULL;
           fingerElement = fingerElement->GetNextElement("finger_link_name")) {
        std::string linkName = fingerElement->Get<std::string>();
        fingerLinkNames.push_back(linkName);
      }

      // add new gripper
      if (grippers.find(gripperName) != grippers.end()) {
        ROS_WARN_NAMED(kLoggerName,
                       "Gripper <%s> Is already in the map, skipping.",
                       gripperName.c_str());
        continue;
      }
      std::vector<std::string> fingerCollisionNames;
      for (std::string finger : fingerLinkNames) {
        try {
          int index = 0;
          std::string collision =
              _parent->GetLink(finger)->GetCollision(index)->GetScopedName();
          fingerCollisionNames.push_back(collision);
        } catch (const std::exception &e) {
          ROS_WARN_NAMED(kLoggerName,
                         "Couldn't find collision for %s, plugin won't work.",
                         finger.c_str());
        }
      }
      physics::LinkPtr palmLink = _parent->GetLink(palmLinkName);
      physics::PhysicsEnginePtr physics = _parent->GetWorld()->Physics();
      physics::JointPtr fixedJoint = physics->CreateJoint("revolute");
      FakeGraspingGripper gripper = FakeGraspingGripper(
          _parent, gripperName, fingerCollisionNames, disableCollisionsOnAttach,
          releaseTolerance, forcesAngleTolerance, minForceRatio, maxGripCount,
          gripCountThreshold, grasping_pub, fixedJoint, palmLink);
      grippers.insert(std::make_pair(gripperName, gripper));
      ROS_INFO_NAMED(kLoggerName, "Successfully initialized gripper %s",
                     gripperName.c_str());
    }
  }

  if (grippers.empty()) {
    ROS_WARN_NAMED(kLoggerName, "No grippers were configured successfully. "
                                "Fake grasping will not work.");
    return;
  }
}

////////////////////////////////////////////////////////////////////////////////
void RobotCollisions::OnContactsUpdate(const ConstContactsPtr &msg) {
  std::lock_guard<std::mutex> guard(mutex_);
  common::Time current_time = world_->SimTime();

  if (current_time - last_time_on_contact_update_ < collisions_min_update_dt_)
    return;

  // Clean stored collisions events
  stored_collision_events_.clear();

  for (int i = 0; i < msg->contact_size(); i++) {

    // Iterate over the robot links and verify if they collide
    // against something
    for (physics::Link_V::iterator it = links_vector_.begin();
         it != links_vector_.end(); ++it) {
      std::string link_name = (*it)->GetName();

      std::string col1 = msg->contact(i).collision1();
      std::string col2 = msg->contact(i).collision2();
      std::string object_collided;
      ROS_DEBUG_STREAM("Collision: " << col1 << "- " << col2);
      // If the collision doesn't involve the link, discard it
      if (col1.find(link_name) != std::string::npos) {
        object_collided = col2.substr(0, col2.find("::"));
      }
      if (col2.find(link_name) != std::string::npos) {
        object_collided = col1.substr(0, col1.find("::"));
      }
      if (object_collided.empty()) {
        continue;
      }

      // Check if the object collided was in the excluded objects list
      if (std::find(excluded_objects_.begin(), excluded_objects_.end(),
                    object_collided) != excluded_objects_.end()) {
        continue;
      }
      // Store collision
      /*ROS_DEBUG_STREAM("Link: " << link_name
                                << " Collision: " << object_collided);*/
      quad_manipulator_msgs::RobotCollisions collision_event;

      collision_event.header.frame_id = link_name;
      collision_event.header.stamp =
          ros::Time(current_time.sec, current_time.nsec);

      collision_event.contact_state.collision1_name = link_name;
      collision_event.contact_state.collision2_name = object_collided;

      stored_collision_events_.push_back(collision_event);

      // Send the collisions to all the grippers.
      physics::CollisionPtr collision1 =
          boost::dynamic_pointer_cast<physics::Collision>(
              world_->EntityByName(col1));

      physics::CollisionPtr collision2 =
          boost::dynamic_pointer_cast<physics::Collision>(
              world_->EntityByName(col2));
      physics::Contact contact;
      contact = msg->contact(i);
      std::map<std::string, FakeGraspingGripper>::iterator gripper_it;
      for (gripper_it = grippers.begin(); gripper_it != grippers.end();
           ++gripper_it) {
        gripper_it->second.processCollision(contact, collision1, collision2);
      }
    }
  }
  last_time_on_contact_update_ = current_time;
}

////////////////////////////////////////////////////////////////////////////////
void RobotCollisions::OnUpdate(const common::UpdateInfo &_info) {
  std::lock_guard<std::mutex> guard(mutex_);
  common::Time current_time = world_->SimTime();

  if (current_time - last_time_collisions_update_ < min_update_dt_)
    return;
  // Publish detected collisions
  for (auto collision_event : stored_collision_events_) {
    collision_pub_queue_->push(collision_event, collision_pub_);
    last_time_collisions_update_ = current_time;
  }
  std::map<std::string, FakeGraspingGripper>::iterator it;
  for (it = grippers.begin(); it != grippers.end(); ++it) {
    it->second.updateGripper();
  }
}

////////////////////////////////////////////////////////////////////////////////
void RobotCollisions::Reset() {
  // Reset time tracking when the simulation is reseted
  last_time_collisions_update_ = world_->SimTime();
  last_time_on_contact_update_ = world_->SimTime();
}
} // namespace gazebo
