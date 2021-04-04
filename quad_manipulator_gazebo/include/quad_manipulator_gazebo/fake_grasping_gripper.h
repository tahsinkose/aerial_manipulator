#pragma once

#include <gazebo/physics/physics.hh>
#include <ros/ros.h>

namespace gazebo {

class FakeGraspingGripper {
public:
  FakeGraspingGripper();
  FakeGraspingGripper(
      physics::ModelPtr &_model, const std::string &_gripperName,
      std::vector<std::string> &_fingerCollisionNames,
      bool _disableCollisionsWhenAttached, double _releaseTolerance,
      double _forcesAngleTolerance, double _minForceRatio, double _maxGripCount,
      double _gripCountThreshold, ros::Publisher _grasping_publisher,
      physics::JointPtr _fixedJoint, physics::LinkPtr _palmLink)
      : model(_model), gripperName(_gripperName),
        fingerCollisionNames(_fingerCollisionNames), fixedJoint(_fixedJoint),
        palmLink(_palmLink),
        disableCollisionsOnAttach(_disableCollisionsWhenAttached),
        releaseTolerance(_releaseTolerance),
        forcesAngleTolerance(_forcesAngleTolerance),
        minForceRatio(_minForceRatio), maxGripCount(_maxGripCount),
        gripCountThreshold(_gripCountThreshold),
        fakeGraspingPublisher(_grasping_publisher) {}

  FakeGraspingGripper(const FakeGraspingGripper &fg)
      : model(fg.model), gripperName(fg.gripperName),
        fingerCollisionNames(fg.fingerCollisionNames),
        fixedJoint(fg.fixedJoint), palmLink(fg.palmLink),
        disableCollisionsOnAttach(fg.disableCollisionsOnAttach),
        releaseTolerance(fg.releaseTolerance),
        forcesAngleTolerance(fg.forcesAngleTolerance),
        minForceRatio(fg.minForceRatio), maxGripCount(fg.maxGripCount),
        gripCountThreshold(fg.gripCountThreshold),
        fakeGraspingPublisher(fg.fakeGraspingPublisher) {}

  virtual ~FakeGraspingGripper();

  // Check time diference between collisions to check if gripping.
  void updateGripper();

  // Returns the name of the gripper
  std::string getGripperName();

  // Returns true if the string provided is the link grasped.
  bool isGraspedObject(const std::string &linkName);

  // Returns the grasped object name.
  std::string graspedObject();

  // Processes and stores collisions with the fingers.
  void processCollision(const physics::Contact &contact,
                        const physics::CollisionPtr &collision1,
                        const physics::CollisionPtr &collision2);

  class CollisionInfo {
  public:
    CollisionInfo() : sum(0) {}
    CollisionInfo(const CollisionInfo &originalCollisionInfo)
        : collidingLink(originalCollisionInfo.collidingLink),
          collidingObject(originalCollisionInfo.collidingObject),
          force(originalCollisionInfo.force),
          position(originalCollisionInfo.position),
          objectPosition(originalCollisionInfo.objectPosition),
          sum(originalCollisionInfo.sum) {}

    // Collision pointer for storing the collision link and object
    physics::CollisionPtr collidingLink, collidingObject;

    // Average force vector of the collision
    ignition::math::Vector3d force;

    // Contact position
    ignition::math::Vector3d position;

    // Object position
    ignition::math::Vector3d objectPosition;

    // Average of force and positoin
    int sum;
  };
  class GripperObjectForceCounter {
  public:
    // Forces affecting the objects
    std::vector<ignition::math::Vector3d> appliedForces;

    // Gripper force count.
    int gripperCounter;

    // maximum number of contacts of the gripper.
    int maxGripperContactCnt;
  };

private:
  // Checks and attachs the links based on the forces and times the check
  // succeeded
  void handleAttachment(
      const std::string &objName, const GripperObjectForceCounter &objContInfo,
      const std::map<std::string, std::map<std::string, CollisionInfo>>
          &contPoints,
      std::set<std::string> &grippedObjects);

  // Checks and detaches the link based
  void handleDetachment(std::set<std::string> &grippedObjects);

  // Creates a joint between the grasped object and configured palm.
  bool graspObject(const std::string &attachedLinkName);

  // Removes the joint between the grasped object and palm.
  bool releaseObject(const std::string &objName);

  // Publishes a grasping event to the grasping topic.
  void publishGraspingEvent(const std::string &graspingGripperName,
                            const std::string &objName, int event);

  /* Checks if an object is considered gripped based on min angle difference and
   * min length ratio. This function is the core part of the plug-in. All
   * contact forces between the left/right gripper and all object are collected.
   * Then all contact forces between an object and a gripper are checked
   * pairwise. The function returns true if at least one pair fulfills the
   * "gripped" criteria. The criteria is fulfilled and the object is considered
   * "gripped" if two contact forces form a minimum angle greater then
   * minAngleDiff and if their norms minimum ratio is at least lengthRatio.
   * checkGrip is executed on every update iteration (OnUpdate). Each time
   * checkGrip returns true the object and gripper specific "gripped" counter is
   * raised. Each time checkGrip returns false this counter is lowered. If this
   * counter exceeds the threshold gripCountThreshold the
   * object is attached to the gripper. */
  bool checkGrip(const std::vector<ignition::math::Vector3d> &forces,
                 float minAngleDiff, float lengthRatio);

  // Pointer to the parent model
  physics::ModelPtr model;

  // Name of the gripper.
  std::string gripperName;

  // Names of the gripper links.
  std::vector<std::string> fingerNames;

  // Names of the gripper links collisions.
  std::vector<std::string> fingerCollisionNames;

  // Joint pointer for the fake grasping joint.
  physics::JointPtr fixedJoint;

  // Palm link which the joint will attach to.
  physics::LinkPtr palmLink;

  // If enabled disables collision with the object.
  bool disableCollisionsOnAttach;

  // The tolerance for vector difference to consider it no longer touching.
  double releaseTolerance;

  /* An Object is considered "gripped" if at least two opposing forces exist
   * which form a minimum angle of forcesAngleTolerance */
  double forcesAngleTolerance;

  /* An Object is considered "gripped" if at least two opposing forces exist
   * which norms have at least a ratio of minForceRatio. */
  double minForceRatio;

  // Grasped link.
  std::string attachedLink;

  // Set of gripped objects.
  std::set<std::string> grippedObjects;

  // Colliding points mapped by gripper and object.
  std::map<std::string, std::map<std::string, CollisionInfo>> contacts;

  // Object name, colliding finger collision info map.
  std::map<std::string, std::map<std::string, CollisionInfo>>
      attachGripContacts;

  // Success grip counter.
  std::map<std::string, int> gripCounts;

  // mutex protects contacts
  std::mutex mutexContacts;

  // Maximum number of gripCounts to consider.
  int maxGripCount;

  // Gripping counter threshold, when its exceeded we grasp the object, if less
  // than it we start to detach the object.
  int gripCountThreshold;

  ros::Publisher fakeGraspingPublisher;
};
} // namespace gazebo
