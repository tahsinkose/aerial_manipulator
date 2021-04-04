#include <algorithm>
#include <boost/bind.hpp>
#include <gazebo_msgs/ContactState.h>
#include <quad_manipulator_gazebo/fake_grasping_gripper.h>
#include <quad_manipulator_msgs/FakeGraspingEvent.h>
#include <stdio.h>

namespace {
double angularDistance(const ignition::math::Vector3d &_v1,
                       const ignition::math::Vector3d &_v2) {
  ignition::math::Vector3d v1 = _v1;
  ignition::math::Vector3d v2 = _v2;
  v1.Normalize();
  v2.Normalize();
  return acos(v1.Dot(v2));
}

using namespace gazebo;

// Adds new force to the object contact info applied forces.
void addNewForce(
    FakeGraspingGripper::GripperObjectForceCounter &gripperForceCnt,
    const FakeGraspingGripper::CollisionInfo &collP) {
  gripperForceCnt.appliedForces.push_back(collP.force / collP.sum);
  gripperForceCnt.maxGripperContactCnt = std::max(
      gripperForceCnt.maxGripperContactCnt, ++gripperForceCnt.gripperCounter);
}
// Returns the difference in the vectors from a colliding point.
float vectorDifference(FakeGraspingGripper::CollisionInfo &collidingPointInfo) {
  // initial distance from link to contact point
  ignition::math::Vector3d relContactPos =
      collidingPointInfo.position / collidingPointInfo.sum;
  // initial distance from link to object
  ignition::math::Vector3d relObjPos =
      collidingPointInfo.objectPosition / collidingPointInfo.sum;

  // get current world pose of object
  ignition::math::Pose3d currObjWorldPose =
      collidingPointInfo.collidingObject->GetLink()->WorldPose();

  // get world pose of link
  ignition::math::Pose3d currLinkWorldPose =
      collidingPointInfo.collidingLink->GetLink()->WorldPose();

  // Get transform for currLinkWorldPose as matrix
  ignition::math::Matrix4d worldToLink =
      ignition::math::Matrix4d(currLinkWorldPose.Rot());
  worldToLink.SetTranslation(currLinkWorldPose.Pos());

  // Get the transform from collision link to contact point
  ignition::math::Matrix4d linkToContact = ignition::math::Matrix4d::Identity;
  linkToContact.SetTranslation(relContactPos);

  // the current world position of the contact point right now is:
  ignition::math::Matrix4d _currContactWorldPose = worldToLink * linkToContact;
  ignition::math::Vector3d currContactWorldPose =
      _currContactWorldPose.Translation();

  // the initial contact point location on the link corresponds
  // to the initial contact point location on the object.

  // initial vector from object center to contact point (relative to link,
  // because relObjPos and relContactPos are from center of link)
  ignition::math::Vector3d oldObjDist = relContactPos - relObjPos;

  ignition::math::Vector3d newObjDist =
      currContactWorldPose - currObjWorldPose.Pos();
  return fabs(oldObjDist.Length() - newObjDist.Length());
}
} // namespace
namespace gazebo {
FakeGraspingGripper::FakeGraspingGripper() {}
FakeGraspingGripper::~FakeGraspingGripper() {}
std::string FakeGraspingGripper::getGripperName() { return this->gripperName; }

bool FakeGraspingGripper::isGraspedObject(const std::string &linkName) {
  return this->attachedLink == linkName;
}

std::string FakeGraspingGripper::graspedObject() { return this->attachedLink; }

bool FakeGraspingGripper::graspObject(const std::string &attachedLinkName) {
  if (!this->palmLink) {
    ROS_WARN("Fake grasping gripper: palm link not found, not enforcing grasp");
    return false;
  }
  physics::WorldPtr world = this->model->GetWorld();
  if (!world.get()) {
    ROS_WARN("Fake grasping gripper: world is NULL");
    return false;
  }
  physics::CollisionPtr obj = boost::dynamic_pointer_cast<physics::Collision>(
      world->EntityByName(attachedLinkName));
  if (!obj.get()) {
    ROS_WARN("Fake grasping gripper: Object ModelPtr %s not found it world, "
             "can't attach it.",
             attachedLinkName.c_str());
    return false;
  }
  ignition::math::Pose3d diff =
      obj->GetLink()->WorldPose() - this->palmLink->WorldPose();
  this->fixedJoint->Load(this->palmLink, obj->GetLink(), diff);
  this->fixedJoint->Init();
  this->fixedJoint->SetUpperLimit(0, 0.0);
  this->fixedJoint->SetLowerLimit(0, 0.0);
  if (this->disableCollisionsOnAttach) {
    // this disables collision of the object with the gripper
    obj->GetLink()->SetCollideMode("none");
  }
  this->attachedLink = attachedLinkName;
  return true;
}

bool FakeGraspingGripper::releaseObject(const std::string &objName) {
  physics::WorldPtr world = this->model->GetWorld();
  if (!world.get()) {
    ROS_WARN("Fake grasping gripper: world is NULL");
    return false;
  }
  physics::CollisionPtr obj = boost::dynamic_pointer_cast<physics::Collision>(
      world->EntityByName(objName));
  if (!obj.get()) {
    ROS_WARN("Fake grasping gripper: Object ModelPtr %s not found it world, "
             "can't detach it.",
             objName.c_str());
    return false;
  } else if (this->disableCollisionsOnAttach) {
    obj->GetLink()->SetCollideMode("all");
  }
  this->fixedJoint->Detach();
  this->attachedLink.clear();
  return true;
}

bool FakeGraspingGripper::checkGrip(
    const std::vector<ignition::math::Vector3d> &forces, float minAngleDiff,
    float lengthRatio) {
  constexpr double kLengthRatioTolerance = 1e-4;
  if (lengthRatio < kLengthRatioTolerance ||
      lengthRatio > 1 - kLengthRatioTolerance) {
    ROS_WARN(
        "Fake grasping gripper: checkGrip: specify a lengthRatio of [0..1]");
    return false;
  }
  if (minAngleDiff < M_PI_2) {
    ROS_WARN("Fake grasping gripper: checkGrip: min angle must be at least 90 "
             "degrees (PI/2)");
    return false;
  }
  if (forces.size() > 1) {
    ROS_DEBUG_STREAM("Fake grasping gripper: Number of forces -> "
                     << forces.size());
  }
  std::vector<ignition::math::Vector3d>::const_iterator it1, it2;
  for (it1 = forces.begin(); it1 != forces.end(); ++it1) {
    ignition::math::Vector3d v1 = *it1;
    for (it2 = it1 + 1; it2 != forces.end(); ++it2) {
      ignition::math::Vector3d v2 = *it2;
      float l1 = v1.Length();
      float l2 = v2.Length();
      ROS_DEBUG_STREAM("Fake grasping gripper: checkGrip: " << l1 << ", " << l2
                                                            << "\n");
      if ((l1 < 1e-04) || (l2 < 1e-04))
        continue;
      float angle = angularDistance(v1, v2);
      ROS_DEBUG_STREAM(
          "Fake grasping gripper: checkGrip: Angle between forces: "
          << angle << ", min required angle diff: " << minAngleDiff);
      if (angle > minAngleDiff) {
        float ratio;
        if (l1 > l2)
          ratio = l2 / l1;
        else
          ratio = l1 / l2;
        ROS_DEBUG_STREAM("Fake grasping gripper: checkGrip: Ratio: "
                         << ratio
                         << ", min required length ratio: " << lengthRatio);
        if (ratio >= lengthRatio) {
          return true;
        }
      }
    }
  }
  return false;
}

void FakeGraspingGripper::updateGripper() {
  this->mutexContacts.lock();
  std::map<std::string, std::map<std::string, CollisionInfo>> contPoints(
      this->contacts);
  this->contacts.clear();
  this->mutexContacts.unlock();

  std::map<std::string, GripperObjectForceCounter> objectContactInfo;
  if (!contPoints.empty()) {
    ROS_DEBUG_STREAM("Fake grasping gripper: Contact points size = "
                     << (int)contPoints.size());
  }
  for (auto &objIt : contPoints) {

    std::string objName = objIt.first;
    // create new entry in accumulated results map and get reference to fill in:
    GripperObjectForceCounter &objContInfo = objectContactInfo[objName];

    // for all links colliding with this object...
    for (auto &lIt : objIt.second) {
      CollisionInfo &collP = lIt.second;
      addNewForce(objContInfo, collP);
    }
  }

  // Handle Attachment
  std::set<std::string> grippedObjects;
  for (auto &ocIt : objectContactInfo) {
    this->handleAttachment(ocIt.first, ocIt.second, contPoints, grippedObjects);
  }
  // Handle detachment
  this->handleDetachment(grippedObjects);
}

void FakeGraspingGripper::processCollision(
    const physics::Contact &contact, const physics::CollisionPtr &collision1,
    const physics::CollisionPtr &collision2) {
  if (!(collision1 && !collision1->IsStatic()) ||
      !(collision2 && !collision2->IsStatic())) {
    return;
  }
  std::string name1 = collision1->GetScopedName();
  std::string name2 = collision2->GetScopedName();
  if (contact.count < 1) {
    ROS_DEBUG(
        "Fake grasping gripper: Not enough forces given for contact of %s/%s",
        name1.c_str(), name2.c_str());
    return;
  }

  std::string collidingObjName, collidingLink, gripperOfCollidingLink;
  physics::CollisionPtr linkCollision;
  physics::CollisionPtr objCollision;

  // all force vectors which are part of this contact
  std::vector<ignition::math::Vector3d> force;

  // diferentiate between finger and object colliding.
  if (std::find(this->fingerCollisionNames.begin(),
                this->fingerCollisionNames.end(),
                name2) != this->fingerCollisionNames.end()) {
    // collision 1 is the object collision 2 is the finger
    collidingObjName = name1;
    collidingLink = name2;
    linkCollision = collision2;
    objCollision = collision1;
    gripperOfCollidingLink = this->gripperName;
    for (int k = 0; k < contact.count; ++k)
      force.push_back(contact.wrench[k].body1Force);
  } else if (std::find(this->fingerCollisionNames.begin(),
                       this->fingerCollisionNames.end(),
                       name1) != this->fingerCollisionNames.end()) {
    // collision 2 is the object collision 1 is the finger
    collidingObjName = name2;
    collidingLink = name1;
    linkCollision = collision1;
    objCollision = collision2;
    gripperOfCollidingLink = this->gripperName;
    for (int k = 0; k < contact.count; ++k)
      force.push_back(contact.wrench[k].body2Force);
  } else {
    return;
  }
  ignition::math::Vector3d initialValue;
  // calculate average forces applied on the object
  ignition::math::Vector3d avgForce =
      std::accumulate(force.begin(), force.end(), initialValue) / force.size();

  ignition::math::Vector3d initialValuet;

  // calculate average pose to get center point of all the origin positions of
  // the forces
  ignition::math::Vector3d avgPos =
      std::accumulate(std::begin(contact.positions),
                      std::end(contact.positions), initialValuet) /
      contact.count;

  // now, get average pose relative to the colliding link
  ignition::math::Pose3d linkWorldPose = linkCollision->GetLink()->WorldPose();

  // Find out the collision point relative to the Link's local coordinate
  // system, get the Poses as 4x4 matrices
  ignition::math::Matrix4d worldToLink =
      ignition::math::Matrix4d(linkWorldPose.Rot());
  worldToLink.SetTranslation(linkWorldPose.Pos());

  ignition::math::Matrix4d worldToContact = ignition::math::Matrix4d::Identity;
  worldToContact.SetTranslation(avgPos);

  ignition::math::Matrix4d worldToLinkInv = worldToLink.Inverse();
  ignition::math::Matrix4d contactInLocal = worldToLinkInv * worldToContact;
  ignition::math::Vector3d contactPosInLocal = contactInLocal.Translation();

  // Get the pose of the object and compute it's relative position to
  // the collision surface.
  ignition::math::Pose3d objWorldPose = objCollision->GetLink()->WorldPose();
  ignition::math::Matrix4d worldToObj =
      ignition::math::Matrix4d(objWorldPose.Rot());
  worldToObj.SetTranslation(objWorldPose.Pos());

  ignition::math::Matrix4d objInLocal = worldToLinkInv * worldToObj;
  ignition::math::Vector3d objPosInLocal = objInLocal.Translation();

  {
    std::lock_guard<std::mutex> guard(this->mutexContacts);
    CollisionInfo &point = this->contacts[collidingObjName][collidingLink];
    point.collidingLink = linkCollision;
    point.collidingObject = objCollision;
    point.force += avgForce;
    point.position += contactPosInLocal;
    point.objectPosition += objPosInLocal;
    point.sum++;
  }
  return;
}

void FakeGraspingGripper::publishGraspingEvent(
    const std::string &graspingGripperName, const std::string &objName,
    int event) {
  // Publish event
  common::Time current_time = this->model->GetWorld()->SimTime();
  quad_manipulator_msgs::FakeGraspingEvent fake_grasping_event;

  fake_grasping_event.header.frame_id = graspingGripperName;
  fake_grasping_event.header.stamp =
      ros::Time(current_time.sec, current_time.nsec);

  fake_grasping_event.gripper = graspingGripperName;
  fake_grasping_event.object = objName;
  fake_grasping_event.event_type = event;
  this->fakeGraspingPublisher.publish(fake_grasping_event);
}

void FakeGraspingGripper::handleAttachment(
    const std::string &objName, const GripperObjectForceCounter &objContInfo,
    const std::map<std::string, std::map<std::string, CollisionInfo>>
        &contPoints,
    std::set<std::string> &grippedObjects) {
  float minAngleDiff = this->forcesAngleTolerance;
  if (!this->checkGrip(objContInfo.appliedForces, minAngleDiff,
                       this->minForceRatio)) {
    return;
  }

  // add to "gripped objects"
  grippedObjects.insert(objName);

  int &counts = this->gripCounts[objName];
  if (counts < this->maxGripCount) {
    ++counts;
  }
  ROS_DEBUG_STREAM("Number of grip counts = " << counts << " threshold = "
                                              << this->gripCountThreshold);
  // only need to attach object if the grip count threshold is exceeded
  if (counts < this->gripCountThreshold) {
    return;
  }
  // Attach the object to the gripper
  if (!this->graspedObject().empty()) {
    ROS_WARN_THROTTLE(
        5,
        "Fake grasping gripper: Object %s already attached to gripper "
        "%s, can't grasp %s",
        this->graspedObject().c_str(), this->gripperName.c_str(),
        objName.c_str());
    return;
  }

  ROS_INFO("Fake grasping gripper: Attaching %s to gripper %s", objName.c_str(),
           this->gripperName.c_str());
  if (!contPoints.count(objName)) {
    ROS_INFO("Fake grasping gripper: %s not present in contPoints, skipping.",
             objName.c_str());
    return;
  }
  // Store the array of contact poses which played part in the grip, sorted by
  // colliding link.
  const std::map<std::string, CollisionInfo> &contPointsTmp =
      contPoints.at(objName);
  std::map<std::string, CollisionInfo> &attGripConts =
      this->attachGripContacts[objName];
  attGripConts.clear();
  std::map<std::string, CollisionInfo>::const_iterator contPointsIt;
  for (contPointsIt = contPointsTmp.begin();
       contPointsIt != contPointsTmp.end(); ++contPointsIt) {
    const std::string &collidingLink = contPointsIt->first;
    const CollisionInfo &collidingPoint = contPointsIt->second;
    attGripConts[collidingLink] = collidingPoint;
  }

  if (!this->graspObject(objName)) {
    ROS_WARN("Fake grasping gripper: Could not attach object %s to gripper %s",
             objName.c_str(), this->gripperName.c_str());
    return;
  } else {
    ROS_DEBUG("Fake grasping gripper: %s is attached on %s.", objName.c_str(),
              this->gripperName.c_str());
    // Publish grasping event.
    this->publishGraspingEvent(
        this->gripperName, objName,
        quad_manipulator_msgs::FakeGraspingEvent::GRASPING);
    return;
  }
}

void FakeGraspingGripper::handleDetachment(
    std::set<std::string> &grippedObjects) {
  std::string attachedObject = this->graspedObject();

  // For all objects that are not currently gripped, decrease grip counter, and
  // possibly release object.
  std::map<std::string, int>::iterator gripCntIt;
  for (gripCntIt = this->gripCounts.begin();
       gripCntIt != this->gripCounts.end(); ++gripCntIt) {

    const std::string &objName = gripCntIt->first;

    if (grippedObjects.find(objName) != grippedObjects.end()) {
      // Object just gripped, no need to consider it for detachment.
      continue;
    }

    // Object does not satisfy "gripped" criteria, so potentially has to be
    // released.
    if (gripCntIt->second > 0)
      --(gripCntIt->second);

    bool isAttached = attachedObject == objName;

    if (!isAttached || (gripCntIt->second > this->gripCountThreshold))
      continue;

    // Object should potentially be detached now.
    std::map<std::string, std::map<std::string, CollisionInfo>>::iterator
        initCollIt = this->attachGripContacts.find(objName);
    if (initCollIt == this->attachGripContacts.end()) {
      ROS_WARN("Fake grasping gripper: implementation error.Could not find "
               "attachGripContacts for  %s",
               objName.c_str());
      continue;
    }

    std::map<std::string, CollisionInfo> &initColls = initCollIt->second;
    bool shouldRelease = false;

    std::map<std::string, CollisionInfo>::iterator pointIt;
    for (pointIt = initColls.begin(); pointIt != initColls.end(); ++pointIt) {
      CollisionInfo &cpInfo = pointIt->second;

      float diff = vectorDifference(cpInfo);
      ROS_DEBUG("Fake grasping gripper: "
                "Difference between old/new object distance is: %lf, release "
                "tolerance: %lf",
                diff, releaseTolerance);

      if (diff > releaseTolerance) {
        shouldRelease = true;
        break;
      }
    }

    if (shouldRelease) {
      const std::string &graspingGripperName = this->gripperName;
      ROS_INFO("Fake grasping gripper: Detaching %s from gripper %s",
               objName.c_str(), graspingGripperName.c_str());
      this->releaseObject(objName);

      gripCntIt->second = 0;
      this->attachGripContacts.erase(initCollIt);

      // Publish detach event
      this->publishGraspingEvent(
          graspingGripperName, objName,
          quad_manipulator_msgs::FakeGraspingEvent::DETACH);
    }
  }
}
} // namespace gazebo
