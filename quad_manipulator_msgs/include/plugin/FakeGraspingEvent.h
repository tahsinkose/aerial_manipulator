#define QUAD_MANIPULATOR_GAZEBO_MESSAGE_FAKEGRASPINGEVENT_PLUGIN_CLASS_BODY \
  static std::string EventTypeString(int event) { \
    using FakeGraspingEvent_ = ::quad_manipulator_gazebo::FakeGraspingEvent_<ContainerAllocator>; \
    switch (event) { \
      case FakeGraspingEvent_::GRASPING: \
        return "GRASPING"; \
      case FakeGraspingEvent_::DETACH: \
        return "DETACH"; \
      case FakeGraspingEvent_::ERROR_GRASPING: \
        return "ERROR_GRASPING"; \
      default: \
        return "[UNKNOWN FakeGraspingEvent " + std::to_string(event) + "]"; \
    } \
  }