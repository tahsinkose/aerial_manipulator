#include <ctime>
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>

namespace gazebo
{
  class FruitDetachment : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      ROS_INFO("Loading fruit detachment on %s\n",_parent->GetName().c_str());
      this->model = _parent;
      this->init = clock();
      this->detached = false;
      this->restored = false;
      // Listen to the begin-update event. This event is broadcast every
      // simulation iteration.
      this->begin_update_connection =
          event::Events::ConnectWorldUpdateBegin(std::bind(&FruitDetachment::OnUpdateBegin, this));
     }

     // Called by the world update start event
    public:
     void OnUpdateBegin() {
       clock_t elapsed = clock() - this->init;
       float elapsed_s = (float)elapsed / CLOCKS_PER_SEC;
       if (!this->detached && elapsed_s > 3.0) {
         this->detached = true;
         ROS_INFO("Detaching fruits..");
         auto joints = this->model->GetJoints();

         for (const auto &jnt : joints) {
           ROS_INFO("Joint name: %s", jnt->GetName().c_str());
           if (jnt->GetName().find("apple_joint") != std::string::npos) {
             ROS_INFO("Removing: %s", jnt->GetName().c_str());
             auto world_T_apple = jnt->GetChild()->WorldPose();
             ROS_INFO("world_T_apple: (%lf,%lf,%lf,%lf,%lf,%lf,%lf)", world_T_apple.Pos().X(), world_T_apple.Pos().Y(),
                      world_T_apple.Pos().X(), world_T_apple.Pos().Z(), world_T_apple.Rot().X(),
                      world_T_apple.Rot().Y(), world_T_apple.Rot().Z(), world_T_apple.Rot().W());
            jnt->Detach();
           }
           auto world_T_model = this->model->WorldPose();
           world_T_model.Pos().X() += 0.01;
           this->model->SetRelativePose(world_T_model);
         }
         return;
         if (this->detached && !this->restored) {
           auto world_T_model = this->model->WorldPose();
           world_T_model.Pos().X() -= 0.01;
           this->model->SetRelativePose(world_T_model);
           this->restored = true;
         }
       }
     }

     // Pointer to the model
    private:
     physics::ModelPtr model;
     clock_t init;
     bool detached;
     bool restored;
     ros::NodeHandle nh;

     // Pointer to the update event connection
    private: event::ConnectionPtr begin_update_connection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(FruitDetachment)
}