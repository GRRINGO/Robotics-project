#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class ModelEncoder : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelEncoder::OnUpdate, this));

      //Set the update rate
      this->updateRate = common::Time(0, common::Time::SecToNano(1.0/20.0));

      //Previously was updated
      this->prevUpdateTime = common::Time::GetWallTime();


    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      if (common::Time::GetWallTime() - this->prevUpdateTime < this->updateRate){
        return;
      }

      double current_vel_left = this->model->GetJoint("left_motor_wheel_hinge")->GetVelocity(2);
      double current_vel_right = this->model->GetJoint("right_motor_wheel_hinge")->GetVelocity(2);

      std::cout<<"Left: "<<current_vel_left<<" / Right: "<<current_vel_right<<std::endl;

      this->prevUpdateTime = common::Time::GetWallTime();
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: common::Time updateRate;
    private: common::Time prevUpdateTime;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelEncoder)
}
