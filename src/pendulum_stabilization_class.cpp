#include <pendulum_stabilization_class.h>

#include <boost/bind.hpp>

#include <stdio.h>

#include <sdf/Param.hh>

namespace gazebo
{

PendulumStabilizationPlugin::PendulumStabilizationPlugin() : ModelPlugin() {
  gzmsg << "MotionControllerPlugin: Created.\n";
}

PendulumStabilizationPlugin::~PendulumStabilizationPlugin(){

  gzmsg << "PendulumStabilizationPlugin: Destroyed.\n";

}

void PendulumStabilizationPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

  // Store the pointer to the model
  this->model_ = _parent;
  this->sdf_ = _sdf;


  cart_joint_ = GetJoint("cart_joint");

  first_pendulum_joint_ = GetJoint("first_pendulum_joint");

  second_pendulum_joint_ = GetJoint("second_pendulum_joint");


  if(nullptr != cart_joint_ &&
     nullptr != first_pendulum_joint_ &&
     nullptr != second_pendulum_joint_) {

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
                                 boost::bind(&PendulumStabilizationPlugin::OnUpdate, this, _1));

    gzmsg << "MotionControllerPlugin: Loaded.\n";

  }
}

// Called by the world update start event

void PendulumStabilizationPlugin::OnUpdate(const common::UpdateInfo & /*_info*/) {

  first_pendulum_joint_->SetPosition(0,1);
  second_pendulum_joint_->SetPosition(0,1);

}

physics::JointPtr PendulumStabilizationPlugin::GetJoint(const std::string& element_name) {

  physics::JointPtr joint_ptr = nullptr;

  if(nullptr != model_ &&
     nullptr != sdf_ ) {

    sdf::ElementPtr joint_name = sdf_->GetElement(element_name);
    if (nullptr == joint_name)
    {
      gzerr << "<" << element_name << ">" << " does not exist\n";

    } else {
      joint_ptr = model_->GetJoint(joint_name->GetValue()->GetAsString());
    }

  }

  return joint_ptr;

}


}
