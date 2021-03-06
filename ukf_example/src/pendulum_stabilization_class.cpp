#include <pendulum_stabilization_class.h>
#include <ukf_example_srvs/ComputeCommand.h>

#include <ros/console.h>
#include <sdf/Param.hh>

#include <boost/bind.hpp>

#include <stdio.h>


namespace gazebo
{

PendulumStabilizationPlugin::PendulumStabilizationPlugin() :ModelPlugin(),
  force_command_(0.0),
  previous_iteration_time_(0.0) {
  gzmsg << "PendulumStabilizationPlugin: Created.\n";
}

PendulumStabilizationPlugin::~PendulumStabilizationPlugin(){

  nh_->shutdown();

  delete nh_;
  nh_ = nullptr;

  gzmsg << "PendulumStabilizationPlugin: Destroyed.\n";

}

void PendulumStabilizationPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin.");
    return;
  }

  nh_ = new ros::NodeHandle();

  if(nh_->ok()) {

    control_client_ = nh_->serviceClient<ukf_example_srvs::ComputeCommand>("ComputeCommandDip");
    gzmsg << "PendulumStabilizationPlugin: Ros Node handle and service client created.\n";

  }

  // Store the pointer to the model
  model_ = _parent;
  sdf_ = _sdf;



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

    gzmsg << "PendulumStabilizationPlugin: Loaded.\n";

  }
}

// Called by the world update start event

void PendulumStabilizationPlugin::OnUpdate(const common::UpdateInfo & /*_info*/) {

  double current_time = model_->GetWorld()->GetSimTime().Double();

  double elasped_time = (current_time - previous_iteration_time_);

  if(0.01 < elasped_time) {
    previous_iteration_time_ = current_time;

    ukf_example_srvs::ComputeCommand srv;

    srv.request.x         = cart_joint_->GetAngle(0).Radian();
    srv.request.dot_x     = cart_joint_->GetVelocity(0);
    srv.request.theta     = first_pendulum_joint_->GetAngle(0).Radian();
    srv.request.dot_theta = first_pendulum_joint_->GetVelocity(0);
    srv.request.phi       = first_pendulum_joint_->GetAngle(0).Radian() + second_pendulum_joint_->GetAngle(0).Radian();
    srv.request.dot_phi   = first_pendulum_joint_->GetVelocity(0) + second_pendulum_joint_->GetVelocity(0);

    if(control_client_.call(srv)) {
      force_command_= srv.response.force;
      //ROS_INFO("Sent: %f %f %f %f %f %f",srv.request.x,srv.request.dot_x,srv.request.theta,srv.request.dot_theta,srv.request.phi,srv.request.dot_phi);
      //ROS_INFO("Received: %f ", force_command_);
    }

  }
  cart_joint_->SetForce(0,force_command_);
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
