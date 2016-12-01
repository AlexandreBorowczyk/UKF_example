#ifndef PENDULUMSTABILIZATIONPLUGIN_H_
#define PENDULUMSTABILIZATIONPLUGIN_H_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
class GAZEBO_VISIBLE PendulumStabilizationPlugin : public ModelPlugin {
    public:
        explicit PendulumStabilizationPlugin();
        virtual ~PendulumStabilizationPlugin();

        void Load(physics::ModelPtr parent, sdf::ElementPtr /*_sdf*/) override;

        void OnUpdate(const common::UpdateInfo & /*_info*/);


        // Pointer to the model
    private:

        physics::JointPtr GetJoint(const std::string& element_name);

        physics::ModelPtr       model_;
        sdf::ElementPtr         sdf_;

        event::ConnectionPtr    update_connection_;

        physics::JointPtr       cart_joint_;
        physics::JointPtr       first_pendulum_joint_;
        physics::JointPtr       second_pendulum_joint_;

};


// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(PendulumStabilizationPlugin)

}

#endif  // PENDULUMSTABILIZATIONPLUGIN_H_
